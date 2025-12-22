/*
 * NIST-developed software is provided by NIST as a public service. You may use,
 * copy, and distribute copies of the software in any medium, provided that you
 * keep intact this entire notice. You may improve, modify, and create
 * derivative works of the software or any portion of the software, and you may
 * copy and distribute such modifications or works. Modified works should carry
 * a notice stating that you changed the software and should note the date and
 * nature of any such change. Please explicitly acknowledge the National
 * Institute of Standards and Technology as the source of the software. 
 *
 * NIST-developed software is expressly provided "AS IS." NIST MAKES NO WARRANTY
 * OF ANY KIND, EXPRESS, IMPLIED, IN FACT, OR ARISING BY OPERATION OF LAW,
 * INCLUDING, WITHOUT LIMITATION, THE IMPLIED WARRANTY OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, NON-INFRINGEMENT, AND DATA ACCURACY. NIST
 * NEITHER REPRESENTS NOR WARRANTS THAT THE OPERATION OF THE SOFTWARE WILL BE
 * UNINTERRUPTED OR ERROR-FREE, OR THAT ANY DEFECTS WILL BE CORRECTED. NIST DOES
 * NOT WARRANT OR MAKE ANY REPRESENTATIONS REGARDING THE USE OF THE SOFTWARE OR
 * THE RESULTS THEREOF, INCLUDING BUT NOT LIMITED TO THE CORRECTNESS, ACCURACY,
 * RELIABILITY, OR USEFULNESS OF THE SOFTWARE.
 * 
 * You are solely responsible for determining the appropriateness of using and
 * distributing the software and you assume all risks associated with its use,
 * including but not limited to the risks and costs of program errors,
 * compliance with applicable laws, damage to or loss of data, programs or
 * equipment, and the unavailability or interruption of operation. This software 
 * is not intended to be used in any situation where a failure could cause risk
 * of injury or damage to property. The software developed by NIST employees is
 * not subject to copyright protection within the United States.
 *
 * Authors: 
 *  Thomas Roth <thomas.roth@nist.gov>
 *  Hadhoum Hajjaj <hadhoum.hajjaj@nist.gov> to include V2I communication using wifi 
 *  Raphael Barbau
*/

#include <string>
#include <vector>

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"

#include "ns3/antenna-module.h"
#include "ns3/nr-module.h"

#include "ns3/json-gateway.h"
#include "ns3/v2x-mobility-helper.h"
#include "ns3/v2x-sidelink-helper.h"
#include "ns3/v2x-vehicle-helper.h"
#include "ns3/v2x-traffic-light-helper.h"

#include "ns3/vehicle-object.h"
#include "ns3/traffic-light-object.h"

#include <iomanip>



using namespace ns3;

NS_LOG_COMPONENT_DEFINE("SimpleJSONGateway");

/*
 * An example that manages a set of nodes (representing vehicles) whose mobility is controlled by the remote server.
 *
 * The received data format is:
 *  {P_X1, P_Y1, P_Z1, V_X1, V_Y1, V_Z1, Send_1, ..., P_Xn, P_Yn, P_Zn, V_X1n, V_Yn, V_Zn, Send_n}
 * where:
 *  {P_Xi, P_Yi, P_Zi} is a Vector that represents the Position of vehicle i
 *  {V_Xi, V_Yi, V_Zi} is a Vector that represents the Velocity of vehicle i
 *  Send_i is a boolean that indicates whether vehicle i should broadcast
 *
 * The response data format is:
 *  {recvCount_1, ..., recvCount_n}
 * where:
 *  recvCount_i is the number of times vehicle i has received a broadcast
 *
 * A response is sent each time data is received.
 */
class SimpleJSONGateway : public JSONGateway 
{
public:
        // initialize a simple gateway where n = vehicles.GetN()
    SimpleJSONGateway();
    void HandleReceiveWithTs(std::string id, Ptr<const Packet> packet,
                             const Address &from, const Address &to,
                             const SeqTsSizeHeader &header);


    void ReportMobility(Ptr<const MobilityModel> mobility);
    

private:

    Ipv4Address broadcastIpv4Address;
    uint16_t applicationPort;

    Address broadcastAddress;
    Address listeningAddress;

    V2XSidelinkHelper sidelink;
    V2XVehicleHelper vehicleHelper;
    V2XTrafficLightHelper trafficLightHelper;

        // this function handles processing the first message received from the remote server
        //  the simple gateway doesn't require any initialization, so this just calls DoUpdate
    virtual void DoInitialize(const json & data);

        // this function handles processing messages received from the remote server
    virtual void DoUpdate(const json & data);


};

SimpleJSONGateway::SimpleJSONGateway()
    : JSONGateway(),broadcastIpv4Address("255.255.255.255"), 
    applicationPort(8000),
    broadcastAddress(InetSocketAddress(broadcastIpv4Address, applicationPort)),
    listeningAddress(InetSocketAddress(Ipv4Address::GetAny(), applicationPort)),
    sidelink(broadcastIpv4Address),
    vehicleHelper(broadcastAddress, listeningAddress, MakeCallback(&SimpleJSONGateway::ReportMobility, this), MakeCallback(&SimpleJSONGateway::HandleReceiveWithTs, this)),
    trafficLightHelper(broadcastAddress)
{
    NS_LOG_INFO("Creating JSON gateway");
    
}

void SimpleJSONGateway::HandleReceiveWithTs(std::string id, Ptr<const Packet> packet,
    const Address &from, const Address &to,
    const SeqTsSizeHeader &header)
{
    Time txTime = header.GetTs();
    NS_LOG_INFO("At time " << Simulator::Now().As(Time::S)
    << ", Node " << id << " received a packet (sent at "
    << txTime.As(Time::S) << ", delta is " << (Simulator::Now()-txTime).As(Time::S) << ")");
    uint32_t index = std::stoi(id);
    //m_count[index] += 1;

    Ptr<VehicleObject> vehicle = DynamicCast<VehicleObject>(GetObjects().at(index));
    if (vehicle)
    {
        vehicle->SetLastReceiveTime(txTime); // store for reporting later
    }
}


void SimpleJSONGateway::DoInitialize(const json & data)
{
    DoUpdate(data);
}

void SimpleJSONGateway::DoUpdate(const json & data)
{
    static Time lastUpdate = Seconds(0);
    Time now = Simulator::Now();
    Time delta = now - lastUpdate;

    NS_LOG_INFO(">>> [ns-3] DoUpdate at " << now.GetSeconds() << " s, Δt = " << delta.GetSeconds() << " s");

    lastUpdate = now;

    json jobjects = data[JSONGateway::JSONGATEWAY_OBJECTS];
    if (jobjects.is_array())
    {
       for(auto it = jobjects.begin(); it != jobjects.end(); ++it) 
       {
            json jobject = *it;
            json id = jobject[JSONObject::JSONOBJECT_ID];
            if (!JSONObject::IsUInt(JSONObject::JSONOBJECT_ID, id))
            {
                NS_LOG_WARN(">>> Skipping JSON object without uint ID attribute");
                continue;
            }

            json alive = jobject[JSONObject::JSONOBJECT_ALIVE];
            if (!JSONObject::IsBool(JSONObject::JSONOBJECT_ALIVE, alive))
            {
                NS_LOG_WARN(">>> Skipping JSON object without boolean alive attribute");
                continue;
            }

            json type = jobject[JSONObject::JSONOBJECT_TYPE];
            if (!JSONObject::IsString(JSONObject::JSONOBJECT_TYPE, type))
            {
                NS_LOG_WARN(">>> Skipping JSON object without string type attribute");
                continue;
            }
            auto itobject = GetObjects().find(id);
            if (itobject != GetObjects().end())
            {
                Ptr<JSONMobilityObject> object = itobject->second;
                // deseralize regardless
                object->Deserialize(jobject);
                if (!alive)
                {
                    NS_LOG_INFO(">>> Removing object " << id);
                    RemoveObject(id);
                }
            }
            else
            {
                if (alive)
                {
                    // use NS3 factory pattern
                    if (type == VehicleObject::VEHICLEOBJECT_TYPE)
                    {
                        Ptr<VehicleObject> object = CreateObject<VehicleObject>(jobject);
                        NS_LOG_INFO("Adding new vehicle: " << type); 
                        Ptr<Node> node = AddObject(object);

                        sidelink.Install(node);
                        // Use JSONObject id instead of GetNodes index
                        vehicleHelper.Install(node, object->GetId());
                        NS_LOG_INFO("Installation Completed");

                    }
                    else if (type == TrafficLightObject::TRAFFICLIGHTOBJECT_TYPE)
                    {
                        Ptr<TrafficLightObject> object = CreateObject<TrafficLightObject>(jobject);
                        NS_LOG_INFO("Adding new traffic light: " << type); 
                        Ptr<Node> node = AddObject(object);
                        sidelink.Install(node);
                        trafficLightHelper.Install(node);
                        NS_LOG_INFO("Installation Completed");

                    }
                    else
                    {
                        
                        NS_LOG_WARN("Object not added: " << type); 
                    }
                    NS_LOG_INFO("Addition Completed");
                }

            }
       }
    }
    else
    {
        NS_LOG_WARN("JSON object not an array: " << jobjects); 
    }

    SendResponse(); // format and send a response based on the most recent SetValue
}

void SimpleJSONGateway::ReportMobility(Ptr<const MobilityModel> mobility)
{
    NS_LOG_DEBUG("At time " << Simulator::Now().As(Time::S)
                << ", Node " << mobility->GetObject<Node>()->GetId()
                << ", Position " << mobility->GetPosition()
                << ", Velocity " << mobility->GetVelocity());
}


int main(int argc, char *argv[])
{
    bool verboseLogs            = true;
    uint16_t serverPort         = 8100;
    std::string serverAddress   = "127.0.0.1";

    CommandLine cmd(__FILE__);
    cmd.AddValue("verbose", "Enable/disable detailed log output", verboseLogs);
    cmd.AddValue("serverPort", "Port number of the UDP Server", serverPort);
    cmd.AddValue("serverAddress", "Address of the UDP Server", serverAddress);
    cmd.Parse(argc, argv);

    Time::SetResolution(Time::NS); // timestamp has nanosecond resolution

    if (verboseLogs)
    {
        LogComponentEnable("BaseGateway", LOG_LEVEL_ALL);
        LogComponentEnable("JSONGateway", LOG_LEVEL_ALL);
        LogComponentEnable("SimpleJSONGateway", LOG_LEVEL_ALL);
        LogComponentEnable("V2XMobilityHelper", LOG_LEVEL_ALL);
        LogComponentEnable("V2XSidelinkHelper", LOG_LEVEL_ALL);
        LogComponentEnable("V2XTrafficLightHelper", LOG_LEVEL_ALL);
        LogComponentEnable("V2XVehicleHelper", LOG_LEVEL_ALL);

        LogComponentEnable("JSONObject", LOG_LEVEL_ALL);
        LogComponentEnable("JSONMobilityObject", LOG_LEVEL_ALL);
        LogComponentEnable("TrafficLightObject", LOG_LEVEL_ALL);
        LogComponentEnable("VehicleObject", LOG_LEVEL_ALL);

        LogComponentEnable("PacketSink", LOG_LEVEL_INFO);
        LogComponentEnable("OnOffApplication", LOG_LEVEL_INFO);

        
        // All the Sidelink layers
        LogLevel logLevel = (LogLevel)(LOG_PREFIX_FUNC | LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL);
        //LogComponentEnable("UdpClient", LOG_LEVEL_ALL);
        //LogComponentEnable("UdpServer", LOG_LEVEL_ALL);
        //// LogComponentEnable("LtePdcp", LOG_LEVEL_ALL);
        //// LogComponentEnable("LteRlc", LOG_LEVEL_ALL);
        //// LogComponentEnable("LteRlcUm", LOG_LEVEL_ALL);
        //// LogComponentEnable("LteRlcAm", LOG_LEVEL_ALL);
        //// LogComponentEnable("LteRlcTm", LOG_LEVEL_ALL);
        //// LogComponentEnable("NrSlUeRrc", LOG_LEVEL_ALL);
        //// LogComponentEnable("NrSlUeMac", LOG_LEVEL_ALL);
        //// LogComponentEnable("NrPhy", LOG_LEVEL_ALL);
        //LogComponentEnable("NrHelper", LOG_LEVEL_ALL);
        //LogComponentEnable("NrUePhy", LOG_LEVEL_INFO);
        ////LogComponentEnable("SpectrumPhy", LOG_LEVEL_ALL);
        //LogComponentEnable("NrSpectrumPhy", LOG_LEVEL_ALL);
        //LogComponentEnable("SingleModelSpectrumChannel", LOG_LEVEL_ALL);
        //LogComponentEnable("MultiModelSpectrumChannel", LOG_LEVEL_ALL);
    }
    else
    {
        LogComponentEnable("Gateway", LOG_LEVEL_INFO);
        LogComponentEnable("SimpleJSONGateway", LOG_LEVEL_INFO);
    }

    // install the external mobility model

    // === Vehicle Mobility Setup ===
    //
    // Create a MobilityHelper to assign mobility settings (position and movement behavior) to vehicle nodes.
    // In this simulation, we use the "ExternalMobilityModel" which allows node positions to be updated
    // externally in real-time.
    //
    // We also assign each vehicle an initial position using the 'positionAllocator' created earlier.
    //
    // We loop through the first 'numberOfVehicles' nodes only, because in this V2I setup,
    // the last node (at index [numberOfVehicles]) is reserved for the traffic light, which will not move
    // and should use a different mobility model.
    //
    // The loop ensures that only vehicle nodes receive the mobility configuration intended for dynamic/mobile entities.

    SimpleJSONGateway gateway;
    // should connect and populate objects, assign position/velocity
    
    // Connection happens right away, so that objects cna be created
    gateway.Connect(serverAddress, serverPort);

    // TODO: set propagation loss and delay
    // Ptr<PropagationDelayModel> propDelay = CreateObject<PropagationDelayModel>();
    // Ptr<PropagationLossModel> propLoss = CreateObject<ThreeGppPropagationLossModel>();
    // channel->SetPropagationDelayModel(propDelay);
    // channel->SetPropagationLossModel(propLoss);

    // TRANSMISSION

    // onoff send stack trace:
    // OnOffApplication::StartApplication
    // OnOffApplication::ScheduleStartEvent
    // OnOffApplication::StartSending (Simulator::Schedule, after offInterval=0)
    // OnOffApplication::ScheduleNextTx
    // OnOffApplication::SendPacket (Simulator::Schedule, after (m_pktSize * 8 - m_residualBits)/m_cbrRate.GetBitRate()=800/8000=0.1s)
    // UdpSocketImpl::Send (m_socket->Send)
    // UdpSocketImpl::DoSend
    // UdpSocketImpl::DoSendTo
    // UdpL4Protocol::Send (m_udp->Send)
    // Ipv4L3Protocol::Send (IPv4:Send through m_downtarget callback)
    // Ipv4L3Protocol::SendRealOut (assume route exists)
    // NrNetDevice::Send??? (outInterface->Send, interface for NrUeNetDevice-NrNetDevice)
    // NrUeNetDevice::DoSend
    // EpcUeNas::Send (m_nas->Send)
    // MemberLteAsSapProvider<C>::SendSidelinkData (m_asSapProvider->SendSidelinkData)
    // LteUeRrc::DoSendSidelinkData (m_owner->DoSendSidelinkData)
    // MemberNrSlPdcpSapProvider<C>::TransmitNrSlPdcpSdu (slDrb->m_pdcp->GetNrSlPdcpSapProvider()->TransmitNrSlPdcpSdu(params))
    // LtePdcp::DoTransmitNrSlPdcpSdu  (m_pdcp->DoTransmitNrSlPdcpSdu)
    // MemberNrSlRlcSapProvider<C>::TransmitNrSlPdcpPdu  (m_nrSlRlcSapProvider->TransmitNrSlPdcpPdu)
    // LteRlcUm::DoTransmitNrSlPdcpPdu? (m_rlc->DoTransmitNrSlPdcpPdu)
    // -> m_txBuffer.emplace_back() now
    // LteRlcUm::DoReportNrSlBufferStatus
    // MemberNrSlMacSapProvider<C>::ReportNrSlBufferStatus (m_nrSlMacSapProvider->ReportNrSlBufferStatus)
    // NrSlUeMac::DoReportNrSlBufferStatus (m_mac->DoReportNrSlBufferStatus)
    // NrSlUeMacScheduler::SchedNrSlRlcBufferReq (m_nrSlUeMacScheduler->SchedNrSlRlcBufferReq)
    // NrSlUeMacSchedulerFixedMcs::DoSchedNrSlRlcBufferReq
    // NrMacSchedulerLCG::UpdateInfo (lcg.second->UpdateInfo)
    // NrSlUeMacSchedulerLC::UpdateLC (m_lcMap.at(params.lcid)->UpdateLC(params))

    // GAP

    // NrUePhy::StartEventLoop
    // NrUePhy::StartSlot
    //   MacUeMemberPhySapUser::SlotIndication (m_phySapUser->SlotIndication)
    //   NrSlUeMac::DoSlotIndication (m_mac->DoSlotIndication)
    //   NrSlUeMacScheduler::SchedNrSlTriggerReq (m_nrSlUeMacScheduler->SchedNrSlTriggerReq)
    //   NrSlUeMacSchedulerFixedMcs::DoSchedNrSlTriggerReq
    //   NrSlUeMacSchedulerFixedMcs::CheckForGrantsToPublish
    //   NrSlUeMac::SchedNrSlConfigInd? GetMac()->SchedNrSlConfigInd
    //   MemberNrSlMacSapUser<C>::NotifyNrSlTxOpportunity (itLc->second.macSapUser->NotifyNrSlTxOpportunity?)
    //   LteRlcUm:DoNotifyNrSlTxOpportunity (m_rlc->DoNotifyNrSlTxOpportunity)
    //   -> m_txBuffer.begin()->m_pdu->Copy()

    //   MemberNrSlMacSapProvider<C>::TransmitNrSlRlcPdu (m_nrSlMacSapProvider->TransmitNrSlRlcPdu)
    //   NrSlUeMac::DoTransmitNrSlRlcPdu (m_mac->DoTransmitNrSlRlcPdu)
    //     NrSlUeMacHarq::AddPacket (m_nrSlHarq->AddPacket)
    //     NrSlUeMacScheduler::NotifyNrSlRlcPduDequeue (m_nrSlUeMacScheduler->NotifyNrSlRlcPduDequeue)
    //   
    //   NrUePhy::StartNrSlSlot
    //   NrUePhy::StartNrSlVarTti after GetSymbolPeriod() * varTtiInfo.symStart <<<<<<<<<<<<< likely 1 symbol period
    //     NrUePhy::SlData
    //     NrUePhy::SendNrSlDataChannels after 1ns, varTtiDuration=GetSymbolPeriod() * varTtiInfo.symLength
    //     NrSpectrumPhy::StartTxSlDataFrames (m_spectrumPhy->StartTxDataFrames)
    //     MultiModelSpectrumChannel::StartTx (m_channel->StartTx)
    //     MultiModelSpectrumChannel::StartRx (Simulator::Schedule after m_propagationDelay->GetDelay(txMobility, receiverMobility)=0)
    //     NrSpectrumPhy::StartRx (receiver->StartRx) -> RECEPTION

    //   NrUePhy::EndNrSlVarTti after GetSymbolPeriod() * varTtiInfo.symLength (both CTRL and DATA)
    //     DATA: GetSymbolPeriod() * varTtiInfo.symLength <<<<<<<<<<<
    //     NrUePhy::StartSlot after m_phy->GetSlotPeriod after last period start
    //     NrUePhy::StartNrSlVarTti after GetSymbolPeriod() * nextVarTtiInfo.symStart after last slot start
    //       NrPhy

    // RECEPTION
    // NrSpectrumPhy:StartRxSlFrame
    // NrSpectrumPhy::EndRxSlFrame after duration (in params)
    // NrUePhy::PhyPsschPduReceived (through m_nrPhyRxPsschEndOkCallback)
    // NrSlUePhySapUser::ReceivePsschPhyPdu (after GetTbDecodeLatency(), 100µs by default)

    // Variables / configuation
    //       m_numSym: 
    //       Slot duration = m_slotPeriod = Seconds(0.001 / m_slotsPerSubframe);
    //         0.001 / (2^numerology) = 0.001/2^2 = 0.00025
    //       Symbol period = m_symbolPeriod = (m_slotPeriod / m_symbolsPerSlot)
    //         0.00025 / 14 = 0.000017857142857142857142857142857143e-5
    //       symLength = varTtiInfo.symLength = m_nrSlAllocInfoQueue.front()..slvarTtiInfoList.begin().symLength
    //         = currentSlot.slPscchSymLength?
    //       subcarrier spacing = (15*2^numerology) = 60kHz
    //       Channel bandwidth = 40MHz
    // (nr-sl-ue-mac)
    //   GetBwInRbs = GetChannelBandwidth / (subcarrierSpacing*subcarrierPerRb) = 40000 / (60*12) =~ 55
    //   GetTotalSubCh = GetBwInRbs / subChSize = 55 / 50 =~1
    //   GetNrSlSubChSize = 50
    //   Feedback: 
    //     symStart=12, 
    //     symLength=1, 
    //     rbStart=0, 
    //     rbLength=GetTotalSubCh()*pool->GetNrSlSubChSize() = 55/50=1
    //   Control: 
    //     symStart = currentSlot.slPscchSymStart, 0 from slSymbolStart
    //     symLength = currentSlot.slPscchSymLength, 1 from nr-sl-comm-resource-pool-factory
    //     rbStart = currentSlot.slPsschSubChStart * pool->GetNrSlSubChSize
    //     rbLength = currentSlot.numSlPscchRbs (5?)
    //   Data: 
    //     symStart=psschSymStart, 
    //     symlength=psschSymLength, 
    //     rbstart = currentSlot.slPsschSubChStart * pool->GetNrSlSubChSize, 
    //     rblength=currentSlot.slPsschSubChLength * pool->GetNrSlSubChSize
    
    // STACK TRACE END?

    // MemberNrSlUePhySapProvider<C>::SendPscchMacPdu (m_nrSlUePhySapProvider->SendPscchMacPdu)
    // NrPhy::DoSendPscchMacPdu (m_owner->DoSendPscchMacPdu)
    // NrPhy::SetPscchMacPdu
    // NrPhy::PopPscchPacketBurst?
    // 
    // ??? GAP ???


    // Optional: Enable trace for packet visualization
    // AsciiTraceHelper ascii;
    // wifiPhy.EnableAsciiAll(ascii.CreateFileStream("wifi-trace.tr"));
    // TODO: add trace

    Simulator::Stop(Seconds(2.5));
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}
