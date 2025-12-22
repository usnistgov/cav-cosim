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

#include "ns3/external-mobility-model.h"
#include "ns3/triggered-send-application.h"
#include "ns3/triggered-send-helper.h"
#include "ns3/text-gateway.h"
#include "ns3/seq-ts-size-header.h"

#include <iomanip>



using namespace ns3;

NS_LOG_COMPONENT_DEFINE("SimpleTextGateway");

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
class SimpleTextGateway : public TextGateway 
{
public:
        // initialize a simple gateway where n = vehicles.GetN()
    SimpleTextGateway(NodeContainer vehicles);
    void HandleReceiveWithTs(std::string id, Ptr<const Packet> packet,
                             const Address &from, const Address &to,
                             const SeqTsSizeHeader &header);

private:
        // this function handles processing the first message received from the remote server
        //  the simple gateway doesn't require any initialization, so this just calls DoUpdate
    virtual void DoInitialize(const std::vector<std::string> & data);

        // this function handles processing messages received from the remote server
    virtual void DoUpdate(const std::vector<std::string> & data);

    NodeContainer m_vehicles;
    std::vector<Time> m_lastRecvTime;
    std::vector<int> m_lastLightStatus;
    std::vector<float> m_lastTimeRemaining;
    Vector m_trafficLightPosition;

};

SimpleTextGateway::SimpleTextGateway(NodeContainer vehicles)
    : TextGateway(vehicles.GetN()),
      m_vehicles(vehicles),
      m_lastRecvTime(vehicles.GetN(), Seconds(0)),
      m_lastLightStatus(vehicles.GetN(), 0),
      m_lastTimeRemaining(vehicles.GetN(), 0.0f)
{
    // Traffic light is the node after the last vehicle node
    Ptr<Node> trafficLightNode = NodeList::GetNode(m_vehicles.GetN()); 
    Ptr<MobilityModel> tlMobility = trafficLightNode->GetObject<MobilityModel>();
    m_trafficLightPosition = tlMobility->GetPosition();
}



void SimpleTextGateway::HandleReceiveWithTs(std::string id, Ptr<const Packet> packet,
    const Address &from, const Address &to,
    const SeqTsSizeHeader &header)
{
    Time txTime = header.GetTs();
    NS_LOG_INFO("At time " << Simulator::Now().As(Time::S)
    << ", Node " << id << " received a packet (sent at "
    << txTime.As(Time::S) << ", delta is " << (Simulator::Now()-txTime).As(Time::S) << ")");
    uint32_t index = std::stoi(id);
    //m_count[index] += 1;
    m_lastRecvTime[index] = txTime; // store for reporting later
}


void SimpleTextGateway::DoInitialize(const std::vector<std::string> &data)
{
    DoUpdate(data);
}

void SimpleTextGateway::DoUpdate(const std::vector<std::string> &data)
{
    static Time lastUpdate = Seconds(0);
    Time now = Simulator::Now();
    Time delta = now - lastUpdate;

    NS_LOG_INFO(">>> [ns-3] DoUpdate at " << now.GetSeconds() << " s, Δt = " << delta.GetSeconds() << " s");

    lastUpdate = now;

    static const uint32_t ELEMENTS_PER_VEHICLE = 9; // Position_{x,y,z} + Velocity_{x,y,z} + SendFlag + LightStatus + TimeRemaining

    for (uint32_t i = 0; i < m_vehicles.GetN(); i++) 
    {

        Ptr<Node> vehicle = m_vehicles.Get(i);
        uint32_t dataIndex = i * ELEMENTS_PER_VEHICLE;

        if (dataIndex + ELEMENTS_PER_VEHICLE > data.size()) 
        {
            NS_FATAL_ERROR("ERROR: received data has insufficient size");
        }

        Vector position(std::stof(data[dataIndex]), std::stof(data[dataIndex + 1]), std::stof(data[dataIndex + 2]));        
        vehicle->GetObject<ExternalMobilityModel>()->SetPosition(position);

        Vector velocity(std::stof(data[dataIndex + 3]), std::stof(data[dataIndex + 4]), std::stof(data[dataIndex + 5]));
        vehicle->GetObject<ExternalMobilityModel>()->SetVelocity(velocity);

        int light_status = std::stoi(data[dataIndex + 7]);
        float time_remaining = std::stof(data[dataIndex + 8]);

        m_lastLightStatus[i] = light_status;
        m_lastTimeRemaining[i] = time_remaining;

        // handle the send flag
        if (std::stoi(data[dataIndex+6]))
        {
            DynamicCast<TriggeredSendApplication>(vehicle->GetApplication(0))->Send(3);
            NS_LOG_INFO("At time " << Simulator::Now().As(Time::S) << ", Node " << i << " sent a broadcast");
            NS_LOG_INFO("Traffic Light Status: " << light_status << ", Time Remaining: " << time_remaining << "s");
        }

        // Respond with the last received timestamp instead of count

        // SetValue(i, std::to_string(m_lastRecvTime[i].GetSeconds()));
        std::ostringstream response;
        response << m_lastRecvTime[i].GetSeconds() << "," 
                 << m_lastLightStatus[i] << "," 
                 << std::fixed << std::setprecision(2) << m_lastTimeRemaining[i] << ","
                 << std::fixed << std::setprecision(2)
                 << m_trafficLightPosition.x << "," 
                 << m_trafficLightPosition.y << "," 
                 << m_trafficLightPosition.z;
        SetValue(i, response.str());
        
        

        //NS_LOG_INFO("Sending response for node " << i << ": last recv time = " << m_lastRecvTime[i].As(Time::S));
        NS_LOG_INFO("Sending response for node " << i
            << ": time = " << m_lastRecvTime[i].As(Time::S)
            << ", state = " << m_lastLightStatus[i]
            << ", remaining = " << m_lastTimeRemaining[i] << "s"
            << ", lightPos = (" << m_trafficLightPosition.x
            << ", " << m_trafficLightPosition.y
            << ", " << m_trafficLightPosition.z << ")");
        

    }
    SendResponse(); // format and send a response based on the most recent SetValue
}

void ReportMobility(Ptr<const MobilityModel> mobility)
{
    NS_LOG_DEBUG("At time " << Simulator::Now().As(Time::S)
                 << ", Node " << mobility->GetObject<Node>()->GetId()
                 << ", Position " << mobility->GetPosition()
                 << ", Velocity " << mobility->GetVelocity());
}

int main(int argc, char *argv[])
{
    bool verboseLogs            = true;
    uint16_t numberOfVehicles   = 1;
    uint16_t serverPort         = 8100;
    std::string serverAddress   = "127.0.0.1";

    CommandLine cmd(__FILE__);
    cmd.AddValue("verbose", "Enable/disable detailed log output", verboseLogs);
    cmd.AddValue("numberOfNodes", "Number of vehicle nodes to simulate", numberOfVehicles);
    cmd.AddValue("serverPort", "Port number of the UDP Server", serverPort);
    cmd.AddValue("serverAddress", "Address of the UDP Server", serverAddress);
    cmd.Parse(argc, argv);

    Time::SetResolution(Time::NS); // timestamp has nanosecond resolution

    if (verboseLogs)
    {
        LogComponentEnable("Gateway", LOG_LEVEL_ALL);
        LogComponentEnable("SimpleTextGateway", LOG_LEVEL_ALL);
        LogComponentEnable("PacketSink", LOG_LEVEL_INFO);
        LogComponentEnable("OnOffApplication", LOG_LEVEL_INFO);

    }
    else
    {
        LogComponentEnable("Gateway", LOG_LEVEL_INFO);
        LogComponentEnable("SimpleTextGateway", LOG_LEVEL_INFO);
    }

    uint16_t totalNodes = numberOfVehicles + 1;
    NodeContainer nodes;
    nodes.Create(totalNodes);

    Ptr<ListPositionAllocator> positionAllocator = CreateObject<ListPositionAllocator>();
    for (uint16_t i = 0; i < numberOfVehicles; i++)
    {
        // TODO: Car position should be coming from CARLA??
        positionAllocator->Add(Vector(-84.98, -20.00, 0.5)); // vehicle(s) initial position
    }
    // TODO: Traffic light position should be coming from CARLA??
    positionAllocator->Add(Vector(-88.21, -145.86, 20.0)); // traffic light at fixed position

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


    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ExternalMobilityModel");
    mobility.SetPositionAllocator(positionAllocator);
    for (uint16_t i = 0; i < numberOfVehicles; i++)
        mobility.Install(nodes.Get(i));
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(nodes.Get(numberOfVehicles));

    // BEG OLD WIFI CODE
    // WifiHelper wifi;
    // wifi.SetStandard(WIFI_STANDARD_80211b);
    // wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 // "DataMode", StringValue("DsssRate1Mbps"),
                                 // "ControlMode", StringValue("DsssRate1Mbps"));

    // YansWifiPhyHelper wifiPhy;
    // wifiPhy.Set("TxPowerStart", DoubleValue(20.0));
    // wifiPhy.Set("TxPowerEnd", DoubleValue(20.0));
    // wifiPhy.Set("RxSensitivity", DoubleValue(-95.0));
    // wifiPhy.Set("CcaEdThreshold", DoubleValue(-95.0));

    // YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    // wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");

    // Easier propagation (Exponent lowered)
    // wifiChannel.AddPropagationLoss("ns3::LogDistancePropagationLossModel", "Exponent", DoubleValue(2.0));
    // Commented out fading
    // wifiChannel.AddPropagationLoss("ns3::NakagamiPropagationLossModel");

    // wifiPhy.SetChannel(wifiChannel.Create());

    // WifiMacHelper wifiMac;
    // wifiMac.SetType("ns3::AdhocWifiMac");
    // NetDeviceContainer devices = wifi.Install(wifiPhy, wifiMac, nodes);
    // END OLD WIFI CODE

    // BEG NEW SIDELINK CODE
    // Sidelink examples from https://gitlab.com/cttc-lena/nr/-/blob/nr-v2x-dev/examples/nr-v2x-examples/cttc-nr-v2x-demo-simple.cc?ref_type=heads
    
    // NETWORKING INTRO

    // 5G/New Radio network layers:
    // L0: PHY
    // L1: MAC: Media Access Control
    // L2: RLC: Radio Link Control
    // L3: PDCP: Packet Data Convergence Protocol
    // L4: RRC: Radio Resource Control
    // L5: NAS

    // Frequency subdivision: 
    // Frame: entire bandwidth
    // Resource block: 12 subcarriers
    // Subcarrier: length depends on numerology

    // Time subdivision:
    // Frame: 10ms
    // Subframe: 1ms
    // Slot: 6-7 OFDM symbol for LTE, 12-14 OFDM symbols for NR
    // OFDM symbol: length depends on numerology

    // Numerology: Subcarrier specing (mu): higher frequency -> higher number of slots
    // Resource Block: 1 slot in time and 12 subcarriers in frequency

    // Sidelink
    // Mode 1: inside eNodeB coverage area
    // Mode 2: outside eNodeB coverage area

    // Physical Sidelink Control Channel: for signalling traffic
    // Physical Sidelink Shared Channel: for data

    // 1. Configuration parameters
    
    // Radio Link Control, Unacklownedged Mode, maximum transmission buffer size
    // Service Data Units (SDUd) above get discarded
    Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue(999999999));

    // 2. Logging
    
    if (verboseLogs)
    {
        LogLevel logLevel = (LogLevel)(LOG_PREFIX_FUNC | LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL);
        LogComponentEnable("UdpClient", LOG_LEVEL_ALL);
        LogComponentEnable("UdpServer", LOG_LEVEL_ALL);
        // LogComponentEnable("LtePdcp", LOG_LEVEL_ALL);
        // LogComponentEnable("LteRlc", LOG_LEVEL_ALL);
        // LogComponentEnable("LteRlcUm", LOG_LEVEL_ALL);
        // LogComponentEnable("LteRlcAm", LOG_LEVEL_ALL);
        // LogComponentEnable("LteRlcTm", LOG_LEVEL_ALL);
        // LogComponentEnable("NrSlUeRrc", LOG_LEVEL_ALL);
        // LogComponentEnable("NrSlUeMac", LOG_LEVEL_ALL);
        // LogComponentEnable("NrPhy", LOG_LEVEL_ALL);
        LogComponentEnable("NrUePhy", LOG_LEVEL_INFO);
        //LogComponentEnable("SpectrumPhy", LOG_LEVEL_ALL);
        LogComponentEnable("NrSpectrumPhy", LOG_LEVEL_ALL);
        LogComponentEnable("SingleModelSpectrumChannel", LOG_LEVEL_ALL);
        LogComponentEnable("MultiModelSpectrumChannel", LOG_LEVEL_ALL);

    }

    // 3. Helpers
    // Evolved Packet Core (EPC) / System Architecture Evolution (SAE) Core NETWORKS
    // Defines User Equipment, eNodeB/gNodeB, Mobility Management Entity (MME), etc.
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetEpcHelper(epcHelper);

    // 4. Configuration of Operation band for sidelink
    // An Operation Band has 1+ Component carrier (CC), which has 1+ Bandwidth part
    CcBwpCreator ccBwpCreator;

    // Bandwidth part numerology mu 
    // Higher mu -> higher frequency -> lower duration
    // subcarrier spacing = 15*2^mu
    uint16_t numerologyBwpSl = 2;
    // Central frequency of bandwidth (DlEarfcn-UlEarfcn)
    // EARFCN: E-UTRA absolute radio-frequency channel number
    // E-UTRA: Evolved UMTS Terrestrial Radio Access
    // EARFCN (Uplink and Downlink) <-> Frequency + LTE band
    // see https://www.sqimway.com/lte_band.php
    // Using LTE band 47 (https://www.qualcomm.com/products/automotive/qualcomm-c-v2x-9150)
    // Uplink and Downlink have same frequency!?!?!?
    double centralFrequencyBandSl = 5.89e9;
    // Sidelink bandwidth, in multiple of 100 KHz: 400 = 40 MHz
    // n47: 10,20,30,40MHz?
    uint16_t bandwidthBandSl = 400;
    // Number of Component Carriers per band
    const uint8_t numCcPerBand = 1;

    // Scenario: do not seem to be used for anything!
    // RMa: LineOfSight/not
    // UMa: LineOfSight/not
    // UMi: LineOfSight/not
    // InH, OfficeOpen/Mixed, LineOfSight/not
    // V2V, Urban/Highway
    CcBwpCreator::SimpleOperationBandConf bandConfSl(centralFrequencyBandSl,
                                                     bandwidthBandSl,
                                                     numCcPerBand,
                                                     BandwidthPartInfo::V2V_Highway);

    OperationBandInfo bandSl = ccBwpCreator.CreateOperationBandContiguousCc(bandConfSl);
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({bandSl});

    // Presumably ThreeGppPropagationLossModel
    // TODO: this might be where to add custom propagation losses
    // Channel coherence time?
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(MilliSeconds(100)));
    // Time between updated of channel condition (line of sight, indoor/outdoor, penetration loss)
    nrHelper->SetChannelConditionModelAttribute("UpdatePeriod", TimeValue(MilliSeconds(0)));
    // To disable shadow fading (?)
    nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));

    nrHelper->InitializeOperationBand(&bandSl);

    // 4. ??
    Packet::EnableChecking();
    Packet::EnablePrinting();

    // 5. Configuration for all nodes
    
    // Delay for S1-U (User plane) link, between eNodeB/gNodeB and Service Gateway (SGW)
    // Likely irrelevant for sidelink!
    epcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));

    // User Equipment Antennas
    // Not sure if it matters
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(2));
    // Isotropic goes in all directions
    // TODO: could replace isotropic antenna with parabolic antenna (Beamwidth/Orientation)
    nrHelper->SetUeAntennaAttribute("AntennaElement", PointerValue(CreateObject<IsotropicAntennaModel>()));

    // Tranmission power, in dBm
    // See https://www.qualcomm.com/products/automotive/qualcomm-c-v2x-9150
    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(23));
    
    // Set User Equipment MAC type to Sidelink
    nrHelper->SetUeMacTypeId(NrSlUeMac::GetTypeId());
    // Random resource/slot selection, *not* based on sensing (PHY can sense sidelink resources)
    nrHelper->SetUeMacAttribute("EnableSensing", BooleanValue(false));
    // Resource/Slot selection window start
    nrHelper->SetUeMacAttribute("T1", UintegerValue(2));
    // Resource/Slot selection window end
    nrHelper->SetUeMacAttribute("T2", UintegerValue(33));
    // Active pool (??)
    nrHelper->SetUeMacAttribute("ActivePoolId", UintegerValue(0));

    // Set Bandwidth part manager type to Sidelink 
    nrHelper->SetBwpManagerTypeId(TypeId::LookupByName("ns3::NrSlBwpManagerUe"));

    // OLD code for MCPTT
    // uint8_t bwpIdForGbrMcptt = 0;
    // nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_MC_PUSH_TO_TALK", UintegerValue(bwpIdForGbrMcptt));
    // std::set<uint8_t> bwpIdContainer;
    // bwpIdContainer.insert(bwpIdForGbrMcptt);

    // NEW code for V2X
    // The bandwidth part manager algorithm returns an index for Quality of Service (QoS) Class Identifier (QCI)
    // Not sure is it should be GBR_V2X (guaranteed bit rate) or NGBR_V2X (non-guaranteed bit rate)
    uint8_t bwpIdForGbrV2x = 0;
    nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_V2X", UintegerValue(bwpIdForGbrV2x));
    std::set<uint8_t> bwpIdContainer;
    bwpIdContainer.insert(bwpIdForGbrV2x);    

    // Create net device container associated with all the bandwidth parts
    NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(nodes, allBwps);

    // Update configuration for each net device container
    for (auto it = ueNetDev.Begin(); it != ueNetDev.End(); ++it)
    {
        DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();
    }

    // 6. Sidelink configuration

    Ptr<NrSlHelper> nrSlHelper = CreateObject<NrSlHelper>();
    nrSlHelper->SetEpcHelper(epcHelper);

    // Set error model (ns3::NrEesmCcT1, ns3::NrEesmCcT2, ns3::NrEesmIrT1, ns3::NrEesmIrT2, ns3::NrLteMiErrorModel)
    // Incremental Redundancy Hybrid ARQ (IR-HARQ) type 1
    std::string errorModel = "ns3::NrEesmIrT1";
    nrSlHelper->SetSlErrorModel(errorModel);
    // Adaptive Modulation and Coding (AMC): 
    // use the output of an error model to find the optimal Modulation and Coding Scheme (MCS)
    nrSlHelper->SetUeSlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));

    // Set scheduler (unclear if round robin, proportinal fair, or max carrier to interference)
    // Allocate the Physical resource blocks (PRBs) of a spectrum to UE in Transmission Time Intervals (TTI)
    // Communicate scheduling allocation (grant)
    nrSlHelper->SetNrSlSchedulerTypeId(NrSlUeMacSchedulerFixedMcs::GetTypeId());
    // modulation and coding scheme, see MCS index table for implications
    nrSlHelper->SetUeSlSchedulerAttribute("Mcs", UintegerValue(14));

    // Not sure what this does
    nrSlHelper->PrepareUeForSidelink(ueNetDev, bwpIdContainer);

    // 7. Service Access Point (SAP) definitions

    // Create a resource (block) pool factory
    Ptr<NrSlCommResourcePoolFactory> ptrFactory = Create<NrSlCommResourcePoolFactory>();

    // Resource pool factory parameters
    std::vector<std::bitset<1>> slBitmap = {1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1};
    // Sidelink time resources?
    ptrFactory->SetSlTimeResources(slBitmap);
    // Sensing window start time in ms
    ptrFactory->SetSlSensingWindow(100);
    // Number of slots in selection window
    ptrFactory->SetSlSelectionWindow(5);
    // Number of resource blocks allocated to physical sidelink control channel (PSCCH) transmission
    ptrFactory->SetSlFreqResourcePscch(10);
    // Number of resource blocks (RBs) per subchannel
    ptrFactory->SetSlSubchannelSize(50);
    // Maximum number of reserved PSCCH/PSSCH resources
    ptrFactory->SetSlMaxNumPerReserve(3);
    std::list<uint16_t> resourceReservePeriodList = {0, 100}; // in ms
    ptrFactory->SetSlResourceReservePeriodList(resourceReservePeriodList);
    // Once parameters are configured, create the resource pool definition
    LteRrcSap::SlResourcePoolNr slResourcePoolNr = ptrFactory->CreatePool();

    // The resource pool configuration definition refers to the previously created resource pool
    LteRrcSap::SlResourcePoolConfigNr slresoPoolConfigNr;
    slresoPoolConfigNr.haveSlResourcePoolConfigNr = true;
    uint16_t poolId = 0;
    LteRrcSap::SlResourcePoolIdNr slResourcePoolIdNr;
    slResourcePoolIdNr.id = poolId;
    slresoPoolConfigNr.slResourcePoolId = slResourcePoolIdNr;
    slresoPoolConfigNr.slResourcePool = slResourcePoolNr;

    // The Bandwidth part pool configuration definition refers to the resource pool configuration definition
    LteRrcSap::SlBwpPoolConfigCommonNr slBwpPoolConfigCommonNr;
    slBwpPoolConfigCommonNr.slTxPoolSelectedNormal[slResourcePoolIdNr.id] = slresoPoolConfigNr;

    // Bandwidth part definition 
    LteRrcSap::Bwp bwp;
    // Numerology used earlier
    bwp.numerology = numerologyBwpSl;
    // NR has 12-14 OFDM symbols
    bwp.symbolsPerSlots = 14;
    // resource block per resource block group
    bwp.rbPerRbg = 1;
    // Sidelink bandwidth used earlier
    bwp.bandwidth = bandwidthBandSl;

    // Sidelink bandwidth part definition
    LteRrcSap::SlBwpGeneric slBwpGeneric;
    slBwpGeneric.bwp = bwp;
    // add number of OFDM symbols
    slBwpGeneric.slLengthSymbols = LteRrcSap::GetSlLengthSymbolsEnum(14);
    // add start OFDM symbol
    slBwpGeneric.slStartSymbol = LteRrcSap::GetSlStartSymbolEnum(0);

    // Create Bandwidth configuration definition
    LteRrcSap::SlBwpConfigCommonNr slBwpConfigCommonNr;
    slBwpConfigCommonNr.haveSlBwpGeneric = true;
    slBwpConfigCommonNr.slBwpGeneric = slBwpGeneric;
    slBwpConfigCommonNr.haveSlBwpPoolConfigCommonNr = true;
    slBwpConfigCommonNr.slBwpPoolConfigCommonNr = slBwpPoolConfigCommonNr;

    // Create sidelink frequency configuration definitions
    LteRrcSap::SlFreqConfigCommonNr slFreConfigCommonNr;
    for (const auto& it : bwpIdContainer)
    {
        slFreConfigCommonNr.slBwpList[it] = slBwpConfigCommonNr;
    }

    // Create time division duplex configuration definition
    // Unclear what the F is for!!
    LteRrcSap::TddUlDlConfigCommon tddUlDlConfigCommon;
    tddUlDlConfigCommon.tddPattern = "DL|DL|DL|F|UL|UL|UL|UL|UL|UL|";

    // Create a general preconfiguration definition
    LteRrcSap::SlPreconfigGeneralNr slPreconfigGeneralNr;
    slPreconfigGeneralNr.slTddConfig = tddUlDlConfigCommon;

    // Create UE preconfiguration definition
    LteRrcSap::SlUeSelectedConfig slUeSelectedPreConfig;
    // probability of keeping resource
    slUeSelectedPreConfig.slProbResourceKeep = 0;

    // Create Physical Sidelink Shared Channel (PSSCH) transmission parameter definitions
    LteRrcSap::SlPsschTxParameters psschParams;
    psschParams.slMaxTxTransNumPssch = 5;
    // Create Physical Sidelink Shared Channel (PSSCH) transmission configuration definitions
    LteRrcSap::SlPsschTxConfigList pscchTxConfigList;
    pscchTxConfigList.slPsschTxParameters[0] = psschParams;
    // Associate PSSCH configuration to UE preconfiguration
    slUeSelectedPreConfig.slPsschTxConfigList = pscchTxConfigList;

    // Create Sidelink preconfiguration
    LteRrcSap::SidelinkPreconfigNr slPreConfigNr;
    slPreConfigNr.slPreconfigGeneral = slPreconfigGeneralNr;
    slPreConfigNr.slUeSelectedPreConfig = slUeSelectedPreConfig;
    slPreConfigNr.slPreconfigFreqInfoList[0] = slFreConfigCommonNr;

    // Install Sidelink preconfiguration on all devices
    nrSlHelper->InstallNrSlPreConfiguration(ueNetDev, slPreConfigNr);

    // END NEW SIDELINK CODE

    // BEG OLD IP CONFIG

    // install an IP network stack
    // InternetStackHelper stack;
    // Install the IP (Internet) protocol stack on all nodes.
    // This includes both the vehicle(s) and the traffic light node.
    // Required for sending/receiving IP-based packets 
    // stack.Install(nodes);

    // allocate IPv4 Addresses from 192.168.1.0/24 
    // Ipv4AddressHelper address;
    // address.SetBase("192.168.1.0", "255.255.255.0");
    // Ipv4InterfaceContainer interfaces = address.Assign(devices);

    //const Ipv4Address broadcastAddress("255.255.255.255");
    // const Ipv4Address broadcastAddress("192.168.1.255");
    // const uint16_t applicationPort = 8100;

    // END OLD IP CONFIG

    // BEG NEW IP CONFIG

    // Install Internet stack (ARP, IPv4, ICMP, UDP, TCP, routing protocols) on all nodes
    InternetStackHelper internet;
    internet.Install(nodes);

    // Set client/server addresses
    // MULTICAST: Ipv4Address groupAddress4("225.0.0.0");
    Ipv4Address groupAddress4("255.255.255.255");
    uint16_t applicationPort = 8000;
    Address remoteAddress = InetSocketAddress(groupAddress4, applicationPort);
    Address localAddress = InetSocketAddress(Ipv4Address::GetAny(), applicationPort);

    // Stores information about the sidelink channel
    SidelinkInfo slInfo;
    // MULTICAST: slInfo.m_castType = SidelinkInfo::CastType::Groupcast;
    slInfo.m_castType = SidelinkInfo::CastType::Broadcast;
    // L2 ID?
    slInfo.m_dstL2Id = 255;
    // Resource Reservation Interval
    slInfo.m_rri = MilliSeconds(100);
    // Packet Delay Budget (PDB): maximum allowable delay between user equipment and Packet Data Network Gateway (P-GW)
    slInfo.m_pdb = Seconds(0);
    // Hybrid automatic repeat request (HARQ), for error control and correction: 
    slInfo.m_harqEnabled = true;

    // Assign IP addresses to all node network devices
    Ipv4InterfaceContainer ueIpIface = epcHelper->AssignUeIpv4Address(ueNetDev);

    // Set default gateway
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    for (uint32_t u = 0; u < nodes.GetN(); ++u)
    {
        Ptr<Node> ueNode = nodes.Get(u);
        // Set the default gateway for the UE
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
        // no backhaul / point to point / sidelink
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    // Traffic Flow Templates (TFT)
    Ptr<LteSlTft> tft = Create<LteSlTft>(LteSlTft::Direction::BIDIRECTIONAL, groupAddress4, slInfo);
    
    // Activation time for Sidelink bearers (???)
    nrSlHelper->ActivateNrSlBearer(Seconds(0.01), ueNetDev, tft);

    // END NEW IP CONFIG

    // Gateway (vehicle side)
    // Create a new NodeContainer to store only the vehicle nodes.
    // This is important because 'nodes' contains both vehicles and the traffic light.
    // The Gateway logic should only interact with the vehicles, not the traffic light.
    NodeContainer vehicleNodes;
    for (uint16_t i = 0; i < numberOfVehicles; i++)
        vehicleNodes.Add(nodes.Get(i));
    SimpleTextGateway gateway(vehicleNodes);

    // install the applications
    for (uint32_t i = 0; i < numberOfVehicles; i++)
    {
        Ptr<Node> vehicle = nodes.Get(i);
        Ptr<ExternalMobilityModel> mobilityModel = vehicle->GetObject<ExternalMobilityModel>();
        mobilityModel->TraceConnectWithoutContext("CourseChange", MakeCallback(&ReportMobility));

        //OLD: TriggeredSendHelper sendHelper("ns3::UdpSocketFactory", InetSocketAddress(groupAddress4, applicationPort));

        TriggeredSendHelper sendHelper("ns3::UdpSocketFactory", remoteAddress);
        sendHelper.SetAttribute("PacketInterval", TimeValue(MilliSeconds(100)));
        ApplicationContainer clientApps = sendHelper.Install(vehicle);
        clientApps.Start(Time(0));

        // OLD: PacketSinkHelper sinkHelper("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), applicationPort));
        PacketSinkHelper sinkHelper("ns3::UdpSocketFactory", localAddress);
        sinkHelper.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
        ApplicationContainer serverApps = sinkHelper.Install(vehicle);

        serverApps.Get(0)->TraceConnect("RxWithSeqTsSize", std::to_string(i), MakeCallback(&SimpleTextGateway::HandleReceiveWithTs, &gateway));
        serverApps.Start(Time(0));
    }

    // === Traffic light broadcasts ===
    Ptr<Node> trafficLight = nodes.Get(numberOfVehicles);
    // OLD OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(groupAddress4, port));
    OnOffHelper onoff("ns3::UdpSocketFactory", remoteAddress);
    onoff.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
    // UDP packet size for best effort (i.e. max)
    uint32_t udpPacketSizeBe = 200;
    // Data rate in kilobits per second for best effort (i.e. max)
    double dataRateBe = 16;
    std::string dataRateBeString = std::to_string(dataRateBe) + "kb/s";
    onoff.SetConstantRate(DataRate(dataRateBeString), udpPacketSizeBe);


    // Configure the OnOff application to transmit SPaT messages at 10 Hz:
    // - PacketSize = 100 bytes (800 bits)
    // - DataRate = 8 kbps => 8000 bits/sec ÷ 800 bits = 10 packets/sec
    // - OnTime = 1.0s and OffTime = 0.0s ensure continuous transmission without idle periods

    onoff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1.0]"));
    onoff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.0]"));
    onoff.SetAttribute("PacketSize", UintegerValue(100));
    onoff.SetAttribute("DataRate", StringValue("8kbps"));
    onoff.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
    onoff.SetAttribute("StartTime", TimeValue(Seconds(1.0)));

    onoff.Install(trafficLight);

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

    gateway.Connect(serverAddress, serverPort);

    // Optional: Enable trace for packet visualization
    // AsciiTraceHelper ascii;
    // wifiPhy.EnableAsciiAll(ascii.CreateFileStream("wifi-trace.tr"));
    // TODO: add trace

    Simulator::Stop(Seconds(2.5));
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}
