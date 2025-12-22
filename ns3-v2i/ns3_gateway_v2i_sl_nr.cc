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
 * Author: Thomas Roth <thomas.roth@nist.gov>
 * Modified by Hadhoum Hajjaj <hadhoum.hajjaj@nist.gov> to include V2I communication using wifi 
*/
#include <string>
#include <vector>

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"
#include "ns3/yans-wifi-helper.h"

#include "ns3/external-mobility-model.h"
#include "ns3/triggered-send-application.h"
#include "ns3/triggered-send-helper.h"
#include "ns3/gateway.h"
#include "ns3/seq-ts-size-header.h"

#include <iomanip>



using namespace ns3;

NS_LOG_COMPONENT_DEFINE("SimpleGateway");

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
class SimpleGateway : public Gateway 
{
public:
        // initialize a simple gateway where n = vehicles.GetN()
    SimpleGateway(NodeContainer vehicles);
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

SimpleGateway::SimpleGateway(NodeContainer vehicles)
    : Gateway(vehicles.GetN()),
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



void SimpleGateway::HandleReceiveWithTs(std::string id, Ptr<const Packet> packet,
    const Address &from, const Address &to,
    const SeqTsSizeHeader &header)
{
    Time txTime = header.GetTs();
    NS_LOG_INFO("At time " << Simulator::Now().As(Time::S)
    << ", Node " << id << " received a packet (sent at "
    << txTime.As(Time::S) << ")");
    uint32_t index = std::stoi(id);
    //m_count[index] += 1;
    m_lastRecvTime[index] = txTime; // store for reporting later
}


void SimpleGateway::DoInitialize(const std::vector<std::string> &data)
{
    DoUpdate(data);
}

void SimpleGateway::DoUpdate(const std::vector<std::string> &data)
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
    bool verboseLogs            = false;
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
        LogComponentEnable("Gateway", LOG_LEVEL_INFO);
        LogComponentEnable("SimpleGateway", LOG_LEVEL_ALL);
        LogComponentEnable("PacketSink", LOG_LEVEL_INFO);
        LogComponentEnable("OnOffApplication", LOG_LEVEL_INFO);
    }
    else
    {
        LogComponentEnable("Gateway", LOG_LEVEL_INFO);
        LogComponentEnable("SimpleGateway", LOG_LEVEL_INFO);
    }

    uint16_t totalNodes = numberOfVehicles + 1;
    NodeContainer nodes;
    nodes.Create(totalNodes);

    // TODO: Traffic light position should be coming from CARLA??
    Ptr<ListPositionAllocator> positionAllocator = CreateObject<ListPositionAllocator>();
    for (uint16_t i = 0; i < numberOfVehicles; i++)
    {
        positionAllocator->Add(Vector(-84.98, -20.00, 0.5)); // vehicle(s) initial position
    }
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

    // TODO: Car position should be coming from CARLA??

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
    // Sidelink examples from https://github.com/usnistgov/psc-ns3
    // C-V2X LTE, Sidelink, in coverage
    // see https://github.com/usnistgov/psc-ns3/blob/psc-7.0/src/lte/test/test-sidelink-in-coverage-comm.cc

    // C-V2X LTE, Sidelink, out of coverage
    // see https://github.com/usnistgov/psc-ns3/blob/psc-7.0/src/lte/test/test-sidelink-out-of-coverage-comm.cc

    // Documentation from https://github.com/usnistgov/psc-ns3/blob/psc-7.0/src/lte/doc/source/lte-user-sidelink.inc

    // 1. Round robin sidelink scheduler (SCHED SAP and CSCHED SAP)
    // Time Resource Pattern (TRP) Index, for Physical Sidelink Shared Channel (PSSCH)
    Config::SetDefault("ns3::RrSlFfMacScheduler::Itrp", UintegerValue(0));
    // Number of RBs allocated per UE for Sidelink
    Config::SetDefault("ns3::RrSlFfMacScheduler::SlGrantSize", UintegerValue(5));

    // 2. Frequency and bandwidth 
    // EARFCN (Uplink and Downlink) <-> Frequency + LTE band (absolute radio-frequency channel number)
    // see https://www.sqimway.com/lte_band.php
    // Using LTE band 47 (https://www.qualcomm.com/products/automotive/qualcomm-c-v2x-9150)
    // Uplink and Downlink have same frequency!?!?!?
    Config::SetDefault("ns3::LteEnbNetDevice::DlEarfcn", UintegerValue(54890));
    Config::SetDefault("ns3::LteUeNetDevice::DlEarfcn", UintegerValue(54890));
    Config::SetDefault("ns3::LteEnbNetDevice::UlEarfcn", UintegerValue(54890));
    Config::SetDefault("ns3::LteEnbNetDevice::DlBandwidth", UintegerValue(70));
    Config::SetDefault("ns3::LteEnbNetDevice::UlBandwidth", UintegerValue(70));    

    // 3. Error models
    // Enables the PSSCH phy error model (labels corrupted "Tbs"")
    Config::SetDefault("ns3::LteSpectrumPhy::SlDataErrorModelEnabled", BooleanValue(true));
    Config::SetDefault("ns3::LteSpectrumPhy::SlCtrlErrorModelEnabled", BooleanValue(true));
    // drop all receptions on colliding RBs (label as corrupted)
    Config::SetDefault("ns3::LteSpectrumPhy::DropRbOnCollisionEnabled", BooleanValue(false));

    // 4. Tranmission power, in dBm
    // See https://www.qualcomm.com/products/automotive/qualcomm-c-v2x-9150
    Config::SetDefault("ns3::LteUePhy::TxPower", DoubleValue(23.0));
    // Note sure this is correct
    Config::SetDefault("ns3::LteEnbPhy::TxPower", DoubleValue(43.0));

    // 5. LTE/Sidelink Helpers
    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    lteHelper->SetEpcHelper(epcHelper);

    Ptr<LteSidelinkHelper> proseHelper = CreateObject<LteSidelinkHelper>();
    proseHelper->SetLteHelper(lteHelper);

    // 6. Propagation loss model
    lteHelper->SetAttribute("PathlossModel", StringValue("ns3::Cost231PropagationLossModel"));
    
    // 7. Enable Sidelink 
    lteHelper->SetAttribute("UseSidelink", BooleanValue(true));

    // 8. Create the eNodeB 
    NodeContainer enbNode;
    enbNode.Create(1);

    // The UEs are already created

    // 9. Position the eNB, somewhere around the car and the traffic light

    Ptr<ListPositionAllocator> positionAllocEnb = CreateObject<ListPositionAllocator>();
    positionAllocEnb->Add (Vector(-80.0, -140.0, 40.0));

    MobilityHelper mobilityeNodeB;
    mobilityeNodeB.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityeNodeB.SetPositionAllocator(positionAllocEnb);
    mobilityeNodeB.Install(enbNode);

    // 10. Install LTE devides on nodes
    NetDeviceContainer enbDevs = lteHelper->InstallEnbDevice(enbNode);
    NetDeviceContainer ueDevs = lteHelper->InstallUeDevice(nodes);

    // 11. Sidelink configutation pools, eNB

    Ptr<LteSlEnbRrc> enbSidelinkConfiguration = CreateObject<LteSlEnbRrc> ();
    enbSidelinkConfiguration->SetSlEnabled (true);

    LteRrcSap::SlCommTxResourcesSetup pool;

    bool scheduledpool = true;
    if (scheduledpool)
    {
        pool.setup = LteRrcSap::SlCommTxResourcesSetup::SCHEDULED;
        //BSR timers
        pool.scheduled.macMainConfig.periodicBsrTimer.period = LteRrcSap::PeriodicBsrTimer::sf16;
        pool.scheduled.macMainConfig.retxBsrTimer.period = LteRrcSap::RetxBsrTimer::sf640;
        //MCS
        pool.scheduled.haveMcs = true;
        pool.scheduled.mcs = 16;
        //resource pool
        LteSlResourcePoolFactory pfactory;
        pfactory.SetHaveUeSelectedResourceConfig(false);

        //Control
        pfactory.SetControlPeriod("sf40");
        pfactory.SetControlBitmap(0x00000000FF);
        pfactory.SetControlOffset(0);
        pfactory.SetControlPrbNum(22);
        pfactory.SetControlPrbStart(0);
        pfactory.SetControlPrbEnd(49);

        pool.scheduled.commTxConfig = pfactory.CreatePool();
    }
    else
    {
        pool.setup = LteRrcSap::SlCommTxResourcesSetup::UE_SELECTED;
        pool.ueSelected.havePoolToRelease = false;
        pool.ueSelected.havePoolToAdd = true;
        pool.ueSelected.poolToAddModList.nbPools = 1;
        pool.ueSelected.poolToAddModList.pools[0].poolIdentity = 1;
        // resource pool (default configuration is UE selected)
        LteSlResourcePoolFactory pfactory;
        pool.ueSelected.poolToAddModList.pools[0].pool = pfactory.CreatePool();
    }

    uint32_t groupL2Address = 255;

    enbSidelinkConfiguration->AddPreconfiguredDedicatedPool(groupL2Address, pool);
    lteHelper->InstallSidelinkConfiguration(enbDevs, enbSidelinkConfiguration);

    // 12. Sidelink congiguration pools, UEs

    //pre-configuration for the UEs
    Ptr<LteSlUeRrc> ueSidelinkConfiguration = CreateObject<LteSlUeRrc>();
    ueSidelinkConfiguration->SetSlEnabled(true);
    LteRrcSap::SlPreconfiguration preconfiguration;
    ueSidelinkConfiguration->SetSlPreconfiguration(preconfiguration);
    lteHelper->InstallSidelinkConfiguration(nodes, ueSidelinkConfiguration);

    // END NEW SIDELINK CODE

    // install an IP network stack
    InternetStackHelper stack;
    // Install the IP (Internet) protocol stack on all nodes.
    // This includes both the vehicle(s) and the traffic light node.
    // Required for sending/receiving IP-based packets 
    stack.Install(nodes);

    // BEG OLD IP CONFIG

    // allocate IPv4 Addresses from 192.168.1.0/24 
    // Ipv4AddressHelper address;
    // address.SetBase("192.168.1.0", "255.255.255.0");
    // Ipv4InterfaceContainer interfaces = address.Assign(devices);

    //const Ipv4Address broadcastAddress("255.255.255.255");
    // const Ipv4Address broadcastAddress("192.168.1.255");
    // const uint16_t applicationPort = 8100;

    // END OLD IP CONFIG

    // BEG NEW IP CONFIG

    Ipv4InterfaceContainer ueIpIface;
    ueIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        Ptr<Node> ueNode = ueNodes.Get(u);
        // Set the default gateway for the UE
        Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(),1);
    }

    lteHelper->Attach(ueDevs);

    // Sidelink Traces
    lteHelper->EnableSlPscchMacTraces();
    lteHelper->EnableSlPsschMacTraces();
    lteHelper->EnableSlTxPhyTraces();
    lteHelper->EnableSlRxPhyTraces();
    lteHelper->EnableSlPscchRxPhyTraces();

    // Activate sidelink bearers (not sure if needed)

    Ipv4Address groupAddress4("225.0.0.0"); // use multicast address as destination
    Ptr<LteSlTft> tft = Create<LteSlTft>(LteSlTft::BIDIRECTIONAL, groupAddress4, groupL2Address);
    proseHelper->ActivateSidelinkBearer(Seconds(0.0), ueDevs, tft);


    // END NEW IP CONFIG

    // Gateway (vehicle side)
    // Create a new NodeContainer to store only the vehicle nodes.
    // This is important because 'nodes' contains both vehicles and the traffic light.
    // The Gateway logic should only interact with the vehicles, not the traffic light.
    NodeContainer vehicleNodes;
    for (uint16_t i = 0; i < numberOfVehicles; i++)
        vehicleNodes.Add(nodes.Get(i));
    SimpleGateway gateway(vehicleNodes);

    // install the applications
    for (uint32_t i = 0; i < numberOfVehicles; i++)
    {
        Ptr<Node> vehicle = nodes.Get(i);
        Ptr<ExternalMobilityModel> mobilityModel = vehicle->GetObject<ExternalMobilityModel>();
        mobilityModel->TraceConnectWithoutContext("CourseChange", MakeCallback(&ReportMobility));

        TriggeredSendHelper sendHelper("ns3::UdpSocketFactory", InetSocketAddress(groupAddress4, applicationPort));
        sendHelper.SetAttribute("PacketInterval", TimeValue(MilliSeconds(100)));
        ApplicationContainer clientApps = sendHelper.Install(vehicle);
        clientApps.Start(Time(0));

        PacketSinkHelper sinkHelper("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), applicationPort));
        sinkHelper.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
        ApplicationContainer serverApps = sinkHelper.Install(vehicle);

        serverApps.Get(0)->TraceConnect("RxWithSeqTsSize", std::to_string(i), MakeCallback(&SimpleGateway::HandleReceiveWithTs, &gateway));
        serverApps.Start(Time(0));
    }

    // === Traffic light broadcasts ===
    Ptr<Node> trafficLight = nodes.Get(numberOfVehicles);
    OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(groupAddress4, applicationPort));

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

    gateway.Connect(serverAddress, serverPort);

    // Optional: Enable trace for packet visualization
    // AsciiTraceHelper ascii;
    // wifiPhy.EnableAsciiAll(ascii.CreateFileStream("wifi-trace.tr"));
    // TODO: add trace


    Simulator::Run();
    Simulator::Destroy();

    return 0;
}
