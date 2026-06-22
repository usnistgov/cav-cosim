/*
 * V2P (Vehicle-to-Pedestrian) gateway example for ns3-cosim.
 *
 * Adapted from ns3_gateway_v2i.cc (NIST: Thomas Roth, modified by Hadhoum Hajjaj).
 *
 * Topology:
 *   Node 0: ego vehicle (ExternalMobilityModel, PacketSink only - receiver)
 *   Node 1: pedestrian (ExternalMobilityModel + OnOffApplication broadcasting CAMs at 10 Hz)
 *
 * Both positions are updated each tick by the external bridge over TCP.
 *
 * Data format CARLA -> ns-3 (one row per tick, 12 floats):
 *   ego_x ego_y ego_z ego_vx ego_vy ego_vz ped_x ped_y ped_z ped_vx ped_vy ped_vz
 *
 * Response ns-3 -> CARLA (per ego node, comma-separated):
 *   last_recv_time, last_received_ped_x, last_received_ped_y, last_received_ped_z,
 *   last_received_ped_vx, last_received_ped_vy, last_received_ped_vz, msg_age_s
 *
 *   - last_recv_time: ns-3 wallclock when ego last received any CAM (s, -1 if none)
 *   - last_received_ped_*: ped position+velocity sampled at the moment the most-recently-received
 *                          CAM was transmitted by the ped (used by AEB to estimate ped state)
 *   - msg_age_s: now - tx_time of the most-recent successfully-received CAM
 */

#include <string>
#include <vector>
#include <map>
#include <iomanip>
#include <sstream>

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

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("V2PGateway");

class V2PGateway : public Gateway
{
public:
    V2PGateway(Ptr<Node> egoNode, Ptr<Node> pedNode);

    // Hook on ego's PacketSink RX trace
    void HandleEgoRx(std::string id, Ptr<const Packet> packet,
                     const Address &from, const Address &to,
                     const SeqTsSizeHeader &header);

    // Hook on ped's OnOff TX trace
    void HandlePedTx(Ptr<const Packet> packet);

private:
    virtual void DoInitialize(const std::vector<std::string> &data);
    virtual void DoUpdate(const std::vector<std::string> &data);

    Ptr<Node> m_egoNode;
    Ptr<Node> m_pedNode;

    // RX state (last successfully-received CAM)
    Time m_lastRxTime;        // ns-3 time when the last CAM was received by ego
    Time m_lastTxTimeOfRx;    // tx_time of the last received CAM
    Vector m_lastRxPedPos;    // ped position at that tx_time
    Vector m_lastRxPedVel;    // ped velocity at that tx_time

    // TX history: keyed by ped's tx_time, value = (pos, vel) at that instant.
    // We trim entries older than 1 s (CAMs that old can't still be in flight).
    std::map<Time, std::pair<Vector, Vector>> m_pedTxLog;
};

V2PGateway::V2PGateway(Ptr<Node> egoNode, Ptr<Node> pedNode)
    : Gateway(2),  // 2 nodes total: ego (idx 0) and ped (idx 1) — both have positions in the data stream
      m_egoNode(egoNode),
      m_pedNode(pedNode),
      m_lastRxTime(Seconds(-1)),
      m_lastTxTimeOfRx(Seconds(-1)),
      m_lastRxPedPos(0, 0, 0),
      m_lastRxPedVel(0, 0, 0)
{
}

void V2PGateway::HandlePedTx(Ptr<const Packet> packet)
{
    Time now = Simulator::Now();
    Vector pos = m_pedNode->GetObject<MobilityModel>()->GetPosition();
    Vector vel = m_pedNode->GetObject<MobilityModel>()->GetVelocity();
    m_pedTxLog[now] = std::make_pair(pos, vel);

    // Trim history older than 1 s — CAMs that old wouldn't still be in flight
    Time cutoff = now - Seconds(1.0);
    for (auto it = m_pedTxLog.begin(); it != m_pedTxLog.end(); )
    {
        if (it->first < cutoff)
            it = m_pedTxLog.erase(it);
        else
            ++it;
    }

    NS_LOG_INFO("[ped TX] t=" << now.GetSeconds()
                << "s pos=(" << pos.x << "," << pos.y << "," << pos.z
                << ") vel=(" << vel.x << "," << vel.y << "," << vel.z << ")");
}

void V2PGateway::HandleEgoRx(std::string id, Ptr<const Packet> packet,
                             const Address &from, const Address &to,
                             const SeqTsSizeHeader &header)
{
    Time txTime = header.GetTs();
    Time now = Simulator::Now();
    m_lastRxTime = now;
    m_lastTxTimeOfRx = txTime;

    auto it = m_pedTxLog.find(txTime);
    if (it != m_pedTxLog.end())
    {
        m_lastRxPedPos = it->second.first;
        m_lastRxPedVel = it->second.second;
    }
    else
    {
        // Fallback: if we somehow missed the TX log, use ped's current position.
        // This can happen if the TX log was trimmed before the packet was delivered.
        m_lastRxPedPos = m_pedNode->GetObject<MobilityModel>()->GetPosition();
        m_lastRxPedVel = m_pedNode->GetObject<MobilityModel>()->GetVelocity();
    }

    NS_LOG_INFO("[ego RX] t=" << now.GetSeconds()
                << "s tx_time=" << txTime.GetSeconds()
                << " latency=" << (now - txTime).GetMilliSeconds() << "ms"
                << " ped@TX=(" << m_lastRxPedPos.x << "," << m_lastRxPedPos.y << ")");
}

void V2PGateway::DoInitialize(const std::vector<std::string> &data)
{
    DoUpdate(data);
}

void V2PGateway::DoUpdate(const std::vector<std::string> &data)
{
    static const uint32_t ELEMENTS_PER_NODE = 6; // pos(3) + vel(3)
    static const uint32_t TOTAL_ELEMENTS    = 2 * ELEMENTS_PER_NODE;

    if (data.size() < TOTAL_ELEMENTS)
    {
        NS_FATAL_ERROR("V2PGateway: expected " << TOTAL_ELEMENTS
                       << " fields per tick (ego + ped), got " << data.size());
    }

    // --- update ego mobility (idx 0..5) ---
    Vector egoPos(std::stof(data[0]), std::stof(data[1]), std::stof(data[2]));
    Vector egoVel(std::stof(data[3]), std::stof(data[4]), std::stof(data[5]));
    m_egoNode->GetObject<ExternalMobilityModel>()->SetPosition(egoPos);
    m_egoNode->GetObject<ExternalMobilityModel>()->SetVelocity(egoVel);

    // --- update ped mobility (idx 6..11) ---
    Vector pedPos(std::stof(data[6]), std::stof(data[7]), std::stof(data[8]));
    Vector pedVel(std::stof(data[9]), std::stof(data[10]), std::stof(data[11]));
    m_pedNode->GetObject<ExternalMobilityModel>()->SetPosition(pedPos);
    m_pedNode->GetObject<ExternalMobilityModel>()->SetVelocity(pedVel);

    // --- build response for ego ---
    Time now = Simulator::Now();
    double age_s = (m_lastRxTime > Seconds(0)) ? (now - m_lastTxTimeOfRx).GetSeconds() : -1.0;
    double last_rx_s = m_lastRxTime.GetSeconds();

    std::ostringstream resp;
    resp << std::fixed << std::setprecision(4)
         << last_rx_s << ","
         << m_lastRxPedPos.x << ","
         << m_lastRxPedPos.y << ","
         << m_lastRxPedPos.z << ","
         << m_lastRxPedVel.x << ","
         << m_lastRxPedVel.y << ","
         << m_lastRxPedVel.z << ","
         << age_s;
    SetValue(0, resp.str());           // ego = node 0
    SetValue(1, "0,0,0,0,0,0,0,-1");   // ped = node 1, placeholder (bridge ignores)

    SendResponse();

    NS_LOG_INFO(">>> tick t=" << now.GetSeconds()
                << " ego=(" << egoPos.x << "," << egoPos.y << ")"
                << " ped=(" << pedPos.x << "," << pedPos.y << ")"
                << " last_rx=" << last_rx_s << " age=" << age_s);
}

int main(int argc, char *argv[])
{
    bool verboseLogs          = false;
    uint16_t serverPort       = 8100;
    std::string serverAddress = "127.0.0.1";
    double simDuration_s      = 60.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("verbose",       "Enable detailed log output", verboseLogs);
    cmd.AddValue("serverPort",    "Bridge server TCP port",     serverPort);
    cmd.AddValue("serverAddress", "Bridge server address",      serverAddress);
    cmd.AddValue("duration",      "Simulation duration (s)",    simDuration_s);
    cmd.Parse(argc, argv);

    Time::SetResolution(Time::NS);

    if (verboseLogs)
    {
        LogComponentEnable("Gateway",     LOG_LEVEL_INFO);
        LogComponentEnable("V2PGateway",  LOG_LEVEL_ALL);
        LogComponentEnable("PacketSink",  LOG_LEVEL_INFO);
    }
    else
    {
        LogComponentEnable("Gateway",    LOG_LEVEL_INFO);
        LogComponentEnable("V2PGateway", LOG_LEVEL_INFO);
    }

    // ---- Two nodes: ego (0) and ped (1) ----
    NodeContainer nodes;
    nodes.Create(2);
    Ptr<Node> egoNode = nodes.Get(0);
    Ptr<Node> pedNode = nodes.Get(1);

    // ---- Initial positions (will be overwritten on first tick) ----
    Ptr<ListPositionAllocator> alloc = CreateObject<ListPositionAllocator>();
    alloc->Add(Vector(0.0, 0.0, 0.5));   // ego
    alloc->Add(Vector(10.0, 0.0, 1.0));  // ped

    // ---- Both nodes use ExternalMobilityModel (positions fed externally) ----
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ExternalMobilityModel");
    mobility.SetPositionAllocator(alloc);
    mobility.Install(nodes);

    // ---- Wi-Fi PHY/MAC (matches V2I example: 802.11b adhoc, 1 Mbps, 20 dBm) ----
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211b);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode",    StringValue("DsssRate1Mbps"),
                                 "ControlMode", StringValue("DsssRate1Mbps"));

    YansWifiPhyHelper wifiPhy;
    wifiPhy.Set("TxPowerStart",    DoubleValue(20.0));
    wifiPhy.Set("TxPowerEnd",      DoubleValue(20.0));
    wifiPhy.Set("RxSensitivity",   DoubleValue(-95.0));
    wifiPhy.Set("CcaEdThreshold",  DoubleValue(-95.0));

    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    wifiPhy.SetChannel(wifiChannel.Create());

    WifiMacHelper wifiMac;
    wifiMac.SetType("ns3::AdhocWifiMac");
    NetDeviceContainer devices = wifi.Install(wifiPhy, wifiMac, nodes);

    // ---- IP stack ----
    InternetStackHelper stack;
    stack.Install(nodes);

    Ipv4AddressHelper address;
    address.SetBase("192.168.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = address.Assign(devices);

    const Ipv4Address broadcastAddress("192.168.1.255");
    const uint16_t applicationPort = 8100;

    // ---- Construct the gateway (manages ego only, but knows about ped) ----
    V2PGateway gateway(egoNode, pedNode);

    // ---- Ego: PacketSink (receives CAMs from ped) ----
    PacketSinkHelper sinkHelper("ns3::UdpSocketFactory",
                                InetSocketAddress(Ipv4Address::GetAny(), applicationPort));
    sinkHelper.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
    ApplicationContainer egoServerApps = sinkHelper.Install(egoNode);
    egoServerApps.Get(0)->TraceConnect("RxWithSeqTsSize", "0",
        MakeCallback(&V2PGateway::HandleEgoRx, &gateway));
    egoServerApps.Start(Time(0));

    // ---- Ped: OnOffApplication broadcasting CAMs at 10 Hz ----
    // 100 byte payload * 10 packets/s = 8 kbps -> 1.0 s on, 0.0 s off (continuous)
    OnOffHelper onoff("ns3::UdpSocketFactory",
                      InetSocketAddress(broadcastAddress, applicationPort));
    onoff.SetAttribute("OnTime",        StringValue("ns3::ConstantRandomVariable[Constant=1.0]"));
    onoff.SetAttribute("OffTime",       StringValue("ns3::ConstantRandomVariable[Constant=0.0]"));
    onoff.SetAttribute("PacketSize",    UintegerValue(100));
    onoff.SetAttribute("DataRate",      StringValue("8kbps"));
    onoff.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
    onoff.SetAttribute("StartTime",     TimeValue(Seconds(0.5)));

    ApplicationContainer pedTxApps = onoff.Install(pedNode);
    // Hook TX trace so we can record ped pos/vel at the instant of each broadcast
    pedTxApps.Get(0)->TraceConnectWithoutContext("Tx",
        MakeCallback(&V2PGateway::HandlePedTx, &gateway));

    // ---- Connect the gateway to the bridge server ----
    gateway.Connect(serverAddress, serverPort);

    // ---- Optional: PCAP/ASCII traces for debugging ----
    AsciiTraceHelper ascii;
    wifiPhy.EnableAsciiAll(ascii.CreateFileStream("v2p-trace.tr"));

    Simulator::Stop(Seconds(simDuration_s));
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}
