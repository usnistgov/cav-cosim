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
 * Modified by Hadhoum Hajjaj <hadhoum.hajjaj@nist.gov> to include V2I communication by adding a traffic light broadcaster
*/

#include <string>
#include <vector>

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"

#include "ns3/external-mobility-model.h"
#include "ns3/triggered-send-application.h"
#include "ns3/triggered-send-helper.h"

#include "ns3/gateway.h"

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

        // this function handles receiving broadcast messages from ns-3 (not the remote server)
        //  id indicates the vehicle index that received the message; the other arguments are ignored
    void HandleReceive(std::string id, Ptr<const Packet> packet, const Address &clientAddress);
private:
        // this function handles processing the first message received from the remote server
        //  the simple gateway doesn't require any initialization, so this just calls DoUpdate
        virtual void DoInitialize(const std::vector<std::string> & data);

        // this function handles processing messages received from the remote server
        virtual void DoUpdate(const std::vector<std::string> & data);

        NodeContainer m_vehicles;       // the nodes representing vehicles that are managed by the gateway
        std::vector<uint16_t> m_count;  // the number of times each vehicle has received a broadcast
};

SimpleGateway::SimpleGateway(NodeContainer vehicles):
    Gateway(vehicles.GetN()),
    m_vehicles(vehicles),
    m_count(vehicles.GetN(), 0)
{
    // do nothing
}

void SimpleGateway::HandleReceive(std::string id, Ptr<const Packet> packet, const Address &clientAddress)
{
    NS_LOG_INFO("At time " << Simulator::Now().As(Time::S) << ", Node " << id << " received a broadcast");
    m_count.at(std::stoi(id)) += 1;
}

void SimpleGateway::DoInitialize(const std::vector<std::string> &data)
{
    DoUpdate(data);
}

void SimpleGateway::DoUpdate(const std::vector<std::string> &data)
{
    NS_LOG_FUNCTION(this << data);

    static const uint32_t ELEMENTS_PER_VEHICLE = 7; // Position_{x,y,z} + Velocity_{x,y,z} + SendFlag

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

        // handle the send flag
        if (std::stoi(data[dataIndex+6]))
        {
            // the index '0' here is because the TriggeredSendApplication is the first application installed in main
            DynamicCast<TriggeredSendApplication>(vehicle->GetApplication(0))->Send(3); // broadcast 3 packets
            NS_LOG_INFO("At time " << Simulator::Now().As(Time::S) << ", Node " << i << " sent a broadcast");
        }

        NS_LOG_INFO("Sending response for node " << i << ": " << m_count[i]);
        SetValue(i, std::to_string(m_count[i])); // update the received broadcast count
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
        // Enable logging for PacketSink (receiver) and OnOffApplication (broadcaster) to debug V2I messages
        LogComponentEnable("PacketSink", LOG_LEVEL_INFO);
        LogComponentEnable("OnOffApplication", LOG_LEVEL_INFO);
    }
    else
    {
        LogComponentEnable("Gateway", LOG_LEVEL_INFO);
        LogComponentEnable("SimpleGateway", LOG_LEVEL_INFO);
    }
    
    // Create total nodes: numberOfVehicles + 1 traffic light node for V2I communication

    uint16_t totalNodes = numberOfVehicles + 1; // +1 for traffic light
    NodeContainer nodes;
    nodes.Create(totalNodes);
    NS_LOG_DEBUG("Creating " << numberOfVehicles << " vehicle node(s) and 1 traffic light node (total: " << totalNodes << ")");

    // generate a list of initial positions for the mobility models
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

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ExternalMobilityModel");
    mobility.SetPositionAllocator(positionAllocator);
    for (uint16_t i = 0; i < numberOfVehicles; i++)
    {
        mobility.Install(nodes.Get(i));
    }

    // === Traffic Light Mobility Setup ===
    // We previously created (numberOfVehicles + 1) nodes:
    //    - nodes[0] to nodes[numberOfVehicles - 1] are the vehicles
    //    - nodes[numberOfVehicles] is the traffic light
    //
    // The traffic light should remain stationary, so we apply the
    // "ns3::ConstantPositionMobilityModel", which keeps its position fixed.
    //
    // We use nodes.Get(numberOfVehicles) to access the traffic light node,
    // because it's the last node created (after all the vehicles).
    //
    // This ensures the traffic light does not move during the simulation.
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(nodes.Get(numberOfVehicles)); // traffic light node

    CsmaHelper csma;
    csma.SetChannelAttribute("DataRate", StringValue("100Mbps"));
   
    // Install a CSMA (Ethernet-like) channel on all nodes.
    // This includes both the vehicle(s) and the traffic light,
    // since both need to be connected to the same network for V2I communication.
    NetDeviceContainer devices = csma.Install(nodes);

    // install an IP network stack
    InternetStackHelper stack;
    // Install the IP (Internet) protocol stack on all nodes.
    // This includes both the vehicle(s) and the traffic light node.
    // Required for sending/receiving IP-based packets 
    stack.Install(nodes);

    // allocate IPv4 Addresses from 192.168.1.0/24 
    Ipv4AddressHelper address;
    address.SetBase("192.168.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = address.Assign(devices);

    const Ipv4Address broadcastAddress("192.168.1.255");
    const uint16_t applicationPort = 8100;

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
        // call ReportMobility when the external mobility model reports a CourseChange
        Ptr<ExternalMobilityModel> mobilityModel = vehicle->GetObject<ExternalMobilityModel>();
        mobilityModel->TraceConnectWithoutContext("CourseChange", MakeCallback(&ReportMobility));

        // install a triggered send application that can be triggered to broadcast messages to the bus
        TriggeredSendHelper sendHelper("ns3::UdpSocketFactory", InetSocketAddress(broadcastAddress, applicationPort));
        sendHelper.SetAttribute("PacketInterval", TimeValue(MilliSeconds(100)));
        ApplicationContainer clientApps = sendHelper.Install(vehicle);
        clientApps.Start(Time(0));

        // install a packet sink that calls HandleReceive when it receives a broadcasted message
        PacketSinkHelper sinkHelper("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), applicationPort));
        ApplicationContainer serverApps = sinkHelper.Install(vehicle);
        serverApps.Get(0)->TraceConnect("Rx", std::to_string(i), MakeCallback(&SimpleGateway::HandleReceive, &gateway));
        serverApps.Start(Time(0));
    }

    // === Traffic light broadcasts ===
    Ptr<Node> trafficLight = nodes.Get(numberOfVehicles);
    OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(broadcastAddress, applicationPort));
    onoff.SetAttribute("PacketSize", UintegerValue(100));
    onoff.SetAttribute("DataRate", StringValue("1kbps"));
    onoff.SetAttribute("StartTime", TimeValue(Seconds(1.0)));
    onoff.SetAttribute("StopTime", TimeValue(Seconds(30.0)));
    onoff.Install(trafficLight);

    // Gateway connects to intermidiate server
    gateway.Connect(serverAddress, serverPort);

    Simulator::Run();
    Simulator::Destroy();

    return 0;
}
