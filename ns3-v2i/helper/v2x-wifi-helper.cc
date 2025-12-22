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
 *  Raphael Barbau
*/

#include "v2x-wifi-helper.h"

NS_LOG_COMPONENT_DEFINE("V2XWiFiHelper");

namespace ns3
{

  V2XWiFiHelper::V2XWiFiHelper(): 
                              wifiChannel(YansWifiChannelHelper::Default()),
                              network("7.0.0.0"),
                              mask("255.0.0.0")
  {
     wifi.SetStandard(WIFI_STANDARD_80211b);
     wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                  "DataMode", StringValue("DsssRate1Mbps"),
                                  "ControlMode", StringValue("DsssRate1Mbps"));

     wifiPhy.Set("TxPowerStart", DoubleValue(20.0));
     wifiPhy.Set("TxPowerEnd", DoubleValue(20.0));
     wifiPhy.Set("RxSensitivity", DoubleValue(-95.0));
     wifiPhy.Set("CcaEdThreshold", DoubleValue(-95.0));

     wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");

    // Easier propagation (Exponent lowered)
     wifiChannel.AddPropagationLoss("ns3::LogDistancePropagationLossModel", "Exponent", DoubleValue(2.0));
    // Commented out fading
    // wifiChannel.AddPropagationLoss("ns3::NakagamiPropagationLossModel");

     wifiPhy.SetChannel(wifiChannel.Create());

     wifiMac.SetType("ns3::AdhocWifiMac");

    

    // allocate IPv4 Addresses 
    addressHelper.SetBase(network, mask);

  }

  NetDeviceContainer V2XWiFiHelper::Install(Ptr<Node> node)
  {
    NetDeviceContainer devices = wifi.Install(wifiPhy, wifiMac, node);
    // there *should* be only one
    NS_ASSERT_MSG(devices.GetN() == 1,
            "Node nos not have exactly one net device");
    Ptr<NetDevice> device = devices.Get(0);
    // install an IP network stack
    // This includes both the vehicle(s) and the traffic light node.
    // Required for sending/receiving IP-based packets 
    // Install Internet stack (ARP, IPv4, ICMP, UDP, TCP, routing protocols) on all nodes
    stack.Install(node);

    // Copied from Ipv4AddressHelper::Assign
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
    int32_t interface = ipv4->GetInterfaceForDevice(device);
    NS_ASSERT_MSG(interface >= 0,
                  "Ipv4AddressHelper::Assign(): "
                  "Interface index not found");
    addressHelper.Assign(devices);
    return devices;
  }

} // namespace ns3