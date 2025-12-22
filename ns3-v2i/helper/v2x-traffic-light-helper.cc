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

#include "v2x-traffic-light-helper.h"

NS_LOG_COMPONENT_DEFINE("V2XTrafficLightHelper");

namespace ns3
{

  V2XTrafficLightHelper::V2XTrafficLightHelper(Address broadcastAddress):
    broadcastAddress(broadcastAddress),
    onoff("ns3::UdpSocketFactory", broadcastAddress)
  {
        NS_LOG_DEBUG("Creating traffic light helper");

        // OLD OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(groupAddress4, port));
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
  }

  void V2XTrafficLightHelper::Install(Ptr<Node> trafficLight)
  {
    NS_LOG_DEBUG("Installing traffic light application");
    onoff.Install(trafficLight);
  }

} // namespace ns3