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

#include "v2x-vehicle-helper.h"

NS_LOG_COMPONENT_DEFINE("V2XVehicleHelper");

namespace ns3
{

  V2XVehicleHelper::V2XVehicleHelper(Address broadcastAddress,
    Address listeningAddress,
    CallbackBase mobilityCallback,
    CallbackBase sinkTraceCallback):
    broadcastAddress(broadcastAddress),
    listeningAddress(listeningAddress),
    mobilityCallback(mobilityCallback),
    sinkTraceCallback(sinkTraceCallback)
  {
        NS_LOG_DEBUG("Creating vehicle helper");

  }

  void V2XVehicleHelper::Install(Ptr<Node> vehicle, uint32_t id)
  {
      NS_LOG_DEBUG("Installing vehicle application");
      // ExternalMobilityModel or JSONObject
      Ptr<MobilityModel> mobilityModel = vehicle->GetObject<MobilityModel>();
      mobilityModel->TraceConnectWithoutContext("CourseChange", mobilityCallback);

      //OLD: TriggeredSendHelper sendHelper("ns3::UdpSocketFactory", InetSocketAddress(groupAddress4, applicationPort));

      TriggeredSendHelper sendHelper("ns3::UdpSocketFactory", broadcastAddress);
      sendHelper.SetAttribute("PacketInterval", TimeValue(MilliSeconds(100)));
      ApplicationContainer clientApps = sendHelper.Install(vehicle);
      clientApps.Start(Time(0));

      // OLD: PacketSinkHelper sinkHelper("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), applicationPort));
      PacketSinkHelper sinkHelper("ns3::UdpSocketFactory", listeningAddress);
      sinkHelper.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
      ApplicationContainer serverApps = sinkHelper.Install(vehicle);

      serverApps.Get(0)->TraceConnect("RxWithSeqTsSize", std::to_string(id), sinkTraceCallback);
      serverApps.Start(Time(0));
  }

} // namespace ns3