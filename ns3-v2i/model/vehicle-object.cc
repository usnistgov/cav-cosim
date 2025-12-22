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

#include "vehicle-object.h"

namespace ns3
{
  
// === BEG GENERATED REGISTRATION ===
NS_LOG_COMPONENT_DEFINE("VehicleObject");
NS_OBJECT_ENSURE_REGISTERED(VehicleObject);
// === END GENERATED REGISTRATION ===


// === BEG GENERATED CONSTANTS ===
const std::string VehicleObject::VEHICLEOBJECT_TYPE = "VehicleObject";
// === END GENERATED CONSTANTS ===


// === BEG GENERATED CONSTRUCTORS ===
VehicleObject::VehicleObject():JSONMobilityObject(){}

VehicleObject::VehicleObject(const json& data):JSONMobilityObject(data){}
// === END GENERATED CONSTRUCTORS ===


    //m_lastRecvTime(Seconds(0))

// === BEG GENERATED GetJSONType ===
std::string VehicleObject::GetJSONType() const{
	return VehicleObject::VEHICLEOBJECT_TYPE;
}
// === END GENERATED GetJSONType ===
// === BEG GENERATED GetTypeId ===
TypeId VehicleObject::GetTypeId()
{
	static TypeId tid = TypeId("ns3::VehicleObject").SetParent<JSONMobilityObject>().SetGroupName("V2X");
	return tid;
}
// === END GENERATED GetTypeId ===
    

// === BEG GENERATED GETTERS/SETTERS ===
// === END GENERATED GETTERS/SETTERS ===

// === BEG GENERATED (DE)SERIALIZATION METHODS ===
void VehicleObject::DoDeserialize(const json& obj)
{
	JSONMobilityObject::DoDeserialize(obj);
	for (json::const_iterator it = obj.begin(); it != obj.end(); ++it)
	{
		std::string key = it.key();
		json value = it.value();
	}
}
void VehicleObject::DoSerialize(json& obj) const
{
	JSONMobilityObject::DoSerialize(obj);
}
// === END GENERATED (DE)SERIALIZATION METHODS ===

void VehicleObject::SetLastReceiveTime(Time lastRecvTime)
{
    m_lastRecvTime = lastRecvTime;
}

}