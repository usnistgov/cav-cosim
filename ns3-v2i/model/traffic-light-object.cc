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

#include "traffic-light-object.h"

namespace ns3
{
  
// === BEG GENERATED REGISTRATION ===
NS_LOG_COMPONENT_DEFINE("TrafficLightObject");

NS_OBJECT_ENSURE_REGISTERED(TrafficLightObject);

// === END GENERATED REGISTRATION ===

// === BEG GENERATED CONSTANTS ===
const std::string TrafficLightObject::TRAFFICLIGHTOBJECT_TYPE = "TrafficLightObject";
const std::string TrafficLightObject::LIGHT_STATUS_KEY = "light_status";
const std::string TrafficLightObject::TIME_REMAINING_KEY = "time_remaining";
// === END GENERATED CONSTANTS ===

// === BEG GENERATED CONSTRUCTORS ===
TrafficLightObject::TrafficLightObject():JSONMobilityObject(){}

TrafficLightObject::TrafficLightObject(const json& data):JSONMobilityObject(data){}

// === END GENERATED CONSTRUCTORS ===

// === BEG GENERATED GetJSONType ===
std::string TrafficLightObject::GetJSONType() const{
	return TrafficLightObject::TRAFFICLIGHTOBJECT_TYPE;
}
// === END GENERATED GetJSONType ===

// === BEG GENERATED GetTypeId ===
TypeId TrafficLightObject::GetTypeId()
{
	static TypeId tid = TypeId("ns3::TrafficLightObject").SetParent<JSONMobilityObject>().SetGroupName("V2X");
	return tid;
}
// === END GENERATED GetTypeId ===

// === BEG GENERATED GETTERS/SETTERS ===
uint TrafficLightObject::GetLight_status() const
{
	return m_light_status;
}

void TrafficLightObject::SetLight_status(uint _light_status)
{
	m_light_status = _light_status;
}
float TrafficLightObject::GetTime_remaining() const
{
	return m_time_remaining;
}

void TrafficLightObject::SetTime_remaining(float _time_remaining)
{
	m_time_remaining = _time_remaining;
}
// === END GENERATED GETTERS/SETTERS ===

// === BEG GENERATED (DE)SERIALIZATION METHODS ===
void TrafficLightObject::DoDeserialize(const json& obj)
{
	JSONMobilityObject::DoDeserialize(obj);
	for (json::const_iterator it = obj.begin(); it != obj.end(); ++it)
	{
		std::string key = it.key();
		json value = it.value();
		if (key == LIGHT_STATUS_KEY && IsUInt(key, value))
			m_light_status = value.template get<uint>();
		else if (key == TIME_REMAINING_KEY && JSONObject::IsNumber(key, value))
			m_time_remaining = value.template get<float>();
	}
}

void TrafficLightObject::DoSerialize(json& obj) const
{
	JSONMobilityObject::DoSerialize(obj);
	obj.emplace("light_status", m_light_status);
	obj.emplace("time_remaining", m_time_remaining);
}
// === END GENERATED (DE)SERIALIZATION METHODS ===

}