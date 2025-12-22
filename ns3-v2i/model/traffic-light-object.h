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

#ifndef TRAFFIC_LIGHT_OBJECT_H
#define TRAFFIC_LIGHT_OBJECT_H

#include <ns3/json-mobility-object.h>

namespace ns3
{
// === BEG GENERATED CLASS DECLARATION ===
class TrafficLightObject : public JSONMobilityObject
// === END GENERATED CLASS DECLARATION ===
{
  public:    

// === BEG GENERATED CONSTANTS ===
	static const std::string TRAFFICLIGHTOBJECT_TYPE;
	static const std::string LIGHT_STATUS_KEY;
	static const std::string TIME_REMAINING_KEY;
// === END GENERATED CONSTANTS ===

// === BEG GENERATED CONSTRUCTORS ===
	TrafficLightObject();
	TrafficLightObject(const json& data);
// === END GENERATED CONSTRUCTORS ===

// === BEG GENERATED JSONType ===
	virtual std::string GetJSONType() const;
// === END GENERATED JSONType ===

// === BEG GENERATED GetTypeId ===
	static TypeId GetTypeId();
// === END GENERATED GetTypeId ===

// === BEG GENERATED GETTERS/SETTERS ===
	uint GetLight_status() const;
	void SetLight_status(uint _light_status);
	float GetTime_remaining() const;
	void SetTime_remaining(float _time_remaining);
// === END GENERATED GETTERS/SETTERS ===

  protected:

// === BEG GENERATED (DE)SERIALIZATION METHODS ===
	virtual void DoDeserialize(const json& obj);
	virtual void DoSerialize(json& obj) const;
// === END GENERATED (DE)SERIALIZATION METHODS ===

  private:
// === BEG GENERATED MEMBERS ===
	uint m_light_status;
	float m_time_remaining;
// === END GENERATED MEMBERS ===

};
}
#endif /* TRAFFIC_LIGHT_OBJECT_H */