"""/*
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
*/"""

import threading

class JSONObject:
    # Keep in sync with ns-3
    JSONOBJECT_ID = "id"
    JSONOBJECT_TYPE = "type"
    JSONOBJECT_ALIVE = "alive"
    
    lock = threading.Lock()
    nextid = 0

    def __init__(self):
        with JSONObject.lock:
            self.id = JSONObject.nextid
            JSONObject.nextid += 1
        self.alive = True
    
    def update(self, time_period):
        pass

    def serialize(self, obj):
        obj[JSONObject.JSONOBJECT_ID] = self.id
        # NO TYPE FOR ABSTRACT OBJECT
        obj[JSONObject.JSONOBJECT_ALIVE] = self.alive

    def deserialize(self, obj):
        if JSONObject.JSONOBJECT_ALIVE in obj: self.alive = obj[JSONObject.JSONOBJECT_ALIVE]

class JSONMobilityObject(JSONObject):
    JSONOBJECT_POS_X = "pos_x"
    JSONOBJECT_POS_Y = "pos_y"
    JSONOBJECT_POS_Z = "pos_z"
    JSONOBJECT_VEL_X = "vel_x"
    JSONOBJECT_VEL_Y = "vel_y"
    JSONOBJECT_VEL_Z = "vel_z"

    def __init__(self, pos_x=0, pos_y=0, pos_z=0, vel_x=0, vel_y=0, vel_z=0):
        super().__init__()
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.pos_z = pos_z
        self.vel_x = vel_x
        self.vel_y = vel_y
        self.vel_z = vel_z

    def update(self, time_period):
        super().update(time_period)
        self.pos_x += self.vel_x/time_period
        self.pos_y += self.vel_y/time_period
        self.pos_z += self.vel_z/time_period

    def serialize(self, obj):
        super().serialize(obj)
        if self.alive:
            obj[JSONMobilityObject.JSONOBJECT_POS_X] = self.pos_x
            obj[JSONMobilityObject.JSONOBJECT_POS_Y] = self.pos_y
            obj[JSONMobilityObject.JSONOBJECT_POS_Z] = self.pos_z
            obj[JSONMobilityObject.JSONOBJECT_VEL_X] = self.vel_x
            obj[JSONMobilityObject.JSONOBJECT_VEL_Y] = self.vel_y
            obj[JSONMobilityObject.JSONOBJECT_VEL_Z] = self.vel_z

    def deserialize(self, obj):
        super().deserialize(obj)
        if obj[JSONObject.JSONOBJECT_ID] == self.id and obj[JSONObject.JSONOBJECT_ALIVE] == self.alive:
            if JSONMobilityObject.JSONOBJECT_POS_X in obj: self.pos_x = obj[JSONMobilityObject.JSONOBJECT_POS_X]
            if JSONMobilityObject.JSONOBJECT_POS_Y in obj: self.pos_y = obj[JSONMobilityObject.JSONOBJECT_POS_Y]
            if JSONMobilityObject.JSONOBJECT_POS_Z in obj: self.pos_z = obj[JSONMobilityObject.JSONOBJECT_POS_Z]
            if JSONMobilityObject.JSONOBJECT_VEL_X in obj: self.vel_x = obj[JSONMobilityObject.JSONOBJECT_VEL_X]
            if JSONMobilityObject.JSONOBJECT_VEL_Y in obj: self.vel_y = obj[JSONMobilityObject.JSONOBJECT_VEL_Y]
            if JSONMobilityObject.JSONOBJECT_VEL_Z in obj: self.vel_z = obj[JSONMobilityObject.JSONOBJECT_VEL_Z]
