#!/usr/bin/env python
import random
import rospy

from madrob_msgs.msg import *
from madrob_srvs.srv import *

door_position_calibration = {
    'cw': -0.9,
    'zero': 0,
    'ccw': 0.9
}

door_controller_mode = SetDoorControllerModeRequest.DOOR_MODE_LUT

def calibrate_door_position(req):
    print "CalibrateDoorPositionRequest(position: {})".format(req.position)
    return CalibrateDoorPositionResponse(True, "Dummy response")

def set_door_position_calibration(req):
    print "SetDoorPositionCalibrationRequest(cw: {}, zero: {}, ccw: {})".format(req.cw, req.zero, req.ccw)
    door_position_calibration['cw'] = req.cw
    door_position_calibration['zero'] = req.zero
    door_position_calibration['ccw'] = req.ccw
    return SetDoorPositionCalibrationResponse(True, "Dummy response")

def get_door_position_calibration(req):
    print "GetDoorPositionCalibrationRequest()"
    return GetDoorPositionCalibrationResponse(door_position_calibration['cw'], door_position_calibration['zero'], door_position_calibration['ccw'], True, "Dummy response")

def set_door_controller_mode(req):
    global door_controller_mode
    print "SetDoorControllerModeRequest(mode: {})".format(req.mode)
    door_controller_mode = req.mode
    return SetDoorControllerModeResponse(True, "Dummy response")

def get_door_controller_mode(req):
    global door_controller_mode
    print "GetDoorControllerModeRequest()"
    return GetDoorControllerModeResponse(door_controller_mode, True, "Dummy response")

def set_door_controller_lut(req):
    print "SetDoorControllerLUT(type: {}, value[-90]: {}, value[0]: {}, value[90]: {})".format(req.type, req.values[0], req.values[90], req.values[180])
    return SetDoorControllerLUTResponse(True, "Dummy response")

def main(): 
    rospy.init_node('dummy_door')

    rospy.Service('~calibrate_door_position', CalibrateDoorPosition, calibrate_door_position)
    rospy.Service('~set_door_position_calibration', SetDoorPositionCalibration, set_door_position_calibration)
    rospy.Service('~get_door_position_calibration', GetDoorPositionCalibration, get_door_position_calibration)
    rospy.Service('~set_door_controller_mode', SetDoorControllerMode, set_door_controller_mode)
    rospy.Service('~get_door_controller_mode', GetDoorControllerMode, get_door_controller_mode)
    rospy.Service('~set_door_controller_lut', SetDoorControllerLUT, set_door_controller_lut)

    force_pub = rospy.Publisher('~door', Door, queue_size=10)

    rate = rospy.Rate(50) # 10hz

    while not rospy.is_shutdown():
        msg = Door()
        msg.angle = random.randint(-90, 90)
        msg.velocity = random.random()
        msg.duty_cycle = random.random()
        msg.supply_current = random.random()
        msg.phase_current = [random.random(), random.random(), random.random()]
        msg.brake_current = random.random()
        msg.brake_force = msg.brake_current

        msg.state = Door.STATE_IDLE
        msg.status = Door.STATUS_OK
        force_pub.publish(msg)

        rate.sleep()

if __name__ == "__main__":
    main()