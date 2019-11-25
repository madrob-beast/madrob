#!/usr/bin/env python
import random
import rospy

from madrob_msgs.msg import *
from madrob_srvs.srv import *
from std_msgs.msg import *

door_position_calibration = {
    'cw': -0.9,
    'zero': 0,
    'ccw': 0.9
}

motor_kt = 1.0

door_controller_mode = SetDoorControllerModeRequest.MODE_LUT

def set_door_brake(req):
    print "SetDoorBrake(method: {}, mu: {}, setpoint: {})".format(req.method, req.mu, req.setpoint)
    return SetDoorBrakeResponse(True, "Dummy response")

def set_door_controller_lut(req):
    print "SetDoorControllerLUT(type: {}, value[-90]: {}, value[0]: {}, value[90]: {})".format(req.type, req.values[0], req.values[90], req.values[180])
    return SetDoorControllerLUTResponse(True, "Dummy response")

def set_door_controller_mode(req):
    global door_controller_mode
    print "SetDoorControllerModeRequest(mode: {})".format(req.mode)
    door_controller_mode = req.mode
    return SetDoorControllerModeResponse(True, "Dummy response")

def set_force_calibration(req):
    global motor_kt
    print "SetDoorControllerCalibration(kt: {})".format(req.kt)
    motor_kt = req.mode
    return SetDoorControllerCalibrationResponse(True, "Dummy response")

def calibrate_door_position(req):
    print "CalibrateDoorPositionRequest(position: {})".format(req.position)
    return CalibrateDoorPositionResponse(True, "Dummy response")

def set_door_position_calibration(req):
    print "SetDoorPositionCalibrationRequest(cw: {}, zero: {}, ccw: {})".format(req.cw, req.zero, req.ccw)
    door_position_calibration['cw'] = req.cw
    door_position_calibration['zero'] = req.zero
    door_position_calibration['ccw'] = req.ccw
    return SetDoorPositionCalibrationResponse(True, "Dummy response")

def brake_setpoint(data):
    print "Brake setpoint = {}".format(data.data)


def main(): 
    rospy.init_node('dummy_door')

    rospy.Service('~set_brake', SetDoorBrake, set_door_brake)
    rospy.Service('~set_lut', SetDoorControllerLUT, set_door_controller_lut)
    rospy.Service('~set_mode', SetDoorControllerMode, set_door_controller_mode)
    rospy.Service('~set_force_calibration', SetDoorControllerCalibration, set_force_calibration)
    rospy.Service('~calibrate_position', CalibrateDoorPosition, calibrate_door_position)
    rospy.Service('~set_position_calibration', SetDoorPositionCalibration, set_door_position_calibration)

    force_pub = rospy.Publisher('~state', Door, queue_size=10)
    setpoint_sub = rospy.Subscriber('~brake_setpoint', Float32, brake_setpoint)

    rate = rospy.Rate(50) # 10hz

    while not rospy.is_shutdown():
        msg = Door()
        msg.header.stamp = rospy.Time.now()

        msg.angle = random.randint(-90, 90)
        msg.velocity = random.random()
        msg.duty_cycle = random.random()
        msg.supply_current = random.random()
        msg.phase_current = random.random()

        msg.mode = door_controller_mode
        msg.state = Door.STATE_IDLE
        msg.status = Door.STATUS_OK


        msg.brake_method = 3
        msg.brake_mu = 1
        msg.brake_setpoint = random.randint(0, 2000)
        msg.brake_current = random.randint(0, 100)
        msg.brake_force = msg.brake_setpoint

        msg.position_calibration_ccw = door_position_calibration['ccw']
        msg.position_calibration_zero = door_position_calibration['zero']
        msg.position_calibration_cw = door_position_calibration['cw']

        msg.kt = motor_kt

        force_pub.publish(msg)

        rate.sleep()

if __name__ == "__main__":
    main()
