#!/usr/bin/env python
import random
import rospy

from madrob_msgs.msg import *
from madrob_srvs.srv import *

handle_calibration = {
    'gain': 1.0,
    'offset': 0.0
}

def calibrate_handle(req):
    print "CalibrateHandleRequest(samples: {}, step: {}, force: {})".format(req.samples, req.step, req.force)
    return CalibrateHandleResponse(True, "Dummy response")

def set_handle_calibration(req):
    print "SetHandleCalibrationRequest(gain: {}, offset: {})".format(req.gain, req.offset)
    handle_calibration['gain'] = req.gain
    handle_calibration['offset'] = req.offset
    return SetHandleCalibrationResponse(True, "Dummy response")

def main(): 
    rospy.init_node('handle')

    rospy.Service('~calibrate', CalibrateHandle, calibrate_handle)
    rospy.Service('~set_calibration', SetHandleCalibration, set_handle_calibration)

    force_pub = rospy.Publisher('~state', Handle, queue_size=10)

    rate = rospy.Rate(50) # 10hz

    while not rospy.is_shutdown():
        msg = Handle()
        msg.header.stamp = rospy.Time.now()

        msg.force = random.random() * 20
        msg.kappa = handle_calibration['gain']
        msg.offset = handle_calibration['offset']
        msg.calibration_status = 3
        
        force_pub.publish(msg)

        rate.sleep()

if __name__ == "__main__":
    main()
