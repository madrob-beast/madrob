#!/usr/bin/env python
import random
import rospy

from madrob_msgs.msg import *
from madrob_srvs.srv import *

handle_calibration = {
    'gains': (1, 2, 3, 4),
    'offsets': (0, 0, 0, 0)
}

def calibrate_handle(req):
    print "CalibrateHandleRequest(step: {}, force: {})".format(req.step, req.force)
    return CalibrateHandleResponse(True, "Dummy response")

def set_handle_calibration(req):
    print "SetHandleCalibrationRequest(gains: {}, offsets: {})".format(req.gains, req.offsets)
    handle_calibration['gains'] = req.gains
    handle_calibration['offsets'] = req.offsets
    return SetHandleCalibrationResponse(True, "Dummy response")

def get_handle_calibration(req):
    print "GetHandleCalibrationRequest()"
    return GetHandleCalibrationResponse(handle_calibration['gains'], handle_calibration['offsets'], True, "Dummy response")

def main(): 
    rospy.init_node('dummy_handle')

    rospy.Service('~calibrate_handle', CalibrateHandle, calibrate_handle)
    rospy.Service('~set_handle_calibration', SetHandleCalibration, set_handle_calibration)
    rospy.Service('~get_handle_calibration', GetHandleCalibration, get_handle_calibration)

    force_pub = rospy.Publisher('~force', Handle, queue_size=10)
    raw_pub = rospy.Publisher('~raw', HandleRAW, queue_size=10)

    rate = rospy.Rate(50) # 10hz

    while not rospy.is_shutdown():
        msg = Handle()
        msg.force = random.random() * 20
        msg.status = Handle.STATUS_OK
        force_pub.publish(msg)

        msg = HandleRAW()
        for i in range(len(msg.loadcells)):
            msg.loadcells[i].adu = random.randint(-2**31, 2**31-1)
            msg.loadcells[i].status = LoadCell.STATUS_OK
        raw_pub.publish(msg)

        rate.sleep()

if __name__ == "__main__":
    main()