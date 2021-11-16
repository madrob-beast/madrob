#!/usr/bin/env python

from enum import Enum
import numpy as np
import rospy
import threading
import time
import sys

from madrob_msgs.msg import CoreHandle
from madrob_srvs.srv import Parameters

SAMPLES = 500

class State(Enum):
    IDLE = 0
    ZERO = 1
    POINT_0 = 2
    POINT_1 = 3
    POINT_2 = 4
    POINT_3 = 5

class HandleCalibration(object):
    def __init__(self, dry=False):
        self.state = State.IDLE
        self.samples = 0
        self.weights = np.array([0, 0, 0, 0])
        self.offsets = np.array([0, 0, 0])
        self.values = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]])
        self.slopes = np.array([0, 0, 0])

        self.dry = dry

        rospy.init_node("handle_calibration")

        self.handle_sub = rospy.Subscriber("/madrob/handle/core", CoreHandle, self.handle_cb)

        rospy.wait_for_service('/madrob/handle/parameters')
        self.handle_params = rospy.ServiceProxy('/madrob/handle/parameters', Parameters)


    def handle_cb(self, msg):
        if self.state == State.IDLE:
            return

        if self.state == State.ZERO:
            if self.samples < SAMPLES:
                self.offsets[0] = self.offsets[0] + msg.loadcells[0].adu
                self.offsets[1] = self.offsets[1] + msg.loadcells[1].adu
                self.offsets[2] = self.offsets[2] + msg.loadcells[2].adu
                self.samples = self.samples + 1
                return

            self.offsets[0] = self.offsets[0] / SAMPLES
            self.offsets[1] = self.offsets[1] / SAMPLES
            self.offsets[2] = self.offsets[2] / SAMPLES
            self.samples = 0
            self.state = State.IDLE
            print("ZERO: {} {} {}".format(self.offsets[0], self.offsets[1] , self.offsets[2]))


        if self.state  == State.POINT_0:
            i = 0
            while self.samples < SAMPLES:
                self.values[i,0] = self.values[i,0] + msg.loadcells[0].adu
                self.values[i,1] = self.values[i,1] + msg.loadcells[1].adu
                self.values[i,2] = self.values[i,2] + msg.loadcells[2].adu
                self.samples = self.samples + 1
                return

            self.values[i,0] = (self.values[i,0] / SAMPLES) - self.offsets[0]
            self.values[i,1] = (self.values[i,1] / SAMPLES) - self.offsets[1]
            self.values[i,2] = (self.values[i,2] / SAMPLES) - self.offsets[2]
            self.samples = 0
            self.state = State.IDLE
            print("POINT {}: {} {} {}".format(i, self.values[i,0], self.values[i,1], self.values[i,2]))

        if self.state  == State.POINT_1:
            i = 1
            while self.samples < SAMPLES:
                self.values[i,0] = self.values[i,0] + msg.loadcells[0].adu
                self.values[i,1] = self.values[i,1] + msg.loadcells[1].adu
                self.values[i,2] = self.values[i,2] + msg.loadcells[2].adu
                self.samples = self.samples + 1
                return

            self.values[i,0] = (self.values[i,0] / SAMPLES) - self.offsets[0]
            self.values[i,1] = (self.values[i,1] / SAMPLES) - self.offsets[1]
            self.values[i,2] = (self.values[i,2] / SAMPLES) - self.offsets[2]
            self.samples = 0
            self.state = State.IDLE
            print("POINT {}: {} {} {}".format(i, self.values[i,0], self.values[i,1], self.values[i,2]))

        if self.state  == State.POINT_2:
            i = 2
            while self.samples < SAMPLES:
                self.values[i,0] = self.values[i,0] + msg.loadcells[0].adu
                self.values[i,1] = self.values[i,1] + msg.loadcells[1].adu
                self.values[i,2] = self.values[i,2] + msg.loadcells[2].adu
                self.samples = self.samples + 1
                return

            self.values[i,0] = (self.values[i,0] / SAMPLES) - self.offsets[0]
            self.values[i,1] = (self.values[i,1] / SAMPLES) - self.offsets[1]
            self.values[i,2] = (self.values[i,2] / SAMPLES) - self.offsets[2]
            self.samples = 0
            self.state = State.IDLE
            print("POINT {}: {} {} {}".format(i, self.values[i,0], self.values[i,1], self.values[i,2]))

        if self.state  == State.POINT_3:
            i = 3
            while self.samples < SAMPLES:
                self.values[i,0] = self.values[i,0] + msg.loadcells[0].adu
                self.values[i,1] = self.values[i,1] + msg.loadcells[1].adu
                self.values[i,2] = self.values[i,2] + msg.loadcells[2].adu
                self.samples = self.samples + 1
                return

            self.values[i,0] = (self.values[i,0] / SAMPLES) - self.offsets[0]
            self.values[i,1] = (self.values[i,1] / SAMPLES) - self.offsets[1]
            self.values[i,2] = (self.values[i,2] / SAMPLES) - self.offsets[2]
            self.samples = 0
            self.state = State.IDLE
            print("POINT {}: {} {} {}".format(i, self.values[i,0], self.values[i,1], self.values[i,2]))

    def worker(self):
        rospy.spin()


    def calibrate(self):
        self.slopes = np.linalg.lstsq(self.values, self.weights, rcond=None)[0]
        print("Offset: {}".format(self.offsets))
        print("Slope: {}".format(self.slopes))


    def commit(self):
        if self.dry:
            print("Load cell 0: {} {}".format(self.slopes[0], self.offsets[0]))
            print("Load cell 1: {} {}".format(self.slopes[1], self.offsets[1]))
            print("Load cell 2: {} {}".format(self.slopes[2], self.offsets[2]))
        else:
            self.handle_params(command=2, index=0, parameter=0, value=self.slopes[0])
            self.handle_params(command=2, index=0, parameter=1, value=self.offsets[0])
            self.handle_params(command=2, index=1, parameter=0, value=self.slopes[1])
            self.handle_params(command=2, index=1, parameter=1, value=self.offsets[1])
            self.handle_params(command=2, index=2, parameter=0, value=self.slopes[2])
            self.handle_params(command=2, index=2, parameter=1, value=self.offsets[2])
            self.handle_params(command=3, index=0, parameter=0, value=0.0)


def main():
    handle = HandleCalibration(dry=False)

    threading.Thread(target=handle.worker).start()

    raw_input("Remove any weights and press Enter to acquire ZERO")
    handle.state = State.ZERO

    while (handle.state == State.ZERO):
        print("Acquiring ZERO, {} samples left".format(SAMPLES - handle.samples))
        time.sleep(1)

    handle.weights[0] = raw_input("Apply a weight to point 0 and enter the weight to acquire POINT_0: ")
    handle.state = State.POINT_0

    while (handle.state == State.POINT_0):
        print("Acquiring POINT_0, {} samples left".format(SAMPLES - handle.samples))
        time.sleep(1)

    handle.weights[1] = raw_input("Apply a weight to point 1 and enter the weight to acquire POINT_1: ")
    handle.state = State.POINT_1

    while (handle.state == State.POINT_1):
        print("Acquiring POINT_1, {} samples left".format(SAMPLES - handle.samples))
        time.sleep(1)

    handle.weights[2] = raw_input("Apply a weight to point 2 and enter the weight to acquire POINT_2: ")
    handle.state = State.POINT_2

    while (handle.state == State.POINT_2):
        print("Acquiring POINT_2, {} samples left".format(SAMPLES - handle.samples))
        time.sleep(1)

    handle.weights[3] = raw_input("Apply a weight to point 3 and enter the weight to acquire POINT_3: ")
    handle.state = State.POINT_3

    while (handle.state == State.POINT_3):
        print("Acquiring POINT_3, {} samples left".format(SAMPLES - handle.samples))
        time.sleep(1)

    handle.calibrate()

    handle.commit()
    
    rospy.signal_shutdown("stop")
    sys.exit(0)
   

if __name__ == "__main__":
    main()
