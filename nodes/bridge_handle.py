#!/usr/bin/env python
import random
import rospy
import paho.mqtt.client as mqtt

from madrob_msgs.msg import *

def callback_raw(client, data):
    payload = "handle_raw adu0={},adu1={},adu2={},adu3={},status0={},status1={},status2={},status3={}".format(data.loadcells[0].adu, data.loadcells[1].adu, data.loadcells[2].adu, data.loadcells[3].adu, data.loadcells[0].status, data.loadcells[1].status, data.loadcells[2].status, data.loadcells[3].status)
    client.publish("tse/dev/test/666/handle_raw", payload)
    
def callback_force(client, data):
    payload = "handle_force force0={},force1={},force2={},force3={},status={}".format(data.forces[0], data.forces[1], data.forces[2], data.forces[3], data.status)
    client.publish("tse/dev/test/666/handle_force", payload)
   
def main(): 
    rospy.init_node('bridge_handle')
    client = mqtt.Client("bridge_handle")
    client.connect("172.31.0.254", port=1883, keepalive=60, bind_address="")
    client.loop_start()
    raw_sub = rospy.Subscriber('handle_raw', HandleRAW, lambda data: callback_raw(client, data))
    force_sub = rospy.Subscriber('handle_force', Handle, lambda data: callback_force(client, data))

    rospy.spin()
    
if __name__ == "__main__":
    main()
