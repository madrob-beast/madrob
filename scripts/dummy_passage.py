#!/usr/bin/env python
import random
import rospy

from madrob_msgs.msg import *
from madrob_srvs.srv import *

def main(): 
    rospy.init_node('dummy_passage')

    ccw_left = rospy.Publisher('~ccw_left', Passage, queue_size=10)
    ccw_right = rospy.Publisher('~ccw_right', Passage, queue_size=10)
    cw_left = rospy.Publisher('~cw_left', Passage, queue_size=10)
    cw_right = rospy.Publisher('~cw_right', Passage, queue_size=10)

    rate = rospy.Rate(50) # 10hz

    while not rospy.is_shutdown():
        msg = Passage()
        msg.header.stamp = rospy.Time.now()

        msg.ranges[0].range = random.randint(0, 2500)
        msg.ranges[1].range = random.randint(0, 2500)
        msg.ranges[2].range = random.randint(0, 2500)
        msg.ranges[3].range = random.randint(0, 2500)
        ccw_left.publish(msg)

        msg = Passage()
        msg.header.stamp = rospy.Time.now()

        msg.ranges[0].range = random.randint(0, 2500)
        msg.ranges[1].range = random.randint(0, 2500)
        msg.ranges[2].range = random.randint(0, 2500)
        msg.ranges[3].range = random.randint(0, 2500)
        ccw_right.publish(msg)

        msg = Passage()
        msg.header.stamp = rospy.Time.now()

        msg.ranges[0].range = random.randint(0, 2500)
        msg.ranges[1].range = random.randint(0, 2500)
        msg.ranges[2].range = random.randint(0, 2500)
        msg.ranges[3].range = random.randint(0, 2500)
        cw_left.publish(msg)

        msg = Passage()
        msg.header.stamp = rospy.Time.now()
        
        msg.ranges[0].range = random.randint(0, 2500)
        msg.ranges[1].range = random.randint(0, 2500)
        msg.ranges[2].range = random.randint(0, 2500)
        msg.ranges[3].range = random.randint(0, 2500)
        cw_right.publish(msg)

        rate.sleep()

if __name__ == "__main__":
    main()
