#!/usr/bin/env python
import argparse
import random
import yaml

import rospy

from madrob_msgs.msg import *
from madrob_srvs.srv import *

def set_lut_client(type, values):
    rospy.wait_for_service('/madrob/door/set_lut')
    try:
        set_lut = rospy.ServiceProxy('/madrob/door/set_lut', SetDoorControllerLUT)
        resp = set_lut(type, values)
        return resp.success
    except rospy.ServiceException as e:
        print "Service call failed: %s"%e


# parser
parser = argparse.ArgumentParser()
parser.add_argument('-f', '--file', help="Set door controller LUT from YAML file")
parser.add_argument('-t', '--type', type=int, help="Specify LUT type (0 = CW, 1 = CCW)")
parser.add_argument('-c', '--constant', type=int, help="Constant LUT value")

args = parser.parse_args()

if args.file:
    with open(args.file, 'r') as stream:
        try:
            data = yaml.safe_load(stream)
            if args.type:
                data['type'] = args.type
            set_lut_client(data['type'], data['values'])
        except yaml.YAMLError as e:
            print(e)

if args.constant:
    data = {}
    data['type'] = args.type
    data['values'] = [args.constant] * 181