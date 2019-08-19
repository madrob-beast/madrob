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
parser.add_argument('-l', '--set_lut', help="Set door controller LUT from YAML file")

args = parser.parse_args()

if args.set_lut:
    with open(args.set_lut, 'r') as stream:
        try:
            data = yaml.safe_load(stream)
            set_lut_client(data['type'], data['values'])
        except yaml.YAMLError as e:
            print(e)
