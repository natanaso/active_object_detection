#!/usr/bin/env python

import roslib
roslib.load_manifest('pr2_plan_utils')
import rospy
import sys

from pr2_plan_utils import base

if __name__=='__main__':
    rospy.init_node('move_base_example')
    x, y, z = -2, -0.5, 1
    b = base.Base()
    b.move_to_look(x, y, z)
