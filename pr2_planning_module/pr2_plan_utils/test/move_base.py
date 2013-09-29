#!/usr/bin/env python

import roslib
roslib.load_manifest('pr2_plan_utils')
import rospy

from pr2_plan_utils import base

if __name__=='__main__':
    rospy.init_node('move_base_example')
    b = base.Base()
    b.move_to(1, 0.0, 0.0)
#    b.move_to(0, 1, 1.57)
