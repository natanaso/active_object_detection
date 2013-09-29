#!/usr/bin/env python

import roslib
roslib.load_manifest('pr2_plan_utils')
import rospy
import sys

from pr2_python import base

if __name__=='__main__':
    rospy.init_node('move_traj_example')
    x = []
    y = []
    theta = []
    b = base.Base()
    p = b.get_current_pose()
    x.append(p[0])
    y.append(p[1])
    theta.append(p[2])
    x.append(0.5)
    y.append(0.13)
    theta.append(1.57)
    x.append(p[0])
    y.append(p[1])
    theta.append(p[2])
    b.follow_trajectory(x, y, theta)
