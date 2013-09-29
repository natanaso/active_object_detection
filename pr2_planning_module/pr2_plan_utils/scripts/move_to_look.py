#!/usr/bin/env python

import roslib
roslib.load_manifest('pr2_python')
import rospy
import sys

from pr2_python import base
from pr2_python import head

if __name__=='__main__':
    rospy.init_node('move_base_example')
    [x, y, z] = [float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])]
    b = base.Base()
    b.move_to_look(x, y, z)
    h = head.Head()
    h.look_at_map_point(x, y, z)
