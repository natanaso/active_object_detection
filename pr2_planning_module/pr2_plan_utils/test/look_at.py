#!/usr/bin/env python

import roslib; 
roslib.load_manifest('pr2_plan_utils')
import rospy

from pr2_plan_utils import head

if __name__=='__main__':
    rospy.init_node("test_look_at", anonymous=True)
    h = head.Head()

    # look at a point defined in the base footprint frame
    h.look_at_relative_point(1.0, 0.5, 0.0)
    
    rospy.sleep(2.0)

    # look at a point in the map frame
    h.look_at_map_point(1.0,0,0)
    
