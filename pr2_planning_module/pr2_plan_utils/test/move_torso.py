#!/usr/bin/env python

import roslib; 
roslib.load_manifest('pr2_plan_utils')
import rospy

from pr2_plan_utils import torso

if __name__=='__main__':
    rospy.init_node("test_torso", anonymous=True)
    tt = torso.Torso()

    # Move to a certain height
    tt.move(0.15)
    rospy.sleep(2.0)

    # Move all the way up
    tt.up()
    rospy.sleep(2.0)
    
    # Move all the way down
    tt.down()
    rospy.sleep(2.0)
