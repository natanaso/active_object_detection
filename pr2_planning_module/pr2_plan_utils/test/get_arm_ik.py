#!/usr/bin/env python

import roslib; 
roslib.load_manifest('pr2_plan_utils')
import rospy

from pr2_plan_utils import arm_planner
from geometry_msgs.msg import PoseStamped

if __name__=='__main__':
    rospy.init_node("test_gripper", anonymous=True)
    arm = arm_planner.ArmPlanner('right_arm')

    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = 'torso_lift_link'
    pose_stamped.pose.position.x = 0.65
    pose_stamped.pose.orientation.w = 1.0

    ik_sol = arm.get_ik(pose_stamped)
    # ik_sol.solution is a JointState
    rospy.loginfo('IK Solution: '+str(ik_sol.solution))
    
