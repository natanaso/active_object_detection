#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Jon Binney

import roslib; 
roslib.load_manifest('pr2_plan_utils')
import rospy

from geometry_msgs.msg import PoseStamped

from pr2_plan_utils.arm_mover import ArmMover

rospy.init_node('test_arm_control', anonymous=True)
arm_mover = ArmMover()

# Tell the left arm to move to a cartesian goal
goal_pose = PoseStamped()
goal_pose.pose.position.x = 0.65
goal_pose.pose.position.y = 0.2
goal_pose.pose.orientation.w = 1.0
goal_pose.header.frame_id = '/torso_lift_link'
handle1 = arm_mover.move_to_goal('left_arm', goal_pose, blocking=False)

# Tell the right arm to move to a joint space goal
joint_position = [-0.817, 0.001, -1.253, -0.892, 60.854, -0.250, 3.338]
js = arm_mover.get_joint_state('right_arm')
js.position = joint_position
handle2 = arm_mover.move_to_goal('right_arm', js, blocking=False)

# wait for moves to complete
handle1.wait()
handle2.wait()

# check whether movements were successful
if handle1.reached_goal():
    print 'Left arm reached the goal'
else:
    print handle1.get_errors()

if handle2.reached_goal():
    print 'Right arm reached the goal'
else:
    print handle2.get_errors()

