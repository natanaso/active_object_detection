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

import roslib; roslib.load_manifest('pr2_plan_utils')
import numpy as np
import rospy
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
from sensor_msgs.msg import JointState
from tf import transformations

from pr2_plan_utils.arm_controller import ArmController
from pr2_plan_utils.exceptions import ArmNavError

groups = {
    'right_arm': ['r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint',
                  'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint'],

    'left_arm': ['l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint',
                 'l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint']
    }

arm_name = 'right_arm'

joint_positions = [
    [-0.817, 0.001, -1.253, -0.892, 60.854, -0.250, 3.338],
    [0.002, 0.076, -1.146, -1.255, 61.306, -0.526, 3.299],
    [0.681, 0.720, 0.147, -0.540, 63.062, -0.013, 2.245]
    ]

rospy.init_node('test_arm_control', anonymous=True)
ac = ArmController(arm_name)

ii = 0
while True:
    js = JointState()
    js.name = groups[arm_name]
    js.position = joint_positions[ii]
    try:
        print 'Moving to %s' % (str(joint_positions[ii]))
        ac.move_to_joint_state(js,
                               joint_bounds=None, try_hard=True, planner_timeout=5., ordered_collisions=None, planner_id='')
    except ArmNavError as e:
        print 'Arm nav error: %s' % str(e)
    ii = (ii + 1) % len(joint_positions)
    rospy.sleep(0.1)
