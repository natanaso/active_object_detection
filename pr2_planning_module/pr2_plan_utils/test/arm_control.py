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

import roslib; roslib.load_manifest('pr2_python')
import numpy as np
import rospy
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
from tf import transformations

from pr2_python.arm_controller import ArmController
from pr2_python.exceptions import ArmNavError

arm_name = 'right_arm'

rospy.init_node('test_arm_control', anonymous=True)
ac = ArmController(arm_name)

while not rospy.is_shutdown():
    rot = transformations.random_quaternion()
    #trans = np.random.uniform(-0.5, 0.5, 3) + np.array((
    trans = np.array((0.65, 0.0, 0.0)) + np.random.uniform(-0.6, 0.6, 3)
    rot = np.array((0.0, 0.0, 0.0, 1.0))
    print 'Moving to %s, %s' % (str(rot), str(trans))
    goal_pose = PoseStamped()
    goal_pose.pose.position = Point(*trans)
    goal_pose.pose.orientation = Quaternion(*rot)
    goal_pose.header.frame_id = '/torso_lift_link'
    try:
        ac.move_to_goal(goal_pose, try_hard=True)
        print ac.get_exceptions()
    except ArmNavError as e:
        print 'Arm nav error: %s' % str(e)
    rospy.sleep(0.1)
