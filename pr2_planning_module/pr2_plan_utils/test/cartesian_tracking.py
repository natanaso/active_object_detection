
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

import roslib; roslib.load_manifest('pr2_plan_utils')
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
import tf

from pr2_plan_utils.controller_manager_client import ControllerManagerClient
from pr2_plan_utils.cartesian_controller_interface import CartesianControllerInterface

arm_name = 'right_arm'
goal_frame = '/cartesian_goal_frame'
cartesian_controller = '%s_cart' % arm_name[0]

# initialize
rospy.init_node('test_cartesian_tracking', anonymous=True)
controller_manager = ControllerManagerClient()
controller_manager.switch_controllers([cartesian_controller])
cci = CartesianControllerInterface(arm_name, 'cci')

# tell cartesian control interface to follow tracking frame
goal_pose = PoseStamped()
goal_pose.pose.position.z = 0.2
goal_pose.pose.orientation.w = 1.0
goal_pose.header.frame_id = goal_frame
goal_pose.header.stamp = rospy.Time(0)
cci.set_desired_pose(goal_pose, 0.02)

# loop, broadcasting the transform to the tracking frame
br = tf.TransformBroadcaster()
r = rospy.Rate(100)
while not rospy.is_shutdown():
    t = rospy.Time.now().to_sec()
    x = .65 + 0.1 * np.sin(t)
    br.sendTransform(
        (x, 0., 0.), (0., 0., 0., 1.),
        rospy.Time.now(),
        goal_frame,
        '/torso_lift_link')
    r.sleep()
        

