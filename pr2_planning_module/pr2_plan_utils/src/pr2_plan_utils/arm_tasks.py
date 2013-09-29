#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
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
# Author: Jenny Barry

'''
Module used for high level tasks involving only the arms.
'''

__docformat__ = "restructuredtext en"


import roslib
roslib.load_manifest('pr2_plan_utils')

import rospy
import pr2_plan_utils.geometry_tools as gt
from pr2_plan_utils.exceptions import ArmNavError
from pr2_plan_utils.joint_state_listener import JointStateListener
from pr2_plan_utils.arm_mover import ArmMover
from pr2_plan_utils.arm_planner import ArmPlanner

import numpy as np

#the first two here move to the position used for manipulation
#the last set are down by the sides for moving around
#this trajectory can be changed using a ROS parameter.
#[-1.6, 1, -1.65, -1.96, 3.53, -1.90, np.pi]]}
#['_shoulder_pan_joint', '_shoulder_lift_joint', '_upper_arm_roll_joint', '_elbow_flex_joint', '_forearm_roll_joint', '_wrist_flex_joint', '_wrist_roll_joint']
DEFAULT_SIDE_JOINT_TRAJECTORY = {'left_arm': [[0.968, 0.129, 0.554, -1.891, 1.786, -1.127, 0.501], 
                                              [2.135, -0.02, 1.64, -2.07, 1.64, -1.68, 1.398],
                                              [2.1, 1.26, 1.8, -1.9, -3.5, -1.8, np.pi/2.0]],
                                 'right_arm': [[-0.968, 0.129, -0.554, -1.891, -1.786, -1.127, 0.501],
                                               [-2.135, -0.02, -1.64, -2.07, -1.64, -1.68, 1.398],
                                               [-2.1, 1.26, -1.8, -1.9, 3.5, -1.8, np.pi/2.0]]}
class ArmTasks:
    '''
    Performs high level tasks for the PR2 arms.  

    This class does high level tasks for the arms alone. 

    **Attributes:**
       
        **side_joint_trajectory ([[double]]):** Joint trajectory for moving to the side
    '''
    def __init__(self):
        self._joint_state_listener = JointStateListener()
        self._arm_planners = {}
        self.side_joint_trajectory = {}
        self._arm_mover = ArmMover()
        for arm in ['left_arm', 'right_arm']:
            self._arm_planners[arm] = ArmPlanner(arm)
            self.side_joint_trajectory[arm] = rospy.get_param('/arm_configurations/side_tuck/trajectory/'+arm,
                                                              DEFAULT_SIDE_JOINT_TRAJECTORY[arm])

    def move_arm_to_side(self, arm_name):
        '''
        A function that persistently tries to move an arm to the side.  

        Unless the arm is physically stuck in the world, the arm will be at the robot's side at the end of this 
        function.  However, as last resort, the robot uses an open-loop movement to a specific joint state so there 
        is no guarantee the arm's path will be collision-free.

        Specifically, this function tries in order to:
        
            1. Move the arm to the side joint pose using collision-free planning (calls move arm)
            2. If the starting state is in collision, it tries twice to move the state out of collision by
               executing a short open-loop trajectory to a free state.
            3. If the starting state is outside joint angles, it tries twice to move the state into joint angles by
               executing a short open-loop trajectory to a state within joint angle limits.
            4. Moves open loop to each of the joint states along the side joint trajectory.  By defalut there are
               three of these points.  The first is high, near the center of the robot's body.  The second is
               high and to the robot's side.  The last is low and at the robots side.  After each of the first two
               moves, the robot tries to move to the final joint state collision free before executing the next
               trajectory open-loop.

        **Args:**
            **arm_name (string):** The name of the arm ('right_arm' or 'left_arm') to be moved to side.
        '''

        if self._at_side(arm_name):
            rospy.loginfo(arm_name + ' is already at the side position')
            return
        self._arm_mover.move_into_joint_limits(arm_name)
        joint_state = self._arm_planners[arm_name].joint_positions_to_joint_state(
            self.side_joint_trajectory[arm_name][-1])

        # first try to move directly there using move arm
        handle = self._arm_mover.move_to_goal_using_move_arm(arm_name, joint_state, 10.0)
        if handle.reached_goal():
            return
        for t in range(2):
            checked_side = False
            for e in handle.get_errors():
                rospy.loginfo('When moving arm, error was:' + str(e))
                if type(e) != ArmNavError or not e.error_code:
                    if not checked_side and self._at_side(arm_name):
                        return
                    checked_side = True
                elif e.error_code.val == e.error_code.START_STATE_IN_COLLISION:
                    self._arm_mover.move_out_of_collision(arm_name)
                    handle = self._arm_mover.move_to_goal_using_move_arm(arm_name, joint_state, 10.0)
                    if handle.reached_goal():
                        return
                    rospy.loginfo('Even after moving out of collision, we are unable to move the arm to the side')
                    break
                elif e.error_code.val == e.error_code.JOINT_LIMITS_VIOLATED:
                    self._arm_mover.move_into_joint_limits(arm_name)
                    handle = self._arm_mover.move_to_goal_using_move_arm(arm_name, joint_state, 10.0)
                    if handle.reached_goal():
                        return
                    rospy.loginfo('Moved into joint limits but still unable to move the arm to the side')
                    break
       
        # now try to move to each point on our open loop trajectories
        # when there, see if we can move to side
        for p in range(len(self.side_joint_trajectory[arm_name])):
            joint_state.position = self.side_joint_trajectory[arm_name][p]
            self._arm_mover.move_to_goal(arm_name, joint_state, try_hard=True)
            if p < len(self.side_joint_trajectory[arm_name]) - 1:
                # try to go directly to side from here
                joint_state.position = self.side_joint_trajectory[arm_name][-1]
                handle = self._arm_mover.move_to_goal_using_move_arm(arm_name, joint_state, 10.0)
                if handle.reached_goal() or self._at_side(arm_name):
                    return

    def move_arms_to_side(self):
        '''
        Moves both arms to side by calling move_arm_to_side on each arm.
        '''
        self.move_arm_to_side('right_arm')
        self.move_arm_to_side('left_arm')


    def _at_side(self, arm_name):
        # check if we're already pretty close to the side position
        # move arm only does this if we're not in collision (which is kind of dumb)
        joint_state = self._arm_planners[arm_name].joint_positions_to_joint_state(
            self.side_joint_trajectory[arm_name][-1])
        current_joint_state = self._joint_state_listener.get_joint_positions(self._arm_planners[arm_name].joint_names)
        already_there = True
        for p in range(len(current_joint_state)):
            if abs(gt.angular_distance(current_joint_state[p], joint_state.position[p])) > 0.2:
                already_there = False
                rospy.loginfo('Joint '+joint_state.name[p]+' is out of position by '
                              + str(gt.angular_distance(current_joint_state[p], joint_state.position[p])))
                break
        return already_there
