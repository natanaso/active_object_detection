/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/** \author Benjamin Cohen, Sachin Chitta */

#include <sbpl_3dnav_planner/test_get_base_poses.h>

namespace sbpl_3dnav_planner
{
TestSBPLBasePoses::TestSBPLBasePoses(): node_handle_("~")
{
  /*  joint_angles_.resize(15,0.0);
  joint_names_.resize(15);
  //pr2 specific
  joint_names_[0] = "l_shoulder_pan_joint";
  joint_names_[1] = "l_shoulder_lift_joint";
  joint_names_[2] = "l_upper_arm_roll_joint";
  joint_names_[3] = "l_elbow_flex_joint";
  joint_names_[4] = "l_forearm_roll_joint";
  joint_names_[5] = "l_wrist_flex_joint";
  joint_names_[6] = "l_wrist_roll_joint";
  joint_names_[7] = "r_shoulder_pan_joint";
  joint_names_[8] = "r_shoulder_lift_joint";
  joint_names_[9] = "r_upper_arm_roll_joint";
  joint_names_[10] = "r_elbow_flex_joint";
  joint_names_[11] = "r_forearm_roll_joint";
  joint_names_[12] = "r_wrist_flex_joint";
  joint_names_[13] = "r_wrist_roll_joint";
  joint_names_[14] = "torso_lift_joint";
  */
  test_client_ = root_handle_.serviceClient<sbpl_3dnav_planner::GetBasePoses>("/sbpl_full_body_planning/find_base_poses");
  joint_states_subscriber_ = root_handle_.subscribe("joint_states", 1, &TestSBPLBasePoses::jointStatesCallback,this);
  ROS_INFO("[test] Initialization complete.");
}

TestSBPLBasePoses::~TestSBPLBasePoses()
{


}

void TestSBPLBasePoses::callService(const std::string &group_name)
{
  sbpl_3dnav_planner::GetBasePoses::Request request;
  sbpl_3dnav_planner::GetBasePoses::Response response;

  joint_states_lock_.lock();
  request.robot_state = robot_state_;
  joint_states_lock_.unlock();
  request.object_pose.header.stamp = ros::Time::now();
  request.object_pose.header.frame_id = "map";
  request.object_pose.pose.position.x = 1.0;
  request.object_pose.pose.position.y = 0.0;
  request.object_pose.pose.position.z = 1.0;
  request.object_pose.pose.orientation.w = 1.0;
  request.group_name = group_name;
  test_client_.call(request,response);
}

void TestSBPLBasePoses::jointStatesCallback(const sensor_msgs::JointStateConstPtr &state)
{
  joint_states_lock_.lock();
  robot_state_.joint_state.position = state->position;
  robot_state_.joint_state.name = state->name;

  // get base position
  try
  {
    tf_.lookupTransform("map", "base_footprint", ros::Time(0), base_map_transform_);
    geometry_msgs::PoseStamped body_pose;
    body_pose.header.frame_id = "map";
    body_pose.header.stamp = ros::Time(0.0);
    body_pose.pose.position.x = base_map_transform_.getOrigin().x();
    body_pose.pose.position.y = base_map_transform_.getOrigin().y();
    tf::quaternionTFToMsg(base_map_transform_.getRotation(),body_pose.pose.orientation);
    robot_state_.multi_dof_joint_state.frame_ids.push_back("map");
    robot_state_.multi_dof_joint_state.child_frame_ids.push_back("base_footprint");
    robot_state_.multi_dof_joint_state.poses.push_back(body_pose.pose);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("Is there a map? The map-robot transform failed. (%s)",ex.what());
  }
  joint_states_lock_.unlock();
}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_base_poses");
  ros::AsyncSpinner spinner(2); // Use 1 thread
  spinner.start();

  sbpl_3dnav_planner::TestSBPLBasePoses test;

  sleep(5.0);
  ROS_INFO("Calling service");
  test.callService("right_arm");

  sleep(5.0);
  ROS_INFO("Calling service");
  test.callService("left_arm");

  ros::waitForShutdown();
    
  return 0;
}
