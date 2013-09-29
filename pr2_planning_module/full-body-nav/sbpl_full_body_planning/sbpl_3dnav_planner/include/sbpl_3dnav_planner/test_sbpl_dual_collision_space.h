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

/** \author Benjamin Cohen */

#ifndef __SBPL_TWO_ARM_PLANNER_NODE_H_
#define __SBPL_TWO_ARM_PLANNER_NODE_H_

#include <iostream>
#include <map>
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <tf/transform_datatypes.h>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <angles/angles.h>
#include <sbpl_two_arm_planner/sbpl_dual_collision_space.h>
#include <sbpl_two_arm_planner_node/visualize_arm.h>
#include <sbpl_arm_planner/sbpl_arm_planner_params.h>
#include <pviz/pviz.h>
#include <sbpl_arm_planner/body_pose.h>

/** Messages **/
#include <geometry_msgs/Pose.h>
#include <arm_navigation_msgs/CollisionMap.h>
#include <arm_navigation_msgs/AttachedCollisionObject.h>
#include <arm_navigation_msgs/GetMotionPlan.h>
#include <trajectory_msgs/JointTrajectoryPoint.h> 
#include <kinematics_msgs/GetPositionIK.h>
#include <kinematics_msgs/GetPositionFK.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>

class TestSBPLCollisionSpace
{
  public:
    /** \brief Constructor-doesn't initialize class, you need to run init() */
    TestSBPLCollisionSpace();

    ~TestSBPLCollisionSpace();

    /** \brief Initialize the planner node. */
    bool init();

    /** @brief Run the node! */
    int run();

  private:

    ros::NodeHandle node_handle_, root_handle_;
    ros::Subscriber object_subscriber_;
    ros::Subscriber joint_states_subscriber_;
    message_filters::Subscriber<arm_navigation_msgs::CollisionMap> collision_map_subscriber_;
    tf::MessageFilter<arm_navigation_msgs::CollisionMap> *collision_map_filter_;

    //remove
    ros::Subscriber collision_object_subscriber_;
    void collisionObjectCallback(const arm_navigation_msgs::CollisionObjectConstPtr &collision_object);
    std::vector<std::vector<double> > col_objects_;

    std::string left_fk_service_name_;
    std::string left_ik_service_name_;
    std::string right_fk_service_name_;
    std::string right_ik_service_name_;

    /* params */
    bool visualize_expanded_states_;
    bool visualize_heuristic_;
    bool visualize_goal_;
    bool print_path_;
    bool visualize_trajectory_;
    bool visualize_collision_model_trajectory_;
    int throttle_;
    int num_joints_;

    std::string collision_map_topic_;
    std::string node_name_;
    std::string reference_frame_;
    std::string map_frame_;
    std::string arm_name_;
    std::string left_arm_description_filename_;
    std::string right_arm_description_filename_;
    std::vector<std::string> ljoint_names_;
    std::vector<std::string> rjoint_names_;
    std::map<std::string, arm_navigation_msgs::CollisionObject> object_map_;

    std::vector<double> langles_;
    std::vector<double> rangles_;
    double torso_lift_;
    double head_pan_;
    double head_tilt_;
    BodyPose body_pos_;

    bool verbose_;

    /* planner & environment */
    sbpl_two_arm_planner::SBPLDualCollisionSpace* cspace_;
    sbpl_arm_planner::SBPLArmModel* larm_;
    sbpl_arm_planner::SBPLArmModel* rarm_;
    sbpl_arm_planner::OccupancyGrid* grid_;
    sbpl_two_arm_planner::VisualizeArm* laviz_;
    sbpl_two_arm_planner::VisualizeArm* raviz_;
    PViz pviz_;
    sbpl_arm_planner::SBPLArmPlannerParams prms_;
    KDL::Frame kdl_transform_;
    tf::StampedTransform base_map_transform_;

    boost::mutex colmap_mutex_;
    boost::mutex object_mutex_;

    /* transforms and kinematics */
    tf::TransformListener tf_;
    tf::StampedTransform transform_;

    void updateMapFromCollisionMap(const arm_navigation_msgs::CollisionMapConstPtr &collision_map);

    void collisionMapCallback(const arm_navigation_msgs::CollisionMapConstPtr &collision_map);

    void jointStatesCallback(const sensor_msgs::JointStateConstPtr &state);

    void setArmToMapTransform(std::string &map_frame);

    void visualizeCollisionObjects();
};

#endif
