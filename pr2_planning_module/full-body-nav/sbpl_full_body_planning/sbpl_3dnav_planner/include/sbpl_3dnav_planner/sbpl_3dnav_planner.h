/*
 * Copyright (c) 2011, Maxim Likhachev
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

/** \author Mike Phillips, Benjamin Cohen  */

#ifndef __SBPL_3DNAV_PLANNER_H_
#define __SBPL_3DNAV_PLANNER_H_

#include <iostream>
#include <map>
#include <boost/thread/mutex.hpp>
#include <log4cxx/logger.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <angles/angles.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <message_filters/subscriber.h>
#include <nav_core/base_global_planner.h>
#include <pviz/pviz.h>
#include <sbpl_arm_planner/body_pose.h>
#include <sbpl_3dnav_planner/visualize_arm.h>
#include <sbpl_3dnav_planner/GetTwoArmPlan.h>
#include <sbpl_3dnav_planner/FullBodyCollisionCheck.h>
#include <sbpl_full_body_planner/environment_dualrobarm3d.h>
#include <sbpl_full_body_planner/pr2_collision_space.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <tf/transform_datatypes.h>

/** Messages **/
#include <arm_navigation_msgs/CollisionMap.h>
#include <arm_navigation_msgs/CollisionObject.h>
#include <arm_navigation_msgs/AttachedCollisionObject.h>
#include <arm_navigation_msgs/GetMotionPlan.h>
#include <kinematics_msgs/GetPositionFK.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectoryPoint.h> 
#include <sbpl_3dnav_planner/GetBasePoses.h> 
#include <sbpl_3dnav_planner/GetBasePose.h> 

/* MY ADDON */
#include <planning_environment/models/collision_models.h>


namespace sbpl_3dnav_planner
{
class Sbpl3DNavPlanner : public nav_core::BaseGlobalPlanner
{
public:
    Sbpl3DNavPlanner();
    ~Sbpl3DNavPlanner();
    
    /** nav_core::base_global_planner interface **/

    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan);

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /** full body interface **/

    bool init();
    
    int run();

    void changeLoggerLevel(std::string name, std::string level);

    bool setStart(geometry_msgs::Pose start, geometry_msgs::Pose rarm_object, geometry_msgs::Pose larm_object);

    bool setGoalPosition(geometry_msgs::Pose goal, geometry_msgs::Pose rarm_object, geometry_msgs::Pose larm_object, std::vector<double> &goal_tolerance);

    bool planToPosition(sbpl_3dnav_planner::GetTwoArmPlan::Request &req, sbpl_3dnav_planner::GetTwoArmPlan::Response &res);
    bool collisionCheck(sbpl_3dnav_planner::FullBodyCollisionCheck::Request &req, sbpl_3dnav_planner::FullBodyCollisionCheck::Response &res);
    bool getRobotPoseFromRobotState(arm_navigation_msgs::RobotState &state, vector<double>& langles, vector<double>& rangles, BodyPose& body);

	 bool getBasePose(sbpl_3dnav_planner::GetBasePose::Request &req,
		      sbpl_3dnav_planner::GetBasePose::Response &res);
    
    bool compBasePoses(geometry_msgs::Pose A, geometry_msgs::Pose B)
    {
			return getBasePoseDist(A) < getBasePoseDist(B);
    }

    double getBasePoseDist( geometry_msgs::Pose A )
    {
			double byaw = tf::getYaw(A.orientation);
			double dist = (body_pos_.x - A.position.x)*(body_pos_.x - A.position.x) + 
					 (body_pos_.y - A.position.y)*(body_pos_.y - A.position.y) +
					 (cos(body_pos_.theta) - cos(byaw))*(cos(body_pos_.theta) - cos(byaw)) +
					 (sin(body_pos_.theta) - sin(byaw))*(sin(body_pos_.theta) - sin(byaw));
			return dist;
    }
    
    bool getBasePoses(sbpl_3dnav_planner::GetBasePoses::Request &req,
		      sbpl_3dnav_planner::GetBasePoses::Response &res);

private:
    const bool forward_search_;

    ros::NodeHandle root_handle_;
    ros::NodeHandle node_handle_;

    /** message subscribers **/

    ros::Subscriber joint_states_subscriber_;
    ros::Subscriber collision_object_subscriber_;
    ros::Subscriber object_subscriber_;
    ros::Subscriber path_subscriber_;
    message_filters::Subscriber<arm_navigation_msgs::CollisionMap> collision_map_subscriber_;
    tf::MessageFilter<arm_navigation_msgs::CollisionMap>* collision_map_filter_;

    boost::mutex joint_states_mutex_;
    boost::mutex collision_map_mutex_;
    boost::mutex colmap_mutex_;

    /** main planning service **/

    ros::ServiceServer planning_service_;
    ros::ServiceServer collision_check_service_;
    ros::ServiceServer find_base_poses_service_;
    ros::ServiceServer find_base_pose_service_;

    /** Params from parameter server **/

    // planner params
    bool search_mode_;
    double allocated_time_;
    double object_radius_;
    std::string left_arm_description_filename_;
    std::string right_arm_description_filename_;
    std::string mprims_filename_;
    std::string base_mprims_filename_;
    bool use_shortened_path_;
    bool use_inner_circle_;

    // debug params
    bool print_path_;
    std::string succ_log_level_;

    // robot params
    double waypoint_time_;
    std::string arm_name_;
    std::string robot_description_;
    int num_joints_;

    // general params
    std::string reference_frame_;
    std::string left_fk_service_name_;
    std::string right_fk_service_name_;
    std::string left_ik_service_name_;
    std::string right_ik_service_name_;
    bool use_collision_map_from_sensors_;
    geometry_msgs::Pose rarm_object_offset_; // position loaded from PS, orient in constructor
    geometry_msgs::Pose larm_object_offset_; // position loaded from PS, orient in constructor

    // collision space params
    std::string collision_map_topic_;

    // visualizations params
    bool visualize_goal_;
    bool visualize_expanded_states_;
    bool visualize_heuristic_;
    double env_resolution_;
    bool visualize_trajectory_;
    bool visualize_end_effector_path_;
    bool visualize_collision_model_trajectory_;
    bool visualize_collision_model_;
    int throttle_;
    bool visualize_heuristic_grid_;

    /** More members **/

    clock_t totalPlanTime;

    bool planner_initialized_;
    std::string planning_joint_;
    std::vector<double> rangles_;
    std::vector<double> langles_;
    std::vector<std::string> ljoint_names_;
    std::vector<std::string> rjoint_names_;
    std::string map_frame_;
    bool attached_object_;

    double torso_lift_;
    double head_pan_;
    double head_tilt_;
    BodyPose body_pos_;

    std::vector<std::vector<double> > dpath_;
    std::vector<std::vector<double> > dpath0_;
    std::vector<std::vector<double> > dpath1_;

    PViz pviz_;

    /** debug **/

    double x_min_, x_max_, x_inc_;
    double y_min_, y_max_, y_inc_;
    double z_min_, z_max_, z_inc_;
    std::string succ_log_name_;
    std::vector<std::string> debug_code_names_;

    /** planner & environment **/

    MDPConfig mdp_cfg_;

    sbpl_full_body_planner::EnvironmentDUALROBARM3D sbpl_arm_env_;

    SBPLPlanner* planner_;
    sbpl_full_body_planner::PR2CollisionSpace* cspace_;
    sbpl_arm_planner::OccupancyGrid* grid_;
    sbpl_full_body_planner::VisualizeArm* laviz_;
    sbpl_full_body_planner::VisualizeArm* raviz_;

    std::vector<int> solution_state_ids_;
    std::vector<int> solution_state_ids_short_;

    arm_navigation_msgs::CollisionMap last_collision_map_;
    /** transforms and kinematics **/

    tf::TransformListener tf_;
    tf::StampedTransform transform_;
    tf::StampedTransform base_map_transform_;
    KDL::Frame kdl_transform_;

    //Arm* rarm_;
    //Arm* larm_;

    std::vector<std::string> stats_field_names_;
    std::vector<double> stats_;

    /** collision objects **/

    std::map<std::string, arm_navigation_msgs::CollisionObject> object_map_;

    /** callbacks **/

    void collisionMapCallback(const arm_navigation_msgs::CollisionMapConstPtr &collision_map);

    void collisionObjectCallback(const arm_navigation_msgs::CollisionObjectConstPtr &collision_object);

    void jointStatesCallback(const sensor_msgs::JointStateConstPtr &state);

    void attachedObjectCallback(const arm_navigation_msgs::AttachedCollisionObjectConstPtr &attached_object);

    /** planning **/

    bool initializePlannerAndEnvironment();

    bool plan(std::vector<trajectory_msgs::JointTrajectoryPoint> &rpath, std::vector<trajectory_msgs::JointTrajectoryPoint> &lpath, std::vector<trajectory_msgs::JointTrajectoryPoint> &bpath);

    bool isGoalConstraintSatisfied(const std::vector<double> &rangles, const std::vector<double> &langles, const geometry_msgs::Pose &goal);

    void setArmToMapTransform(std::string &map_frame);
    
    /** kinematics **/

    bool computeFK(const std::vector<double> &jnt_pos, std::string arm_name, geometry_msgs::Pose &pose);

    void getRobotState(BodyPose &body_pos, std::vector<double> &rangles, std::vector<double> &langles);

    /** visualizations **/

    void visualizeExpansions();

    void visualizeUniqueExpansions();

    void displayShortestPath();

    void printPath(FILE* fOut, const std::vector<std::vector<double> > path);

    void printPath(const std::vector<trajectory_msgs::JointTrajectoryPoint> &rpath, const std::vector<trajectory_msgs::JointTrajectoryPoint> &lpath, const std::vector<trajectory_msgs::JointTrajectoryPoint> &bpath);

    void visualizeGoal(geometry_msgs::Pose goal);

    void visualizeCollisionObjects(bool delete_first=false);

    void visualizeGoalPosition(const arm_navigation_msgs::Constraints &goal);
    
    void visualizeEndEffectorPath();
    
    void visualizeHeuristicInfo();

    void visualizeExpansionsPerHValue();

    void visualizeHeuristicGrid();

    void visualizeAttachedObject(bool delete_first=false);

    void visualizeCollisionObject(const arm_navigation_msgs::CollisionObject &object);

    void visualizeTrajectory(std::vector<trajectory_msgs::JointTrajectoryPoint> &rpath, std::vector<trajectory_msgs::JointTrajectoryPoint> &lpath, std::vector<trajectory_msgs::JointTrajectoryPoint> &bpath);

    void printRobotState(std::vector<double> &rangles, std::vector<double> &langles, BodyPose &body_pos, std::string text);

    void visualizeObjectPath();

    void visualizeCollisionModel(std::vector<double> &rangles, std::vector<double> &langles, BodyPose &body_pos, std::string text);
    
    /** base pose computation **/
    int yaw_steps_, radii_steps_;
    double minimum_working_distance_, maximum_working_distance_;

    bool getWorkingDistance(const geometry_msgs::Pose &object_pose,
        const geometry_msgs::Pose &shoulder_pose,
        double &distance);

    bool getPosesToCollisionCheck(const geometry_msgs::Pose &object_pose,
        const double &working_distance,
        std::vector<geometry_msgs::Pose> &base_poses);

    bool getPosesToCollisionCheck(const geometry_msgs::Pose &object_pose,
        const double &working_distance,
        const geometry_msgs::Point &offset,
        std::vector<geometry_msgs::Pose> &base_poses);


    /* attached objects */
    void attachObject(const arm_navigation_msgs::CollisionObject &obj, std::string link_name);
    
    /* MY ADDON */
    //ros::ServiceClient ik_query_client_;
    ros::ServiceClient ik_client_;
    
    bool checkIK(const geometry_msgs::PoseStamped &pose_stamped_msg,
					const std::string &link_name, arm_navigation_msgs::RobotState &rs);
	
	/*		
	void pose_stamped_to_position_orientation_constraints( const geometry_msgs::PoseStamped &pose_stamped_msg,
																		  const std::string &link_name,
																		  arm_navigation_msgs::PositionConstraint &pc,
																		  arm_navigation_msgs::OrientationConstraint &oc );
																			  
	*/
	
	/*																		  
	// For getting the current robot state
	planning_environment::CollisionModels* collision_models_;
	planning_models::KinematicState* planning_scene_state_;
	ros::ServiceClient set_planning_scene_diff_client_;
	bool getAndSetPlanningScene(const arm_navigation_msgs::PlanningScene& planning_diff,
		                      const arm_navigation_msgs::OrderedCollisionOperations& operations);
	*/

};

}

#endif
