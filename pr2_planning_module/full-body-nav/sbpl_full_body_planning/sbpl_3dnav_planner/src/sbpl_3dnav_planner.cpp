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

#include <pluginlib/class_list_macros.h>
#include <sbpl_3dnav_planner/sbpl_3dnav_planner.h>

// My IK Addon
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <planning_environment/models/model_utils.h>
#include <tf/transform_broadcaster.h>

clock_t starttime;

using namespace std;
using namespace sbpl_arm_planner;

PLUGINLIB_REGISTER_CLASS(Sbpl3DNavPlanner, sbpl_3dnav_planner::Sbpl3DNavPlanner, nav_core::BaseGlobalPlanner);

namespace sbpl_3dnav_planner
{
/** Initializers -------------------------------------------------------------*/
Sbpl3DNavPlanner::Sbpl3DNavPlanner() :
	forward_search_(true),
	node_handle_("~"),
	collision_map_subscriber_(root_handle_, "collision_map_occ", 1),
	collision_map_filter_(NULL),
	planner_initialized_(false),
	planning_joint_("r_wrist_roll_link"),
	map_frame_("map"),
	attached_object_(false),
	x_min_(0.0),
	x_max_(2.0),
	x_inc_(0.04),
	y_min_(0.0),
	y_max_(2.0),
	y_inc_(0.04),
	z_min_(0.7),
	z_max_(1.1),
	z_inc_(0.04),
	succ_log_name_("succ"),
	grid_(NULL),
	laviz_(NULL),
	raviz_(NULL),
  yaw_steps_(16),
  radii_steps_(16),
  minimum_working_distance_(0.3),
  maximum_working_distance_(0.7)
{
	langles_.resize(7, 0);
	rangles_.resize(7, 0);

	ljoint_names_.resize(7);
	rjoint_names_.resize(7);

	// PR2 specific joint names
	ljoint_names_[0] = "l_shoulder_pan_joint";
	ljoint_names_[1] = "l_shoulder_lift_joint";
	ljoint_names_[2] = "l_upper_arm_roll_joint";
	ljoint_names_[3] = "l_elbow_flex_joint";
	ljoint_names_[4] = "l_forearm_roll_joint";
	ljoint_names_[5] = "l_wrist_flex_joint";
	ljoint_names_[6] = "l_wrist_roll_joint";

	rjoint_names_[0] = "r_shoulder_pan_joint";
	rjoint_names_[1] = "r_shoulder_lift_joint";
	rjoint_names_[2] = "r_upper_arm_roll_joint";
	rjoint_names_[3] = "r_elbow_flex_joint";
	rjoint_names_[4] = "r_forearm_roll_joint";
	rjoint_names_[5] = "r_wrist_flex_joint";
	rjoint_names_[6] = "r_wrist_roll_joint";

	rarm_object_offset_.orientation.x = 0.0;
	rarm_object_offset_.orientation.y = 0.0;
	rarm_object_offset_.orientation.z = 0.0;
	rarm_object_offset_.orientation.w = 1.0;
	larm_object_offset_.orientation.x = 0.0;
	larm_object_offset_.orientation.y = 0.0;
	larm_object_offset_.orientation.z = 0.0;
	larm_object_offset_.orientation.w = 1.0;

  //TODO: These should be moved to be with the enum DebugCodes
	debug_code_names_.push_back("valid successor");
	debug_code_names_.push_back("collision between arms");
	debug_code_names_.push_back("right arm in collision");
	debug_code_names_.push_back("left arm in collision");
	debug_code_names_.push_back("attached object in collision");
	debug_code_names_.push_back("right ik fail ik search success");
	debug_code_names_.push_back("right ik fail ik search fail");
	debug_code_names_.push_back("left ik fail ik search success");
	debug_code_names_.push_back("left ik fail ik search fail");
	debug_code_names_.push_back("invalid right shoulder pan");
	debug_code_names_.push_back("invalid right shoulder pitch");
	debug_code_names_.push_back("invalid right upper arm roll");
	debug_code_names_.push_back("invalid right elbow flex");
	debug_code_names_.push_back("invalid right forearm roll");
	debug_code_names_.push_back("invalid right wrist pitch");
	debug_code_names_.push_back("invalid right wrist roll");
	debug_code_names_.push_back("invalid left shoulder pan");
	debug_code_names_.push_back("invalid left shoulder pitch");
	debug_code_names_.push_back("invalid left upper arm roll");
	debug_code_names_.push_back("invalid left elbow flex");
	debug_code_names_.push_back("invalid left forearm roll");
	debug_code_names_.push_back("invalid left wrist pitch");
	debug_code_names_.push_back("invalid left wrist roll");
}

Sbpl3DNavPlanner::~Sbpl3DNavPlanner()
{
	if (laviz_ != NULL) delete laviz_;
	if (raviz_ != NULL) delete raviz_;
	if (collision_map_filter_ != NULL) delete collision_map_filter_;
	if (planner_ != NULL) delete planner_;
	//if (rarm_ != NULL) delete rarm_;
	//if (larm_ != NULL) delete larm_;
}

bool Sbpl3DNavPlanner::init()
{
	// planner params
	node_handle_.param("planner/search_mode", search_mode_, true); // true: stop after first solution
	node_handle_.param("planner/allocated_time", allocated_time_, 60.0);
	node_handle_.param("planner/object_radius", object_radius_, 0.10);
	node_handle_.param<std::string>("planner/left_arm_description_file", left_arm_description_filename_, "");
	node_handle_.param<std::string>("planner/right_arm_description_file", right_arm_description_filename_, "");
	node_handle_.param<std::string>("planner/motion_primitive_file", mprims_filename_, "");
	node_handle_.param<std::string>("planner/base_motion_primitive_file", base_mprims_filename_, "");
	node_handle_.param("planner/use_shortened_path", use_shortened_path_, false);

	// debug params
	node_handle_.param("debug/print_out_path", print_path_, true);
	node_handle_.param<std::string>("debug/succesors_logger_level", succ_log_level_, "info");

	// robot params
	node_handle_.param("robot/waypoint_time", waypoint_time_, 0.2);
	node_handle_.param<std::string>("robot/arm_name", arm_name_, "right_arm");	//robot description
	std::string robot_urdf_param;
	if (!node_handle_.searchParam("robot_description", robot_urdf_param)) {
		ROS_ERROR("Unable to find robot description on param server (/robot_description is not set). Exiting");
		return false;
	}
	node_handle_.param<std::string>(robot_urdf_param, robot_description_, "robot_description");
	node_handle_.param("robot/num_joints", num_joints_, 7);

	// general params
	node_handle_.param<std::string>("reference_frame", reference_frame_, std::string("map"));
	node_handle_.param<std::string>("left_fk_service_name", left_fk_service_name_, "pr2_left_arm_kinematics/get_fk");
	node_handle_.param<std::string>("left_ik_service_name", left_ik_service_name_, "pr2_left_arm_kinematics/get_ik");
	node_handle_.param<std::string>("right_fk_service_name", right_fk_service_name_, "pr2_right_arm_kinematics/get_fk");
	node_handle_.param<std::string>("right_ik_service_name", right_ik_service_name_, "pr2_right_arm_kinematics/get_ik");
	node_handle_.param("use_collision_map_from_sensors", use_collision_map_from_sensors_, true);
	node_handle_.param("right_arm_pose_on_object_x", rarm_object_offset_.position.x, 0.0);
	node_handle_.param("right_arm_pose_on_object_y", rarm_object_offset_.position.y, -0.15);
	node_handle_.param("right_arm_pose_on_object_z", rarm_object_offset_.position.z, 0.0);
	node_handle_.param("left_arm_pose_on_object_x", larm_object_offset_.position.x, 0.0);
	node_handle_.param("left_arm_pose_on_object_y", larm_object_offset_.position.y, 0.15);
	node_handle_.param("left_arm_pose_on_object_z", larm_object_offset_.position.z, 0.0);

  //base pose determination params
	node_handle_.param("minimum_working_distance", minimum_working_distance_, 0.3);
	node_handle_.param("maximum_working_distance", maximum_working_distance_, 0.7);
	node_handle_.param<int>("yaw_steps", yaw_steps_, 16);
	node_handle_.param<int>("radii_steps", radii_steps_, 16);

	// collision space params
	node_handle_.param<std::string>("collision_space/collision_map_topic", collision_map_topic_, "collision_map_occ");

	// visualizations params
	node_handle_.param("visualizations/goal", visualize_goal_, true);
	node_handle_.param("visualizations/expanded_states", visualize_expanded_states_, true);
	node_handle_.param("visualizations/heuristic", visualize_heuristic_, true);
	node_handle_.param("visualizations/voxel_size", env_resolution_, 0.02);
	node_handle_.param("visualizations/trajectory", visualize_trajectory_, false);
	node_handle_.param("visualizations/end_effector_path", visualize_end_effector_path_, false);
	node_handle_.param("visualizations/collision_model_trajectory", visualize_collision_model_trajectory_, false);
	node_handle_.param("visualizations/collision_model", visualize_collision_model_, false);
	node_handle_.param("visualizations/trajectory_throttle", throttle_, 4);
	node_handle_.param("visualizations/heuristic_grid", visualize_heuristic_grid_, false);

	// initialize planner
	if (!initializePlannerAndEnvironment()) {
		return false;
	}

	typedef tf::MessageFilter<arm_navigation_msgs::CollisionMap> collisionMapFilter;
	collision_map_filter_ = new collisionMapFilter(collision_map_subscriber_, tf_, reference_frame_, 1);
	collision_map_filter_->registerCallback(boost::bind(&Sbpl3DNavPlanner::collisionMapCallback, this, _1));

	joint_states_subscriber_ = root_handle_.subscribe("joint_states", 1, &Sbpl3DNavPlanner::jointStatesCallback, this);
	collision_object_subscriber_ = root_handle_.subscribe("collision_object", 5, &Sbpl3DNavPlanner::collisionObjectCallback, this);
	object_subscriber_ = root_handle_.subscribe("attached_collision_object", 3, &Sbpl3DNavPlanner::attachedObjectCallback, this);
	collision_check_service_ = root_handle_.advertiseService("/sbpl_full_body_planning/collision_check", &Sbpl3DNavPlanner::collisionCheck,this);
	find_base_poses_service_ = root_handle_.advertiseService("/sbpl_full_body_planning/find_base_poses", &Sbpl3DNavPlanner::getBasePoses,this);
	find_base_pose_service_ = root_handle_.advertiseService("/sbpl_full_body_planning/find_base_pose", &Sbpl3DNavPlanner::getBasePose,this);

	// My IK addon
	ROS_INFO("[3dnav] Waiting for ik services...");
	//ros::service::waitForService("pr2_right_arm_kinematics/get_ik_solver_info");
	//ros::service::waitForService("pr2_right_arm_kinematics/get_ik");
	ros::service::waitForService("pr2_right_arm_kinematics/get_constraint_aware_ik");
  	ROS_INFO("[3dnav] Got IK services!");

	//ik_query_client_ = root_handle_.serviceClient<kinematics_msgs::GetKinematicSolverInfo>("pr2_right_arm_kinematics/get_ik_solver_info");
	//ik_client_ = root_handle_.serviceClient<kinematics_msgs::GetPositionIK>("pr2_right_arm_kinematics/get_ik");
	ik_client_ = root_handle_.serviceClient<kinematics_msgs::GetConstraintAwarePositionIK>("pr2_right_arm_kinematics/get_constraint_aware_ik");
	/*
	collision_models_ = new planning_environment::CollisionModels("robot_description");
	planning_scene_state_ = NULL;
	set_planning_scene_diff_client_ = root_handle_.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>("/environment_server/set_planning_scene_diff");
	ROS_INFO("[3dnav] Waiting for the SetPlanningSceneDiff service...");
	ros::service::waitForService("/environment_server/set_planning_scene_diff");
	ROS_INFO("[3dnav] Got SetPlanningSceneDiff service!");
	*/
	
	planner_initialized_ = true;
	ROS_INFO("[3dnav] The SBPL arm planner node initialized succesfully.");
	return true;
}

int Sbpl3DNavPlanner::run()
{
	ros::spin();
	return 0;
}

bool Sbpl3DNavPlanner::initializePlannerAndEnvironment()
{
	if (robot_description_.empty()) {
		ROS_ERROR("[3dnav] Robot description file is empty. Exiting.");
		return false;
	}

	// initialize arm planner environment
	if (!sbpl_arm_env_.initEnvironment(right_arm_description_filename_, left_arm_description_filename_,
	                                   mprims_filename_, base_mprims_filename_)) {
		ROS_ERROR("[3dnav] ERROR: initEnvironment failed");
		return false;
	}

	cspace_ = sbpl_arm_env_.getCollisionSpace();
	grid_ = sbpl_arm_env_.getOccupancyGrid();

	planner_ = new ARAPlanner(&sbpl_arm_env_, forward_search_);

	// set epsilon
	planner_->set_initialsolution_eps(sbpl_arm_env_.getEpsilon());
	sbpl_arm_env_.computeNormalHeuristic = true;

	// set search mode (true - settle with first solution)
	planner_->set_search_mode(search_mode_);

	laviz_ = new sbpl_full_body_planner::VisualizeArm(std::string("left_arm"));
	raviz_ = new sbpl_full_body_planner::VisualizeArm(std::string("right_arm"));
	laviz_->setReferenceFrame(reference_frame_);
	raviz_->setReferenceFrame(reference_frame_);

	ROS_INFO("[3dnav] Initialized sbpl planning environment.");
	return true;
}

/********************************** Callbacks *********************************/

void Sbpl3DNavPlanner::collisionMapCallback(const arm_navigation_msgs::CollisionMapConstPtr &collision_map)
{
  colmap_mutex_.lock();
  ROS_DEBUG("collision map callback");
  if (collision_map->header.frame_id.compare(reference_frame_) != 0 &&
      collision_map->header.frame_id.compare("/" + reference_frame_) != 0) {
    ROS_WARN("[3dnav] The collision map received is in %s frame but expected in %s frame.", collision_map->header.frame_id.c_str(), reference_frame_.c_str());
  }

  // add collision map msg
  if (use_collision_map_from_sensors_) {
    grid_->updateFromCollisionMap(*collision_map);
  }

  last_collision_map_ = *collision_map;
  cspace_->storeCollisionMap(*collision_map);
  map_frame_ = collision_map->header.frame_id;
  setArmToMapTransform(map_frame_);

  cspace_->putCollisionObjectsInGrid();
  colmap_mutex_.unlock();
  grid_->visualize();

}

void Sbpl3DNavPlanner::collisionObjectCallback(const arm_navigation_msgs::CollisionObjectConstPtr &collision_object)
{
  colmap_mutex_.lock();

  if (collision_object->id.compare("all") == 0) {
    cspace_->removeAllCollisionObjects();
  }

  // debug: have we seen this collision object before?
  if (object_map_.find(collision_object->id) != object_map_.end()) {
    ROS_INFO("[3dnav] [collision_objects] We have seen this object ('%s')  before.",
        collision_object->id.c_str());
  }
  else {
    ROS_INFO("[3dnav] [collision_objects] We have NOT seen this object ('%s') before.",
        collision_object->id.c_str());
  }

  // add the object to our internal map for later attaching
  object_map_[collision_object->id] = (*collision_object);

  cspace_->processCollisionObjectMsg(*collision_object);

  //visualizeCollisionObject(*collision_object);

  colmap_mutex_.unlock();

  grid_->visualize();
  //visualize collision object voxels
  visualizeCollisionObjects(true);
}

void Sbpl3DNavPlanner::jointStatesCallback(const sensor_msgs::JointStateConstPtr &state)
{
	ROS_DEBUG("joint states callback");

  // arms
	for(unsigned int i=0; i<rjoint_names_.size(); i++){
		unsigned int j;
		for(j=0; j<state->name.size(); j++)
			if(rjoint_names_[i].compare(state->name[j])==0)
				break;
		if(j==state->name.size())
			ROS_WARN("[jointStatesCallback] Missing the value for planning joint (%s)\n",rjoint_names_[i].c_str());
		else
			rangles_[i] = state->position[j];
	}
	for(unsigned int i=0; i<ljoint_names_.size(); i++){
		unsigned int j;
		for(j=0; j<state->name.size(); j++)
			if(ljoint_names_[i].compare(state->name[j])==0)
				break;
		if(j==state->name.size())
			ROS_WARN("[jointStatesCallback] Missing the value for planning joint (%s)\n",rjoint_names_[i].c_str());
		else
			langles_[i] = state->position[j];
	}

  // torso
	unsigned int j;
	for (j = 0; j < state->name.size(); j++)
		if (state->name[j].compare("torso_lift_joint") == 0) break;
	if(j==state->name.size())
		ROS_WARN("[jointStatesCallback] Missing the value for planning joint torso_lift_joint\n");
	else
		body_pos_.z = state->position[j];

  // base
	try {
		tf_.lookupTransform("map", "base_footprint", ros::Time(0), base_map_transform_);
		body_pos_.x = base_map_transform_.getOrigin().x();
		body_pos_.y = base_map_transform_.getOrigin().y();
		body_pos_.theta = 2 * atan2(base_map_transform_.getRotation().getZ(), base_map_transform_.getRotation().getW());
		ROS_DEBUG("Received transform from base_footprint to map (x: %f y: %f yaw: %f)", body_pos_.x, body_pos_.y, body_pos_.theta);
	}
	catch (tf::TransformException& ex) {
		ROS_ERROR("Is there a map? The map-robot transform failed. (%s)", ex.what());
	}

  if(visualize_collision_model_)
    visualizeCollisionModel(rangles_, langles_, body_pos_, "pr2_collision_model");
	
  if (attached_object_) {
		visualizeAttachedObject();
	}

  ROS_DEBUG("[3dnav] [robot state] x: %0.3f  y: %0.3f  theta: %0.3f  torso: %0.3f", body_pos_.x, body_pos_.y, body_pos_.theta, body_pos_.z); 
}

void Sbpl3DNavPlanner::attachedObjectCallback(const arm_navigation_msgs::AttachedCollisionObjectConstPtr &attached_object)
{
  // remove all objects
  /*if(attached_object->link_name.compare(arm_navigation_msgs::AttachedCollisionObject::REMOVE_ALL_ATTACHED_OBJECTS) == 0)*/
  if(attached_object->link_name.compare("all") == 0)/* &&
      attached_object->object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::REMOVE) */
  {
    ROS_INFO("[3dnav] Removing all attached objects.");
    attached_object_ = false;
    cspace_->removeAllAttachedObjects();
    visualizeAttachedObject(true);
  }
  // add object
  else if(attached_object->object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::ADD)
  {
    ROS_INFO("[3dnav] Received a message to ADD an object (%s) with %d shapes.", attached_object->object.id.c_str(), int(attached_object->object.shapes.size()));
    object_map_[attached_object->object.id] = attached_object->object;
    attachObject(attached_object->object, attached_object->link_name);
  }
  // attach object and remove it from collision space
  else if(attached_object->object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::ATTACH_AND_REMOVE_AS_OBJECT)
  {
    ROS_INFO("[3dnav] Received a message to ATTACH_AND_REMOVE_AS_OBJECT of object: %s", attached_object->object.id.c_str());
    // have we seen this collision object before?
    if(object_map_.find(attached_object->object.id) != object_map_.end())
    {
      ROS_INFO("[3dnav] We have seen this object (%s) before (it's in my internal object map).", attached_object->object.id.c_str());
      attachObject(object_map_.find(attached_object->object.id)->second, attached_object->link_name);
    }
    else
    {
      ROS_INFO("[3dnav] We have NOT seen this object (%s) before (it's not in my internal object map).", attached_object->object.id.c_str());
      if(attached_object->object.shapes.empty())
      {
        ROS_WARN("[3dnav] '%s' is not in my internal object map and the message doesn't contain any shapes. Can't attach it.", attached_object->object.id.c_str());
        return;
      }
      object_map_[attached_object->object.id] = attached_object->object;
      attachObject(attached_object->object, attached_object->link_name);
    }
    ROS_INFO("[3dnav] Just attached '%s', now I'll remove it from the world.", attached_object->object.id.c_str());
    cspace_->removeCollisionObject(attached_object->object);
    visualizeCollisionObjects(true);
    grid_->visualize();
  }
  // remove object
  else if(attached_object->object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::REMOVE)
  {
    ROS_INFO("[3dnav] Removing object (%s) from gripper.", attached_object->object.id.c_str());
    cspace_->removeAttachedObject(attached_object->object.id);
    visualizeAttachedObject(true);
  }
  // detach and add as object
  else if(attached_object->object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::DETACH_AND_ADD_AS_OBJECT)
  {
    ROS_INFO("[3dnav] Received a message to DETACH_AND_ADD_AS_OBJECT of object: %s", attached_object->object.id.c_str());
    //ROS_INFO("[3dnav] Removing object (%s) from gripper and adding to collision map.", attached_object->object.id.c_str());
    cspace_->removeAttachedObject(attached_object->object.id);
    attached_object_ = cspace_->isObjectAttached();
    visualizeAttachedObject(true);
    //sometimes people are lazy and they don't fill in the object's
    //description. in those cases - we have to depend on our stored
    //description of the object to be added. if we can't find it in our
    //object map, we hope that they added in a description themselves.
    if(object_map_.find(attached_object->object.id) != object_map_.end())
    {
      ROS_INFO("[3dnav] Detached '%s' and now I'll add it as a known collision object. The message does not contain its description but I had it stored so I know what it is.", attached_object->object.id.c_str());
      cspace_->addCollisionObject(object_map_.find(attached_object->object.id)->second);
    }
    else
    {
      if(attached_object->object.shapes.empty())
      {
        ROS_WARN("[3dnav] '%s' was attached and now you want to add it as a collision object. Unfortunatly the message does not contain its decription and I don't have it in my internal database for some reason so I can't attach it. This is probably a bug.", attached_object->object.id.c_str());
        return;
      }
      object_map_[attached_object->object.id] = attached_object->object;
      cspace_->addCollisionObject(attached_object->object);
    }
    //visualize exact collision object
    //visualizeCollisionObject(attached_object->object);
    visualizeCollisionObjects(true);
    grid_->visualize();
  }
  else
    ROS_WARN("[3dnav] Received a collision object with an unknown operation");

  attached_object_ = cspace_->isObjectAttached();
}

void Sbpl3DNavPlanner::attachObject(const arm_navigation_msgs::CollisionObject &obj, std::string link_name)
{
  geometry_msgs::PoseStamped pose_in, pose_out;
  arm_navigation_msgs::CollisionObject object(obj);
  attached_object_ = true;
  ROS_INFO("[3dnav] Received a collision object message with id, '%s' and it contains %d shapes.", object.id.c_str(), int(object.shapes.size()));

  for(size_t i = 0; i < object.shapes.size(); i++)
  {
    // transform the object's pose into the link_name's pose
    // (we know that that link must be on the robot)
    pose_in.header = object.header;
    pose_in.header.stamp = ros::Time();
    pose_in.pose = object.poses[i];
    try
    {
      tf_.transformPose(cspace_->getExpectedAttachedObjectFrame(link_name), pose_in, pose_out);
    }
    catch(int e)
    {
      ROS_ERROR("[3dnav] Failed to transform the pose of the attached object from %s to %s. (exception: %d)", object.header.frame_id.c_str(), cspace_->getExpectedAttachedObjectFrame(link_name).c_str(), e);
      ROS_ERROR("[3dnav] Failed to attach '%s' object.", object.id.c_str());
      return;
    }
    object.poses[i] = pose_out.pose;
    ROS_INFO("[3dnav] Converted attached object pose of '%s' shape from %s (%0.2f %0.2f %0.2f) to %s (%0.3f %0.3f %0.3f)", obj.id.c_str(), pose_in.header.frame_id.c_str(), pose_in.pose.position.x, pose_in.pose.position.y, pose_in.pose.position.z, pose_out.header.frame_id.c_str(), pose_out.pose.position.x, pose_out.pose.position.y, pose_out.pose.position.z);

    if(object.shapes[i].type == arm_navigation_msgs::Shape::SPHERE)
    {
      ROS_INFO("[3dnav] Attaching a '%s' sphere with radius: %0.3fm", object.id.c_str(), object.shapes[i].dimensions[0]);
      cspace_->attachSphere(object.id, link_name, object.poses[i], object.shapes[i].dimensions[0]);
    }
    else if(object.shapes[i].type == arm_navigation_msgs::Shape::CYLINDER)
    {
      ROS_INFO("[3dnav] Attaching a '%s' cylinder with radius: %0.3fm & length %0.3fm", object.id.c_str(), object.shapes[i].dimensions[0], object.shapes[i].dimensions[1]);
      cspace_->attachCylinder(object.id, link_name, object.poses[i], object.shapes[i].dimensions[0], object.shapes[i].dimensions[1]);
    }
    else if(object.shapes[i].type == arm_navigation_msgs::Shape::MESH)
    {
      ROS_INFO("[3dnav] Attaching a '%s' mesh with %d triangles/3  & %d vertices.", object.id.c_str(), int(object.shapes[i].triangles.size()/3), int(object.shapes[i].vertices.size()));
      cspace_->attachMesh(object.id, link_name, object.poses[i], object.shapes[i].vertices, object.shapes[i].triangles);
    }
    else if(object.shapes[i].type == arm_navigation_msgs::Shape::BOX)
    {
      ROS_INFO("[3dnav] Attaching a '%s' cube with dimensions {%0.3fm x %0.3fm x %0.3fm}.", object.id.c_str(), object.shapes[i].dimensions[0], object.shapes[i].dimensions[1], object.shapes[i].dimensions[2]);
      cspace_->attachCube(object.id, link_name, object.poses[i], object.shapes[i].dimensions[0], object.shapes[i].dimensions[1], object.shapes[i].dimensions[2]);
    }
    else
      ROS_WARN("[3dnav] Currently attaching objects of type '%d' aren't supported.", object.shapes[i].type);
  }

  visualizeAttachedObject(true);
}

/********************* Get Valid Base Poses Service ********************************/

bool Sbpl3DNavPlanner::getWorkingDistance(const geometry_msgs::Pose &object_pose,
                                          const geometry_msgs::Pose &shoulder_pose,
                                          double &distance)
{
  double delta_z = object_pose.position.z - shoulder_pose.position.z;
  if(fabs(delta_z) > maximum_working_distance_)
  {
    ROS_ERROR("Location unreachable");
    return false;
  }
  distance = sqrt(maximum_working_distance_*maximum_working_distance_ - delta_z*delta_z);
  return true;
}

bool Sbpl3DNavPlanner::getPosesToCollisionCheck(const geometry_msgs::Pose &object_pose,
                                                const double &working_distance,
                                                const geometry_msgs::Point &offset,
                                                std::vector<geometry_msgs::Pose> &base_poses)
{
  std::vector<double> yaws_to_try, radii_to_try;
  double yaw_delta = 2 * M_PI/ yaw_steps_;
  for(int i=0; i < yaw_steps_; i++)
  {
   yaws_to_try.push_back(i*yaw_delta);
  }

  double radius_delta = (working_distance - minimum_working_distance_)/radii_steps_;
  ROS_DEBUG("Radius delta %f, Working distance: %f",radius_delta,working_distance);
  for(int i=0; i < radii_steps_; i++)
    radii_to_try.push_back(working_distance - i*radius_delta);

  for(unsigned int i=0; i < yaws_to_try.size(); i++)
  {
    for(unsigned int j=0; j < radii_to_try.size(); j++)
    {
      geometry_msgs::Pose pose;
      pose.orientation.z = 1.0;
      pose.position.x = object_pose.position.x + (radii_to_try[j]+offset.x) * cos(yaws_to_try[i]) - offset.y* sin(yaws_to_try[i]);
      pose.position.y = object_pose.position.y + (radii_to_try[j]+offset.x) * sin(yaws_to_try[i]) + offset.y* cos(yaws_to_try[i]);
      tf::Quaternion quat;
      quat.setRPY(0.0,0.0,yaws_to_try[i] + M_PI);
      tf::quaternionTFToMsg(quat,pose.orientation);
      base_poses.push_back(pose);
    }
  }
  return true;
}

bool Sbpl3DNavPlanner::getPosesToCollisionCheck(const geometry_msgs::Pose &object_pose,
                                                const double &working_distance,
                                                std::vector<geometry_msgs::Pose> &base_poses)
{
  std::vector<double> yaws_to_try, radii_to_try;
  double yaw_delta = 2 * M_PI/ yaw_steps_;
  for(int i=0; i < yaw_steps_; i++)
  {
   yaws_to_try.push_back(i*yaw_delta);
  }

  double radius_delta = (working_distance - minimum_working_distance_)/radii_steps_;
  ROS_DEBUG("[3dnav] Radius delta %f, Working distance: %f",radius_delta,working_distance);
  for(int i=0; i < radii_steps_; i++)
   radii_to_try.push_back(working_distance - i*radius_delta);

  for(unsigned int i=0; i < yaws_to_try.size(); i++)
  {
    for(unsigned int j=0; j < radii_to_try.size(); j++)
    {
      geometry_msgs::Pose pose;
      pose.orientation.z = 1.0;
      pose.position.x = object_pose.position.x + radii_to_try[j] * cos(yaws_to_try[i]);
      pose.position.y = object_pose.position.y + radii_to_try[j] * sin(yaws_to_try[i]);
      tf::Quaternion quat;
      quat.setRPY(0.0,0.0,yaws_to_try[i] + M_PI);
      tf::quaternionTFToMsg(quat,pose.orientation);
      base_poses.push_back(pose);
    }
  }
  return true;
}

bool Sbpl3DNavPlanner::getBasePose(sbpl_3dnav_planner::GetBasePose::Request &req,
                                   sbpl_3dnav_planner::GetBasePose::Response &res)
{
	bool look_goal = false;
	std::string link_name;
	if(req.group_name == "right_arm")
		link_name = "r_shoulder_pan_link";
	else if (req.group_name == "left_arm")
		link_name = "l_shoulder_pan_link";
	else if(req.group_name == "torso")
		link_name = "torso_lift_link";
	else
	{
		look_goal = true;
		link_name = "torso_lift_link";
	}
    
	// link_name
	tf::StampedTransform link_map_transform;
	try 
	{
		tf_.lookupTransform("map",link_name,ros::Time(0),link_map_transform);
	}
	catch (tf::TransformException& ex) 
	{
		ROS_ERROR("***********Is there a map? The map-robot transform failed. (%s)", ex.what());
		res.error_code.val = res.error_code.FRAME_TRANSFORM_FAILURE;
		return true;
	}

	geometry_msgs::PoseStamped my_object_pose = req.object_pose;
	my_object_pose.header.stamp = ros::Time(0.0);
	try 
   {
   	tf_.transformPose("map",my_object_pose,my_object_pose);
	}
	catch (tf::TransformException& ex) 
	{
		ROS_ERROR("**********Is there a map? The map-robot transform failed. (%s)", ex.what());
		res.error_code.val = res.error_code.FRAME_TRANSFORM_FAILURE;
		return true;
	}

    geometry_msgs::PointStamped offset;
    offset.header.stamp = ros::Time(0.0);
    offset.header.frame_id = link_name;
    try 
    {
      tf_.transformPoint("torso_lift_link",offset,offset);
    }
    catch (tf::TransformException& ex) 
    {
      ROS_ERROR("**********Is there a torso_lift_link? The transform failed. (%s)", ex.what());
      res.error_code.val = res.error_code.FRAME_TRANSFORM_FAILURE;
      return true;
    }

    if(look_goal)
    {
      // we are trying to look. Set the height to a comfortable height for the robot
      my_object_pose.pose.position.z = 1.0;
    }
   
   geometry_msgs::TransformStamped link_transform;
   tf::transformStampedTFToMsg(link_map_transform,link_transform);
   geometry_msgs::Pose link_pose;
   link_pose.position.x = link_transform.transform.translation.x;
   link_pose.position.y = link_transform.transform.translation.y;
   link_pose.position.z = link_transform.transform.translation.z;
   link_pose.orientation = link_transform.transform.rotation;
   double working_distance;
   if(!getWorkingDistance(my_object_pose.pose,link_pose,working_distance))
   {
     res.error_code.val = res.error_code.PLANNING_FAILED;
     return true;
   }


	/*
	 * DETERMINE THE BASE POSES!!!
	 * Here my_object_pose is in the map frame! 
	 * Base poses are from base to map transforms
	 */
	std::vector<geometry_msgs::Pose> base_poses;
	double base_yaw = tf::getYaw( my_object_pose.pose.orientation ) + M_PI;
	double yaw_dev = 2.62;	// 150 deg
	int yaw_steps = yaw_steps_ / 2;
	double yaw_delta = yaw_dev / yaw_steps;

	std::vector<double> yaws_to_try, radii_to_try;
	yaws_to_try.push_back( base_yaw );
  	for(int i=1; i <= yaw_steps; i++)
  	{
  		yaws_to_try.push_back(base_yaw + i*yaw_delta);
		yaws_to_try.push_back(base_yaw - i*yaw_delta);
  	}

	double radius_delta = (working_distance - minimum_working_distance_)/radii_steps_;
  	ROS_WARN("Radius delta %f, Working distance: %f",radius_delta,working_distance);
  	for(int i=0; i < radii_steps_; i++)
   	radii_to_try.push_back(working_distance - i*radius_delta);

  	for(unsigned int i=0; i < yaws_to_try.size(); i++)
  	{
   	for(unsigned int j=0; j < radii_to_try.size(); j++)
   	{
   		geometry_msgs::Pose pose;
      	pose.orientation.z = 1.0;
      	pose.position.x = my_object_pose.pose.position.x + (radii_to_try[j]+offset.point.x) * cos(yaws_to_try[i]) - offset.point.y* sin(yaws_to_try[i]);
      	pose.position.y = my_object_pose.pose.position.y + (radii_to_try[j]+offset.point.x) * sin(yaws_to_try[i]) + offset.point.y* cos(yaws_to_try[i]);
      	tf::Quaternion quat;
      	quat.setRPY(0.0,0.0,yaws_to_try[i] + M_PI);
      	tf::quaternionTFToMsg(quat,pose.orientation);
      	base_poses.push_back(pose);
    	}
  	}
  	
  	// sort bases poses according to distance away from current pose
  	//std::sort (base_poses.begin(), base_poses.end(), boost::bind( &Sbpl3DNavPlanner::compBasePoses, this, _1, _2 ));
	/*************** BASE POSE DETERMINATION *******************/

   sbpl_3dnav_planner::FullBodyCollisionCheck::Request fb_request;
   sbpl_3dnav_planner::FullBodyCollisionCheck::Response fb_response;

   fb_request.robot_states.resize(base_poses.size());
   // my object is in map frame
	ROS_WARN("GOAL OBJECT: %f %f %f", my_object_pose.pose.position.x,my_object_pose.pose.position.y,base_yaw);
   for(unsigned int i=0; i < base_poses.size(); i++)
   {
     fb_request.robot_states[i] = req.robot_state;
     fb_request.robot_states[i].multi_dof_joint_state.frame_ids.clear();
     fb_request.robot_states[i].multi_dof_joint_state.child_frame_ids.clear();
     fb_request.robot_states[i].multi_dof_joint_state.poses.clear();
     fb_request.robot_states[i].multi_dof_joint_state.frame_ids.push_back("map");
     fb_request.robot_states[i].multi_dof_joint_state.child_frame_ids.push_back("base_footprint");
     fb_request.robot_states[i].multi_dof_joint_state.poses.push_back(base_poses[i]);
     double yaw = tf::getYaw(base_poses[i].orientation);
	  if( i < 10)
     		ROS_DEBUG("Trying: %f %f %f",base_poses[i].position.x,base_poses[i].position.y,yaw);
   }

   ROS_DEBUG("Robot request");
   for(unsigned int i=0; i < req.robot_state.joint_state.name.size(); i++)
   {
     ROS_DEBUG("%d name: %s",i,req.robot_state.joint_state.name[i].c_str());
   }

   if(!collisionCheck(fb_request,fb_response))
   {
     ROS_ERROR("Error in collision checking");
     return false;
   }


	// goal pose to map transform
	// object_pose is from r_wrist_roll_link to map transform
   tf::Transform mTk;
	mTk.setOrigin( tf::Vector3(req.object_pose.pose.position.x, 
										req.object_pose.pose.position.y,
									   req.object_pose.pose.position.z) );
									   
	mTk.setRotation( tf::Quaternion( req.object_pose.pose.orientation.x,
												req.object_pose.pose.orientation.y,
												req.object_pose.pose.orientation.z,
												req.object_pose.pose.orientation.w) );
	// current pose transform											
	tf::Transform mTp;
	mTp.setOrigin( tf::Vector3(body_pos_.x, body_pos_.y, 0.0));
	tf::Quaternion q_mTp;
  	q_mTp.setRPY( 0.0, 0.0, body_pos_.theta );								
	mTp.setRotation( q_mTp );

	static tf::TransformBroadcaster br;
	   
   for(unsigned int i=0; i < fb_response.error_codes.size(); i++)
   {
     if (fb_response.error_codes[i].val == arm_navigation_msgs::ArmNavigationErrorCodes::SUCCESS) 
       {
         geometry_msgs::PoseStamped pose_stamped;
         pose_stamped.header.frame_id = "map";
         pose_stamped.header.stamp = ros::Time(0.0);
         pose_stamped.pose = base_poses[i];
         
         // Check IK for the req.object_pose (mTk), which is in map (m) frame
         // Current pose (mTp): body_pos_.x, body_pos_.y, body_pos_.theta
         // Future pose (mTb): base_poses[i]
         // Requested pose (mTk): req.object_pose
         // mTf = mTp bTm mTk
			tf::Transform mTb;
			mTb.setOrigin( tf::Vector3( base_poses[i].position.x, 
												 base_poses[i].position.y, 0.0));
			mTb.setRotation( tf::Quaternion( base_poses[i].orientation.x, 
														base_poses[i].orientation.y,
														base_poses[i].orientation.z, 
														base_poses[i].orientation.w) );
														
			tf::Transform map_T_imginary( mTp * mTb.inverseTimes( mTk ));
			
			br.sendTransform(tf::StampedTransform( map_T_imginary, ros::Time::now(), "/map", "/imaginary_wrist"));
			
         geometry_msgs::PoseStamped map_T_rwrist;
         map_T_rwrist.header.frame_id = "map";	//XXX: important that it is not /map
         map_T_rwrist.header.stamp = ros::Time::now();
         map_T_rwrist.pose.position.x = map_T_imginary.getOrigin().x();
         map_T_rwrist.pose.position.y = map_T_imginary.getOrigin().y();
         map_T_rwrist.pose.position.z = map_T_imginary.getOrigin().z();
         
         map_T_rwrist.pose.orientation.x = map_T_imginary.getRotation().x();
         map_T_rwrist.pose.orientation.y = map_T_imginary.getRotation().y();
         map_T_rwrist.pose.orientation.z = map_T_imginary.getRotation().z();
         map_T_rwrist.pose.orientation.w = map_T_imginary.getRotation().w(); 
         
         bool valid_goal = checkIK(map_T_rwrist, "r_wrist_roll_link", req.robot_state);
			
			if(valid_goal){		 
         	res.base_poses.push_back(pose_stamped);
         	break;
         }
         else{
         	ROS_INFO("[3dnav] IK for base pose is not valid!");
         }
       }
   }
   								 
	// NOTE: final poses are sorted according to distance away from the required yaw!
	//ROS_INFO("There are %d valid poses out of %d!", res.base_poses.size(), base_poses.size());

   return true; 
}

bool Sbpl3DNavPlanner::getBasePoses(sbpl_3dnav_planner::GetBasePoses::Request &req,
                                    sbpl_3dnav_planner::GetBasePoses::Response &res)
{
  bool look_goal = false;
  ROS_DEBUG("In Get base poses");
  std::string link_name;
    if(req.group_name == "right_arm")
      link_name = "r_shoulder_pan_link";
    else if (req.group_name == "left_arm")
      link_name = "l_shoulder_pan_link";
    else if(req.group_name == "torso")
      link_name = "torso_lift_link";
    else
    {
      look_goal = true;
      link_name = "torso_lift_link";
    }
    
    // link_name
    tf::StampedTransform link_map_transform;
    try 
    {
      tf_.lookupTransform("map",link_name,ros::Time(0),link_map_transform);
    }
    catch (tf::TransformException& ex) 
    {
      ROS_ERROR("***********Is there a map? The map-robot transform failed. (%s)", ex.what());
      res.error_code.val = res.error_code.FRAME_TRANSFORM_FAILURE;
      return true;
    }

    geometry_msgs::PoseStamped my_object_pose = req.object_pose;
    my_object_pose.header.stamp = ros::Time(0.0);
    try 
    {
      tf_.transformPose("map",my_object_pose,my_object_pose);
    }
    catch (tf::TransformException& ex) 
    {
      ROS_ERROR("**********Is there a map? The map-robot transform failed. (%s)", ex.what());
      res.error_code.val = res.error_code.FRAME_TRANSFORM_FAILURE;
      return true;
    }

    geometry_msgs::PointStamped offset;
    offset.header.stamp = ros::Time(0.0);
    offset.header.frame_id = link_name;
    try 
    {
      tf_.transformPoint("torso_lift_link",offset,offset);
    }
    catch (tf::TransformException& ex) 
    {
      ROS_ERROR("**********Is there a torso_lift_link? The transform failed. (%s)", ex.what());
      res.error_code.val = res.error_code.FRAME_TRANSFORM_FAILURE;
      return true;
    }

    if(look_goal)
    {
      // we are trying to look. Set the height to a comfortable height for the robot
      my_object_pose.pose.position.z = 1.0;
    }
   
   geometry_msgs::TransformStamped link_transform;
   tf::transformStampedTFToMsg(link_map_transform,link_transform);
   geometry_msgs::Pose link_pose;
   link_pose.position.x = link_transform.transform.translation.x;
   link_pose.position.y = link_transform.transform.translation.y;
   link_pose.position.z = link_transform.transform.translation.z;
   link_pose.orientation = link_transform.transform.rotation;
   double working_distance;
   if(!getWorkingDistance(my_object_pose.pose,link_pose,working_distance))
   {
     res.error_code.val = res.error_code.PLANNING_FAILED;
     return true;
   }

   std::vector<geometry_msgs::Pose> base_poses;
   getPosesToCollisionCheck(my_object_pose.pose,working_distance,offset.point,base_poses);
   
   sbpl_3dnav_planner::FullBodyCollisionCheck::Request fb_request;
   sbpl_3dnav_planner::FullBodyCollisionCheck::Response fb_response;

   fb_request.robot_states.resize(base_poses.size());
   for(unsigned int i=0; i < base_poses.size(); i++)
   {
     fb_request.robot_states[i] = req.robot_state;
     fb_request.robot_states[i].multi_dof_joint_state.frame_ids.clear();
     fb_request.robot_states[i].multi_dof_joint_state.child_frame_ids.clear();
     fb_request.robot_states[i].multi_dof_joint_state.poses.clear();
     fb_request.robot_states[i].multi_dof_joint_state.frame_ids.push_back("map");
     fb_request.robot_states[i].multi_dof_joint_state.child_frame_ids.push_back("base_footprint");
     fb_request.robot_states[i].multi_dof_joint_state.poses.push_back(base_poses[i]);
     double yaw = tf::getYaw(base_poses[i].orientation);
     ROS_DEBUG("Trying: %f %f %f",base_poses[i].position.x,base_poses[i].position.y,yaw);
   }

   ROS_DEBUG("Robot request");
   for(unsigned int i=0; i < req.robot_state.joint_state.name.size(); i++)
   {
     ROS_DEBUG("%d name: %s",i,req.robot_state.joint_state.name[i].c_str());
   }

   if(!collisionCheck(fb_request,fb_response))
   {
     ROS_ERROR("Error in collision checking");
     return false;
   }

   for(unsigned int i=0; i < fb_response.error_codes.size(); i++)
   {
     if (fb_response.error_codes[i].val == arm_navigation_msgs::ArmNavigationErrorCodes::SUCCESS) 
       {
         geometry_msgs::PoseStamped pose_stamped;
         pose_stamped.header.frame_id = "map";
         pose_stamped.header.stamp = ros::Time(0.0);
         pose_stamped.pose = base_poses[i];
         res.base_poses.push_back(pose_stamped);
       }
   }
   return true; 
}


/**************************** Collision Check Service *******************************/

bool Sbpl3DNavPlanner::collisionCheck(sbpl_3dnav_planner::FullBodyCollisionCheck::Request &req,
                                      sbpl_3dnav_planner::FullBodyCollisionCheck::Response &res)
{
	colmap_mutex_.lock();

	for(unsigned int i=0; i<req.robot_states.size(); i++){
		vector<double> langles;
		vector<double> rangles;
		BodyPose pose;
		getRobotPoseFromRobotState(req.robot_states[i],langles,rangles,pose);
		ROS_DEBUG("[collisionCheck] x = %f, y = %f, theta = %f", pose.x, pose.y, pose.theta);
		//TODO:Adjust for map origin offset
		unsigned char dist_temp;
		int debug_code;
		if(cspace_->checkAllMotion(langles,rangles,pose,true,dist_temp,debug_code)) {
			arm_navigation_msgs::ArmNavigationErrorCodes errorCode;
			errorCode.val = arm_navigation_msgs::ArmNavigationErrorCodes::SUCCESS;
			res.error_codes.push_back(errorCode);
		}
		else {
			arm_navigation_msgs::ArmNavigationErrorCodes errorCode;
			errorCode.val = arm_navigation_msgs::ArmNavigationErrorCodes::COLLISION_CONSTRAINTS_VIOLATED;
			res.error_codes.push_back(errorCode);
		}

		if (res.error_codes[i].val == arm_navigation_msgs::ArmNavigationErrorCodes::SUCCESS) {
			ROS_DEBUG("SBPL full body planning collision checking service returned SUCCESS");
			pviz_.visualizeRobot(rangles, langles, pose, 150, "collision_checking_valid", 30 * i);
		}
		else {
			pviz_.visualizeRobot(rangles, langles, pose, 10, "collision_checking_invalid", 30 * i);
		}
	}

	ROS_INFO("[CollisionCheckService] Checked %d robot states for collisions.", int(req.robot_states.size()));
	colmap_mutex_.unlock();
	return true;
}





bool Sbpl3DNavPlanner::getRobotPoseFromRobotState(arm_navigation_msgs::RobotState &state, vector<double>& langles, vector<double>& rangles, BodyPose& body)
{
	unsigned int lind = 0, rind = 0;
	langles.resize(ljoint_names_.size());
	rangles.resize(rjoint_names_.size());

	// arms
	for(size_t i = 0; i < state.joint_state.name.size(); i++)
	{
    ROS_DEBUG("Joint name: %s",state.joint_state.name[i].c_str());
    for(unsigned int j=0; j < rjoint_names_.size(); j++)
    {
			if(rjoint_names_[j].compare(state.joint_state.name[i]) == 0)
			{
				ROS_DEBUG("[exp] [right-start] %-20s: %0.3f", rjoint_names_[j].c_str(), state.joint_state.position[i]);
				rangles[j] = state.joint_state.position[i];
				rind++;
			}
    }
    for(unsigned int j=0; j < ljoint_names_.size(); j++)
		{
			if(ljoint_names_[j].compare(state.joint_state.name[i]) == 0)
			{
				ROS_DEBUG("[exp] [left-start] %-20s: %0.3f", ljoint_names_[j].c_str(), state.joint_state.position[i]);
				langles[j] = state.joint_state.position[i];
				lind++;
			}
		}
		if(rind == rjoint_names_.size() && lind == ljoint_names_.size())
			break;
	}

	if(rind != rjoint_names_.size() || lind != ljoint_names_.size())
	{
		ROS_WARN("[exp] Not all of the expected joints were assigned a starting position.");
		return false;
	}

	// torso
	for(size_t i = 0; i < state.joint_state.name.size(); i++)
	{
		if(state.joint_state.name[i].compare("torso_lift_joint") == 0)
		{
			body.z = state.joint_state.position[i];
			break;
		}
	}

	// base
	for(size_t i = 0; i < state.multi_dof_joint_state.frame_ids.size(); ++i)
	{
		if(state.multi_dof_joint_state.frame_ids[i].compare("map") == 0 &&
				state.multi_dof_joint_state.child_frame_ids[i].compare("base_footprint") == 0)
		{
			body.x = state.multi_dof_joint_state.poses[i].position.x;
			body.y = state.multi_dof_joint_state.poses[i].position.y;

			tf::Quaternion qbase;
			tf::quaternionMsgToTF(state.multi_dof_joint_state.poses[i].orientation, qbase);
			body.theta =  2 * atan2(qbase.getZ(), qbase.getW());
			break;
		}
	}
	return true;
}



/**************************** Planner Interface *******************************/

bool Sbpl3DNavPlanner::setStart(geometry_msgs::Pose start, geometry_msgs::Pose rarm_object,
                                geometry_msgs::Pose larm_object)
{
	ROS_INFO("[3dnav] start0: %1.3f %1.3f %1.3f %1.3f %1.3f %1.3f %1.3f", rangles_[0], rangles_[1], rangles_[2],
	         rangles_[3], rangles_[4], rangles_[5], rangles_[6]);
	ROS_INFO("[3dnav] start1: %1.3f %1.3f %1.3f %1.3f %1.3f %1.3f %1.3f", langles_[0], langles_[1], langles_[2],
	         langles_[3], langles_[4], langles_[5], langles_[6]);

	std::vector<double> vstart(6, 0);
	vstart[0] = start.position.x;
	vstart[1] = start.position.y;
	vstart[2] = start.position.z;

	tf::Pose tstart;
	double roll, pitch, yaw;
	tf::poseMsgToTF(start, tstart);
	tstart.getBasis().getRPY(roll, pitch, yaw);
	vstart[3] = roll;
	vstart[4] = pitch;
	vstart[5] = yaw;

	KDL::Frame rarm_offset, larm_offset;
	rarm_offset.p.x(rarm_object.position.x);
	rarm_offset.p.y(rarm_object.position.y);
	rarm_offset.p.z(rarm_object.position.z);
	larm_offset.p.x(larm_object.position.x);
	larm_offset.p.y(larm_object.position.y);
	larm_offset.p.z(larm_object.position.z);
	rarm_offset.M = KDL::Rotation::Quaternion(rarm_object.orientation.x, rarm_object.orientation.y,
	                                          rarm_object.orientation.z, rarm_object.orientation.w);
	larm_offset.M = KDL::Rotation::Quaternion(larm_object.orientation.x, larm_object.orientation.y,
	                                          larm_object.orientation.z, larm_object.orientation.w);

	int startid = -1;
	printRobotState(rangles_, langles_, body_pos_, "start state");
	//pviz_.visualizeRobotWithTitle(rangles_, langles_, body_pos_, 30, "start", 0, "start");
	vector<double> temp1(7.0);
//	pviz_.visualizeRobotWithTitle(temp1, temp1, body_pos_, 30, "start1", 0, "start1");
	if ((startid = sbpl_arm_env_.setStartConfiguration(rangles_, langles_, body_pos_, rarm_offset, larm_offset)) == -1) {
		ROS_ERROR("[3dnav] Environment failed to set start state. Not Planning.");
		return false;
	}
	ROS_INFO("[3dnav] Start stateid: %d", startid);

	if (planner_->set_start(startid) == 0) {
		ROS_ERROR("[3dnav] Failed to set start state. Not Planning.");
		return false;
	}

	if (attached_object_) {
		visualizeAttachedObject();
	}

	return true;
}

bool Sbpl3DNavPlanner::setGoalPosition(geometry_msgs::Pose goal, geometry_msgs::Pose rarm_object,
                                       geometry_msgs::Pose larm_object, std::vector<double> &goal_tolerance)
{
	int goalid = -1;
	double roll, pitch, yaw;
	geometry_msgs::Pose pgoal;
	tf::Pose tf_pose;
	std::vector<std::vector<double> > sbpl_goal(1, std::vector<double>(11, 0)); //Changed to include Quaternion
	std::vector<std::vector<double> > sbpl_tolerance(1, std::vector<double>(12, 0));

	//currently only supports one goal
	sbpl_goal[0][0] = goal.position.x;
	sbpl_goal[0][1] = goal.position.y;
	sbpl_goal[0][2] = goal.position.z;

	// perturb quaternion to prevent gimbal lock when using grasping pipeline
	pgoal = goal;
	pgoal.orientation.w += 0.005;

	tf::poseMsgToTF(pgoal, tf_pose);
	tf_pose.getBasis().getRPY(roll, pitch, yaw);
	sbpl_goal[0][3] = roll;
	sbpl_goal[0][4] = pitch;
	sbpl_goal[0][5] = yaw;

	//6dof goal: true, 3dof: false
	sbpl_goal[0][6] = true;

	//orientation constraint as a quaternion
	sbpl_goal[0][7] = goal.orientation.x;
	sbpl_goal[0][8] = goal.orientation.y;
	sbpl_goal[0][9] = goal.orientation.z;
	sbpl_goal[0][10] = goal.orientation.w;

	//allowable tolerance from goal
	sbpl_tolerance[0] = goal_tolerance;

	ROS_INFO("[3dnav] goal xyz(%s): %.3f %.3f %.3f (tol: %.3fm) rpy: %.3f %.3f %.3f (tol: %.3frad)", map_frame_.c_str(),
	         sbpl_goal[0][0], sbpl_goal[0][1], sbpl_goal[0][2], sbpl_tolerance[0][0], sbpl_goal[0][3], sbpl_goal[0][4],
	         sbpl_goal[0][5], sbpl_tolerance[0][1]);

	KDL::Frame rarm_offset, larm_offset;
	rarm_offset.p.x(rarm_object.position.x);
	rarm_offset.p.y(rarm_object.position.y);
	rarm_offset.p.z(rarm_object.position.z);
	larm_offset.p.x(larm_object.position.x);
	larm_offset.p.y(larm_object.position.y);
	larm_offset.p.z(larm_object.position.z);

	rarm_offset.M = KDL::Rotation::Quaternion(rarm_object.orientation.x, rarm_object.orientation.y, rarm_object.orientation.z, rarm_object.orientation.w);
	larm_offset.M = KDL::Rotation::Quaternion(larm_object.orientation.x, larm_object.orientation.y, larm_object.orientation.z, larm_object.orientation.w);

	//set object_radius if no object attached
	if (!attached_object_) {
		sbpl_arm_env_.setObjectRadius(object_radius_);
	}

	ROS_INFO("[3dnav] Setting goal position.");
	//set sbpl environment goal
	if ((goalid = sbpl_arm_env_.setGoalPosition(sbpl_goal, sbpl_tolerance, rarm_offset, larm_offset, object_radius_)) == -1) {
		ROS_ERROR("[3dnav] Failed to set goal state. Perhaps goal position is out of reach. Exiting.");
		return false;
	}
	ROS_INFO("[3dnav] Goal stateID: %d", goalid);

	//set planner goal
	if (planner_->set_goal(goalid) == 0) {
		ROS_ERROR("[3dnav] Failed to set goal state. Exiting.");
		return false;
	}

	return true;
}

bool Sbpl3DNavPlanner::planToPosition(GetTwoArmPlan::Request &req, GetTwoArmPlan::Response &res)
{
	std::vector<trajectory_msgs::JointTrajectoryPoint> lpath, rpath, bpath;

	starttime = clock();

	ROS_INFO("[3dnav] About to set start configuration");

	// set start
	rangles_ = req.rarm_start;
	langles_ = req.larm_start;
	body_pos_.x = req.body_start[0];
	body_pos_.y = req.body_start[1];
	body_pos_.z = req.body_start[2];
	body_pos_.theta = req.body_start[3];
	if (!setStart(req.start.pose, req.rarm_object.pose, req.larm_object.pose)) {
		ROS_ERROR("[3dnav] Failed to set the starting configuration.");
		return false;
	}

	rarm_object_offset_ = req.rarm_object.pose;
	larm_object_offset_ = req.larm_object.pose;

//	std::vector<double> solution(7, 0);
//	if (!larm_->computeIK(req.goal, langles_, solution)) {
//		ROS_WARN("[3dnav] Failed to compute an IK solution for the left arm.");
//	}
//
//	if (!rarm_->computeIK(req.goal, rangles_, solution)) {
//		ROS_WARN("[3dnav] Failed to compute an IK solution for the right arm.");
//	}

	totalPlanTime = clock();

	// set goal
	if (!setGoalPosition(req.goal.pose, req.rarm_object.pose, req.larm_object.pose, req.absolute_xyzrpy_tolerance)) {
		ROS_ERROR("[3dnav] Failed to set the goal pose.");
		return false;
	}

	if (visualize_goal_) {
		visualizeGoal(req.goal.pose);
	}

	if (visualize_heuristic_grid_) {
		visualizeHeuristicGrid();
		return true;
	}

	//plan a path
	if (plan(rpath, lpath, bpath)) {
		//fill the response message
		res.trajectory.header.stamp = ros::Time::now();
		res.trajectory.header.frame_id = reference_frame_;
		res.trajectory.points.resize(rpath.size());
		res.trajectory.joint_names.resize(14);

		// added for base trajectory
		res.body_trajectory.points = bpath;

		for (size_t i = 0; i < 7; ++i) {
			res.trajectory.joint_names[i] = rjoint_names_[i];
			res.trajectory.joint_names[i + 7] = ljoint_names_[i];
		}
		for (size_t i = 0; i < res.trajectory.points.size(); ++i) {
			res.trajectory.points[i].positions.resize(14);
			for (size_t j = 0; j < 7; ++j) {
				res.trajectory.points[i].positions[j] = rpath[i].positions[j];
				res.trajectory.points[i].positions[j + 7] = lpath[i].positions[j];
			}
			if (use_shortened_path_) {
				res.trajectory.points[i].time_from_start = ros::Duration((i + 1) * waypoint_time_ * 3);
			}
			else {
				res.trajectory.points[i].time_from_start = ros::Duration((i + 1) * waypoint_time_);
			}

			if (i == res.trajectory.points.size() - 1) {
				ROS_INFO("[3dnav] Adding an additional second for the 'adaptive mprim' waypoint time.");
				res.trajectory.points[i].time_from_start += ros::Duration(1.0);
			}
		}

		res.stats_field_names = stats_field_names_;
		res.stats = stats_;

		if (print_path_)
			printPath(rpath, lpath, bpath);

		// visualizations
		if (visualize_expanded_states_)
			visualizeUniqueExpansions();

		if (visualize_heuristic_)
			//visualizeHeuristicInfo();
			displayShortestPath();

		if (visualize_trajectory_) {
			ROS_INFO("[3dnav] Visualizing trajectory...");
			//visualizeTrajectory(rpath, lpath, bpath);
      pviz_.visualizeTrajectory(rpath, lpath, bpath, throttle_);
      ROS_INFO("[3dnav] Finished visualizing trajectory.");
		}

		if (visualize_end_effector_path_) {
			ROS_INFO("[3dnav] Visualizing end effector path...");
			visualizeObjectPath();
			//visualizeEndEffectorPath();
			//visualizeExpansionsPerHValue();
		}
	}
	else {
		ROS_ERROR("[3dnav] Failed to plan within alotted time frame (%0.2f seconds).", allocated_time_);

		res.stats_field_names = stats_field_names_;
		res.stats = stats_;

		if (visualize_expanded_states_) {
			visualizeUniqueExpansions();
		}
		//visualizeExpansions();

		if (visualize_heuristic_) {
			//visualizeHeuristicInfo();
			displayShortestPath();
		}

//		if (!sbpl_arm_env_.checkExpandedStatesAreValid())
//			ROS_WARN("[3dnav] Invalid states were found.");

		std::vector<std::vector<double> > arm0, arm1;
		sbpl_arm_env_.getFinalArmConfigurations(arm0, arm1);


//		for (size_t i = 0; i < arm0.size(); ++i) {
//			raviz_->visualizeArmConfiguration((i % 260), arm0[i]);
//			laviz_->visualizeArmConfiguration((i % 260), arm1[i]);
//		}


	}

	return true;
}

bool Sbpl3DNavPlanner::plan(std::vector<trajectory_msgs::JointTrajectoryPoint> &rpath,
                            std::vector<trajectory_msgs::JointTrajectoryPoint> &lpath,
                            std::vector<trajectory_msgs::JointTrajectoryPoint> &bpath)
{
	bool b_ret = false;
	int solution_cost;
	std::vector<double> angles(num_joints_, 0);

	rpath.clear();
	lpath.clear();
	bpath.clear();

	ROS_INFO(" ");
	ROS_INFO("[3dnav] Calling planner");

	// reinitialize the search space
	planner_->force_planning_from_scratch();

	sbpl_arm_env_.recordDebugData(true);

	clock_t starttime = clock();

  ReplanParams params(allocated_time_);	// planning time: was set to 15.0 sec.
  params.initial_eps = sbpl_arm_env_.getEpsilon();
  params.final_eps = 1.0;
  params.dec_eps = 0.2;
  params.return_first_solution = false;
  params.repair_time = 4.0;

	// plan
	b_ret = planner_->replan(&solution_state_ids_, params, &solution_cost);

	totalPlanTime = clock() - totalPlanTime;

	ROS_INFO("[3dnav] Planning Time: %0.4fsec", (clock() - starttime) / (double)CLOCKS_PER_SEC);

	sbpl_arm_env_.recordDebugData(false);

	// check if an empty plan was received.
	if ((b_ret && solution_state_ids_.size() <= 0) || !b_ret) {
		ROS_WARN("[3dnav] Planning failed.");
	}
	else {
		ROS_INFO("[3dnav] Planning succeeded.");
	}

	// if a path is returned, then pack it into msg form
	if (b_ret && (solution_state_ids_.size() > 0)) {
		std::vector<std::vector<double> > angles_path, shortened_path;
		sbpl_arm_env_.convertStateIDPathToJointAnglesPath(solution_state_ids_, angles_path);
//		sbpl_arm_env_.convertStateIDPathToShortenedJointAnglesPath(solution_state_ids_,shortened_path, solution_state_ids_short_);

		ROS_INFO("[3dnav] A path was returned with %d waypoints. Shortened path has %d waypoints.", int(solution_state_ids_.size()), int(solution_state_ids_short_.size()));
		ROS_INFO("[3dnav] Initial Epsilon: %0.3f  Final Epsilon: %0.3f Solution Cost: %d", planner_->get_initial_eps(), planner_->get_final_epsilon(), solution_cost);

		if (use_shortened_path_) {
			ROS_INFO("[3dnav] Using shortened path.");
			angles_path = shortened_path;
		}
		else {
			ROS_INFO("[3dnav] Not using shortened path.");
		}

		if (angles_path.size() == 0) {
			ROS_ERROR("[3dnav] Returned path has at least 1 stateid but is empty after converting to joint angles.");
			return false;
		}

		if (angles_path.size() % 4 != 0) {
			ROS_ERROR("[3dnav] Length of path received from environment is not a multiple of 4. (length: %d)", int(angles_path.size()));
			return false;
		}

		trajectory_msgs::JointTrajectoryPoint traj_point;
		trajectory_msgs::JointTrajectoryPoint body_point;

		body_point.positions.resize(4);
		for (size_t i = 0; i < angles_path.size(); ++i) {
			traj_point.positions.resize(angles_path.size());
			for (size_t p = 0; p < angles_path[i].size(); ++p) {
				traj_point.positions[p] = angles_path[i][p];
			}

			if (i % 4 == 0) {
				rpath.push_back(traj_point);
			}
			else if (i % 4 == 1) {
				lpath.push_back(traj_point);
			}
			else if (i % 4 == 2) {
				body_point.positions[2] = traj_point.positions[0];
			}
			else {
				body_point.positions[0] = traj_point.positions[0];
				body_point.positions[1] = traj_point.positions[1];
				body_point.positions[3] = traj_point.positions[2];
				bpath.push_back(body_point);
			}

//			  printf("[3dnav] [%d] [stateid: %d] ", int(i), solution_state_ids_[i/4]);
//			  for(size_t q = 0; q < angles_path[i].size(); ++q)
//			  printf("% 1.4f ", angles_path[i][q]);
//			  printf("\n");
		}
	}

	// planner stats
	int num_planning_stats = 12;
	int total_num_stats = num_planning_stats + sbpl_arm_planner::NUM_DEBUG_CODES;

	stats_field_names_.resize(total_num_stats);
	stats_field_names_[0] = "total plan time";
	stats_field_names_[1] = "initial solution planning time";
	stats_field_names_[2] = "initial epsilon";
	stats_field_names_[3] = "initial solution expansions";
	stats_field_names_[4] = "final epsilon planning time";
	stats_field_names_[5] = "final epsilon";
	stats_field_names_[6] = "solution epsilon";
	stats_field_names_[7] = "expansions";
	stats_field_names_[8] = "solution cost";
	stats_field_names_[9] = "path length";
//	stats_field_names_[10] = "initial heuristic time";
//	stats_field_names_[11] = "in search heuristic time";

	stats_.resize(total_num_stats);
	stats_[0] = double(totalPlanTime) / CLOCKS_PER_SEC;
	stats_[1] = planner_->get_initial_eps_planning_time();
	stats_[2] = planner_->get_initial_eps();
	stats_[3] = planner_->get_n_expands_init_solution();
	stats_[4] = planner_->get_final_eps_planning_time();
	stats_[5] = planner_->get_final_epsilon();
	stats_[6] = planner_->get_solution_eps();
	stats_[7] = planner_->get_n_expands();
	stats_[8] = solution_cost;
	stats_[9] = solution_state_ids_.size();
//	double initial_heuristic_time, in_search_heuristic_time;
//	sbpl_arm_env_.getHeuristicTime(&initial_heuristic_time, &in_search_heuristic_time);
//	stats_[10] = initial_heuristic_time;
//	stats_[11] = in_search_heuristic_time;

	// environment stats
	std::vector<double> env_stats = sbpl_arm_env_.getPlanningStats();

	for (size_t i = 0; i < env_stats.size(); ++i) {
		stats_field_names_[i + num_planning_stats] = debug_code_names_[i];
		stats_[i + num_planning_stats] = env_stats[i];
	}

	return b_ret;
}

bool Sbpl3DNavPlanner::isGoalConstraintSatisfied(const std::vector<double> &rangles, const std::vector<double> &langles, const geometry_msgs::Pose &goal)
{
	bool satisfied = true;
	geometry_msgs::Pose rpose, lpose, lerr, rerr, rgoal, lgoal;
	tf::Pose tgoal, tright, tleft;

	tf::poseMsgToTF(goal, tgoal);
	tf::poseMsgToTF(rarm_object_offset_, tright);
	tf::poseMsgToTF(larm_object_offset_, tleft);

	tright = tgoal * tright;
	tleft = tgoal * tleft;

	tf::poseTFToMsg(tright, rgoal);
	tf::poseTFToMsg(tleft, lgoal);

	if (!computeFK(rangles, "right_arm", rpose)) {
		ROS_ERROR("[3dnav] Failed to check if goal constraint is satisfied because the right arm FK service failed.");
		return false;
	}

	if (!computeFK(langles, "left_arm", lpose)) {
		ROS_ERROR("[3dnav] Failed to check if goal constraint is satisfied because the left arm FK service failed.");
		return false;
	}

	lerr.position.x = fabs(lpose.position.x - lgoal.position.x);
	lerr.position.y = fabs(lpose.position.y - lgoal.position.y);
	lerr.position.z = fabs(lpose.position.z - lgoal.position.z);
	lerr.orientation.x = fabs(lpose.orientation.x - lgoal.orientation.x);
	lerr.orientation.y = fabs(lpose.orientation.y - lgoal.orientation.y);
	lerr.orientation.z = fabs(lpose.orientation.z - lgoal.orientation.z);
	lerr.orientation.w = fabs(lpose.orientation.w - lgoal.orientation.w);

	rerr.position.x = fabs(rpose.position.x - rgoal.position.x);
	rerr.position.y = fabs(rpose.position.y - rgoal.position.y);
	rerr.position.z = fabs(rpose.position.z - rgoal.position.z);
	rerr.orientation.x = fabs(rpose.orientation.x - rgoal.orientation.x);
	rerr.orientation.y = fabs(rpose.orientation.y - rgoal.orientation.y);
	rerr.orientation.z = fabs(rpose.orientation.z - rgoal.orientation.z);
	rerr.orientation.w = fabs(rpose.orientation.w - rgoal.orientation.w);

	ROS_INFO(" ");
	ROS_INFO("[3dnav] -- Right Gripper --");
	ROS_INFO("[3dnav]  Pose:  xyz: %0.4f %2.4f %0.4f  quat: %0.4f %0.4f %0.4f %0.4f", rpose.position.x, rpose.position.y, rpose.position.z, rpose.orientation.x, rpose.orientation.y, rpose.orientation.z, rpose.orientation.w);
	ROS_INFO("[3dnav]  Goal:  xyz: %0.4f %2.4f %0.4f  quat: %0.4f %0.4f %0.4f %0.4f", rgoal.position.x, rgoal.position.y, rgoal.position.z, rgoal.orientation.x, rgoal.orientation.y, rgoal.orientation.z, rgoal.orientation.w);
	ROS_INFO("[3dnav] Error:  xyz: %0.4f %2.4f %0.4f  quat: %0.4f %0.4f %0.4f %0.4f", rerr.position.x, rerr.position.y, rerr.position.z, rerr.orientation.x, rerr.orientation.y, rerr.orientation.z, rerr.orientation.w);
	ROS_INFO(" ");
	ROS_INFO("[3dnav] -- Left Gripper --");
	ROS_INFO("[3dnav]  Pose:  xyz: %0.4f %2.4f %0.4f  quat: %0.4f %0.4f %0.4f %0.4f", lpose.position.x, lpose.position.y, lpose.position.z, lpose.orientation.x, lpose.orientation.y, lpose.orientation.z, lpose.orientation.w);
	ROS_INFO("[3dnav]  Goal:  xyz: %0.4f %2.4f %0.4f  quat: %0.4f %0.4f %0.4f %0.4f", lgoal.position.x, lgoal.position.y, lgoal.position.z, lgoal.orientation.x, lgoal.orientation.y, lgoal.orientation.z, lgoal.orientation.w);
	ROS_INFO("[3dnav] Error:  xyz: %0.4f %2.4f %0.4f  quat: %0.4f %0.4f %0.4f %0.4f", lerr.position.x, lerr.position.y, lerr.position.z, lerr.orientation.x, lerr.orientation.y, lerr.orientation.z, lerr.orientation.w);
	ROS_INFO(" ");

//	 if (goal.position_constraints[0].constraint_region_shape.type == arm_navigation_msgs::Shape::BOX) {
//		if (goal.position_constraints[0].constraint_region_shape.dimensions.size() < 3) {
//			ROS_WARN("[3dnav] Goal constraint region shape is a BOX but fewer than 3 dimensions are defined.");
//			return false;
//		}
//		if (err.position.x >= goal.position_constraints[0].constraint_region_shape.dimensions[0]) {
//			ROS_WARN("X is not satisfied (error: %0.4f   tolerance: %0.4f)", err.position.x, goal.position_constraints[0].constraint_region_shape.dimensions[0]);
//			satisfied = false;
//		}
//		if (err.position.y >= goal.position_constraints[0].constraint_region_shape.dimensions[1]) {
//			ROS_WARN("Y is not satisfied (error: %0.4f   tolerance: %0.4f)", err.position.y, goal.position_constraints[0].constraint_region_shape.dimensions[1]);
//			satisfied = false;
//		}
//		if (err.position.z >= goal.position_constraints[0].constraint_region_shape.dimensions[2]) {
//			ROS_WARN("Z is not satisfied (error: %0.4f   tolerance: %0.4f)", err.position.z, goal.position_constraints[0].constraint_region_shape.dimensions[2]);
//			satisfied = false;
//		}
//	}
//	else if (goal.position_constraints[0].constraint_region_shape.type == arm_navigation_msgs::Shape::SPHERE) {
//		if (goal.position_constraints[0].constraint_region_shape.dimensions.size() < 1) {
//			ROS_WARN("Goal constraint region shape is a SPHERE but it has no dimensions...");
//			return false;
//		}
//		if (err.position.x >= goal.position_constraints[0].constraint_region_shape.dimensions[0]) {
//			ROS_WARN("X is not satisfied (error: %0.4f   tolerance: %0.4f)", err.position.x, goal.position_constraints[0].constraint_region_shape.dimensions[0]);
//			satisfied = false;
//		}
//		if (err.position.y >= goal.position_constraints[0].constraint_region_shape.dimensions[0]) {
//			ROS_WARN("Y is not satisfied (error: %0.4f   tolerance: %0.4f)", err.position.y, goal.position_constraints[0].constraint_region_shape.dimensions[1]);
//			satisfied = false;
//		}
//		if (err.position.z >= goal.position_constraints[0].constraint_region_shape.dimensions[0]) {
//			ROS_WARN("Z is not satisfied (error: %0.4f   tolerance: %0.4f)", err.position.z, goal.position_constraints[0].constraint_region_shape.dimensions[2]);
//			satisfied = false;
//		}
//	}
//	else
//		ROS_WARN("Goal constraint region shape is of type %d.", goal.position_constraints[0].constraint_region_shape.type);

	return satisfied;
}

/**************************** nav_core interface methods **********************/

bool Sbpl3DNavPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                const geometry_msgs::PoseStamped& goal,
                                std::vector<geometry_msgs::PoseStamped>& plan)
{
	ROS_INFO("[makePlan] start = {frame: %s, position: {x: %.3f, y: %.3f, z: %.3f},"
		     " orientation: {x: %.3f, y: %.3f, z: %.3f, w:%.3f}",
	         start.header.frame_id.c_str(), start.pose.position.x, start.pose.position.y, start.pose.position.z,
	         start.pose.orientation.x, start.pose.orientation.y, start.pose.orientation.z, start.pose.orientation.w);

	ROS_INFO("[makePlan] goal = {frame: %s, position: {x: %.3f, y: %.3f, z: %.3f},"
			 " orientation: {x: %.3f, y: %.3f, z: %.3f, w:%.3f}",
		     goal.header.frame_id.c_str(), goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
		     goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w);

	GetTwoArmPlan::Request req;
	GetTwoArmPlan::Response res;

	req.start.header.frame_id = "map";
	req.start.header.stamp = ros::Time();

	req.rarm_start = rangles_;
	req.larm_start = langles_;

	//TODO:Adjust for map origin offset
	req.body_start.push_back(start.pose.position.x);
	req.body_start.push_back(start.pose.position.y);
	req.body_start.push_back(body_pos_.z);
	//req.body_start.push_back(start.pose.position.z);
	
	tf::Pose tfStart;
	tf::poseMsgToTF(start.pose, tfStart);
	double startRoll, startPitch, startYaw;
	tfStart.getBasis().getRPY(startRoll, startPitch, startYaw);
	req.body_start.push_back(startYaw);
//	req.body_start.push_back(start.pose.orientation.z);
	ROS_INFO("[makePlan] req.rarm_start: (%.3f, %.3f, %.3f, %.3f, %.3f, %.3f)",
	         rangles_[0], rangles_[1], rangles_[2], rangles_[3], rangles_[4], rangles_[5]);
	ROS_INFO("[makePlan] req.larm_start: (%.3f, %.3f, %.3f, %.3f, %.3f, %.3f)",
	         langles_[0], langles_[1], langles_[2], langles_[3], langles_[4], langles_[5]);
	ROS_INFO("[makePlan] req.body_start: (%.3f, %.3f, %.3f, %.3f)",
	         req.body_start[0], req.body_start[1], req.body_start[2], req.body_start[3]);

	req.goal.header.frame_id = "map";
	req.goal.header.stamp = ros::Time();
	req.goal.pose.position.x = goal.pose.position.x;
	req.goal.pose.position.y = goal.pose.position.y;
	req.goal.pose.position.z = goal.pose.position.z;
	req.goal.pose.orientation.x = goal.pose.orientation.x;
	req.goal.pose.orientation.y = goal.pose.orientation.y;
	req.goal.pose.orientation.z = goal.pose.orientation.z;
	req.goal.pose.orientation.w = goal.pose.orientation.w;

	// TODO: make tolerance configurable
	req.absolute_xyzrpy_tolerance.resize(4, 0.02); // xyz tolerance
	req.absolute_xyzrpy_tolerance[3] = 0.12; // rpy tolerance

	req.rarm_object.pose = rarm_object_offset_;
	req.larm_object.pose = larm_object_offset_;

	colmap_mutex_.lock();

	// call the full body planner
	if (planToPosition(req, res)) {
		ROS_INFO("[3dnav] Planner returned.");
		if (res.body_trajectory.points.empty()) {
			ROS_ERROR("[3dnav] Planner returned empty path.");
			colmap_mutex_.unlock();
			return false;
		}
	}
	else {
		ROS_ERROR("Planning service failed to respond. Exiting...");
		colmap_mutex_.unlock();
		return false;
	}

	colmap_mutex_.unlock();

	// convert its returned planned to the plan needed by move_base
	plan.clear();
	for (int i = 0; i < (int)res.body_trajectory.points.size(); i++) {
		geometry_msgs::PoseStamped poseAlongPath;
		poseAlongPath.header.frame_id = "/map";
		poseAlongPath.pose.position.x = res.body_trajectory.points[i].positions[0];
		poseAlongPath.pose.position.y = res.body_trajectory.points[i].positions[1];

		tf::Quaternion temp;
		temp.setEuler(0.0, 0.0, res.body_trajectory.points[i].positions[3]);
//		temp.setEuler(res.body_trajectory.points[i].positions[3], 0.0, 0.0);
		poseAlongPath.pose.orientation.x = temp.x();
		poseAlongPath.pose.orientation.y = temp.y();
		poseAlongPath.pose.orientation.z = temp.z();
		poseAlongPath.pose.orientation.w = temp.w();
//		ROS_INFO("Adding pose {x: %.3f, y: %.3f, theta: %.3f to nav path.",
//		         poseAlongPath.pose.position.x, poseAlongPath.pose.position.y,
//		         res.body_trajectory.points[i].positions[3]);
		plan.push_back(poseAlongPath);
	}

	if (plan.empty()) {
		ROS_INFO("[3dnav] Sbpl3DNavPlanner returned empty path: no move necessary");
	}
	else {
		ROS_INFO("[3dnav] Sbpl3DNavPlanner returned path with %u points.", int(plan.size()));
	}

	return true;
}

void Sbpl3DNavPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
	init();
}

/* Kinematics ----------------------------------------------------------------*/
bool Sbpl3DNavPlanner::computeFK(const std::vector<double> &jnt_pos, std::string arm_name, geometry_msgs::Pose &pose)
{
	kinematics_msgs::GetPositionFK::Request request;
	kinematics_msgs::GetPositionFK::Response response;

	std::string fk_service;
	request.header.stamp = ros::Time();
	request.header.frame_id = reference_frame_;
	request.fk_link_names.resize(1);

	for (size_t j = 0; j < jnt_pos.size(); ++j)
		request.robot_state.joint_state.position.push_back(jnt_pos[j]);

	if (arm_name.compare("right_arm") == 0) {
		request.robot_state.joint_state.name = rjoint_names_;
		request.fk_link_names[0] = "r_wrist_roll_link";
		fk_service = right_fk_service_name_;
	}
	else if (arm_name.compare("left_arm") == 0) {
		request.robot_state.joint_state.name = ljoint_names_;
		request.fk_link_names[0] = "l_wrist_roll_link";
		fk_service = left_fk_service_name_;
	}
	else {
		ROS_ERROR("Invalid arm name. Forward kinematics only supports 'right_arm' and 'left_arm'. (%s)", arm_name.c_str());
		return false;
	}

	ROS_DEBUG("waiting for %s service", fk_service.c_str());
	ros::service::waitForService(fk_service);
	ros::ServiceClient client = root_handle_.serviceClient < kinematics_msgs::GetPositionFK > (fk_service);

	if (client.call(request, response)) {
		if (response.error_code.val == response.error_code.SUCCESS) {
			pose = response.pose_stamped[0].pose;
			return true;
		}
		else
			return false;
	}
	else {
		ROS_ERROR("FK service failed");
		return false;
	}
}

void Sbpl3DNavPlanner::setArmToMapTransform(std::string &map_frame)
{
	std::string fk_root_frame;

	// frame that the sbpl_arm_model is working in
	sbpl_arm_env_.getArmChainRootLinkName(fk_root_frame);

	// get transform to frame that collision map is in
	try {
		tf_.lookupTransform(map_frame, fk_root_frame, ros::Time(0), transform_);

		ROS_DEBUG("Received transform from %s to %s (translation: %f %f %f)", fk_root_frame.c_str(), map_frame.c_str(), transform_.getOrigin().x(), transform_.getOrigin().y(), transform_.getOrigin().z());
	}
	catch (tf::TransformException ex) {
		ROS_ERROR("%s", ex.what());
	}

	// convert transform to a KDL object
	tf::TransformTFToKDL(transform_, kdl_transform_);
	sbpl_arm_env_.setReferenceFrameTransform(kdl_transform_, map_frame);
}

/* Visualizations ------------------------------------------------------------*/
void Sbpl3DNavPlanner::visualizeExpansions()
{
	geometry_msgs::Pose pose;
	std::vector<std::vector<double> > expanded_states;
	std::vector<double> color(4, 1);

	sbpl_arm_env_.getExpandedStates(expanded_states);

	if (!expanded_states.empty()) {
		std::vector<std::vector<double> > detailed_color(2);
		detailed_color[0].resize(4, 0.0);
		detailed_color[0][0] = 1;
		detailed_color[0][3] = 1;

		detailed_color[1].resize(4, 0.0);
		detailed_color[1][1] = 1;
		detailed_color[1][3] = 1;

		laviz_->visualizeDetailedStates(expanded_states, detailed_color, "expansions", 0.01);

		for (size_t i = 0; i < expanded_states.size(); ++i) {
			pose.position.x = expanded_states[i][0];
			pose.position.y = expanded_states[i][1] + 0.01;
			pose.position.z = expanded_states[i][2] + 0.01;
			raviz_->visualizeText(pose, boost::lexical_cast < std::string > (expanded_states[i][11]), "expansions-heuristic", i, color, 0.01);
			usleep(4000);
		}
		ROS_INFO("[3dnav] Displaying %d expanded states", int(expanded_states.size()));

		laviz_->visualizeSphere(expanded_states[0], 250, "expansions-start", 0.015);
	}
	else
		ROS_WARN("[3dnav] No expanded states to display.");
}

void Sbpl3DNavPlanner::visualizeUniqueExpansions()
{
	geometry_msgs::Pose pose;
	double size = 0.15;
	std::vector<std::vector<double> > expanded_states;
	std::vector<double> color(4, 1);
	color[0] = 0.0;

	sbpl_arm_env_.getUniqueExpandedStates(expanded_states);

	if (!expanded_states.empty()) {
		std::vector<std::vector<double> > detailed_color(2);
		detailed_color[0].resize(4, 0.0);
		detailed_color[0][0] = 1;
		detailed_color[0][3] = 1;

		detailed_color[1].resize(4, 0.0);
		detailed_color[1][0] = 1;
		detailed_color[1][3] = 1;

		laviz_->visualizeDetailedStates(expanded_states, detailed_color, "expansions", 0.01);

		for (size_t i = 0; i < expanded_states.size(); ++i) {
			pose.position.x = expanded_states[i][0];
			pose.position.y = expanded_states[i][1] + 0.005;
			pose.position.z = expanded_states[i][2] + 0.0075;
			if (expanded_states[i][3] >= 100)
				size = 0.01;
			else
				size = 0.015;
			raviz_->visualizeText(pose, boost::lexical_cast < std::string > (expanded_states[i][3]), "expansions-heuristic", i, color, size);
			usleep(4000);
		}
		ROS_INFO("[3dnav] Displaying %d expanded states", int(expanded_states.size()));

		//laviz_->visualizeSphere(expanded_states[0], 250, "expansions-start", 0.015);
	}
	else
		ROS_WARN("[3dnav] No expanded states to display.");
}

void Sbpl3DNavPlanner::visualizeGoalPosition(const arm_navigation_msgs::Constraints &goal_pose)
{
	geometry_msgs::Pose pose;
	pose.position = goal_pose.position_constraints[0].position;
	pose.orientation = goal_pose.orientation_constraints[0].orientation;
	laviz_->visualizePose(pose, "goal_pose");
	ROS_DEBUG("[3dnav] publishing goal marker visualizations.");
}

void Sbpl3DNavPlanner::displayShortestPath()
{
	// right arm path
	dpath_ = sbpl_arm_env_.getShortestPath(0);

	if (dpath_.empty()) {
		ROS_INFO("The heuristic path has a length of 0");
		return;
	}
	else
		ROS_DEBUG("Visualizing heuristic path from start to goal with %d waypoints.", int(dpath_.size()));

	raviz_->visualizeSpheres(dpath_, 45, "right_heuristic_path", 0.03);

	// left arm path
	dpath_ = sbpl_arm_env_.getShortestPath(1);

	if (dpath_.empty()) {
		ROS_INFO("The heuristic path has a length of 0");
		return;
	}
	else
		ROS_DEBUG("Visualizing heuristic path from start to goal with %d waypoints.", int(dpath_.size()));

	laviz_->visualizeSpheres(dpath_, 100, "left_heuristic_path", 0.03);

	// object path
	dpath_ = sbpl_arm_env_.getShortestPath(2);

	if (dpath_.empty()) {
		ROS_INFO("The heuristic path has a length of 0");
		return;
	}
	else
		ROS_DEBUG("Visualizing heuristic path from start to goal with %d waypoints.", int(dpath_.size()));

	laviz_->visualizeSpheres(dpath_, 268, "object_heuristic_path", 0.03);
}

void Sbpl3DNavPlanner::visualizeEndEffectorPath()
{
	std::vector<geometry_msgs::Point> points;
	std::vector<std::vector<double> > path, path0, path1;

	sbpl_arm_env_.convertStateIDPathToPoints(solution_state_ids_, path);

	points.resize(path.size());

	for (size_t i = 0; i < path.size(); ++i) {
		if (path[i][3] == 1)
			path1.push_back(path[i]);
		else
			path0.push_back(path[i]);

		points[i].x = path[i][0];
		points[i].y = path[i][1];
		points[i].z = path[i][2];
	}

	ROS_DEBUG("[viz] path0: %d, path1: %d", int(path0.size()), int(path1.size()));
	laviz_->visualizeSpheres(path0, 230, "end_effector_path0", 0.01);
	laviz_->visualizeSpheres(path1, 30, "end_effector_path1", 0.01);
	laviz_->visualizeLine(points, "end_effector_path", 0, 120, 0.005);
}

void Sbpl3DNavPlanner::visualizeObjectPath()
{
	std::vector<geometry_msgs::Point> points;
	std::vector<std::vector<double> > path;

	sbpl_arm_env_.convertStateIDPathToPoints(solution_state_ids_, path);

	points.resize(path.size());

	for (size_t i = 0; i < path.size(); ++i) {
		points[i].x = path[i][0];
		points[i].y = path[i][1];
		points[i].z = path[i][2];
	}

	ROS_INFO("[3dnav] path length: %d", int(path.size()));
	laviz_->visualizeSpheres(path, 0, "object_path", 0.015);
	usleep(50000);
	laviz_->visualizeLine(points, "object_path_line", 0, 120, 0.005);
}

void Sbpl3DNavPlanner::visualizeExpansionsPerHValue()
{
	int last_dist = 0, dist = 0, total_num_hvals = 0;
	std::vector<std::vector<double> > path;
	std::string text = "";
	std::vector<double> color(4, 1.0);
	std::vector<int> hvals, num_hvals;
	geometry_msgs::Pose pose;
	pose.orientation.x = 0;
	pose.orientation.y = 0;
	pose.orientation.z = 0;
	pose.orientation.w = 1;

	sbpl_arm_env_.convertStateIDPathToPoints(solution_state_ids_, path);

	sbpl_arm_env_.getHeuristicDebugStats(hvals, num_hvals);

	if (hvals.size() != num_hvals.size())
		ROS_WARN("Heuristic debugging information doesn't make sense.");
	for (size_t i = 0; i < hvals.size(); ++i)
		ROS_DEBUG("[%d] h-val: %d  # exp: %d", int(i), hvals[i], num_hvals[i]);

	for (size_t i = 0; i < path.size(); ++i) {
		dist = sbpl_arm_env_.getDijkstraDistance(path[i][0], path[i][1], path[i][2]);

		if (dist == last_dist) {
			ROS_DEBUG("[%d] dist: %d last_dist: %d CONTINUE", int(i), dist, last_dist);
			continue;
		}
		ROS_DEBUG("[%d] dist: %d last_dist: %d", int(i), dist, last_dist);

		//text = "";
		total_num_hvals = 0;
		for (size_t j = 0; j < (hvals.size() / 2); ++j) {
			if (hvals[j] == dist) {
				total_num_hvals += num_hvals[j];
				/*
				 if(text == "")
				 text = boost::lexical_cast<std::string>(num_hvals[j]);
				 else
				 text = text + "," + boost::lexical_cast<std::string>(num_hvals[j]);
				 */
			}
		}

		if (total_num_hvals == 0)
			total_num_hvals = 1;

		text = boost::lexical_cast < std::string > (total_num_hvals);
		pose.position.x = path[i][0];
		pose.position.y = path[i][1] - 0.02;
		pose.position.z = path[i][2];

		raviz_->visualizeText(pose, text, "num_expansions_per_hval", i, color, 0.03);
		last_dist = dist;
	}
}

void Sbpl3DNavPlanner::visualizeHeuristicInfo()
{
	int dist = 0;
	std::string text = "";
	std::vector<double> color(4, 1.0);
	std::vector<int> hvals, num_hvals;
	geometry_msgs::Pose pose;
	pose.orientation.x = 0;
	pose.orientation.y = 0;
	pose.orientation.z = 0;
	pose.orientation.w = 1;

	sbpl_arm_env_.getHeuristicDebugStats(hvals, num_hvals);

	if (hvals.size() != num_hvals.size())
		ROS_WARN("[3dnav] Heuristic debugging information doesn't make sense.");

	ROS_DEBUG("\b Number of expansions per H-value");
	for (size_t i = 0; i < hvals.size(); ++i)
		ROS_DEBUG("[%d] h-val: %d  # exp: %d", int(i), hvals[i], num_hvals[i]);

	dpath0_.clear();
	dpath0_ = sbpl_arm_env_.getShortestPath(0);

	//check if the list is empty
	if (dpath0_.empty()) {
		ROS_INFO("[3dnav] The heuristic path has a length of 0");
		return;
	}
	else
		ROS_INFO("node] Visualizing right arm heuristic path from start to goal with %d waypoints.", int(dpath0_.size()));

	raviz_->visualizeSpheres(dpath0_, 45, "right_arm_heuristic_path", 0.04);

	ROS_DEBUG("[3dnav] Right Arm Heuristic Visualization:");
	for (size_t i = 0; i < dpath0_.size(); ++i) {
		dist = sbpl_arm_env_.getDijkstraDistance(dpath0_[i][0], dpath0_[i][1], dpath0_[i][2]);
		text = "";
		for (size_t j = 0; j < (hvals.size() / 2); ++j) {
			if (hvals[j] == dist) {
				if (text == "")
					text = boost::lexical_cast < std::string > (num_hvals[j]);
				else
					text = text + "," + boost::lexical_cast < std::string > (num_hvals[j]);
			}
		}
		pose.position.x = dpath0_[i][0];
		pose.position.y = dpath0_[i][1];
		pose.position.z = dpath0_[i][2];
		raviz_->visualizeText(pose, text, "right_arm_heuristic_text", i, color, 0.03);
		ROS_DEBUG("[%d] Distance: %d  Text: %s  Pose: %0.3f %0.3f %0.3f", int(i), dist, text.c_str(), dpath0_[i][0], dpath0_[i][1], dpath0_[i][2]);
	}

	//get the heuristic for the left arm path
	dpath1_.clear();
	dpath1_ = sbpl_arm_env_.getShortestPath(1);

	//check if the list is empty
	if (dpath1_.empty()) {
		ROS_INFO("The heuristic path has a length of 0");
		return;
	}
	else
		ROS_INFO("Visualizing left arm heuristic path from start to goal with %d waypoints.", int(dpath1_.size()));

	//visualize the path
	laviz_->visualizeSpheres(dpath1_, 100, "left_arm_heuristic_path", 0.04);

	//visualize the # expansions for each heuristic value
	ROS_DEBUG("Left Arm Heuristic Visualization:");
	for (size_t i = 0; i < dpath1_.size(); ++i) {
		dist = sbpl_arm_env_.getDijkstraDistance(dpath1_[i][0], dpath1_[i][1], dpath1_[i][2]);
		text = "";
		for (size_t j = 0; j < (hvals.size() / 2); ++j) {
			if (hvals[j] == dist) {
				if (text == "")
					text = boost::lexical_cast < std::string > (num_hvals[j]);
				else
					text = text + "," + boost::lexical_cast < std::string > (num_hvals[j]);
			}
			if (dist == 1000000000)
				text = "impossible";
		}
		pose.position.x = dpath1_[i][0];
		pose.position.y = dpath1_[i][1];
		pose.position.z = dpath1_[i][2];
		laviz_->visualizeText(pose, text, "left_arm_heuristic_text", i, color, 0.03);
		ROS_DEBUG("[%d] Distance: %d  Text: %s  Pose: %0.3f %0.3f %0.3f", int(i), dist, text.c_str(), dpath1_[i][0], dpath1_[i][1], dpath1_[i][2]);
	}
}

void Sbpl3DNavPlanner::visualizeGoal(geometry_msgs::Pose goal)
{
  pviz_.visualizePose(goal, "3dnav_goal");
}

void Sbpl3DNavPlanner::visualizeCollisionObjects(bool delete_first)
{
	std::vector<geometry_msgs::Pose> poses;
	std::vector<std::vector<double> > points(1, std::vector<double>(3, 0));
	std::vector<double> color(4, 1);
	color[2] = 0;

  // first delete old visualizations
  if(delete_first)
    pviz_.deleteVisualizations("known_objects", 1000);
  
	cspace_->getCollisionObjectVoxelPoses(poses);

	points.resize(poses.size());
	for (size_t i = 0; i < poses.size(); ++i) {
		points[i].resize(3);
		points[i][0] = poses[i].position.x;
		points[i][1] = poses[i].position.y;
		points[i][2] = poses[i].position.z;
	}

	ROS_DEBUG("[3dnav] Displaying %d known collision object voxels.", int(points.size()));
	laviz_->visualizeBasicStates(points, color, "known_objects", 0.01);
}

void Sbpl3DNavPlanner::visualizeAttachedObject(bool delete_first)
{
	std::vector<std::vector<double> > spheres;
	std::vector<double> color(4, 1);
	std::vector<double> radius;
	color[2] = 0;

  // first delete old visualizations
  if(delete_first)
    pviz_.deleteVisualizations("attached_objects", 500);

  cspace_->getAttachedObjectSpheres(langles_, rangles_, body_pos_, spheres); 
  
  if(spheres.empty())
    return;

	ROS_DEBUG("[3dnav] Displaying %d spheres for the attached object.", int(spheres.size()));
	for (size_t i = 0; i < spheres.size(); ++i)
    ROS_DEBUG("[%d] xyz: %0.3f %0.3f %0.3f radius: %0.3f", int(i), spheres[i][0], spheres[i][1] , spheres[i][2], spheres[i][3]);

  pviz_.visualizeSpheres(spheres, 163, "attached_objects");
}

void Sbpl3DNavPlanner::printPath(const std::vector<trajectory_msgs::JointTrajectoryPoint> &rpath, const std::vector<trajectory_msgs::JointTrajectoryPoint> &lpath, const std::vector<trajectory_msgs::JointTrajectoryPoint> &bpath)
{
	double roll, pitch, yaw;
	tf::Pose tf_pose;
	geometry_msgs::Pose pose;
	geometry_msgs::PoseStamped pose_in, pose_out;
	std::vector<double> jnt_pos(num_joints_, 0);
	ROS_INFO("Right Arm Path:");
	for (size_t i = 0; i < rpath.size(); i++) {
		for (int j = 0; j < num_joints_; ++j)
			jnt_pos[j] = rpath[i].positions[j];

		computeFK(jnt_pos, "right_arm", pose);
		tf::poseMsgToTF(pose, tf_pose);
		tf_pose.getBasis().getRPY(roll, pitch, yaw);

		ROS_INFO("%3d: %1.4f %1.4f %1.4f %1.4f %1.4f %1.4f %1.4f\n   xyz: %2.3f %2.3f %2.3f  rpy: %0.3f %0.3f %0.3f  quat: %0.2f %0.2f %0.2f %0.2f", int(i), rpath[i].positions[0], rpath[i].positions[1], rpath[i].positions[2], rpath[i].positions[3], rpath[i].positions[4], rpath[i].positions[5], rpath[i].positions[6], pose.position.x, pose.position.y, pose.position.z, roll, pitch, yaw, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
	}

	ROS_INFO("Left Arm Path:");
	for (size_t i = 0; i < lpath.size(); i++) {
		for (int j = 0; j < num_joints_; ++j)
			jnt_pos[j] = lpath[i].positions[j];

		computeFK(jnt_pos, "left_arm", pose);
		tf::poseMsgToTF(pose, tf_pose);
		tf_pose.getBasis().getRPY(roll, pitch, yaw);

		ROS_INFO("%3d: %1.4f %1.4f %1.4f %1.4f %1.4f %1.4f %1.4f\n   xyz: %2.3f %2.3f %2.3f  rpy: %0.3f %0.3f %0.3f  quat: %0.2f %0.2f %0.2f %0.2f", int(i), lpath[i].positions[0], lpath[i].positions[1], lpath[i].positions[2], lpath[i].positions[3], lpath[i].positions[4], lpath[i].positions[5], lpath[i].positions[6], pose.position.x, pose.position.y, pose.position.z, roll, pitch, yaw, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
	}
	ROS_INFO("Body Path:");
	for (size_t i = 0; i < bpath.size(); i++) {
		ROS_INFO("%3d: x: %0.3f  y: %0.3f  torso: %2.3f  theta: %0.3f", int(i), bpath[i].positions[0], bpath[i].positions[1], bpath[i].positions[2], bpath[i].positions[3]);
	}
}

void Sbpl3DNavPlanner::printPath(FILE* fOut, const std::vector<std::vector<double> > path)
{
	time_t init_time;
	time(&init_time);
	std::string str_time(asctime(localtime(&init_time)));

	fprintf(fOut, "%s", str_time.c_str());
	for (unsigned int i = 0; i < path.size(); i++)
		fprintf(fOut, "state %3d: %2.3f %2.3f %2.3f %2.3f %2.3f %2.3f %2.3f", i, path[i][0], path[i][1], path[i][2], path[i][3], path[i][4], path[i][5], path[i][6]);
	fprintf(fOut, "---------------------------------");
}

void Sbpl3DNavPlanner::visualizeHeuristicGrid()
{
	bool indent = false, x_indent = false;
	;
	int dist = 0, cntr = 0;
	double y_left = 0, x_left = 0;
	std::vector<double> color(4), blue(4, 0.0), white(4, 1.0), red(4, 0.0), orange(4, 0.0);
	red[0] = 1.0;
	red[3] = 1.0;
	blue[2] = 0.5;
	blue[3] = 1.0;
	orange[0] = 0.5;
	orange[1] = 0.5;
	orange[3] = 1.0;
	std::vector<double> goal(3);
	geometry_msgs::Pose pose;
	pose.orientation.x = 0;
	pose.orientation.y = 0;
	pose.orientation.z = 0;
	pose.orientation.w = 1;

	for (double z = z_min_; z < z_max_; z += z_inc_) {
		y_left = y_min_;
		//x_left = x_min_;
		if (indent) {
			y_left = y_min_ - (y_inc_ / 2.0);
			//x_left = x_min_ - (x_inc_/2.0);
			//ROS_INFO("Indent x: %f y: %f", x_left, y_left);
			color = blue;
			indent = false;
		}
		else {
			color = white;
			//ROS_INFO("Don't Indent");
			indent = true;
		}

		for (double y = y_left; y < y_max_; y += y_inc_) {
			x_left = x_min_;
			if (x_indent) {
				x_left = x_min_ - (x_inc_ / 2.0);
				x_indent = false;
				color = orange;
			}
			else {
				x_indent = true;
				color = blue;
			}

			for (double x = x_left; x < x_max_; x += x_inc_) {
				dist = sbpl_arm_env_.getDijkstraDistance(x, y, z);

				if (dist == 0) {
					goal[0] = x;
					goal[1] = y;
					goal[2] = z;
					raviz_->visualizeSphere(goal, 160, "heuristic_grid_goal-" + boost::lexical_cast < std::string > (cntr), 0.015);
				}
				else if (dist < 10000) {
					pose.position.x = x;
					pose.position.y = y;
					pose.position.z = z;
					raviz_->visualizeText(pose, boost::lexical_cast < std::string > (dist), "heuristic_grid", cntr, color, 0.015);
				}
				else {
					pose.position.x = x;
					pose.position.y = y;
					pose.position.z = z;
					raviz_->visualizeText(pose, "x", "heuristic_grid", cntr, red, 0.02);
				}

				usleep(5000);
				cntr++;
			}
		}
	}
}

void Sbpl3DNavPlanner::visualizeCollisionObject(const arm_navigation_msgs::CollisionObject &object)
{
	geometry_msgs::PoseStamped pose;
	std::vector<double> dim, color(4, 0.0);
  color[1] = 1.0;
  color[2] = 1.0;
  color[3] = 0.8;

	for (size_t i = 0; i < object.shapes.size(); ++i) {
		if (object.shapes[i].type == arm_navigation_msgs::Shape::BOX) {
			pose.pose = object.poses[i];
			pose.header = object.header;
			dim.resize(object.shapes[i].dimensions.size());
			for (size_t j = 0; j < object.shapes[i].dimensions.size(); ++j)
				dim[j] = object.shapes[i].dimensions[j];

			ROS_INFO("[3dnav] Visualizing %s", object.id.c_str());
			raviz_->visualizeCube(pose, color, "3dnav_" + object.id, int(i), dim);
		}
		else
			ROS_DEBUG("[3dnav] Visualizations of collision objects of type %d are not yet supported.", object.shapes[i].type);
	}
}

void Sbpl3DNavPlanner::changeLoggerLevel(std::string name, std::string level)
{
	//ROSCONSOLE_AUTOINIT;

	std::string logger_name = ROSCONSOLE_DEFAULT_NAME + std::string(".") + name;

	ROS_INFO("[3dnav] Setting %s to %s level", logger_name.c_str(), level.c_str());

	log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(logger_name);

	// Set the logger for this package to output all statements
	if (level.compare("debug") == 0)
		my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
	else
		my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Info]);

	ROS_DEBUG("This is a debug statement, and should print if you enabled debug.");
}

void Sbpl3DNavPlanner::getRobotState(BodyPose &body_pos, std::vector<double> &rangles, std::vector<double> &langles)
{
	sensor_msgs::JointStateConstPtr state = ros::topic::waitForMessage < sensor_msgs::JointState > ("joint_states");
	rangles[0] = state->position[17];
	rangles[1] = state->position[18];
	rangles[2] = state->position[16];
	rangles[3] = state->position[20];
	rangles[4] = state->position[19];
	rangles[5] = state->position[21];
	rangles[6] = state->position[22];

	langles[0] = state->position[29];
	langles[1] = state->position[30];
	langles[2] = state->position[28];
	langles[3] = state->position[32];
	langles[4] = state->position[31];
	langles[5] = state->position[33];
	langles[6] = state->position[34];
	body_pos_.z = state->position[12];

	try {
		tf_.lookupTransform("map", "base_footprint", ros::Time(0), base_map_transform_);
		body_pos.x = base_map_transform_.getOrigin().x();
		body_pos.y = base_map_transform_.getOrigin().y();
		body_pos_.theta = 2 * atan2(base_map_transform_.getRotation().getX(), base_map_transform_.getRotation().getW());
	}
	catch (tf::TransformException& ex) {
		ROS_ERROR("Is there a map? The map-robot transform failed. (%s)", ex.what());
	}
}

void Sbpl3DNavPlanner::visualizeTrajectory(std::vector<trajectory_msgs::JointTrajectoryPoint> &rpath, std::vector<trajectory_msgs::JointTrajectoryPoint> &lpath, std::vector<trajectory_msgs::JointTrajectoryPoint> &bpath)
{
	int length = rpath.size();
	std::vector<double> rangles(7, 0), langles(7, 0);
	BodyPose body_pos;

	if (rpath.size() != lpath.size() || rpath.size() != bpath.size()) {
		ROS_ERROR("[3dnav] The right arm, left arm and body trajectories are of unequal lengths.");
		return;
	}

	int color_inc = 240.0 / (length / throttle_); // hue: red -> blue
	ROS_DEBUG("[3dnav] length: %d color_inc: %d throttle: %d)", length, color_inc, throttle_);

	for (int i = 0; i < length; ++i) {
		for (int j = 0; j < 7; ++j) {
			rangles[j] = rpath[i].positions[j];
			langles[j] = lpath[i].positions[j];
		}
		body_pos.x = bpath[i].positions[0];
		body_pos.y = bpath[i].positions[1];
		body_pos.z = bpath[i].positions[2];
		body_pos.theta = bpath[i].positions[3];

		if ((i != length - 1) && (i % throttle_ != 0))
			continue;

		rangles_ = rangles;
		visualizeAttachedObject();

		ROS_DEBUG("[3dnav] length: %d color_inc: %d throttle: %d", length, color_inc, throttle_);
		ROS_DEBUG("[3dnav] Visualizing waypoint #%d (i mod color_inc: %d) with color: %d (color_inc: %d, throttle: %d)", i, (i / throttle_), (i / throttle_) * color_inc, color_inc, throttle_);

		pviz_.visualizeRobot(rangles, langles, body_pos, (i / throttle_) * color_inc, "waypoint_" + boost::lexical_cast < std::string > (i), i);
		usleep(3000);
	}
}

void Sbpl3DNavPlanner::printRobotState(std::vector<double> &rangles, std::vector<double> &langles, BodyPose &body_pos, std::string text)
{
	ROS_INFO("robot state:  %s", text.c_str());
	ROS_INFO("     x: %0.3f  y: % 0.3f  z: %0.3f yaw: % 0.3f", body_pos.x, body_pos.y, body_pos.z, body_pos.theta);
	ROS_INFO(" right: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", rangles[0], rangles[1], rangles[2], rangles[3], rangles[4], rangles[5], rangles[6]);
	ROS_INFO("  left: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", langles[0], langles[1], langles[2], langles[3], langles[4], langles[5], langles[6]);
}

void Sbpl3DNavPlanner::visualizeCollisionModel(std::vector<double> &rangles, std::vector<double> &langles, BodyPose &body_pos, std::string text)
{
  std::vector<std::vector<double> > spheres, all_spheres;
  std::vector<int> hues, all_hues;
  std::string frame;
/*
  frame = pviz_.getReferenceFrame();
  pviz_.setReferenceFrame("map");
  cspace_->getCollisionSpheres(langles, rangles, body_pos, "base", spheres);
  pviz_.visualizeSpheres(spheres, 10, text+"-base");
  cspace_->getCollisionSpheres(langles, rangles, body_pos, "torso_upper", spheres);
  pviz_.visualizeSpheres(spheres, 30, text+"-torso_upper");
  cspace_->getCollisionSpheres(langles, rangles, body_pos, "torso_lower", spheres);
  pviz_.visualizeSpheres(spheres, 50, text+"-torso_lower");
  cspace_->getCollisionSpheres(langles, rangles, body_pos, "turrets", spheres);
  pviz_.visualizeSpheres(spheres, 70, text+"-turrets");
  cspace_->getCollisionSpheres(langles, rangles, body_pos, "tilt_laser", spheres);
  pviz_.visualizeSpheres(spheres, 90, text+"-tilt_laser");
  cspace_->getCollisionSpheres(langles, rangles, body_pos, "head", spheres);
  pviz_.visualizeSpheres(spheres, 110, text+"-head");
  cspace_->getCollisionSpheres(langles, rangles, body_pos, "left_gripper", spheres);
  pviz_.visualizeSpheres(spheres, 130, text+"-left_gripper");
  cspace_->getCollisionSpheres(langles, rangles, body_pos, "right_gripper", spheres);
  pviz_.visualizeSpheres(spheres, 150, text+"-right_gripper");
  cspace_->getCollisionSpheres(langles, rangles, body_pos, "left_forearm", spheres);
  pviz_.visualizeSpheres(spheres, 170, text+"-left_forearm");
  cspace_->getCollisionSpheres(langles, rangles, body_pos, "right_forearm", spheres);
  pviz_.visualizeSpheres(spheres, 190, text+"-right_forearm");
  pviz_.setReferenceFrame(frame);
*/
  std::vector<std::string> sphere_groups(10);
  sphere_groups[0] = "base"; 
  sphere_groups[1] = "torso_upper";
  sphere_groups[2] = "torso_lower";
  sphere_groups[3] = "turrets";
  sphere_groups[4] = "tilt_laser";
  sphere_groups[5] = "head";
  sphere_groups[6] = "left_gripper";
  sphere_groups[7] = "right_gripper";
  sphere_groups[8] = "left_forearm";
  sphere_groups[9] = "right_forearm";

  for(std::size_t i = 0; i < sphere_groups.size(); ++i)
  {
    cspace_->getCollisionSpheres(langles, rangles, body_pos, sphere_groups[i], spheres); 
    all_spheres.insert(all_spheres.end(), spheres.begin(), spheres.end());
    hues.clear();
    hues.resize(spheres.size(), (i+1)*20);
    all_hues.insert(all_hues.end(), hues.begin(), hues.end());
  }
  pviz_.visualizeSpheres(all_spheres, all_hues, text);  
}


//XXX: IMPORTANT: pose_stamped_msg should be in the MAP frame!!!!
bool 
Sbpl3DNavPlanner::checkIK( const geometry_msgs::PoseStamped &pose_stamped_msg,
									const std::string &link_name,
									arm_navigation_msgs::RobotState &rs )
{
	double ik_allowed_time = 2.0;

   kinematics_msgs::GetConstraintAwarePositionIK::Request request;
   kinematics_msgs::GetConstraintAwarePositionIK::Response response;
	//kinematics_msgs::GetPositionIK::Request  request;
	//kinematics_msgs::GetPositionIK::Response response;
   request.ik_request.pose_stamped = pose_stamped_msg;
   request.ik_request.robot_state = rs;
   request.ik_request.ik_seed_state = request.ik_request.robot_state;
   request.ik_request.ik_link_name = link_name;
   request.timeout = ros::Duration(ik_allowed_time);	   

   // constraint around the imaginary pose
   arm_navigation_msgs::Constraints gc;
   arm_navigation_msgs::PositionConstraint pc;
   arm_navigation_msgs::OrientationConstraint oc;
   
   pc.header = pose_stamped_msg.header;
   pc.position = pose_stamped_msg.pose.position;
   pc.link_name = link_name;
   pc.constraint_region_shape.type = pc.constraint_region_shape.BOX;
   pc.constraint_region_shape.dimensions.push_back(0.01);
   pc.constraint_region_shape.dimensions.push_back(0.01);
   pc.constraint_region_shape.dimensions.push_back(0.01);
   pc.constraint_region_orientation.w = 1;
   pc.weight = 1.0;
   
   oc.header = pose_stamped_msg.header;
   oc.link_name = link_name;
   oc.type = oc.HEADER_FRAME;
   oc.orientation = pose_stamped_msg.pose.orientation;
   oc.absolute_roll_tolerance = 0.1;
   oc.absolute_pitch_tolerance = 0.1;
   oc.absolute_yaw_tolerance = 0.1;
   oc.weight = 1.0;
   
   /*
   arm_navigation_msgs::poseStampedToPositionOrientationConstraints(
   			pose_stamped_msg, link_name, pc, oc, 0.01, 0.1);
   */
   gc.position_constraints.push_back(pc);
   gc.orientation_constraints.push_back(oc);
   request.constraints = gc;
		
	/**/	 
	 /* 
	ROS_INFO("checkIK request");
	ROS_INFO("Frame: %s", pose_stamped_msg.header.frame_id.c_str());
	ROS_INFO("(x = %f, y = %f, z = %f)", pose_stamped_msg.pose.position.x,
													 pose_stamped_msg.pose.position.y,
													 pose_stamped_msg.pose.position.z );
   ROS_INFO("pc Frame: %s", pc.header.frame_id.c_str());
   ROS_INFO("pc link_name: %s", pc.link_name.c_str());
	ROS_INFO("(x = %f, y = %f, z = %f)", pc.position.x,
													 pc.position.y,
													 pc.position.z );   
   
   ROS_INFO("oc Frame: %s", oc.header.frame_id.c_str());
   ROS_INFO("oc link_name: %s", oc.link_name.c_str()); 	
	*/
	

	if (ik_client_.call(request, response))
	{
		if(response.error_code.val != response.error_code.SUCCESS)
		{
			ROS_ERROR("[3dnav] IK Solution not found, IK returned with error_code: %d", response.error_code.val);
			return false;
		}
	}
	else
	{
		ROS_ERROR("[3dnav] IK service call failed");
		return false;
	}
	return true;

	
	/* CONSTRAINT AWARE IK 
	arm_navigation_msgs::SetPlanningSceneDiff::Request planning_scene_req;
  	arm_navigation_msgs::SetPlanningSceneDiff::Response planning_scene_res;
  
   if(!set_planning_scene_diff_client_.call(planning_scene_req, planning_scene_res)) {
   	ROS_ERROR("Can't get planning scene");
   	return false;
  	}

	kinematics_msgs::GetConstraintAwarePositionIK::Request request;
	kinematics_msgs::GetConstraintAwarePositionIK::Response response;
	
	request.timeout = ros::Duration(ik_allowed_time);
	request.ik_request.ik_link_name = link_name;
	
	request.ik_request.pose_stamped = pose_stamped_msg;
	
	request.ik_request.robot_state = rs;
	request.ik_request.ik_seed_state = request.ik_request.robot_state;
	arm_navigation_msgs::PositionConstraint pc;
	arm_navigation_msgs::OrientationConstraint oc;
	
	pose_stamped_to_position_orientation_constraints( pose_stamped_msg, link_name, pc, oc );
	request.constraints.position_constraints.push_back(pc);
	request.constraints.orientation_constraints.push_back(oc);
	*/
	
	/* Seeding with current joint states 
	kinematics_msgs::GetKinematicSolverInfo::Request req_info;
  	kinematics_msgs::GetKinematicSolverInfo::Response res_info;
	if( !ik_query_client_.call(req_info, res_info) )
	{
		ROS_ERROR("Could not call ik_query_client_ service");
		return false;
	}
	
	request.ik_request.ik_seed_state.joint_state.position.resize(res_info.kinematic_solver_info.joint_names.size());
  	request.ik_request.ik_seed_state.joint_state.name = res_info.kinematic_solver_info.joint_names;
  	for(unsigned int i=0; i< res_info.kinematic_solver_info.joint_names.size(); i++)
  	{
   	request.ik_request.ik_seed_state.joint_state.position[i] = (res_info.kinematic_solver_info.limits[i].min_position + res_info.kinematic_solver_info.limits[i].max_position)/2.0;
  	}
  	*/
  	

	/* Get the current robot state!
	const arm_navigation_msgs::PlanningScene planning_diff;
   const arm_navigation_msgs::OrderedCollisionOperations operations;
	if(!getAndSetPlanningScene(planning_diff, operations)){
      ROS_ERROR("Problem setting planning scene");
      return false;
   }
   
   arm_navigation_msgs::RobotState rs_new;
   planning_environment::convertKinematicStateToRobotState(*planning_scene_state_,
                                                            ros::Time::now(),
                                                            collision_models_->getWorldFrameId(),
                                                            rs_new);                              
	request.ik_request.robot_state = rs_new;
	request.ik_request.ik_seed_state = request.ik_request.robot_state;

   collision_models_->convertConstraintsGivenNewWorldTransform(*planning_scene_state_, request.constraints);
   */
      
    /* DISPLAY 
    ROS_INFO("FAKE:");
	 ROS_INFO_STREAM("pose_stamped = " << pose_stamped_msg.header.frame_id << " " 
	 											  << pose_stamped_msg.pose.position.x << " " 
	 											  << pose_stamped_msg.pose.position.y << " " 
	 											  << pose_stamped_msg.pose.position.z << " " 
	 											  << pose_stamped_msg.pose.orientation.x << " " 
	 											  << pose_stamped_msg.pose.orientation.y << " " 
	 											  << pose_stamped_msg.pose.orientation.z << " " 
	 											  << pose_stamped_msg.pose.orientation.w );
	 	
	 ROS_INFO("Number of position constraints: %d", request.constraints.position_constraints.size());
	 for(int i = 0; i < request.constraints.position_constraints.size(); ++i){
	 	ROS_INFO_STREAM("poition_constraint" << i 
	 			<< " = " << request.constraints.position_constraints[i].position.x
	 						<< request.constraints.position_constraints[i].position.y
	 						<< request.constraints.position_constraints[i].position.z );
	 	ROS_INFO_STREAM("poition_constraint link_name = " << request.constraints.position_constraints[i].link_name);
	 }
	 
	 ROS_INFO("Number of orinetation constraints: %d", request.constraints.orientation_constraints.size());
	 for(int i = 0; i < request.constraints.orientation_constraints.size(); ++i){
	 	ROS_INFO_STREAM("orientation_constraint" << i 
	 			<< " = " << request.constraints.orientation_constraints[i].orientation.x
	 						<< request.constraints.orientation_constraints[i].orientation.y
	 						<< request.constraints.orientation_constraints[i].orientation.z
	 						<< request.constraints.orientation_constraints[i].orientation.w );
	 	ROS_INFO_STREAM("orientation_constraint link_name = " << request.constraints.orientation_constraints[i].link_name);
	 }
	ROS_INFO("END FAKE:");
   */
}

/*
void 
Sbpl3DNavPlanner::pose_stamped_to_position_orientation_constraints( const geometry_msgs::PoseStamped &pose_stamped_msg,
																						  const std::string &link_name,
																						  arm_navigation_msgs::PositionConstraint &pc,
																						  arm_navigation_msgs::OrientationConstraint &oc )
{
	pc.header = pose_stamped_msg.header;
	pc.position = pose_stamped_msg.pose.position;
	pc.link_name = link_name;
	pc.constraint_region_shape.type = pc.constraint_region_shape.BOX;
	pc.constraint_region_shape.dimensions.push_back(0.01);
	pc.constraint_region_shape.dimensions.push_back(0.01);
	pc.constraint_region_shape.dimensions.push_back(0.01);
	pc.constraint_region_orientation.w = 1;
	pc.weight = 1.0;
	
	oc.header = pose_stamped_msg.header;
	oc.link_name = link_name;
	oc.type = oc.HEADER_FRAME;
	oc.orientation = pose_stamped_msg.pose.orientation;
	oc.absolute_roll_tolerance = 0.1;
	oc.absolute_pitch_tolerance = 0.1;
	oc.absolute_yaw_tolerance = 0.1;
	oc.weight = 1.0;
}
*/

/*
bool 
Sbpl3DNavPlanner::getAndSetPlanningScene(const arm_navigation_msgs::PlanningScene& planning_diff,
                              				const arm_navigation_msgs::OrderedCollisionOperations& operations) 
{
	arm_navigation_msgs::SetPlanningSceneDiff::Request planning_scene_req;
	arm_navigation_msgs::SetPlanningSceneDiff::Response planning_scene_res;

	if(planning_scene_state_ != NULL) {
		collision_models_->revertPlanningScene(planning_scene_state_);
		planning_scene_state_ = NULL;
	}

	planning_scene_req.operations = operations;
	planning_scene_req.planning_scene_diff = planning_diff;

	ROS_DEBUG_STREAM("Getting and setting planning scene");

	if(!set_planning_scene_diff_client_.call(planning_scene_req, planning_scene_res)) {
		ROS_WARN("Can't get planning scene");
		return false;
	}

	planning_scene_state_ = collision_models_->setPlanningScene(planning_scene_res.planning_scene);

	collision_models_->disableCollisionsForNonUpdatedLinks("right_arm");

	if(planning_scene_state_ == NULL) {
		ROS_WARN("Problems setting local state");
		return false;
	}
	return true;
}
*/

}

/* Node --------------------------------------------------------------------------- */
int main(int argc, char **argv)
{
	ROSCONSOLE_AUTOINIT;
	ros::init(argc, argv, "sbpl_full_body_planner");
	sbpl_3dnav_planner::Sbpl3DNavPlanner arm_planner;
	if (!arm_planner.init()) {
		ROS_ERROR("Failed to initialize arm planner node. Exiting.");
		return 0;
	}

	return arm_planner.run();
}

