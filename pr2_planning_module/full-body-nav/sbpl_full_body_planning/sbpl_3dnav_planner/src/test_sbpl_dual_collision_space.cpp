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

#include <sbpl_two_arm_planner_node/test_sbpl_dual_collision_space.h>

clock_t starttime;

using namespace std;

/** Initializers -------------------------------------------------------------*/
TestSBPLCollisionSpace::TestSBPLCollisionSpace() : node_handle_("~"),collision_map_subscriber_(root_handle_,"collision_map_occ",1), collision_map_filter_(NULL),grid_(NULL),laviz_(NULL),raviz_(NULL)
{
  langles_.resize(7,0);
  rangles_.resize(7,0);

  ljoint_names_.resize(7);
  rjoint_names_.resize(7);
}

TestSBPLCollisionSpace::~TestSBPLCollisionSpace()
{
  if(laviz_ != NULL)
    delete laviz_;
  if(raviz_ != NULL)
    delete raviz_;
  if(larm_ != NULL)
    delete larm_;
  if(rarm_ != NULL)
    delete rarm_;
  if(grid_ != NULL)
    delete grid_;

  if(collision_map_filter_ != NULL)
    delete collision_map_filter_;
}

bool TestSBPLCollisionSpace::init()
{
  //planner
  node_handle_.param<std::string>("planner/left_arm_description_file", left_arm_description_filename_, "");
  node_handle_.param<std::string>("planner/right_arm_description_file", right_arm_description_filename_, "");
  node_handle_.param<std::string>("reference_frame", reference_frame_, std::string("base_link"));
  node_handle_.param<std::string>("left_fk_service_name", left_fk_service_name_, "pr2_left_arm_kinematics/get_fk");
  node_handle_.param<std::string>("left_ik_service_name", left_ik_service_name_, "pr2_left_arm_kinematics/get_ik");
  node_handle_.param<std::string>("right_fk_service_name", right_fk_service_name_, "pr2_right_arm_kinematics/get_fk");
  node_handle_.param<std::string>("right_ik_service_name", right_fk_service_name_, "pr2_right_arm_kinematics/get_ik");

  node_handle_.param<std::string>("robot/arm_name", arm_name_, "right_arm");
  node_handle_.param ("robot/num_joints", num_joints_, 7);
  
  //pr2 specific
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
  
  //collision space
  node_handle_.param<std::string>("collision_space/collision_map_topic", collision_map_topic_, "collision_map_occ");

  //visualizations
  node_handle_.param ("visualizations/goal", visualize_goal_, true);
  node_handle_.param ("visualizations/expanded_states",visualize_expanded_states_,true);
  node_handle_.param ("visualizations/heuristic", visualize_heuristic_, true);
  node_handle_.param ("visualizations/trajectory", visualize_trajectory_, false);
  node_handle_.param ("visualizations/collision_model_trajectory", visualize_collision_model_trajectory_, false);
  node_handle_.param ("visualizations/trajectory_throttle", throttle_, 4);

  map_frame_ = "/base_footprint";

  prms_.initFromParamServer();
 
  grid_ = new sbpl_arm_planner::OccupancyGrid(3.0, 2.0, 2.0, prms_.resolution_, 0.0, 0.0, 0.0);
    
  FILE* left_file = fopen(left_arm_description_filename_.c_str(), "r"); 
  FILE* right_file = fopen(right_arm_description_filename_.c_str(), "r"); 
  
  larm_ = new sbpl_arm_planner::SBPLArmModel(left_file);
  rarm_ = new sbpl_arm_planner::SBPLArmModel(right_file);
 
  ROS_INFO("[test] Getting left arm params from param server");
  if(!larm_->initKDLChainFromParamServer())
    return false;

  ROS_INFO("[test] Getting right arm params from param server");
  if(!rarm_->initKDLChainFromParamServer())
    return false;

  larm_->setResolution(prms_.resolution_);
  rarm_->setResolution(prms_.resolution_);
 
  printf("\nLeft Arm:\n");
  larm_->printArmDescription(std::string("larm"));

  printf("\nRight Arm:\n");
  rarm_->printArmDescription(std::string("rarm"));

  cspace_ = new sbpl_two_arm_planner::SBPLDualCollisionSpace(rarm_, larm_, grid_);
  cspace_->addArmCuboidsToGrid(0);
  cspace_->addArmCuboidsToGrid(1);

  laviz_ = new sbpl_two_arm_planner::VisualizeArm(std::string("left_arm"));
  raviz_ = new sbpl_two_arm_planner::VisualizeArm(std::string("right_arm"));
  laviz_->setReferenceFrame(reference_frame_);
  raviz_->setReferenceFrame(reference_frame_);

  collision_map_filter_ = new tf::MessageFilter<arm_navigation_msgs::CollisionMap>(collision_map_subscriber_,tf_,reference_frame_,1);
  collision_map_filter_->registerCallback(boost::bind(&TestSBPLCollisionSpace::collisionMapCallback, this, _1));

  collision_object_subscriber_ = root_handle_.subscribe("collision_object", 3, &TestSBPLCollisionSpace::collisionObjectCallback, this);

  joint_states_subscriber_ = root_handle_.subscribe("joint_states", 1, &TestSBPLCollisionSpace::jointStatesCallback,this);
  ROS_INFO("[test] Initialization complete.");

  fclose(left_file);
  fclose(right_file);

  if(cspace_->getSphereGroups())
    cspace_->printSphereGroups();
  else
    return false;

  return true;
}

int TestSBPLCollisionSpace::run()
{
  ros::spin();
  return 0;
}

void TestSBPLCollisionSpace::jointStatesCallback(const sensor_msgs::JointStateConstPtr &state)
{
  double dist_m;
  unsigned char dist = 100;

  //laviz_->deleteVisualizations("text", 2);
  laviz_->deleteVisualizations("left_arm_model_0", 70);
  raviz_->deleteVisualizations("right_arm_model_0", 70);

  geometry_msgs::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 1.6;

  rangles_[0] = state->position[17];
  rangles_[1] = state->position[18];
  rangles_[2] = state->position[16];
  rangles_[3] = state->position[20];
  rangles_[4] = state->position[19];
  rangles_[5] = state->position[21];
  rangles_[6] = state->position[22];

  langles_[0] = state->position[29];
  langles_[1] = state->position[30];
  langles_[2] = state->position[28];
  langles_[3] = state->position[32];
  langles_[4] = state->position[31];
  langles_[5] = state->position[33];
  langles_[6] = state->position[34];

  body_pos_.z = state->position[12];
  head_pan_ = state->position[13];
  head_tilt_ = state->position[14];

  // get base position
  try
  {
    tf_.lookupTransform("map", "base_footprint", ros::Time(0), base_map_transform_);
    body_pos_.x = base_map_transform_.getOrigin().x();
    body_pos_.y = base_map_transform_.getOrigin().y();
    //body_pos_.theta = tf::getYaw(base_map_transform_.getRotation().getYaw());
    body_pos_.theta = 2 * atan2(base_map_transform_.getRotation().getZ(), base_map_transform_.getRotation().getW()); 
    ROS_DEBUG("Received transform from base_footprint to map (x: %f y: %f yaw: %f)", body_pos_.x, body_pos_.y, body_pos_.theta);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("Is there a map? The map-robot transform failed. (%s)",ex.what());
  }

  std::vector<std::vector<double> > spheres;
  cspace_->getCollisionSpheres(langles_,rangles_,body_pos_,"turrets",spheres);
  pviz_.visualizeSpheres(spheres, 40, "turrets");
  cspace_->getCollisionSpheres(langles_,rangles_,body_pos_,"base",spheres);
  pviz_.visualizeSpheres(spheres, 80, "base");
  cspace_->getCollisionSpheres(langles_,rangles_,body_pos_,"tilt_laser",spheres);
  pviz_.visualizeSpheres(spheres, 120, "tilt_laser");
  cspace_->getCollisionSpheres(langles_,rangles_,body_pos_,"head",spheres);
  pviz_.visualizeSpheres(spheres, 160, "head");
  cspace_->getCollisionSpheres(langles_,rangles_,body_pos_,"left_gripper",spheres);
  pviz_.visualizeSpheres(spheres, 200, "left_gripper");
  cspace_->getCollisionSpheres(langles_,rangles_,body_pos_,"right_gripper",spheres);
  pviz_.visualizeSpheres(spheres, 240, "right_gripper");
  cspace_->getCollisionSpheres(langles_,rangles_,body_pos_,"right_forearm",spheres);
  pviz_.visualizeSpheres(spheres, 280, "right_forearm");
  cspace_->getCollisionSpheres(langles_,rangles_,body_pos_,"left_forearm",spheres);
  pviz_.visualizeSpheres(spheres, 320, "left_forearm");
  cspace_->getCollisionSpheres(langles_,rangles_,body_pos_,"torso_upper",spheres);
  pviz_.visualizeSpheres(spheres, 360, "torso_upper");
  cspace_->getCollisionSpheres(langles_,rangles_,body_pos_,"torso_lower",spheres);
  pviz_.visualizeSpheres(spheres, 20, "torso_lower");



  if(!cspace_->isBodyValid(body_pos_.x, body_pos_.y, body_pos_.theta, body_pos_.z, dist))
  {
    dist_m = double(int(dist)*prms_.resolution_);
    ROS_INFO("dist = %0.3fm (%d cells)  BODY COLLISION", dist_m, int(dist));
  }
  else
  { 
    dist_m = double(int(dist)*prms_.resolution_);
    ROS_INFO("dist = %0.3fm (%d cells)  NO BODY COLLISION", dist_m, int(dist));
  }

  if(!cspace_->checkCollisionArmsToBody(langles_, rangles_, body_pos_, dist))
  {
    dist_m = double(int(dist)*prms_.resolution_);
    ROS_INFO("dist = %0.3fm (%d cells)  SELF COLLISION", dist_m, int(dist));
    //printf("%s\n", cspace_->code_.c_str());
    //laviz_->visualizeText(pose, cspace_->code_, "text",0,320);
  }
  else
  {
    dist_m = double(int(dist)*prms_.resolution_);
    ROS_INFO("dist = %0.3fm (%d cells) NO SELF COLLISION", dist_m, int(dist));
    //laviz_->visualizeText(pose, "No Collision", "text",0,100);
  }
  ROS_INFO("---------------------------------------------------------");

/*
    printf("\n");
    printf("right arm: ");
    for(size_t i = 0; i < 7; ++i)
      printf("%0.3f ", rangles_[i]);
    printf("\n");

    printf("left arm:  ");
    for(size_t i = 0; i < 7; ++i)
      printf("%0.3f ", langles_[i]);
    printf("\n");
*/

  int debug_code;
  if(!cspace_->checkCollision(langles_, rangles_, body_pos_, true, dist, debug_code))
  {
    dist_m = double(int(dist)*prms_.resolution_);
    ROS_INFO("dist = %0.3fm (%d cells)  COLLISION", dist_m, int(dist));
    printf("%s\n", cspace_->code_.c_str());
    laviz_->visualizeText(pose, cspace_->code_, "text",0,320);
  }
  else
  {
    dist_m = double(int(dist)*prms_.resolution_);
    ROS_INFO("dist = %0.3fm (%d cells)", dist_m, int(dist));
    laviz_->visualizeText(pose, "No Collision", "text",0,100);
  }

  /*
  std::vector<std::vector<double> > path(1,std::vector<double> (7,0));
  path[0] = langles_;
  laviz_->visualizeCollisionModel(path, *cspace_, 1);
  path[0] = rangles_;
  raviz_->visualizeCollisionModel(path, *cspace_, 1, 140);
  */
  
  visualizeCollisionObjects();
}

void TestSBPLCollisionSpace::collisionMapCallback(const arm_navigation_msgs::CollisionMapConstPtr &collision_map)
{
  updateMapFromCollisionMap(collision_map);
}

void TestSBPLCollisionSpace::updateMapFromCollisionMap(const arm_navigation_msgs::CollisionMapConstPtr &collision_map)
{
  ROS_INFO("map callback");

  if(collision_map->header.frame_id.compare(reference_frame_) != 0)
  {
    ROS_WARN("collision_map_occ is in %s not in %s", collision_map->header.frame_id.c_str(), reference_frame_.c_str());
    ROS_DEBUG("the collision map has %i cubic obstacles", int(collision_map->boxes.size()));
  }

  // add collision map msg
  grid_->updateFromCollisionMap(*collision_map);

  // add self collision blocks
  cspace_->addArmCuboidsToGrid(0);

  //cspace_->putCollisionObjectsInGrid();

  map_frame_ = collision_map->header.frame_id; 
  setArmToMapTransform(map_frame_);

  //visualizeCollisionObjects();
  grid_->visualize();
  ROS_INFO("leaving map callback");
}

void TestSBPLCollisionSpace::collisionObjectCallback(const arm_navigation_msgs::CollisionObjectConstPtr &collision_object)
{
  // for some reason, it wasn't getting all of the 'all' messages...
  if(collision_object->id.compare("all") == 0)
    cspace_->removeAllCollisionObjects();

  // debug: have we seen this collision object before?
  if(object_map_.find(collision_object->id) != object_map_.end())
    ROS_DEBUG("[collisionObjectCallback] We have seen this object ('%s')  before.", collision_object->id.c_str());
  else
    ROS_DEBUG("[collisionObjectCallback] We have NOT seen this object ('%s') before.", collision_object->id.c_str());
  object_map_[collision_object->id] = (*collision_object);

  ROS_INFO("\n\n[collisionObjectCallback] %s\n\n", collision_object->id.c_str());
  cspace_->processCollisionObjectMsg((*collision_object));

  visualizeCollisionObjects();
}

void TestSBPLCollisionSpace::setArmToMapTransform(std::string &map_frame)
{
  std::string right_fk_root_frame, left_fk_root_frame;

  // frame that the sbpl_arm_model is working in
  rarm_->getArmChainRootLinkName(right_fk_root_frame);
  larm_->getArmChainRootLinkName(left_fk_root_frame);

  // get transform to frame that collision map is in
  try
  {
    tf_.lookupTransform(map_frame, right_fk_root_frame, ros::Time(0), transform_);

    ROS_DEBUG("Received transform from %s to %s (translation: %f %f %f)",right_fk_root_frame.c_str(),map_frame.c_str(), transform_.getOrigin().x(),transform_.getOrigin().y(),transform_.getOrigin().z());
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
  }

  // convert transform to a KDL object
  tf::TransformTFToKDL(transform_,kdl_transform_);
  rarm_->setRefFrameTransform(kdl_transform_, map_frame);

  // get transform to frame that collision map is in
  try
  {
    tf_.lookupTransform(map_frame, left_fk_root_frame, ros::Time(0), transform_);

    ROS_DEBUG("Received transform from %s to %s (translation: %f %f %f)",left_fk_root_frame.c_str(),map_frame.c_str(), transform_.getOrigin().x(),transform_.getOrigin().y(),transform_.getOrigin().z());
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
  }

  // convert transform to a KDL object
  tf::TransformTFToKDL(transform_,kdl_transform_);
  larm_->setRefFrameTransform(kdl_transform_, map_frame);
}

void TestSBPLCollisionSpace::visualizeCollisionObjects()
{
  std::vector<geometry_msgs::Pose> poses;
  std::vector<std::vector<double> > points(1,std::vector<double>(3,0));
  std::vector<double> color(4,1);
  color[2] = 0;

  cspace_->getCollisionObjectVoxelPoses(poses);

  points.resize(poses.size());
  for(size_t i = 0; i < poses.size(); ++i)
  {
    points[i].resize(3);
    points[i][0] = poses[i].position.x;
    points[i][1] = poses[i].position.y;
    points[i][2] = poses[i].position.z;
  }

  ROS_DEBUG("[visualizeCollisionObjects] Displaying %d known collision object voxels.", int(points.size()));
  raviz_->visualizeBasicStates(points, color, "known_objects", 0.01);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_sbpl_collision_space");
  TestSBPLCollisionSpace test;

  if(!test.init())
  {
    ROS_INFO("Something is fucked");
    return 0;
  }

  return test.run();
}

