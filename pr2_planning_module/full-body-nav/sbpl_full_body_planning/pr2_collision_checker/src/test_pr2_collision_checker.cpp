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

#include <iostream>
#include <ros/ros.h>
#include <sbpl_arm_planner/occupancy_grid.h>
#include <sbpl_full_body_planner/pr2_collision_space.h>
#include <sbpl_full_body_planner/sbpl_full_body_params.h>
#include <pr2_collision_checker/pr2_collision_space.h>
#include <pr2_collision_checker/pr2_collision_space_monitor.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_pr2_collision_space");
  ros::NodeHandle nh("~");
  ROSCONSOLE_AUTOINIT;
  sleep(1);
  FILE* rarm_fp=NULL;
  FILE* larm_fp=NULL;
  std::string reference_frame, larm_filename, rarm_filename;
  sbpl_full_body_planner::SBPLFullBodyParams prms;
  sbpl_arm_planner::SBPLArmModel *larm, *rarm;
  sbpl_arm_planner::OccupancyGrid *grid;
  pr2_collision_checker::PR2CollisionSpace *cspace;
  pr2_collision_checker::PR2CollisionSpaceMonitor *cspace_mon;
  nh.param<std::string>("planner/left_arm_description_file", larm_filename, "");
  nh.param<std::string>("planner/right_arm_description_file", rarm_filename, "");

  // initialize basic sbpl arm planner parameters
  prms.initFromParamServer();

  // create the left & right arm models
  if((rarm_fp=fopen(rarm_filename.c_str(),"r")) == NULL)
  {
    ROS_ERROR("[pr2cc] Failed to open right arm description file.");
    return -1;
  }
  if((larm_fp=fopen(larm_filename.c_str(),"r")) == NULL)
  {
    ROS_ERROR("[pr2cc] Failed to open left arm description file.");
    return -1;
  }
  ROS_INFO("Initializing Arms");
  rarm = new sbpl_arm_planner::SBPLArmModel(rarm_fp);
  larm = new sbpl_arm_planner::SBPLArmModel(larm_fp);
  rarm->setResolution(prms.resolution_);
  larm->setResolution(prms.resolution_);
  rarm->setDebugLogName(prms.arm_log_);
  larm->setDebugLogName(prms.arm_log_);
  ROS_INFO("Initialized Arms");

  if(!rarm->initKDLChainFromParamServer() || !larm->initKDLChainFromParamServer())
  {
    ROS_ERROR("[pr2cc] Failed to initialize arm models.");
    return -1;
  }
  fclose(rarm_fp);
  fclose(larm_fp);

  // create the occupancy grid
  grid = new sbpl_arm_planner::OccupancyGrid(prms.sizeX_,prms.sizeY_,prms.sizeZ_, prms.resolution_,prms.originX_,prms.originY_,prms.originZ_);
  grid->setReferenceFrame(prms.reference_frame_);

  // create the collision space
  cspace = new pr2_collision_checker::PR2CollisionSpace(rarm, larm, grid);
  cspace->setDebugLogName(prms.cspace_log_);
  if(!cspace->getSphereGroups())
  {
    ROS_ERROR("[pr2cc] Failed to get the full body spheres from the param server.");
    return -1;
  }
  else
    cspace->printSphereGroups();

  // create the collision space monitor
  cspace_mon = new pr2_collision_checker::PR2CollisionSpaceMonitor(grid, cspace);


  if(!cspace_mon->init())
  {
    ROS_ERROR("[pr2cc] Something is fucked");
    return -1;
  }

  ROS_INFO("[pr2cc] Spinning...");
  ros::spin();

  return 1;
}


