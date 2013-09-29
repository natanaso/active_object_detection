/*
 * Copyright (c) 2010, Maxim Likhachev
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
 *     * Neither the name of the University of Pennsylvania nor the names of its
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

#include <sbpl_full_body_planner/sbpl_full_body_params.h>

 
namespace sbpl_full_body_planner {

SBPLFullBodyParams::SBPLFullBodyParams()
{
  epsilon_ = 10;
  epsilon2_ = 2;
  use_multires_mprims_ = true;
  use_dijkstra_heuristic_ = true;
  use_6d_pose_goal_ = true;
  sum_heuristics_ = false;
  use_uniform_cost_ = true;
  use_ik_ = true;
  use_orientation_solver_ = true;

  verbose_ = false;
  verbose_heuristics_ = false;
  verbose_collisions_ = false;
  angle_delta_ = 360;

  num_mprims_ = 0;
  num_long_dist_mprims_ = 0;
  num_short_dist_mprims_ = 0;
  
  short_dist_mprims_thresh_c_ = 20;
  short_dist_mprims_thresh_m_ = 0.2;
  
  cost_multiplier_ = 1000;
  cost_per_cell_ = 0;
  cost_per_meter_ = 0;

  range1_cost_ = 12;
  range2_cost_ = 7;
  range3_cost_ = 2;

  solve_for_ik_thresh_ = 1000;
  solve_for_ik_thresh_m_= 0.20;

  two_calls_to_op_ = false;
  is_goal_function_ = 0;

  /* cartesian planner */
  xyz_resolution_ = 0.02;
  rpy_resolution_ = 0.034906585; // 2 deg
  fa_resolution_ = 0.0523598776; // 3 deg

  base_theta_resolution_ = 0.3926991; //22.5 deg
  num_base_dirs_ = 16;

  use_orbit_motions_ = false;
  num_orbit_motions_ = 0;
  nav3d_ = true;

  expands_log_ = "expands";
  expands2_log_ = "expands2";
  arm_log_ = "arm";
  ik_log_ = "ik";
  cspace_log_ = "cspace";
  solution_log_ = "solution";

  expands_log_level_ = "info";
  expands2_log_level_ = "info";
  ik_log_level_ = "info";
  arm_log_level_ = "info";
  cspace_log_level_ = "info";
  solution_log_level_ = "solution";

  cost_per_second_ = cost_multiplier_;
  time_per_cell_ = 0.02;
  joint_vel_.resize(7, 1.75);

  use_research_grid_ = false;

  //pulled from URDF 
  double desired_speed_pct = 0.8;

  joint_vel_[0] = 0.6*3.48*desired_speed_pct;
  joint_vel_[1] = 0.6*3.47*desired_speed_pct;
  joint_vel_[2] = 0.6*5.45*desired_speed_pct;
  joint_vel_[3] = 0.6*5.50*desired_speed_pct;
  joint_vel_[4] = 0.6*6.00*desired_speed_pct;
  joint_vel_[5] = 0.6*5.13*desired_speed_pct;
  joint_vel_[6] = 0.6*6.00*desired_speed_pct;

  ROS_INFO("[params] Joint Velocities:  %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", joint_vel_[0],joint_vel_[1], joint_vel_[2],joint_vel_[3],joint_vel_[4],joint_vel_[5],joint_vel_[6]);
  ROS_INFO("[params] Compute Time Per Cell: %0.3f (temporarily hard coded)", time_per_cell_);
}

void SBPLFullBodyParams::initFromParamServer()
{
  ros::NodeHandle nh("~");

  /* planner */
  nh.param("planner/epsilon",epsilon_, 10.0);
  nh.param("planner/epsilon2",epsilon2_, 1.0);
  nh.param("planner/use_dijkstra_heuristic",use_dijkstra_heuristic_,true);
  nh.param("planner/use_research_heuristic",use_research_heuristic_,false);
  nh.param("planner/use_research_grid",use_research_grid_,false);
  nh.param("planner/use_multiresolution_motion_primitives",use_multires_mprims_,true);
  nh.param("planner/use_uniform_obstacle_cost", use_uniform_cost_,false);
  nh.param("planner/verbose",verbose_,false);
  nh.param("planner/obstacle_distance_cost_far",range3_cost_,12);
  nh.param("planner/obstacle_distance_cost_mid",range2_cost_,7);
  nh.param("planner/obstacle_distance_cost_near",range1_cost_,2);
  
  //base parameters
  nh.param("planner/nominalvel_mpersecs",nominalvel_mpersecs_,0.5);
  nh.param("planner/timetoturn45degsinplace_secs",timetoturn45degsinplace_secs_,0.5);
  nh.param("planner/spine_cost",spine_cost_, 500);
  nh.param("planner/nav3d",nav3d_, true);
  nh.param("planner/use_orbit_motions",use_orbit_motions_, false);
  if(use_orbit_motions_)
    num_orbit_motions_ = 3;
  else
    num_orbit_motions_ = 0;

  /* research params of the planner */
  nh.param("planner/research/solve_with_ik_threshold",solve_for_ik_thresh_m_,0.15);
  nh.param("planner/research/sum_heuristics",sum_heuristics_,false);
  nh.param("planner/research/short_distance_mprims_threshold",short_dist_mprims_thresh_m_, 0.2);

  /* occupancy grid */
  nh.param("collision_space/resolution",resolution_,0.02);
  nh.param<std::string>("collision_space/reference_frame",reference_frame_,"base_link");
  nh.param("collision_space/occupancy_grid/origin_x",originX_,-0.6);
  nh.param("collision_space/occupancy_grid/origin_y",originY_,-1.15);
  nh.param("collision_space/occupancy_grid/origin_z",originZ_,-0.05);  
  nh.param("collision_space/occupancy_grid/size_x",sizeX_,1.6);
  nh.param("collision_space/occupancy_grid/size_y",sizeY_,1.8);
  nh.param("collision_space/occupancy_grid/size_z",sizeZ_,1.4);

  /* logging */
  nh.param<std::string>("debug/logging/expands", expands_log_level_, "info");
  nh.param<std::string>("debug/logging/expands2", expands2_log_level_, "info");
  nh.param<std::string>("debug/logging/ik", ik_log_level_, "info");
  nh.param<std::string>("debug/logging/arm_model", arm_log_level_, "info");
  nh.param<std::string>("debug/logging/collisions", cspace_log_level_, "debug");
  nh.param<std::string>("debug/logging/solution", solution_log_level_, "info");

  //TODO: This should be done in a way in which the package name of each
  //logger doesn't have to be known....
  changeLoggerLevel(ROSCONSOLE_DEFAULT_NAME + std::string(".") + expands_log_, expands_log_level_);
  changeLoggerLevel(ROSCONSOLE_DEFAULT_NAME + std::string(".") + expands2_log_, expands2_log_level_);
  changeLoggerLevel(ROSCONSOLE_DEFAULT_NAME + std::string(".") + ik_log_, ik_log_level_);
  changeLoggerLevel(std::string("ros.sbpl_arm_planner.") + arm_log_, arm_log_level_);
  changeLoggerLevel(ROSCONSOLE_DEFAULT_NAME + std::string(".") + cspace_log_, cspace_log_level_);
  changeLoggerLevel(ROSCONSOLE_DEFAULT_NAME + std::string(".") + solution_log_, solution_log_level_);
}

bool SBPLFullBodyParams::initFromParamFile(std::string param_file)
{
  char* filename = new char[param_file.length()+1];
  param_file.copy(filename, param_file.length(),0);
  filename[param_file.length()] = '\0';
  FILE* fCfg = fopen(filename, "r");
  
  if(initFromParamFile(fCfg))
  {
    delete filename;
    fclose(fCfg);
    delete fCfg;
    return true;
  }
  else
  {
    delete filename;
    fclose(fCfg);
    delete fCfg;
    return false;
  }
}

bool SBPLFullBodyParams::initMotionPrimsFromFile(FILE* fCfg)
{
  char sTemp[1024];
  int nrows=0,ncols=0, short_mprims=0;


  if(fCfg == NULL)
  {
    SBPL_ERROR("ERROR: unable to open the params file. Exiting.");
    return false;
  }

  if(fscanf(fCfg,"%s",sTemp) < 1)
    SBPL_WARN("Parsed string has length < 1."); 
  if(strcmp(sTemp, "Motion_Primitives(degrees):") != 0)
  {
    SBPL_ERROR("ERROR: First line of motion primitive file should be 'Motion_Primitives(degrees):'. Please check your file. (parsed string: %s)\n", sTemp);
    return false;
  }

  //number of actions
  if(fscanf(fCfg,"%s",sTemp) < 1) 
  {
    SBPL_WARN("Parsed string has length < 1.");
    return false;
  }
  else
    nrows = atoi(sTemp);
  
  //length of joint array
  if(fscanf(fCfg,"%s",sTemp) < 1)
  {
    SBPL_WARN("Parsed string has length < 1.");
    return false;
  }
  else
    ncols = atoi(sTemp);

  //number of short distance motion primitives
  if(fscanf(fCfg,"%s",sTemp) < 1)
  { 
    SBPL_WARN("Parsed string has length < 1.");
    return false;
  }
  else
    short_mprims = atoi(sTemp);

  if(short_mprims == nrows)
    SBPL_ERROR("Error: # of motion prims == # of short distance motion prims. No long distance motion prims set.");

  std::vector<double> mprim(ncols,0);

  for (int i=0; i < nrows; ++i)
  {
    for(int j=0; j < ncols; ++j)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_WARN("Parsed string has length < 1.");
      if(!feof(fCfg) && strlen(sTemp) != 0)
        mprim[j] = atof(sTemp);
      else
      {
        SBPL_ERROR("ERROR: End of parameter file reached prematurely. Check for newline.");
        return false;
      }
    }
    if(i < (nrows-short_mprims))
      addMotionPrim(mprim,true,false);
    else
      addMotionPrim(mprim,true,true);
  }
 
  max_mprim_offset_ = getLargestMotionPrimOffset(); 

  return true;
}

bool SBPLFullBodyParams::initLongMotionPrimsFromFile(FILE* fCfg)
{
  char sTemp[1024];
  int ndof=0;
  MotionPrimitive temp;
  std::vector<double> p;
  std::vector<bool> add_reflect(6,0);

  if(fCfg == NULL)
  {
    SBPL_ERROR("Failed to open the motion primitive file. Exiting.");
    return false;
  }

  if(fscanf(fCfg,"%s",sTemp) < 1) 
    SBPL_INFO("Parsed string has length < 1.\n");

  while(!feof(fCfg))
  {
    if(sTemp[0] == '#') //comments
      fgets(sTemp, 1024,fCfg);
    else if(strcmp(sTemp, "degrees_of_freedom:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        SBPL_WARN("Failed to parse # degrees of freedom."); 
      ndof = atoi(sTemp);
    }
    else if(strcmp(sTemp, "xyz_resolution(meters):") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        SBPL_WARN("Failed to parse xyz resolution."); 
      xyz_resolution_ = atof(sTemp);
      if(xyz_resolution_ != resolution_)
        ROS_ERROR("Resolution in motion primitive file does not match the resolution of the occupancy grid.");
    }
    else if(strcmp(sTemp, "rpy_resolution(degrees):") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        SBPL_WARN("Failed to parse rpy resolution."); 
      rpy_resolution_ = atof(sTemp) * M_PI/180.0;
    }
    else if(strcmp(sTemp, "free_angle_resolution(degrees):") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        SBPL_WARN("Failed to parse free angle resolution."); 
      fa_resolution_ = atof(sTemp) * M_PI/180.0;
    }
    //type needs to be defined first
    else if(strcmp(sTemp, "type:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        SBPL_WARN("Parsed string has length < 1. (type)"); 
 
      if(strcmp(sTemp,"long_distance") == 0)
        temp.type = LONG_DISTANCE;   
      else if(strcmp(sTemp,"short_distance") == 0)
        temp.type = SHORT_DISTANCE;   
      else
      {
        SBPL_ERROR("Invalid primitive type. Currently only two types of motion primitives - 'short_distance' or 'long_distance'. Assuming long_distance.");
        temp.type = 0;
      }

      add_reflect.resize(6,0);
      temp.m.clear();
      temp.group = -1;
      temp.coord.resize(ndof,0);
    }
    else if(strcmp(sTemp, "group:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        SBPL_WARN("Parsed string has length < 1. (group)");
      temp.group = atoi(sTemp);
    }
    else if(strcmp(sTemp, "id:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        SBPL_WARN("Parsed string has length < 1. (id)");
      temp.id = atoi(sTemp);
    }
    else if(strcmp(sTemp, "end_coords(cells):") == 0)
    {
      for(int k = 0; k < ndof; ++k)
      {
        if(fscanf(fCfg,"%s",sTemp) < 1)
          SBPL_WARN("Parsed string has length < 1. (coords)");
        temp.coord[k] = atoi(sTemp);
      }
    }
    //add same primitive flipped around a certain axis
    else if(strcmp(sTemp, "add_reflect(no/x/y/z/xy/xz/yz):") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        SBPL_WARN("Parsed string has length < 1. (add_reflect)");
    
      if(strcmp(sTemp,"x") == 0)
        add_reflect[0] = true;    
      else if(strcmp(sTemp,"y") == 0)
        add_reflect[1] = true;    
      else if(strcmp(sTemp,"z") == 0)
        add_reflect[2] = true;    
      else if(strcmp(sTemp,"xy") == 0)
        add_reflect[3] = true;    
      else if(strcmp(sTemp,"xz") == 0)
        add_reflect[4] = true;    
      else if(strcmp(sTemp,"yz") == 0)
        add_reflect[5] = true;    
      else  
        SBPL_ERROR("Not adding inverse.Either invalid axis or 'no' selected.");
    }
    //add same primitive rotated around a certain axis n times
    /*
    else if(strcmp(sTemp, "evenly_distribute_about_x:") == 0)
    {
      if(fscanf(fCfg,"%d",n_rotate_x) < 1)
        SBPL_WARN("Failed to parse # of evenly distributed primitives about x-axis.");
    }
    else if(strcmp(sTemp, "evenly_distribute_about_y:") == 0)
    {
      if(fscanf(fCfg,"%d",n_rotate_y) < 1)
        SBPL_WARN("Failed to parse # of evenly distributed primitives about y-axis.");
    }
    else if(strcmp(sTemp, "evenly_distribute_about_z:") == 0)
    {
      if(fscanf(fCfg,"%d",n_rotate_z) < 1)
        SBPL_WARN("Failed to parse # of evenly distributed primitives about z-axis.");
    }
    */
    //parse the primitive
    else if(strcmp(sTemp, "intermediate_steps:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        SBPL_WARN("Parsed string has length < 1. (intermediate_steps)"); 
    
      temp.nsteps = atoi(sTemp);

      for(int i=0; i < temp.nsteps; ++i)
      {
        p.resize(ndof,0);

        for(int j=0; j < ndof; ++j)
        {
          if(fscanf(fCfg,"%s",sTemp) < 1)
            SBPL_WARN("Expected %d values in motion primitive. Adding zeroes instead.", ndof);

          p[j] = atof(sTemp);  
        }
        temp.m.push_back(p);
      }
      mp_.push_back(temp);
    }
    else
      SBPL_WARN("Found unexpected line in motion primitive file. (%s)", sTemp);

    if(fscanf(fCfg,"%s",sTemp) < 1)
      SBPL_INFO("Parsed string has length < 1. (%s)", sTemp);
  }

  // add an orientation solving motion primitive
  temp.nsteps = 1;
  temp.type = ADAPTIVE;
  temp.group = 0;
  temp.id = mp_.back().id + 1;
  temp.coord.clear();   //hack
  temp.coord.resize(ndof,0);
  p.clear();
  p.resize(ndof,0);
  temp.m.clear();
  temp.m.push_back(p);
  mp_.push_back(temp);


  ROS_INFO("[params] There are %d motion primitives.", int(mp_.size()));
  return true;
}

bool SBPLFullBodyParams::initBaseMotionPrimsFromFile(FILE* fMotPrims){
  char sTemp[1024], sExpected[1024];
  float fTemp;
  int dTemp;
  int totalNumofActions = 0;

  SBPL_PRINTF("Reading in motion primitives...");

  //read in the resolution
  strcpy(sExpected, "resolution_m:");
  if(fscanf(fMotPrims, "%s", sTemp) == 0)
    return false;
  if(strcmp(sTemp, sExpected) != 0){
    SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
    return false;
  }
  if(fscanf(fMotPrims, "%f", &fTemp) == 0)
    return false;
  if(fabs(fTemp-xyz_resolution_) > ERR_EPS){
    SBPL_ERROR("ERROR: invalid resolution %f (instead of %f) in the dynamics file\n",
        fTemp, xyz_resolution_);
    return false;
  }

  //read in the angular resolution
  strcpy(sExpected, "numberofangles:");
  if(fscanf(fMotPrims, "%s", sTemp) == 0)
    return false;
  if(strcmp(sTemp, sExpected) != 0){
    SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
    return false;
  }
  if(fscanf(fMotPrims, "%d", &dTemp) == 0)
    return false;
  /*
  if(dTemp != EnvNAVXYTHETALATCfg.NumThetaDirs){
    SBPL_ERROR("ERROR: invalid angular resolution %d angles (instead of %d angles) in the motion primitives file\n",
        dTemp, EnvNAVXYTHETALATCfg.NumThetaDirs);
    return false;
  }
  */
  base_theta_resolution_ = 2*M_PI/dTemp;
  num_base_dirs_ = dTemp;

  //read in the total number of actions
  strcpy(sExpected, "totalnumberofprimitives:");
  if(fscanf(fMotPrims, "%s", sTemp) == 0)
    return false;
  if(strcmp(sTemp, sExpected) != 0){
    SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
    return false;
  }
  if(fscanf(fMotPrims, "%d", &totalNumofActions) == 0){
    return false;
  }

  for(int i = 0; i < totalNumofActions; i++){
    SBPL_xytheta_mprimitive motprim;

    if(ReadinMotionPrimitive(&motprim, fMotPrims) == false)
      return false;

    base_mp_.push_back(motprim);
  }
  SBPL_PRINTF("done reading base motion primitives");

  PrecomputeActionswithCompleteMotionPrimitive(&base_mp_);

  return true;
}

bool SBPLFullBodyParams::ReadinMotionPrimitive(SBPL_xytheta_mprimitive* pMotPrim, FILE* fIn){
  char sTemp[1024];
  int dTemp;
  char sExpected[1024];
  int numofIntermPoses;

  //read in actionID
  strcpy(sExpected, "primID:");
  if(fscanf(fIn, "%s", sTemp) == 0)
    return false;
  if(strcmp(sTemp, sExpected) != 0){
    SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
    return false;
  }
  if(fscanf(fIn, "%d", &pMotPrim->motprimID) != 1)
    return false;

  //read in start angle
  strcpy(sExpected, "startangle_c:");
  if(fscanf(fIn, "%s", sTemp) == 0)
    return false;
  if(strcmp(sTemp, sExpected) != 0){
    SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
    return false;
  }
  if(fscanf(fIn, "%d", &dTemp) == 0)
  {
    SBPL_ERROR("ERROR reading startangle\n");
    return false;
  }
  pMotPrim->starttheta_c = dTemp;

  //read in end pose
  strcpy(sExpected, "endpose_c:");
  if(fscanf(fIn, "%s", sTemp) == 0)
    return false;
  if(strcmp(sTemp, sExpected) != 0){
    SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
    return false;
  }

  if(ReadinCell(&pMotPrim->endcell, fIn) == false){
    SBPL_ERROR("ERROR: failed to read in endsearchpose\n");
    return false;
  }

  //read in action cost
  strcpy(sExpected, "additionalactioncostmult:");
  if(fscanf(fIn, "%s", sTemp) == 0)
    return false;
  if(strcmp(sTemp, sExpected) != 0){
    SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
    return false;
  }
  if(fscanf(fIn, "%d", &dTemp) != 1)
    return false;
  pMotPrim->additionalactioncostmult = dTemp;

  //read in intermediate poses
  strcpy(sExpected, "intermediateposes:");
  if(fscanf(fIn, "%s", sTemp) == 0)
    return false;
  if(strcmp(sTemp, sExpected) != 0){
    SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
    return false;
  }
  if(fscanf(fIn, "%d", &numofIntermPoses) != 1)
    return false;
  //all intermposes should be with respect to 0,0 as starting pose since it will be added later and should be done 
  //after the action is rotated by initial orientation
  for(int i = 0; i < numofIntermPoses; i++){
    sbpl_xy_theta_pt_t intermpose;
    if(ReadinPose(&intermpose, fIn) == false){
      SBPL_ERROR("ERROR: failed to read in intermediate poses\n");
      return false;
    }
    pMotPrim->intermptV.push_back(intermpose);
  }


  //check that the last pose corresponds correctly to the last pose
  sbpl_xy_theta_pt_t sourcepose;
  sourcepose.x = DISCXY2CONT(0, xyz_resolution_);
  sourcepose.y = DISCXY2CONT(0, xyz_resolution_);
  sourcepose.theta = DiscTheta2Cont(pMotPrim->starttheta_c, num_base_dirs_);
  double mp_endx_m = sourcepose.x + pMotPrim->intermptV[pMotPrim->intermptV.size()-1].x;
  double mp_endy_m = sourcepose.y + pMotPrim->intermptV[pMotPrim->intermptV.size()-1].y;
  double mp_endtheta_rad = pMotPrim->intermptV[pMotPrim->intermptV.size()-1].theta;
  int endx_c = CONTXY2DISC(mp_endx_m, xyz_resolution_);
  int endy_c = CONTXY2DISC(mp_endy_m, xyz_resolution_);
  int endtheta_c = ContTheta2Disc(mp_endtheta_rad, num_base_dirs_);
  if(endx_c != pMotPrim->endcell.x || endy_c != pMotPrim->endcell.y || endtheta_c != pMotPrim->endcell.theta)
  {
    SBPL_ERROR("ERROR: incorrect primitive %d with startangle=%d last interm point %f %f %f does not match end pose %d %d %d\n",
        pMotPrim->motprimID, pMotPrim->starttheta_c,
        pMotPrim->intermptV[pMotPrim->intermptV.size()-1].x, pMotPrim->intermptV[pMotPrim->intermptV.size()-1].y, pMotPrim->intermptV[pMotPrim->intermptV.size()-1].theta,
        pMotPrim->endcell.x, pMotPrim->endcell.y,pMotPrim->endcell.theta);
    return false;
  }

  return true;
}

bool SBPLFullBodyParams::ReadinCell(sbpl_xy_theta_cell_t* cell, FILE* fIn){
  char sTemp[60];

  if(fscanf(fIn, "%s", sTemp) == 0)
    return false;
  cell->x = atoi(sTemp);
  if(fscanf(fIn, "%s", sTemp) == 0)
    return false;
  cell->y = atoi(sTemp);
  if(fscanf(fIn, "%s", sTemp) == 0)
    return false;
  cell->theta = atoi(sTemp);

  //normalize the angle
  cell->theta = NORMALIZEDISCTHETA(cell->theta, num_base_dirs_);

  return true;
}

bool SBPLFullBodyParams::ReadinPose(sbpl_xy_theta_pt_t* pose, FILE* fIn){
  char sTemp[60];

  if(fscanf(fIn, "%s", sTemp) == 0)
    return false;
  pose->x = atof(sTemp);
  if(fscanf(fIn, "%s", sTemp) == 0)
    return false;
  pose->y = atof(sTemp);
  if(fscanf(fIn, "%s", sTemp) == 0)
    return false;
  pose->theta = atof(sTemp);

  pose->theta = normalizeAngle(pose->theta);

  return true;
}

void SBPLFullBodyParams::PrecomputeActionswithCompleteMotionPrimitive(vector<SBPL_xytheta_mprimitive>* motionprimitiveV){

  SBPL_PRINTF("Pre-computing action data using motion primitives for every angle...\n");
  base_actions_ = new EnvNAVXYTHETALATAction_t* [num_base_dirs_];
  vector<sbpl_2Dcell_t> footprint;

  if(motionprimitiveV->size()%num_base_dirs_ != 0){
    SBPL_ERROR("ERROR: motionprimitives should be uniform across actions\n");
    throw new SBPL_Exception();
  }

  base_action_width_ = ((int)motionprimitiveV->size())/num_base_dirs_;

  //iterate over source angles
  int maxnumofactions = 0;
  for(int tind = 0; tind < num_base_dirs_; tind++){
    SBPL_PRINTF("pre-computing for angle %d out of %d angles\n", tind, num_base_dirs_);

    base_actions_[tind] = new EnvNAVXYTHETALATAction_t[base_action_width_];

    //compute sourcepose
    sbpl_xy_theta_pt_t sourcepose;
    sourcepose.x = DISCXY2CONT(0, xyz_resolution_);
    sourcepose.y = DISCXY2CONT(0, xyz_resolution_);
    sourcepose.theta = DiscTheta2Cont(tind, num_base_dirs_);


    //iterate over motion primitives
    int numofactions = 0;
    int aind = -1;
    for(int mind = 0; mind < (int)motionprimitiveV->size(); mind++)
    {
      //find a motion primitive for this angle
      if(motionprimitiveV->at(mind).starttheta_c != tind)
        continue;

      aind++;
      numofactions++;

      //action index
      base_actions_[tind][aind].aind = aind;

      //start angle
      base_actions_[tind][aind].starttheta = tind;

      //compute dislocation
      base_actions_[tind][aind].endtheta = motionprimitiveV->at(mind).endcell.theta;
      base_actions_[tind][aind].dX = motionprimitiveV->at(mind).endcell.x;
      base_actions_[tind][aind].dY = motionprimitiveV->at(mind).endcell.y;

      //compute and store interm points as well as intersecting cells
      base_actions_[tind][aind].intersectingcellsV.clear();
      base_actions_[tind][aind].intermptV.clear();
      base_actions_[tind][aind].interm3DcellsV.clear();
      sbpl_xy_theta_cell_t previnterm3Dcell;
      previnterm3Dcell.theta = 0; previnterm3Dcell.x = 0; previnterm3Dcell.y = 0;

      for (int pind = 0; pind < (int)motionprimitiveV->at(mind).intermptV.size(); pind++){
        sbpl_xy_theta_pt_t intermpt = motionprimitiveV->at(mind).intermptV[pind];
        base_actions_[tind][aind].intermptV.push_back(intermpt);
      }

      //compute linear and angular time
      double linear_distance = 0;
      for(unsigned int i=1; i<base_actions_[tind][aind].intermptV.size(); i++){
        double x0 = base_actions_[tind][aind].intermptV[i-1].x;
        double y0 = base_actions_[tind][aind].intermptV[i-1].y;
        double x1 = base_actions_[tind][aind].intermptV[i].x;
        double y1 = base_actions_[tind][aind].intermptV[i].y;
        double dx = x1-x0;
        double dy = y1-y0;
        linear_distance += sqrt(dx*dx + dy*dy);
      }
      double linear_time = linear_distance/nominalvel_mpersecs_;
      double angular_distance = fabs(computeMinUnsignedAngleDiff(DiscTheta2Cont(base_actions_[tind][aind].endtheta, num_base_dirs_),
                            DiscTheta2Cont(base_actions_[tind][aind].starttheta, num_base_dirs_)));
      double angular_time = angular_distance/((PI_CONST/4.0)/timetoturn45degsinplace_secs_);
      //make the cost the max of the two times
      base_actions_[tind][aind].cost = (int)(ceil(NAVXYTHETALAT_COSTMULT_MTOMM*max(linear_time,angular_time)));
      //use any additional cost multiplier
      base_actions_[tind][aind].cost *= motionprimitiveV->at(mind).additionalactioncostmult;


      //now compute the intersecting cells for this motion (including ignoring the source footprint)
      //get_2d_motion_cells(EnvNAVXYTHETALATCfg.FootprintPolygon, motionprimitiveV->at(mind).intermptV, &base_actions_[tind][aind].intersectingcellsV, xyz_resolution_);
    }

    if(maxnumofactions < numofactions)
      maxnumofactions = numofactions;
  }


  //at this point we don't allow nonuniform number of actions
  if(motionprimitiveV->size() != (size_t)(num_base_dirs_*maxnumofactions))
  {
    SBPL_ERROR("ERROR: nonuniform number of actions is not supported (maxnumofactions=%d while motprims=%d thetas=%d\n",
        maxnumofactions, (unsigned int)motionprimitiveV->size(), num_base_dirs_);
    throw new SBPL_Exception();
  }

  SBPL_PRINTF("done pre-computing action data based on motion primitives\n");
}


bool SBPLFullBodyParams::initFromParamFile(FILE* fCfg)
{ 
  char sTemp[1024];
  //int nrows=0,ncols=0, short_mprims=0;

  if(fCfg == NULL)
  {
    SBPL_ERROR("ERROR: unable to open the params file. Exiting.\n");
    return false;
  }

  if(fscanf(fCfg,"%s",sTemp) < 1) 
    SBPL_INFO("Parsed string has length < 1.\n");
  while(!feof(fCfg) && strlen(sTemp) != 0)
  {
    if(strcmp(sTemp, "environment_size(meters):") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        SBPL_INFO("Parsed string has length < 1.\n");
      sizeX_ = atof(sTemp);
      if(fscanf(fCfg,"%s",sTemp) < 1)
        SBPL_INFO("Parsed string has length < 1.\n");
      sizeY_ = atof(sTemp);
      if(fscanf(fCfg,"%s",sTemp) < 1)
        SBPL_INFO("Parsed string has length < 1.\n");
      sizeZ_ = atof(sTemp);
    }
    else if(strcmp(sTemp, "environment_origin(meters):") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        SBPL_INFO("Parsed string has length < 1.\n");
      originX_ = atof(sTemp);
      if(fscanf(fCfg,"%s",sTemp) < 1)
        SBPL_INFO("Parsed string has length < 1.\n");
      originY_ = atof(sTemp);
      if(fscanf(fCfg,"%s",sTemp) < 1)
        SBPL_INFO("Parsed string has length < 1.\n");
      originZ_ = atof(sTemp);
    }
    else if(strcmp(sTemp, "resolution(meters):") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        SBPL_INFO("Parsed string has length < 1.\n");
      resolution_ = atof(sTemp);
    }
    else if(strcmp(sTemp, "use_dijkstra_heuristic:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_INFO("Parsed string has length < 1.\n");
      use_dijkstra_heuristic_ = atoi(sTemp);
    }
    else if(strcmp(sTemp, "use_orientation_solver:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_INFO("Parsed string has length < 1.\n");
      use_orientation_solver_ = atoi(sTemp);
    }
    else if(strcmp(sTemp, "use_ik:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_INFO("Parsed string has length < 1.\n");
      use_ik_ = atoi(sTemp);
    }
    else if(strcmp(sTemp, "sum_heuristics:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_INFO("Parsed string has length < 1.\n");
      sum_heuristics_ = atoi(sTemp);
    }
    else if(strcmp(sTemp, "use_research_heuristic:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_INFO("Parsed string has length < 1.\n");
      use_research_heuristic_ = atoi(sTemp);
    }
    else if(strcmp(sTemp, "plan_to_6d_pose_constraint:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_INFO("Parsed string has length < 1.\n");
      use_6d_pose_goal_ = atoi(sTemp);
    }
    else if(strcmp(sTemp,"planner_epsilon:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_INFO("Parsed string has length < 1.\n");
      epsilon_ = atof(sTemp);
    }
    else if(strcmp(sTemp,"use_multiresolution_motion_primitives:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_INFO("Parsed string has length < 1.\n");
      use_multires_mprims_ = atoi(sTemp);
    }
    else if(strcmp(sTemp,"use_uniform_obstacle_cost:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_INFO("Parsed string has length < 1.\n");
      use_uniform_cost_ = atoi(sTemp);
    }
    else if(strcmp(sTemp,"short_distance_mprims_threshold(meters):") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_INFO("Parsed string has length < 1.\n");
      short_dist_mprims_thresh_m_ = atof(sTemp);
    }
    else if(strcmp(sTemp,"check_if_at_goal_function(0:IK,1:OP):") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_INFO("Parsed string has length < 1.\n");
      is_goal_function_ = atoi(sTemp);
    }   
    else if(strcmp(sTemp,"two_calls_to_orientation_planner:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_INFO("Parsed string has length < 1.\n");
      two_calls_to_op_ = atoi(sTemp);
    }   
    else if(strcmp(sTemp,"verbose:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_INFO("Parsed string has length < 1.");
      verbose_ = atoi(sTemp);
    }
    else if(strcmp(sTemp,"solve_for_ik_threshold(distance):") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_INFO("Parsed string has length < 1.");
      solve_for_ik_thresh_m_ = atof(sTemp);
    }
    //motion primitives must be last thing in the file

    else if(strcmp(sTemp, "Motion_Primitives(degrees):") == 0)
    {
      break;
    }

    else
    {
      SBPL_INFO("Error: Invalid Field name (%s) in parameter file.",sTemp);
      //return false;
    }
    if(fscanf(fCfg,"%s",sTemp) < 1) 
      SBPL_INFO("Parsed string has length < 1. (%s)", sTemp);
  }

  max_mprim_offset_ = getLargestMotionPrimOffset(); 
    
  SBPL_INFO("Successfully parsed parameters file");
  return true;
}

void SBPLFullBodyParams::setCellCost(int cost_per_cell)
{
  cost_per_cell_ = cost_per_cell;
  solve_for_ik_thresh_ = (solve_for_ik_thresh_m_ / resolution_) * cost_per_cell_;
  short_dist_mprims_thresh_c_ = short_dist_mprims_thresh_m_/resolution_ * cost_per_cell;
}

void SBPLFullBodyParams::addMotionPrim(std::vector<double> mprim, bool add_converse, bool short_dist_mprim)
{
  if(short_dist_mprim)
  {
    mprims_.push_back(mprim);
    num_short_dist_mprims_++;
    if(add_converse)
    {
      for(int i = 0; i < int(mprim.size()); ++i)
      {
        if(mprim[i] != 0)
          mprim[i] *=  -1;
      }
      mprims_.push_back(mprim);
      num_short_dist_mprims_++;
    }
  }
  else
  {
    std::vector<std::vector<double> >::iterator it_long_dist;
    it_long_dist = mprims_.begin();
    if(num_long_dist_mprims_ > 1)
    {
      advance(it_long_dist,num_long_dist_mprims_);
      mprims_.insert(it_long_dist, mprim);
    }
    else if(it_long_dist == mprims_.end() || mprims_.empty())
      mprims_.push_back(mprim);

    num_long_dist_mprims_++;
    if(add_converse)
    {
      for(int i = 0; i < int(mprim.size()); ++i)
      {
        if(mprim[i] != 0)
          mprim[i] *=  -1;
      }

      it_long_dist=mprims_.begin();
      advance(it_long_dist,num_long_dist_mprims_);
      mprims_.insert(it_long_dist,mprim);
      num_long_dist_mprims_++;
    } 
  }
  num_mprims_ = num_short_dist_mprims_ + num_long_dist_mprims_;
}

void SBPLFullBodyParams::printMotionPrims(std::string stream)
{
  int i;
  SBPL_DEBUG_NAMED(stream,"Long Distance Motion Primitives: %d", num_long_dist_mprims_);
  for(i = 0; i < num_long_dist_mprims_; ++i)
  {
    SBPL_DEBUG_NAMED(stream,"%2d: ",i);
    for(int j = 0; j < int(mprims_[i].size()); ++j)
      SBPL_DEBUG_NAMED(stream,"%4.1f ",mprims_[i][j]);
    SBPL_DEBUG_NAMED(stream," ");
  }

  SBPL_DEBUG_NAMED(stream,"Short Distance Motion Primitives: %d", num_short_dist_mprims_);
  for(; i < num_mprims_; ++i)
  {
    SBPL_DEBUG_NAMED(stream,"%2d: ",i);
    for(int j = 0; j < int(mprims_[i].size()); ++j)
      SBPL_DEBUG_NAMED(stream,"%4.1f ",mprims_[i][j]);
    SBPL_DEBUG_NAMED(stream," ");
  }
}

void SBPLFullBodyParams::printLongMotionPrims(std::string stream)
{
  ROS_INFO_NAMED(stream, "Statespace Resolution: xyz: %0.3f rpy: %0.3f fa: %0.3f",xyz_resolution_, rpy_resolution_,fa_resolution_);
  ROS_INFO_NAMED(stream, "# Motion Primitives: %d", int(mp_.size()));
  for(int i = 0; i < int(mp_.size()); ++i)
  {
    if(mp_[i].type == LONG_DISTANCE)
      ROS_INFO_NAMED(stream,"[%d] type: long_distance    nsteps: %d group: %d", i, mp_[i].nsteps, mp_[i].group);
    else if(mp_[i].type == SHORT_DISTANCE)  
      ROS_INFO_NAMED(stream,"[%d] type: short_distance   nsteps: %d group: %d", i, mp_[i].nsteps, mp_[i].group);
    else
      ROS_INFO_NAMED(stream,"[%d] type: adaptive   nsteps: %d group: %d", i, mp_[i].nsteps, mp_[i].group);

    for(int j = 0; j < int(mp_[i].m.size()); ++j)
      ROS_INFO_NAMED(stream,"%0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", mp_[i].m[j][0],mp_[i].m[j][1],mp_[i].m[j][2],mp_[i].m[j][3],mp_[i].m[j][4],mp_[i].m[j][5],mp_[i].m[j][6],mp_[i].m[j][7]);

    ROS_INFO_NAMED(stream," ");
  }
}

void SBPLFullBodyParams::printParams(std::string stream)
{
  SBPL_DEBUG_NAMED(stream,"Full Body Planner Parameters:");
  SBPL_DEBUG_NAMED(stream,"%40s: %d", "# motion primitives",num_mprims_);
  SBPL_DEBUG_NAMED(stream,"%40s: %d", "# short distance motion primitives", num_short_dist_mprims_);
  SBPL_DEBUG_NAMED(stream,"%40s: %d", "# long distance motion primitives", num_long_dist_mprims_);
  SBPL_DEBUG_NAMED(stream,"%40s: %.2f", "epsilon",epsilon_);
  SBPL_DEBUG_NAMED(stream,"%40s: %s", "use multi-resolution motion primitives", use_multires_mprims_ ? "yes" : "no");
  SBPL_DEBUG_NAMED(stream,"%40s: %s", "use dijkstra heuristic", use_dijkstra_heuristic_ ? "yes" : "no");
  SBPL_DEBUG_NAMED(stream,"%40s: %s", "use research heuristic", use_research_heuristic_ ? "yes" : "no");
  SBPL_DEBUG_NAMED(stream,"%40s: %s", "h = h_elbow + h_endeff", sum_heuristics_ ? "yes" : "no"); 
  SBPL_DEBUG_NAMED(stream,"%40s: %s", "use a uniform cost",use_uniform_cost_ ? "yes" : "no");
  SBPL_DEBUG_NAMED(stream,"%40s: %d", "cost per cell", cost_per_cell_);
  SBPL_DEBUG_NAMED(stream,"%40s: %.5f", "distance from goal to start using IK:",solve_for_ik_thresh_m_);
  SBPL_DEBUG_NAMED(stream,"%40s: %d", "cost from goal to start using IK:",solve_for_ik_thresh_);
  SBPL_DEBUG_NAMED(stream," ");
}

double SBPLFullBodyParams::getSmallestShoulderPanMotion()
{
  double min_pan = 360.0;
  for (int i = 0; i < num_mprims_; i++)
  {
    if(mprims_[i][0] > 0 && mprims_[i][0] < min_pan) 
      min_pan = mprims_[i][0];
  }

  min_pan = angles::normalize_angle(angles::from_degrees(min_pan));
  SBPL_INFO("Smallest shoulder pan motion is %0.3f rad.\n", min_pan);
  
  return min_pan;
}

double SBPLFullBodyParams::getLargestMotionPrimOffset()
{
  double max_offset = 0;
  for (int i = 0; i < num_mprims_; i++)
  {
    for(size_t j = 0; j < mprims_[i].size(); j++)
    {
      if(fabs(mprims_[i][j]) > max_offset)
        max_offset = fabs(mprims_[i][j]);
    }
  }

  max_offset = angles::normalize_angle(angles::from_degrees(max_offset));
  SBPL_INFO("Largest single joint offset in all Motion Prims is %0.3f rad.", max_offset);
  
  return max_offset;
}

void SBPLFullBodyParams::changeLoggerLevel(std::string name, std::string level)
{
  ROSCONSOLE_AUTOINIT;

  //std::string logger_name = ROSCONSOLE_DEFAULT_NAME + std::string(".") + name;
  std::string logger_name = name;

  ROS_INFO("[params] Setting %s to %s level", logger_name.c_str(), level.c_str());

  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(logger_name);

  // Set the logger for this package to output all statements
  if(level.compare("debug") == 0)
    my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
  else
    my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Info]);

  ROS_DEBUG_NAMED(name, "This is a debug statement, and should print if you enabled debug.");
}

/*
void SBPLFullBodyParams::addLongMotionPrimitives(MotionPrimitive m, std::vector<bool> &reflect, std::vector<int> &rotate)
{
  double theta = 0;

  //convert from discrete to continuous coordinates

  //construct transformation matrix for primitive
  
  //for each axis of rotation
  for(size_t i = 0; i < rotate.size(); ++i)
  {
    //compute angular distance between prims
    theta = (2.0*M_PI)/rotate[i];

    //for each rotation around the axis
    for(size_t j = 0; j < rotate[i]; j++)
    {
      //construct rotation matrix - theta degrees around axis i

      //rotate original motion prim

      //for each reflection about an axis
      for(size_t j = 0; j < reflect.size(); ++j)
      {

      }
    }
  }
}
*/

}
