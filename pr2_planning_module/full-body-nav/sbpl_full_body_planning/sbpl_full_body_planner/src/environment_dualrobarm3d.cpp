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
/** \author Benjamin Cohen, Maxim Likhachev */

#include <sbpl_full_body_planner/environment_dualrobarm3d.h>

#define DEG2RAD(d) ((d)*(M_PI/180.0))
#define RAD2DEG(r) ((r)*(180.0/M_PI))

#define DEBUG_SEARCH 0
#define DEBUG_HEURISTIC 0

#define DRAW_EXPANDS 0

#if DEBUG_SEARCH
  FILE* fSucc = SBPL_FOPEN("/tmp/debug_succs.txt", "w");
#endif

FILE* fMP = fopen("/tmp/debug_succs.csv", "w");

//Statistics
bool near_goal = false;
clock_t starttime;
double time_to_goal_region;

using namespace std;
namespace sbpl_full_body_planner{

EnvironmentDUALROBARM3D::EnvironmentDUALROBARM3D()
{
  wtf_[0] = 0;
  wtf_[1] = 0;
  wtf_[2] = 0;

  EnvROBARMCfg.bInitialized = false;
  save_expanded_states = true;

  params_filename_ = "./config/params.cfg";
  arm0_filename_ = "./config/pr2_right_arm.cfg"; 
  arm1_filename_ = "./config/pr2_left_arm.cfg"; 
  free_angle_index_ = 2;
  using_short_mprims_ = false;
  ndof_ = 12;
  njoints_ = 7;
  succ_stats_.resize(sbpl_arm_planner::NUM_DEBUG_CODES,0);

  arm_grid_offset_[0] = 0.60;
  arm_grid_offset_[1] = 1.24;
  arm_grid_offset_[2] = 0.80;
  useGoalID_ = true;

  //debugging
  record_data_ = false;
  incorrect_right_states_.resize(ndof_,0);
  incorrect_left_states_.resize(ndof_,0);
}

EnvironmentDUALROBARM3D::~EnvironmentDUALROBARM3D()
{
  if(cspace_ != NULL)
    delete cspace_;
  if(arm_[0] != NULL)
    delete arm_[0];
  if(arm_[1] != NULL)
    delete arm_[1];
  if(dijkstra_ != NULL)
    delete dijkstra_;
  if(grid2D != NULL){
    int dimX,dimY,dimZ;
    grid_->getGridSize(dimX, dimY, dimZ);
    for (int x = 0; x < dimX; x++)
      delete [] grid2D[x];
    delete [] grid2D;
  }
  if(grid_ != NULL)
    delete grid_;
  if(grid2Dsearchfromgoal != NULL)
    delete grid2Dsearchfromgoal;

  for(size_t i = 0; i < EnvROBARM.StateID2CoordTable.size(); i++)
  {
    delete EnvROBARM.StateID2CoordTable.at(i);
    EnvROBARM.StateID2CoordTable.at(i) = NULL;
  }
  EnvROBARM.StateID2CoordTable.clear();

  if(EnvROBARM.Coord2StateIDHashTable != NULL)
  {
    delete [] EnvROBARM.Coord2StateIDHashTable;
    EnvROBARM.Coord2StateIDHashTable = NULL;
  }

  if(fMP != NULL)
    fclose(fMP);

#if DEBUG_SEARCH
    SBPL_FCLOSE(fSucc);
#endif

#if DEBUG_HEURISTIC
    SBPL_FCLOSE(fHeur);
#endif
}

/////////////////////////////////////////////////////////////////////////////
//                      SBPL Planner Interface
/////////////////////////////////////////////////////////////////////////////

bool EnvironmentDUALROBARM3D::InitializeMDPCfg(MDPConfig *MDPCfg)
{
  MDPCfg->goalstateid = EnvROBARM.goalHashEntry->stateID;
  MDPCfg->startstateid = EnvROBARM.startHashEntry->stateID;
  return true;
}

int EnvironmentDUALROBARM3D::GetFromToHeuristic(int FromStateID, int ToStateID)
{
#if USE_HEUR==0
  return 0;
#endif

  return getEndEffectorHeuristic(FromStateID,ToStateID);
}

int EnvironmentDUALROBARM3D::GetGoalHeuristic(int stateID)
{
#if USE_HEUR==0
  return 0;
#endif

#if DEBUG_HEUR
  if(stateID >= (int)EnvROBARM.StateID2CoordTable.size())
  {
    ROS_ERROR("ERROR in EnvROBARM... function: stateID illegal");
    return -1;
  }
#endif

  return GetFromToHeuristic(stateID, EnvROBARM.goalHashEntry->stateID);
}

int EnvironmentDUALROBARM3D::GetStartHeuristic(int stateID)
{
#if USE_HEUR==0
  return 0;
#endif

#if DEBUG_HEUR
  if(stateID >= (int)EnvROBARM.StateID2CoordTable.size())
  {
    ROS_ERROR("ERROR in EnvROBARM... function: stateID illegal\n");
    return -1;
  }
#endif

  return GetFromToHeuristic(stateID, EnvROBARM.startHashEntry->stateID);
}

int EnvironmentDUALROBARM3D::SizeofCreatedEnv()
{
  return EnvROBARM.StateID2CoordTable.size();
}

void EnvironmentDUALROBARM3D::PrintState(int stateID, bool bVerbose, FILE* fOut /*=NULL*/)
{
#if DEBUG_HEUR
  if(stateID >= (int)EnvROBARM.StateID2CoordTable.size())
  {
    ROS_ERROR("ERROR in EnvROBARM... function: stateID illegal (2)\n");
    throw new SBPL_Exception();
  }
#endif

  if(fOut == NULL)
    fOut = stdout;

  EnvDUALROBARM3DHashEntry_t* HashEntry = EnvROBARM.StateID2CoordTable[stateID];

  bool bGoal = false;
  if(stateID == EnvROBARM.goalHashEntry->stateID)
    bGoal = true;

  printJointArray(fOut, HashEntry, bGoal, bVerbose);
}

void EnvironmentDUALROBARM3D::PrintEnv_Config(FILE* fOut)
{
  //implement this function if the planner needs to print out the EnvROBARM. configuration
  ROS_ERROR("ERROR in EnvROBARM... function: PrintEnv_Config is undefined\n");
  throw new SBPL_Exception();
}

void EnvironmentDUALROBARM3D::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV)
{
  GetSuccs(SourceStateID,SuccIDV,CostV,NULL);
}

void EnvironmentDUALROBARM3D::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV, vector<int>* ActionV)
{
  clock_t t0 = clock();
  bool invalid_prim = false;
  char use_prim_type=sbpl_full_body_planner::LONG_DISTANCE;
  unsigned char dist=0, dist_temp=0;
  size_t i=0, a=0, j=0, q=0;
  int motion_cost=0;
  std::vector<short unsigned int> succcoord(ndof_,0);
  std::vector<int> mp_coord(ndof_,0);
  std::vector<double> pose(6,0), angles0(njoints_,0), angles1(njoints_,0), parent_angles0(njoints_,0),parent_angles1(njoints_,0),sangles0(njoints_,0),sangles1(njoints_,0),pangles0(njoints_,0), pangles1(njoints_,0),iangles0(njoints_,0),iangles1(njoints_,1),succ_wcoord(ndof_,0), parent_wcoord(ndof_,0);

  short unsigned int xyz[3]={0},rpy[3]={0},fa[2]={0};
  BodyCell body_cell;
  double wxyz[3]={0},wrpy[3]={0},wfa[2]={0},xyz_source[3]={0},rpy_source[3]={0},fa_source[2]={0};
  BodyPose body_pose;
  BodyPose pose_source;

  //clear the successor arrays
  SuccIDV->clear();
  CostV->clear();
  SuccIDV->reserve(prms_.mp_.size());
  CostV->reserve(prms_.mp_.size());
  if(ActionV != NULL)
  {
    ActionV->clear();
    ActionV->reserve(prms_.mp_.size());
  }

  //goal state should be absorbing
  if(SourceStateID == EnvROBARM.goalHashEntry->stateID)
    return;

  //get X, Y, Z for the state
  EnvDUALROBARM3DHashEntry_t* parent = EnvROBARM.StateID2CoordTable[SourceStateID];

  //default coords of successor
  for(i = 0; i < parent->coord.size(); i++)
    succcoord[i] = parent->coord[i];

  stateIDToWorldPose(SourceStateID, xyz_source, rpy_source, fa_source, &pose_source);

  for(size_t p = 0; p < parent->angles0.size(); ++p)
  {
    parent_angles0[p] = parent->angles0[p];
    parent_angles1[p] = parent->angles1[p];
  }

#if DRAW_EXPANDS
  body_cell.x = succcoord[8];
  body_cell.y = succcoord[9];
  body_cell.z = succcoord[10];
  body_cell.theta = succcoord[11];
  discToWorldBody(body_cell, &body_pose);
  pviz_.visualizeRobot(parent_angles0, parent_angles1, body_pose, 150.0, "exp", 0);
  usleep(5000);
#endif

  int dist_to_goal = GetFromToHeuristic(SourceStateID,EnvROBARM.goalHashEntry->stateID);
  
  ROS_DEBUG_NAMED(prms_.expands_log_, "\n[Source: %d] xyz: %.3f %.3f %.3f  rpy: %.3f %.3f %.3f fa0: %.3f %.3f heur: %d",SourceStateID,xyz_source[0],xyz_source[1],xyz_source[2],rpy_source[0],rpy_source[1],rpy_source[2],fa_source[0],fa_source[1], dist_to_goal);

#if DRAW_EXPANDS
  //printf("[env] [%d] object %d %d %d %d  xyz=(%d %d %d) rpy=(%d %d %d) fa=(%d %d) body=(%d %d %d %d) heur: %2d goal_yaw: %2d\n",SourceStateID, parent->object_pose[0],parent->object_pose[1],parent->object_pose[2],parent->object_pose[3],parent->coord[0],parent->coord[1],parent->coord[2],parent->coord[3],parent->coord[4],parent->coord[5],parent->coord[6],parent->coord[7],parent->coord[8],parent->coord[9],parent->coord[10],parent->coord[11],dist_to_goal, EnvROBARM.goalHashEntry->object_pose[3]);
#endif
  
  clock_t arms_t0 = clock();
  if(!prms_.nav3d_){
    //precalculate the difference to the goal orientation to create an adaptive mprim
    if(dist_to_goal <= prms_.cost_per_cell_)
    {

      prms_.mp_[prms_.mp_.size()-1].m[0][0] = EnvROBARMCfg.goal.xyz[0] - xyz_source[0];
      prms_.mp_[prms_.mp_.size()-1].m[0][1] = EnvROBARMCfg.goal.xyz[1] - xyz_source[1];
      prms_.mp_[prms_.mp_.size()-1].m[0][2] = EnvROBARMCfg.goal.xyz[2] - xyz_source[2];

      prms_.mp_[prms_.mp_.size()-1].m[0][3] = angles::normalize_angle(double(EnvROBARM.goalHashEntry->coord[3] - parent->coord[3])*EnvROBARMCfg.coord_delta[3]);
      prms_.mp_[prms_.mp_.size()-1].m[0][4] = angles::normalize_angle(double(EnvROBARM.goalHashEntry->coord[4] - parent->coord[4])*EnvROBARMCfg.coord_delta[4]);
      prms_.mp_[prms_.mp_.size()-1].m[0][5] = angles::normalize_angle(double(EnvROBARM.goalHashEntry->coord[5] - parent->coord[5])*EnvROBARMCfg.coord_delta[5]);

      prms_.mp_[prms_.mp_.size()-1].coord[0] = EnvROBARM.goalHashEntry->coord[0] - parent->coord[0];
      prms_.mp_[prms_.mp_.size()-1].coord[1] = EnvROBARM.goalHashEntry->coord[1] - parent->coord[1];
      prms_.mp_[prms_.mp_.size()-1].coord[2] = EnvROBARM.goalHashEntry->coord[2] - parent->coord[2];
      prms_.mp_[prms_.mp_.size()-1].coord[3] = EnvROBARM.goalHashEntry->coord[3] - parent->coord[3];
      prms_.mp_[prms_.mp_.size()-1].coord[4] = EnvROBARM.goalHashEntry->coord[4] - parent->coord[4];
      prms_.mp_[prms_.mp_.size()-1].coord[5] = EnvROBARM.goalHashEntry->coord[5] - parent->coord[5];

      ROS_DEBUG_NAMED(prms_.expands_log_, "[parent: %d] Adaptive MP: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f Coord: %d %d %d (dist: %d)",SourceStateID,prms_.mp_[prms_.mp_.size()-1].m[0][0],prms_.mp_[prms_.mp_.size()-1].m[0][1],prms_.mp_[prms_.mp_.size()-1].m[0][2],prms_.mp_[prms_.mp_.size()-1].m[0][3],prms_.mp_[prms_.mp_.size()-1].m[0][4],prms_.mp_[prms_.mp_.size()-1].m[0][5],prms_.mp_[prms_.mp_.size()-1].m[0][6],prms_.mp_[prms_.mp_.size()-1].m[0][7], prms_.mp_[prms_.mp_.size()-1].coord[3], prms_.mp_[prms_.mp_.size()-1].coord[4], prms_.mp_[prms_.mp_.size()-1].coord[5], dist_to_goal);
      ROS_DEBUG_NAMED(prms_.expands_log_, "[env] [parentid: %d] adaptive: xyz: %0.3f %0.3f %0.3f rpy: %0.3f %0.3f %0.3f fa: %0.3f %0.3f\n          [coord] xyz: %d %d %d rpy: %d %d %d (dist to goal: %d)",SourceStateID,prms_.mp_[prms_.mp_.size()-1].m[0][0],prms_.mp_[prms_.mp_.size()-1].m[0][1],prms_.mp_[prms_.mp_.size()-1].m[0][2],prms_.mp_[prms_.mp_.size()-1].m[0][3],prms_.mp_[prms_.mp_.size()-1].m[0][4],prms_.mp_[prms_.mp_.size()-1].m[0][5],prms_.mp_[prms_.mp_.size()-1].m[0][6],prms_.mp_[prms_.mp_.size()-1].m[0][7], prms_.mp_[prms_.mp_.size()-1].coord[0], prms_.mp_[prms_.mp_.size()-1].coord[1], prms_.mp_[prms_.mp_.size()-1].coord[2], prms_.mp_[prms_.mp_.size()-1].coord[3], prms_.mp_[prms_.mp_.size()-1].coord[4], prms_.mp_[prms_.mp_.size()-1].coord[5], dist_to_goal);
    }

    //iterate through successors of source state
    for (i = 0; i < prms_.mp_.size(); i++)
    {
      //if adaptive mprim, are we at goal yet?
      if(prms_.mp_[i].type == sbpl_full_body_planner::ADAPTIVE)
      {
        if(dist_to_goal > prms_.cost_per_cell_)
          continue;
      }
      //is primitive of right type? (short or long distance) 
      else if(prms_.mp_[i].type != use_prim_type)
        continue;

      dist = 100;
      invalid_prim = false;
      motion_cost = 0;

      for(q = 0; q < parent->coord.size(); ++q)
        succcoord[q] = parent->coord[q];

      iangles0 = pangles0 = parent_angles0;
      iangles1 = pangles1 = parent_angles1;

      coordToWorldPose(parent->coord, parent_wcoord);

      mp_coord = prms_.mp_[i].coord;

      //get the successor
      EnvDUALROBARM3DHashEntry_t* OutHashEntry;
      bool bSuccisGoal = false;

      //loop through the waypoints in the motion primitive
      for(j = 0; j < prms_.mp_[i].m.size(); ++j)
      {
        dist_temp = 100;
        debug_code_ = sbpl_arm_planner::SUCCESS;
        right_ik_search_ = 0;
        left_ik_search_ = 0;

        //add the motion primitive to the current joint configuration
        for(a = 0; a < prms_.mp_[i].m[j].size(); ++a)
          succ_wcoord[a] = parent_wcoord[a] + prms_.mp_[i].m[j][a];

        ROS_DEBUG_NAMED(prms_.expands2_log_,"[%d] -> [%d-%d] xyz: %0.3f %0.3f %0.3f rpy: %0.3f %0.3f %0.3f fa0: %0.3f fa1: %0.3f", SourceStateID, int(i), int(j), succ_wcoord[0],succ_wcoord[1],succ_wcoord[2],succ_wcoord[3],succ_wcoord[4],succ_wcoord[5],succ_wcoord[6], succ_wcoord[7]);

        //seed IK or pass on configuration
        sangles0 = parent->angles0;
        sangles1 = parent->angles1;

        sangles0[2] = succ_wcoord[6];
        sangles1[2] = succ_wcoord[7];

        clock_t ik_t0 = clock();
        //if x,y,z,r,p,y is modified
        if(prms_.mp_[i].group != 2)
        {
          //run IK - does solution exist?
          if(!convertWorldPoseToAngles(succ_wcoord, sangles0, sangles1, true))
          {
            ROS_DEBUG_NAMED(prms_.expands_log_, " succ: %2d Failed to convert coords into angles. IK failed.", int(i));
            ROS_DEBUG_NAMED(prms_.expands2_log_, "[%d] -> [%d-%d] Failed to convert coords into angles. IK failed.", SourceStateID, int(i), int(j));
            invalid_prim = true;
            clock_t ik_t1 = clock();
            ik_time += ik_t1-ik_t0;
            break;
          }
        }
        //don't allow search over free angle
        else
        {
          //run IK - does solution exist?
          if(!convertWorldPoseToAngles(succ_wcoord, sangles0, sangles1, false))
          {
            ROS_DEBUG_NAMED(prms_.expands_log_, " succ: %2d Failed to convert coords into angles. IK failed. (group: 2)", int(i));
            ROS_DEBUG_NAMED(prms_.expands2_log_, "[%d] -> [%d-%d] Failed to convert coords into angles. IK failed. (group: 2)", SourceStateID, int(i), int(j));
            invalid_prim = true;
            clock_t ik_t1 = clock();
            ik_time += ik_t1-ik_t0;
            break;
          }
        }

        //ik search found solution for either arm with different free angle
        //Nothing bad happens because this runs on mprims not in group 2
        wfa[0] = sangles0[free_angle_index_];
        wfa[1] = sangles1[free_angle_index_];
        worldToDiscFAngle(wfa, fa);


        //check attached object for collision
        if(!cspace_->isAttachedObjectValid(sangles1, sangles0, pose_source, prms_.verbose_, dist_temp, debug_code_))
        {
          ROS_DEBUG_NAMED(prms_.expands_log_, " succ: %2d  dist: %2d attached object is in collision.", int(i), int(dist_temp));
          invalid_prim = true;
          break;
        }
        else
        ROS_DEBUG_NAMED(prms_.expands_log_, " succ: %2d  dist: %2d attached object is NOT in collision.", int(i), int(dist_temp));

        //check for collisions
        if(!cspace_->checkCollision(sangles1, sangles0, pose_source, true, dist_temp, debug_code_))
        {
          ROS_DEBUG_NAMED(prms_.expands_log_, " succ: %2d  dist: %2d is in collision.", int(i), int(dist_temp));
          invalid_prim = true;
          break;
        }

        //check for collision along interpolated path between sourcestate and succ
        //if(!cspace_->checkPathForCollision(iangles0, sangles0, iangles1, sangles1, pose_source, true, dist_temp, debug_code_))
        //{
        //ROS_DEBUG_NAMED(prms_.expands_log_, " right: {%0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f} -> {%0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f}", iangles0[0],iangles0[1],iangles0[2],iangles0[3],iangles0[4],iangles0[5],iangles0[6], sangles0[0],sangles0[1],sangles0[2],sangles0[3],sangles0[4],sangles0[5],sangles0[6]);
        //ROS_DEBUG_NAMED(prms_.expands_log_, "  left: {%0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f} -> {%0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f}", iangles1[0],iangles1[1],iangles1[2],iangles1[3],iangles1[4],iangles1[5],iangles1[6], sangles1[0],sangles1[1],sangles1[2],sangles1[3],sangles1[4],sangles1[5],sangles1[6]);
        //ROS_DEBUG_NAMED(prms_.expands_log_, " [yes] succ: %2d  dist: %2d is in collision along interpolated path.", int(i), int(dist_temp));
        //invalid_prim = true;
        //break;
        //}

        if(dist_temp < dist)
          dist = dist_temp;

        // add up motion cost for all waypoints in mprim
        //motion_cost += max(computeMotionCost(iangles0, sangles0, 0),computeMotionCost(iangles1, sangles1, 1));

        //ROS_DEBUG("parentid: %d  mprim.group: %d  mprim.id: %d  cost: %d (right arm: %d, left arm: %d)", SourceStateID, prms_.mp_[i].group, prms_.mp_[i].id, motion_cost, computeMotionCost(iangles0, sangles0), computeMotionCost(iangles1, sangles1));

        //save joint angles of current waypoint for next waypoint in mprim
        iangles0 = sangles0;
        iangles1 = sangles1;

        ROS_DEBUG_NAMED(prms_.expands2_log_, "     [%d-%d] xyz: %0d %d %d rpy: %d %d %d fa0: %d fa1: %d", int(i), int(j), succcoord[0],succcoord[1],succcoord[2],succcoord[3],succcoord[4],succcoord[5],succcoord[6],succcoord[7]);
      }

      updateSuccStats(debug_code_);

      if(invalid_prim)
        continue;

      //HACK using euclidean distance to compute the cost
      motion_cost =  getEuclideanDistance(succ_wcoord[0],succ_wcoord[1],succ_wcoord[2],parent_wcoord[0],parent_wcoord[1],parent_wcoord[2]) * prms_.cost_per_cell_ * 50.0;
      if(motion_cost == 0)
        motion_cost = 30;

      //compute the successor coords
      //FIXME: only 3-7 (inclusive) should wrap around
      for(a = 0; a < mp_coord.size(); ++a)
      { 
        if((succcoord[a] + mp_coord[a]) < 0)
          succcoord[a] = ((EnvROBARMCfg.coord_vals[a] + succcoord[a] + int(mp_coord[a])) % EnvROBARMCfg.coord_vals[a]);
        else
          succcoord[a] = ((int)(succcoord[a] + int(mp_coord[a])) % EnvROBARMCfg.coord_vals[a]);
      }

      if(succcoord[6] != fa[0])
      {
        ROS_DEBUG_NAMED(prms_.expands_log_,"  Right arm free angle changed from: %0.3frad (%d)  to: %.3frad (%d)",succ_wcoord[6],succcoord[6], sangles0[free_angle_index_], fa[0]);
        succcoord[6] = fa[0];
      }

      if(succcoord[7] != fa[1])
      {
        ROS_DEBUG_NAMED(prms_.expands_log_,"  Left arm free angle changed from: %0.3frad (%d)  to: %.3frad (%d)",succ_wcoord[7],succcoord[7], sangles1[free_angle_index_], fa[1]);
        succcoord[7] =fa[1];
      }

      ROS_DEBUG_NAMED(prms_.expands2_log_, "[parent id: %d mprim: %d] parent: %d %d %d %d %d %d %d %d  (mprim: %d %d %d %d %d %d %d %d)", SourceStateID, int(i), parent->coord[0], parent->coord[1],parent->coord[2],parent->coord[3],parent->coord[4],parent->coord[5],parent->coord[6],parent->coord[7], int(mp_coord[0]), int(mp_coord[1]), int(mp_coord[2]), int(mp_coord[3]), int(mp_coord[4]), int(mp_coord[5]),int(mp_coord[6]),int(mp_coord[7]));
      ROS_DEBUG_NAMED(prms_.expands2_log_, "[parent id: %d mprim: %d]  child: %d %d %d %d %d %d %d %d  (mprim: %d %d %d %d %d %d %d %d) cost: %d", SourceStateID, int(i), succcoord[0],succcoord[1],succcoord[2], succcoord[3], succcoord[4], succcoord[5],succcoord[6],succcoord[7],int(mp_coord[0]), int(mp_coord[1]), int(mp_coord[2]), int(mp_coord[3]),int(mp_coord[4]),int(mp_coord[5]),int(mp_coord[6]),int(mp_coord[7]), motion_cost);

      // write a function that makes this conversion smarter 
      coordToPose(succcoord,xyz,rpy,fa,&body_cell);
      discToWorldXYZ(xyz,wxyz,false);
      discToWorldRPY(rpy,wrpy);
      discToWorldFAngle(fa,wfa);
      discToWorldBody(body_cell,&body_pose);

      clock_t fk_t0 = clock();
      short unsigned int obj_x,obj_y,obj_z,obj_yaw;
      if(!computeObjectPose(body_pose,parent->angles0,obj_x,obj_y,obj_z,obj_yaw))
        ROS_ERROR("[env] computeObjectPose failed...");
      ROS_DEBUG("[env] fresh parent object pose=(%d %d %d %d)",obj_x,obj_y,obj_z,obj_yaw);

      if(!computeObjectPose(body_pose,sangles0,obj_x,obj_y,obj_z,obj_yaw))
        ROS_ERROR("[env] computeObjectPose failed...");
      clock_t fk_t1 = clock();
      fk_time += fk_t1-fk_t0;

      ROS_DEBUG("[env] child object pose=(%d %d %d %d)",obj_x,obj_y,obj_z,obj_yaw);

      //check if this state meets the goal criteria
      if(isGoalPosition(obj_x,obj_y,obj_z,obj_yaw))
      {
        bSuccisGoal = true;
        ROS_INFO("[env] Goal state has been found. Parent StateID: %d (obstacle distance: %d)",SourceStateID,int(dist));
        ROS_INFO("[env]   coord: %d %d %d %d %d %d %d %d",succcoord[0],succcoord[1],succcoord[2],succcoord[3],succcoord[4],succcoord[5],succcoord[6],succcoord[7]);
      }

      clock_t hash_t0 = clock();
      //check if hash entry already exists, if not then create one
      if((OutHashEntry = getHashEntry(succcoord, bSuccisGoal)) == NULL)
      {
        OutHashEntry = createHashEntry(succcoord);
        OutHashEntry->dist = dist;

        OutHashEntry->angles0 = sangles0;
        OutHashEntry->angles1 = sangles1;

        OutHashEntry->object_pose[0] = obj_x;
        OutHashEntry->object_pose[1] = obj_y;
        OutHashEntry->object_pose[2] = obj_z;
        OutHashEntry->object_pose[3] = obj_yaw;

        if(prms_.mp_[i].type == sbpl_full_body_planner::ADAPTIVE)
          motion_cost = 1;
        else
          motion_cost += cost(parent,OutHashEntry,bSuccisGoal);

        ROS_DEBUG_NAMED(prms_.expands_log_, " succ: %2d  parentid: %d  stateid: %d  mprim.group: %d  mprim.id: %d  cost: %d heur: %d  xyz: %d %d %d  rpy: %d %d %d  fa0: %d fa1: %d  dist: %d", int(i),SourceStateID, OutHashEntry->stateID, prms_.mp_[i].group, prms_.mp_[i].id,  motion_cost, GetFromToHeuristic(OutHashEntry->stateID, EnvROBARM.goalHashEntry->stateID), OutHashEntry->coord[0],OutHashEntry->coord[1],OutHashEntry->coord[2], OutHashEntry->coord[3],OutHashEntry->coord[4],OutHashEntry->coord[5],OutHashEntry->coord[6],OutHashEntry->coord[7],int(dist));
        ROS_DEBUG_NAMED(prms_.expands_log_,"             object_pose: %d %d %d %d", OutHashEntry->object_pose[0], OutHashEntry->object_pose[1], OutHashEntry->object_pose[2], OutHashEntry->object_pose[3]); 
      }
      else
        ROS_DEBUG_NAMED(prms_.expands_log_, " succ: %2d  already in graph.", int(i));
      clock_t hash_t1 = clock();

      hash_time += hash_t1-hash_t0;

      //put successor on successor list with the proper cost
      SuccIDV->push_back(OutHashEntry->stateID);

      if(prms_.mp_[i].type == sbpl_full_body_planner::ADAPTIVE)
        CostV->push_back(1);
      else
        CostV->push_back(motion_cost + cost(parent,OutHashEntry,bSuccisGoal));

      if(ActionV != NULL)
        ActionV->push_back(i);
    }
  }
  clock_t arms_t1 = clock();


  clock_t base_t0 = clock();
  ROS_DEBUG_NAMED(prms_.expands_log_, "-----------Base Motion Primitives---------");

  //now iterate through base actions
  for(unsigned int i=0; i<parent->coord.size(); i++)
    succcoord[i] = parent->coord[i];
  for(int aind=0; aind < prms_.base_action_width_+prms_.num_orbit_motions_; aind++){
    EnvNAVXYTHETALATAction_t nav3daction;
    if(aind==prms_.base_action_width_){
      if(!computeOrbitMotion(1,parent,&nav3daction))
        continue;
    }
    else if(aind==prms_.base_action_width_+1){
      if(!computeOrbitMotion(-1,parent,&nav3daction))
        continue;
    }
    else if(aind==prms_.base_action_width_+2){
      if(dist_to_goal < prms_.cost_per_cell_){
        //adaptive base motion
        if(!computeAdaptiveBaseMotion(parent,&nav3daction))
          continue;
      }
      else
        continue;
    }
    else{
      nav3daction = prms_.base_actions_[(unsigned int)parent->coord[11]][aind];
    }

    int newX = parent->coord[8] + nav3daction.dX;
    int newY = parent->coord[9] + nav3daction.dY;
    int newTheta = NORMALIZEDISCTHETA(nav3daction.endtheta, prms_.num_base_dirs_);

    succcoord[8] = newX;
    succcoord[9] = newY;
    succcoord[11] = newTheta;

    //collision check
    bool inCollision = false;
    dist_temp = 100; // padding
    for(unsigned int i=0; i < nav3daction.intermptV.size(); i++){
      sbpl_xy_theta_pt_t pt = nav3daction.intermptV.at(i);

      BodyCell bc(parent->coord[8], parent->coord[9], parent->coord[10], 0);
      BodyPose body_pose;
      discToWorldBody(bc,&body_pose);
      body_pose.x += pt.x;
      body_pose.y += pt.y;
      body_pose.theta = pt.theta;

      //first bounds check the bad boy
      if(!boundsCheckBase(body_pose)){
ROS_ERROR("bound check fail");
        inCollision = true;
        break;
      }

      
      if(!cspace_->checkBaseMotion(parent->angles1, parent->angles0, body_pose, prms_.verbose_, dist_temp /* padding */, debug_code_))
      {
        ROS_DEBUG_NAMED(prms_.expands_log_, " [env] collision (dist: %d)", int(dist_temp));
        inCollision = true;
        break;
      }

        // padding
      if (dist_temp < dist) {
        dist = dist_temp;
      }
    }
    BodyPose viz_pose;
    BodyCell viz_cell(succcoord[8],succcoord[9],succcoord[10], succcoord[11]);
    discToWorldBody(viz_cell,&viz_pose);
    if(inCollision)
    {
      /*
      if(aind >= prms_.base_action_width_){
        printf("\n\n inCollision %d (%d %d %d %d)\n",aind,viz_cell.x,viz_cell.y,viz_cell.z,viz_cell.theta);
        printf("\n\n inCollision %d (%f %f %f %f)\n",aind,viz_pose.x,viz_pose.y,viz_pose.z,viz_pose.theta);
        pviz_.visualizeRobot(parent->angles0, parent->angles1, viz_pose, 10,"adaptive_base", 0);
        usleep(10000);
      }
      */
      continue;
    }
    /*
    else
    {
      if(aind >= prms_.base_action_width_){
        printf("\n\n not in Collision %d (%d %d %d %d)\n",aind,viz_cell.x,viz_cell.y,viz_cell.z,viz_cell.theta);
        printf("\n\n not in Collision %d (%f %f %f %f)\n",aind,viz_pose.x,viz_pose.y,viz_pose.z,viz_pose.theta);
        pviz_.visualizeRobot(parent->angles0, parent->angles1, viz_pose, 200,"adaptive_base", 0);
        usleep(10000);
      }
    }
    */

    //compute the object pose
    BodyCell bc(succcoord[8], succcoord[9], succcoord[10], succcoord[11]);
    BodyPose body_pose;
    discToWorldBody(bc,&body_pose);
    short unsigned int obj_x,obj_y,obj_z,obj_yaw;
    computeObjectPose(body_pose,parent->angles0,obj_x,obj_y,obj_z,obj_yaw);

    //if(aind==prms_.base_action_width_)
      //printf("  object %d %d %d %d\n",obj_x,obj_y,obj_z,obj_yaw);

    //check if this is a goal
    bool bSuccisGoal = !prms_.nav3d_ && isGoalPosition(obj_x,obj_y,obj_z,obj_yaw);

    EnvDUALROBARM3DHashEntry_t* OutHashEntry;
    if((OutHashEntry = getHashEntry(succcoord, bSuccisGoal)) == NULL){
      OutHashEntry = createHashEntry(succcoord);

      OutHashEntry->angles0 = parent->angles0;
      OutHashEntry->angles1 = parent->angles1;

      OutHashEntry->object_pose[0] = obj_x;
      OutHashEntry->object_pose[1] = obj_y;
      OutHashEntry->object_pose[2] = obj_z;
      OutHashEntry->object_pose[3] = obj_yaw;
    }

    ROS_DEBUG_NAMED(prms_.expands_log_, " succ: %2d  parentid: %d  stateid: %d  cost: %d heur: %d  base_xyz: %d %d %d  theta: %d", int(aind+prms_.mp_.size()),SourceStateID, OutHashEntry->stateID, nav3daction.cost, GetFromToHeuristic(OutHashEntry->stateID, EnvROBARM.goalHashEntry->stateID), OutHashEntry->coord[8],OutHashEntry->coord[9],OutHashEntry->coord[10], OutHashEntry->coord[11]);
    ROS_DEBUG_NAMED(prms_.expands_log_,"             object_pose: %d %d %d %d", OutHashEntry->object_pose[0], OutHashEntry->object_pose[1], OutHashEntry->object_pose[2], OutHashEntry->object_pose[3]); 

    SuccIDV->push_back(OutHashEntry->stateID);
//    CostV->push_back(nav3daction.cost * max(3 - dist, 1)); // padding
    CostV->push_back(nav3daction.cost); // padding
    if(ActionV != NULL)
      ActionV->push_back(prms_.mp_.size()+aind);
  }
  clock_t base_t1 = clock();


  clock_t torso_t0 = clock();
  if(!prms_.nav3d_){
    ROS_DEBUG_NAMED(prms_.expands_log_, "-----------Torso Motion Primitives---------");

    //finally, try to move the spine up and down
    for(unsigned int i=0; i<parent->coord.size(); i++)
      succcoord[i] = parent->coord[i];
    int count = -1;
    for(int i=-1; i<=1; i+=2){
      count++;

      succcoord[10] = parent->coord[10] + i;

      //collision check
      BodyCell bc(parent->coord[8], parent->coord[9], parent->coord[10]+i, parent->coord[11]);
      BodyPose body_pose;
      discToWorldBody(bc,&body_pose);

      //first bounds check the bad boy
      if(body_pose.z<0 || body_pose.z>0.31)
        continue;

      if(!cspace_->checkSpineMotion(parent->angles1, parent->angles0, body_pose, prms_.verbose_, dist, debug_code_))
        continue;

      //compute the object pose
      short unsigned int obj_x = parent->object_pose[0];
      short unsigned int obj_y = parent->object_pose[1];
      short unsigned int obj_z = parent->object_pose[2]+i;
      short unsigned int obj_yaw = parent->object_pose[3];

      //check if this is a goal
      bool bSuccisGoal = isGoalPosition(obj_x,obj_y,obj_z,obj_yaw);

      EnvDUALROBARM3DHashEntry_t* OutHashEntry;
      if((OutHashEntry = getHashEntry(succcoord, bSuccisGoal)) == NULL){
        OutHashEntry = createHashEntry(succcoord);

        OutHashEntry->angles0 = parent->angles0;
        OutHashEntry->angles1 = parent->angles1;

        OutHashEntry->object_pose[0] = obj_x;
        OutHashEntry->object_pose[1] = obj_y;
        OutHashEntry->object_pose[2] = obj_z;
        OutHashEntry->object_pose[3] = obj_yaw;
      }

      ROS_DEBUG_NAMED(prms_.expands_log_, " succ: %2d  parentid: %d  stateid: %d  cost: %d heur: %d  base_xyz: %d %d %d  theta: %d", int(count+prms_.base_action_width_+prms_.mp_.size()+3),SourceStateID, OutHashEntry->stateID, prms_.spine_cost_, GetFromToHeuristic(OutHashEntry->stateID, EnvROBARM.goalHashEntry->stateID), OutHashEntry->coord[8],OutHashEntry->coord[9],OutHashEntry->coord[10], OutHashEntry->coord[11]);
      ROS_DEBUG_NAMED(prms_.expands_log_,"             object_pose: %d %d %d %d", OutHashEntry->object_pose[0], OutHashEntry->object_pose[1], OutHashEntry->object_pose[2], OutHashEntry->object_pose[3]); 

      SuccIDV->push_back(OutHashEntry->stateID);
      CostV->push_back(prms_.spine_cost_);
      if(ActionV != NULL)
        ActionV->push_back(prms_.mp_.size()+prms_.base_action_width_+count+prms_.num_orbit_motions_);
    }
  }
  clock_t torso_t1 = clock();

  if(save_expanded_states)
    expanded_states.push_back(SourceStateID);

  clock_t t1 = clock();

  succ_time += t1-t0;
  arms_time += arms_t1-arms_t0;
  base_time += base_t1-base_t0;
  torso_time += torso_t1-torso_t0;
}

bool EnvironmentDUALROBARM3D::boundsCheckBase(BodyPose pose){
  double r = 0.325;
  double dx[4] = {r,r,-r,-r};
  double dy[4] = {r,-r,-r,r};
  double ca = cos(pose.theta);
  double sa = sin(pose.theta);
  for(int i=0; i<4; i++){
    double x = ca*dx[i] - sa*dy[i] + pose.x;
    double y = sa*dx[i] + ca*dy[i] + pose.y;
    //printf("x=%f y=%f\n",x,y);
    if(x<prms_.originX_ || y<prms_.originY_ || x>=prms_.sizeX_-prms_.originX_ || y>=prms_.sizeY_-prms_.originY_)
	 {
		ROS_WARN("(x=%f, y=%f), (oX=%f,oY=%f), (sX=%f, sY=%f)", x,y,prms_.originX_ ,prms_.originY_, prms_.sizeX_, prms_.sizeY_);
      return false;
	 }
  }
  return true;
}

inline double sign(double x){
  return (x > 0) - (x < 0);
}

bool EnvironmentDUALROBARM3D::computeAdaptiveBaseMotion(EnvDUALROBARM3DHashEntry_t* parent, EnvNAVXYTHETALATAction_t* action){
  //printf("object_pose: %d %d %d %d\n", parent->object_pose[0], parent->object_pose[1], parent->object_pose[2], parent->object_pose[3]); 
  //first compute the object pose if the base is at 0,0 with the proper yaw
  double w_body_yaw = angles::normalize_angle(double(EnvROBARM.goalHashEntry->object_pose[3] - parent->coord[5])*EnvROBARMCfg.coord_delta[5]);
  //printf("w_body_yaw=%f\n",w_body_yaw);
  
  int body_yaw = (int)((angles::normalize_angle_positive(w_body_yaw + EnvROBARMCfg.coord_delta[11]*0.5))/EnvROBARMCfg.coord_delta[11]);
  //printf("body_yaw=%d\n",body_yaw);

  short unsigned int obj_x, obj_y, obj_z, obj_yaw;
  BodyCell bc(0,0,0,body_yaw);
  BodyPose body_pose;
  discToWorldBody(bc,&body_pose);
  
  computeObjectPose(body_pose,parent->angles0,obj_x,obj_y,obj_z,obj_yaw);
  //printf("object %d %d %d %d\n",obj_x,obj_y,obj_z,obj_yaw);

  //then apply the proper translation to put the object at the goal
  action->dX = EnvROBARM.goalHashEntry->object_pose[0] - obj_x - parent->coord[8];
  action->dY = EnvROBARM.goalHashEntry->object_pose[1] - obj_y - parent->coord[9];
  action->endtheta = body_yaw;

  //we don't use the cell list
  action->interm3DcellsV.clear();
  action->intermptV.clear();

  //compute each of the intermediate points and add them to the motion
  //to do this we will trace the shortest arc from the parent to the target
  KDL::Frame to_wrist;

  if(!arm_[0]->computeFK(parent->angles0,body_pose,10,&to_wrist))
    ROS_ERROR("base adaptive motion FK failed...");
  KDL::Frame f = arm0_offset_.Inverse() * to_wrist;
  double radius = sqrt(f.p.x()*f.p.x()+f.p.y()*f.p.y());

  double final_theta = body_pose.theta;
  //and the shortest angle 
  double parent_theta = angles::normalize_angle(double(parent->coord[11])*EnvROBARMCfg.coord_delta[11]);
  //printf("parent theta=%f desired theta=%f\n",parent_theta,body_pose.theta);
  double angle = shortest_angular_distance(parent_theta, body_pose.theta);
  if(angle==0)
    return false;

  double temp_x, temp_y, temp_z;
  temp_x = 0;
  temp_y = 0;
  double angle_step = 2.0*M_PI/180 * sign(angle);
  //printf("angle=%f step=%f\n",angle,angle_step);
  for(double a=parent_theta; sign(shortest_angular_distance(a,final_theta))==sign(angle); a=angles::normalize_angle(a+angle_step)){
    sbpl_xy_theta_pt_t intermpt;

    body_pose.theta = a;
    if(!arm_[0]->computeFK(parent->angles0,body_pose,10,&to_wrist))
      ROS_ERROR("base adaptive motion FK failed...");
    KDL::Frame f = arm0_offset_.Inverse() * to_wrist;

    intermpt.x = -f.p.x();
    intermpt.y = -f.p.y();

    if(a==parent_theta){
      //if this is the first iteration, save the x and y as the origin for the other points
      temp_x = intermpt.x;
      temp_y = intermpt.y;
    }
    intermpt.x -= temp_x;
    intermpt.y -= temp_y;
    intermpt.theta = a;
    
    /*
    double blah_x, blah_y, blah_z;
    discToWorldXYZ(parent->coord[8],parent->coord[9],0,blah_x,blah_y,blah_z,true);
    std::vector<double> p(6,0);
    p[0] = intermpt.x+blah_x;
    p[1] = intermpt.y+blah_y;
    pviz_.visualizeSphere(p,139,"intermediate_pose_"+boost::lexical_cast<std::string>(a), 0.02);usleep(5000);
    printf("a=%f (%f %f %f)\n",a,intermpt.x,intermpt.y,intermpt.theta);
    */
    
    action->intermptV.push_back(intermpt);

    /*
    BodyPose pPose(p[0],p[1],0,a);
    computeObjectPose(pPose,parent->angles0,obj_x,obj_y,obj_z,obj_yaw);
    discToWorldXYZ(obj_x,obj_y,0,blah_x,blah_y,blah_z,true);
    printf("obj %d %d %d %d\n",obj_x,obj_y,obj_z,obj_yaw);
    p[0]=blah_x;
    p[1]=blah_y;
    pviz_.visualizeSphere(p,60,"object_pose_"+boost::lexical_cast<std::string>(a), 0.02);usleep(5000);
    */
  }
  /*
  std::vector<double> p(6,0);
  double blah_x, blah_y, blah_z;
  discToWorldXYZ(parent->coord[8],parent->coord[9],0,blah_x,blah_y,blah_z,true);
  p[0] = action->intermptV.back().x + blah_x;
  p[1] = action->intermptV.back().y + blah_y;
  p[2] = action->intermptV.back().theta;
  pviz_.visualizeRobot(parent->angles0, parent->angles1, p, 0, 250,"adaptive_base", 0);usleep(5000);
  */

  //add the final point (where the robot ends up) to the motion
  sbpl_xy_theta_pt_t intermpt;
  discToWorldXYZ(action->dX,action->dY,0,intermpt.x,intermpt.y,temp_z,true);
  intermpt.theta = body_pose.theta;
  //printf("(%f %f %f)\n",intermpt.x,intermpt.y,intermpt.theta);
  action->intermptV.push_back(intermpt);
  
  //compute the cost (max of translation time and rotation time)
  double translation_time = fabs(angle)*radius/prms_.nominalvel_mpersecs_;
  double rotation_time = fabs(angle)/((PI_CONST/4.0)/prms_.timetoturn45degsinplace_secs_);
  action->cost = (int)(ceil(NAVXYTHETALAT_COSTMULT_MTOMM/10*max(translation_time, rotation_time)));

  //printf("using adaptive base motion %d %d %d\n",action->dX,action->dY,action->endtheta);
  //printf("cost=%d",action->cost);
  return true;
}

bool EnvironmentDUALROBARM3D::computeOrbitMotion(int dir, EnvDUALROBARM3DHashEntry_t* parent, EnvNAVXYTHETALATAction_t* action){
  int body_yaw = parent->coord[11] + dir;
  if(body_yaw<0)
    body_yaw = prms_.num_base_dirs_;
  else if(body_yaw >= prms_.num_base_dirs_)
    body_yaw = 0;

  short unsigned int obj_x, obj_y, obj_z, obj_yaw;
  BodyCell bc(0,0,0,body_yaw);
  BodyPose body_pose;
  discToWorldBody(bc,&body_pose);
  
  computeObjectPose(body_pose,parent->angles0,obj_x,obj_y,obj_z,obj_yaw);
  //printf("object %d %d %d %d\n",obj_x,obj_y,obj_z,obj_yaw);

  //then apply the proper translation
  action->dX = parent->object_pose[0] - obj_x - parent->coord[8];
  action->dY = parent->object_pose[1] - obj_y - parent->coord[9];
  action->endtheta = body_yaw;
  //printf("\n\n  %d %d\n\n",action->dX,action->dY);

  //we don't use the cell list
  action->interm3DcellsV.clear();
  action->intermptV.clear();

  //compute each of the intermediate points and add them to the motion
  //to do this we will trace the shortest arc from the parent to the target
  KDL::Frame to_wrist;

  if(!arm_[0]->computeFK(parent->angles0,body_pose,10,&to_wrist))
    ROS_ERROR("base adaptive motion FK failed...");
  KDL::Frame f = arm0_offset_.Inverse() * to_wrist;
  double radius = sqrt(f.p.x()*f.p.x()+f.p.y()*f.p.y());

  double final_theta = body_pose.theta;
  //and the shortest angle 
  double parent_theta = angles::normalize_angle(double(parent->coord[11])*EnvROBARMCfg.coord_delta[11]);
  //printf("parent theta=%f desired theta=%f\n",parent_theta,body_pose.theta);
  double angle = shortest_angular_distance(parent_theta, body_pose.theta);
  if(angle==0)
    return false;

  double temp_x, temp_y, temp_z;
  temp_x = 0;
  temp_y = 0;
  double angle_step = 2.0*M_PI/180 * sign(angle);
  //printf("angle=%f step=%f\n",angle,angle_step);
  for(double a=parent_theta; sign(shortest_angular_distance(a,final_theta))==sign(angle); a=angles::normalize_angle(a+angle_step)){
    sbpl_xy_theta_pt_t intermpt;

    body_pose.theta = a;
    if(!arm_[0]->computeFK(parent->angles0,body_pose,10,&to_wrist))
      ROS_ERROR("base adaptive motion FK failed...");
    KDL::Frame f = arm0_offset_.Inverse() * to_wrist;

    intermpt.x = -f.p.x();
    intermpt.y = -f.p.y();

    if(a==parent_theta){
      //if this is the first iteration, save the x and y as the origin for the other points
      temp_x = intermpt.x;
      temp_y = intermpt.y;
    }
    intermpt.x -= temp_x;
    intermpt.y -= temp_y;
    intermpt.theta = a;
    
    /*
    double blah_x, blah_y, blah_z;
    discToWorldXYZ(parent->coord[8],parent->coord[9],0,blah_x,blah_y,blah_z,true);
    std::vector<double> p(6,0);
    p[0] = intermpt.x+blah_x;
    p[1] = intermpt.y+blah_y;
    pviz_.visualizeSphere(p,139,"intermediate_pose_"+boost::lexical_cast<std::string>(a), 0.02);usleep(5000);
    printf("a=%f (%f %f %f)\n",a,intermpt.x,intermpt.y,intermpt.theta);
    */
    
    action->intermptV.push_back(intermpt);

    /*
    BodyPose pPose(p[0],p[1],0,a);
    computeObjectPose(pPose,parent->angles0,obj_x,obj_y,obj_z,obj_yaw);
    discToWorldXYZ(obj_x,obj_y,0,blah_x,blah_y,blah_z,true);
    printf("obj %d %d %d %d\n",obj_x,obj_y,obj_z,obj_yaw);
    p[0]=blah_x;
    p[1]=blah_y;
    pviz_.visualizeSphere(p,60,"object_pose_"+boost::lexical_cast<std::string>(a), 0.02);usleep(5000);
    */
  }
  /*
  std::vector<double> p(6,0);
  double blah_x, blah_y, blah_z;
  discToWorldXYZ(parent->coord[8],parent->coord[9],0,blah_x,blah_y,blah_z,true);
  p[0] = action->intermptV.back().x + blah_x;
  p[1] = action->intermptV.back().y + blah_y;
  p[2] = action->intermptV.back().theta;
  pviz_.visualizeRobot(parent->angles0, parent->angles1, p, 0, 250,"hey_hey", 0);usleep(5000);
  */
  

  //add the final point (where the robot ends up) to the motion
  sbpl_xy_theta_pt_t intermpt;
  discToWorldXYZ(action->dX,action->dY,0,intermpt.x,intermpt.y,temp_z,true);
  intermpt.theta = body_pose.theta;
  //printf("(%f %f %f)\n",intermpt.x,intermpt.y,intermpt.theta);
  action->intermptV.push_back(intermpt);
  
  //compute the cost (max of translation time and rotation time)
  //double translation_time = fabs(angle)*radius/prms_.nominalvel_mpersecs_;
  //double rotation_time = fabs(angle)/((PI_CONST/4.0)/prms_.timetoturn45degsinplace_secs_);
  action->cost = 50;//(int)(ceil(NAVXYTHETALAT_COSTMULT_MTOMM/10*max(translation_time, rotation_time)));

  //printf("using adaptive base motion %d %d %d\n",action->dX,action->dY,action->endtheta);
  //printf("cost=%d",action->cost);
  return true;
}

void EnvironmentDUALROBARM3D::GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV)
{
  ROS_ERROR("ERROR in EnvROBARM... function: GetPreds is undefined\n");
  throw new SBPL_Exception();
}

bool EnvironmentDUALROBARM3D::AreEquivalent(int StateID1, int StateID2)
{
  ROS_ERROR("Error: AreEquivalent() is undefined.");
  return false;
}

void EnvironmentDUALROBARM3D::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{
  ROS_ERROR("Error: SetAllActionsandOutcomes is undefined.");
  throw new SBPL_Exception();
}

void EnvironmentDUALROBARM3D::SetAllPreds(CMDPSTATE* state)
{
  //implement this if the planner needs access to predecessors
  ROS_ERROR("Error: SetAllPreds is undefined.");
  throw new SBPL_Exception();
}

/////////////////////////////////////////////////////////////////////////////
//                      End of SBPL Planner Interface
/////////////////////////////////////////////////////////////////////////////

void EnvironmentDUALROBARM3D::printHashTableHist()
{
  int s0=0, s1=0, s50=0, s100=0, s200=0, s300=0, slarge=0;

  for(int  j = 0; j < EnvROBARM.HashTableSize; j++)
  {
    if((int)EnvROBARM.Coord2StateIDHashTable[j].size() == 0)
      s0++;
    else if((int)EnvROBARM.Coord2StateIDHashTable[j].size() < 50)
      s1++;
    else if((int)EnvROBARM.Coord2StateIDHashTable[j].size() < 100)
      s50++;
    else if((int)EnvROBARM.Coord2StateIDHashTable[j].size() < 200)
      s100++;
    else if((int)EnvROBARM.Coord2StateIDHashTable[j].size() < 300)
      s200++;
    else if((int)EnvROBARM.Coord2StateIDHashTable[j].size() < 400)
      s300++;
    else
      slarge++;
  }
  ROS_DEBUG("hash table histogram: 0:%d, <50:%d, <100:%d, <200:%d, <300:%d, <400:%d >400:%d",
      s0,s1, s50, s100, s200,s300,slarge);
}

EnvDUALROBARM3DHashEntry_t* EnvironmentDUALROBARM3D::getHashEntry(const std::vector<short unsigned int> &coord, bool bIsGoal)
{
  //if it is goal
  if(bIsGoal)
    return EnvROBARM.goalHashEntry;

  // if out of bounds, return NULL


  int binid = getHashBin(coord);

  //iterate over the states in the bin and select the perfect match
  for(size_t ind = 0; ind < EnvROBARM.Coord2StateIDHashTable[binid].size(); ind++)
  {
    size_t j = 0;
    
    for(j=0; j<coord.size(); j++)
    {
      if(EnvROBARM.Coord2StateIDHashTable[binid][ind]->coord[j] != coord[j])
        break;
    }

    if (int(j) == ndof_)
      return EnvROBARM.Coord2StateIDHashTable[binid][ind];
  }

  return NULL;
}

EnvDUALROBARM3DHashEntry_t* EnvironmentDUALROBARM3D::getHashEntry(short unsigned int *xyz, short unsigned int *rpy, short unsigned int *fangle, BodyCell body, bool bIsGoal)
{
  std::vector<short unsigned int> coord(ndof_,0);
  coord[0] = xyz[0];
  coord[1] = xyz[1];
  coord[2] = xyz[2];
  coord[3] = rpy[0];
  coord[4] = rpy[1];
  coord[5] = rpy[2];
  coord[6] = fangle[0];
  coord[7] = fangle[1];
  coord[8] = body.x;
  coord[9] = body.y;
  coord[10] = body.z;
  coord[11] = body.theta;

  return getHashEntry(coord,bIsGoal);
}

EnvDUALROBARM3DHashEntry_t* EnvironmentDUALROBARM3D::createHashEntry(const std::vector<short unsigned int> &coord)
{
  EnvDUALROBARM3DHashEntry_t* HashEntry = new EnvDUALROBARM3DHashEntry_t;

  HashEntry->coord = coord;
  HashEntry->dist = 200;
  HashEntry->angles0.resize(njoints_,-1.0);
  HashEntry->angles1.resize(njoints_,-1.0);
  HashEntry->object_pose.resize(4,0);

  //assign a stateID to HashEntry to be used 
  HashEntry->stateID = EnvROBARM.StateID2CoordTable.size();

  //insert into the tables
  EnvROBARM.StateID2CoordTable.push_back(HashEntry);

  //insert the entry into the bin
  EnvROBARM.Coord2StateIDHashTable[getHashBin(HashEntry->coord)].push_back(HashEntry);

  //insert into and initialize the mappings
  int* entry = new int [NUMOFINDICES_STATEID2IND];
  StateID2IndexMapping.push_back(entry);
  for(int i = 0; i < NUMOFINDICES_STATEID2IND; i++)
    StateID2IndexMapping[HashEntry->stateID][i] = -1;

  if(HashEntry->stateID != (int)StateID2IndexMapping.size()-1)
  {
    ROS_ERROR("ERROR in Env... function: last state has incorrect stateID");
    throw new SBPL_Exception();
  }

  return HashEntry;
}

void EnvironmentDUALROBARM3D::initDijkstra()
{
  int dimX,dimY,dimZ;
  grid_->getGridSize(dimX, dimY, dimZ);

  if(!prms_.nav3d_){
    dijkstra_ = new sbpl_arm_planner::BFS3D(dimX, dimY, dimZ, int(arm_[0]->getLinkRadiusCells(2)), prms_.cost_per_cell_);
    dijkstra_->configDistanceField(true, grid_->getDistanceFieldPtr());
#if DEBUG_SEARCH
    if(prms_.verbose_)
      dijkstra_->printConfig(fSucc);
#endif 
  }
  else{
    grid2D = new unsigned char* [dimX];
    for (int x = 0; x < dimX; x++)
      grid2D[x] = new unsigned char [dimY];
    grid2Dsearchfromgoal = new SBPL2DGridSearch(dimX, dimY, prms_.xyz_resolution_);
    //set OPEN type to sliding buckets
    grid2Dsearchfromgoal->setOPENdatastructure(SBPL_2DGRIDSEARCH_OPENTYPE_SLIDINGBUCKETS);
  }

  ROS_DEBUG("[initDijkstra] BFS is initialized.");
}

int EnvironmentDUALROBARM3D::cost(EnvDUALROBARM3DHashEntry_t* HashEntry1, EnvDUALROBARM3DHashEntry_t* HashEntry2, bool bState2IsGoal)
{
  if(prms_.use_uniform_cost_)
    return prms_.cost_per_cell_;
  else
  {
    //TODO: the dist is questionable....it assumes the environment doesn't change...ask Ben!
    // Max's suggestion is to just put a high cost on being close to
    // obstacles but don't provide some sort of gradient 
    if(int(HashEntry2->dist) < 4) // in cells
      return (prms_.cost_per_cell_*0.5) * prms_.range1_cost_;
    else if(int(HashEntry2->dist) < 6)
      return (prms_.cost_per_cell_*0.5) * prms_.range2_cost_;
    else if(int(HashEntry2->dist) < 10)
      return (prms_.cost_per_cell_*0.5) * prms_.range3_cost_;
    else
      return prms_.cost_per_cell_;
  }
}

bool EnvironmentDUALROBARM3D::initEnvConfig()
{
  std::vector<short unsigned int> coord(ndof_,0);

  EnvROBARMCfg.coord_delta.resize(ndof_,1);
  EnvROBARMCfg.coord_vals.resize(ndof_,360);

  EnvROBARMCfg.coord_delta[0] = prms_.xyz_resolution_;
  EnvROBARMCfg.coord_delta[1] = prms_.xyz_resolution_;
  EnvROBARMCfg.coord_delta[2] = prms_.xyz_resolution_;
  EnvROBARMCfg.coord_delta[3] = prms_.rpy_resolution_;
  EnvROBARMCfg.coord_delta[4] = prms_.rpy_resolution_;
  EnvROBARMCfg.coord_delta[5] = prms_.rpy_resolution_;
  EnvROBARMCfg.coord_delta[6] = prms_.fa_resolution_;
  EnvROBARMCfg.coord_delta[7] = prms_.fa_resolution_;

  EnvROBARMCfg.coord_delta[8] = prms_.xyz_resolution_;
  EnvROBARMCfg.coord_delta[9] = prms_.xyz_resolution_;
  EnvROBARMCfg.coord_delta[10] = prms_.xyz_resolution_;
  EnvROBARMCfg.coord_delta[11] = prms_.base_theta_resolution_;

  EnvROBARMCfg.coord_vals[0] = prms_.sizeX_ / prms_.xyz_resolution_ + 0.5;
  EnvROBARMCfg.coord_vals[1] = prms_.sizeY_ / prms_.xyz_resolution_ + 0.5;
  EnvROBARMCfg.coord_vals[2] = prms_.sizeZ_ / prms_.xyz_resolution_ + 0.5;
  EnvROBARMCfg.coord_vals[3] = (2.0*M_PI) / prms_.rpy_resolution_ + 0.5;
  EnvROBARMCfg.coord_vals[4] = (2.0*M_PI) / prms_.rpy_resolution_ + 0.5;
  EnvROBARMCfg.coord_vals[5] = (2.0*M_PI) / prms_.rpy_resolution_ + 0.5;
  EnvROBARMCfg.coord_vals[6] = (2.0*M_PI) / prms_.fa_resolution_ + 0.5;
  EnvROBARMCfg.coord_vals[7] = (2.0*M_PI) / prms_.fa_resolution_ + 0.5;

  EnvROBARMCfg.coord_vals[8] = prms_.sizeX_ / prms_.xyz_resolution_ + 0.5;
  EnvROBARMCfg.coord_vals[9] = prms_.sizeY_ / prms_.xyz_resolution_ + 0.5;
  EnvROBARMCfg.coord_vals[10] = prms_.sizeZ_ / prms_.xyz_resolution_ + 0.5;
  EnvROBARMCfg.coord_vals[11] = (2.0*M_PI) / prms_.base_theta_resolution_ + 0.5;

  SBPL_INFO("[env] Discretization of Statespace:");
  for(int i = 0; i < ndof_; ++i)
    SBPL_INFO("[env] [%d] delta: %0.3f  vals: %d",i, EnvROBARMCfg.coord_delta[i],EnvROBARMCfg.coord_vals[i]);

  //initialize the map from Coord to StateID
  EnvROBARM.HashTableSize = 32*1024; //should be power of two
  EnvROBARM.Coord2StateIDHashTable = new std::vector<EnvDUALROBARM3DHashEntry_t*>[EnvROBARM.HashTableSize];

  //initialize the map from StateID to Coord
  EnvROBARM.StateID2CoordTable.clear();

  //create empty start & goal states
  //EnvROBARM.startHashEntry = createHashEntry(coord);
  EnvROBARM.goalHashEntry = createHashEntry(coord);

  return true;
}

bool EnvironmentDUALROBARM3D::initArmModel(FILE* arm0_file, FILE* arm1_file,  const std::string robot_description)
{
  arm_[0] = new sbpl_arm_planner::SBPLArmModel(arm0_file);
  arm_[1] = new sbpl_arm_planner::SBPLArmModel(arm1_file);

  arm_[0]->setResolution(prms_.resolution_);
  arm_[1]->setResolution(prms_.resolution_);

  if(robot_description.compare("ROS_PARAM") == 0)
  {
    if(arm_[0]->initKDLChainFromParamServer() && arm_[1]->initKDLChainFromParamServer())
        return true;
  }
  else
  {
    if(arm_[0]->initKDLChain(robot_description) && arm_[1]->initKDLChain(robot_description))
      return true;
  }

  return false; 
}

bool EnvironmentDUALROBARM3D::initEnvironment(std::string arm0_filename, std::string arm1_filename, std::string mprims_filename, std::string base_mprim_filename)
{
  FILE* mprims_fp=NULL;
  FILE* arm0_fp=NULL;
  FILE* arm1_fp=NULL;
  FILE* base_mprim_fp=NULL;

  //initialize the arm planner parameters
  //TODO: update this function
  prms_.initFromParamServer();

  //parse motion primitives file
  if((mprims_fp=fopen(mprims_filename.c_str(),"r")) == NULL)
  {
    ROS_ERROR("Failed to open motion primitive file. (%s)", mprims_filename.c_str());
    return false;
  }
  if(!prms_.initLongMotionPrimsFromFile(mprims_fp))
  {
    ROS_ERROR("Failed to parse motion primitive file.");
    fclose(mprims_fp);
    return false;
  }
  fclose(mprims_fp);

  //parse base motion primitive file
  if((base_mprim_fp=fopen(base_mprim_filename.c_str(),"r")) == NULL)
  {
    ROS_ERROR("Failed to open base motion primitive file. (%s)", base_mprim_filename.c_str());
    return false;
  }
  //TODO: implement this function
  if(!prms_.initBaseMotionPrimsFromFile(base_mprim_fp))
  {
    ROS_ERROR("Failed to parse base motion primitive file.");
    fclose(base_mprim_fp);
    return false;
  }
  fclose(base_mprim_fp);

  //initialize the arm model
  if((arm0_fp=fopen(arm0_filename.c_str(),"r")) == NULL)
  {
    ROS_ERROR("Failed to open right arm description file.");
    return false;
  }
  if((arm1_fp=fopen(arm1_filename.c_str(),"r")) == NULL)
  {
    ROS_ERROR("Failed to open left arm description file.");
    return false;
  }
  std::string ros_param("ROS_PARAM");
  if(!initArmModel(arm0_fp,arm1_fp,ros_param))
  {
    ROS_ERROR("Failed to initialize one or both arm models.");
    fclose(arm0_fp);
    fclose(arm1_fp);
    return false;
  }
  fclose(arm0_fp);
  fclose(arm1_fp);

  njoints_ = arm_[0]->num_joints_;

  //initialize the environment & planning variables  
  if(!initGeneral())
  {
    ROS_ERROR("[env] Failed to initialize environment.");
    return false;
  }

  //set 'Environment is Initialized' flag
  EnvROBARMCfg.bInitialized = true;

  SBPL_INFO("[env] Environment has been initialized.");

  //for statistics purposes
  starttime = clock();

  return true;
}

bool EnvironmentDUALROBARM3D::initGeneral()
{
  //create the occupancy grid
  grid_ = new sbpl_arm_planner::OccupancyGrid(prms_.sizeX_,prms_.sizeY_,prms_.sizeZ_, prms_.resolution_,prms_.originX_,prms_.originY_,prms_.originZ_);

  grid_->setReferenceFrame(prms_.reference_frame_);

  //create the collision space
  cspace_ = new sbpl_full_body_planner::PR2CollisionSpace(arm_[0], arm_[1], grid_);

  cspace_->setDebugLogName(prms_.cspace_log_);

  if(!cspace_->getSphereGroups())
  {
    ROS_ERROR("[env] Failed to get the full body spheres from the param server.");
    return false;
  }
  else
    cspace_->printSphereGroups();

  arm_[0]->setDebugLogName(prms_.arm_log_);
  arm_[1]->setDebugLogName(prms_.arm_log_);

#if DEBUG_SEARCH
  cspace_->setDebugFile(fSucc);
  arm_[0]->setDebugFile(std::string("sbpl"));
  arm_[1]->setDebugFile(std::string("sbpl"));
#endif

  //initialize Environment
  if(initEnvConfig() == false)
    return false;

  //compute the cost per cell to be used by heuristic
  computeCostPerCell();

  //initialize dijkstra 
  initDijkstra();

  if(prms_.verbose_)
  {
    arm_[0]->printArmDescription(std::string("sbpl_two_arm"));
    ROS_INFO("----------------------------------------------------------");
    arm_[1]->printArmDescription(std::string("sbpl_two_arm"));
    ROS_INFO("----------------------------------------------------------");
    prms_.printParams(std::string("sbpl_two_arm"));
    ROS_INFO("----------------------------------------------------------");
    prms_.printLongMotionPrims(std::string("sbpl_two_arm"));
    ROS_INFO("----------------------------------------------------------");
    dijkstra_->printConfig(stdout);
  }

  return true;
}

double EnvironmentDUALROBARM3D::getEpsilon()
{
  return prms_.epsilon_;
}

double EnvironmentDUALROBARM3D::getEpsilon2()
{
  return prms_.epsilon2_;
}

bool EnvironmentDUALROBARM3D::isGoalPosition(int x, int y, int z, int yaw)
{
  if(!useGoalID_)
    return false;
  int x_tol, y_tol, z_tol;
  worldToDiscXYZ(EnvROBARMCfg.goal.xyz_tol-0.005,EnvROBARMCfg.goal.xyz_tol-0.005,EnvROBARMCfg.goal.xyz_tol-0.005,x_tol,y_tol,z_tol,true);

  //ROS_DEBUG("[env] xyz: %d %d %d yaw: %d tolerances: %d %d %d (goal: %d %d %d %d)", x,y,z, yaw, x_tol,y_tol,z_tol, EnvROBARM.goalHashEntry->object_pose[0], EnvROBARM.goalHashEntry->object_pose[1], EnvROBARM.goalHashEntry->object_pose[2], EnvROBARM.goalHashEntry->object_pose[3]);

  //check position
  if(fabs(x-EnvROBARM.goalHashEntry->object_pose[0]) <= x_tol &&
      fabs(y-EnvROBARM.goalHashEntry->object_pose[1]) <= y_tol &&
      fabs(z-EnvROBARM.goalHashEntry->object_pose[2]) <= z_tol)
  {
    //log the amount of time required for the search to get close to the goal
    if(!near_goal && save_expanded_states)
    {
      time_to_goal_region = (clock() - starttime) / (double)CLOCKS_PER_SEC;
      near_goal = true;
      SBPL_INFO("Search is within %0.3f meters of the goal (%0.3f %0.3f %0.3f) after %.4f sec. (after %d expansions)", EnvROBARMCfg.goal.xyz_tol, EnvROBARMCfg.goal.xyz[0], EnvROBARMCfg.goal.xyz[1], EnvROBARMCfg.goal.xyz[2], time_to_goal_region, (int)expanded_states.size());
      geometry_msgs::Pose pose;
      pose.position.x = 1.5;
      pose.position.y = 1.5;
      pose.position.z = 2.0;
      //pviz_.visualizeText(pose, "At goal position", "goal announcement", 1, 190);
    }
    //check orientation
    int r,p,yaw_tol;
    worldToDiscRPY(0,0,EnvROBARMCfg.goal.yaw_tol,r,p,yaw_tol);
    if (fabs(yaw-EnvROBARM.goalHashEntry->object_pose[3]) <= yaw_tol)
      return true;

    //ROS_WARN("xyz goal is met. yaw goal isn't. yaw: %d desired_yaw: %d  tolerance: %d (%0.3frad)", yaw, EnvROBARM.goalHashEntry->object_pose[3],yaw_tol,EnvROBARMCfg.goal.yaw_tol);
  }
  return false;
}

int EnvironmentDUALROBARM3D::getActionCost(const std::vector<double> &from_config, const std::vector<double> &to_config, int dist)
{
  ROS_ERROR("getActionCost() is not yet implemented.");
  return 1;
}

int EnvironmentDUALROBARM3D::getEdgeCost(int FromStateID, int ToStateID)
{
#if DEBUG
  if(FromStateID >= (int)EnvROBARM.StateID2CoordTable.size() 
      || ToStateID >= (int)EnvROBARM.StateID2CoordTable.size())
  {
    ROS_ERROR("ERROR in EnvROBARM... function: stateID illegal\n");
    throw new SBPL_Exception();
  }
#endif
  //this is sketchy.....it will return a cost of 1 cell regardless of the two states....
  ROS_ERROR("[env] sketchy edge cost...");
  exit(0);

  //get X, Y for the state
  EnvDUALROBARM3DHashEntry_t* FromHashEntry = EnvROBARM.StateID2CoordTable[FromStateID];
  EnvDUALROBARM3DHashEntry_t* ToHashEntry = EnvROBARM.StateID2CoordTable[ToStateID];

  return cost(FromHashEntry, ToHashEntry, false);
}

int EnvironmentDUALROBARM3D::setStartConfiguration(const std::vector<double> &angles0,
                                                   const std::vector<double> &angles1,
                                                   const BodyPose pose,
                                                   KDL::Frame &arm0_offset, KDL::Frame &arm1_offset)
{
  succ_time = 0;
  arms_time = 0;
  base_time = 0;
  torso_time = 0;
  ik_time = 0;
  fk_time = 0;
  hash_time = 0;
  if(int(angles0.size()) < njoints_ || int(angles1.size()) < njoints_)
  {
    ROS_ERROR("[env] Inputs to setStartConfiguration() are invalid.");
    return -1;
  }

  arm0_offset_ = arm0_offset;
  arm1_offset_ = arm1_offset;

  //check joint limits of starting configuration but plan anyway
  if(!arm_[0]->checkJointLimits(angles0, true))
    SBPL_WARN("Initial configuration of the right arm violates the joint limits. Attempting to plan anyway.");

  if(!arm_[1]->checkJointLimits(angles1, true))
    SBPL_WARN("Initial configuration of the left arm violates the joint limits. Attempting to plan anyway.");

  //set start position 
  std::vector<short unsigned int> coord(12,0);

  if(!computeObjectPose(pose,angles0,coord[0],coord[1],coord[2],coord[5],false))
    return -1;
  coord[3] = 0;
  coord[4] = 0;

  double wfa[2] = {angles0[free_angle_index_],angles1[free_angle_index_]};
  short unsigned int fa[2];
  worldToDiscFAngle(wfa,fa);
  coord[6] = fa[0];
  coord[7] = fa[1];

  BodyCell bc;
  //printf("z before %f\n",pose.z);
  worldToDiscBody(pose, &bc);
  //printf("z after %d\n",bc.z);

  coord[8] = bc.x;
  coord[9] = bc.y;
  coord[10] = bc.z;
  coord[11] = bc.theta;
  
  short unsigned int object_x, object_y, object_z, object_yaw;
  if(!computeObjectPose(pose,angles0,object_x,object_y,object_z,object_yaw))
    return -1;

  ROS_INFO("[env] xyz=(%d %d %d) rpy=(%d %d %d) fa=(%d %d) body=(%d %d %d %d) object=(%d %d %d %d)",
      coord[0],coord[1],coord[2],coord[3],coord[4],coord[5],coord[6],coord[7],coord[8],coord[9],coord[10],coord[11],object_x,object_y,object_z,object_yaw);
  
  EnvDUALROBARM3DHashEntry_t* HashEntry;
  if((HashEntry = getHashEntry(coord, false)) == NULL){
    HashEntry = createHashEntry(coord);
    HashEntry->angles0 = angles0;
    HashEntry->angles1 = angles1;
    HashEntry->object_pose[0] = object_x;
    HashEntry->object_pose[1] = object_y;
    HashEntry->object_pose[2] = object_z;
    HashEntry->object_pose[3] = object_yaw;
  }

  EnvROBARM.startHashEntry = HashEntry;

  ROS_INFO("[env] [start]");
  ROS_INFO("[env]   xyz: %u %u %u rpy: %u %u %u fa: %u %u",EnvROBARM.startHashEntry->coord[0],EnvROBARM.startHashEntry->coord[1],EnvROBARM.startHashEntry->coord[2],EnvROBARM.startHashEntry->coord[3],EnvROBARM.startHashEntry->coord[4],EnvROBARM.startHashEntry->coord[5],EnvROBARM.startHashEntry->coord[6],EnvROBARM.startHashEntry->coord[7]);


  BodyPose bp(pose.x,pose.y,pose.z,pose.theta);
  unsigned char dist_temp;
  if(!cspace_->checkAllMotion(HashEntry->angles1,HashEntry->angles0,bp,true,dist_temp,debug_code_)){
    ROS_ERROR("[env] The start state is in collision!");
    return -1;
  }

  return EnvROBARM.startHashEntry->stateID;
}

int EnvironmentDUALROBARM3D::setGoalPosition(const std::vector<std::vector<double> > &goals, const std::vector<std::vector<double> > &tolerances, KDL::Frame &arm0_offset, KDL::Frame &arm1_offset, double object_radius)
{
  //goals: {{x1,y1,z1,r1,p1,y1,fangle,is_6dof,is_global},...}
  //is_global_xyz means it is pose of the object in the map frame, otherwise it means the torso_lift_link (the base/spine pose doesn't matter)

  if(!EnvROBARMCfg.bInitialized)
  {
    ROS_ERROR("Cannot set goal position because environment is not initialized.");
    return -1;
  }

  if(goals.empty())
  {
    ROS_ERROR("[setGoalPosition] No goal constraint set.");
    return -1;
  }

  // debugging - check if an IK solution exists for the goal pose before we do the search
  // we plan even if there is no solution
  std::vector<double> pose(njoints_,0), jnt_angles(njoints_,0), ik_solution(njoints_,0);
  pose = goals[0];
  if(!arm_[0]->computeIK(pose, EnvROBARM.startHashEntry->angles0, ik_solution))
    ROS_DEBUG("[setGoalPosition] No valid IK solution for the right arm at the goal pose.");
  if(!arm_[1]->computeIK(pose, EnvROBARM.startHashEntry->angles1, ik_solution))
    ROS_DEBUG("[setGoalPosition] No valid IK solution for the left arm at the goal pose.");

  double dummy_x, dummy_y;
  discToWorldXYZ(0,0,EnvROBARM.startHashEntry->coord[10],dummy_x,dummy_y,EnvROBARMCfg.goal.xyz[2],true);

  // only supports a single goal
  EnvROBARMCfg.goal.xyz[0] = goals[0][0];
  EnvROBARMCfg.goal.xyz[1] = goals[0][1];
  //EnvROBARMCfg.goal.xyz[2] = goals[0][2];
  EnvROBARMCfg.goal.rpy[0] = goals[0][3];
  EnvROBARMCfg.goal.rpy[1] = goals[0][4];
  EnvROBARMCfg.goal.rpy[2] = goals[0][5];

  //free angles may not be set for the goal
  if(int(goals[0].size()) == ndof_)
  {
    EnvROBARMCfg.goal.fangle[0] = goals[0][6];
    EnvROBARMCfg.goal.fangle[1] = goals[0][7];
  }
  else
  {
    EnvROBARMCfg.goal.fangle[0] = 0;
    EnvROBARMCfg.goal.fangle[1] = 0;
  }

  EnvROBARMCfg.goal.xyz_tol = tolerances[0][0];
  EnvROBARMCfg.goal.roll_tol = tolerances[0][3];
  EnvROBARMCfg.goal.pitch_tol = tolerances[0][3];
  EnvROBARMCfg.goal.yaw_tol = tolerances[0][3];

  EnvROBARMCfg.goal.type = goals[0][7];
  prms_.use_6d_pose_goal_ = goals[0][7];

  //TODO: handle is_global flag!!!!
  prms_.is_global_goal_ = goals[0][8];

  arm0_offset_ = arm0_offset;
  arm1_offset_ = arm1_offset;

  //TODO: the goal has to be handled specially because we don't know the pose of the base yet...
  //TODO: after the search (and the goal is found) a post-processing step will be used to create a real hash entry for it
  int temp_x, temp_y, temp_z;
  worldToDiscXYZ(EnvROBARMCfg.goal.xyz[0],EnvROBARMCfg.goal.xyz[1],EnvROBARMCfg.goal.xyz[2],temp_x,temp_y,temp_z,true);
  if(!prms_.nav3d_){
    EnvROBARM.goalHashEntry->object_pose[0] = temp_x;
    EnvROBARM.goalHashEntry->object_pose[1] = temp_y;
    EnvROBARM.goalHashEntry->object_pose[2] = temp_z;
  }
  int roll,pitch,yaw;
  worldToDiscRPY(0, 0, EnvROBARMCfg.goal.rpy[2], roll, pitch, yaw);
  if(!prms_.nav3d_)
    EnvROBARM.goalHashEntry->object_pose[3] = yaw;
  //worldPoseToState(EnvROBARMCfg.goal.xyz, EnvROBARMCfg.goal.rpy, EnvROBARMCfg.goal.fangle, true, EnvROBARM.goalHashEntry);
  
  if(prms_.nav3d_){
    //set goal to have same arms and spine but base pose to be temp_x, temp_y, yaw
    std::vector<short unsigned int> coord(12,0);
    for(int i=0; i<12; i++)
      coord[i] = EnvROBARM.startHashEntry->coord[i];
    BodyPose bp(EnvROBARMCfg.goal.xyz[0],EnvROBARMCfg.goal.xyz[1],EnvROBARMCfg.goal.xyz[2],EnvROBARMCfg.goal.rpy[2]);
    BodyCell bc;
    worldToDiscBody(bp, &bc);
    coord[8] = bc.x;
    coord[9] = bc.y;
    coord[11] = bc.theta;
    ROS_INFO("[env] goal x=%d y=%d th=%d z=%d",coord[8],coord[9],coord[11],coord[10]);
    EnvDUALROBARM3DHashEntry_t* HashEntry;
    if((HashEntry = getHashEntry(coord, false)) == NULL){
      HashEntry = createHashEntry(coord);
      HashEntry->angles0 = EnvROBARM.startHashEntry->angles0;
      HashEntry->angles1 = EnvROBARM.startHashEntry->angles1;
    }
    EnvROBARM.goalHashEntry = HashEntry;
    unsigned char dist_temp;
    
    //BodyPose bp(EnvROBARMCfg.goal.xyz[0],EnvROBARMCfg.goal.xyz[1],EnvROBARMCfg.goal.xyz[2],EnvROBARMCfg.goal.rpy[2]);
    pviz_.visualizeRobot(HashEntry->angles0, HashEntry->angles1, bp, 150.0, "3dnav_goal", 100);
    usleep(5000);
    if(!cspace_->checkAllMotion(HashEntry->angles1,HashEntry->angles0,bp,true,dist_temp,debug_code_)){
	ROS_ERROR("[env] The goal state is in collision!");
        return -1;
    }
  }

  if(!prms_.use_6d_pose_goal_)
    ROS_DEBUG("[setGoalPosition] Goal position constraint set. No goal orientation constraint requested.\n");

  SBPL_INFO("[env] [goal]");
  SBPL_INFO("[env]   xyz: %.2f %.2f %.2f (meters) (tol: %.3fm)", EnvROBARMCfg.goal.xyz[0],EnvROBARMCfg.goal.xyz[1],EnvROBARMCfg.goal.xyz[2],EnvROBARMCfg.goal.xyz_tol);
  SBPL_INFO("[env]   rpy: %1.2f %1.2f %1.2f (radians) (tol: %.3frad)", EnvROBARMCfg.goal.rpy[0],EnvROBARMCfg.goal.rpy[1],EnvROBARMCfg.goal.rpy[2],EnvROBARMCfg.goal.roll_tol);
  SBPL_INFO("[env]  grid: %u %u %u %u (cells)", EnvROBARM.goalHashEntry->object_pose[0], EnvROBARM.goalHashEntry->object_pose[1], EnvROBARM.goalHashEntry->object_pose[2], EnvROBARM.goalHashEntry->object_pose[3]);

  std::vector<double> pose0(6,0),pose1(6,0);
  getGripperPosesFromObjectPose(goals[0],pose0,pose1);
  ROS_INFO("[env] [left arm pose]");
  ROS_INFO("[env]   xyz: %0.3f %0.3f %0.3f rpy: %0.3f %0.3f %0.3f", pose0[0],pose0[1],pose0[2],pose0[3],pose0[4],pose0[5]);
  ROS_INFO("[env] [right arm pose]");
  ROS_INFO("[env]   xyz: %0.3f %0.3f %0.3f rpy: %0.3f %0.3f %0.3f", pose1[0],pose1[1],pose1[2],pose1[3],pose1[4],pose1[5]);

  // precompute heuristics
  if(computeNormalHeuristic){
    if(!precomputeHeuristics())
    {
      ROS_ERROR("[env] Precomputing heuristics failed. Exiting.");
      return -1;
    }
  }

  //debugging
  right_ik_search_success_ = 0;
  left_ik_search_success_ = 0;

  incorrect_right_states_.clear();
  incorrect_right_states_.resize(ndof_,0);
  incorrect_left_states_.clear();
  incorrect_left_states_.resize(ndof_,0);

  clearStats();

  return EnvROBARM.goalHashEntry->stateID;
}

bool EnvironmentDUALROBARM3D::precomputeHeuristics()
{
  if(prms_.nav3d_){
    //update the grid2D based on 3D grid
    int dimX,dimY,dimZ;
    grid_->getGridSize(dimX, dimY, dimZ);
    int thresh = 0;//0.3/prms_.xyz_resolution_;
    for(int x=0; x<dimX; x++){
      for(int y=0; y<dimY; y++){
        grid2D[x][y]=0;
        for(int z=0; z<dimZ; z++){
          if(grid_->getCell(x,y,z)<=thresh){
            grid2D[x][y]=1;
            break;
          }
        }
      }
    }
    if(!grid2Dsearchfromgoal->search(grid2D, 1,
                                 EnvROBARM.goalHashEntry->coord[8], EnvROBARM.goalHashEntry->coord[9], 
                                 EnvROBARM.startHashEntry->coord[8], EnvROBARM.startHashEntry->coord[9],
                                 SBPL_2DGRIDSEARCH_TERM_CONDITION_TWOTIMESOPTPATH))
        return false;
    ROS_INFO("[env] 2dsolcost_infullunits=%d", (int)(grid2Dsearchfromgoal->getlowerboundoncostfromstart_inmm(EnvROBARM.startHashEntry->coord[8], EnvROBARM.startHashEntry->coord[9])/prms_.nominalvel_mpersecs_));
  }
  else{
    std::vector<int> pose0(6,0), pose1(6,0);
    std::vector<double> object(ndof_,0);

    coordToWorldPose(EnvROBARM.goalHashEntry->coord, object);
    //getGripperCoordsFromObjectPose(object, pose0, pose1);

    //TODO: remove 2 gripper dijkstra searches

    //set the radius of the object
    dijkstra_->setRadius(object_radius_);
    ROS_INFO("[env] Set the object radius to %0.3fm",object_radius_);

    dijkstra_->useResearchGrid(prms_.use_research_grid_);
    if(prms_.use_research_grid_)
    {
      ROS_INFO("[env] Initializing the XYGrid");
      dijkstra_->initializeXYGrid();
      ROS_INFO("[env] Inflating the XY Grid");
      dijkstra_->inflateXYGrid();
    }

    if(!dijkstra_->setGoal(EnvROBARM.goalHashEntry->object_pose[0], EnvROBARM.goalHashEntry->object_pose[1], EnvROBARM.goalHashEntry->object_pose[2]))
    {
      ROS_ERROR("[env] Failed to set goal for object bfs. (%d %d %d)", EnvROBARM.goalHashEntry->object_pose[0], EnvROBARM.goalHashEntry->object_pose[1], EnvROBARM.goalHashEntry->object_pose[2]);
      return false;
    }

    if(!dijkstra_->runBFS())
    {
      ROS_ERROR("[env] Precomputing the BFS for the object heuristic failed. Exiting.");
      return false;
    }

    std::vector<int> start0(6,0), start1(6,0);
    coordToWorldPose(EnvROBARM.startHashEntry->coord, object);
    //getGripperCoordsFromObjectPose(object, start0, start1);

    ROS_INFO("[env] [heuristics]");
    ROS_INFO("[env]   [object] goal: %3d %3d %3d  radius: %d  (start: %3d %3d %3d  distance: %d)",EnvROBARM.goalHashEntry->object_pose[0],EnvROBARM.goalHashEntry->object_pose[1],EnvROBARM.goalHashEntry->object_pose[2],dijkstra_->getRadiusCells(),EnvROBARM.startHashEntry->object_pose[0],EnvROBARM.startHashEntry->object_pose[1],EnvROBARM.startHashEntry->object_pose[2],getDijkstraDistance(EnvROBARM.startHashEntry->coord));

    //ROS_DEBUG_NAMED(prms_.expands_log_,"[env] Object goal dijkstra: %d", dijkstra_->getDist(int(EnvROBARM.goalHashEntry->object_pose[0]),int(EnvROBARM.goalHashEntry->object_pose[1]),int(EnvROBARM.goalHashEntry->object_pose[2])));
  }

  return true;
}

void EnvironmentDUALROBARM3D::clearStats()
{
  //a flag used for debugging only
  near_goal = false;

  //clear lists of stateIDs (for debugging only)
  expanded_states.clear();

  h_values_.clear();

  num_exp_per_hval_.clear();

  //start the 'planning time' clock
  starttime = clock();
}

void EnvironmentDUALROBARM3D::stateIDToPose(int stateID, short unsigned int *xyz, short unsigned int *rpy, short unsigned int *fangle, BodyCell* pose)
{
  EnvDUALROBARM3DHashEntry_t* HashEntry = EnvROBARM.StateID2CoordTable[stateID];

  if(stateID == EnvROBARM.goalHashEntry->stateID)
  {
    xyz[0] = EnvROBARM.goalHashEntry->coord[0];
    xyz[1] = EnvROBARM.goalHashEntry->coord[1];
    xyz[2] = EnvROBARM.goalHashEntry->coord[2];
    rpy[0] = EnvROBARM.goalHashEntry->coord[3];
    rpy[1] = EnvROBARM.goalHashEntry->coord[4];
    rpy[2] = EnvROBARM.goalHashEntry->coord[5];
    ROS_ERROR("[env] HEY HEY HEY! THIS SHOULDN'T HAPPEN!!!!\n\n\n\n");
    exit(0);
    //fangle[0] = EnvROBARM.goalHashEntry->coord[6];
    //fangle[1] = EnvROBARM.goalHashEntry->coord[7];
  }
  else
  {
    xyz[0] = HashEntry->coord[0];
    xyz[1] = HashEntry->coord[1];
    xyz[2] = HashEntry->coord[2];
    rpy[0] = HashEntry->coord[3];
    rpy[1] = HashEntry->coord[4];
    rpy[2] = HashEntry->coord[5];
    fangle[0] = HashEntry->coord[6];
    fangle[1] = HashEntry->coord[7];
    pose->x = HashEntry->coord[8];
    pose->y = HashEntry->coord[9];
    pose->z = HashEntry->coord[10];
    pose->theta = HashEntry->coord[11];
  }
}

void EnvironmentDUALROBARM3D::stateIDToWorldPose(int stateID, double *xyz, double *rpy, double *fangle, BodyPose* pose)
{
  short unsigned int dxyz[3]={0}, drpy[3]={0}, dfangle[2]={0};
  BodyCell dpose;

  stateIDToPose(stateID, dxyz, drpy, dfangle, &dpose);

  discToWorldXYZ(dxyz, xyz, false);
  discToWorldRPY(drpy, rpy);
  discToWorldFAngle(dfangle, fangle);
  discToWorldBody(dpose, pose);
}

void EnvironmentDUALROBARM3D::worldPoseToCoord(double *wxyz, double *wrpy, double *wfangle, BodyPose pose, std::vector<short unsigned int> &coord)
{
  short unsigned int xyz[3]={0}, rpy[3]={0}, fangle[2]={0};
  BodyCell bc;

  worldToDiscXYZ(wxyz, xyz,false);
  worldToDiscRPY(wrpy, rpy);
  worldToDiscFAngle(wfangle, fangle);
  worldToDiscBody(pose,&bc);

  coord.resize(ndof_);
  coord[0] = xyz[0];
  coord[1] = xyz[1];
  coord[2] = xyz[2];
  coord[3] = rpy[0];
  coord[4] = rpy[1];
  coord[5] = rpy[2];
  coord[6] = fangle[0];
  coord[7] = fangle[1];
  coord[8] = bc.x;
  coord[9] = bc.y;
  coord[10] = bc.z;
  coord[11] = bc.theta;
}

void EnvironmentDUALROBARM3D::worldPoseToState(double *wxyz, double *wrpy, double *wfangle, BodyPose pose, bool is_goal, EnvDUALROBARM3DHashEntry_t *state)
{
  std::vector<short unsigned int> coord(ndof_,0);

  worldPoseToCoord(wxyz,wrpy,wfangle,pose,coord);

  ROS_WARN("[env] Watch out! this function doesn't set the object pose!!!\n");
  if((state = getHashEntry(coord,is_goal)) == NULL)
    state = createHashEntry(coord);
  else
    state->coord = coord;
}

void EnvironmentDUALROBARM3D::printJointArray(FILE* fOut, EnvDUALROBARM3DHashEntry_t* HashEntry, bool bGoal, bool bVerbose)
{
/* CARTTODO
  if(bGoal)
    coordToAngles(EnvROBARM.goalHashEntry->coord, angles);
  else
    coordToAngles(HashEntry->coord, angles);
*/

  fprintf(fOut, "xyz: %-2d %-2d %-2d  rpy: %-2d %-2d %-2d  fangle0: %-2d fangle1: %-2d body %d %d %d %d", HashEntry->coord[0], HashEntry->coord[1], HashEntry->coord[2], HashEntry->coord[3], HashEntry->coord[4], HashEntry->coord[5], HashEntry->coord[6], HashEntry->coord[7], HashEntry->coord[8], HashEntry->coord[9], HashEntry->coord[10], HashEntry->coord[11]);
}

void EnvironmentDUALROBARM3D::computeCostPerCell()
{
  int cost_per_cell = prms_.cost_per_second_ * prms_.time_per_cell_;
  prms_.setCellCost(cost_per_cell);
  prms_.cost_per_meter_ = int(prms_.cost_per_cell_ / EnvROBARMCfg.xyz_resolution);
  ROS_INFO("[env] Cost per cell: %d, Time per cell: %0.3fsec", cost_per_cell, prms_.time_per_cell_);
}

int EnvironmentDUALROBARM3D::computeMotionCost(const std::vector<double> &a, const std::vector<double> &b, int i_arm)
{
  //TODO: return 40??? really???
  return 40;

  //TODO
  double time = 0, time_max = 0, dist = 0;

  for(size_t i = 0; i < a.size(); ++i)
  {
    if(i == 4 || i == 6)
      dist = angles::shortest_angular_distance(a[i], b[i]);
    else
    {
      if(!angles::shortest_angular_distance_with_limits(a[i], b[i], arm_[i_arm]->getMinJointLimit(i), arm_[i_arm]->getMaxJointLimit(i),dist))
        ROS_WARN("[env] computeMotionCost not working right");
    }

    time = fabs(dist) / prms_.joint_vel_[i];

    ROS_INFO("%d: a: %0.4f b: %0.4f dist: %0.4f vel:%0.4f time: %0.4f", int(i), a[i], b[i],dist, prms_.joint_vel_[i], time);

    if(time > time_max)
      time_max = time;
  }

  ROS_INFO("[env] motion cost: %d  max_time:%0.4f",  int(prms_.cost_per_second_ * time_max), time_max);

  return prms_.cost_per_second_ * time_max;
}

int EnvironmentDUALROBARM3D::getDijkstraDistToGoal(short unsigned int x, short unsigned int y, short unsigned int z) const
{
  return dijkstra_->getDist(int(x),int(y),int(z));
}

void EnvironmentDUALROBARM3D::updateOccupancyGridFromCollisionMap(const arm_navigation_msgs::CollisionMap &collision_map)
{
  if(collision_map.boxes.empty())
  {
    ROS_ERROR("[updateOccupancyGridFromCollisionMap] collision map received is empty.");
    return;
  }
  else
    ROS_DEBUG("[updateOccupancyGridFromCollisionMap] updating distance field with collision map with %d boxes.", int(collision_map.boxes.size()));

  grid_->updateFromCollisionMap(collision_map);
}

std::vector<std::vector<double> > EnvironmentDUALROBARM3D::getShortestPath()
{
  std::vector<short unsigned int> start(3,0);
  std::vector<double> waypoint(3,0);
  std::vector<std::vector<int> > path;
  std::vector<std::vector<double> > dpath; 

  //compute a Dijkstra path to goal 
  if(prms_.use_dijkstra_heuristic_)
  {
    start[0] = EnvROBARM.startHashEntry->coord[0];
    start[1] = EnvROBARM.startHashEntry->coord[1];
    start[2] = EnvROBARM.startHashEntry->coord[2];

    if(!dijkstra_->getShortestPath(start, path))
    {
      ROS_WARN("Unable to retrieve shortest path.");
      return dpath;
    }

    for(int i=0; i < (int)path.size(); ++i)
    {
      grid_->gridToWorld(path[i][0], path[i][1], path[i][2], waypoint[0], waypoint[1], waypoint[2]);
      dpath.push_back(waypoint);
    }
  }
  //compute a straight line path to goal
  else
  {
    getBresenhamPath(EnvROBARM.startHashEntry->xyz,EnvROBARM.goalHashEntry->xyz,&path);

    //convert to continuous
    for(int i=0; i < int(path.size()); ++i)
    {
      grid_->gridToWorld(path[i][0], path[i][1], path[i][2], waypoint[0], waypoint[1], waypoint[2]);
      dpath.push_back(waypoint);
    }
  }

  return dpath;
}

std::vector<std::vector<double> > EnvironmentDUALROBARM3D::getShortestPath(int dij_num)
{
  // dij_num is ignored for now.
  std::vector<short unsigned int> p0(3,0), p1(3,0);
  std::vector<double> waypoint(3,0);
  std::vector<std::vector<int> > path;
  std::vector<std::vector<double> > dpath; 
  std::vector<int> pose0(3,0), pose1(3,0);
  std::vector<double> object(ndof_,0);
  
  //compute a Dijkstra path to goal 
  if(prms_.use_dijkstra_heuristic_)
  {
    coordToWorldPose(EnvROBARM.startHashEntry->coord, object);

    if(!dijkstra_->getShortestPath(EnvROBARM.startHashEntry->object_pose[0],EnvROBARM.startHashEntry->object_pose[1],EnvROBARM.startHashEntry->object_pose[2], path))
    {
      ROS_WARN("[env] Unable to retrieve shortest path for the object.");
      return dpath;
    }
  }
  //compute a straight line path to goal
  else
    getBresenhamPath(EnvROBARM.startHashEntry->xyz,EnvROBARM.goalHashEntry->xyz,&path);

  //convert to continuous
  for(int i=0; i < int(path.size()); ++i)
  {
    grid_->gridToWorld(path[i][0], path[i][1], path[i][2], waypoint[0], waypoint[1], waypoint[2]);
    dpath.push_back(waypoint);
  }

  return dpath;
}

void EnvironmentDUALROBARM3D::getBresenhamPath(const short unsigned int a[],const short unsigned int b[], std::vector<std::vector<int> > *path)
{
  bresenham3d_param_t params;
  std::vector<int> nXYZ(3,0);

  //iterate through the points on the segment
  get_bresenham3d_parameters(a[0], a[1], a[2], b[0], b[1], b[2], &params);
  do {
    get_current_point3d(&params, &(nXYZ[0]), &(nXYZ[1]), &(nXYZ[2]));

    path->push_back(nXYZ);
  } while (get_next_point3d(&params));

  ROS_DEBUG("[getBresenhamPath] Path has %d waypoints.",int(path->size()));
}

void EnvironmentDUALROBARM3D::visualizeOccupancyGrid()
{
  grid_->visualize();
}

void EnvironmentDUALROBARM3D::setReferenceFrameTransform(KDL::Frame f, std::string &name)
{
  //TODO
  arm_[0]->setRefFrameTransform(f, name);
  arm_[1]->setRefFrameTransform(f, name);
}

std::vector<double> EnvironmentDUALROBARM3D::getPlanningStats()
{
  return succ_stats_;
}

int EnvironmentDUALROBARM3D::getEndEffectorHeuristic(int FromStateID, int ToStateID)
{
  EnvDUALROBARM3DHashEntry_t* FromHashEntry = EnvROBARM.StateID2CoordTable[FromStateID];
  if(prms_.nav3d_)
    return double(grid2Dsearchfromgoal->getlowerboundoncostfromstart_inmm(FromHashEntry->coord[8], FromHashEntry->coord[9]))/prms_.nominalvel_mpersecs_;
  else
    return dijkstra_->getDist(FromHashEntry->object_pose[0],FromHashEntry->object_pose[1],FromHashEntry->object_pose[2]);
}

std::vector<int> EnvironmentDUALROBARM3D::debugExpandedStates()
{
  return expanded_states;
}

void EnvironmentDUALROBARM3D::getExpandedStates(std::vector<std::vector<double> > &ara_states)
{
  std::vector<double>state(12,0);
  ROS_ERROR("(getExpandedStates) This looks like it might do bad things....since it uses 8-11 in the state...\n\n\n");

  for(size_t i = 0; i < expanded_states.size(); ++i)
  {
    coordToWorldPose(EnvROBARM.StateID2CoordTable[expanded_states[i]]->coord,state);
    state[8] = getDijkstraDistance(EnvROBARM.StateID2CoordTable[expanded_states[i]]->coord);
    state[9] = GetFromToHeuristic(expanded_states[i], EnvROBARM.goalHashEntry->stateID);
    ara_states.push_back(state);
  }
}

void EnvironmentDUALROBARM3D::getUniqueExpandedStates(std::vector<std::vector<double> > &ara_states)
{
  int sanity_check=0;
  std::vector<double>state(8,0);
  std::vector<short unsigned int>xyz(3,0);
  std::vector<int> unique_coords;
  std::map<std::vector<short unsigned int>, int> xyz_map;
  std::map<std::vector<short unsigned int>, int>::iterator itr1;

  ROS_ERROR("Removed 2 other heuristics from states that are returned. Need to update in node.");
  for(size_t i = 0; i < expanded_states.size(); ++i)
  {
    xyz[0] = EnvROBARM.StateID2CoordTable[expanded_states[i]]->coord[0];
    xyz[1] = EnvROBARM.StateID2CoordTable[expanded_states[i]]->coord[1];
    xyz[2] = EnvROBARM.StateID2CoordTable[expanded_states[i]]->coord[2];

    if(xyz_map.count(xyz) == 0)
    {
      xyz_map[xyz] = 1;
      unique_coords.push_back(expanded_states[i]);
    }
    else
      xyz_map[xyz] = xyz_map[xyz]+1;
  }

  ROS_INFO("[env] There are %d unique xyz positions among %d expanded states.",int(xyz_map.size()),int(expanded_states.size()));

  //TODO: Use unique_coords. Its not being used right now.
  ROS_DEBUG_NAMED(prms_.expands_log_, "[env] There are %d states in unique_coords.",int(unique_coords.size()));
  
  for(itr1 = xyz_map.begin(); itr1 != xyz_map.end(); itr1++)
  {
    discToWorldXYZ(itr1->first[0],itr1->first[1],itr1->first[2],state[0],state[1],state[2],false);

    xyz = itr1->first; 
    state[3] = itr1->second;
    state[4] = getDijkstraDistance(xyz);
    state[5] = max(state[3],max(state[4],state[5]));
    ara_states.push_back(state);
  
    sanity_check += itr1->second;
  }

  ROS_DEBUG_NAMED(prms_.expands_log_, "[env] I counted %d total expanded states in the unique states list.", sanity_check);
}

void EnvironmentDUALROBARM3D::getFinalArmConfigurations(std::vector<std::vector<double> > &arm0, std::vector<std::vector<double> > &arm1)
{
  int h=0, h_min=10000;
  double xyz[3]={0}, rpy[3]={0}, fa[2]={0};
  BodyPose pose;
  std::vector<int> closest_states;

  for(size_t i = 0; i < expanded_states.size(); ++i)
  {
    h = GetFromToHeuristic(expanded_states[i],EnvROBARM.goalHashEntry->stateID);

    if(h < h_min)
    {
      h_min = h;
      closest_states.clear();
      closest_states.push_back(expanded_states[i]);
    }
    else if(h == h_min)
      closest_states.push_back(expanded_states[i]);
  }

  for(size_t j = 0; j < closest_states.size(); ++j)
  {
    arm0.push_back(EnvROBARM.StateID2CoordTable[closest_states[j]]->angles0);
    arm1.push_back(EnvROBARM.StateID2CoordTable[closest_states[j]]->angles1);

    coordToWorldPose(EnvROBARM.StateID2CoordTable[closest_states[j]]->coord, xyz, rpy, fa, &pose);
    ROS_DEBUG("[env] [%d] xyz: %0.3f(%0.3f) %0.3f(%0.3f) %0.3f(%0.3f) rpy: %0.3f %0.3f %0.3f fa: %0.3f %0.3f", int(j), xyz[0],fabs(xyz[0]-EnvROBARMCfg.goal.xyz[0]),xyz[1],fabs(xyz[1]-EnvROBARMCfg.goal.xyz[1]),xyz[2],fabs(xyz[2]-EnvROBARMCfg.goal.xyz[2]),rpy[0],rpy[1],rpy[2],fa[0],fa[1]);
  }

  ROS_INFO("[env] There are %d states with the minimum h-value of %d.",int(closest_states.size()),h_min);
  ROS_DEBUG("[env] There are %d arm0 configurations and %d arm1 configurations.",int(arm0.size()),int(arm1.size()));
}

bool EnvironmentDUALROBARM3D::convertCoordToAngles(const std::vector<short unsigned int> &coord, std::vector<double> &angles0, std::vector<double> &angles1)
{
  
  ROS_DEBUG_NAMED(prms_.expands_log_, "[env] Converting:  xyz: %u %u %u  rpy: %u %u %u  fa0: %u %u",coord.at(0),coord.at(1),coord.at(2),coord.at(3),coord.at(4),coord.at(5),coord.at(6), coord.at(7));

  double wxyz[3]={0}, wrpy[3]={0}, wfangle[2]={0};
  std::vector<double> pose(6,0), seed(njoints_,0);
  BodyPose body_pose;

  coordToWorldPose(coord,wxyz,wrpy,wfangle,&body_pose);
  pose[0] = wxyz[0];
  pose[1] = wxyz[1];
  pose[2] = wxyz[2];
  pose[3] = wrpy[0];
  pose[4] = wrpy[1];
  pose[5] = wrpy[2];

  frame_b_o_.p.x(wxyz[0]);
  frame_b_o_.p.y(wxyz[1]);
  frame_b_o_.p.z(wxyz[2]);
  frame_b_o_.M = KDL::Rotation::RPY(wrpy[0],wrpy[1],wrpy[2]);

  //right arm
  frame_b_w_ = arm0_offset_ * frame_b_o_;
  seed = angles0;
  seed[free_angle_index_] = wfangle[0];
  if(!arm_[0]->computeFastIK(frame_b_w_,seed,angles0))
  {
    ROS_DEBUG("[Coord->Angles] computeFastIK failed to return a solution for the right arm.");
    return false;
    /*
    if(!arm_[0]->computeIK(pose, seed, angles0))
    {
      ROS_DEBUG("IK Search found solution");
      return false;
    }
    */
  }
  
  //left arm
  frame_b_w_ = arm1_offset_ * frame_b_o_;
  seed = angles1;
  seed[free_angle_index_] = wfangle[1];
  if(!arm_[1]->computeFastIK(frame_b_w_,seed,angles1))
  {
    ROS_DEBUG("[Coord->Angles] computeFastIK failed to return a solution for the left arm.");
    return false;
    /*
    if(!arm_[1]->computeIK(pose, seed, angles1))
    {
      ROS_DEBUG("IK Search found solution");
      return false;
    }
    */
  }

  return true;
}

bool EnvironmentDUALROBARM3D::convertWorldPoseToAngles(const std::vector<double> &wpose, std::vector<double> &angles0, std::vector<double> &angles1, bool allow_search, bool draw)
{
  std::vector<double> seed(njoints_,0);

  if(int(wpose.size()) < ndof_ || int(angles0.size()) < njoints_ || int(angles1.size()) < njoints_)
  {
    ROS_ERROR("[env] Input to convertWorldPoseToAngles() is of the wrong size.");
    return false;
  }

  ROS_DEBUG_NAMED(prms_.ik_log_, "[env] Attempting to convert object pose to arm angles.  xyz: %0.3f %0.3f %0.3f", wpose[0],wpose[1],wpose[2]);
  
  frame_b_o_.p.x(wpose[0]);
  frame_b_o_.p.y(wpose[1]);
  frame_b_o_.p.z(wpose[2]);
  frame_b_o_.M = KDL::Rotation::RPY(wpose[3],wpose[4],wpose[5]);


  if(draw){
    BodyPose pose(0,0,0,0);
    pviz_.visualizeRobot(angles0,angles1, pose, 150.0, "wrists", 0);
    usleep(5000);
  }

  //right arm
  frame_b_w_ = frame_b_o_ * arm0_offset_;

  if(draw){
    KDL::Frame a = frame_b_w_;
    vector<double> s(6,0);
    s[0] = a.p.x();s[1] = a.p.y();s[2] = a.p.z();
    pviz_.visualizeSphere(s,0,"right_arm",0.025);
    usleep(5000);
  }

  seed = angles0;
  seed[free_angle_index_] = wpose[6];
  if(!arm_[0]->computeFastIK(frame_b_w_,seed,angles0))
  { 
    ROS_DEBUG_NAMED(prms_.ik_log_, "[env] computeFastIK failed to return a solution for the right arm.");
    ROS_DEBUG_NAMED(prms_.ik_log_, "[env]    seed: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", seed[0],seed[1],seed[2],seed[3],seed[4],seed[5],seed[6]);
    if(allow_search)
    {
      if(!arm_[0]->computeIK(frame_b_w_,seed,angles0))
      {
        ROS_DEBUG_NAMED(prms_.ik_log_, "[env] IK Search failed to find a solution for the right arm.");
        debug_code_ = sbpl_arm_planner::RIGHT_IK_FAIL_IK_SEARCH_FAIL;
        return false;
      }
      debug_code_ = sbpl_arm_planner::RIGHT_IK_FAIL_IK_SEARCH_SUCCESS;
      right_ik_search_ = 1;
      right_ik_search_success_++;
    }
    else
      return false;
  }
  else
  {
    ROS_DEBUG_NAMED(prms_.ik_log_, "[env] computeFastIK returned a solution for the right arm.");
    ROS_DEBUG_NAMED(prms_.ik_log_, "[env]             seed: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", seed[0],seed[1],seed[2],seed[3],seed[4],seed[5],seed[6]);
    ROS_DEBUG_NAMED(prms_.ik_log_, "[env]    right_solution: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", angles0[0],angles0[1],angles0[2],angles0[3],angles0[4],angles0[5],angles0[6]);
  }

  //left arm
  frame_b_w_ = frame_b_o_ * arm1_offset_;

  if(draw){
    KDL::Frame a = frame_b_w_;
    vector<double> s(6,0);
    s[0] = a.p.x();s[1] = a.p.y();s[2] = a.p.z();
    pviz_.visualizeSphere(s,120,"left_arm",0.025);
    usleep(5000);
  }

  seed = angles1;
  seed[free_angle_index_] = wpose[7];
  if(!arm_[1]->computeFastIK(frame_b_w_,seed,angles1))
  {
    ROS_DEBUG_NAMED(prms_.ik_log_, "[env] computeFastIK failed to return a solution for the left arm.");
    ROS_DEBUG_NAMED(prms_.ik_log_, "[env]    seed: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", seed[0],seed[1],seed[2],seed[3],seed[4],seed[5],seed[6]);
    if(allow_search)
    {
      if(!arm_[1]->computeIK(frame_b_w_,seed,angles1))
      {
        ROS_DEBUG_NAMED(prms_.ik_log_, "[env] IK Search failed to find a solution for the left arm.");
        debug_code_ = sbpl_arm_planner::LEFT_IK_FAIL_IK_SEARCH_FAIL;
        return false;
      }
      debug_code_ = sbpl_arm_planner::LEFT_IK_FAIL_IK_SEARCH_SUCCESS;
      left_ik_search_ = 1;
      left_ik_search_success_++;
    }
    else
      return false;
  }
  else
  {
    ROS_DEBUG_NAMED(prms_.ik_log_, "[env] computeFastIK returned a solution for the left arm.");
    ROS_DEBUG_NAMED(prms_.ik_log_, "[env]             seed: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", seed[0],seed[1],seed[2],seed[3],seed[4],seed[5],seed[6]);
    ROS_DEBUG_NAMED(prms_.ik_log_, "[env]    left_solution: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", angles1[0],angles1[1],angles1[2],angles1[3],angles1[4],angles1[5],angles1[6]);
  }
  return true;
}

void EnvironmentDUALROBARM3D::getArmChainRootLinkName(std::string &name)
{
  //assuming both arms are using the same root link name
  arm_[0]->getArmChainRootLinkName(name);
}

void EnvironmentDUALROBARM3D::StateID2Angles(int stateID, std::vector<double> &angles0, std::vector<double> &angles1)
{
  //not being called by any function in the environment

  EnvDUALROBARM3DHashEntry_t* HashEntry = EnvROBARM.StateID2CoordTable[stateID];

  if(stateID == EnvROBARM.goalHashEntry->stateID)
  {
    angles0 = EnvROBARM.goalHashEntry->angles0;
    angles1 = EnvROBARM.goalHashEntry->angles1;
  }
  else
  {
    angles0 = HashEntry->angles0;
    angles1 = HashEntry->angles1;
  }

  for (size_t i = 0; i < angles0.size(); i++)
  {
    if(angles0[i] >= M_PI)
      angles0[i] = -2.0*M_PI + angles0[i];
    if(angles1[i] >= M_PI)
      angles1[i] = -2.0*M_PI + angles1[i];
  }
}

//computes the intermediate points between two states using linear interpolation
//returns true if there are no collisions
//if arm0_pts, arm1_pts, and body_pts vectors are provided it will return the intermediate points
bool EnvironmentDUALROBARM3D::computeIntermPoints(EnvDUALROBARM3DHashEntry_t* entry0, EnvDUALROBARM3DHashEntry_t* entry1, vector<vector<double> >* arm0_pts, vector<vector<double> >* arm1_pts, vector<BodyPose>* body_pts){
  //compute the deltas for each element in the coord (store them in a vector)
  //find the one that changed the most (this will determine the step size)
  //make of vector of step sizes
  BodyCell bc0(entry0->coord[8],entry0->coord[9],entry0->coord[10],entry0->coord[11]);
  BodyPose bp0;
  discToWorldBody(bc0,&bp0);
  BodyCell bc1(entry1->coord[8],entry1->coord[9],entry1->coord[10],entry1->coord[11]);
  BodyPose bp1;
  discToWorldBody(bc1,&bp1);

  double max_angle_step = 2.0*M_PI/180.0;
  double max_trans_step = 1.0;

  int max_steps = 0;

  vector<double> base_delta(3,0);
  base_delta[0] = bp1.x - bp0.x;
  max_steps = max(max_steps,int(base_delta[0]/max_trans_step));
  base_delta[1] = bp1.y - bp0.y;
  max_steps = max(max_steps,int(base_delta[1]/max_trans_step));
  base_delta[2] = angles::shortest_angular_distance(bp0.theta,bp1.theta);
  max_steps = max(max_steps,int(base_delta[2]/max_angle_step));

  double spine_delta = bp1.z - bp0.z;
  max_steps = max(max_steps,int(spine_delta/max_trans_step));

  vector<double> arm_delta(14,0);
  for(unsigned int j=0; j<entry1->angles0.size(); j++){
    arm_delta[j] = angles::shortest_angular_distance(entry0->angles0[j],entry1->angles0[j]);
    max_steps = max(max_steps,int(arm_delta[j]/max_angle_step));
    arm_delta[j+7] = angles::shortest_angular_distance(entry0->angles1[j],entry1->angles1[j]);
    max_steps = max(max_steps,int(arm_delta[j+7]/max_angle_step));
  }

  max_steps++;

  BodyPose body_step(base_delta[0]/max_steps, base_delta[1]/max_steps, spine_delta/max_steps, base_delta[2]/max_steps);
  vector<double> arm0_step(7,0);
  vector<double> arm1_step(7,0);
  for(unsigned int j=0; j<7; j++){
    arm0_step[j] = arm_delta[j]/max_steps;
    arm1_step[j] = arm_delta[j+7]/max_steps;
  }

  bool fillVectors = arm0_pts && arm1_pts && body_pts;

  unsigned char dist_temp;
  int debug_code_;
  BodyPose temp_body;
  vector<double> temp_arm0(7,0);
  vector<double> temp_arm1(7,0);
  bool isCollision = false;
  for(int j=0; j<=max_steps; j++){
    temp_body.x = bp0.x+j*body_step.x;
    temp_body.y = bp0.y+j*body_step.y;
    temp_body.z = bp0.z+j*body_step.z;
    temp_body.theta = bp0.theta+j*body_step.theta;

    for(unsigned int k=0; k<7; k++){
      temp_arm0[k] = entry0->angles0[k]+j*arm0_step[k];
      temp_arm1[k] = entry0->angles1[k]+j*arm1_step[k];
    }
    if(!cspace_->checkAllMotion(temp_arm1,temp_arm0,temp_body,true,dist_temp,debug_code_)){
      isCollision = true;
      break;
    }
    if(fillVectors){
      arm0_pts->push_back(temp_arm0);
      arm1_pts->push_back(temp_arm1);
      body_pts->push_back(temp_body);
    }
  }

  return !isCollision;
}



void EnvironmentDUALROBARM3D::convertStateIDPathToJointAnglesPath(const std::vector<int> &idpath, std::vector<std::vector<double> > &path)
{
  ROS_INFO("\n[env] Times: GetSuccs=%f Arms=%f Base=%f Torso=%f\n IK=%f FK=%f Hash=%f\n",double(succ_time)/CLOCKS_PER_SEC,double(arms_time)/CLOCKS_PER_SEC,double(base_time)/CLOCKS_PER_SEC,double(torso_time)/CLOCKS_PER_SEC,double(ik_time)/CLOCKS_PER_SEC,double(fk_time)/CLOCKS_PER_SEC,double(hash_time)/CLOCKS_PER_SEC);

  int sourceid, targetid, bestcost, bestsucc;
  std::vector<int> cost, succid, action;
  std::vector<int> num_per_group(4,0); //TODO: Assumes there are 4 groups of mprims. Fix this.
  std::vector<double> sangles0(njoints_,0), sangles1(njoints_,0), interm_angles0(njoints_,0), interm_angles1(njoints_,0), interm_wcoord(ndof_,0), source_wcoord(ndof_,0);
  std::vector<double> spine(1,0), base(3,0);
  std::vector<short unsigned int> interm_coord(ndof_,0);
  EnvDUALROBARM3DHashEntry_t* source_entry;
 
  path.clear();

  ROS_DEBUG_NAMED(prms_.solution_log_, "[env] idpath has length %d", int(idpath.size()));
  if(idpath.size()==1){
    source_entry = EnvROBARM.StateID2CoordTable[idpath[0]];
    coordToWorldPose(source_entry->coord, source_wcoord);

    base[0] = source_wcoord[8];
    base[1] = source_wcoord[9];
    base[2] = source_wcoord[11];

    spine[0] = source_wcoord[10];

    //store
    path.push_back(source_entry->angles0);
    path.push_back(source_entry->angles1);
    path.push_back(spine);
    path.push_back(base);
    path.push_back(source_entry->angles0);
    path.push_back(source_entry->angles1);
    path.push_back(spine);
    path.push_back(base);
    return;
  }
  if(idpath.size()==0)
  {
    ROS_ERROR("[env] idpath has length = 0. Not returning the joint angles path.");
    return;
  }

  for(size_t p = 0; p < idpath.size()-1; ++p)
  {
    sourceid = idpath[p];
    targetid = idpath[p+1];

    GetSuccs(sourceid, &succid, &cost, &action);
    //ROS_ERROR("has anybody seen a %d???",targetid);

    bestcost = INFINITECOST;
    bestsucc = -2;

    for(size_t s = 0; s < succid.size(); ++s)
    {
      if(succid[s] == targetid && cost[s] <= bestcost)
      {
        bestcost = cost[s];
        bestsucc = action[s];
      }
    }

    if(bestsucc == -2){
      ROS_ERROR("[%i] Successor not found for transition.", int(p));
      source_entry = EnvROBARM.StateID2CoordTable[sourceid];
      EnvDUALROBARM3DHashEntry_t* target_entry = EnvROBARM.StateID2CoordTable[targetid];
      if(source_entry->coord[8]==target_entry->coord[8] &&
         source_entry->coord[9]==target_entry->coord[9] &&
         source_entry->coord[10]==target_entry->coord[10] &&
         source_entry->coord[11]==target_entry->coord[11]){
        SBPL_INFO("Only the arms changed in this missing transition. IK must not have found the reverse move. Let it slide...");
        BodyCell bc(source_entry->coord[8],source_entry->coord[9],source_entry->coord[10],source_entry->coord[11]);
        BodyPose body_pose;
        discToWorldBody(bc,&body_pose);

        spine[0] = body_pose.z;
        base[0] = body_pose.x;
        base[1] = body_pose.y;
        base[2] = body_pose.theta;

        path.push_back(target_entry->angles0);
        path.push_back(target_entry->angles1);
        path.push_back(spine);
        path.push_back(base);
      }
      continue;
    }

    source_entry = EnvROBARM.StateID2CoordTable[sourceid];
    sangles0 = source_entry->angles0;
    sangles1 = source_entry->angles1;
   
    coordToWorldPose(source_entry->coord, source_wcoord);

    if(bestsucc == -1){
      //special case for snap motions
      EnvDUALROBARM3DHashEntry_t* target_entry = EnvROBARM.StateID2CoordTable[targetid];
      vector<vector<double> > arm0_pts;
      vector<vector<double> > arm1_pts;
      vector<BodyPose> body_pts;
      computeIntermPoints(source_entry, target_entry, &arm0_pts, &arm1_pts, &body_pts);
      for(unsigned int i=0; i<body_pts.size(); i++){
        spine[0] = body_pts[i].z;
        base[0] = body_pts[i].x;
        base[1] = body_pts[i].y;
        base[2] = body_pts[i].theta;
        path.push_back(arm0_pts[i]);
        path.push_back(arm1_pts[i]);
        path.push_back(spine);
        path.push_back(base);

        ROS_DEBUG_NAMED(prms_.solution_log_, "[%i-%i] snap sourceid: %d targetid: %d xyz: %0.3f %0.3f %0.3f rpy: %0.3f %0.3f %0.3f fa: %0.3f %0.3f base_xyz: %0.3f %0.3f %0.3f %0.3f",int(p),int(i), sourceid, targetid, interm_wcoord[0],interm_wcoord[1],interm_wcoord[2],interm_wcoord[3],interm_wcoord[4],interm_wcoord[5],interm_wcoord[6],interm_wcoord[7],base[0],base[1],spine[0],base[2]);
      }
    }
    else if(bestsucc < int(prms_.mp_.size())){
      //arm motion
      for(size_t i = 0; i < prms_.mp_[bestsucc].m.size(); ++i)
      {
        for(size_t a = 0; a < source_wcoord.size()-4; ++a)
          interm_wcoord[a] = source_wcoord[a] + prms_.mp_[bestsucc].m[i][a];
        //interm_wcoord[11] = prms_.mp_[bestsucc].m[i][11];
        
        num_per_group[prms_.mp_[bestsucc].group]++;

        ROS_DEBUG_NAMED(prms_.solution_log_, "[%i-%i] sourceid: %d targetid: %d mprim.id: %d mprim.group: %d xyz: %0.3f %0.3f %0.3f rpy: %0.3f %0.3f %0.3f fa: %0.3f %0.3f",int(p),int(i), sourceid, targetid, prms_.mp_[bestsucc].id, prms_.mp_[bestsucc].group, interm_wcoord[0],interm_wcoord[1],interm_wcoord[2],interm_wcoord[3],interm_wcoord[4],interm_wcoord[5],interm_wcoord[6],interm_wcoord[7]);

        if(convertWorldPoseToAngles(interm_wcoord, interm_angles0, interm_angles1, true))
        {
          for(size_t q = 0; q < interm_angles0.size(); ++q)
          {
            interm_angles0[q] = angles::normalize_angle(interm_angles0[q]);
            interm_angles1[q] = angles::normalize_angle(interm_angles1[q]);
          }
          spine[0] = source_wcoord[10];//interm_wcoord[10];
          base[0] = source_wcoord[8];//interm_wcoord[8];
          base[1] = source_wcoord[9];//interm_wcoord[9];
          base[2] = source_wcoord[11];//interm_wcoord[11];

          //right arm, left arm, spine, xytheta
          path.push_back(interm_angles0); 
          path.push_back(interm_angles1); 
          path.push_back(spine);
          path.push_back(base);
        }
        else
        {
          SBPL_WARN("[%i-%i] Can't convert world coords to angles.",int(p),int(i));
          SBPL_WARN("This MP has %d steps", int(prms_.mp_[bestsucc].m.size()));
          SBPL_WARN("    MP: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", prms_.mp_[bestsucc].m[i][0], prms_.mp_[bestsucc].m[i][1], prms_.mp_[bestsucc].m[i][2], prms_.mp_[bestsucc].m[i][3], prms_.mp_[bestsucc].m[i][4], prms_.mp_[bestsucc].m[i][5], prms_.mp_[bestsucc].m[i][6], prms_.mp_[bestsucc].m[i][7]);
          SBPL_WARN("    hash_entry->angles0: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", sangles0[0],sangles0[1],sangles0[2],sangles0[3],sangles0[4],sangles0[5],sangles0[6]);
          SBPL_WARN("    hash_entry->angles1: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", sangles1[0],sangles1[1],sangles1[2],sangles1[3],sangles1[4],sangles1[5],sangles1[6]);
          path.push_back(sangles0);
          path.push_back(sangles1);

          spine[0] = interm_wcoord[10];
          base[0] = interm_wcoord[8];
          base[1] = interm_wcoord[9];
          base[2] = interm_wcoord[11];

          path.push_back(spine);
          path.push_back(base);
        }
      }
    }
    else if(bestsucc < int(prms_.mp_.size() + prms_.base_action_width_+prms_.num_orbit_motions_)){//+3 for adaptive
      //base motion
      int a = bestsucc - prms_.mp_.size();
      int i;
      EnvNAVXYTHETALATAction_t action;
      if(bestsucc == int(prms_.mp_.size() + prms_.base_action_width_)){
        //adaptive base motion
        computeOrbitMotion(1,source_entry,&action);
      }
      else if(bestsucc == int(prms_.mp_.size() + prms_.base_action_width_+1)){
        //adaptive base motion
        computeOrbitMotion(-1,source_entry,&action);
      }
      else if(bestsucc == int(prms_.mp_.size() + prms_.base_action_width_+2)){
        //adaptive base motion
        computeAdaptiveBaseMotion(source_entry,&action);
      }
      else
        action = prms_.base_actions_[source_entry->coord[11]][a];
      for(i=0; i < ((int)action.intermptV.size()); i++){
        //translate appropriately
        sbpl_xy_theta_pt_t intermpt = action.intermptV[i];
        base[0] = source_wcoord[8] + intermpt.x;
        base[1] = source_wcoord[9] + intermpt.y;
        base[2] = intermpt.theta;

        spine[0] = source_wcoord[10];

        //store
        path.push_back(source_entry->angles0);
        path.push_back(source_entry->angles1);
        path.push_back(spine);
        path.push_back(base);
      }
      ROS_DEBUG_NAMED(prms_.solution_log_, "[%i-%i] sourceid: %d targetid: %d mprim.group: base  base_xyz: %0.3f %0.3f %0.3f base_theta: %0.3f",int(p),int(i), sourceid, targetid, base[0],base[1],spine[0],base[2]);
    }
    else{
      //spine motion
      int a = bestsucc - (prms_.mp_.size() + prms_.base_action_width_ + prms_.num_orbit_motions_);//+3 for base adaptive
      if(a==0){
        //spine down
        spine[0] = source_wcoord[10] - prms_.xyz_resolution_;
      }
      else{
        //spine up
        spine[0] = source_wcoord[10] - prms_.xyz_resolution_;
      }

      base[0] = source_wcoord[8];
      base[1] = source_wcoord[9];
      base[2] = source_wcoord[11];

      //store
      path.push_back(source_entry->angles0);
      path.push_back(source_entry->angles1);
      path.push_back(spine);
      path.push_back(base);
      ROS_DEBUG_NAMED(prms_.solution_log_, "[%d-%d] sourceid: %d targetid: %d mprim.group: torso  base_xyz: %0.3f %0.3f %0.3f base_theta: %0.3f",int(p),int(a), sourceid, targetid, base[0],base[1],spine[0],base[2]);
    }
  }

  ROS_INFO("[env] mprim.group 0: %d  mprim.group 1: %d  mprim.group 2: %d  mprim.group 3: %d", num_per_group[0],num_per_group[1],num_per_group[2],num_per_group[3]); 

  /*
  for(unsigned int i=0; i<path.size(); i++){
    for(unsigned int j=0; j<path[i].size(); j++)
      printf("%f ",path[i][j]);
    printf("\n");
  }
  */

}

void EnvironmentDUALROBARM3D::convertStateIDPathToShortenedJointAnglesPath(const std::vector<int> &idpath, std::vector<std::vector<double> > &path, std::vector<int> &idpath_short)
{
  //TODO: No shortcutting yet...
  ROS_ERROR("[env] Haven't implemented convertStateIDPathToShortenedJointAnglesPath() yet...");
  return;

  int sourceid, targetid, bestcost, bestsucc;
  std::vector<int> cost, succid, action, mp_path, idpath2;
  std::vector<int> num_per_group(4,0); //TODO: Assumes there are 4 groups of mprims. Fix this.
  std::vector<double> interm_angles0(njoints_,0), interm_angles1(njoints_,0), interm_wcoord(ndof_,0), source_wcoord(ndof_,0);
  std::vector<short unsigned int> interm_coord(ndof_,0);
  EnvDUALROBARM3DHashEntry_t* source_entry;
  path.clear();
  idpath_short.clear();

  //retrieve list of motion primitives used
  for(size_t p = 0; p < idpath.size()-1; ++p)
  {
    sourceid = idpath[p];
    targetid = idpath[p+1];

    GetSuccs(sourceid, &succid, &cost, &action);

    bestcost = INFINITECOST;
    bestsucc = -1;

    for(size_t s = 0; s < succid.size(); ++s)
    {
      if(succid[s] == targetid && cost[s] <= bestcost)
      {
        bestcost = cost[s];
        bestsucc = action[s];
      }
    }

    if(bestsucc == -1)
    {
      ROS_ERROR("[%i] When retrieving the path, successor not found for transition.", int(p));
      return;
    }
    mp_path.push_back(bestsucc);
  }

  //remove stateids that use the same mprim as previous stateid
  idpath2 = idpath;
  for(int p = int(mp_path.size()-1); p > 0; p--)
  {
    if(mp_path[p] == mp_path[p-1])
      idpath2[p-1] = -1;
  }

  //debugging
  for(size_t p = 0; p < idpath.size()-1; ++p)
    ROS_DEBUG_NAMED(prms_.solution_log_,"[%d] original path: %d  mprim: %d  adjusted path %d", int(p), idpath[p], mp_path[p], idpath2[p]);

  //get joint angles for shortened paths
  for(size_t p = 0; p < idpath2.size()-1; ++p)
  {
    if(idpath2[p] == -1)
      continue;

    idpath_short.push_back(idpath[p]);

    sourceid = idpath2[p];
    targetid = idpath2[p+1];

    source_entry = EnvROBARM.StateID2CoordTable[sourceid];
    coordToWorldPose(source_entry->coord, source_wcoord);

    bestsucc = mp_path[p];
    for(size_t i = 0; i < prms_.mp_[bestsucc].m.size(); ++i)
    {
      for(size_t a = 0; a < source_wcoord.size(); ++a)
        interm_wcoord[a] = source_wcoord[a] + prms_.mp_[bestsucc].m[i][a];

      ROS_DEBUG_NAMED(prms_.solution_log_,"[%i-%i] sourceid: %d targetid: %d mprim: %d xyz: %0.3f %0.3f %0.3f rpy: %0.3f %0.3f %0.3f fa: %0.3f %0.3f",int(p),int(i), sourceid, targetid, bestsucc, interm_wcoord[0],interm_wcoord[1],interm_wcoord[2],interm_wcoord[3],interm_wcoord[4],interm_wcoord[5],interm_wcoord[6],interm_wcoord[7]);

      /*
      //TODO: allow_search should not always be set to true. not for FA mprims
      if(convertWorldPoseToAngles(interm_wcoord, interm_angles0, interm_angles1, true))
      {
        for(size_t q = 0; q < interm_angles0.size(); ++q)
        {
          interm_angles0[q] = angles::normalize_angle(interm_angles0[q]);
          interm_angles1[q] = angles::normalize_angle(interm_angles1[q]);
        }

        path.push_back(interm_angles0); 
        path.push_back(interm_angles1); 
      }
      else
      {
        SBPL_WARN("[%i-%i] When retrieving the path, can't convert world coords to angles. Using stored angles.",int(p),int(i));
      */
        path.push_back(source_entry->angles0);
        path.push_back(source_entry->angles1);
      //}
    }
  }
}

void EnvironmentDUALROBARM3D::convertStateIDPathToPoints(const std::vector<int> &idpath, std::vector<std::vector<double> > &path)
{
  path.clear();
  for(unsigned int i=0; i<idpath.size(); i++){
    vector<double> v(3,0);
    discToWorldXYZ(EnvROBARM.StateID2CoordTable[idpath[i]]->object_pose[0],
                   EnvROBARM.StateID2CoordTable[idpath[i]]->object_pose[1],
                   EnvROBARM.StateID2CoordTable[idpath[i]]->object_pose[2],
                   v[0],v[1],v[2],true);
    path.push_back(v);
  }
}

void EnvironmentDUALROBARM3D::convertShortStateIDPathToPoints(const std::vector<int> &idpath, std::vector<std::vector<double> > &path)
{
  std::vector<double> interm_point(4,0), wcoord(ndof_,0);
 
  path.clear();

  for(size_t p = 0; p < idpath.size()-1; ++p)
  {
    //coordToWorldPose(EnvROBARM.StateID2CoordTable[idpath[p]]->coord, wcoord);
    vector<short unsigned int> c = EnvROBARM.StateID2CoordTable[idpath[p]]->object_pose;
    discToWorldXYZ(int(c[0]),int(c[1]),int(c[2]),interm_point[0],interm_point[1],interm_point[2],true);

    //interm_point[0] = wcoord[0];
    //interm_point[1] = wcoord[1];
    //interm_point[2] = wcoord[2];
    if(p % 2 == 0)
      interm_point[3] = 0;
    else
      interm_point[3] = 1;

    path.push_back(interm_point);
  }
  ROS_INFO("NOTE: convertStateIDPathToPoints will only work with one waypoint motion primitives. It's temporary.");
}

void EnvironmentDUALROBARM3D::getContMotionPrims(char type, std::vector<std::vector<tf::Vector3> > &mprims)
{
  std::vector<tf::Vector3> m;
  tf::Vector3 origin(0.0, 0.0, 0.0);
  mprims.clear();

  for(size_t i = 0; i < prms_.mp_.size(); ++i)
  {
    if(prms_.mp_[i].type == type)
    {
      m.resize(prms_.mp_[i].nsteps+1);
      m[0] = origin;

      for(int j = 0; j < prms_.mp_[i].nsteps; ++j)
      {
        m[j+1].setX(prms_.xyz_resolution_*prms_.mp_[i].m[j][0]);
        m[j+1].setY(prms_.xyz_resolution_*prms_.mp_[i].m[j][1]);
        m[j+1].setZ(prms_.xyz_resolution_*prms_.mp_[i].m[j][2]);

        m[j+1] += m[j];
      }
      mprims.push_back(m);
    }
  }

  ROS_DEBUG("[getContMotionPrimitives] Returning %d motions of type %d.", int(mprims.size()),int(type));
}

void EnvironmentDUALROBARM3D::getGripperCoordsFromObjectPose(const std::vector<double> &object, std::vector<int> &pose0, std::vector<int> &pose1)
{
  //object should be of size 6 {x,y,z,r,p,y}
  //pose0,pose1 should be of size 6
  double roll,pitch,yaw;
  pose0.resize(6,0);
  pose1.resize(6,0);

  //object
  frame_b_o_.p.x(object[0]);
  frame_b_o_.p.y(object[1]);
  frame_b_o_.p.z(object[2]);
  frame_b_o_.M = KDL::Rotation::RPY(object[3],object[4],object[5]);
  ROS_DEBUG_NAMED(prms_.expands2_log_,"[env] [object] xyz: %0.3f %0.3f %0.3f rpy: %0.3f %0.3f %0.3f", frame_b_o_.p[0],frame_b_o_.p[1],frame_b_o_.p[2],object[3],object[4],object[5]);
  
  //right arm
  frame_b_w_ = frame_b_o_ * arm0_offset_;
  worldToDiscXYZ(frame_b_w_.p[0],frame_b_w_.p[1],frame_b_w_.p[2],pose0[0],pose0[1],pose0[2],false);
  frame_b_w_.M.GetRPY(roll,pitch,yaw);
  worldToDiscRPY(roll,pitch,yaw,pose0[3],pose0[4],pose0[5]);
  ROS_DEBUG_NAMED(prms_.expands2_log_,"[env] [right]  xyz: %0.3f %0.3f %0.3f rpy: %0.3f %0.3f %0.3f", frame_b_w_.p[0],frame_b_w_.p[1],frame_b_o_.p[2],roll,pitch,yaw);

  //left arm
  frame_b_w_ = frame_b_o_ * arm1_offset_;
  worldToDiscXYZ(frame_b_w_.p[0],frame_b_w_.p[1],frame_b_w_.p[2],pose1[0],pose1[1],pose1[2],false);
  frame_b_w_.M.GetRPY(roll,pitch,yaw);
  worldToDiscRPY(roll,pitch,yaw,pose1[3],pose1[4],pose1[5]);
  ROS_DEBUG_NAMED(prms_.expands2_log_,"[env] [left]   xyz: %0.3f %0.3f %0.3f rpy: %0.3f %0.3f %0.3f", frame_b_w_.p[0],frame_b_w_.p[1],frame_b_o_.p[2],roll,pitch,yaw);
}

void EnvironmentDUALROBARM3D::getGripperPosesFromObjectPose(const std::vector<double> &object, std::vector<double> &pose0, std::vector<double> &pose1)
{
  //object should be of size 6 {x,y,z,r,p,y}
  //pose0,pose1 should be of size 6
  pose0.resize(6,0);
  pose1.resize(6,0);

  //object
  frame_b_o_.p.x(object[0]);
  frame_b_o_.p.y(object[1]);
  frame_b_o_.p.z(object[2]);
  frame_b_o_.M = KDL::Rotation::RPY(object[3],object[4],object[5]);
  ROS_DEBUG_NAMED(prms_.expands2_log_,"[env] [object] xyz: %0.3f %0.3f %0.3f rpy: %0.3f %0.3f %0.3f", frame_b_o_.p[0],frame_b_o_.p[1],frame_b_o_.p[2],object[3],object[4],object[5]);
  
  //right arm
  frame_b_w_ = frame_b_o_ * arm0_offset_;
  pose0[0] = frame_b_w_.p[0];
  pose0[1] = frame_b_w_.p[1];
  pose0[2] = frame_b_w_.p[2];
  frame_b_w_.M.GetRPY(pose0[3],pose0[4],pose0[5]);
  ROS_DEBUG_NAMED(prms_.expands2_log_,"[env] [right]  xyz: %0.3f %0.3f %0.3f rpy: %0.3f %0.3f %0.3f",pose0[0],pose0[1],pose0[2],pose0[3],pose0[4],pose0[5]);

  //left arm
  frame_b_w_ = frame_b_o_ * arm1_offset_;
  pose1[0] = frame_b_w_.p[0];
  pose1[1] = frame_b_w_.p[1];
  pose1[2] = frame_b_w_.p[2];
  frame_b_w_.M.GetRPY(pose1[3],pose1[4],pose1[5]);
  ROS_DEBUG_NAMED(prms_.expands2_log_,"[env] [left]   xyz: %0.3f %0.3f %0.3f rpy: %0.3f %0.3f %0.3f",pose1[0],pose1[1],pose1[2],pose1[3],pose1[4],pose1[5]);
}

void EnvironmentDUALROBARM3D::getHeuristicDebugStats(std::vector<int> &hval, std::vector<int> &num_exp)
{
  hval = h_values_;
  num_exp = num_exp_per_hval_;
}

void EnvironmentDUALROBARM3D::recordDebugData(bool record)
{
  record_data_ = record;
}

int EnvironmentDUALROBARM3D::getDijkstraDistance(double x, double y, double z)
{
  int gx, gy, gz;
  std::vector<short unsigned int> p(ndof_,0);

  worldToDiscXYZ(x,y,z,gx,gy,gz,true);
  p[0] = gx;
  p[1] = gy;
  p[2] = gz;

  ROS_INFO("[node] world_xyz: %0.2f %0.2f %0.2f     disc_xyz: %u %u %u", x,y,z, p[0], p[1], p[2]);
  return getDijkstraDistToGoal(p[0], p[1], p[2]);
}

int EnvironmentDUALROBARM3D::getDijkstraDistance(std::vector<short unsigned int> &coord)
{
  
  /* std::vector<int> pose0(3,0), pose1(3,0);
  std::vector<double> object(ndof_,0);
  coordToWorldPose(coord, object);
  */
  return getDijkstraDistToGoal(coord[0], coord[1], coord[2]);
}

bool EnvironmentDUALROBARM3D::checkExpandedStatesAreValid()
{ 
  bool ret_val = true;
  int cntr=0;

  for(size_t i = 0; i < expanded_states.size(); ++i)
  {
    if(!checkCoordMatchesAngles(EnvROBARM.StateID2CoordTable[expanded_states[i]]->coord,EnvROBARM.StateID2CoordTable[expanded_states[i]]->angles0, EnvROBARM.StateID2CoordTable[expanded_states[i]]->angles1))
    {
      ROS_DEBUG("[env] [%d] State %d is invalid. Angles don't match coords.",int(i), expanded_states[i]);
      ret_val = false;
      cntr++;
    }
  }

  ROS_INFO("[env] There are %d out of %d invalid expanded states.",cntr, int(expanded_states.size())); 
  ROS_INFO("[env] [right] x: %3d y: %3d  z: %3d  r: %3d p: %3d y: %3d fa0: %3d fa1: %3d", incorrect_right_states_[0],incorrect_right_states_[1],incorrect_right_states_[2],incorrect_right_states_[3],incorrect_right_states_[4],incorrect_right_states_[5],incorrect_right_states_[6],incorrect_right_states_[7]);
  ROS_INFO("[env]  [left] x: %3d y: %3d  z: %3d  r: %3d p: %3d y: %3d fa0: %3d fa1: %3d", incorrect_left_states_[0],incorrect_left_states_[1],incorrect_left_states_[2],incorrect_left_states_[3],incorrect_left_states_[4],incorrect_left_states_[5],incorrect_left_states_[6],incorrect_left_states_[7]);

  return ret_val;
}

bool EnvironmentDUALROBARM3D::checkCoordMatchesAngles(const std::vector<short unsigned int> &coord,
                                                      std::vector<double> &angles0,
                                                      std::vector<double> &angles1)
{
  bool ret_val=true;
  int lcntr=0, rcntr=0;
  double xyz[3] = {0, 0, 0}, rpy[3] = {0, 0, 0}, fa[2] = {0, 0};
  std::vector<double> wpose0, wpose1, object(ndof_,0);
  std::vector<int> pose0(ndof_, 0), pose1(ndof_, 0), rerr(ndof_, 0),lerr(ndof_, 0);
  std::vector<short unsigned int> coord0(ndof_, 0), coord1(ndof_, 0);
  coordToWorldPose(coord, object);
  frame_b_o_.p.x(object[0]);
  frame_b_o_.p.y(object[1]);
  frame_b_o_.p.z(object[2]);
  frame_b_o_.M = KDL::Rotation::RPY(object[3], object[4], object[5]);

  if(!arm_[0]->getPlanningJointPose(angles0, wpose0))
  {
    ROS_WARN("[env] Failed to compute forward kinematics for the right arm.");
    return false;
  }
  if(!arm_[1]->getPlanningJointPose(angles1, wpose1))
  {
    ROS_WARN("[env] Failed to compute forward kinematics for the left arm.");
    return false;
  }

  //right arm desired
  frame_b_w_ = frame_b_o_ * arm0_offset_;
  worldToDiscXYZ(frame_b_w_.p[0],frame_b_w_.p[1],frame_b_w_.p[2],pose0[0],pose0[1],pose0[2],false);
  frame_b_w_.M.GetRPY(rpy[0],rpy[1],rpy[2]);
  worldToDiscRPY(rpy[0],rpy[1],rpy[2],pose0[3],pose0[4],pose0[5]);
  pose0[6] = coord[6];
  pose0[7] = coord[7];

  //right arm actual
  xyz[0] = wpose0[0]+0.002;   //adding 0.002 is a bad fix for arm_->getPlanningJointPose passing back positions with garbage added to it
  xyz[1] = wpose0[1]+0.002;
  xyz[2] = wpose0[2]+0.002;
  frame_b_w_.M.GetRPY(rpy[0],rpy[1],rpy[2]);
  fa[0] = angles0[free_angle_index_];
  fa[1] = angles1[free_angle_index_];
  BodyPose body_pose;
  worldPoseToCoord(xyz,rpy,fa,body_pose,coord0);

  //left arm desired
  frame_b_w_ = frame_b_o_ * arm1_offset_;
  worldToDiscXYZ(frame_b_w_.p[0],frame_b_w_.p[1],frame_b_w_.p[2],pose1[0],pose1[1],pose1[2],false);
  frame_b_w_.M.GetRPY(rpy[0],rpy[1],rpy[2]);
  worldToDiscRPY(rpy[0],rpy[1],rpy[2],pose1[3],pose1[4],pose1[5]);
  pose1[6] = coord[6];
  pose1[7] = coord[7];

  //left arm actual 
  xyz[0] = wpose1[0]+0.002;
  xyz[1] = wpose1[1]+0.002;
  xyz[2] = wpose1[2]+0.002;
  frame_b_w_.M.GetRPY(rpy[0],rpy[1],rpy[2]);
  fa[0] = angles0[free_angle_index_];
  fa[1] = angles1[free_angle_index_];
  worldPoseToCoord(xyz,rpy,fa,body_pose,coord1);

  for(size_t i = 0; i < coord.size(); ++i)
  {
    rerr[i] = fabs(coord0[i] - pose0[i]);
    lerr[i] = fabs(coord1[i] - pose1[i]);

    if(rerr[i] > 0)
    {
      incorrect_right_states_[i]++;
      rcntr++;
    }
    if(lerr[i] > 0)
    {
      incorrect_left_states_[i]++;
      lcntr++;
    }
  }
  
  if(rcntr > 0 || lcntr > 0)
    ret_val = false;

  if(!ret_val)
  {
    ROS_DEBUG_NAMED(prms_.expands_log_, "[env] coord: xyz: %u %u %u rpy: %2u %2u %2u fa: %2u %2u",coord[0],coord[1],coord[2],coord[3],coord[4],coord[5],coord[6],coord[7]);
    ROS_DEBUG_NAMED(prms_.expands_log_, "[env] right: xyz: %u(%2d) %u(%2d) %u(%2d) rpy: %2u(%2u) %2u(%u) %2u(%2u) fa: %2u(%2u) %2u(%2u)  (# wrong coord: %d)",coord0[0],pose0[0],coord0[1],pose0[1],coord0[2],pose0[2],coord0[3],coord[3],coord0[4],coord[4],coord0[5],coord[5],coord0[6],coord[6],coord0[7],coord[7],rcntr);
    ROS_DEBUG_NAMED(prms_.expands_log_,"[env]  left: xyz: %u(%2d) %u(%2d) %u(%2d) rpy: %2u(%2u) %2u(%u) %2u(%2u) fa: %2u(%2u) %2u(%2u)  (# wrong coord: %d)",coord1[0],pose1[0],coord1[1],pose1[1],coord1[2],pose1[2],coord1[3],coord[3],coord1[4],coord[4],coord1[5],coord[5],coord1[6],coord[6],coord1[7],coord[7],lcntr);
  }

  return ret_val;
}

void EnvironmentDUALROBARM3D::updateSuccStats(int code)
{
  succ_stats_[code]++;
}

void EnvironmentDUALROBARM3D::setObjectRadius(double radius)
{
  object_radius_ = radius;
  //dijkstra_->setRadius(radius);
  ROS_INFO("[env] Setting the radius of the object to %0.2fm.",radius);
}

void EnvironmentDUALROBARM3D::setObjectZInflation(int cells_above, int cells_below)
{
  dijkstra_->setZInflation(cells_above, cells_below);
}

bool EnvironmentDUALROBARM3D::computeObjectPose(BodyPose pose, std::vector<double> angles0, short unsigned int& x, short unsigned int& y, short unsigned int& z, short unsigned int& yaw, bool in_map_frame/*=true*/, bool draw){
  KDL::Frame to_wrist;
  if(in_map_frame)
  {
    if(!arm_[0]->computeFK(angles0,pose,10,&to_wrist))
      return false;
  }
  else
  {
    if(!arm_[0]->computeArmFK(angles0,10,&to_wrist))
      return false;
  }

  double roll1,pitch1,yaw1;
  to_wrist.M.GetRPY(roll1,pitch1,yaw1);
  
  if(draw)
    ROS_INFO("[env] wrist=%0.3f %0.3f %0.3f",roll1,pitch1,yaw1);

  /*
  std::vector<double>s(6,0);
  s[0] = to_wrist.p.x();
  s[1] = to_wrist.p.y();
  s[2] = to_wrist.p.z();
  pviz_.visualizeSphere(s,80,"right_wrist_pose",0.01);

  pviz_.setReferenceFrame("r_wrist_roll_link");
  s[0] = arm0_offset_.p.x();
  s[1] = arm0_offset_.p.y();
  s[2] = arm0_offset_.p.z();
  //printKDLFrame(arm0_offset_,"right arm to object offset AGAIN");
  pviz_.visualizeSphere(s,300,"arm0_offset",0.015);
  pviz_.setReferenceFrame("map");
  */

  int temp_x, temp_y, temp_z;
  //worldToDiscXYZ(to_wrist.p.x(),to_wrist.p.y(),to_wrist.p.z(),temp_x,temp_y,temp_z,in_map_frame);
  if(draw){
    KDL::Frame a;
    vector<double> s(6,0);
    a = arm0_offset_.Inverse() * to_wrist;// * arm0_offset_;
    s[0] = a.p.x();s[1] = a.p.y();s[2] = a.p.z();
    pviz_.visualizeSphere(s,0,"ai_w",0.015);
    usleep(5000);

    char shit;
    if(fabs(s[0] - wtf_[0]) > 0.25 || 
        fabs(s[1] - wtf_[1]) > 0.25 ||
        fabs(s[2] - wtf_[2]) > 0.25)
    {
      printf("Error! The marble may have moved too much!...check.\n");
      cin >> shit;
      sleep(2); 
    }
    wtf_[0] = s[0];wtf_[1] = s[1];wtf_[2] = s[2];
    

    a = arm0_offset_ * to_wrist;// * arm0_offset_;
    s[0] = a.p.x();s[1] = a.p.y();s[2] = a.p.z();
    pviz_.visualizeSphere(s,40,"a_w",0.015);
    usleep(5000);
    
    a = arm0_offset_ * to_wrist.Inverse();// * arm0_offset_;
    s[0] = a.p.x();s[1] = a.p.y();s[2] = a.p.z();
    pviz_.visualizeSphere(s,80,"a_wi",0.015);
    usleep(5000);

    a = arm0_offset_.Inverse() * to_wrist.Inverse();// * arm0_offset_;
    s[0] = a.p.x();s[1] = a.p.y();s[2] = a.p.z();
    pviz_.visualizeSphere(s,120,"ai_wi",0.015);
    usleep(5000);

    a = to_wrist * arm0_offset_.Inverse(); // * arm0_offset_;
    s[0] = a.p.x();s[1] = a.p.y();s[2] = a.p.z();
    pviz_.visualizeSphere(s,160,"w_ai",0.015);
    usleep(5000);

    a = to_wrist * arm0_offset_;// * arm0_offset_;
    s[0] = a.p.x();s[1] = a.p.y();s[2] = a.p.z();
    pviz_.visualizeSphere(s,200,"w_a",0.015);
    usleep(5000);
    
    a = to_wrist.Inverse() * arm0_offset_;// * arm0_offset_;
    s[0] = a.p.x();s[1] = a.p.y();s[2] = a.p.z();
    pviz_.visualizeSphere(s,240,"wi_a",0.015);
    usleep(5000);

    a = to_wrist.Inverse() * arm0_offset_.Inverse();// * arm0_offset_;
    s[0] = a.p.x();s[1] = a.p.y();s[2] = a.p.z();
    pviz_.visualizeSphere(s,280,"wi_ai",0.015);
    usleep(5000);
  }
  
  //KDL::Frame f = arm0_offset_.Inverse() * to_wrist;// * arm0_offset_;
  KDL::Frame f = to_wrist * arm0_offset_.Inverse();
  worldToDiscXYZ(f.p.x(),f.p.y(),f.p.z(),temp_x,temp_y,temp_z,in_map_frame);
  x = temp_x;
  y = temp_y;
  z = temp_z;

  double wr,wp,wy;
  //to_wrist.M.GetRPY(wr,wp,wy);
  //printf("wrist world yaw: %f %f %f\n",wr,wp,wy);
  f.M.GetRPY(wr,wp,wy);
  //printf("object world yaw: %f %f %f\n",wr,wp,wy);

  int roll,pitch,temp_yaw;
  worldToDiscRPY(wr,wp,wy,roll,pitch,temp_yaw);
  yaw = temp_yaw;
  return true;
}

void EnvironmentDUALROBARM3D::printRobotState(std::vector<double> &rangles, std::vector<double> &langles, BodyPose &body_pos, std::string text)
{
    ROS_INFO("robot state:  %s", text.c_str());
      ROS_INFO("     x: %0.3f  y: % 0.3f  z: %0.3f yaw: % 0.3f", body_pos.x, body_pos.y, body_pos.z, body_pos.theta);
        ROS_INFO(" right: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", rangles[0], rangles[1], rangles[2], rangles[3], rangles[4], rangles[5], rangles[6]);
          ROS_INFO("  left: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", langles[0], langles[1], langles[2], langles[3], langles[4], langles[5], langles[6]);
}

void EnvironmentDUALROBARM3D::printKDLFrame(KDL::Frame &f, std::string text)
{
  double r,p,y;
  f.M.GetRPY(r,p,y);
  ROS_INFO("frame: %s", text.c_str());
  ROS_INFO("      x: %0.3f  y: %0.3f  z: %0.3f   r: %0.3f  p: %0.3f  y: %0.3f", f.p.x(), f.p.y(), f.p.z(), r, p, y);
}

}
