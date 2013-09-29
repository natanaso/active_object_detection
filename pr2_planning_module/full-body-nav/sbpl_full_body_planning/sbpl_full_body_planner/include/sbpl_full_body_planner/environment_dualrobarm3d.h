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

/** \Author: Benjamin Cohen /bcohen@seas.upenn.edu **/

#ifndef __ENVIRONMENT_DUALROBARM3D_H_
#define __ENVIRONMENT_DUALROBARM3D_H_

#include <time.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fstream>
#include <vector>
#include <string>
#include <list>
#include <algorithm>

#include <sbpl/headers.h>
#include <angles/angles.h>
#include <tf/LinearMath/Vector3.h>
#include <sbpl_arm_planner/bfs_3d.h>
#include <sbpl_arm_planner/occupancy_grid.h>
#include <sbpl_arm_planner/sbpl_arm_planning_error_codes.h>
#include <sbpl_full_body_planner/pr2_collision_space.h>
#include <sbpl_full_body_planner/sbpl_full_body_params.h>
#include <pviz/pviz.h>
#include <boost/lexical_cast.hpp>

namespace sbpl_full_body_planner {

/** @brief struct that describes a basic pose constraint */
typedef struct
{
  char type;
  double xyz[3];
  double rpy[3];
  double fangle[2];
  double xyz_tol;
  double roll_tol;
  double pitch_tol;
  double yaw_tol;
} GoalPos;

/** @brief A hash entry that contains all state information */
typedef struct
{
  int stateID;
  double dist;
  short unsigned int xyz[3];
  std::vector<short unsigned int> coord;
  std::vector<double> angles0;
  std::vector<double> angles1;

  //x,y,z,theta
  std::vector<short unsigned int> object_pose;
} EnvDUALROBARM3DHashEntry_t;

/* @brief an outdated struct that is gradually being torn apart */
typedef struct
{
  bool bInitialized;

  double xyz_resolution;
  double rpy_resolution;
  double fangle_resolution;

  std::vector<double> start_configuration;
  std::vector<double> coord_delta;
  std::vector<int> coord_vals;
  std::vector<std::vector<double> > obstacles;

  GoalPos goal;
  std::vector <std::vector <double> > ParsedGoals;
  std::vector <std::vector <double> > ParsedGoalTolerance;
} EnvDUALROBARM3DConfig_t;

/** main structure that stores environment data used in planning */
typedef struct
{
  EnvDUALROBARM3DHashEntry_t* goalHashEntry;
  EnvDUALROBARM3DHashEntry_t* startHashEntry;

  //Maps from coords to stateID
  int HashTableSize;
  std::vector<EnvDUALROBARM3DHashEntry_t*>* Coord2StateIDHashTable;

  //vector that maps from stateID to coords	
  std::vector<EnvDUALROBARM3DHashEntry_t*> StateID2CoordTable;

} EnvironmentDUALROBARM3D_t;


/** Environment to be used when planning for a Robotic Arm using the SBPL. */
class EnvironmentDUALROBARM3D: public DiscreteSpaceInformation 
{
  public:

    std::vector<int> expanded_states;
    bool save_expanded_states;

    EnvironmentDUALROBARM3D();

    ~EnvironmentDUALROBARM3D();

    bool AreEquivalent(int StateID1, int StateID2);

    bool InitializeMDPCfg(MDPConfig *MDPCfg);

    bool InitializeEnv(const char*){printf("Initialization from a text file is not implemented!");exit(0);};

    bool initEnvironment(std::string arm0_filename, std::string arm1_filename, std::string mprims_filename, std::string base_mprim_filename);
    
    int setStartConfiguration(const std::vector<double> &angles0, const std::vector<double> &angles1, const BodyPose pose, KDL::Frame &arm0_offset, KDL::Frame &arm1_offset);

    int GetFromToHeuristic(int FromStateID, int ToStateID);
    
    int GetGoalHeuristic(int stateID);

    int GetStartHeuristic(int stateID);

    void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV);
    bool boundsCheckBase(BodyPose pose);
    bool computeAdaptiveBaseMotion(EnvDUALROBARM3DHashEntry_t* parent, EnvNAVXYTHETALATAction_t* action);
    bool computeOrbitMotion(int dir, EnvDUALROBARM3DHashEntry_t* parent, EnvNAVXYTHETALATAction_t* action);

    void GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV);
    
    void StateID2Angles(int stateID, std::vector<double> &angles0, std::vector<double> &angles1);

    int	SizeofCreatedEnv();

    void PrintState(int stateID, bool bVerbose, FILE* fOut=NULL);

    void SetAllActionsandAllOutcomes(CMDPSTATE* state);

    void SetAllPreds(CMDPSTATE* state);

    void PrintEnv_Config(FILE* fOut);

    int setGoalPosition(const std::vector <std::vector<double> > &goals, const std::vector<std::vector<double> > &tolerances, KDL::Frame &arm0_offset, KDL::Frame &arm1_offset, double object_radius);

    double getEpsilon();
    double getEpsilon2();

    void updateOccupancyGridFromCollisionMap(const arm_navigation_msgs::CollisionMap &collision_map);
    
    std::vector<int> debugExpandedStates();
    
    std::vector<std::vector<double> > getShortestPath();
   
    std::vector<std::vector<double> > getShortestPath(int dij_num);

    void getExpandedStates(std::vector<std::vector<double> > &ara_states);

    void getUniqueExpandedStates(std::vector<std::vector<double> > &ara_states);

    void visualizeOccupancyGrid();

    void setReferenceFrameTransform(KDL::Frame f, std::string &name);

    sbpl_full_body_planner::PR2CollisionSpace* getCollisionSpace() const;
 
    sbpl_arm_planner::OccupancyGrid* getOccupancyGrid() const;
    
    std::vector<double> getPlanningStats();

    void getArmChainRootLinkName(std::string &name);

    bool computeIntermPoints(EnvDUALROBARM3DHashEntry_t* entry0, EnvDUALROBARM3DHashEntry_t* entry1, vector<vector<double> >* arm0_pts=NULL, vector<vector<double> >* arm1_pts=NULL, vector<BodyPose>* body_pts=NULL);

    void convertStateIDPathToJointAnglesPath(const std::vector<int> &idpath, std::vector<std::vector<double> > &path);
    
    void convertStateIDPathToShortenedJointAnglesPath(const std::vector<int> &idpath, std::vector<std::vector<double> > &path, std::vector<int> &idpath_short);

    void convertStateIDPathToPoints(const std::vector<int> &idpath, std::vector<std::vector<double> > &path);

    //TODO: Will not work with multiple waypoint motion prims...It's just for fast debugging.
    void convertShortStateIDPathToPoints(const std::vector<int> &idpath, std::vector<std::vector<double> > &path);

    void getContMotionPrims(char type, std::vector<std::vector<tf::Vector3> > &mprims);

    void getHeuristicDebugStats(std::vector<int> &hval, std::vector<int> &num_exp);

    void recordDebugData(bool record);

    int getDijkstraDistance(double x, double y, double z);
    
    int getDijkstraDistance(std::vector<short unsigned int> &coord);

    void getFinalArmConfigurations(std::vector<std::vector<double> > &arm0, std::vector<std::vector<double> > &arm1);

    bool checkCoordMatchesAngles(const std::vector<short unsigned int> &coord, std::vector<double> &angles0, std::vector<double> &angles1);

    bool checkExpandedStatesAreValid();

    void setObjectRadius(double radius);
 
    void setObjectZInflation(int cells_above, int cells_below);

    bool useGoalID_;
    bool computeNormalHeuristic;

    void getHeuristicTime(double* init_time, double* in_search_time){dijkstra_->getHeuristicTime(init_time,in_search_time);};

    sbpl_full_body_planner::SBPLFullBodyParams prms_;
  protected:
    virtual void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV, vector<int>* ActionV=NULL );


    EnvDUALROBARM3DConfig_t EnvROBARMCfg;
    EnvironmentDUALROBARM3D_t EnvROBARM;
    bool using_short_mprims_;

    double arm_grid_offset_[3];
    clock_t succ_time;
    clock_t arms_time;
    clock_t base_time;
    clock_t torso_time;
    clock_t ik_time;
    clock_t fk_time;
    clock_t hash_time;

    unsigned char** grid2D;
    SBPL2DGridSearch* grid2Dsearchfromgoal;
    sbpl_arm_planner::OccupancyGrid *grid_;
    sbpl_arm_planner::BFS3D* dijkstra_;
    sbpl_arm_planner::SBPLArmModel* arm_[2];
    sbpl_full_body_planner::PR2CollisionSpace *cspace_;

    std::string params_filename_;
    std::string arm0_filename_;
    std::string arm1_filename_;
    std::string mprims_filename_;
    std::string base_mprim_filename_;
    std::vector<double> start0_;
    std::vector<double> start1_;
    int free_angle_index_;
    int ndof_;
    int njoints_;
    double object_radius_;

    KDL::Frame arm0_offset_;
    KDL::Frame arm1_offset_;
    KDL::Frame frame_b_o_;    //temp
    KDL::Frame frame_b_w_;    //temp

    /* for debugging */
    bool record_data_;
    int debug_code_;
    std::vector<int> h_values_;
    std::vector<int> num_exp_per_hval_;
    std::vector<double> succ_stats_;
    int right_ik_search_success_;
    int left_ik_search_success_;
    int left_ik_search_;
    int right_ik_search_;
    std::vector<int> incorrect_left_states_;
    std::vector<int> incorrect_right_states_;
    PViz pviz_;

    double wtf_[3];

    /** hash table */
    unsigned int intHash(unsigned int key);
    unsigned int getHashBin(const std::vector<short unsigned int> &coord);

    EnvDUALROBARM3DHashEntry_t* getHashEntry(const std::vector<short unsigned int> &coord, bool bIsGoal);
    EnvDUALROBARM3DHashEntry_t* getHashEntry(short unsigned int *xyz, short unsigned int *rpy, short unsigned int *fangle, BodyCell body, bool bIsGoal);
    EnvDUALROBARM3DHashEntry_t* createHashEntry(const std::vector<short unsigned int> &coord);

    /** initialization */
    bool initEnvConfig();
    bool initGeneral();
    void initDijkstra(); //+ Increase radius
    bool initArmModel(FILE* arm0_file, FILE* arm1_file, const std::string robot_description);
    void readConfiguration(FILE* fCfg);

    /* discretization */
    void coordToPose(const std::vector<short unsigned int> &coord, short unsigned int *xyz, short unsigned int *rpy, short unsigned int *fangle, BodyCell* pose);
    void coordToWorldPose(const std::vector<short unsigned int> &coord, double *xyz, double *rpy, double *fangle, BodyPose* pose);
    void coordToWorldPose(const std::vector<short unsigned int> &coord, std::vector<double> &wcoord);
    void stateIDToPose(int stateID, short unsigned int *xyz, short unsigned int *rpy, short unsigned int *fangle, BodyCell* pose);
    void stateIDToWorldPose(int stateID, double *wxyz, double *wrpy, double *wfangle, BodyPose* pose);
    void worldPoseToCoord(double *wxyz, double *wrpy, double *wfangle, BodyPose pose, std::vector<short unsigned int> &coord);
    void worldPoseToState(double *wxyz, double *wrpy, double *wfangle, BodyPose pose, bool is_goal, EnvDUALROBARM3DHashEntry_t *state);
    void coordToAngles(const std::vector<short unsigned int> &coord, std::vector<double> &angles0, std::vector<double> &angles1);

    void discToWorldXYZ(short unsigned int *xyz, double *wxyz, bool inMap);
    void discToWorldXYZ(int x, int y, int z, double &wx, double &wy, double &wz, bool inMap);
    void discToWorldRPY(short unsigned int *rpy, double *wrpy);
    void discToWorldFAngle(short unsigned int *fangle, double *wfangle);
    void discToWorldBody(BodyCell bc, BodyPose* pose);
    void worldToDiscXYZ(double wx, double wy, double wz, int &x, int &y, int &z, bool inMap);
    void worldToDiscXYZ(double *wxyz, short unsigned int *xyz, bool inMap);
    void worldToDiscRPY(double *wrpy, short unsigned int *rpy);
    void worldToDiscRPY(double wr, double wp, double wy, int &r, int &p, int &y);
    void worldToDiscFAngle(double *wfangle, short unsigned int *fangle);
    void worldToDiscBody(BodyPose pose, BodyCell* bc);
    bool convertCoordToAngles(const std::vector<short unsigned int> &coord, std::vector<double> &angles0, std::vector<double> &angles1);
    bool convertWorldPoseToAngles(const std::vector<double> &wpose, std::vector<double> &angles0, std::vector<double> &angles1, bool allow_search, bool draw=false);
    void getGripperCoordsFromObjectPose(const std::vector<double> &object, std::vector<int> &pose0, std::vector<int> &pose1);
    void getGripperPosesFromObjectPose(const std::vector<double> &object, std::vector<double> &pose0, std::vector<double> &pose1);

    /** planning */
    bool isGoalPosition(double *xyz, double *rpy, double *fangle, BodyPose pose);
    bool isGoalPosition(int x, int y, int z, int yaw);
    bool precomputeHeuristics();

    /** costs */
    int cost(EnvDUALROBARM3DHashEntry_t* HashEntry1, EnvDUALROBARM3DHashEntry_t* HashEntry2, bool bState2IsGoal);
    int getEdgeCost(int FromStateID, int ToStateID);
    void computeCostPerCell();
    int getActionCost(const std::vector<double> &from_config, const std::vector<double> &to_config, int dist);
    int computeMotionCost(const std::vector<double> &a, const std::vector<double> &b, int i_arm);

    /** output */
    void printHashTableHist();
    void printJointArray(FILE* fOut, EnvDUALROBARM3DHashEntry_t* HashEntry, bool bGoal, bool bVerbose);

    /** distance */
    int getDijkstraDistToGoal(short unsigned int x, short unsigned int y, short unsigned int z) const;
    int getEndEffectorHeuristic(int FromStateID, int ToStateID);
    double getEuclideanDistance(double x1, double y1, double z1, double x2, double y2, double z2);
    void getBresenhamPath(const short unsigned int a[],const short unsigned int b[], std::vector<std::vector<int> > *path);

    /* debugging */
    void clearStats();
    void updateSuccStats(int code);

    bool computeObjectPose(BodyPose pose, std::vector<double> angles0, short unsigned int& x, short unsigned int& y, short unsigned int& z, short unsigned int& yaw, bool in_map_frame=true, bool draw=false);

    void printRobotState(std::vector<double> &rangles, std::vector<double> &langles, BodyPose &body_pos, std::string text);
    void printKDLFrame(KDL::Frame &f, std::string text);
};

inline unsigned int EnvironmentDUALROBARM3D::intHash(unsigned int key)
{
  key += (key << 12); 
  key ^= (key >> 22);
  key += (key << 4);
  key ^= (key >> 9);
  key += (key << 10);
  key ^= (key >> 2);
  key += (key << 7);
  key ^= (key >> 12);
  return key;
}

inline unsigned int EnvironmentDUALROBARM3D::getHashBin(const std::vector<short unsigned int> &coord)
{
  int val = 0;

  for(size_t i = 0; i<coord.size(); ++i)
    val += intHash(coord[i]) << i;

  return intHash(val) & (EnvROBARM.HashTableSize-1);
}

inline double EnvironmentDUALROBARM3D::getEuclideanDistance(double x1, double y1, double z1, double x2, double y2, double z2)
{
  return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2));
}

inline sbpl_full_body_planner::PR2CollisionSpace* EnvironmentDUALROBARM3D::getCollisionSpace() const
{
  return cspace_;
}

inline sbpl_arm_planner::OccupancyGrid* EnvironmentDUALROBARM3D::getOccupancyGrid() const
{
  return grid_;
}

inline void EnvironmentDUALROBARM3D::discToWorldXYZ(short unsigned int *xyz, double *wxyz, bool inMap)
{
  grid_->gridToWorld(int(xyz[0]),int(xyz[1]),int(xyz[2]),wxyz[0],wxyz[1],wxyz[2]);
  if(!inMap){
    wxyz[0] -= arm_grid_offset_[0];
    wxyz[1] -= arm_grid_offset_[1];
    wxyz[2] -= arm_grid_offset_[2];
    SBPL_DEBUG("[env] [discToWorldXYZ] xyz: %2d %2d %2d --> %2.3f %2.3f %2.3f",xyz[0],xyz[1],xyz[2],wxyz[0],wxyz[1],wxyz[2]);
  }
  SBPL_DEBUG("[env] [discToWorldXYZ] xyz: %2d %2d %2d --> %2.3f %2.3f %2.3f",xyz[0],xyz[1],xyz[2],wxyz[0],wxyz[1],wxyz[2]);
}

inline void EnvironmentDUALROBARM3D::discToWorldXYZ(int x, int y, int z, double &wx, double &wy, double &wz, bool inMap)
{
  grid_->gridToWorld(x,y,z,wx,wy,wz);
  if(!inMap){
    wx -= arm_grid_offset_[0];
    wy -= arm_grid_offset_[1];
    wz -= arm_grid_offset_[2];
    SBPL_DEBUG("[env] [discToWorldXYZ] xyz: %2d %2d %2d --> %2.3f %2.3f %2.3f",x,y,z,wx,wy,wz);
  }
  SBPL_DEBUG("[env] [discToWorldXYZ] xyz: %2d %2d %2d --> %2.3f %2.3f %2.3f",x,y,z,wx,wy,wz);
}

inline void EnvironmentDUALROBARM3D::worldToDiscXYZ(double *wxyz, short unsigned int *xyz, bool inMap)
{
  int x,y,z;
  if(inMap)
    grid_->worldToGrid(wxyz[0],wxyz[1],wxyz[2],x,y,z);
  else
    grid_->worldToGrid(wxyz[0]+arm_grid_offset_[0],wxyz[1]+arm_grid_offset_[1],wxyz[2]+arm_grid_offset_[2],x,y,z);
  xyz[0] = x;
  xyz[1] = y;
  xyz[2] = z;
  SBPL_DEBUG("[env] [worldToDiscXYZ 1] xyz: %.12f %.12f %.12f --> %2d %2d %2d",wxyz[0],wxyz[1],wxyz[2],x,y,z);
}

inline void EnvironmentDUALROBARM3D::worldToDiscXYZ(double wx, double wy, double wz, int &x, int &y, int &z, bool inMap)
{
  if(inMap)
    grid_->worldToGrid(wx,wy,wz,x,y,z);
  else
    grid_->worldToGrid(wx+arm_grid_offset_[0],wy+arm_grid_offset_[1],wz+arm_grid_offset_[2],x,y,z);
  SBPL_DEBUG("[env] [worldToDiscXYZ 2] xyz: %.10f %.10f %.10f --> %2d %2d %2d",wx,wy,wz,x,y,z);
}

inline void EnvironmentDUALROBARM3D::discToWorldRPY(short unsigned int *rpy, double *wrpy)
{
  wrpy[0] = angles::normalize_angle(double(rpy[0])*EnvROBARMCfg.coord_delta[3]);// + EnvROBARMCfg.coord_delta[3]*0.5);
  wrpy[1] = angles::normalize_angle(double(rpy[1])*EnvROBARMCfg.coord_delta[4]);// + EnvROBARMCfg.coord_delta[4]*0.5);
  wrpy[2] = angles::normalize_angle(double(rpy[2])*EnvROBARMCfg.coord_delta[5]);// + EnvROBARMCfg.coord_delta[5]*0.5);

  SBPL_DEBUG("[discToWorldRPY] rpy: %d %d %d --> %2.3f %2.3f %2.3f",rpy[0],rpy[1],rpy[2],wrpy[0],wrpy[1],wrpy[2]);
}

inline void EnvironmentDUALROBARM3D::worldToDiscRPY(double *wrpy, short unsigned int *rpy)
{
  rpy[0] = (int)((angles::normalize_angle_positive(wrpy[0] + EnvROBARMCfg.coord_delta[3]*0.5))/EnvROBARMCfg.coord_delta[3]);
  rpy[1] = (int)((angles::normalize_angle_positive(wrpy[1] + EnvROBARMCfg.coord_delta[4]*0.5))/EnvROBARMCfg.coord_delta[4]);
  rpy[2] = (int)((angles::normalize_angle_positive(wrpy[2] + EnvROBARMCfg.coord_delta[5]*0.5))/EnvROBARMCfg.coord_delta[5]);

  SBPL_DEBUG("[worldToDiscRPY] rpy: %2.3f %2.3f %2.3f --> %d %d %d",wrpy[0],wrpy[1],wrpy[2],rpy[0],rpy[1],rpy[2]);
}

inline void EnvironmentDUALROBARM3D::worldToDiscRPY(double wr, double wp, double wy, int &r, int &p, int &y)
{
  r = (int)((angles::normalize_angle_positive(wr + EnvROBARMCfg.coord_delta[3]*0.5))/EnvROBARMCfg.coord_delta[3]);
  p = (int)((angles::normalize_angle_positive(wp + EnvROBARMCfg.coord_delta[4]*0.5))/EnvROBARMCfg.coord_delta[4]);
  y = (int)((angles::normalize_angle_positive(wy + EnvROBARMCfg.coord_delta[5]*0.5))/EnvROBARMCfg.coord_delta[5]);

}

inline void EnvironmentDUALROBARM3D::discToWorldFAngle(short unsigned int *fangle, double *wfangle)
{
  wfangle[0] = angles::normalize_angle(double(fangle[0])*EnvROBARMCfg.coord_delta[6]);
  wfangle[1] = angles::normalize_angle(double(fangle[1])*EnvROBARMCfg.coord_delta[7]);

  SBPL_DEBUG("[discToWorldFAngle] fangle[0]: %d --> %2.3f",fangle[0],wfangle[0]);
  SBPL_DEBUG("[discToWorldFAngle] fangle[1]: %d --> %2.3f",fangle[1],wfangle[1]);
}

inline void EnvironmentDUALROBARM3D::worldToDiscFAngle(double *wfangle, short unsigned int *fangle)
{
  fangle[0] = (int)((angles::normalize_angle_positive(wfangle[0] + EnvROBARMCfg.coord_delta[6]*0.5))/EnvROBARMCfg.coord_delta[6]);
  fangle[1] = (int)((angles::normalize_angle_positive(wfangle[1] + EnvROBARMCfg.coord_delta[7]*0.5))/EnvROBARMCfg.coord_delta[7]);

  SBPL_DEBUG("[worldToDiscFAngle] fangle[0]: %2.3f --> %u",wfangle[0],fangle[0]);
  SBPL_DEBUG("[worldToDiscFAngle] fangle[1]: %2.3f --> %u",wfangle[1],fangle[1]);
}

inline void EnvironmentDUALROBARM3D::discToWorldBody(BodyCell bc, BodyPose* pose){
  grid_->gridToWorld(bc.x,bc.y,bc.z,pose->x,pose->y,pose->z);
  pose->theta = angles::normalize_angle(double(bc.theta)*EnvROBARMCfg.coord_delta[11]);
}

inline void EnvironmentDUALROBARM3D::worldToDiscBody(BodyPose pose, BodyCell* bc){
  grid_->worldToGrid(pose.x,pose.y,pose.z,bc->x,bc->y,bc->z);
  bc->theta = (int)((angles::normalize_angle_positive(pose.theta + EnvROBARMCfg.coord_delta[11]*0.5))/EnvROBARMCfg.coord_delta[11]);
}

inline void EnvironmentDUALROBARM3D::coordToPose(const std::vector<short unsigned int> &coord, short unsigned int *xyz, short unsigned int *rpy, short unsigned int *fangle, BodyCell* pose)
{
  xyz[0] = coord[0];
  xyz[1] = coord[1];
  xyz[2] = coord[2];
  rpy[0] = coord[3];
  rpy[1] = coord[4];
  rpy[2] = coord[5];
  fangle[0] = coord[6];
  fangle[1] = coord[7];
  pose->x = coord[8];
  pose->y = coord[9];
  pose->z = coord[10];
  pose->theta = coord[11];

  ROS_DEBUG("[coordToPose] xyz: %u %u %u  rpy: %u %u %u  fa: %u %u body: %u %u %u %u", xyz[0],xyz[1],xyz[2],rpy[0],rpy[1],rpy[2],fangle[0],fangle[1],pose->x,pose->y,pose->z,pose->theta);
}

inline void EnvironmentDUALROBARM3D::coordToWorldPose(const std::vector<short unsigned int> &coord, double *wxyz, double *wrpy, double *wfangle, BodyPose* pose)
{
  short unsigned int xyz[3]={0}, rpy[3]={0}, fangle[2]={0};
  BodyCell bc;
  
  coordToPose(coord,xyz,rpy,fangle,&bc);

  discToWorldXYZ(xyz,wxyz,false);
  discToWorldRPY(rpy,wrpy);
  discToWorldFAngle(fangle,wfangle);
  discToWorldBody(bc,pose);
  
  ROS_DEBUG("[coordToWorldPose] xyz: %0.3f %0.3f %0.3f  rpy: %0.3f %0.3f %0.3f  fa: %0.3f %0.3f", wxyz[0],wxyz[1],wxyz[2],wrpy[0],wrpy[1],wrpy[2],wfangle[0],wfangle[1]);
}

inline void EnvironmentDUALROBARM3D::coordToWorldPose(const std::vector<short unsigned int> &coord, std::vector<double> &wcoord)
{
  double xyz[3]={0.0}, rpy[3]={0.0}, fangle[2]={0.0};
  BodyPose pose;
  
  coordToWorldPose(coord, xyz, rpy, fangle, &pose);
  
  wcoord[0] = xyz[0];
  wcoord[1] = xyz[1];
  wcoord[2] = xyz[2];
  wcoord[3] = rpy[0];
  wcoord[4] = rpy[1];
  wcoord[5] = rpy[2]; 
  wcoord[6] = fangle[0];
  wcoord[7] = fangle[1];
  wcoord[8] = pose.x;
  wcoord[9] = pose.y;
  wcoord[10] = pose.z;
  wcoord[11] = pose.theta;
  
  ROS_DEBUG("[coordToWorldPose] xyz: %0.3f %0.3f %0.3f  rpy: %0.3f %0.3f %0.3f  fa: %0.3f %0.3f", xyz[0],xyz[1],xyz[2],rpy[0],rpy[1],rpy[2],fangle[0],fangle[1]);
}

//angles are counterclockwise from 0 to 360 in radians, 0 is the center of bin 0, ...
inline void EnvironmentDUALROBARM3D::coordToAngles(const std::vector<short unsigned int> &coord, std::vector<double> &angles0, std::vector<double> &angles1)
{
  EnvDUALROBARM3DHashEntry_t* h;
  if((h = getHashEntry(coord,false)) == NULL)
  {
    ROS_WARN("[coordToAngles] Failed to fetch hash entry");
    return;
  }

  angles0 = h->angles0;
  angles1 = h->angles1;
}

} //namespace

#endif

