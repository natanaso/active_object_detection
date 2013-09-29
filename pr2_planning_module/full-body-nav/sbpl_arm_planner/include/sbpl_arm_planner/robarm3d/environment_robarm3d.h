/*
 * Copyright (c) 2008, Maxim Likhachev
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

/** \Author: Benjamin Cohen /bcohen@willowgarage.com **/

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

#include <angles/angles.h>
#include <sbpl/headers.h>
#include <sbpl_arm_planner/bfs_3d.h>
#include <sbpl_arm_planner/sbpl_collision_space.h>
#include <sbpl_arm_planner/sbpl_arm_planner_params.h>
#include <sbpl_arm_planner/pr2/sbpl_math.h>
#include <sbpl_arm_planner/pr2/orientation_solver.h>
#include <sbpl_arm_planner/pr2/pr2_workspace.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#ifndef __ENVIRONMENT_ROBARM3D_H_
#define __ENVIRONMENT_ROBARM3D_H_

namespace sbpl_arm_planner
{


/** @brief struct that describes a basic pose constraint */
typedef struct
{
  bool is_6dof_goal;
  short unsigned int xyz_tolerance;
  int type;
  int xyz[3];
  double pos[3];
  double rpy[3];
  double q[4];
  double pos_tolerance[3];
  double rpy_tolerance[3];
} GoalPos;

/** @brief A hash entry that contains all state information */
typedef struct
{
  unsigned char dist;											//distance to closest obstacle
  short unsigned int action;              //successor action number
  short unsigned int xyz[3];              //end eff pos (xyz)
  int stateID;                            //hash entry ID number
  int heur;
  std::vector<short unsigned int> coord;
  double rpy[3];
} EnvROBARM3DHashEntry_t;

/* @brief an outdated struct that is gradually being torn apart */
typedef struct
{
  bool bInitialized;

  bool solved_by_ik;
  bool solved_by_os; 
  bool ik_solution_is_valid;

  int num_no_ik_solutions;
  int num_ik_invalid_joint_limits;
  int num_calls_to_ik;
  int num_ik_invalid_path;
  int num_invalid_ik_solutions;
  int num_expands_to_position_constraint;
  double goal_to_obstacle_distance;
  std::vector<double> ik_solution;

  std::vector<double> start_configuration;
  std::vector<double> angledelta;
  std::vector<int> anglevals;
  std::vector<std::vector<double> > obstacles;

  std::vector <GoalPos> EndEffGoals;
  std::vector <std::vector <double> > ParsedGoals;
  std::vector <std::vector <double> > ParsedGoalTolerance;
} EnvROBARM3DConfig_t;

/** main structure that stores environment data used in planning */
typedef struct
{
  EnvROBARM3DHashEntry_t* goalHashEntry;
  EnvROBARM3DHashEntry_t* startHashEntry;

  //Maps from coords to stateID
  int HashTableSize;
  std::vector<EnvROBARM3DHashEntry_t*>* Coord2StateIDHashTable;

  //vector that maps from stateID to coords	
  std::vector<EnvROBARM3DHashEntry_t*> StateID2CoordTable;

}EnvironmentROBARM3D_t;


/** Environment to be used when planning for a Robotic Arm using the SBPL. */
class EnvironmentROBARM3D: public DiscreteSpaceInformation 
{
  public:

    std::vector<int> expanded_states;
    bool save_expanded_states;

    /**
     * @brief Default constructor
    */
    EnvironmentROBARM3D();

    /**
     * @brief Destructor
    */
    ~EnvironmentROBARM3D();

    /** 
     * @brief Initialize the environment from a text file 
     * @param name of environment text file
     * @return true if successful, false otherwise
    */
    bool InitializeEnv(const char* sEnvFile);
    
    bool InitializeEnv(const char* sEnvFile, std::string params_file, std::string arm_file);

    /**
     * @brief Check if the states with StateID1 & StateID2 are equivalent
     * based on an equivalency function with some declared tolerance.
     * @param stateID of first state
     * @param stateID of second state
     * @return true if equivalent, false otherwise
    */
    bool AreEquivalent(int StateID1, int StateID2);

    /*!
     * @brief Initialize the environment and arm planner parameters from
     * text files. Also, initialize KDL chain from a URDF file.
     * @param is pointer to file describing the environment
     * @param is a pointer to file with the Arm planner parameters
     * @param is a URDF describing the manipulator
     * @return true if successful, false otherwise
    */
    bool initEnvironment(std::string arm_description_filename, std::string mprims_filename);

    /**
     * @brief Initialize the start and goal states of the MDP
     * @param always returns true...
    */
    bool InitializeMDPCfg(MDPConfig *MDPCfg);
    
    /**
     * @brief Set the initial joint configuration of the manipulator. Needs
     * to be set every time the planner is called.
     * @param an array of joint angles
     * @return true if successful, false otherwise
    */
    bool setStartConfiguration(std::vector<double> angles);

    /**
     * @brief Get the heuristic from one state to another state. 
     * @param the stateID of the current state
     * @param the stateID of the goal state
     * @return h(s,s')
    */
    int GetFromToHeuristic(int FromStateID, int ToStateID);
    
    /**
     * @brief Get the heuristic of a state to the planner's goal state.
     * @param the stateID of the current state
     * @return h(s,s_goal)
    */
    int GetGoalHeuristic(int stateID);

    /**
     * @brief Get the heuristic of a state to the planner's start state.
     * @param the stateID of the current state
     * @return h(s,s_start)
    */
    int GetStartHeuristic(int stateID);

    /** 
     * @brief Get the successors of the desired state to be expanded.
     * Return vectors with the successors' state IDs and the cost to move
     * from the current state to that state. If the vectors return to the
     * planner empty then the search quits.
     * @param the state ID of the state to be expanded
     * @param a pointer to a vector that will be populated with the
     * successor state IDs.
     * @param a pointer to a vector that will be populated with the costs of
     * transitioning from the current state to the successor state.
    */
    void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV);
    
    /** @brief Not defined. */
    void GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV);
    
    /** 
     * @brief This function searches the hash table for a state by
     * a stateID and returns the joint angles of that entry.
     * @param the stateID of the state to fetch
     * @param a vector of joint angles 
    */
    void StateID2Angles(int stateID, std::vector<double> &angles);

    /** 
     * @brief This function returns the number of hash entries created.
     * @return number of hash entries
    */
    int	SizeofCreatedEnv();

    /**
     * @brief This function prints out the state information of a state.
     * @param the state ID of the desired state
     * @param prints out a little extra information
     * @param the file pointer to print to (stdout by default)
    */
    void PrintState(int stateID, bool bVerbose, FILE* fOut=NULL);

    /** @brief Not defined. */
    void SetAllActionsandAllOutcomes(CMDPSTATE* state);

    /** @brief Not defined. */
    void SetAllPreds(CMDPSTATE* state);

    /** @brief Not defined. */
    void PrintEnv_Config(FILE* fOut);

    /**
     * @brief This function sets the goal position. It must be called before
     * starting a new search.
     * @param a 2D vector of pose constraints {{x1,y1,z1,r1,p1,y1},...}
     * @param a 2D vector of tolerances on the pose constraints
     * {{allowed_err_meters1,allowed_error_radians1},...}
     * @return true if succesful, false otherwise
    */
    bool setGoalPosition(const std::vector <std::vector<double> > &goals, const std::vector<std::vector<double> > &tolerances);

    /**
     * @brief Check if the path is valid using the current occupancy grid.
     * @param a 2D vector of joint angles
     * @return true if successful, false otherwise
    */
    bool isPathValid(const std::vector<std::vector<double> > path);

    /**
     * @brief Check if a certain joint configuration is valid using the current occupancy grid.
     * @param a vector of joint angles
     * @return true if successful, false otherwise
    */
    bool isValidJointConfiguration(const std::vector<double> angles);

    /**
     * @brief This function returns a vector of all of the stateIDs of the
     * states that were expanded during the search. (To be updated soon to
     * return the coordinates of each state
     * @return a vector of state IDs
    */ 
    std::vector<int> debugExpandedStates();

    /**
     * @brief Get the epsilon value used by the planner. Epsilon is a bounds
     * on the suboptimality allowed by the planner.
     * @return epsilon
    */
    double getEpsilon();

    /**
     * @brief This function returns the shortest path to the goal from the
     * starting state. If the dijkstra heuristic is enabled, then it returns
     * the shortest path solved for by dijkstra's algorithm. If it is
     * disabled then it returns the straight line path to the goal.
     * @return a 2D vector of waypoints {{x1,y1,z1},{x2,y2,z2},...}
    */ 
    std::vector<std::vector<double> > getShortestPath();

    /*
     * @brief This function receives a collision map object to update the
     * occupancy grid with.
     * @param a collision map
    */
    void updateOccupancyGridFromCollisionMap(const arm_navigation_msgs::CollisionMap &collision_map);

    /** 
     * @brief For debugging. Print out the IK stats and return the IK
     * solution found during the search.
    */
    void debugAdaptiveMotionPrims();
  
    /**
     * @brief This function is for debugging purposes. It returns the
     * pose of the states that were expanded. The planner node has
     * a function to display these as visualizations in rviz.
     * @param a pointer to a vector of the poses of all of the states
     * expanded during the search (when using ARA*)
    */
    void getExpandedStates(std::vector<std::vector<double> >* ara_states);

    /**
     * @brief Visualize the occupancy grid in rviz.
    */
    void visualizeOccupancyGrid();

    void setReferenceFrameTransform(KDL::Frame f, std::string &name);

    SBPLCollisionSpace* getCollisionSpace() const;
    
    OccupancyGrid* getOccupancyGrid() const;
    
    std::vector<double> getPlanningStats();
    
    bool initArmModel(FILE* aCfg, const std::string robot_description);

    /**
     * @brief interpolate the path between the waypoints in path by desired
     * increment
     * @param path
     * @param increment to interpolate by
    */
    bool interpolatePathToGoal(std::vector<std::vector<double> > &path, double inc);

    /** 
     * @brief compute possible elbow positions based on end effector goal 
     * @param cell of shoulder position {x,y,z} 
     * @param end effector goal coordinates {x, y, z}
     * @param radius of upper arm link in meters 
     * @param radius of fore arm link in meters 
     * @param list of possible elbow cells {{x,y,z}, {x,y,z}...}
    */
    bool getElbowCellsAtGoal(std::vector<int> &shoulder, std::vector<double> &goal_m, double rad1, double rad2, std::vector<std::vector<int> > &cells);

    void getElbowPoints(std::vector<std::vector<double> > &elbow_points);

    void getArmChainRootLinkName(std::string &name);

  private:

    EnvROBARM3DConfig_t EnvROBARMCfg;
    EnvironmentROBARM3D_t EnvROBARM;

    boost::thread *heuristic_thread_;
    boost::mutex heuristic_mutex_;
    bool using_short_mprims_;
    bool elbow_heuristic_completed_;
    std::vector<std::vector<double> > elbow_poses_;
    std::vector<std::vector<int> > elbow_cells_;

    OccupancyGrid *grid_;
    BFS3D *dijkstra_;
    BFS3D *elbow_dijkstra_;
    SBPLArmModel *arm_;
    RPYSolver* rpysolver_;
    SBPLCollisionSpace *cspace_;
    SBPLArmPlannerParams prms_;

    std::string params_filename_;
    std::string arm_desc_filename_;
    
    // function pointers for heuristic function
    int (EnvironmentROBARM3D::*getHeuristic_) (int FromStateID, int ToStateID);

    std::vector<double> final_joint_config;
    /*Configuration at start of orientation planning*/
    std::vector<double> prefinal_joint_config;

    /** hash table */
    unsigned int intHash(unsigned int key);
    unsigned int getHashBin(const std::vector<short unsigned int> &coord);
    EnvROBARM3DHashEntry_t* getHashEntry(const std::vector<short unsigned int> &coord, short unsigned int action, bool bIsGoal);
    EnvROBARM3DHashEntry_t* createHashEntry(const std::vector<short unsigned int> &coord, short unsigned int endeff[3], short unsigned int action);

    /** initialization */
    bool initEnvConfig();
    bool initGeneral();
    void initDijkstra();
    void initElbowDijkstra();
    void readConfiguration(FILE* fCfg);

    /** coordinate frame/angle functions */
    void discretizeAngles();
    void coordToAngles(const std::vector<short unsigned int> &coord, std::vector<double> &angles);
    void anglesToCoord(const std::vector<double> &angle, std::vector<short unsigned int> &coord);

    /** planning */
    bool isGoalPosition(const std::vector<double> &pose, const GoalPos &goal, std::vector<double> jnt_angles, int &cost);
    bool isGoalStateWithIK(const std::vector<double> &pose, const GoalPos &goal, std::vector<double> jnt_angles);
    bool isGoalStateWithOrientationSolver(const GoalPos &goal, std::vector<double> jnt_angles);
    bool precomputeHeuristics();
    bool precomputeElbowHeuristic();

    /** costs */
    int cost(EnvROBARM3DHashEntry_t* HashEntry1, EnvROBARM3DHashEntry_t* HashEntry2, bool bState2IsGoal);
    int getEdgeCost(int FromStateID, int ToStateID);
    void computeCostPerCell();
    void computeCostPerRadian();
    int getActionCost(const std::vector<double> &from_config, const std::vector<double> &to_config, int dist);

    /** output */
    void printHashTableHist();
    void printJointArray(FILE* fOut, EnvROBARM3DHashEntry_t* HashEntry, bool bGoal, bool bVerbose);

    /** distance */
    int getDijkstraDistToGoal(short unsigned int x, short unsigned int y, short unsigned int z) const;
    double getDistToClosestGoal(double *xyz,int *goal_num); 
    int getElbowHeuristic(int FromStateID, int ToStateID);
    int getEndEffectorHeuristic(int FromStateID, int ToStateID);
    int getCombinedHeuristic(int FromStateID, int ToStateID);
    int getEuclideanDistance(int x1, int y1, int z1, int x2, int y2, int z2) const; 
    void getBresenhamPath(const short unsigned int a[],const short unsigned int b[], std::vector<std::vector<int> > *path);

    void clearStats();
};


inline unsigned int EnvironmentROBARM3D::intHash(unsigned int key)
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

inline unsigned int EnvironmentROBARM3D::getHashBin(const std::vector<short unsigned int> &coord)
{
  int val = 0;

  for(short unsigned int i = 0; i < coord.size(); i++)
    val += intHash(coord[i]) << i;

  return intHash(val) & (EnvROBARM.HashTableSize-1);
}

//angles are counterclockwise from 0 to 360 in radians, 0 is the center of bin 0, ...
inline void EnvironmentROBARM3D::coordToAngles(const std::vector<short unsigned int> &coord, std::vector<double> &angles)
{
  for(short unsigned int i = 0; i < coord.size(); i++)
    angles[i] = coord[i]*EnvROBARMCfg.angledelta[i];
}

inline void EnvironmentROBARM3D::anglesToCoord(const std::vector<double> &angle, std::vector<short unsigned int> &coord)
{
  double pos_angle;

  for(int i = 0; i < int(angle.size()); i++)
  {
    //NOTE: Added 3/1/09
    pos_angle = angle[i];
    if(pos_angle < 0.0)
      pos_angle += 2*M_PI;

    coord[i] = (int)((pos_angle + EnvROBARMCfg.angledelta[i]*0.5)/EnvROBARMCfg.angledelta[i]);

    if(coord[i] == EnvROBARMCfg.anglevals[i])
      coord[i] = 0;
  }
}

/* intended to be used to measure distance in cells on a grid */
int EnvironmentROBARM3D::getEuclideanDistance(int x1, int y1, int z1, int x2, int y2, int z2) const
{
  return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2));
}

inline SBPLCollisionSpace* EnvironmentROBARM3D::getCollisionSpace() const
{
  return cspace_;
}

inline OccupancyGrid* EnvironmentROBARM3D::getOccupancyGrid() const
{
  return grid_;
}


} //namespace

#endif

