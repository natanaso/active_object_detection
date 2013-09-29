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

#include <sbpl_arm_planner/robarm3d/environment_robarm3d.h>

#define DEG2RAD(d) ((d)*(M_PI/180.0))
#define RAD2DEG(r) ((r)*(180.0/M_PI))

#define DEBUG_SEARCH 0
#define DEBUG_HEURISTIC 0

#if DEBUG_SEARCH
  FILE* fSucc = SBPL_FOPEN("/tmp/debug_succs.txt", "w");
#endif

#if DEBUG_HEURISTIC
  FILE* fHeur = SBPL_FOPEN("/tmp/debug_heur.txt","w");
#endif

//Statistics
bool near_goal = false;
clock_t starttime;
double time_to_goal_region;

using namespace std;

static bool xyzCompare (vector<int> i, vector<int> j) 
{
  if(i[0] < j[0])
    return true;
  else if(i[1] < j[1])
    return true;
  else if(i[2] < j[2])
    return true;
  else
    return false;
};

namespace sbpl_arm_planner{

  EnvironmentROBARM3D::EnvironmentROBARM3D() : grid_(NULL),dijkstra_(NULL),elbow_dijkstra_(NULL),arm_(NULL),rpysolver_(NULL),cspace_(NULL)
  {
    EnvROBARM.Coord2StateIDHashTable = NULL;
    EnvROBARMCfg.bInitialized = false;
    EnvROBARMCfg.ik_solution_is_valid = false;
    EnvROBARMCfg.solved_by_ik = 0;
    EnvROBARMCfg.solved_by_os = 0;
    EnvROBARMCfg.num_calls_to_ik = 0;
    EnvROBARMCfg.num_ik_invalid_path = 0; 
    EnvROBARMCfg.num_invalid_ik_solutions = 0;
    EnvROBARMCfg.num_no_ik_solutions = 0;
    EnvROBARMCfg.num_ik_invalid_joint_limits = 0;
    EnvROBARMCfg.ik_solution.resize(7,0);
    save_expanded_states = true;
    using_short_mprims_ = false;

    getHeuristic_ = &sbpl_arm_planner::EnvironmentROBARM3D::getEndEffectorHeuristic;

    params_filename_ = "./config/params.cfg";
    arm_desc_filename_ = "./config/pr2_right_arm.cfg";
   
    elbow_heuristic_completed_ = false;
  }

  EnvironmentROBARM3D::~EnvironmentROBARM3D()
  {
    if(rpysolver_ != NULL)
      delete rpysolver_;
    if(cspace_ != NULL)
      delete cspace_;
    if(arm_ != NULL)
      delete arm_;
    if(dijkstra_ != NULL)
      delete dijkstra_;
    if(elbow_dijkstra_ != NULL)
      delete elbow_dijkstra_;
    if(grid_ != NULL)
      delete grid_;

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

bool EnvironmentROBARM3D::InitializeMDPCfg(MDPConfig *MDPCfg)
{
  //initialize MDPCfg with the start and goal ids	
  MDPCfg->goalstateid = EnvROBARM.goalHashEntry->stateID;
  MDPCfg->startstateid = EnvROBARM.startHashEntry->stateID;
  return true;
}

bool EnvironmentROBARM3D::InitializeEnv(const char* sEnvFile, std::string params_file, std::string arm_file)
{
  params_filename_ = params_file;
  arm_desc_filename_ = arm_file;

  return InitializeEnv(sEnvFile);
}

bool EnvironmentROBARM3D::InitializeEnv(const char* sEnvFile)
{
  //parse the parameter file
  FILE* fCfg = fopen(params_filename_.c_str(), "r");
  if(fCfg == NULL)
  {
    SBPL_ERROR("Unable to open params file: %s. Exiting.", params_filename_.c_str());
    return false;
  }

  SBPL_DEBUG("[InitializeEnv] Using param file: %s", params_filename_.c_str());
  prms_.initFromParamFile(fCfg);
  SBPL_FCLOSE(fCfg);

  FILE* aCfg = fopen(arm_desc_filename_.c_str(), "r");
  if(aCfg == NULL)
  {
    SBPL_ERROR("Unable to open arm_desc file: %s. Exiting.", arm_desc_filename_.c_str());
    return false;
  }

  SBPL_DEBUG("Using arm description file: %s", arm_desc_filename_.c_str());

  std::string robot_description("ROS_PARAM");
  SBPL_DEBUG("[InitializeEnv] Getting URDF from ROS Param server.");
  initArmModel(aCfg, robot_description); 
  SBPL_FCLOSE(aCfg);

  //parse the environment file
  fCfg = fopen(sEnvFile, "r");
  if(fCfg == NULL)
  {
    SBPL_ERROR("Unable to open environment file: %s. Exiting.", sEnvFile);
    return false;
  }

  readConfiguration(fCfg);
  SBPL_FCLOSE(fCfg);

  SBPL_DEBUG("Parsed arm description, parameter file, environment file.");

  if(!initGeneral())
  {
    SBPL_ERROR("InitGeneral() failed.");
    return false;
  }

  SBPL_DEBUG("Ready to start planning. Setting start configuration.");

  //add self collision cuboids to grid
  cspace_->addArmCuboidsToGrid();

  //add obstacle cuboids in environment to grid
  for(int i = 0; i < (int)EnvROBARMCfg.obstacles.size(); i++)
  {
    if(EnvROBARMCfg.obstacles[i].size() == 6)
      grid_->addCollisionCuboid(EnvROBARMCfg.obstacles[i][0],EnvROBARMCfg.obstacles[i][1],EnvROBARMCfg.obstacles[i][2],EnvROBARMCfg.obstacles[i][3],EnvROBARMCfg.obstacles[i][4],EnvROBARMCfg.obstacles[i][5]);
    else
      SBPL_DEBUG("[InitializeEnv] Obstacle #%d has an incomplete obstacle description. Not adding obstacle it to grid.", i);
  }

  //initialize the angles of the start states
  if(!setStartConfiguration(EnvROBARMCfg.start_configuration))
  {
    SBPL_ERROR("Failed to set initial state of robot. Exiting.");
    return false;
  }

  //set Environment is Initialized flag(so fk can be used)
  EnvROBARMCfg.bInitialized = true;

  //set goals
  if(!setGoalPosition(EnvROBARMCfg.ParsedGoals,EnvROBARMCfg.ParsedGoalTolerance))
  {
    SBPL_ERROR("Failed to set goal pose. Exiting.");
    return false;
  }

  grid_->visualize();

  //for statistics purposes
  starttime = clock();		

  SBPL_INFO("Successfully initialized environment.");
  return true;
}

int EnvironmentROBARM3D::GetFromToHeuristic(int FromStateID, int ToStateID)
{
  return (*this.*getHeuristic_)(FromStateID,ToStateID);
}

int EnvironmentROBARM3D::GetGoalHeuristic(int stateID)
{
#if DEBUG_HEUR
  if(stateID >= (int)EnvROBARM.StateID2CoordTable.size())
  {
    SBPL_ERROR("ERROR in EnvROBARM... function: stateID illegal");
    return -1;
  }
#endif

  return GetFromToHeuristic(stateID, EnvROBARM.goalHashEntry->stateID);
}

int EnvironmentROBARM3D::GetStartHeuristic(int stateID)
{
#if DEBUG_HEUR
  if(stateID >= (int)EnvROBARM.StateID2CoordTable.size())
  {
    SBPL_ERROR("ERROR in EnvROBARM... function: stateID illegal\n");
    return -1;
  }
#endif

  return GetFromToHeuristic(stateID, EnvROBARM.startHashEntry->stateID);
}

int EnvironmentROBARM3D::SizeofCreatedEnv()
{
  return EnvROBARM.StateID2CoordTable.size();
}

void EnvironmentROBARM3D::PrintState(int stateID, bool bVerbose, FILE* fOut /*=NULL*/)
{
#if DEBUG_HEUR
  if(stateID >= (int)EnvROBARM.StateID2CoordTable.size())
  {
    SBPL_ERROR("ERROR in EnvROBARM... function: stateID illegal (2)\n");
    throw new SBPL_Exception();
  }
#endif

  if(fOut == NULL)
    fOut = stdout;

  EnvROBARM3DHashEntry_t* HashEntry = EnvROBARM.StateID2CoordTable[stateID];

  bool bGoal = false;
  if(stateID == EnvROBARM.goalHashEntry->stateID)
    bGoal = true;

  if(stateID == EnvROBARM.goalHashEntry->stateID && bVerbose)
  {
    //SBPL_DEBUG_NAMED(fOut, "the state is a goal state\n");
    bGoal = true;
  }

  printJointArray(fOut, HashEntry, bGoal, bVerbose);
}

void EnvironmentROBARM3D::PrintEnv_Config(FILE* fOut)
{
  //implement this if the planner needs to print out EnvROBARM. configuration

  SBPL_ERROR("ERROR in EnvROBARM... function: PrintEnv_Config is undefined\n");
  throw new SBPL_Exception();
}

void EnvironmentROBARM3D::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV)
{
  int i, a, final_mp_cost = 0;
  unsigned char dist=0;
  int endeff[3]={0};
  std::vector<short unsigned int> succcoord(arm_->num_joints_,0);
  std::vector<double> pose(6,0), angles(arm_->num_joints_,0), source_angles(arm_->num_joints_,0);

  //to support two sets of succesor actions
  int actions_i_min = 0, actions_i_max = prms_.num_long_dist_mprims_;

  //clear the successor array
  SuccIDV->clear();
  CostV->clear();

  //goal state should be absorbing
  if(SourceStateID == EnvROBARM.goalHashEntry->stateID)
    return;

  //get X, Y, Z for the state
  EnvROBARM3DHashEntry_t* HashEntry = EnvROBARM.StateID2CoordTable[SourceStateID];

  //default coords of successor
  for(i = 0; i < arm_->num_joints_; i++)
    succcoord[i] = HashEntry->coord[i];

  //used for interpolated collision check
  coordToAngles(succcoord, source_angles);

  //check if cell is close to enough to goal to use higher resolution actions
  int dist_to_goal = getDijkstraDistToGoal(HashEntry->xyz[0],HashEntry->xyz[1],HashEntry->xyz[2]);

  if(prms_.use_multires_mprims_)
  {
    if(dist_to_goal <= prms_.short_dist_mprims_thresh_c_)
    {
      if(!using_short_mprims_)
      {
        SBPL_DEBUG("[GetSuccs] Switching to short distance motion prims after %d expansions. (SourceState with ID, #%d, is %0.3fm from goal)", int(expanded_states.size()), SourceStateID, double(dist_to_goal)/double(prms_.cost_per_cell_)*prms_.resolution_);
        using_short_mprims_ = true;
      }
      actions_i_min = prms_.num_long_dist_mprims_;
      actions_i_max = prms_.num_mprims_;
    }
  }

#if DEBUG_SEARCH
if(prms_.verbose_)
  SBPL_DEBUG_NAMED("search", "\nstate %d: %.2f %.2f %.2f %.2f %.2f %.2f %.2f  endeff: %3d %3d %3d",SourceStateID, source_angles[0],source_angles[1],source_angles[2],source_angles[3],source_angles[4],source_angles[5],source_angles[6], HashEntry->xyz[0],HashEntry->xyz[1],HashEntry->xyz[2]);
#endif

  //iterate through successors of state s (possible actions)
  for (i = actions_i_min; i < actions_i_max; i++)
  {
    //add the motion primitive to the current joint configuration
    for(a = 0; a < arm_->num_joints_; ++a)
    {
      if((HashEntry->coord[a] + int(prms_.mprims_[i][a])) < 0)
        succcoord[a] = ((EnvROBARMCfg.anglevals[a] + HashEntry->coord[a] + int(prms_.mprims_[i][a])) % EnvROBARMCfg.anglevals[a]);
      else
        succcoord[a] = ((int)(HashEntry->coord[a] + int(prms_.mprims_[i][a])) % EnvROBARMCfg.anglevals[a]);
    }

    //get the successor
    EnvROBARM3DHashEntry_t* OutHashEntry;
    bool bSuccisGoal = false;
    final_mp_cost = 0;

    //get continous angles
    coordToAngles(succcoord, angles);

    //check joint limits
    if(!arm_->checkJointLimits(angles, prms_.verbose_))
      continue;

    //check for collisions
    if(!cspace_->checkCollision(angles, prms_.verbose_, false, dist))
    {
#if DEBUG_SEARCH
  if(prms_.verbose_)
      SBPL_DEBUG_NAMED("search", " succ: %2d  dist: %2d is in collision.", i, int(dist));
#endif
      continue;
    }

    // check for collision along interpolated path between sourcestate and succ
    if(!cspace_->checkPathForCollision(source_angles, angles, prms_.verbose_, dist))
    {
#if DEBUG_SEARCH
      if(prms_.verbose_)
        SBPL_DEBUG_NAMED("search", " succ: %2d  dist: %2d is in collision along interpolated path.", i, int(dist));
#endif
      continue;
    }

    //get end effector position 
    if(!arm_->getPlanningJointPose(angles, pose))
      continue;

    //get grid coordinates of endeff
    grid_->worldToGrid(pose[0],pose[1],pose[2],endeff[0],endeff[1],endeff[2]);

    short unsigned int endeff_short[3]={endeff[0],endeff[1],endeff[2]};

    //check if this state meets the goal criteria
    if(isGoalPosition(pose,EnvROBARMCfg.EndEffGoals[0], angles, final_mp_cost))
    {
      bSuccisGoal = true;

      for (int j = 0; j < arm_->num_joints_; j++)
        EnvROBARM.goalHashEntry->coord[j] = succcoord[j];

      EnvROBARM.goalHashEntry->xyz[0] = endeff[0];
      EnvROBARM.goalHashEntry->xyz[1] = endeff[1];
      EnvROBARM.goalHashEntry->xyz[2] = endeff[2];
      EnvROBARM.goalHashEntry->action = i;
      EnvROBARM.goalHashEntry->dist = dist;

      SBPL_DEBUG("[GetSuccs] Goal successor is generated. SourceStateID: %d (distance to nearest obstacle: %0.2fm)",SourceStateID,  (double)dist*grid_->getResolution());
    }

    //check if hash entry already exists, if not then create one
    if((OutHashEntry = getHashEntry(succcoord, i, bSuccisGoal)) == NULL)
    {
      OutHashEntry = createHashEntry(succcoord, endeff_short, i);
      OutHashEntry->rpy[0] = pose[3];
      OutHashEntry->rpy[1] = pose[4];
      OutHashEntry->rpy[2] = pose[5];
      OutHashEntry->dist = dist;

#if DEBUG_SEARCH
  if(prms_.verbose_)
      SBPL_DEBUG_NAMED("search", "%5i: action: %2d dist: %2d edge_distance_cost: %5d heur: %2d endeff: %3d %3d %3d", OutHashEntry->stateID, i, int(OutHashEntry->dist), cost(HashEntry,OutHashEntry,bSuccisGoal), GetFromToHeuristic(OutHashEntry->stateID, EnvROBARM.goalHashEntry->stateID), OutHashEntry->xyz[0],OutHashEntry->xyz[1],OutHashEntry->xyz[2]);
#endif
    }

    //put successor on successor list with the proper cost
    SuccIDV->push_back(OutHashEntry->stateID);
    CostV->push_back(cost(HashEntry,OutHashEntry,bSuccisGoal) + final_mp_cost);
  }

  if(save_expanded_states)
    expanded_states.push_back(SourceStateID);
}

void EnvironmentROBARM3D::GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV)
{
  SBPL_ERROR("ERROR in EnvROBARM... function: GetPreds is undefined\n");
  throw new SBPL_Exception();
}

bool EnvironmentROBARM3D::AreEquivalent(int StateID1, int StateID2)
{
  SBPL_ERROR("ERROR in EnvROBARM... function: AreEquivalent is undefined\n");
  throw new SBPL_Exception();
}

void EnvironmentROBARM3D::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{
  SBPL_ERROR("ERROR in EnvROBARM..function: SetAllActionsandOutcomes is undefined\n");
  throw new SBPL_Exception();
}

void EnvironmentROBARM3D::SetAllPreds(CMDPSTATE* state)
{
  SBPL_ERROR("ERROR in EnvROBARM... function: SetAllPreds is undefined\n");
  throw new SBPL_Exception();
}

/////////////////////////////////////////////////////////////////////////////
//                      End of SBPL Planner Interface
/////////////////////////////////////////////////////////////////////////////

void EnvironmentROBARM3D::printHashTableHist()
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
  SBPL_DEBUG("hash table histogram: 0:%d, <50:%d, <100:%d, <200:%d, <300:%d, <400:%d >400:%d",
      s0,s1, s50, s100, s200,s300,slarge);
}

EnvROBARM3DHashEntry_t* EnvironmentROBARM3D::getHashEntry(const std::vector<short unsigned int> &coord, short unsigned int action, bool bIsGoal)
{
  //if it is goal
  if(bIsGoal)
    return EnvROBARM.goalHashEntry;

  int binid = getHashBin(coord);

#if DEBUG
  if ((int)EnvROBARM.Coord2StateIDHashTable[binid].size() > 500)
  {
    SBPL_WARN("WARNING: Hash table has a bin %d (coord0=%d) of size %d", 
        binid, coord[0], int(EnvROBARM.Coord2StateIDHashTable[binid].size()));
    printHashTableHist();
  }
#endif

  //iterate over the states in the bin and select the perfect match
  for(int ind = 0; ind < (int)EnvROBARM.Coord2StateIDHashTable[binid].size(); ind++)
  {
    int j = 0;

    for(j = 0; j < int(coord.size()); j++)
    {
      if(EnvROBARM.Coord2StateIDHashTable[binid][ind]->coord[j] != coord[j]) 
        break;
    }

    if (j == int(coord.size()))
      return EnvROBARM.Coord2StateIDHashTable[binid][ind];
  }

  return NULL;
}

EnvROBARM3DHashEntry_t* EnvironmentROBARM3D::createHashEntry(const std::vector<short unsigned int> &coord, short unsigned int endeff[3], short unsigned int action)
{
  int i;
  EnvROBARM3DHashEntry_t* HashEntry = new EnvROBARM3DHashEntry_t;

  HashEntry->coord = coord;

  memcpy(HashEntry->xyz, endeff, 3*sizeof(short unsigned int));

  HashEntry->action = action;

  // assign a stateID to HashEntry to be used 
  HashEntry->stateID = EnvROBARM.StateID2CoordTable.size();

  //insert into the tables
  EnvROBARM.StateID2CoordTable.push_back(HashEntry);

  //get the hash table bin
  i = getHashBin(HashEntry->coord);

  //insert the entry into the bin
  EnvROBARM.Coord2StateIDHashTable[i].push_back(HashEntry);

  //insert into and initialize the mappings
  int* entry = new int [NUMOFINDICES_STATEID2IND];
  StateID2IndexMapping.push_back(entry);
  for(i = 0; i < NUMOFINDICES_STATEID2IND; i++)
  {
    StateID2IndexMapping[HashEntry->stateID][i] = -1;
  }

  if(HashEntry->stateID != (int)StateID2IndexMapping.size()-1)
  {
    SBPL_ERROR("ERROR in Env... function: last state has incorrect stateID");
    throw new SBPL_Exception();
  }

  return HashEntry;
}

void EnvironmentROBARM3D::initDijkstra()
{
  int dimX,dimY,dimZ;
  grid_->getGridSize(dimX, dimY, dimZ);

  dijkstra_ = new BFS3D(dimX, dimY, dimZ, int(arm_->getLinkRadiusCells(2)), prms_.cost_per_cell_);

  dijkstra_->configDistanceField(true, grid_->getDistanceFieldPtr());

#if DEBUG_SEARCH
  if(prms_.verbose_)
    dijkstra_->printConfig(fSucc);
#endif 
  SBPL_DEBUG("[initDijkstra] BFS is initialized.");
}

void EnvironmentROBARM3D::initElbowDijkstra()
{
  int dimX,dimY,dimZ;
  grid_->getGridSize(dimX, dimY, dimZ);

  elbow_dijkstra_ = new BFS3D(dimX, dimY, dimZ, int(arm_->getLinkRadiusCells(2)), prms_.cost_per_cell_);

  elbow_dijkstra_->configDistanceField(true, grid_->getDistanceFieldPtr());

#if DEBUG_SEARCH
  if(prms_.verbose_)
    elbow_dijkstra_->printConfig(fSucc);
#endif 

  SBPL_DEBUG("[initElbowDijkstra] BFS is initialized.");
}

void EnvironmentROBARM3D::discretizeAngles()
{
  for(int i = 0; i < arm_->num_joints_; i++)
  {
    EnvROBARMCfg.angledelta[i] = (2.0*M_PI) / prms_.angle_delta_;
    EnvROBARMCfg.anglevals[i] = prms_.angle_delta_;
  }
}

int EnvironmentROBARM3D::cost(EnvROBARM3DHashEntry_t* HashEntry1, EnvROBARM3DHashEntry_t* HashEntry2, bool bState2IsGoal)
{
  if(prms_.use_uniform_cost_)
    return prms_.cost_multiplier_;
  else
  {
    // Max's suggestion is to just put a high cost on being close to
    // obstacles but don't provide some sort of gradient 
    if(int(HashEntry2->dist) < 7) // in cells
    {
      //printf("%d->%d: dist: %d cost: %d heur: %d\n",int(HashEntry1->stateID), int(HashEntry2->stateID), int(HashEntry2->dist), prms_.cost_multiplier_ * prms_.range1_cost_, GetFromToHeuristic(HashEntry1->stateID, HashEntry2->stateID));
      return prms_.cost_multiplier_ * prms_.range1_cost_;
    }
    else if(int(HashEntry2->dist) < 12)
      return prms_.cost_multiplier_ * prms_.range2_cost_;
    else if(int(HashEntry2->dist) < 17)
      return prms_.cost_multiplier_ * prms_.range3_cost_;
    else
      return prms_.cost_multiplier_;
  }
}

bool EnvironmentROBARM3D::initEnvConfig()
{
  short unsigned int endeff[3] = {0};
  std::vector<short unsigned int> coord(arm_->num_joints_,0);

  //these probably don't have to be done
  EnvROBARMCfg.start_configuration.resize(arm_->num_joints_);
  EnvROBARMCfg.ik_solution.resize(arm_->num_joints_,0);

  EnvROBARMCfg.angledelta.resize(arm_->num_joints_);
  EnvROBARMCfg.anglevals.resize(arm_->num_joints_);

  discretizeAngles();

  //initialize the map from Coord to StateID
  EnvROBARM.HashTableSize = 32*1024; //should be power of two
  EnvROBARM.Coord2StateIDHashTable = new std::vector<EnvROBARM3DHashEntry_t*>[EnvROBARM.HashTableSize];

  //initialize the map from StateID to Coord
  EnvROBARM.StateID2CoordTable.clear();

  //create an empty start state
  EnvROBARM.startHashEntry = createHashEntry(coord, endeff, 0);

  //create the goal state
  EnvROBARM.goalHashEntry = createHashEntry(coord, endeff, 0);

  return true;
}

bool EnvironmentROBARM3D::initArmModel(FILE* aCfg, const std::string robot_description)
{
  arm_ = new SBPLArmModel(aCfg);

  arm_->setResolution(prms_.resolution_);

  if(robot_description.compare("ROS_PARAM") == 0)
  {
    if(arm_->initKDLChainFromParamServer())
      return true;
  }
  else
  {
    if(arm_->initKDLChain(robot_description))
      return true;
  }

  return false; 
}

bool EnvironmentROBARM3D::initEnvironment(std::string arm_description_filename, std::string mprims_filename)
{
  FILE* mprims_fp=NULL;
  FILE* arm_fp=NULL;

  //initialize the arm planner parameters
  prms_.initFromParamServer();

  //parse motion primitives file
  if((mprims_fp=fopen(mprims_filename.c_str(),"r")) == NULL)
  {
    SBPL_ERROR("Failed to open motion primitive file.");
    return false;
  }
  if(!prms_.initMotionPrimsFromFile(mprims_fp))
  {
    SBPL_ERROR("Failed to parse motion primitive file.");
    fclose(mprims_fp);
    return false;
  }
  fclose(mprims_fp);

  //initialize the arm model
  if((arm_fp=fopen(arm_description_filename.c_str(),"r")) == NULL)
  {
    SBPL_ERROR("Failed to open arm description file.");
    return false;
  }
  std::string ros_param("ROS_PARAM");
  if(!initArmModel(arm_fp,ros_param))
  {
    SBPL_ERROR("Failed to initialize arm model.");
    fclose(arm_fp);
    return false;
  }
  fclose(arm_fp);

  //initialize the environment & planning variables  
  if(!initGeneral())
  {
    SBPL_ERROR("Failed to initialize environment.");
    return false;
  }

  //set 'Environment is Initialized' flag
  EnvROBARMCfg.bInitialized = true;

  prms_.printMotionPrims(std::string("sbpl_arm"));

  SBPL_INFO("[initEnvironment] Environment has been initialized.");

  //for statistics purposes
  starttime = clock();

  return true;
}

bool EnvironmentROBARM3D::initGeneral()
{
  //create the occupancy grid
  grid_ = new OccupancyGrid(prms_.sizeX_,prms_.sizeY_,prms_.sizeZ_, prms_.resolution_,prms_.originX_,prms_.originY_,prms_.originZ_);

  grid_->setReferenceFrame(prms_.reference_frame_);

  //create the collision space
  cspace_ = new SBPLCollisionSpace(arm_, grid_);

  //create the rpysolver
  rpysolver_ = new RPYSolver(arm_, cspace_);

#if DEBUG_SEARCH
  arm_->setDebugFile(std::string("sbpl"));
  cspace_->setDebugFile(fSucc);
#endif

  //initialize Environment
  if(initEnvConfig() == false)
    return false;

  //compute the cost per cell to be used by heuristic
  computeCostPerCell();

  //initialize dijkstra 
  initDijkstra();
  
  if(prms_.use_research_heuristic_)
    initElbowDijkstra();

  if(prms_.verbose_)
  {
    arm_->printArmDescription(std::string("sbpl_arm"));
    prms_.printParams(std::string("sbpl_arm"));
    prms_.printMotionPrims(std::string("sbpl_arm"));
  }

  if(prms_.verbose_)
  {
    arm_->printArmDescription(std::string("sbpl_arm"));
    prms_.printParams(std::string("sbpl_arm"));
  }

  //set heuristic function pointer
  if(prms_.use_research_heuristic_)
    getHeuristic_ = &sbpl_arm_planner::EnvironmentROBARM3D::getCombinedHeuristic;
  else
    getHeuristic_ = &sbpl_arm_planner::EnvironmentROBARM3D::getEndEffectorHeuristic;

  prefinal_joint_config.resize(7);
  final_joint_config.resize(7);
 
  return true;
}

double EnvironmentROBARM3D::getEpsilon()
{
  return prms_.epsilon_;
}

void EnvironmentROBARM3D::readConfiguration(FILE* fCfg)
{
  char sTemp[1024];
  int i;
  std::vector<double> cube(6,0);

  if(fscanf(fCfg,"%s",sTemp) < 1) 
    SBPL_DEBUG("Parsed string has length < 1.\n");
  while(!feof(fCfg) && strlen(sTemp) != 0)
  {
    if(strcmp(sTemp, "linkstartangles(radians):") == 0)
    {
      EnvROBARMCfg.start_configuration.resize(arm_->num_joints_,0);
      for(i = 0; i < arm_->num_joints_; i++)
      {
        if(fscanf(fCfg,"%s",sTemp) < 1) 
          SBPL_DEBUG("Parsed string has length < 1.\n");
        EnvROBARMCfg.start_configuration[i] = atof(sTemp);
      }
    }
    else if(strcmp(sTemp, "endeffectorgoal(meters):") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        SBPL_DEBUG("Parsed string has length < 1.\n");
      EnvROBARMCfg.ParsedGoals.resize(atoi(sTemp));
      EnvROBARMCfg.ParsedGoalTolerance.resize(1);
      EnvROBARMCfg.ParsedGoalTolerance[0].resize(2);
      EnvROBARMCfg.ParsedGoalTolerance[0][0] = 0.04;
      EnvROBARMCfg.ParsedGoalTolerance[0][1] = 0.25;

      for(unsigned int i = 0; i < EnvROBARMCfg.ParsedGoals.size(); i++)
      {
        EnvROBARMCfg.ParsedGoals[i].resize(7);

        for(unsigned int k = 0; k < 7; k++)
        {
          if(fscanf(fCfg,"%s",sTemp) < 1) 
            SBPL_DEBUG("Parsed string has length < 1.\n");
          EnvROBARMCfg.ParsedGoals[i][k] = atof(sTemp);
        }
      }
    }
    else if(strcmp(sTemp, "goal_tolerance(meters,radians):") == 0)
    {
      EnvROBARMCfg.ParsedGoalTolerance.resize(1);
      EnvROBARMCfg.ParsedGoalTolerance[0].resize(2);

      //distance tolerance (m)
      if(fscanf(fCfg,"%s",sTemp) < 1)
        SBPL_DEBUG("Parsed string has length < 1.\n");
      EnvROBARMCfg.ParsedGoalTolerance[0][0] = atof(sTemp);
      //orientation tolerance (rad)
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_DEBUG("Parsed string has length < 1.\n");
      EnvROBARMCfg.ParsedGoalTolerance[0][1] = atof(sTemp);
    }
    else if(strcmp(sTemp, "cube:") == 0)
    {
      for(int j = 0; j < 6; j++)
      {
        if(fscanf(fCfg,"%s",sTemp) < 1)
          SBPL_DEBUG("Parsed string has length < 1.\n");
        cube[j] = atof(sTemp);
      }
      EnvROBARMCfg.obstacles.push_back(cube);
    }
    else
      SBPL_WARN("ERROR: Unknown parameter name in environment config file: %s.\n", sTemp);

    if(fscanf(fCfg,"%s",sTemp) < 1) 
      SBPL_DEBUG("Parsed string has length < 1.\n");
  }
}

bool EnvironmentROBARM3D::isGoalPosition(const std::vector<double> &pose, const GoalPos &goal, std::vector<double> jnt_angles, int &cost)
{
    if(goal.is_6dof_goal)
    {
      //check if position constraint is satisfied  
      if(fabs(pose[0]-goal.pos[0]) <= goal.pos_tolerance[0] && 
          fabs(pose[1]-goal.pos[1]) <= goal.pos_tolerance[1] && 
          fabs(pose[2]-goal.pos[2]) <= goal.pos_tolerance[2])
      {
        //log the amount of time required for the search to get close to the goal
        if(!near_goal)
        {
          time_to_goal_region = (clock() - starttime) / (double)CLOCKS_PER_SEC;
          near_goal = true;
          printf("Search is at %0.2f %0.2f %0.2f, within %0.3fm of the goal (%0.2f %0.2f %0.2f) after %.4f sec. (after %d expansions)\n", pose[0],pose[1],pose[2],goal.pos_tolerance[0], goal.pos[0], goal.pos[1], goal.pos[2], time_to_goal_region,(int)expanded_states.size());
          EnvROBARMCfg.num_expands_to_position_constraint = expanded_states.size();
        }
        
        if(prms_.use_orientation_solver_)
        {
          //try to reach orientation constraint with orientation solver 
          if(isGoalStateWithOrientationSolver(goal,jnt_angles))
          {
            EnvROBARMCfg.solved_by_os++;

            //compute cost of the motion
            cost = getActionCost(jnt_angles,final_joint_config,0);
            return true;
          }
        } 
      }

      if(prms_.use_ik_)
      {
        //try to reach orientation constraint with IK
        if(isGoalStateWithIK(pose,goal,jnt_angles))
        {
          EnvROBARMCfg.solved_by_ik++;

          //compute cost of the motion
          cost = getActionCost(jnt_angles,final_joint_config,0);
          return true;
        }
      }
    }
    else
    {
      //check if position constraint is satisfied  
      if(fabs(pose[0]-goal.pos[0]) <= goal.pos_tolerance[0] && 
         fabs(pose[1]-goal.pos[1]) <= goal.pos_tolerance[1] && 
         fabs(pose[2]-goal.pos[2]) <= goal.pos_tolerance[2])
        return true;
    }

  return false;
}

bool EnvironmentROBARM3D::isGoalStateWithIK(const std::vector<double> &pose, const GoalPos &goal, std::vector<double> jnt_angles)
{
  //check distance to goal, if within range then run IK
  if(prms_.use_dijkstra_heuristic_)
  {
    int endeff[3];
    grid_->worldToGrid(pose[0],pose[1],pose[2],endeff[0],endeff[1],endeff[2]);
    short unsigned int endeff_short[3]={endeff[0],endeff[1],endeff[2]};

    if(dijkstra_->getDist(endeff_short[0],endeff_short[1],endeff_short[2]) > prms_.solve_for_ik_thresh_)
      return false;
  }

  EnvROBARMCfg.ik_solution=jnt_angles;

  std::vector<double> goal_pose(7,0);  //changed from 6
  unsigned char dist = 0;

  goal_pose[0] = goal.pos[0];
  goal_pose[1] = goal.pos[1];
  goal_pose[2] = goal.pos[2];
/*
  goal_pose[3] = goal.rpy[0];
  goal_pose[4] = goal.rpy[1];
  goal_pose[5] = goal.rpy[2];
*/
  goal_pose[3] = goal.q[0];
  goal_pose[4] = goal.q[1];
  goal_pose[5] = goal.q[2];
  goal_pose[6] = goal.q[3];

  EnvROBARMCfg.num_calls_to_ik++;

  //generate an IK solution
  if(!arm_->computeIK(goal_pose, jnt_angles, EnvROBARMCfg.ik_solution))
  {
    EnvROBARMCfg.num_no_ik_solutions++;
    return false;
  }

  //check joint limits
  if(!arm_->checkJointLimits(EnvROBARMCfg.ik_solution, false))
  {
    EnvROBARMCfg.num_ik_invalid_joint_limits++;
    return false;
  }

  //check for collisions
  if(!cspace_->checkCollision(EnvROBARMCfg.ik_solution, prms_.verbose_, false, dist))
  {
    EnvROBARMCfg.num_invalid_ik_solutions++;
    return false;
  }

  std::vector<std::vector<double> > path(2, std::vector<double>(arm_->num_joints_,0));
  for(unsigned int i = 0; i < path[0].size(); ++i)
  {
    path[0][i] = jnt_angles[i];
    path[1][i] = EnvROBARMCfg.ik_solution[i];
  }

  //check for collisions along the path
  if(!cspace_->checkPathForCollision(jnt_angles, EnvROBARMCfg.ik_solution, prms_.verbose_, dist))
  {
    EnvROBARMCfg.num_ik_invalid_path++;
    return false;
  }

  SBPL_DEBUG("[isGoalStateWithIK] The path to the IK solution for the goal is valid.");

  //added to be compatible with OP - 7/5/2010
  prefinal_joint_config = jnt_angles;
  final_joint_config = EnvROBARMCfg.ik_solution;

  EnvROBARMCfg.ik_solution_is_valid = true; 
  
  return true;
}

int EnvironmentROBARM3D::getActionCost(const std::vector<double> &from_config, const std::vector<double> &to_config, int dist)
{
  int num_prims = 0, cost = 0;
  double diff = 0, max_diff = 0;

  if(from_config.size() != to_config.size())
    return -1;

/* NOTE: Not including forearm roll OR wrist roll movement to calculate mprim cost */

  for(size_t i = 0; i < 6; i++)
  {
    if(i == 4)
      continue;

    diff = fabs(angles::shortest_angular_distance(from_config[i], to_config[i]));
    if(max_diff < diff)
      max_diff = diff;
  }

  num_prims = max_diff / prms_.max_mprim_offset_ + 0.5;
  cost = num_prims * prms_.cost_multiplier_;

#if DEBUG_SEARCH
  SBPL_DEBUG_NAMED("search", "from: %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f", angles::normalize_angle(from_config[0]),angles::normalize_angle(from_config[1]),angles::normalize_angle(from_config[2]),angles::normalize_angle(from_config[3]),angles::normalize_angle(from_config[4]),angles::normalize_angle(from_config[5]),angles::normalize_angle(from_config[6]));
  SBPL_DEBUG_NAMED("search", "  to: %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f diff: %0.2f num_prims: %d cost: %d (mprim_size: %0.3f)", to_config[0],to_config[1],to_config[2],to_config[3],to_config[4],to_config[5],to_config[6], max_diff, num_prims, cost, prms_.max_mprim_offset_);
#endif

  return cost;
}

int EnvironmentROBARM3D::getEdgeCost(int FromStateID, int ToStateID)
{
#if DEBUG
  if(FromStateID >= (int)EnvROBARM.StateID2CoordTable.size() 
      || ToStateID >= (int)EnvROBARM.StateID2CoordTable.size())
  {
    SBPL_ERROR("ERROR in EnvROBARM... function: stateID illegal\n");
    throw new SBPL_Exception();
  }
#endif

  //get X, Y for the state
  EnvROBARM3DHashEntry_t* FromHashEntry = EnvROBARM.StateID2CoordTable[FromStateID];
  EnvROBARM3DHashEntry_t* ToHashEntry = EnvROBARM.StateID2CoordTable[ToStateID];

  return cost(FromHashEntry, ToHashEntry, false);
}

bool EnvironmentROBARM3D::isGoalStateWithOrientationSolver(const GoalPos &goal, std::vector<double> jnt_angles)
{
  return rpysolver_->isOrientationFeasible(goal.rpy, jnt_angles, prefinal_joint_config, final_joint_config);
}

bool EnvironmentROBARM3D::setStartConfiguration(const std::vector<double> angles)
{
  unsigned char dist = 255;
  int x,y,z;
  std::vector<double> pose(6,0);

  if(int(angles.size()) < arm_->num_joints_)
    return false;

  //get joint positions of starting configuration
  if(!arm_->getPlanningJointPose(angles, pose))
    SBPL_WARN("Unable to compute forward kinematics for initial robot state. Attempting to plan anyway.");

  //check joint limits of starting configuration but plan anyway
  if(!arm_->checkJointLimits(angles, true))
    SBPL_WARN("Starting configuration violates the joint limits. Attempting to plan anyway.");

  //check if the start configuration is in collision but plan anyway
  if(!cspace_->checkCollision(angles, true, false, dist))
    SBPL_WARN("The starting configuration is in collision. Attempting to plan anyway. (distance to nearest obstacle %0.2fm)", double(dist)*grid_->getResolution());

  //get arm position in environment
  anglesToCoord(angles, EnvROBARM.startHashEntry->coord);

  grid_->worldToGrid(pose[0],pose[1],pose[2],x,y,z);
  EnvROBARM.startHashEntry->xyz[0] = (short unsigned int)x;
  EnvROBARM.startHashEntry->xyz[1] = (short unsigned int)y;
  EnvROBARM.startHashEntry->xyz[2] = (short unsigned int)z;

  EnvROBARM.startHashEntry->rpy[0] = pose[3];
  EnvROBARM.startHashEntry->rpy[1] = pose[4];
  EnvROBARM.startHashEntry->rpy[2] = pose[5];

  return true;
}

bool EnvironmentROBARM3D::setGoalPosition(const std::vector <std::vector<double> > &goals, const std::vector<std::vector<double> > &tolerances)
{
  //goals: {{x1,y1,z1,r1,p1,y1,is_6dof},{x2,y2,z2,r2,p2,y2,is_6dof}...}

  if(!EnvROBARMCfg.bInitialized)
  {
    SBPL_ERROR("Cannot set goal position because environment is not initialized.");
    return false;
  }

  if(goals.empty())
  {
    SBPL_ERROR("[setGoalPosition] No goal constraint set.");
    return false;
  }

  // Check if an IK solution exists for the goal pose before we do the search
  // we plan even if there is no solution
  std::vector<double> pose(7,0);
  std::vector<double> jnt_angles(7,0);
  pose = goals[0];
  unsigned char dist_ik;
  if(!arm_->computeIK(pose, jnt_angles, EnvROBARMCfg.ik_solution))
    SBPL_DEBUG("[setGoalPosition] No valid IK solution for the goal pose.");
  else
  {
    if(!cspace_->checkCollision(EnvROBARMCfg.ik_solution, false, false, dist_ik))
      SBPL_DEBUG("[setGoalPosition] An IK solution for the goal pose was found but it's in collision. Valid solutions may exist.");
    else
      SBPL_DEBUG("[setGoalPosition] A valid IK solution for the goal pose was found.");
  }

  EnvROBARMCfg.EndEffGoals.resize(goals.size());
  for(unsigned int i = 0; i < EnvROBARMCfg.EndEffGoals.size(); i++)
  {
    EnvROBARMCfg.EndEffGoals[i].pos[0] = goals[i][0];
    EnvROBARMCfg.EndEffGoals[i].pos[1] = goals[i][1];
    EnvROBARMCfg.EndEffGoals[i].pos[2] = goals[i][2];

    EnvROBARMCfg.EndEffGoals[i].rpy[0] = goals[i][3];
    EnvROBARMCfg.EndEffGoals[i].rpy[1] = goals[i][4];
    EnvROBARMCfg.EndEffGoals[i].rpy[2] = goals[i][5];

    EnvROBARMCfg.EndEffGoals[i].xyz_tolerance = tolerances[i][0] / grid_->getResolution();
    
    EnvROBARMCfg.EndEffGoals[i].pos_tolerance[0] = tolerances[i][0];
    EnvROBARMCfg.EndEffGoals[i].pos_tolerance[1] = tolerances[i][1];
    EnvROBARMCfg.EndEffGoals[i].pos_tolerance[2] = tolerances[i][2];
    
    EnvROBARMCfg.EndEffGoals[i].rpy_tolerance[0] = tolerances[i][3];
    EnvROBARMCfg.EndEffGoals[i].rpy_tolerance[1] = tolerances[i][4];
    EnvROBARMCfg.EndEffGoals[i].rpy_tolerance[2] = tolerances[i][5];

    EnvROBARMCfg.EndEffGoals[i].is_6dof_goal = goals[i][6];
    prms_.use_6d_pose_goal_ = goals[i][6];

    EnvROBARMCfg.EndEffGoals[i].q[0] = goals[i][7];
    EnvROBARMCfg.EndEffGoals[i].q[1] = goals[i][8];
    EnvROBARMCfg.EndEffGoals[i].q[2] = goals[i][9];
    EnvROBARMCfg.EndEffGoals[i].q[3] = goals[i][10];
    
    if(!prms_.use_6d_pose_goal_)
      SBPL_DEBUG("[setGoalPosition] Goal position constraint set. No goal orientation constraint requested.\n");

    //convert goal position from meters to cells
    grid_->worldToGrid(goals[i][0], goals[i][1], goals[i][2], EnvROBARMCfg.EndEffGoals[i].xyz[0], EnvROBARMCfg.EndEffGoals[i].xyz[1], EnvROBARMCfg.EndEffGoals[i].xyz[2]);

    //TODO: Check if goal is on an invalid cell
  }

  // set goal hash entry
  EnvROBARM.goalHashEntry->xyz[0] = EnvROBARMCfg.EndEffGoals[0].xyz[0];
  EnvROBARM.goalHashEntry->xyz[1] = EnvROBARMCfg.EndEffGoals[0].xyz[1];
  EnvROBARM.goalHashEntry->xyz[2] = EnvROBARMCfg.EndEffGoals[0].xyz[2]; 
  EnvROBARM.goalHashEntry->rpy[0] = EnvROBARMCfg.EndEffGoals[0].rpy[0];
  EnvROBARM.goalHashEntry->rpy[1] = EnvROBARMCfg.EndEffGoals[0].rpy[1];
  EnvROBARM.goalHashEntry->rpy[2] = EnvROBARMCfg.EndEffGoals[0].rpy[2];
  EnvROBARM.goalHashEntry->action = 0;

  for(int i = 0; i < arm_->num_joints_; i++)
    EnvROBARM.goalHashEntry->coord[i] = 0;

  for(unsigned int i = 0; i < EnvROBARMCfg.EndEffGoals.size(); i++)
  {
    SBPL_INFO("goal %i:  grid: %u %u %u (cells)  xyz: %0.2f %0.2f %0.2f (meters)  (tol: %0.3fm %0.3fm %0.3fm) rpy: %1.2f %1.2f %1.2f (radians) (tol: %0.3f %0.3f %0.3f)",i,EnvROBARMCfg.EndEffGoals[i].xyz[0], EnvROBARMCfg.EndEffGoals[i].xyz[1],EnvROBARMCfg.EndEffGoals[i].xyz[2],EnvROBARMCfg.EndEffGoals[i].pos[0],EnvROBARMCfg.EndEffGoals[i].pos[1],EnvROBARMCfg.EndEffGoals[i].pos[2],tolerances[i][0],tolerances[i][1],tolerances[i][2],EnvROBARMCfg.EndEffGoals[i].rpy[0],EnvROBARMCfg.EndEffGoals[i].rpy[1],EnvROBARMCfg.EndEffGoals[i].rpy[2],tolerances[i][3],tolerances[i][4],tolerances[i][5]);
    SBPL_DEBUG("quat: %0.3f %0.3f %0.3f %0.3f",EnvROBARMCfg.EndEffGoals[i].q[0], EnvROBARMCfg.EndEffGoals[i].q[1], EnvROBARMCfg.EndEffGoals[i].q[2], EnvROBARMCfg.EndEffGoals[i].q[3]);
  }

#if DEBUG_SEARCH
  if(prms_.verbose_)
  {
    SBPL_DEBUG_NAMED("search", "\n-----------------------------------------------------------------------------------");
    SBPL_DEBUG_NAMED("search", "time: %f", clock() / (double)CLOCKS_PER_SEC);
    SBPL_DEBUG_NAMED("search", "A new goal has been set.");
    SBPL_DEBUG_NAMED("search", "grid: %u %u %u (cells)  xyz: %.2f %.2f %.2f (meters)  (tol: %.3f) rpy: %1.2f %1.2f %1.2f (radians) (tol: %.3f)",
        EnvROBARMCfg.EndEffGoals[0].xyz[0], EnvROBARMCfg.EndEffGoals[0].xyz[1],EnvROBARMCfg.EndEffGoals[0].xyz[2],
        EnvROBARMCfg.EndEffGoals[0].pos[0],EnvROBARMCfg.EndEffGoals[0].pos[1],EnvROBARMCfg.EndEffGoals[0].pos[2],tolerances[0][0],
        EnvROBARMCfg.EndEffGoals[0].rpy[0],EnvROBARMCfg.EndEffGoals[0].rpy[1],EnvROBARMCfg.EndEffGoals[0].rpy[2],tolerances[0][1]);

    SBPL_DEBUG_NAMED("search", "-----------------------------------------------------------------------------------\n");
  }
#endif

  // precompute heuristics
  if(!precomputeHeuristics())
  {
    SBPL_ERROR("Precomputing heuristics failed. Exiting.");
    return false;
  }

  clearStats();

  return true;
}

bool EnvironmentROBARM3D::precomputeHeuristics()
{
  std::vector<short unsigned int> dij_goal(3,0);

  dij_goal[0] = EnvROBARMCfg.EndEffGoals[0].xyz[0];
  dij_goal[1] = EnvROBARMCfg.EndEffGoals[0].xyz[1];
  dij_goal[2] = EnvROBARMCfg.EndEffGoals[0].xyz[2];

  SBPL_DEBUG("[precomputeHeuristics] Dijkstra Goal: %d %d %d",dij_goal[0],dij_goal[1],dij_goal[2]); 

  //set the goal for h_endeff
  if(!dijkstra_->setGoal(dij_goal))
  {
    SBPL_ERROR("[precomputeHeuristics] Failed to set goal for Dijkstra search.");
    return false;
  }

  //precompute h_elbow
  if(prms_.use_research_heuristic_)
  {
    SBPL_DEBUG("[precomputeHeuristics] Spawning a new thread to compute elbow heuristic.");
    heuristic_thread_ = new boost::thread(boost::bind(&EnvironmentROBARM3D::precomputeElbowHeuristic, this));
  }

  //precompute h_endeff
  if(!dijkstra_->runBFS())
  {
    SBPL_ERROR("Executing the BFS for the end-effector heuristic failed. Exiting.");
    return false;
  }

  //kill the elbow heuristic thread 
  if(prms_.use_research_heuristic_)
  {
    SBPL_DEBUG("[precomputeHeuristics] Attempting to get the heuristic mutex.");
    heuristic_mutex_.lock();
    SBPL_DEBUG("[precomputeHeuristics] Got the heuristic mutex. Now killing the thread.");    
    if(!elbow_heuristic_completed_)
    {
      SBPL_ERROR("[precomputeHeuristics] elbow heuristic failed.");
      if(heuristic_thread_ != NULL)
      {
        heuristic_thread_->join();
        delete heuristic_thread_;
      }
      heuristic_mutex_.unlock();
      return false;
    }

    if(heuristic_thread_ != NULL)
    {
      heuristic_thread_->join();
      delete heuristic_thread_;
    }

    heuristic_mutex_.unlock();
  }

  SBPL_DEBUG("Completed heuristic pre-computation");
  return true;
}

bool EnvironmentROBARM3D::precomputeElbowHeuristic()
{
  heuristic_mutex_.lock();
  elbow_heuristic_completed_ = false;

  std::vector<std::vector<int> > elbow_cells;

  //added in support for gokul's new function (8/5/2010) -- needs to be cleaned up!
  std::vector<double> angles(7,0); 
  std::vector<double> xyzrpy(6,0), goal(3,0);
  std::vector<int> shoulder(3,0);
  if(!arm_->computeFK(angles, 1, xyzrpy))
  {
    elbow_heuristic_completed_ = false;
    heuristic_mutex_.unlock();
    return false;
  }

  goal[0] = EnvROBARMCfg.EndEffGoals[0].pos[0];
  goal[1] = EnvROBARMCfg.EndEffGoals[0].pos[1];
  goal[2] = EnvROBARMCfg.EndEffGoals[0].pos[2];

  grid_->worldToGrid(xyzrpy[0],xyzrpy[1],xyzrpy[2],shoulder[0],shoulder[1],shoulder[2]);
  if(!getElbowCellsAtGoal(shoulder, goal, arm_->getLinkRadius(0), arm_->getLinkRadius(1), elbow_cells))
  {
    SBPL_WARN("[precomputeElbowHeuristic] No elbow cells returned. Exiting.\n");
    elbow_heuristic_completed_ = false;
    heuristic_mutex_.unlock();
    return false;
  }

  std::vector<std::vector<short unsigned int> > elbow_shorts(elbow_cells.size(), std::vector<short unsigned int> (3,0));
  for(unsigned int q = 0; q < elbow_cells.size(); q++)
  {
    elbow_shorts[q][0] = elbow_cells[q][0];
    elbow_shorts[q][1] = elbow_cells[q][1];
    elbow_shorts[q][2] = elbow_cells[q][2];
  }

  if(!elbow_dijkstra_->setGoals(elbow_shorts))
  {
    SBPL_ERROR("[setGoalPosition] Failed to set goal for the h_elbow computation.\n");
    elbow_heuristic_completed_ = false;
    heuristic_mutex_.unlock();
    return false;
  }

  if(!elbow_dijkstra_->runBFS())
  {
    elbow_heuristic_completed_ = false;
    heuristic_mutex_.unlock();
    return false;
  }

  elbow_heuristic_completed_ = true;
  heuristic_mutex_.unlock();

  SBPL_DEBUG("[precomputeElbowHeuristic] Exiting.\n");
  return true;
}

void EnvironmentROBARM3D::clearStats()
{
  //a flag used for debugging only
  near_goal = false;

  //clear lists of stateIDs  (for debugging only)
  expanded_states.clear();
  EnvROBARMCfg.ik_solution_is_valid = false;

  EnvROBARMCfg.num_calls_to_ik = 0;
  EnvROBARMCfg.num_ik_invalid_path = 0;
  EnvROBARMCfg.num_invalid_ik_solutions = 0;
  EnvROBARMCfg.num_no_ik_solutions = 0;
  EnvROBARMCfg.num_ik_invalid_joint_limits = 0;

  //store the distance of the goal pose to the nearest obstacle (6/20/10)
  int xyz[3] = {(int)(EnvROBARMCfg.EndEffGoals[0].xyz[0]), 
                (int)(EnvROBARMCfg.EndEffGoals[0].xyz[1]),
                (int)(EnvROBARMCfg.EndEffGoals[0].xyz[2])};
  EnvROBARMCfg.goal_to_obstacle_distance = grid_->getCell(xyz);


  EnvROBARMCfg.solved_by_os = 0;
  EnvROBARMCfg.solved_by_ik = 0;
  EnvROBARMCfg.num_expands_to_position_constraint = 0;

  elbow_cells_.clear();
  elbow_poses_.clear();

  //start the 'planning time' clock
  starttime = clock();
}

void EnvironmentROBARM3D::StateID2Angles(int stateID, std::vector<double> &angles)
{
  EnvROBARM3DHashEntry_t* HashEntry = EnvROBARM.StateID2CoordTable[stateID];

  if(stateID == EnvROBARM.goalHashEntry->stateID)
  {
    coordToAngles(EnvROBARM.goalHashEntry->coord, angles);
    
    angles.resize(14);
    for(unsigned int i = 0; i < (unsigned int)angles.size(); i++)
    {
      if(i < 7)
        angles[i] = prefinal_joint_config[i];
      else
        angles[i] = final_joint_config[i-7];
    }
  }
  else
    coordToAngles(HashEntry->coord, angles);

  for (size_t i = 0; i < angles.size(); i++)
  {
    if(angles[i] >= M_PI)
      angles[i] = -2.0*M_PI + angles[i];
  }
}

void EnvironmentROBARM3D::printJointArray(FILE* fOut, EnvROBARM3DHashEntry_t* HashEntry, bool bGoal, bool bVerbose)
{
  int i;
  std::vector<double> angles(arm_->num_joints_,0);

  if(bGoal)
    coordToAngles(EnvROBARM.goalHashEntry->coord, angles);
  else
    coordToAngles(HashEntry->coord, angles);

  if(bVerbose)
    SBPL_DEBUG_NAMED(fOut, "angles: ");

  for(i = 0; i < int(angles.size()); i++)
  {
    if(i > 0)
      SBPL_DEBUG_NAMED(fOut, "%-.3f ", angles[i]-angles[i-1]);
    else
      SBPL_DEBUG_NAMED(fOut, "%-.3f ", angles[i]);
  }

  if(bVerbose)
    SBPL_DEBUG_NAMED(fOut, "  xyz: %-2d %-2d %-2d  rpy: %2.2f %2.2f %2.2f  action: %d", HashEntry->xyz[0],HashEntry->xyz[1],HashEntry->xyz[2], HashEntry->rpy[0], HashEntry->rpy[1], HashEntry->rpy[2], HashEntry->action);
  else
    SBPL_DEBUG_NAMED(fOut, "%-2d %-2d %-2d %-2d", HashEntry->xyz[0],HashEntry->xyz[1],HashEntry->xyz[2],HashEntry->action);
}

std::vector<int> EnvironmentROBARM3D::debugExpandedStates()
{
  return expanded_states;
}

void EnvironmentROBARM3D::getExpandedStates(std::vector<std::vector<double> >* ara_states)
{
  std::vector<double> angles(arm_->num_joints_,0);
  std::vector<double>state(7,0);

  for(unsigned int i = 0; i < expanded_states.size(); ++i)
  {
    StateID2Angles(expanded_states[i],angles);
    arm_->getPlanningJointPose(angles,state);
    state[6] = EnvROBARM.StateID2CoordTable[expanded_states[i]]->heur;
    ara_states->push_back(state);
  }
}

void EnvironmentROBARM3D::computeCostPerCell()
{
  int largest_action=0;
  double gridcell_size, eucl_dist, max_dist = 0;
  std::vector<double> pose(6,0), start_pose(6,0), angles(arm_->num_joints_,0), start_angles(arm_->num_joints_,0);

  gridcell_size = grid_->getResolution();

  //starting at zeroed angles, find end effector position after each action
  arm_->getPlanningJointPose(start_angles, start_pose);

  //iterate through all possible actions and find the one with the minimum cost per cell
  for (int i = 0; i < prms_.num_mprims_; i++)
  {
    for(int j = 0; j < arm_->num_joints_; j++)
      angles[j] = DEG2RAD(prms_.mprims_[i][j]);

    //starting at zeroed angles, find end effector position after each action
    if(!arm_->getPlanningJointPose(angles, pose))
    {
      SBPL_WARN("Failed to compute cost per cell because forward kinematics is returning an error.");
      return;
    }

    eucl_dist = sqrt((start_pose[0]-pose[0])*(start_pose[0]-pose[0]) +
        (start_pose[1]-pose[1])*(start_pose[1]-pose[1]) +
        (start_pose[2]-pose[2])*(start_pose[2]-pose[2]));

    if (eucl_dist > max_dist)
    {
      max_dist = eucl_dist;
      largest_action = i;
    }
  }

  prms_.setCellCost(int(prms_.cost_multiplier_ / (max_dist/gridcell_size)));

  prms_.cost_per_meter_ = int(prms_.cost_per_cell_ / gridcell_size);

  SBPL_DEBUG("[computeCostPerCell] cost per cell: %d cost per meter: %d",prms_.cost_per_cell_,prms_.cost_per_meter_);
}

double EnvironmentROBARM3D::getDistToClosestGoal(double *xyz, int* goal_num)
{
  unsigned int i,ind = 0;
  double dist, min_dist = 10000000;

  for(i=0; i < EnvROBARMCfg.EndEffGoals.size(); i++)
  {
    dist = sqrt((EnvROBARMCfg.EndEffGoals[i].pos[0] - xyz[0])*(EnvROBARMCfg.EndEffGoals[i].pos[0] - xyz[0]) +
        (EnvROBARMCfg.EndEffGoals[i].pos[1] - xyz[1])*(EnvROBARMCfg.EndEffGoals[i].pos[1] - xyz[1]) +
        (EnvROBARMCfg.EndEffGoals[i].pos[2] - xyz[2])*(EnvROBARMCfg.EndEffGoals[i].pos[2] - xyz[2]));
    if(dist < min_dist)
    {
      ind = i;
      min_dist = dist;
    }
  }

  (*goal_num) = ind;
  return min_dist;
}

int EnvironmentROBARM3D::getDijkstraDistToGoal(short unsigned int x, short unsigned int y, short unsigned int z) const
{
  int endeff_cc[3] = {x, y, z};

  return int(dijkstra_->getDist(endeff_cc[0],endeff_cc[1],endeff_cc[2]));
}

void EnvironmentROBARM3D::updateOccupancyGridFromCollisionMap(const arm_navigation_msgs::CollisionMap &collision_map)
{
  if(collision_map.boxes.empty())
  {
    SBPL_ERROR("[updateOccupancyGridFromCollisionMap] collision map received is empty.");
    return;
  }
  else
    SBPL_DEBUG("[updateOccupancyGridFromCollisionMap] updating distance field with collision map with %d boxes.", int(collision_map.boxes.size()));

  grid_->updateFromCollisionMap(collision_map);
}

bool EnvironmentROBARM3D::isValidJointConfiguration(const std::vector<double> angles)
{
  unsigned char dist;
  return cspace_->checkCollision(angles,prms_.verbose_, false, dist);
}

bool EnvironmentROBARM3D::isPathValid(const std::vector<std::vector<double> > path)
{
  unsigned char dist;

  //loop through each waypoint of the path and check if it's valid
  for(int i = 0; i < int(path.size()); ++i)
  {
    if(!cspace_->checkCollision(path[i], false, false, dist))
    {
      SBPL_ERROR("[isPathValid] Waypoint #%d in path is invalid.", i);
      return false;
    }
  }
  return true;
}

bool EnvironmentROBARM3D::interpolatePathToGoal(std::vector<std::vector<double> > &path, double inc)
{
  unsigned int i = 0, completed_joints = 0, pind = path.size() - 2;
  double diff;
  std::vector<double> waypoint(path[pind].size(), 0);
  std::vector<double> angles(arm_->num_joints_,0);

  if(path.size() < 2)
  {
    SBPL_ERROR("[interpolatePathToGoal] no waypoints in path.");
    return false;
  }

  for(unsigned int i = 0; i < path.size(); ++i)
  {
    //normalize joint angles between 0->2PI
    for(unsigned int j = 0; j < path[0].size(); ++j)
    {
      if(path[i][j] < 0)
        path[i][j] = 2.0*M_PI + path[i][j];
    }
  }

  while(path[pind].size() != completed_joints)
  {
    completed_joints = 0;
    for(i = 0; i < path[pind].size(); ++i)
    {
      if(path[pind][i] == path.back()[i])
      {
        waypoint[i] = path[pind][i];
        ++completed_joints;
      }
      else
      {
        diff = path.back()[i] - path[pind][i];
        if(diff < 0) // the current goal is higher than the goal angle
        {
          if(min(fabs(diff), inc) == inc)
            waypoint[i]= path[pind][i] - inc;
          else
            waypoint[i]= path[pind][i] + diff;
        }
        else
        {
          if(min(fabs(diff), inc) == inc)
            waypoint[i]= path[pind][i] + inc;
          else
            waypoint[i]= path[pind][i] + diff;
        }
      }
    }
    ++pind;

    for(int j = 0; j < arm_->num_joints_; ++j)
      angles[j] = waypoint[j];

    if(!isValidJointConfiguration(waypoint))
      return false;
    
    path.insert(path.begin()+pind, waypoint);
  }
  return true;
}

std::vector<std::vector<double> > EnvironmentROBARM3D::getShortestPath()
{
  std::vector<short unsigned int> start(3,0);
  std::vector<double> waypoint(3,0);
  std::vector<std::vector<int> > path;
  std::vector<std::vector<double> > dpath; 

  //compute a Dijkstra path to goal 
  if(prms_.use_dijkstra_heuristic_)
  {
    start[0] = EnvROBARM.startHashEntry->xyz[0];
    start[1] = EnvROBARM.startHashEntry->xyz[1];
    start[2] = EnvROBARM.startHashEntry->xyz[2];

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
  }

  for(int i=0; i < int(path.size()); ++i)
  {
    grid_->gridToWorld(path[i][0], path[i][1], path[i][2], waypoint[0], waypoint[1], waypoint[2]);
    dpath.push_back(waypoint);
  }

  return dpath;
}

void EnvironmentROBARM3D::getBresenhamPath(const short unsigned int a[],const short unsigned int b[], std::vector<std::vector<int> > *path)
{
  bresenham3d_param_t params;
  std::vector<int> nXYZ(3,0);

  //iterate through the points on the segment
  get_bresenham3d_parameters(a[0], a[1], a[2], b[0], b[1], b[2], &params);
  do {
    get_current_point3d(&params, &(nXYZ[0]), &(nXYZ[1]), &(nXYZ[2]));

    path->push_back(nXYZ);
  } while (get_next_point3d(&params));

  SBPL_DEBUG("[getBresenhamPath] Path has %d waypoints.",int(path->size()));
}

void EnvironmentROBARM3D::debugAdaptiveMotionPrims()
{
  if(prms_.use_6d_pose_goal_)
  {
    rpysolver_->printStats();

    SBPL_INFO("Calls to IK: %d   No Solutions: %d  Invalid Joint Limits: %d   Invalid Solutions: %d   Invalid Paths: %d", EnvROBARMCfg.num_calls_to_ik, EnvROBARMCfg.num_no_ik_solutions, EnvROBARMCfg.num_ik_invalid_joint_limits,EnvROBARMCfg.num_invalid_ik_solutions, EnvROBARMCfg.num_ik_invalid_path);
  }
  
  //print out elbow cell for final joint configuration - 9/5/2010
  if(prms_.use_research_heuristic_)
  {
    int x,y,z;
    std::vector<double> pose(6,0);
    if(arm_->computeFK(final_joint_config, 7, pose))
    {
      grid_->worldToGrid(pose[0],pose[1],pose[2], x, y, z);
      bool in_list = false;
      for(size_t i = 0; i < elbow_cells_.size(); i++)
      {
        if(elbow_cells_[i][0] == x && elbow_cells_[i][1] == y && elbow_cells_[i][2] ==z)
        {
          SBPL_DEBUG("Elbow pose at goal *is* one of the elbow goal poses.");
          in_list = true;
        }
      }
      if(!in_list)
      {
        SBPL_DEBUG("Elbow pose at goal (%d %d %d) *is not* one of the elbow goal poses.\nPoses are:", x, y, z);
       for(size_t i = 0; i < elbow_cells_.size(); i++)
          SBPL_DEBUG("%d: %d %d %d\n", int(i), elbow_cells_[i][0], elbow_cells_[i][1], elbow_cells_[i][2]);
      }
    }
  }
}

void EnvironmentROBARM3D::visualizeOccupancyGrid()
{
  grid_->visualize();
}

void EnvironmentROBARM3D::setReferenceFrameTransform(KDL::Frame f, std::string &name)
{
  arm_->setRefFrameTransform(f, name);
}

void EnvironmentROBARM3D::getArmChainRootLinkName(std::string &name)
{
  arm_->getArmChainRootLinkName(name);
}

std::vector<double> EnvironmentROBARM3D::getPlanningStats()
{
  /* Planning Stats returned from this function:
   *
   * {dist_from_goal_to_nearest_obs(m), x_error, y_error, z_error,
   * roll_error, pitch_error, yaw_error, num_OS_calls,
   * num_OS_predicts_invalid, num_OS_invalid_config,
   * num_OS_invalid_path, num_IK_calls, num_no_IK_solution,
   * num_IK_invalid_config, num_IK_invalid_path, num_expands_to_3dof_goal,
   * num_elbow_cells, is_elbow_cell_in_list}
   */  
  bool in_list = false;
  int x,y,z;
  std::vector<double> stats, pose(6,0), angles(7,0);

  // position & orientation error in meters & radians
  if(prms_.use_6d_pose_goal_)
    angles = final_joint_config;
  else
    coordToAngles(EnvROBARM.goalHashEntry->coord, angles);

  if(!arm_->getPlanningJointPose(angles, pose))
  {
    SBPL_ERROR("[getPlanningStats] Can not compute final pose of trajectory.\n");
    return stats;
  }

  // log if elbow pose is in list of elbow cells
  if(prms_.use_research_heuristic_)
  {
    if(arm_->computeFK(final_joint_config, 7, pose))
    {
      grid_->worldToGrid(pose[0],pose[1],pose[2],x,y,z);
      for(size_t i = 0; i < elbow_cells_.size(); i++)
      {
        if(elbow_cells_[i][0] == x && elbow_cells_[i][1] == y && elbow_cells_[i][2] ==z)
          in_list = true;
      }
    }
  }

  // distance of goal pose to nearest obstacle
  stats.push_back(EnvROBARMCfg.goal_to_obstacle_distance);

  // goal position & orientation error (should be improved)
  stats.push_back(fabs(EnvROBARMCfg.EndEffGoals[0].pos[0] - pose[0]));
  stats.push_back(fabs(EnvROBARMCfg.EndEffGoals[0].pos[1] - pose[1]));
  stats.push_back(fabs(EnvROBARMCfg.EndEffGoals[0].pos[2] - pose[2]));
  stats.push_back(fabs(EnvROBARMCfg.EndEffGoals[0].rpy[0] - pose[3]));
  stats.push_back(fabs(EnvROBARMCfg.EndEffGoals[0].rpy[1] - pose[4]));
  stats.push_back(fabs(EnvROBARMCfg.EndEffGoals[0].rpy[2] - pose[5]));

  // orientation solver & IK stats
//  stats.push_back(EnvROBARMCfg.num_orientation_planner_calls);
//  stats.push_back(EnvROBARMCfg.num_orientation_planner_predicts_invalid);
//  stats.push_back(EnvROBARMCfg.num_orientation_planner_invalid_config);
//  stats.push_back(EnvROBARMCfg.num_orientation_planner_invalid_path);
  stats.push_back(EnvROBARMCfg.num_calls_to_ik);
  stats.push_back(EnvROBARMCfg.num_no_ik_solutions);
  stats.push_back(EnvROBARMCfg.num_invalid_ik_solutions);
  stats.push_back(EnvROBARMCfg.num_ik_invalid_path);

  // # expansions until position constraint satisfied
  stats.push_back(EnvROBARMCfg.num_expands_to_position_constraint);
 
  stats.push_back(EnvROBARMCfg.solved_by_ik); 
  stats.push_back(EnvROBARMCfg.solved_by_os); 

  // number of elbow cells
  stats.push_back(elbow_cells_.size());

  // is actual elbow cell in elbow cells list (1:yes, 0:no)
  stats.push_back(double(in_list));

  return stats;
}

bool EnvironmentROBARM3D::getElbowCellsAtGoal(std::vector<int> &shoulder, std::vector<double> &goal_m, double rad1, double rad2, std::vector<std::vector<int> > &cells)
{
  std::vector<int> cell(3,0), goal(3,0);
  std::list<std::vector<int> > valid_cells;
  std::vector<double> angles(7,0);
  std::vector<std::vector<double> > points_vector;
  
  grid_->worldToGrid(goal_m[0], goal_m[1], goal_m[2], cell[0], cell[1], cell[2]);
 
  //get starting shoulder_pan angle
  coordToAngles(EnvROBARM.startHashEntry->coord, angles);

  points_vector = elbow_positions_given_endeff_pose(angles::normalize_angle(angles[0]), prms_.getSmallestShoulderPanMotion(), goal_m[0], goal_m[1], goal_m[2]);
  std::list<std::vector<double> > valid_points(points_vector.begin(), points_vector.end());

  if(points_vector.empty())
  {
    SBPL_ERROR("No potential elbow cells found. Exiting.");
    return false;
  }

  //added for debugging & visualization 9/5/2010
  elbow_poses_ = points_vector;

  //convert to grid cells
  for(std::list<std::vector<double> >::iterator it = valid_points.begin(); it != valid_points.end(); ++it)
  {
    grid_->worldToGrid((*it)[0], (*it)[1], (*it)[2], cell[0], cell[1], cell[2]);
    valid_cells.push_back(cell);
  }

  SBPL_DEBUG("[getElbowCellsAtGoal] There are %d cells before removing of duplicates",int(valid_cells.size()));
 
  //sort cells to speed up the removing of duplicates
  valid_cells.sort(xyzCompare);

  std::list<std::vector<int> >::iterator last_dup, next_it, end_it, delete_it;

  end_it = valid_cells.end();
  end_it--;

  //remove duplicates
  for(std::list<std::vector<int> >::iterator it = valid_cells.begin(); it != end_it; ++it)
  {
    next_it = it;
    next_it++;

    for (std::list<std::vector<int> >::iterator jt = next_it; jt != valid_cells.end(); ++jt)
    {
      last_dup = jt;
      
      if((*it)[0] != (*jt)[0] || (*it)[1] != (*jt)[1] || (*it)[2] != (*jt)[2])
        break;
      //else
      //  SBPL_PRINTF("[getElbowCellsAtGoal] removing duplicate cell %d, %d\n",int(distance(valid_cells.begin(),it)), int(distance(valid_cells.begin(),jt)));
    }

    if(last_dup != next_it)
      valid_cells.erase(next_it, last_dup);

    //check each elbow pose for a collision free path to goal and shoulder
    if(cspace_->isValidLineSegment(goal, (*it), rad1) <= rad1)
    {
      delete_it = it;
      it--;
      valid_cells.erase(delete_it);
    }
    else if(cspace_->isValidLineSegment(shoulder, (*it), rad2) <= rad2)
    {
      delete_it = it;
      it--;
      valid_cells.erase(delete_it);
    }
  }

  SBPL_DEBUG("After removing duplicates & points in collision, %d cells remain.", int(valid_cells.size()));

  for(std::list<std::vector<int> >::iterator it = valid_cells.begin(); it != valid_cells.end(); ++it)
    cells.push_back((*it));

  elbow_cells_ = cells;
  return true;
}

int EnvironmentROBARM3D::getElbowHeuristic(int FromStateID, int ToStateID)
{
  int x,y,z,heur = 0;
  std::vector<double> angles(7,0);
  KDL::Frame F;
  std::vector<std::vector<double> > links;
  
  EnvROBARM3DHashEntry_t* FromHashEntry = EnvROBARM.StateID2CoordTable[FromStateID];

  if(!prms_.use_research_heuristic_)
    return 0;

  //get elbow pose on grid 
  coordToAngles(FromHashEntry->coord, angles);
  arm_->getJointPositions(angles, links, F);
  
  grid_->worldToGrid(links[1][0],links[1][1],links[1][2],x,y,z);

  //get distance heuristic
  if(prms_.use_dijkstra_heuristic_)
    heur = elbow_dijkstra_->getDist(x,y,z);

  return heur;
}

int EnvironmentROBARM3D::getEndEffectorHeuristic(int FromStateID, int ToStateID)
{
  int heur = 0, closest_goal = 0;
  double FromEndEff_m[3];
  double edist_to_goal_m;

  //get X, Y, Z for the state
  EnvROBARM3DHashEntry_t* FromHashEntry = EnvROBARM.StateID2CoordTable[FromStateID];
  int temp[3] = {FromHashEntry->xyz[0], FromHashEntry->xyz[1], FromHashEntry->xyz[2]};

  //distance to closest goal in meters
  grid_->gridToWorld(FromHashEntry->xyz[0],FromHashEntry->xyz[1],FromHashEntry->xyz[2],FromEndEff_m[0],FromEndEff_m[1],FromEndEff_m[2]);
  edist_to_goal_m = getDistToClosestGoal(FromEndEff_m, &closest_goal);

  //get distance heuristic
  if(prms_.use_dijkstra_heuristic_)
    heur = dijkstra_->getDist(temp[0],temp[1],temp[2]);
  else
    heur = edist_to_goal_m * prms_.cost_per_meter_;

  //storing heuristic now for debugging 5/20/10
  FromHashEntry->heur = heur;

  return heur;
}

int EnvironmentROBARM3D::getCombinedHeuristic(int FromStateID, int ToStateID)
{
  int h_endeff = 0, h_elbow = 0, heur = 0;

  h_endeff = getEndEffectorHeuristic(FromStateID, ToStateID);
  h_elbow = getElbowHeuristic(FromStateID, ToStateID);

  //add the heuristics
  if(prms_.sum_heuristics_ == 1)
    heur = h_endeff + h_elbow;
  //use the max heuristic
  else
    heur = max(h_endeff, h_elbow);

#if DEBUG_HEURISTIC
  SBPL_DEBUG_NAMED("heuristic","%5d: h_endeff: %5d  h_elbow: %5d   heur: %5d", FromStateID, h_endeff, h_elbow, heur);
#endif

  return heur;
}

void EnvironmentROBARM3D::getElbowPoints(std::vector<std::vector<double> > &elbow_points)
{
  elbow_points = elbow_poses_;
}

}

