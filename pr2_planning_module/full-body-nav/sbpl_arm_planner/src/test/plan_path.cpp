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

#include <iostream>
#include <sbpl_arm_planner/robarm3d/environment_robarm3d.h>
#include <ros/ros.h>

#define VERBOSE 1
#define MAX_RUNTIME 10.0

/**
 \brief Test file for the sbpl_arm_planner.
 \author Ben Cohen
*/

void PrintUsage(char *argv[])
{
  printf("SBPL Arm Planner Test Program\n\n");
  printf("Usage: %s [arguments] <cfg file>\n\n", argv[0]);

  printf("Arguments:\n");
  printf("      -A          ARA* - Anytime Repairing A* (default)\n");
  printf("      -R          R*   - Random A*\n\n");
}

int planRobarm(int argc, char *argv[])
{
  int bRet = 0, env_arg = 2;
  double allocated_time_secs = MAX_RUNTIME; //in seconds
  MDPConfig MDPCfg;
  SBPLPlanner *planner;
  int planner_type = 0;
  std::string args(argv[1]);

  //if the user chose the planner type 
  if(argc > 2)
  {
    if(args.at(0) == '-')
      //RStar
      if(args.find_first_of('R') != std::string::npos)
        planner_type = 1;
  }
  else
    env_arg = 1;

  clock_t totaltime = clock();

  //Initialize Environment (should be called before initializing anything else)
  sbpl_arm_planner::EnvironmentROBARM3D environment_robarm;

  if(!environment_robarm.InitializeEnv(argv[env_arg]))
  {
    printf("ERROR: InitializeEnv failed\n");
    return 0;
  }

  //Initialize MDP Info
  if(!environment_robarm.InitializeMDPCfg(&MDPCfg))
  {
    printf("ERROR: InitializeMDPCfg failed\n");
    return 0;
  }

  //plan a path
  clock_t starttime = clock();

  vector<int> solution_stateIDs_V;
  bool bforwardsearch = true;

  if(planner_type == 1)
  {
    environment_robarm.SetEnvParameter("useRSTAR", 1.0);
    planner = new RSTARPlanner(&environment_robarm, bforwardsearch);
  }
  else
  {
    environment_robarm.SetEnvParameter("useRSTAR",0);
    planner = new ARAPlanner(&environment_robarm, bforwardsearch);
  }

  if(planner->set_start(MDPCfg.startstateid) == 0)
  {
    printf("ERROR: failed to set start state\n");
    return 0;
  }

  if(planner->set_goal(MDPCfg.goalstateid) == 0)
  {
    printf("ERROR: failed to set goal state\n");
    return 0;
  }

  //set epsilon
  planner->set_initialsolution_eps(environment_robarm.getEpsilon());

  //set search mode (true - settle with first solution)
  planner->set_search_mode(false);

  int sol_cost;
  printf("start planning...\n");
  bRet = planner->replan(allocated_time_secs, &solution_stateIDs_V,&sol_cost);

  printf("completed in %.4f seconds.\n", double(clock()-starttime) / CLOCKS_PER_SEC);

  //std::vector<std::vector<double> > dpath = environment_robarm.getShortestPath();
  
  printf("done planning\n");
  std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;
  std::cout << "cost of solution=" << sol_cost << std::endl;

  printf("\ntotal planning time is %.4f seconds.\n", double(clock()-totaltime) / CLOCKS_PER_SEC);

  // create filename with current time
  string outputfile = "sol";
  outputfile.append(".txt");

  FILE* fSol = fopen(outputfile.c_str(), "w");
  for(unsigned int i = 0; i < solution_stateIDs_V.size(); i++) 
    environment_robarm.PrintState(solution_stateIDs_V[i], true, fSol);

  fclose(fSol);

  //to get the trajectory as an array
  //     double angles_r[NUMOFLINKS];
  //     for(unsigned int i = 0; i < solution_stateIDs_V.size(); i++) 
  //     {
  //         environment_robarm.StateID2Angles(solution_stateIDs_V[i], angles_r);
  //         for (int p = 0; p < 7; p++)
  //             printf("% 0.2f  ",angles_r[p]);
  //         printf("\n");
  //     }

  // #if VERBOSE
  //     environment_robarm.OutputPlanningStats();
  // #endif

  return bRet;
}

int main(int argc, char *argv[])
{
  if(argc < 2)
  {
    PrintUsage(argv);
    return 0;
  }
  
  ros::init(argc, argv, "actual_arm_planner");

  if(!planRobarm(argc, argv))
  {
    printf("Planning failed\n");
    return 0;
  }


  return 1;
}

