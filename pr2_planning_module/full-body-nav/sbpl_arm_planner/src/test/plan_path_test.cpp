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

#define MAX_RUNTIME 200.0
#define NUM_JOINTS 7

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
  int bRet = 0, env_arg = 4;
  int sol_cost;
  double allocated_time_secs = MAX_RUNTIME; //in seconds
  MDPConfig MDPCfg;
  SBPLPlanner *planner;
  int planner_type = 0;
  std::string args(argv[1]);
  std::string arm_desc_filename("./config/pr2_right_arm.cfg"), params_filename("./config/params.cfg");

  bool save_traj = false;
  char* name = NULL;
  char* path = NULL;

  //if the user chose the planner type 
  if(argc > 2)
  {
    if(args.at(0) == '-')
    {
      //RStar
      if(args.find_first_of('R') != std::string::npos)
        planner_type = 1;
      //Name of Planning Request (for documenting stats)
      else if(args.find_first_of('N') != std::string::npos)
      {
        save_traj = true;
        if(argc < 4)
        {
          printf("Argument '-N' must be accompanied by a path and a test name.\n");
          return 0;
        }
        path = argv[2];
        name = argv[3];
      }
      
      // params file location
      if(argc > 5)
        params_filename = std::string(argv[5]);

      //arm description file location
      if (argc > 6)
        arm_desc_filename = std::string(argv[6]);
    }
  }
  else
    env_arg = 1;

  //Initialize Environment (should be called before initializing anything else)
  sbpl_arm_planner::EnvironmentROBARM3D environment_robarm;

  if(!environment_robarm.InitializeEnv(argv[env_arg], params_filename, arm_desc_filename))
  {
    printf("ERROR: InitializeEnv failed\n");
    return 0;
  }

  //Initialize MDP Info
  if(!environment_robarm.InitializeMDPCfg(&MDPCfg))
  {
    printf("ERROR: InitializeMDPCfg failed\n");
    exit(1);
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
    exit(1);
  }

  if(planner->set_goal(MDPCfg.goalstateid) == 0)
  {
    printf("ERROR: failed to set goal state\n");
    exit(1);
  }

  //set epsilon
  planner->set_initialsolution_eps(environment_robarm.getEpsilon());

  //set search mode (true - settle with first solution)
  planner->set_search_mode(false);

  //plan
  printf("start planning...\n");
  bRet = planner->replan(allocated_time_secs, &solution_stateIDs_V,&sol_cost);
  printf("completed in %.4f seconds.\n", double(clock()-starttime) / CLOCKS_PER_SEC);

  environment_robarm.debugAdaptiveMotionPrims();

  //create arm model so we can run IK to verify final waypoint
  FILE* afid = fopen("./config/pr2_right_arm.cfg","r");
  sbpl_arm_planner::SBPLArmModel arm(afid);
  arm.initKDLChainFromParamServer();

  std::vector<double> pose;

/*
  //record stats
  if(save_traj)
  {
    // write trajectory to text file
    std::vector<double> angles(NUM_JOINTS,0);
    string traj_filename(path);
    traj_filename.append(name);
    traj_filename.append(".csv");

    FILE* fPath = fopen(traj_filename.c_str(),"wt");

    if(fPath != NULL)
    {
      for(int i = 0; i < (int)solution_stateIDs_V.size(); i++) 
      {
        environment_robarm.StateID2Angles(solution_stateIDs_V[i], angles);
        for (int p = 0; p < (int)angles.size(); p++)
        {
          if(p == 7)
            fprintf(fPath,"\n");
          fprintf(fPath, "%0.3f, ",angles[p]);
        }
        fprintf(fPath,"\n");
      }
      fclose(fPath);
    }
    else
      printf("[planPathTest] Unable to create trajectory file. (%s)\n", traj_filename.c_str());

    // append planning stats to experiment_stats.csv
    string stats_filename(path);
    stats_filename.append("/experiment_stats.csv");
    FILE* fStats = fopen(stats_filename.c_str(),"a");

    if(fStats != NULL)
    {

      std::vector<double> stats = planner->get_planning_stats();

      if(stats.empty())
        fprintf(fStats,"[planPathTest] No stats were returned by planner.\n");


      std::vector<double> env_stats = environment_robarm.getPlanningStats();

      fprintf(fStats,"%s, ",name);
      for(int i = 0; i < (int)stats.size(); i++)
        fprintf(fStats,"%0.3f, ",stats[i]);

      for(int i = 0; i < (int)env_stats.size(); i++)
        fprintf(fStats,"%0.3f, ",env_stats[i]);
       fprintf(fStats,"\n");
      fclose(fStats);

    }
    else
      printf("[planPathTest] Unable to open file experiment_stats.csv to append to it. (%s)\n", stats_filename.c_str());
  }
*/

  printf("\ndone\n");

  return bRet;
}

int main(int argc, char *argv[])
{
  if(argc < 2)
  {
    PrintUsage(argv);
    exit(1);
  }
  
  ros::init(argc, argv, "sbpl_arm_planner");

  planRobarm(argc, argv);

  return 0;
}

