/* \author Gokul Subramanian */

#ifndef _PR2_WORKSPACE_
#define _PR2_WORKSPACE_


#include <iostream>
#include <fstream>
#include <cstdlib>
#include <vector>
#include <math.h>
#include <sbpl_arm_planner/pr2/sbpl_math.h>
using namespace std;

//Function prototypes
void grid2world(long, long, long, double&, double&, double&);
void world2grid(double, double, double, long&, long&, long&);
vector<vector<double> > elbow_positions_given_pan(double, double, double, double, double, double, double, double, double[3]);
vector<vector<double> > elbow_positions_given_endeff_pose(double, double, double, double, double);
void elbow_positions_given_endeff_pose(vector<vector<double> >&, vector<vector<double> >&, double, double, double, double, double);
void separate_rejected_points(vector<vector<double> >, vector<vector<double> >&, vector<vector<double> >&, vector<vector<double> >&);    
bool check_joint_limits_and_append_joint_angles(double, double[3], double[3], vector<double>&, double[3]);
bool check_joint_limits(double[3], double[3], vector<double>, double[3]);
bool position(long, long, long);

#endif
