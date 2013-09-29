#include <iostream>
#include <vector>
#include <string>
#include <limits>
#include <stdio.h>  	// defines FILENAME_MAX
#include <boost/multi_array.hpp>
#include <boost/lexical_cast.hpp>
#include "planning_module/sarsop_user.hpp"


std::string get_working_path()
{
   char temp[FILENAME_MAX];
   return ( getcwd(temp, FILENAME_MAX) ? std::string(temp) : std::string("") );
}

void test_su(bool test_new)
{
	// num viewpoints + 1 sink state
	int num_sta = 16;
	
	// number of hypothese
	int num_hid = 5;
	
	// num real viewpints + num decision actions
	// act_(0) = go to state 0
	// act_(num_sta - 2) = go to last real state
	// act_(num_sta - 1) = first decision action
	// act_(num_act-1) = last decision action
	int num_act = (num_sta-1) + num_hid;
	
	// discretization of the detection scores
	int num_obs = 21;
	
	// initial state
	int init_sta = 0;
	
	// initial belief
	std::vector<double> init_bel;
	for(int hid = 0; hid < num_hid; ++hid)
		init_bel.push_back(1.0/num_hid);
	
	// NB: USE ABSOLUTE FILE PATHS
	std::string sarsopPath("/media/Data/Stuff/Research/GloballyAvailableSoftware/CppLibraries/POMDP/SARSOP/appl-0.95/src/pomdpsol");
	std::string pomdpxFilePath(get_working_path()+"/test_" + boost::lexical_cast<std::string>(init_sta) + ".pomdpx");
	std::string policyFilePath(get_working_path()+"/test_"  + boost::lexical_cast<std::string>(init_sta) + ".policy");
	
	// Initialize fMap, cMap, and oMap
	sarsop_user::array2I fMap(boost::extents[num_act][num_sta]);
	sarsop_user::array3D cMap(boost::extents[num_act][num_sta][num_hid]);
	sarsop_user::array3D oMap(boost::extents[num_obs][num_sta][num_hid]);
	// int fMap[num_act][num_sta];
	// double cMap[num_act][num_sta][num_hid];
	// double oMap[num_obs][num_sta][num_hid];

	
	
	for (int act = 0; act < num_act; ++act)
		for (int sta = 0; sta < num_sta; ++sta){
			
			// always go to sink state after a decision
			// and if in sink state stay there
			if((act >= (num_sta-1))||(sta == (num_sta - 1)))
				fMap[act][sta] = (num_sta - 1);
			else
				fMap[act][sta] = act;
		}
		
	// Initialize cMap
	double L[num_hid][num_hid];
	for(int a = 0; a<num_hid; ++a)
		for(int b = 0; b<num_hid; ++b){
			if(a == b)
				L[a][b] = 0;
			else
				L[a][b] = 50;
		}
			

	double msrmnt_cost = 1;
	int next_idx = 0;
	double dist = 0.0;
	

	for (int act = 0; act < num_act; ++act)
		for (int sta = 0; sta < num_sta; ++sta)
			for(int hid = 0; hid < num_hid; ++hid){
					
				if(sta == (num_sta-1))
					cMap[act][sta][hid] = 0.0;	// sink state
					
				else if(act >= (num_sta-1))
					cMap[act][sta][hid] = L[(act - num_sta+1)][hid];
					
				else{
					next_idx = fMap[act][sta];
					
					if (next_idx != sta){
						dist = 1.0;	// Call great circle dist here
						cMap[act][sta][hid] = msrmnt_cost + dist;
					}
					else{
						cMap[act][sta][hid] = std::numeric_limits<double>::infinity();
					}
				}
			}
	
	// Initialize oMap
	for (int obs = 0; obs < num_obs; ++obs)
		for (int sta = 0; sta < num_sta; ++sta)
			for(int hid = 0; hid < num_hid; ++hid){
				if(sta < (num_sta-1)){
					// oMap[obs][sta][hid] = h(obs,sta,hid);
					// use Observation model here
					oMap[obs][sta][hid] = 1.0/num_obs;
				}else{
					// uniform distribution in sink state
					oMap[obs][sta][hid] = 1.0/num_obs;
				}
			}
			
	// Create a sarsop_user object
	sarsop_user su(sarsopPath);


	// Generate a pomdpx model
	bool status;
	if(test_new)
		status = su.modelToFile( fMap, cMap, oMap, pomdpxFilePath, init_sta, init_bel.begin());
	else
		status = su.modelToFile( num_sta, num_hid, num_act, num_obs,
					   				 fMap.data(), cMap.data(), oMap.data(),
									 pomdpxFilePath, init_sta, init_bel.begin() );		

	// Calculate the optimal policy
	double precision = 0.5; //difference between upper and lower bound in optimal value
	double time_limit = 60;	//[sec]
	status = su.computePolicy(precision, time_limit, pomdpxFilePath, policyFilePath);
	
	// Load the policy
	su.loadPolicy( init_sta, get_working_path()+"/test" );
	
	// Get the best action
	int best_act = -1;
	if(status){
		best_act = su.getBestAction(init_sta, init_bel.begin(),
									init_bel.end());
	}
	
	// Done
	std::cout << "The status is " << status << std::endl;
	std::cout << "The best action is " << best_act << std::endl;
	//std::cout << "WD = " << get_working_path() << std::endl;
}


int main(int argc, char **argv){

	test_su(true);
	
	return 0;
}
