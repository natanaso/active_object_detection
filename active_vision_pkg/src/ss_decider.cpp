// Standard
#include <stdexcept>
#include <stdlib.h>	// rand()
#include <cmath>	// log()

#include <pcl/io/pcd_io.h>
#include <ros/package.h>

// Custom
#include <misc.hpp>
#include <io_utils.hpp>
#include "ss_decider.hpp"
#include <planning_module/ejs_planner.hpp>
#include <planning_module/gmi_planner.hpp>
#include <planning_module/cvp_planner.hpp>
	
int
ss_decider::start_offline( std::string obj_list_path,
						    std::string hyp_path,
							 std::string tree_vps_path,
							 std::string omap_vps_path,
							 std::string plan_vps_path,
							 std::string omap_dir,
							 std::string save_dir,
							 std::string database_dir,
							 std::string clouds_dir )
{
	start( obj_list_path, hyp_path, tree_vps_path,
				omap_vps_path, plan_vps_path);
				
	this->omap_dir = omap_dir;
	this->save_dir = save_dir;

	write_fmap();
	write_cmap();
	write_omap();
	
	set_prior();
	write_policy();

	int num_vp = 1; //num_sta-1;
	su.loadAllPolicies( num_vp, save_dir + "/plan" );
	
	/*
	bool valid = su.fileToPolicy( save_dir + "/plan.policy" );
	if(!valid)
		throw std::runtime_error("Policy file read failed...\n");
	*/
	
	vtu.set_database_dir( database_dir );
	vtu.set_clouds_dir( clouds_dir );
	if( vtu.start() )
		throw std::runtime_error("Loading vocabulary tree database failed...\n");	
	
	return 0;
}

int
ss_decider::start_online( std::string obj_list_path,
							std::string hyp_path,
							std::string tree_vps_path,
							std::string omap_vps_path,
							std::string plan_vps_path,
	                  std::string save_dir,
							std::string omap_file_path,
							std::string cmap_file_path,
							std::string database_dir,
							std::string clouds_dir )
{
	start( obj_list_path, hyp_path, tree_vps_path,
				omap_vps_path, plan_vps_path);

	//this-> policy_file_path = policy_file_path;
	this->save_dir = save_dir;
	this-> omap_file_path = omap_file_path;
	this-> cmap_file_path = cmap_file_path;
	
	read_omap();
	read_dmap();
	set_prior();
	
	int num_vp = 1; //num_sta-1
	su.loadAllPolicies( num_vp, save_dir + "/plan" );
	
	/*
	bool valid = su.fileToPolicy( policy_file_path );
	if(!valid)
		throw std::runtime_error("Policy file read failed...\n");
	*/
	
	vtu.set_database_dir( database_dir );
	vtu.set_clouds_dir( clouds_dir );
	if( vtu.start() )
		throw std::runtime_error("Loading vocabulary tree database failed...\n");
	
	return 0;
}


int
ss_decider::start( std::string obj_list_path,
					  std::string hyp_path,
					  std::string tree_vps_path,
					  std::string omap_vps_path,
					  std::string plan_vps_path )
{
	this->obj_list_path = obj_list_path;
	this->hyp_path = hyp_path;
	this->tree_vps_path = tree_vps_path;
	this->omap_vps_path = omap_vps_path;
	this->plan_vps_path = plan_vps_path;
	
	read_obj();	
	read_hyp();				// sets num_hid
	read_vps();				// sets num_sta and num_act and tree_vps
	fill_obj_id_map();	// sets num_obs
	
	std::string planning_module_dir( ros::package::getPath("planning_module") );
	su.set_sarsop_path( planning_module_dir + "/appl-0.95/src/pomdpsol" );
	
	return 0;
}


//**************************************************************************
// MAIN FUNCTIONS			  
int
ss_decider::initial_alignment( Eigen::Vector3d const& position, Eigen::Vector3d const& target, 
					   		   Eigen::Vector3d & goal_position,  Eigen::Vector4d & goal_orientation )
{
	// determine the closest point from the plan view points
	int vp = misc::find_closest_viewpoint( target, plan_vps, position, goal_position, goal_orientation );


	// if we are too far away from the target change the goal_position to be closer
	double rad = (target - goal_position).norm();
	if ( rad > 1.4 )
	{
		rad = 1.0;
		goal_position = (rad * plan_vps.row(vp).transpose()) + target;
	}
	
	return vp;
}

// returns true if done and false otherwise. The optimal action is returned as second
std::pair<ss_decider::Decider_State,int> 
ss_decider::decider_spin( Eigen::Vector3d const& position, Eigen::Vector3d const& target,
					  	  pcl::PointCloud<pcl::PointXYZ>::Ptr cld_ptr, 
					  	  std::vector<double>::iterator curr_bel_start, 
					  	  std::vector<double>::iterator curr_bel_end,
					  	  Eigen::Vector3d & goal_position,  Eigen::Vector4d & goal_orientation )
{	
	// determine the closest point from the plan view points
	int vp = misc::find_closest_viewpoint( target, plan_vps, position, goal_position, goal_orientation );
	
	std::cout << "vp = " << vp << std::endl;
	
	// Get the observation from the vocabulary tree if the test surface contains points
	if( cld_ptr -> size() < 2000 ){
		ROS_WARN("The object is completely occluded! Skipping observation!");
	}
	else
	{
		/*
		// Save the test surface for viewing purposes (in ASCII)
		ROS_WARN("Current vp %d", vp);
		pcl::PCDWriter writer;
		writer.writeASCII( "/media/natanaso/Data/Stuff/Research/git/icra_2013/ns_shared/test_vp_"
								 + boost::lexical_cast<std::string>(vp) + ".pcd", *cld_ptr );
		*/
	
		int obs = get_observation( cld_ptr );
		//int obs = get_fake_observation( vp );
	
		std::cout << "obs = " << obs << std::endl;
	
		// update the belief
		double denum = 0.0;
	
		{
			int h = 0;
			for( std::vector<double>::iterator iter = curr_bel_start;
				 iter != curr_bel_end; ++iter, ++h)
			{
				(*iter) = oMap[obs][vp][h] * (*iter);
				denum += (*iter);
			}
		}
	
		// normalize
		for( std::vector<double>::iterator iter = curr_bel_start;
			 iter != curr_bel_end; ++iter)
		{
			(*iter) = (*iter) / denum;
		}
	}
	
	// Use the policy to get the best action
	int best_act = su.getBestAction( vp, curr_bel_start, curr_bel_end );
		
	std::cout << "act = " << best_act << std::endl;
	
	// calculate the goal pose
	ss_decider::Decider_State st;
	if((best_act >=0) && (best_act < num_sta-1))
	{	// movement action
		std::cout << "Got a movement action : " << best_act << std::endl;
		
		double rad = 1.0;	//note: radius is kept at 1m
		goal_position = (rad * plan_vps.row(best_act).transpose()) + target;		
		goal_orientation = misc::target2quat( goal_position, target );
		st = MOVE;
	}
	else if ((best_act >= num_sta-1) && (best_act < num_act))
	{	// decision action
		best_act = (best_act - (num_sta-1));
		std::cout << "Got a decision action : " << best_act << std::endl;
		std::cout << "Object is : " << hyp_list[best_act].name << std::endl;
		
		st = DONE;
	}
	else
		throw std::runtime_error("Invalid action received...\n");
	
		
	return std::make_pair(st, best_act);
}



// returns true if done and false otherwise. The optimal action is returned as second
std::pair<ss_decider::Decider_State,int> 
ss_decider::decider_spin( Eigen::Vector3d const& position, Eigen::Vector3d const& target,
					  	  pcl::PointCloud<pcl::PointXYZ>::Ptr cld_ptr, 
					  	  std::map< int, boost::shared_ptr<obj> > & obj_list, int obj_id,
					  	  Eigen::Vector3d & goal_position,  Eigen::Vector4d & goal_orientation )
{	
	// current object
	boost::shared_ptr<obj> & curr_OBJ_ptr = obj_list[obj_id];
	
	// determine the closest point from the plan view points
	int vp = misc::find_closest_viewpoint( target, plan_vps, position, goal_position, goal_orientation );
	curr_OBJ_ptr -> set_visited_vps(vp, get_num_sta()-1);
	
	
	std::cout << "vp = " << vp << std::endl;
	
	std::vector<double>::iterator curr_bel_start;
	std::vector<double>::iterator curr_bel_end;
	curr_OBJ_ptr->get_belief( curr_bel_start, curr_bel_end);
		
	// Get the observation from the vocabulary tree if the test surface contains points
	int best_act = -1;
	if( cld_ptr -> size() < 2000 )
	{
		ROS_WARN("[ss_decider] The object is completely occluded! Size is: %d. Skipping observation!", cld_ptr -> size());
		// USE GMI to go to a viewpoint that was not visited before
		std::vector<int> vp_seq;
		gmi_planner::get_GMI_sequence( 0.4, vp, oMap, dMap, curr_bel_start, curr_bel_end, vp_seq );
		
		ROS_WARN("[ss_decider] Inspecting greedy mutual information sequence for a non-occluded viewpoint!");
		
		/*
		ROS_INFO_STREAM("[ss_decider] GMI sequence: " << vp_seq[0] << " "
																	 << vp_seq[1] << " "
																	 << vp_seq[2]);
		*/
		for(vp = 0 ; vp<num_sta-1; ++vp)
		{
			// if decision action you can stop
			if(vp_seq[vp] >= num_sta-1 )
			{
				best_act = vp_seq[vp];		
				break;
			}
		
			// if the point is not visited, check occlusion
			if(!curr_OBJ_ptr->is_vp_visited(vp_seq[vp]))
			{
				// check occlusions versus all objects
				//note: radius is kept at 1m
				goal_position = plan_vps.row(vp_seq[vp]).transpose() + target;
				bool occluded = false;
				for( std::map< int, boost::shared_ptr<obj> >::iterator it = obj_list.begin();
						it != obj_list.end(); ++it)
				{
					// check only the other objects
					if( ( (it->second)->get_id() != obj_id ) &&
						 ( (it->second)->ray_intersect( goal_position, target)) )
					{
						occluded = true;
						break;
					}		
				}
				if( !occluded )
				{
					best_act = vp_seq[vp];
					break;
				}
			}	
		}
		if(best_act == -1)
			best_act = misc::uniform_int( 0, num_sta-2 );
	}
	else
	{
		/*
		// Save the test surface for viewing purposes (in ASCII)
		//ROS_WARN("Current vp %d", vp);
		pcl::PCDWriter writer;
		writer.writeASCII( "/home/natanaso/Desktop/test_surf/obj_"
								 + boost::lexical_cast<std::string>(curr_OBJ_ptr->get_id()) + "_vp_"
								 + boost::lexical_cast<std::string>(vp) + ".pcd", *cld_ptr );
		
		*/
	
		
		int obs = get_observation( cld_ptr );
		//int obs = get_fake_observation( vp );
		//int obs = 49;
		std::cout << "obs = " << obs << std::endl;
	
		// update the belief		
		double denum = 0.0;
	
		{
			int h = 0;
			for( std::vector<double>::iterator iter = curr_bel_start;
				 iter != curr_bel_end; ++iter, ++h)
			{
				(*iter) = oMap[obs][vp][h] * (*iter);
				denum += (*iter);
			}
		}
	
		// normalize
		for( std::vector<double>::iterator iter = curr_bel_start;
			 iter != curr_bel_end; ++iter)
		{
			(*iter) = (*iter) / denum;
		}
		
		// Use the policy to get the best action
		best_act = su.getBestAction( vp, curr_bel_start, curr_bel_end );
	}
	
	
	std::cout << "act = " << best_act << std::endl;
	
	// calculate the goal pose
	ss_decider::Decider_State st;
	if((best_act >=0) && (best_act < num_sta-1))
	{	// movement action
		std::cout << "Got a movement action : " << best_act << std::endl;
		
		double rad = 1.0;	//note: radius is kept at 1m
		goal_position = (rad * plan_vps.row(best_act).transpose()) + target;		
		goal_orientation = misc::target2quat( goal_position, target );
		st = MOVE;
	}
	else if ((best_act >= num_sta-1) && (best_act < num_act))
	{	// decision action
		best_act = (best_act - (num_sta-1));
		std::cout << "Got a decision action : " << best_act << std::endl;
		std::cout << "Object is : " << hyp_list[best_act].name << std::endl;
		
		st = DONE;
	}
	else
		throw std::runtime_error("Invalid action received...\n");
	
		
	return std::make_pair(st, best_act);
}

// returns true if done and false otherwise. The optimal action is returned as second
std::pair<ss_decider::Decider_State,int> 
ss_decider::decider_spin_v2( Eigen::Vector3d const& position, Eigen::Vector3d const& target,
					  	  pcl::PointCloud<pcl::PointXYZ>::Ptr cld_ptr, 
					  	  boost::shared_ptr<obj> const& curr_OBJ,
					  	  std::vector<double>::iterator curr_bel_start, 
					  	  std::vector<double>::iterator curr_bel_end,
					  	  Eigen::Vector3d & goal_position,  Eigen::Vector4d & goal_orientation )
{	
	// determine the closest point from the plan view points
	int vp = misc::find_closest_viewpoint( target, plan_vps, position, goal_position, goal_orientation );
	
	std::cout << "vp = " << vp << std::endl;
	
	// Get the observation from the vocabulary tree if the test surface contains points
	//if( cld_ptr -> size() < 50 ){
	if( true ){
		ROS_WARN("The object is completely occluded! Skipping observation!");
	}
	else
	{
		/*
		// Save the test surface for viewing purposes (in ASCII)
		ROS_WARN("Current vp %d", vp);
		pcl::PCDWriter writer;
		writer.writeASCII( "/media/natanaso/Data/Stuff/Research/git/icra_2013/ns_shared/test_vp_"
								 + boost::lexical_cast<std::string>(vp) + ".pcd", *cld_ptr );
		*/
	
		int obs = get_observation( cld_ptr );
		//int obs = get_fake_observation( vp );
	
		std::cout << "obs = " << obs << std::endl;
	
		// update the belief
		double denum = 0.0;
	
		{
			int h = 0;
			for( std::vector<double>::iterator iter = curr_bel_start;
				 iter != curr_bel_end; ++iter, ++h)
			{
				(*iter) = oMap[obs][vp][h] * (*iter);
				denum += (*iter);
			}
		}
	
		// normalize
		for( std::vector<double>::iterator iter = curr_bel_start;
			 iter != curr_bel_end; ++iter)
		{
			(*iter) = (*iter) / denum;
		}
	}
	
	// Use the policy to get the best action
	int best_act = -1;
	//best_act = su.getBestAction( vp, curr_bel_start, curr_bel_end );
	
	std::vector<int> vp_seq;
	//EJS
	//ejs_planner::get_EJS_sequence( 0.4, vp, oMap, dMap, curr_bel_start, curr_bel_end, vp_seq );
	//GMI
	gmi_planner::get_GMI_sequence( 0.4, vp, oMap, dMap, curr_bel_start, curr_bel_end, vp_seq );
	
	//RANDOM
	//cvp_planner::get_CVP_sequence( 0.4, vp, dMap, curr_bel_start, curr_bel_end, vp_seq );
	
	/* 
	std::cout << "vp seq: "<<std::endl;
	for( std::vector<int>::iterator it1 = vp_seq.begin(); it1 != vp_seq.end(); ++it1)
		std::cout << *it1 << " ";
	std::cout << std::endl;
	*/

	for(vp = 0 ; vp<num_sta-1; ++vp)
		if((vp_seq[vp] >= num_sta-1 ) || (!curr_OBJ->is_vp_visited(vp_seq[vp])))
		{
			best_act = vp_seq[vp];		
			break;
		}
	if(best_act == -1)
		best_act = misc::uniform_int( 0, num_sta-2 );
	
	
	/* 
	// Static
	{
		double max_bel = 0.0;
		int h = 0;
		for( std::vector<double>::iterator iter = curr_bel_start;
				 iter != curr_bel_end; ++iter, ++h)
		{
			if( *iter > max_bel )
			{
				max_bel = *iter;
				best_act = num_sta-1 + h;
			}
		}
	}
	*/
		
	std::cout << "act = " << best_act << std::endl;
	
	// calculate the goal pose
	ss_decider::Decider_State st;
	if((best_act >=0) && (best_act < num_sta-1))
	{	// movement action
		std::cout << "Got a movement action : " << best_act << std::endl;
		
		double rad = 1.0;	//note: radius is kept at 1m
		goal_position = (rad * plan_vps.row(best_act).transpose()) + target;		
		goal_orientation = misc::target2quat( goal_position, target );
		st = MOVE;
	}
	else if ((best_act >= num_sta-1) && (best_act < num_act))
	{	// decision action
		best_act = (best_act - (num_sta-1));
		std::cout << "Got a decision action : " << best_act << std::endl;
		st = DONE;
	}
	else
		throw std::runtime_error("Invalid action received...\n");
	
		
	return std::make_pair(st, best_act);
}


// Global variables: oMap, vtu, neg_obj_set, obj_id_map
int
ss_decider::get_observation( pcl::PointCloud<pcl::PointXYZ>::Ptr cld_ptr )
{
	// get the score from the vocabulary tree
	std::pair<float,std::string> vp_score = vtu.top_match( cld_ptr->getMatrixXfMap() );
	
	ROS_WARN("The top match is %s!", vp_score.second.c_str());
	
	// convert the match_name to a number representing the obs
	size_t pcd_pos = (vp_score.second).find_last_of(".pcd");
	size_t underscore_pos = (vp_score.second).find_last_of("_");
	size_t slash_pos = (vp_score.second).find_last_of("/");
	if( (pcd_pos == std::string::npos) || (underscore_pos == std::string::npos)
			|| (slash_pos == std::string::npos) )
		throw std::runtime_error("Unknown match name...\n");
	
	std::string tmp( (vp_score.second).substr(slash_pos+1, underscore_pos-slash_pos-1 ) );
	
	int num_obj = obj_set.size();
	int obs;
	if( null_obj_set.count( tmp ) > 0)
		obs = num_obs-(num_obj-obj_id_map[tmp]);
	else
		obs = obj_id_map[tmp] * tree_vps.rows()
			+ atoi( (vp_score.second).substr(underscore_pos+1, pcd_pos-underscore_pos-1).c_str() );

	return obs;	
}

int
ss_decider::get_fake_observation( int vp )
{
	int obs = -1;
	
	// uniform random observation (should guess H0)
	obs = misc::uniform_int( 0, num_obs-1 );

	
	/*
	// The most likely observations for H3 (should guess H3)
	double max_obs, sec_obs;
	int max_id;
	if (oMap[0][vp][2] < oMap[1][vp][2])
	{
		max_obs = oMap[1][vp][2];
		sec_obs = oMap[0][vp][2];
		max_id = 1;
		obs = 0;
	}
	else
	{
		max_obs = oMap[0][vp][2];
		sec_obs = oMap[1][vp][2];
		max_id = 0;
		obs = 1;
	}

	
	for (int b = 2; b < num_obs; ++b)
	{
		if( oMap[b][vp][2] > max_obs )
		{
			sec_obs = max_obs;
			obs = max_id;
			
			max_obs = oMap[b][vp][2];
			max_id = b;
		}
		else if( oMap[b][vp][2] > sec_obs )
		{
			sec_obs = oMap[b][vp][2];
			obs = b;
		}
	}
	int dec = misc::uniform_int( 0, 1 );
	if(dec == 1)
		obs = max_id;
		
	*/
	return obs;
}


//****************************************************************
// READING FUNCTIONS
void
ss_decider::read_obj()
{
	std::ifstream in_file;
	io_utils::open_in_file( in_file, obj_list_path );
	
	if( !in_file )
		throw std::runtime_error("[decider] Cannot open object list file...\n");
	
	// Read the object names from the file and store them
	std::string obj_name;
	while ( in_file >> obj_name )
	{
		obj_set.insert( obj_name );
	}
	
	in_file.close();
}

void 
ss_decider::read_hyp()
{
	std::ifstream in_file;
	io_utils::open_in_file( in_file, hyp_path );
	
	if( !in_file )
		throw std::runtime_error("[decider] Cannot open hypotheses file...\n");
	
	std::string name;
	double roll;
	double pitch;
	double yaw;
	std::set<std::string> pos_obj_set;
	while( ( in_file >> name )&&( in_file >> roll )&&
		   ( in_file >> pitch )&&( in_file >> yaw ) )
	{
		hyp_list.push_back( Hypothesis( true, name, misc::deg2rad(roll), 
							misc::deg2rad(pitch), misc::deg2rad(yaw) ) );
		
		pos_obj_set.insert( name );
		
		if( obj_numhyp_map.count(name) == 0)
			obj_numhyp_map[name] = 1;
		else
			obj_numhyp_map[name] = obj_numhyp_map[name] + 1;
	}
	
	in_file.close();
	
	// get the set of null objects
	std::set_difference( obj_set.begin(), obj_set.end(), 
						 pos_obj_set.begin(), pos_obj_set.end(), 
						 std::inserter(null_obj_set, null_obj_set.end()) );
						 
	for(std::set<std::string>::iterator it = null_obj_set.begin();
		it != null_obj_set.end(); ++it)
	{
		hyp_list.push_back( Hypothesis( false, *it, 0.0, 0.0, 0.0) );
		obj_numhyp_map[*it] = 1;
	}
	
	num_hid = hyp_list.size();
}


void
ss_decider::read_vps()
{

	io_utils::file2matrix( tree_vps_path, tree_vps, 3 );
	io_utils::file2matrix( omap_vps_path, omap_vps, 3 );
	io_utils::file2matrix( plan_vps_path, plan_vps, 3 );
	
	num_sta = plan_vps.rows() + 1; // add 1 for the dummy terminal state
	num_act = (num_sta-1) + num_hid;
}

void
ss_decider::fill_obj_id_map()
{
	// Determine the number of observations
	int num_obj = obj_set.size();
	int num_null_obj = null_obj_set.size();
	
	// positive objects have scores for each view
	// negative objects have a single score
	num_obs = (num_obj - num_null_obj) * tree_vps.rows() + num_null_obj;
		
	// associate ids with each object
	int fi=0, bi=num_obj-1;
	for(std::set<std::string>::iterator iter = obj_set.begin();
		iter != obj_set.end(); ++iter)
	{
		if( null_obj_set.find( *iter ) == null_obj_set.end() )
			obj_id_map[ *iter ] = fi++;
		else
			obj_id_map[ *iter ] = bi--;
	}
}


void
ss_decider::read_omap()
{
	std::ifstream in_file;
	io_utils::open_in_file( in_file, omap_file_path );
	
	if( !in_file )
		throw std::runtime_error("[decider] Cannot open oMap file...\n");
		
	oMap.resize(boost::extents[num_obs][num_sta][num_hid]);
	for(int hid = 0; hid < num_hid; ++hid)
		for (int sta = 0; sta < num_sta; ++sta)
			for (int obs = 0; obs < num_obs; ++obs)
				in_file >> oMap[obs][sta][hid];

	in_file.close();
	
	// compute the denominators
	typedef boost::multi_array_types::index_range range;
	boost::multi_array<double,2> denum(boost::extents[num_sta][num_hid]);
	for(int hid = 0; hid < num_hid; ++hid)
		for (int sta = 0; sta < num_sta; ++sta)
		{
			if( sta < num_sta - 1)
			{
				// find the sum of the observations
				boost::multi_array<double,3>::array_view<1>::type obs_vec = 
						oMap[ boost::indices[range()][sta][hid] ];
			
				denum[sta][hid] = std::accumulate(obs_vec.begin(), obs_vec.end(), 0.0);
			}
			else
				denum[sta][hid] = num_obs;
		}
	
	// normalize
	for (int obs = 0; obs < num_obs; ++obs)
		for (int sta = 0; sta < num_sta; ++sta)
			for(int hid = 0; hid < num_hid; ++hid)
				oMap[obs][sta][hid] = oMap[obs][sta][hid] / denum[sta][hid];
	
	/*			
	// display
	for(int hid = 0; hid < num_hid; ++hid)
	{
		for (int sta = 0; sta < num_sta; ++sta)
		{
			for (int obs = 0; obs < num_obs; ++obs)
				std::cout << oMap[obs][sta][hid] << " ";

			std::cout << std::endl;
		}
		std::cout << std::endl << std::endl << std::endl;
	}
	*/
}

void
ss_decider::read_dmap()
{
	std::ifstream in_file;
	io_utils::open_in_file( in_file, cmap_file_path );
	
	if( !in_file )
		throw std::runtime_error("[decider] Cannot open cMap file...\n");	
		
	int num_vp = num_sta - 1;
	dMap.resize(boost::extents[num_vp][num_vp]);
	
	double tmp;
	for (int act = 0; act < num_vp; ++act)
		for (int sta = 0; sta < num_sta; ++sta)
		{
			if( sta < num_vp) 
				in_file >> dMap[sta][act];
			else
				in_file >> tmp;
		}
	in_file.close();
	
	/*			
	// display
	for (int vp = 0; vp < num_vp; ++vp)
	{
		for (int nvp = 0; nvp < num_vp; ++nvp)
			std::cout << dMap[vp][nvp] << " ";
		std::cout << std::endl;
	}
	*/	
	
}

void
ss_decider::set_prior()
{
	int num_obj = obj_set.size();
	int num_pos_hyp = num_hid - null_obj_set.size();
	
	double scale = 7.5;  // add more weight on the positive hypotheses
								// defines the deviation from the uniform distribution
	init_bel.resize(num_hid);
	
	for(int hid = 0 ; hid < num_hid; ++hid)
	{
		if( hid < num_pos_hyp )
		{
			init_bel[hid] = 0.031;
		}
		else
			init_bel[hid] = 0.0102105263157895;
		/*
		if( hid < num_pos_hyp )
		{
			init_bel[hid] = scale * 1.0 / num_obj / obj_numhyp_map[ hyp_list[hid].name ];
		}
		else
			init_bel[hid] = (num_obj - scale) / (num_obj - 1) * 1.0 / num_obj;
		*/
		// display
		std::cout << init_bel[hid] << " ";
	}
	std::cout << std::endl;
}

//****************************************************************
// WRITING FUNCTIONS

void 
ss_decider::write_fmap()
{
	// Initialize fMap
	fMap.resize(boost::extents[num_act][num_sta]);
	for (int act = 0; act < num_act; ++act)
		for (int sta = 0; sta < num_sta; ++sta)
		{
			// always go to sink state after a decision
			// and if in sink state stay there
			if((act >= (num_sta-1))||(sta == (num_sta - 1)))
				fMap[act][sta] = (num_sta - 1);
			else
				fMap[act][sta] = act;
		}
		
	// Write fMap to file
	std::ofstream out_file;
	io_utils::open_out_file( out_file, save_dir + "/fMap.txt" );
	for (int act = 0; act < num_act; ++act)
	{
		for (int sta = 0; sta < num_sta; ++sta)
			out_file << fMap[act][sta] << " ";

		out_file << std::endl;
	}
	
	out_file.close();
}

void 
ss_decider::write_cmap()
{	

	// terminal costs for false positives and false negatives
	boost::multi_array<double,2> L(boost::extents[num_hid][num_hid]);
	int num_pos_hyp = num_hid - null_obj_set.size();
	// a is guess, b is correct
	for(int a = 0; a<num_hid; ++a)
		for(int b = 0; b<num_hid; ++b)
		{
			if( (b < num_pos_hyp) && (a != b) )
			{
				if( a < num_pos_hyp)
					L[a][b] = 130;	// Wrong positive hyp (orientation) 
				else
					L[a][b] = 130;	// Missed a positive
			}
			else if( (b >= num_pos_hyp) && (a < num_pos_hyp) )
				L[a][b] = 130;	// False alarm
			else
				L[a][b] = 0;
			
			/*
			if(a == b)
				L[a][b] = 0;
			else if((a < num_pos_hyp) && (b < num_pos_hyp))
				L[a][b] = 30;	//TODO: set penalties well
			else
				L[a][b] = 30;
			*/
		}

	double msrmnt_cost = 1; //TODO: set a good measurement cost
 	int next_idx = 0;
 	double dist = 0.0;

	// Initialize cMap
	cMap.resize(boost::extents[num_act][num_sta][num_hid]);

 	for (int act = 0; act < num_act; ++act)
 		for (int sta = 0; sta < num_sta; ++sta)
 			for(int hid = 0; hid < num_hid; ++hid)
 			{
 				if(sta == (num_sta-1))
 					cMap[act][sta][hid] = 0.0;
 				else if(act >= (num_sta-1))	// decison action
 					cMap[act][sta][hid] = L[(act - num_sta + 1)][hid];
 				else{
 					next_idx = fMap[act][sta];
 					// Check if the action is allowed!
 					if (next_idx != sta)
 					{
 						
 						dist = misc::great_circle_dist( plan_vps.row(sta).transpose(), 
 														plan_vps.row(next_idx).transpose(),
 														Eigen::Vector3d(0,0,0) );

 						cMap[act][sta][hid] = msrmnt_cost + dist;
 					
 					}else{
 						// discourage staying
 						cMap[act][sta][hid] = msrmnt_cost + 2.0;// + 0.6; //+ std::numeric_limits<double>::infinity();
 					}
 					
 				}
 			}
	
	// Write cMap to file
	std::ofstream out_file;
	io_utils::open_out_file( out_file, save_dir + "/cMap.txt" );
	for(int hid = 0; hid < num_hid; ++hid)
	{
		for (int act = 0; act < num_act; ++act)
		{
			for (int sta = 0; sta < num_sta; ++sta)
				out_file << cMap[act][sta][hid] << " ";

			out_file << std::endl;
		}
		out_file << std::endl << std::endl << std::endl;
	}
	
	out_file.close();
	
	
	// Get dMap from cMap
	int num_vp = num_sta - 1;
	dMap.resize(boost::extents[num_vp][num_vp]);
	for( int act = 0; act < num_vp; ++act)
		for( int sta = 0; sta < num_vp; ++sta )
			dMap[sta][act] = cMap[act][sta][0];
}

void 
ss_decider::write_omap()
{
	
	// read the individual object omap files
	// Store in obj_id x omap_vp x score matrix
	int num_obj = obj_set.size();
	int num_null_obj = null_obj_set.size();
	int num_ovps = omap_vps.rows();
	boost::multi_array<int,3> id_x_ovp_x_score(boost::extents[num_obj][num_ovps][num_obs]);
	
	for(std::set<std::string>::iterator iter = obj_set.begin();
		iter != obj_set.end(); ++iter)
	{
		read_omap_a(*iter, id_x_ovp_x_score);
	}

	// Write id_x_ovp_x_score to file
	std::ofstream out_file;
	io_utils::open_out_file( out_file, save_dir + "/id_x_ovp_x_score.txt" );
	for(int ob = 0; ob < num_obj; ++ob)
	{
		for (int ovp = 0; ovp < num_ovps; ++ovp)
		{
			for (int obs = 0; obs < num_obs; ++obs)
				out_file << id_x_ovp_x_score[ob][ovp][obs] << " ";

			out_file << std::endl;
		}
		out_file << std::endl << std::endl << std::endl;
	}


	// Populate oMap
	typedef boost::multi_array_types::index_range range;
	oMap.resize(boost::extents[num_obs][num_sta][num_hid]);
	
	// populate the positive hypotheses
	Eigen::Vector3d position_obj_frame;
	int num_pos_hyp = num_hid - null_obj_set.size();
	for(int hid = 0; hid < num_pos_hyp; ++hid)
	{	
		// wRo = rotation from object to world frame!
		int obj_id = obj_id_map[hyp_list[hid].name];
		Eigen::Matrix<double,3,3> wRo = misc::rotz(hyp_list[hid].yaw) * 
									  misc::roty(hyp_list[hid].pitch) * 
									  misc::rotx(hyp_list[hid].roll);
		
		for (int sta = 0; sta < num_sta; ++sta)
		{
			if( sta < num_sta - 1)
			{
				// rotate the state by the hypothesis
				Eigen::Vector3d position_world_frame = plan_vps.row(sta).transpose();
				position_obj_frame = wRo.transpose() * position_world_frame;
			
				// find the closest omap_vp
				Eigen::Matrix<int,1,1> vp_id;
				Eigen::Matrix<double,1,1> vp_dist;
				misc::NNsearch_3D( omap_vps, position_obj_frame.transpose(),
									vp_id, vp_dist );
				int ovp = vp_id(0,0);
			
				// copy the corresponding observations from id_x_ovp_x_score
				oMap[ boost::indices[range()][sta][hid] ] = 
						id_x_ovp_x_score[ boost::indices[obj_id][ovp][range()] ];
			}
			else
			{
				for( int obs=0; obs < num_obs; ++obs)
					if( obs < num_obs - num_null_obj)
						oMap[obs][sta][hid] = 1;
					else
						oMap[obs][sta][hid] = 1;//tree_vps.rows();
			}
		}
	}
	
	// populate the null hypothesis
	// to get the null hypothesis sum id_x_ovp_x_score along the null objects and along all states to get a vector with counts for each obs
	int hid = num_hid - null_obj_set.size();
	for(std::set<std::string>::iterator it = null_obj_set.begin();
		it != null_obj_set.end(); ++it, ++hid)
	{
		int obj_id = obj_id_map[*it];
		for( int obs = 0; obs < num_obs; ++obs)
		{
			boost::multi_array<int,3>::array_view<1>::type ovp_vec = 
					id_x_ovp_x_score[ boost::indices[obj_id][range()][obs] ];
			int sum = std::accumulate(ovp_vec.begin(), ovp_vec.end(), 0);
			
			for (int sta = 0; sta < num_sta; ++sta)
			{
				if( sta < num_sta - 1)
					oMap[obs][sta][hid] = sum;
				else
				{
					if( obs < num_obs - num_null_obj)
						oMap[obs][sta][hid] = 1;
					else
						oMap[obs][sta][hid] = 1;//tree_vps.rows();
				}
			}
		}
	}
				
	// Write oMap to file
	io_utils::open_out_file( out_file, save_dir + "/oMap.txt" );
	for(int hid = 0; hid < num_hid; ++hid)
	{
		for (int sta = 0; sta < num_sta; ++sta)
		{
			for (int obs = 0; obs < num_obs; ++obs)
				out_file << oMap[obs][sta][hid] << " ";

			out_file << std::endl;
		}
		out_file << std::endl << std::endl << std::endl;
	}

	out_file.close();
	
	// compute the denominators
	boost::multi_array<double,2> denum(boost::extents[num_sta][num_hid]);
	for(int hid = 0; hid < num_hid; ++hid)
		for (int sta = 0; sta < num_sta; ++sta)
		{
			// find the sum of the observations
			boost::multi_array<double,3>::array_view<1>::type obs_vec = 
					oMap[ boost::indices[range()][sta][hid] ];
	
			denum[sta][hid] = std::accumulate(obs_vec.begin(), obs_vec.end(), 0.0);
				
			/*	
			if( sta < num_sta - 1)
			{
			}
			else
				denum[sta][hid] = num_obj*tree_vps.rows();
			*/
		}
		
	// normalize
	for (int obs = 0; obs < num_obs; ++obs)
		for (int sta = 0; sta < num_sta; ++sta)
			for(int hid = 0; hid < num_hid; ++hid)
				oMap[obs][sta][hid] = oMap[obs][sta][hid] / denum[sta][hid];
}

void
ss_decider::read_omap_a( std::string obj_name, 
						 boost::multi_array<int,3> &id_x_ovp_x_score)
{
	//std::cout << "obj_name = " << obj_name << std::endl;
				   
	std::ifstream in_file;
	std::string omap_file_name(omap_dir + "/oMap_" + obj_name + "_combined.txt");
	io_utils::open_in_file( in_file, omap_file_name );
	
	if( !in_file ){
		std::cout << "File " << omap_file_name << " cannot be opened!" << std::endl;
		throw std::runtime_error("[decider] Cannot open hypotheses file...\n");
	}
	
	
	
	// Initial observation weight
	int num_obj = obj_set.size();
	int num_null_obj = null_obj_set.size();
	int num_ovps = omap_vps.rows();
	int obj_id = obj_id_map[obj_name];
	
	//std::fill( id_x_ovp_x_score.data(), 
	//		   id_x_ovp_x_score.data() + id_x_ovp_x_score.num_elements(), 3 );
	for (int ovp = 0; ovp < num_ovps; ++ovp)
		for (int obs = 0; obs < num_obs; ++obs)
		{
				if( obs < num_obs - num_null_obj)
				{
					if( ( obs >= obj_id * tree_vps.rows() ) && ( obs < (obj_id+1)*tree_vps.rows() ) )
						id_x_ovp_x_score[obj_id][ovp][obs] = 10;	// put a higher weight on own observations
					else
						id_x_ovp_x_score[obj_id][ovp][obs] = 4;
				}else{
					if( obs == obj_id )
						id_x_ovp_x_score[obj_id][ovp][obs] = tree_vps.rows();
					else
						id_x_ovp_x_score[obj_id][ovp][obs] = tree_vps.rows();
				}
		}
			
	
	// weight used for the actual observations; should be large enough to make the initial weighting insignificant
	int obs_weight = 22;
	int ovp;
	int obs;
	std::string match_name;
	double match_score;
	
	while( ( in_file >> ovp )&&( in_file >> match_name )
			&&( in_file >> match_score) )
	{
		// conver the match_name to a number representing the obs
		size_t pcd_pos = match_name.find_last_of(".pcd");
		size_t underscore_pos = match_name.find_last_of("_");
		size_t slash_pos = match_name.find_last_of("/");
		if( (pcd_pos == std::string::npos) || (underscore_pos == std::string::npos)
				|| (slash_pos == std::string::npos) )
			throw std::runtime_error("Unknown match name...\n");
		
		std::string tmp( match_name.substr(slash_pos+1, underscore_pos-slash_pos-1 ) );
		
		
		if( null_obj_set.count( tmp ) > 0)
			obs = num_obs-(num_obj-obj_id_map[tmp]);
		else
			obs = obj_id_map[tmp] * tree_vps.rows()
				+ atoi( match_name.substr(underscore_pos+1, pcd_pos-underscore_pos-1).c_str() );
		
		//std::cout << ovp << " " << tmp << " " << obs << std::endl;
			
		// add obs_weight to the id_x_ovp_x_score
		id_x_ovp_x_score[obj_id][ovp][obs] += obs_weight;
	}
	
	in_file.close();
}


void
ss_decider::write_policy()
{
	// Sarsop parameters	
	double precision = 0.05;	// precision = difference between upper and lower bound in optimal value
	double time_limit = 900;	// time_limit = [sec] for anytime planning
	//sarsop_user su;
	
	int num_vp = 1;	//num_sta-1;
	for(int vp = 0; vp < num_vp; ++vp)
	{	
		// Compute the policy
		std::string model_path( save_dir + "/model_" + boost::lexical_cast<std::string>(vp) + ".pomdpx" );
		bool valid = su.modelToFile( fMap, cMap, oMap, model_path, vp, init_bel.begin() );
	
		if(!valid)
			throw std::runtime_error("Cannot write " +  model_path + "...\n");
	
		std::string policy_path( save_dir + "/plan_" + boost::lexical_cast<std::string>(vp) + ".policy" );
		valid = su.computePolicy(precision, time_limit, model_path, policy_path);
		if(!valid)
			throw std::runtime_error("Cannot compute " + policy_path + "...\n");
	}
}


