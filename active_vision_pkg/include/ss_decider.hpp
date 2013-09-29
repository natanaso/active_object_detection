#ifndef _SS_DECIDER_HPP_
#define _SS_DECIDER_HPP_

#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <map>


// Pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// Boost
#include <boost/multi_array.hpp>
#include <boost/lexical_cast.hpp>
#include <Eigen/Core>

// Custom
#include "obj.hpp"
#include "hypothesis.hpp"
#include <planning_module/sarsop_user.hpp>
#include <vision_module/vtree_user.hpp>



class ss_decider
{
public:
	enum Decider_State{ DONE, STAY, MOVE };
					 
private:
	int num_sta, num_hid, num_act, num_obs;
	
	// Object lists
	std::string obj_list_path;
	std::set<std::string> obj_set;
	std::map<std::string, int> obj_id_map;
	std::map<std::string, int> obj_numhyp_map;
	
	// Hypotheses
	std::string hyp_path;
	std::vector<Hypothesis> hyp_list;
	std::set<std::string> null_obj_set;

	// Viewpoints
	std::string tree_vps_path;
	Eigen::MatrixXd tree_vps;
	
	std::string omap_vps_path;
	Eigen::MatrixXd omap_vps;
	
	std::string plan_vps_path;
	Eigen::MatrixXd plan_vps;
		

	// oMap, cMap, fMap
	std::string omap_dir;
	std::string save_dir;
	
	boost::multi_array<int,2> fMap;
	boost::multi_array<double,3> cMap;
	boost::multi_array<double,2> dMap;
	boost::multi_array<double,3> oMap;
	
	// planning
	std::vector<double> init_bel;
	sarsop_user su;
	//std::string policy_file_path;
	std::string omap_file_path;
	std::string cmap_file_path;
			
	// vision
	vtree_user vtu;
		
public:
	ss_decider():
		vtu("/load", "", "")
	{}
	
	~ss_decider(){}
	
	int start_offline( std::string obj_list_path,
						    std::string hyp_path,
							 std::string tree_vps_path,
							 std::string omap_vps_path,
							 std::string plan_vps_path,
							 std::string omap_dir,
							 std::string save_dir,
							 std::string database_dir,
							 std::string clouds_dir );
							 
	int start_online( std::string obj_list_path,
							std::string hyp_path,
							std::string tree_vps_path,
							std::string omap_vps_path,
							std::string plan_vps_path,
	                  std::string save_dir,
							std::string omap_file_path,
							std::string cmap_file_path,
							std::string database_dir,
							std::string clouds_dir );
	
	int initial_alignment( Eigen::Vector3d const& position, Eigen::Vector3d const& target, 
					   		Eigen::Vector3d & goal_position,  Eigen::Vector4d & goal_orientation );
	/*
	 * Returns true if done and false otherwise. Returns the optimal action as second. 
	 * Upon decision this will correspond to the chosen hypothesis.
	 */			   		
	std::pair<ss_decider::Decider_State,int> 
	decider_spin( Eigen::Vector3d const& position, Eigen::Vector3d const& target,
				  	  pcl::PointCloud<pcl::PointXYZ>::Ptr cld_ptr, 
				  	  std::vector<double>::iterator curr_bel_start, 
				  	  std::vector<double>::iterator curr_bel_end,
				  	  Eigen::Vector3d & goal_position,  Eigen::Vector4d & goal_orientation );

	std::pair<ss_decider::Decider_State,int> 
	decider_spin( Eigen::Vector3d const& position, Eigen::Vector3d const& target,
				  	  pcl::PointCloud<pcl::PointXYZ>::Ptr cld_ptr, 
					  std::map< int, boost::shared_ptr<obj> > & obj_list, int obj_id,
				  	  Eigen::Vector3d & goal_position,  Eigen::Vector4d & goal_orientation );
					  	  
	/*
	 * Returns true if done and false otherwise. Returns the optimal action as second. 
	 * Upon decision this will correspond to the chosen hypothesis.
	 */			   		
	std::pair<ss_decider::Decider_State,int> 
	decider_spin_v2( Eigen::Vector3d const& position, Eigen::Vector3d const& target,
				  	  pcl::PointCloud<pcl::PointXYZ>::Ptr cld_ptr, 
				  	  boost::shared_ptr<obj> const& curr_OBJ, 
				  	  std::vector<double>::iterator curr_bel_start, 
				  	  std::vector<double>::iterator curr_bel_end,
				  	  Eigen::Vector3d & goal_position,  Eigen::Vector4d & goal_orientation );
				  	  
	/*
	 * Returns the hid-th hypothesis
	 */
	Hypothesis & get_hyp(int hid)
	{
		if( (hid > -1) && (hid < num_hid) )
			return hyp_list[hid];
		else
		{
			throw std::runtime_error("REQUESTED NONEXISTENT HYPOTHESIS!\n");
			return hyp_list[0];
		}
	}
	
	boost::multi_array<double,2> const& get_dMap()
	{
		return dMap;
	}
	
	
	void get_prior( std::vector<double>::iterator & bel_start,
					 	 std::vector<double>::iterator & bel_end )
	{
		if( !init_bel.empty() ){
			bel_start = init_bel.begin();
			bel_end = init_bel.end();
		}
	}
	
	int get_num_sta() const
	{
		return num_sta;
	}
	
	int get_num_hid() const
	{
		return num_hid;
	}
	
	int get_num_pos_hyp() const
	{
		return num_hid - null_obj_set.size();
	}
	
	
	Eigen::MatrixXd const& get_plan_vps() const
	{
		return plan_vps;
	}
	
private:
	int start( std::string obj_list_path,
				  std::string hyp_path,
				  std::string tree_vps_path,
				  std::string omap_vps_path,
				  std::string plan_vps_path );
				  
	// start offline
	void read_obj();
	void read_hyp();
	void read_vps();
	void fill_obj_id_map();
	void read_omap_a( std::string obj_name, boost::multi_array<int,3> &id_x_ovp_x_score);
	
	void set_prior();
	
	void write_fmap();
	void write_cmap();
	void write_omap();
	void write_policy();
				  
	// start online
	void read_omap();
	void read_dmap();
	
	int get_observation( pcl::PointCloud<pcl::PointXYZ>::Ptr cld_ptr );
	int get_fake_observation( int vp );
			
//**************************************
//*** Helper functions *****************
//**************************************
/*
public:


	// Scale and translate a sphere, which is centered at the origin
	static Eigen::MatrixXd transform_sphere( Eigen::MatrixXd const& base_sphere, Eigen::Vector3d const& cntr, double rad )
	{
		return (rad*base_sphere).rowwise() + cntr.transpose();
	}
*/
	
};

#endif
