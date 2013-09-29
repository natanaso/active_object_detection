// Ros
#include <ros/package.h>

// Custom
#include "ss_decider.hpp"

int ss_decider_main(int argc, char **argv)
{
	//int num_hid = 7;			//4
	
	std::string pkg_root_dir( ros::package::getPath("active_vision_pkg") );
	std::string vision_module_dir( ros::package::getPath("vision_module") );
	std::string planning_module_dir( ros::package::getPath("planning_module") );
	
	//size_t last_slash_pos = vision_module_dir.find_last_of("/vision_module");
	//std::string main_dir ( vision_module_dir.substr( 0, last_slash_pos) );
	//planning_module_dir + "/appl-0.95/src/pomdpsol",
	ss_decider sd;
	
	
	sd.start_online( pkg_root_dir + "/data/obj_list.txt",
				   pkg_root_dir + "/data/hypotheses.txt",
				   vision_module_dir + "/data/tree_vps.txt",
				   vision_module_dir + "/data/omap_vps.txt",
				   pkg_root_dir + "/data/plan_vps.txt",
				   pkg_root_dir + "/data/sarsop_data/plan.policy",
				   pkg_root_dir + "/data/sarsop_data/oMap.txt",
				   pkg_root_dir + "/data/sarsop_data/cMap.txt",
				   vision_module_dir + "/data",
				   vision_module_dir + "/../database/cloud_data" );
	
	Eigen::Vector3d target(0,0,0);
	Eigen::Vector3d position(1,2,5);
	
	Eigen::Vector3d goal_position;
	Eigen::Vector4d goal_orientation;
	
	sd.initial_alignment( position, target, goal_position, goal_orientation );
	
	std::cout << "The requested pose is:" << std::endl;
	std::cout << goal_position.transpose() << std::endl;
	std::cout << goal_orientation.transpose() << std::endl;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cld_ptr( new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<double> bel;
	const double arr[] = {0.46, 0.09, 0.09, 0.09, 0.09, 0.09, 0.09};
	//{0.34, 0.11, 0.11, 0.11, 0.11, 0.11, 0.11};
	//{0.5, 0.0833, 0.0833, 0.0833, 0.0833, 0.0834, 0.0834};
	//{0.28571, 0.119048, 0.119048, 0.119048, 0.119048, 0.119048, 0.11905}; 
	//{3.0/5.0, 1.0/10.0, 1.0/10.0, 1.0/5.0};
	bel.assign(arr, arr + sizeof(arr) / sizeof(arr[0]));
	
	// seed the random generator
	srand((unsigned)time(NULL));
	
	// make a done action pair
	std::pair<ss_decider::Decider_State,int> da(std::make_pair(ss_decider::MOVE,-1));
	while( da.first != ss_decider::DONE )
	{
		position = goal_position;
		da = sd.decider_spin( position, target, cld_ptr, bel.begin(), bel.end(), goal_position, goal_orientation );
		
		std::cout << "The updated belief is:" << std::endl;
		for( std::vector<double>::iterator it = bel.begin();
			it != bel.end(); ++it)
			std::cout << *it << " ";
		std::cout << std::endl << std::endl;
		 
		std::cout << "The requested pose is:" << std::endl;
		std::cout << goal_position.transpose() << std::endl;
		std::cout << goal_orientation.transpose() << std::endl;
		std::cout << std::endl;
	}
	
	
	return 0;
}

int main(int argc, char **argv)
{
	return ss_decider_main(argc, argv);
}
