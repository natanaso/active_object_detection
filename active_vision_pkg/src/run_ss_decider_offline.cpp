#include <string>
#include <ros/package.h>
#include "ss_decider.hpp"

int ss_decider_offline(int argc, char **argv)
{
	std::string pkg_root_dir( ros::package::getPath("active_vision_pkg") );
	std::string vision_module_dir( ros::package::getPath("vision_module") );
	std::string planning_module_dir( ros::package::getPath("planning_module") );
	//planning_module_dir + "/appl-0.95/src/pomdpsol"
	
	ss_decider ssd;
	
	std::cout << "Starting offline computation..." << std::endl << std::endl;
	
	
	return ssd.start_offline( pkg_root_dir + "/data/obj_list.txt",
					 pkg_root_dir + "/data/hypotheses.txt",
					 vision_module_dir + "/data/tree_vps.txt",
					 vision_module_dir + "/data/omap_vps.txt",
					 pkg_root_dir + "/data/plan_vps.txt",
					 vision_module_dir + "/data/omap/combined",
					 pkg_root_dir + "/data/sarsop_data",
					 vision_module_dir + "/data",
				    vision_module_dir + "/../database/cloud_data" );
}

int main(int argc, char **argv)
{
	return ss_decider_offline(argc, argv);
}
