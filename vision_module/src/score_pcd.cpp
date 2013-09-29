// Standard
#include <iostream>
#include <fstream>
#include <string>

// Ros
#include <ros/package.h>

// Pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>

// Boost
//#include <boost/lexical_cast.hpp>

// Custom
#include "io_utils.hpp"
#include "vision_module/vtree_user.hpp"


void show_help(char *cmd_name)
{
	std::cout << "Usage " << cmd_name << " [options] file.pcd" << std::endl;
	std::cout << " * where options are:" << std::endl;
	std::cout << "     -sp path : (save_path) place the extracted pcd file in the vocabulary tree and record the matches in the file given in path" << std::endl;
	std::cout << std::endl;
}

int score_pcd_main(int argc, char **argv)
{
	if ( (pcl::console::find_switch (argc, argv, "-h")) || (argc < 2) )
	{
		show_help (argv[0]);
		exit (0);
	}
	
	// parse the command line
	std::vector<int> pcd_idx = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
	std::string pcd_file_path( argv[pcd_idx[0]] );
	
	std::string score_save_path("./");
	pcl::console::parse_argument (argc, argv, "-sp", score_save_path);
	
	
	// Load the pcd file
	pcl::PointCloud<pcl::PointXYZ>::Ptr cld_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path.c_str(), *cld_ptr);

	// Create and initialize an nbv tree
	std::string vision_module_path(ros::package::getPath("vision_module"));
	vtree_user vt("/load", vision_module_path + "/data", vision_module_path + "/../database/cloud_data");
	vt.start();
	
	/*
	std::string node_id("node_"+boost::lexical_cast<std::string>(time(NULL)));
	std::cout << "node_id = " << node_id << std::endl;
	ros::init(argc, argv, node_id);
	ros::NodeHandle node_handle("~");
	NBVTree nbv_tree(node_handle);
	nbv_tree.start();
	*/
	
	// Get the score list
	ROS_INFO("Getting matches...");
	std::vector<std::pair<float,std::string> > match_names;
	vt.match_list( cld_ptr->getMatrixXfMap(), match_names, 100 );
		
	// write to file
	std::ofstream outstr;
	io_utils::open_out_file( outstr, score_save_path );

	if( !outstr )
		throw std::runtime_error("Cannot open scores save file...\n");

	for( size_t i = 0; i < match_names.size(); ++i)
	{
		outstr << match_names[i].first << " " << match_names[i].second << std::endl;
		//outstr << cluster_match_names[i].first << " " << cluster_match_names[i].second << std::endl;
	}

	outstr.close();
	
	return 0;
}


int main(int argc, char **argv)
{
	return score_pcd_main( argc, argv);
}
