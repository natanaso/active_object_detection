
// Standard
#include <iostream>
#include <fstream>
#include <string>

// Ros
#include "ros/package.h"

// Pcl
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>

// Custom
#include "io_utils.hpp"
#include "pcd_utils.hpp"
#include "vision_module/vtree_user.hpp"
#include "virtual_kinect_pkg/vkin_offline.hpp"

/*
std::ofstream& open_out_file( std::ofstream &fsr, const std::string &file)
{
	// put file in valid state first
	fsr.close();	// close in case it was already open
	fsr.clear();	// clear any existing errors
	fsr.open( file.c_str() );
	return fsr;
}

void voxel_grid_subsample( const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cld_in, 
					  pcl::PointCloud<pcl::PointXYZ>::Ptr & cld_out,
					  float cell_size )
{
	pcl::VoxelGrid< pcl::PointXYZ > sor;
	sor.setInputCloud (cld_in);
	sor.setLeafSize (cell_size, cell_size, cell_size);	//leaf size in meters
	sor.filter ( *cld_out );
	
}

void rnd_subsample( const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cld_in, 
					  pcl::PointCloud<pcl::PointXYZ>::Ptr & cld_out,
					  int size )
{
	pcl::RandomSample<pcl::PointXYZ> sor;
	sor.setInputCloud( cld_in );
	sor.setSample( size );
	sor.filter( *cld_out );
}

*/
void show_help(char *cmd_name)
{
	std::cout << "Usage " << cmd_name << " [options] model.ply extracted.pcd" << std::endl;
	std::cout << " * where options are:" << std::endl;
	std::cout << "     -coord a : save the dataset in object coordinates (a = 0) or optical (z = forward, x = right, y = down) camera coordinates (a = 1) or standard (x = forward, y = left, z = up) camera coordinates (a = 2)" << std::endl;
	std::cout << "     -vp x,y,z : set the camera viewpoint from where the acquisition will take place" << std::endl;
	std::cout << "     -tp x,y,z : the target point that the camera should look at" << std::endl;
	std::cout << "     -add_noise a : add gausian noise (a = 1) or keep the cloud noiseless (a = 0)" << std::endl;
	std::cout << "     -org a : create an organized, grid-like point cloud of width x height (a = 1), or keep it unorganized with height = 1 (a = 0)" << std::endl;
	std::cout << "     -save_scores path : place the extracted pcd file in the vocabulary tree and record the matches in the file given in path" << std::endl;
	std::cout << "     -ascii a: save the pcd file in binary compressed (a = 0) or ascii format (a = 1)" << std::endl;
	std::cout << "     -subsamp a : subsample the cloud using a voxelgrid with leaf size a (float)" << std::endl;
	std::cout << std::endl;
}


int pcd_from_ply_main(int argc, char **argv)
{

	if (pcl::console::find_switch (argc, argv, "-h"))
	{
		show_help (argv[0]);
		exit (0);
	}
  		
	// parse the command line
	std::vector<int> ply_idx = pcl::console::parse_file_extension_argument (argc, argv, ".ply");
	std::string ply_file_path( argv[ply_idx[0]] );
	
	std::vector<int> pcd_idx = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
	std::string pcd_file_path( argv[pcd_idx[0]] );
	
	// default options
	double vpx = 1.0, vpy = 0.0, vpz = 0.0;
	double tpx = 0.0, tpy = 0.0, tpz = 0.0;
	int coord = 1; 
	bool org = false, add_noise = false, ascii = false;
	double sub_leaf_size = -1.0;
	
	// command line options
	pcl::console::parse_3x_arguments (argc, argv, "-vp", vpx, vpy, vpz);
	pcl::console::parse_3x_arguments (argc, argv, "-tp", tpx, tpy, tpz);
	pcl::console::parse_argument (argc, argv, "-coord", coord);
	pcl::console::parse_argument (argc, argv, "-add_noise", add_noise);
	pcl::console::parse_argument (argc, argv, "-org", org);
	pcl::console::parse_argument (argc, argv, "-ascii", ascii);
	pcl::console::parse_argument (argc, argv, "-subsamp", sub_leaf_size);
	
	// Construct and initialize the virtual kinect
	vkin_offline vko( Eigen::Vector3d(0,0,0), Eigen::Vector4d(0,0,0,1), coord, org, add_noise );
	vko.init_vkin( ply_file_path );
	
	// get a noiseless cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cld_ptr( vko.sense( Eigen::Vector3d(vpx, vpy, vpz), 
															Eigen::Vector3d(tpx, tpy, tpz) ) );
	
	
	if( sub_leaf_size > 0 )
		cld_ptr = pcd_utils::voxel_grid_subsample( cld_ptr, static_cast<float>(sub_leaf_size) );
	
															 
	// save the scan to file
	pcl::PCDWriter writer;
	if(ascii)
		writer.writeASCII( pcd_file_path.c_str(), *cld_ptr );
	else
    	writer.writeBinaryCompressed( pcd_file_path.c_str(), *cld_ptr );
    
    // Extract the tree scores if requested
    std::string score_save_path;
    if (pcl::console::parse_argument (argc, argv, "-save_scores", score_save_path) != -1)
    {
    	// Create and initialize an nbv tree
    	std::string vision_module_path(ros::package::getPath("vision_module"));
    	vtree_user vt("/load", vision_module_path + "/data", vision_module_path + "/../data/cloud_data");
		vt.start();
    	
    	/*
    	ros::init(argc, argv, "ply2pcd_node");
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
    }
    
    return 0;
}

int main(int argc, char **argv)
{
	return pcd_from_ply_main( argc, argv);
}
