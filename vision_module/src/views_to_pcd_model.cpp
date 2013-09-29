
#include <string>
#include <stdexcept>

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// ros
#include <ros/package.h>
#include <pcl_ros/publisher.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

// boost
#include <boost/lexical_cast.hpp>

// custom
#include <io_utils.hpp>
#include <misc.hpp>

int
vpm(int argc, char **argv)
{
	if( argc < 2 )
		throw std::runtime_error("Provide model name as second argument");
	std::string obj_name(argv[1]);
	std::string vision_module_dir( ros::package::getPath("vision_module") );
	std::string tree_vps_path( vision_module_dir + "/data/tree_vps.txt");
	std::string views_path( vision_module_dir + "/../database/cloud_data/"
														   + obj_name + "_views" );
														   
														   
	// Read the tree vps file
	Eigen::MatrixXd tree_vps;
	io_utils::file2matrix( tree_vps_path, tree_vps, 3 );
	int num_vps = tree_vps.rows();
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_model_ptr( new pcl::PointCloud<pcl::PointXYZ>);
	for(int vp = 0; vp < num_vps; ++vp)
	{
		// load the view
		pcl::PointCloud<pcl::PointXYZ>::Ptr view_ptr( new pcl::PointCloud<pcl::PointXYZ>);
		std::string view_path( views_path + "/" + obj_name + "_" 
													 + boost::lexical_cast<std::string>(vp) +".pcd" );
		pcl::io::loadPCDFile<pcl::PointXYZ> ( view_path, *view_ptr );
		
		// rotate to world frame
		Eigen::Vector3d camera_position = tree_vps.row(vp).transpose();
		Eigen::Vector4d camera_orientation = misc::target2quat( camera_position, 
																				  Eigen::Vector3d(0,0,0) );
		
		tf::Transform map_T_snsr;
		map_T_snsr.setOrigin( tf::Vector3(camera_position.x(), 
													 camera_position.y(), 
													 camera_position.z()) );
		map_T_snsr.setRotation( tf::Quaternion( camera_orientation.x(), 
															 camera_orientation.y(), 
															 camera_orientation.z(), 
															 camera_orientation.w()) );
		
		tf::Transform snsr_T_optical;
		snsr_T_optical.setOrigin( tf::Vector3( 0.0, 0.0, 0.0 ) );
		snsr_T_optical.setRotation( tf::Quaternion( -0.5, 0.5, -0.5, 0.5 ) );
		
		tf::Transform map_T_optical = map_T_snsr * snsr_T_optical;

		pcl_ros::transformPointCloud( *view_ptr, *view_ptr, map_T_optical);
		
		// add to pcd_model
		*pcd_model_ptr +=  *view_ptr;
	}
	
	// Save
	pcl::PCDWriter writer;
	writer.writeASCII( vision_module_dir + "../database/pcd_models/" + obj_name + "_complete.pcd", *pcd_model_ptr );
	
	return 0;
}

int
main(int argc, char **argv)
{
	return vpm(argc, argv);
}
