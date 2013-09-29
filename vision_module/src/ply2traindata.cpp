// Standard
#include <fstream>
#include <stdexcept>

// Ros
#include "ros/package.h"

// Boost
#include <boost/filesystem.hpp>	//boost::filesystem::path(fpath).stem().string()

// Pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>

// Custom
#include "virtual_kinect_pkg/vkin_offline.hpp"


std::ifstream& open_file( std::ifstream &in, const std::string &file)
{
	// put file in valid state first
	in.close();	// close in case it was already open
	in.clear();	// clear any existing errors
	in.open( file.c_str() );
	return in;
}


void file2matrix( const std::string filename, Eigen::MatrixXd & mat )
{
	std::ifstream in;
	
	open_file( in, filename );
	
	if(!in)
		throw std::runtime_error("Cannot open view point positions file...\n");
		
	std::vector<double> data;
	double val;
	while ( in >> val ){
		data.push_back( val );
	}

	// copy the vector to an eigen matrix
	int cols = 3;
	int rows = data.size() / 3;
	mat.resize( rows, cols );
	std::copy( data.data(), data.data()+data.size(), mat.data() );
		
	for(int r = 0; r < rows; ++r)
		for(int c = 0; c < cols; ++c)
		{
			mat(r,c) = data[cols*r + c];
		} 

	in.close();
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

int ply2trainpcd(int argc, char **argv)
{
	if(argc < 3)
		throw std::runtime_error("A pcd directory path is required as a second argument...\n");
		
	if(argc < 2)
		throw std::runtime_error("A ply file is required as a first argument...\n");
	
	
	std::string pcd_dir_path( argv[2] );		
	std::string ply_file_path( argv[1] );
	std::string ply_file_name( boost::filesystem::path(ply_file_path).stem().string() );
	
	std::string vp_file_path(ros::package::getPath("vision_module"));
	vp_file_path.append("/data/tree_vps.txt");
	Eigen::MatrixXd vp_positions;
	
	file2matrix( vp_file_path, vp_positions );
	int num_vp = vp_positions.rows();
	int dim_vp = vp_positions.cols();
	
	// get a virtual kinect object
	int coord = 1; // 0 = obj coord, 1 = optical sensor coord, 2 = standard sensor coord
	bool add_noise = false;
	vkin_offline vko( Eigen::Vector3d(0,0,0), Eigen::Vector4d(0,0,0,1),
					  		coord, false, add_noise );
	
	vko.init_vkin( ply_file_path );
	
	Eigen::Vector3d target(0,0,0);
	for(int vp = 0; vp < num_vp; ++vp)
	{
		// get a noiseless scan in the sensor frame
		pcl::PointCloud<pcl::PointXYZ>::Ptr cld_ptr = vko.sense( vp_positions.row(vp).transpose(), 
																 target );
		/*
		// subsample the cloud if it is too large
		if(cld_ptr->size() > 5000)
			rnd_subsample( cld_ptr, cld_ptr, (cld_ptr->size() / 3) );
		*/
		
		// write the scan to disk
		std::stringstream ss;
		ss << pcd_dir_path << "/" << ply_file_name << "_" << vp << ".pcd";
		pcl::io::savePCDFileBinary( ss.str(), *cld_ptr );
	}
	
	return 0;
}


int main(int argc, char **argv)
{
	return ply2trainpcd(argc, argv);
}
