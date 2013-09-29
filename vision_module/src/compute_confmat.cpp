// Standard
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "vision_module/vtree_user.hpp"
#include "virtual_kinect_pkg/vkin_offline.hpp"
#include "misc.hpp"
#include "io_utils.hpp"


void addNoise( Eigen::Vector3d const & position,
			   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
			   boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> > &generator)
{
	Eigen::Vector3d ray_vec;
	double noise_param = 0.005;		// TRUE IS 0.0005
	double noise_std;
	double sq_norm;
	for (std::size_t cp = 0; cp < cloud->points.size(); cp++)
	{
		// get a vector pointing from the camera to the point
		ray_vec.x() = cloud->points[cp].x - position.x();
		ray_vec.y() = cloud->points[cp].y - position.y();
		ray_vec.z() = cloud->points[cp].z - position.z();

		// normalize it
		sq_norm = ray_vec.squaredNorm();
		ray_vec = ray_vec / sqrt(sq_norm);

		// get the noise vector magnitude
		noise_std = noise_param * sq_norm;

		//set the correct size
		ray_vec = generator()*noise_std*ray_vec;

		// add the noise
		cloud->points[cp].x += ray_vec.x();
		cloud->points[cp].y += ray_vec.y();
		cloud->points[cp].z += ray_vec.z();
	}
}
/**
 * argv[1] = path to training ply file
 * argv[2] = hypothesis id
 * argv[3] = path to omap save directory
 * argv[4] = vpx
 * argv[5] = vpy
 * argv[6] = vpz
 * argv[7] = vp_id
 */
int build_omap_main(int argc, char **argv)
{
	std::cout << "Starting oMap build process..." << std::endl;

	if(argc < 4)
		throw std::runtime_error("A directory to save the confMat.txt is required...\n");
	if(argc < 3)
		throw std::runtime_error("Hypothesis id is required as second argument...\n");
	if(argc < 2)
		throw std::runtime_error("A ply file is required as a first argument...\n");

	// Get directory paths and hypothesis id
	std::string ply_file_path( argv[1] );
	std::string hyp_id( argv[2] );
	std::string save_dir_path( argv[3] );
	std::string vision_module_path("/home/bharath/GRASP_Code/ns_shared/penn_nbv_slam/vision_module");

	// Form the output filename
	std::stringstream omap_file_path;
	if(argc > 4){
		omap_file_path << save_dir_path << "/confMatorg_" << hyp_id << "_" << argv[7] << ".txt";
	}
	else{
		omap_file_path << save_dir_path << "/confMatorg_" << hyp_id << ".txt";
	}
	std::ofstream outfile ( omap_file_path.str().c_str(), ofstream::app );

	if(!outfile.is_open())
		throw std::runtime_error("Unable to open conf mat save file...\n");

	// Import the sensor locations
	int num_vp;
	int dim_vp = 3;

	Eigen::MatrixXd vp_positions;

	num_vp = 1;
	vp_positions.resize( num_vp, dim_vp );
	double vpx = std::atof(argv[4]);
	double vpy = std::atof(argv[5]);
	double vpz = std::atof(argv[6]);
	vp_positions << vpx, vpy, vpz;

	// Construct and initialize the virtual kinect
	int coord = 1;		// 0 = obj coord, 1 = optical sensor coord, 2 = standard sensor coord
	bool add_noise = false;
	vkin_offline vko( Eigen::Vector3d(0,0,0), Eigen::Vector4d(0,0,0,1),
					  coord, false, add_noise );
	vko.init_vkin( ply_file_path );
	Eigen::Vector3d target(0,0,0);

	// Construct and start a vocabulary tree
	vtree_user vt("/load", vision_module_path + "/data", vision_module_path + "/../data/cloud_data/");
	vt.start();

	// noise
	boost::mt19937 rng(time(0));
	boost::normal_distribution<double> normal_distrib(0.0, 1.0);
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> > gaussian_rng( rng, normal_distrib );


	double vp_noise_param = 0.04;	// standard deviation in position
	double tp_noise_param = 0.015;	// standard deviation in target
	int num_rep = 20;
		for(int vp = 0; vp < num_vp; ++vp)
		{
			for(int r = 0; r < num_rep; ++r)
			{
				// get cloud copy
				Eigen::Vector3d noise_vp;
				noise_vp.x() = vp_positions(vp,0) + vp_noise_param*gaussian_rng();
				noise_vp.y() = vp_positions(vp,1) + vp_noise_param*gaussian_rng();
				noise_vp.z() = vp_positions(vp,2) + vp_noise_param*gaussian_rng();

				Eigen::Vector3d noise_tp;
				noise_tp.x() = target.x() + tp_noise_param*gaussian_rng();
				noise_tp.y() = target.y() + tp_noise_param*gaussian_rng();
				noise_tp.z() = target.z() + tp_noise_param*gaussian_rng();

				pcl::PointCloud<pcl::PointXYZ>::Ptr noise_cloud_ptr = vko.sense( noise_vp, noise_tp );
				//add noise
				addNoise( Eigen::Vector3d(0,0,0), noise_cloud_ptr, gaussian_rng );
				std::pair<float,std::string> vp_score = vt.top_match( noise_cloud_ptr->getMatrixXfMap() );

				// append vp and vp_score to the file
				outfile << argv[1]<<" "<<argv[7] << " " << vp_score.second << " " << vp_score.first << std::endl;
			}
		}

		outfile.close();

	return 0;
}


int main(int argc, char **argv)
{
	return build_omap_main( argc, argv );
}
