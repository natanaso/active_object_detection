//
#include <iostream>
#include <string>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/passthrough.h>
#include <pcd_utils.hpp>
//#include <pcd17_util.hpp>

int aligner(int argc, char **argv)
{
	// load pcds
	pcl::PointCloud<pcl::PointXYZ>::Ptr cld_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cld_ptr_original(new pcl::PointCloud<pcl::PointXYZ>);
	
   pcl::io::loadPCDFile<pcl::PointXYZ> ( "/home/natanaso/Desktop/cld_experiment/cld_7189.3909999999996.pcd", *cld_ptr);
   pcl::io::loadPCDFile<pcl::PointXYZ> ( "/home/natanaso/Desktop/cld_experiment/orig_7189.3909999999996.pcd", *xyz_cld_ptr_original);
   
   
	// remove NaNs
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cld_ptr, *cld_ptr, indices);

	
	pcl::PCDWriter writer;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cld_ptr_tmp(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cld_ptr_a(new pcl::PointCloud<pcl::PointXYZ>);
	
	pcl::PassThrough<pcl::PointXYZ> pass_;
	pass_.setFilterFieldName ("y");
	pass_.setFilterLimits (-1, 1);
	pass_.setInputCloud(cld_ptr);
	pass_.filter(*cld_ptr_tmp);
	
	pass_.setFilterFieldName ("z");
	pass_.setFilterLimits (0.55, 2.0);	
	pass_.setInputCloud(cld_ptr_tmp);
	pass_.filter(*cld_ptr_a);
		
	cld_ptr_a = pcd_utils::voxel_grid_subsample( cld_ptr_a, 0.003f );
	
	
	writer.writeASCII( "/home/natanaso/Desktop/cld_experiment/init_0.pcd", *cld_ptr_a + *xyz_cld_ptr_original );	
	writer.writeASCII( "/home/natanaso/Desktop/cld_experiment/filt_0.pcd", *cld_ptr_a );
	
	// Get an initial guess for the transform
	pass_.setFilterLimits (0.75, 2.0);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cld_ptr_filt(new pcl::PointCloud<pcl::PointXYZ>);
	pass_.setInputCloud(cld_ptr_a);
	pass_.filter(*cld_ptr_filt);
	pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cld_ptr_filt(new pcl::PointCloud<pcl::PointXYZ>);
	pass_.setInputCloud(xyz_cld_ptr_original);
	pass_.filter(*xyz_cld_ptr_filt);

	
	writer.writeASCII( "/home/natanaso/Desktop/cld_experiment/filt_a.pcd", *cld_ptr_filt );
	writer.writeASCII( "/home/natanaso/Desktop/cld_experiment/filt_b.pcd", *xyz_cld_ptr_filt );
		
	double fitness_score = 1000;
	Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f fTrans = Eigen::Matrix4f::Identity();
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	tmp_ptr = pcd_utils::align_clouds( cld_ptr_filt, xyz_cld_ptr_filt, 0.2, guess, fTrans, fitness_score );
	
	/*
	// XXX: USE PCL17
	pcd17_util::align_clouds17( cld_ptr_filt->getMatrixXfMap(),
										xyz_cld_ptr_filt->getMatrixXfMap(),
										0.5, guess, fTrans, fitness_score );
	pcl::transformPointCloud ( *cld_ptr_filt, *tmp_ptr, fTrans );
	// *** END ***
	*/
	
	writer.writeASCII( "/home/natanaso/Desktop/cld_experiment/combined_a.pcd", *tmp_ptr + * xyz_cld_ptr_filt);
	
	// Use the guess to allign a larger portion of the cloud
	//	pass_.setFilterLimits (0.55, 2.0);
	//	pass_.setInputCloud(cld_ptr);
	//	pass_.filter(*cld_ptr_filt);
	//	pass_.setInputCloud(xyz_cld_ptr_original);
	//	pass_.filter(*xyz_cld_ptr_filt);
	//	guess = fTrans;
	//	tmp_ptr = pcd_utils::align_clouds( cld_ptr_filt, xyz_cld_ptr_filt, 0.2, guess, fTrans, fitness_score );

	// Use the guess to align the large cloud	
	guess = fTrans;
	tmp_ptr = pcd_utils::align_clouds( cld_ptr_a, xyz_cld_ptr_original, 0.2, guess, fTrans, fitness_score );	


	Eigen::Matrix4f best_trans = fTrans;
	double best_score = fitness_score;
	if( fitness_score > 0.0005 )
	{
		const double arr[] = {0.2, 0.0, -0.2, 0.0, 0.0, 0.2, 0.0, -0.2};
		
		for(int k = 0; k < 4; ++k)
		{
			// generate the offset cloud above the table
			pcl::PointCloud<pcl::PointXYZ>::Ptr cld_ptr_off(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::copyPointCloud(*cld_ptr_filt, *cld_ptr_off);
			for( pcl::PointCloud<pcl::PointXYZ>::iterator it = cld_ptr_off->begin();
				it != cld_ptr_off->end(); ++it)
			{
				(*it).x += arr[2*k];
				(*it).y += arr[2*k+1];
			}
			guess = Eigen::Matrix4f::Identity();
			pcd_utils::align_clouds( cld_ptr_off, xyz_cld_ptr_filt, 0.2, guess, fTrans, fitness_score );	
			
			
			pcl::PointCloud<pcl::PointXYZ>::Ptr cld_ptr_a_off(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::copyPointCloud(*cld_ptr_a, *cld_ptr_a_off);
			for( pcl::PointCloud<pcl::PointXYZ>::iterator it = cld_ptr_a_off->begin();
				it != cld_ptr_a_off->end(); ++it)
			{
				(*it).x += arr[2*k];
				(*it).y += arr[2*k+1];
			}		
			guess = fTrans;
			pcd_utils::align_clouds( cld_ptr_a_off, xyz_cld_ptr_original, 0.2, guess, fTrans, fitness_score );
			
			/*
			if( k == 3)
			{
				pcl::transformPointCloud ( *cld_ptr_a_off, *tmp_ptr, fTrans );
				writer.writeASCII( "/home/natanaso/Desktop/cld_experiment/combined_b.pcd", *tmp_ptr + *xyz_cld_ptr_original );
			}
			*/
			
			if( fitness_score < best_score)
			{
				best_score = fitness_score;
				
				Eigen::Matrix4f T = Eigen::Matrix4f::Identity();				
				T(0,3) = arr[2*k];
				T(1,3) = arr[2*k+1];
				best_trans = fTrans * T;
			}	
		}
	}
				
								
	pcl::PointCloud<pcl::PointXYZ>::Ptr best_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud ( *cld_ptr_a, *best_ptr, best_trans );
   
   
   cld_ptr = best_ptr;	
	*xyz_cld_ptr_original = *cld_ptr + *xyz_cld_ptr_original;		// add the two clouds

	//writer.writeASCII( "/home/natanaso/Desktop/cld_experiment/filt_c.pcd", *cld_ptr_filt );
	//writer.writeASCII( "/home/natanaso/Desktop/cld_experiment/filt_d.pcd", *xyz_cld_ptr_filt );
	writer.writeASCII( "/home/natanaso/Desktop/cld_experiment/combined1.pcd", *xyz_cld_ptr_original );
   
   /*
   // First cut the table off
	// first align only the clouds above the table to get a guess for the transformation
	Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f fTrans = Eigen::Matrix4f::Identity();


	pcl::PointCloud<pcl::PointXYZ>::Ptr cam_cld_ptr_filt(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr head_cld_ptr_filt(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	
	pcl::PassThrough<pcl::PointXYZ> pass_;
	pass_.setFilterFieldName ("z");
	pass_.setFilterLimits (-0.35, 2.5);
	pass_.setInputCloud(cam_cld_ptr);
	pass_.filter(*cam_cld_ptr_filt);
	pass_.setInputCloud(head_cld_ptr);
	pass_.filter(*head_cld_ptr_filt);

	
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp1;
	icp1.setMaxCorrespondenceDistance(0.1);
	icp1.setTransformationEpsilon(1e-6);	// transformation convergence epsilon
	icp1.setInputCloud(cam_cld_ptr_filt);
	icp1.setInputTarget(head_cld_ptr_filt);
	icp1.align(*tmp_ptr, guess);
	std::cout << "has converged:" << icp1.hasConverged() << " score: " <<
	icp1.getFitnessScore() << std::endl;
	std::cout << icp1.getFinalTransformation() << std::endl;	
	
	fTrans = icp1.getFinalTransformation();
	
			
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setMaxCorrespondenceDistance(0.1);
	icp.setTransformationEpsilon(1e-6);	// transformation convergence epsilon
	//icp.setEuclideanFitnessEpsilon(1e-8);
	
	icp.setInputCloud(cam_cld_ptr);
	icp.setInputTarget(head_cld_ptr);
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final,fTrans);
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
	icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;
	
	
	// Save clouds for viewing
	pcl::PCDWriter writer;
	Final += *head_cld_ptr;
	writer.writeASCII( "/home/natanaso/Desktop/cld_experiment/filt_b.pcd", *head_cld_ptr_filt );
	writer.writeASCII( "/home/natanaso/Desktop/cld_experiment/combined_a.pcd", *tmp_ptr+*head_cld_ptr_filt );
	writer.writeASCII( "/home/natanaso/Desktop/cld_experiment/combined1.pcd", Final );
   */
	return 0;
}

int main(int argc, char **argv)
{
	return aligner(argc, argv);
}
