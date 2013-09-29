
#ifndef PCD_UTILS_HPP_
#define PCD_UTILS_HPP_

#include <ros/console.h>

// PCL 
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>

// Ann
#include <ANN/ANN.h>

class pcd_utils
{
private:
	class ex_VoxelGrid: public pcl::VoxelGrid< pcl::PointXYZ >
	{
		public:
			ex_VoxelGrid(){
				extract_removed_indices_ = true;
			}
			virtual ~ex_VoxelGrid(){}
	};

public:

	template <typename PointT>
	static boost::shared_ptr< pcl::PointCloud<PointT> >
	voxel_grid_subsample( const boost::shared_ptr< pcl::PointCloud<PointT> > & cld_in, float cell_size );

	template <typename PointT>
	static boost::shared_ptr< pcl::PointCloud<PointT> >
	rnd_subsample( const boost::shared_ptr< pcl::PointCloud<PointT> > & cld_in, int size );
	
	static void color_pointcloud( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cld_ptr,
					   				pcl::PointIndices::Ptr indices, Eigen::Vector3f color );
	
	/**
	 * \brief cluster keypoints in 2D image according to the logsig criterion
	 * using KdTree
	 * \param points 2D coordinate of every point
	 * \param points_count number of keypoints
	 * \param membership membership vector
	 * \param r_max max radius for adaptive radius calculation
	 * \param r_min min radius for adaptive radius calculation
	 * \param A
	 * \param K
	 */
	static int cluster_points( ANNpointArray points, int points_count,
                   			   std::vector<int>& membership, float r_max = 600.0f, 
                   			   float r_min = 200.0f, float A = 800.0f, float K = 0.02f );
   

	template <typename PointT>
	static boost::shared_ptr< pcl::PointCloud<PointT> >
	align_clouds( boost::shared_ptr< pcl::PointCloud<PointT> > &query_cld, 
					  boost::shared_ptr< pcl::PointCloud<PointT> > &base_cld );

	template <typename PointT>
	static boost::shared_ptr< pcl::PointCloud<PointT> >
	align_clouds( boost::shared_ptr< pcl::PointCloud<PointT> > &query_cld, 
					  boost::shared_ptr< pcl::PointCloud<PointT> > &base_cld,
					  double max_corr_dist,
					  double & fitness_score );
					  
	template <typename PointT>
	static boost::shared_ptr< pcl::PointCloud<PointT> >
	align_clouds( boost::shared_ptr< pcl::PointCloud<PointT> > &query_cld, 
					  boost::shared_ptr< pcl::PointCloud<PointT> > &base_cld,
					  double max_corr_dist,
					  const Eigen::Matrix4f &guess,
					  double & fitness_score );

	template <typename PointT>
	static boost::shared_ptr< pcl::PointCloud<PointT> >
	align_clouds( boost::shared_ptr< pcl::PointCloud<PointT> > &query_cld, 
					  boost::shared_ptr< pcl::PointCloud<PointT> > &base_cld,
					  double max_corr_dist,
					  const Eigen::Matrix4f &guess,
					  Eigen::Matrix4f &fTrans,
					  double & fitness_score );					  
					    
	static double logsig(double x)
	{
		return 1.0 / (1.0 + exp(-x));
	}
	
};


//**************************************************************************************
//*************** Templated Function Implementations
//**************************************************************************************


//pcl::IndicesConstPtr	//pcl::PointIndices::Ptr const boost::shared_ptr<const pcl::PointCloud<PointT> > & cld_in
template <typename PointT>
boost::shared_ptr< pcl::PointCloud<PointT> >
pcd_utils::voxel_grid_subsample( const boost::shared_ptr< pcl::PointCloud<PointT> > & cld_in, float cell_size )
{
	//ex_VoxelGrid sor;
	pcl::VoxelGrid< PointT > sor;
	sor.setInputCloud (cld_in);
	sor.setLeafSize (cell_size, cell_size, cell_size);	//leaf size in meters

	boost::shared_ptr< pcl::PointCloud<PointT> > final_cld( new pcl::PointCloud<PointT>);
	sor.filter ( *final_cld );
	return final_cld;

	//return sor.getRemovedIndices();
}

template <typename PointT>
boost::shared_ptr< pcl::PointCloud<PointT> >	//boost::shared_ptr<pcl::PointCloud<PointT> > & cld_out
pcd_utils::rnd_subsample( const boost::shared_ptr< pcl::PointCloud<PointT> > & cld_in, int size )
{
	pcl::RandomSample<PointT> sor;
	sor.setInputCloud( cld_in );
	sor.setSample( size );

	boost::shared_ptr< pcl::PointCloud<PointT> > final_cld( new pcl::PointCloud<PointT>);
	sor.filter( *final_cld );
	return final_cld;
}

template <typename PointT>
boost::shared_ptr< pcl::PointCloud<PointT> >
pcd_utils::align_clouds( boost::shared_ptr< pcl::PointCloud<PointT> > &query_cld, 
								 boost::shared_ptr< pcl::PointCloud<PointT> > &base_cld )
{
	pcl::IterativeClosestPoint< PointT, PointT > icp;
	icp.setMaxCorrespondenceDistance(0.1);
	icp.setTransformationEpsilon(1e-7);	// transformation convergence epsilon
	//icp.setMaximumIterations(1000);			
	//icp.setEuclideanFitnessEpsilon(1.0); // maximum allowed error between two consecutive steps
	icp.setInputCloud( query_cld );
	icp.setInputTarget( base_cld );

	boost::shared_ptr< pcl::PointCloud<PointT> > final_cld( new pcl::PointCloud<PointT>);

	icp.align( *final_cld );		// final cloud contains the transformed query cloud
	//*final_cld += *base_cld;		// add the two clouds

	if( icp.hasConverged() )
	{
		ROS_INFO("[pcd_utils] ICP has converged with score: %f", icp.getFitnessScore());
		return final_cld;
	}else{
		ROS_WARN("[pcd_utils] ICP did not converge!"); 
		return query_cld;
	}
}

template <typename PointT>
boost::shared_ptr< pcl::PointCloud<PointT> >
pcd_utils::align_clouds( boost::shared_ptr< pcl::PointCloud<PointT> > &query_cld, 
								 boost::shared_ptr< pcl::PointCloud<PointT> > &base_cld,
								 double max_corr_dist,
								 double & fitness_score )
{
	pcl::IterativeClosestPoint< PointT, PointT > icp;
	icp.setMaxCorrespondenceDistance(max_corr_dist);
	icp.setTransformationEpsilon(1e-7);	// transformation convergence epsilon
	//icp.setMaximumIterations(1000);			
	//icp.setEuclideanFitnessEpsilon(1.0); // maximum allowed error between two consecutive steps
	icp.setInputCloud( query_cld );
	icp.setInputTarget( base_cld );

	boost::shared_ptr< pcl::PointCloud<PointT> > final_cld( new pcl::PointCloud<PointT>);

	icp.align( *final_cld );		// final cloud contains the transformed query cloud
	//*final_cld += *base_cld;		// add the two clouds

	if( icp.hasConverged() )
	{
		fitness_score = icp.getFitnessScore();
		ROS_INFO("[pcd_utils] ICP has converged with score: %f", fitness_score);
		return final_cld;
	}else{
		ROS_WARN("[pcd_utils] ICP did not converge!"); 
		return query_cld;
	}
}

template <typename PointT>
boost::shared_ptr< pcl::PointCloud<PointT> >
pcd_utils::align_clouds( boost::shared_ptr< pcl::PointCloud<PointT> > &query_cld, 
								 boost::shared_ptr< pcl::PointCloud<PointT> > &base_cld,
								 double max_corr_dist,
								 const Eigen::Matrix4f &guess,
								 double & fitness_score )
{
	pcl::IterativeClosestPoint< PointT, PointT > icp;
	icp.setMaxCorrespondenceDistance(max_corr_dist);	// 0.1
	icp.setTransformationEpsilon(1e-7);	// transformation convergence epsilon
	//icp.setMaximumIterations(1000);			
	//icp.setEuclideanFitnessEpsilon(1.0); // maximum allowed error between two consecutive steps
	icp.setInputCloud( query_cld );
	icp.setInputTarget( base_cld );

	boost::shared_ptr< pcl::PointCloud<PointT> > final_cld( new pcl::PointCloud<PointT>);

	icp.align( *final_cld, guess );		// final cloud contains the transformed query cloud
	//*final_cld += *base_cld;		// add the two clouds

	if( icp.hasConverged() )
	{
		fitness_score = icp.getFitnessScore();
		ROS_INFO("[pcd_utils] ICP has converged with score: %f", fitness_score);
		return final_cld;
	}else{
		ROS_WARN("[pcd_utils] ICP did not converge!"); 
		return query_cld;
	}
}

template <typename PointT>
boost::shared_ptr< pcl::PointCloud<PointT> >
pcd_utils::align_clouds( boost::shared_ptr< pcl::PointCloud<PointT> > &query_cld, 
								  boost::shared_ptr< pcl::PointCloud<PointT> > &base_cld,
								  double max_corr_dist,
								  const Eigen::Matrix4f &guess,
								  Eigen::Matrix4f &fTrans,
								  double & fitness_score )
{
	pcl::IterativeClosestPoint< PointT, PointT > icp;
	icp.setMaxCorrespondenceDistance(max_corr_dist);	// 0.1
	icp.setTransformationEpsilon(1e-7);	// transformation convergence epsilon
	//icp.setMaximumIterations(1000);			
	//icp.setEuclideanFitnessEpsilon(1.0); // maximum allowed error between two consecutive steps
	icp.setInputCloud( query_cld );
	icp.setInputTarget( base_cld );

	boost::shared_ptr< pcl::PointCloud<PointT> > final_cld( new pcl::PointCloud<PointT>);

	icp.align( *final_cld, guess );		// final cloud contains the transformed query cloud
	//*final_cld += *base_cld;		// add the two clouds

	if( icp.hasConverged() )
	{
		fitness_score = icp.getFitnessScore();
		ROS_INFO("[pcd_utils] ICP has converged with score: %f", fitness_score);
		fTrans = icp.getFinalTransformation();
		return final_cld;
	}else{
		ROS_WARN("[pcd_utils] ICP did not converge!"); 
		return query_cld;
	}
}

#endif
