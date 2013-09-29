#ifndef _TABLETOP_ANALYZER_HPP_
#define _TABLETOP_ANALYZER_HPP_

// Ros
#include <ros/ros.h>

// Pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


class tabletop_analyzer
{
public:
	typedef pcl::PointXYZ    PointT;
	typedef pcl::search::KdTree<PointT> SearchMethod;

	struct TableInfo
	{
		pcl::ModelCoefficients::Ptr tab_coefs;
		pcl::PointCloud<PointT>::Ptr table_hull_ptr;
		Eigen::Vector3d table_centroid;
		
		pcl::PointCloud<PointT>::Ptr obj_cld_ptr;
		std::vector<pcl::PointCloud<PointT>::Ptr> obj_surf;	// return downsampled cloud surfaces directly
		//std::vector<pcl::PointIndices> obj_idx;	
													
		pcl::PointCloud<PointT>::Ptr obj_maxp;		// return bounding boxes too
		pcl::PointCloud<PointT>::Ptr obj_minp;
		pcl::PointCloud<PointT>::Ptr obj_centroids;
		pcl::PointCloud<PointT>::Ptr obj_proj_centroids;

		TableInfo() :
			table_hull_ptr( new pcl::PointCloud<PointT>),
			obj_cld_ptr( new pcl::PointCloud<PointT>),
			obj_maxp( new pcl::PointCloud<PointT>),
			obj_minp( new pcl::PointCloud<PointT>),
			obj_centroids( new pcl::PointCloud<PointT>),
			obj_proj_centroids( new pcl::PointCloud<PointT>)
		{}			
	};

private:
	//! Min number of inliers for reliable plane detection
	int table_inlier_threshold_;
	//! Size of downsampling grid before performing plane detection
	double table_detection_voxel_size_;
	//! Size of downsampling grid before performing clustering
	double object_detection_voxel_size_;
	//! Filtering of original point cloud along the z axis
	double z_filter_min_, z_filter_max_;
	//! Filtering of point cloud in table frame after table detection
	double table_z_filter_min_, table_z_filter_max_;
	//! Min distance between two clusters
	double object_cluster_distance_;
	double table_cluster_distance_;
	//! Min and max number of points for an object cluster
	int max_object_cluster_size_;
	int min_object_cluster_size_;
	//! Min number of points for a table cluster
	int min_table_cluster_size_;

	// Ros stuff
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
		
protected:
	virtual ros::NodeHandle& getNodeHandle() { return nh_; }
  	virtual ros::NodeHandle& getPrivateNodeHandle() { return private_nh_; }

public:
	tabletop_analyzer();
	tabletop_analyzer( ros::NodeHandle & nh );
	virtual ~tabletop_analyzer() {}
	
	std::vector<TableInfo> detect_tabletop_objects( const pcl::PointCloud<PointT>::ConstPtr & cld_ptr );

	TableInfo process_table( pcl::ModelCoefficients::Ptr & table_coeffs,
									 const pcl::PointCloud<PointT>::ConstPtr & cld_ptr,
									 const pcl::PointCloud<PointT>::ConstPtr & tab_ptr );

	void obj_cld2clusters( pcl::ModelCoefficients::Ptr & table_coeffs,
								  pcl::PointCloud<pcl::PointXYZ>::Ptr & obj_cld,
								  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & obj_surf,
								  pcl::PointCloud<pcl::PointXYZ>::Ptr & obj_maxp,
								  pcl::PointCloud<pcl::PointXYZ>::Ptr & obj_minp,
								  pcl::PointCloud<pcl::PointXYZ>::Ptr & obj_centroids,
								  pcl::PointCloud<pcl::PointXYZ>::Ptr & obj_proj_centroids );
								  
	void obj_cld2clusters( pcl::ModelCoefficients::Ptr & table_coeffs,
								  pcl::PointCloud<pcl::PointXYZRGB>::Ptr & obj_cld,
								  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> & obj_surf,
								  pcl::PointCloud<pcl::PointXYZ>::Ptr & obj_maxp,
								  pcl::PointCloud<pcl::PointXYZ>::Ptr & obj_minp,
								  pcl::PointCloud<pcl::PointXYZ>::Ptr & obj_centroids,
								  pcl::PointCloud<pcl::PointXYZ>::Ptr & obj_proj_centroids );
private:
	// Area centroid for a convex hull in 2D, any other input will not produce expected result
	template <typename T>
	Eigen::Vector3d getXYAreaCentroid( boost::shared_ptr< pcl::PointCloud<T> > &cld_2d_ptr);
};


template <typename T>
Eigen::Vector3d
tabletop_analyzer::getXYAreaCentroid( boost::shared_ptr< pcl::PointCloud<T> > & cld_2d_ptr)
{
	if( cld_2d_ptr->size() < 3 )
	{
		return Eigen::Vector3d();
	}

	double a_sum = 0, xa_sum = 0, ya_sum = 0;
	double z_sum = cld_2d_ptr->points[0].z;

	for(unsigned int i = 1; i < cld_2d_ptr->size()-1; ++i)
	{
		z_sum += cld_2d_ptr->points[i].z;
		double triangle_x_centroid = (cld_2d_ptr->points[0].x + 
												cld_2d_ptr->points[i].x + 
												cld_2d_ptr->points[i+1].x)/3;

		double triangle_y_centroid = (cld_2d_ptr->points[0].y + 
												cld_2d_ptr->points[i].y + 
												cld_2d_ptr->points[i+1].y)/3;

		double triangle_area = fabs( 
				(cld_2d_ptr->points[0].x*(cld_2d_ptr->points[i].y-cld_2d_ptr->points[i+1].y) +
		       cld_2d_ptr->points[i].x*(cld_2d_ptr->points[i+1].y-cld_2d_ptr->points[0].y) +
		       cld_2d_ptr->points[i+1].x*(cld_2d_ptr->points[0].y-cld_2d_ptr->points[i].y)) / 2);

		a_sum += triangle_area;
		xa_sum += triangle_x_centroid*triangle_area;
		ya_sum += triangle_y_centroid*triangle_area;
	}

	z_sum += cld_2d_ptr->points.back().z;

	if(a_sum < 1e-8)
	{
		return Eigen::Vector3d();
	}

	return Eigen::Vector3d(xa_sum/a_sum, ya_sum/a_sum, z_sum/cld_2d_ptr->points.size());
}





#endif
