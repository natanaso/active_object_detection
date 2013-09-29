// Pcl
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/convex_hull.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>


#include "tabletop_analyzer.hpp"

tabletop_analyzer::tabletop_analyzer() :
	table_inlier_threshold_(100),
	table_detection_voxel_size_(0.005),	//0.04
	object_detection_voxel_size_(0.0005),//0.02
	z_filter_min_(-0.2),
	z_filter_max_(1.5),
	table_z_filter_min_(0.009),	//0.02
	table_z_filter_max_(1.00),
	object_cluster_distance_(0.01),
	table_cluster_distance_(0.06),
	max_object_cluster_size_(500000),
	min_object_cluster_size_(300),
	min_table_cluster_size_(100),
	private_nh_("~")
{ 

}

tabletop_analyzer::tabletop_analyzer( ros::NodeHandle & nh ) :
	nh_(nh), private_nh_("~")
{	
	private_nh_.param<int>("table_inlier_threshold", table_inlier_threshold_, 100);
	private_nh_.param<double>("table_detection_voxel_size", table_detection_voxel_size_, 0.005);
	private_nh_.param<double>("object_detection_voxel_size", object_detection_voxel_size_, 0.0005);
	private_nh_.param<double>("z_filter_min", z_filter_min_, 0.4);
	private_nh_.param<double>("z_filter_max", z_filter_max_, 2.65);		// 1.25
	private_nh_.param<double>("table_z_filter_min", table_z_filter_min_, 0.009);	// 0.01
	private_nh_.param<double>("table_z_filter_max", table_z_filter_max_, 1.00);	// 0.5
	private_nh_.param<double>("object_cluster_distance", object_cluster_distance_, 0.008);	//0.03
	private_nh_.param<double>("table_cluster_distance", table_cluster_distance_, 0.06);
	private_nh_.param<int>("max_object_cluster_size", max_object_cluster_size_, 500000);
	private_nh_.param<int>("min_object_cluster_size", min_object_cluster_size_, 300);
	private_nh_.param<int>("min_table_cluster_size", min_table_cluster_size_, 100);
}

std::vector<tabletop_analyzer::TableInfo> 
tabletop_analyzer::detect_tabletop_objects( const pcl::PointCloud<PointT>::ConstPtr & cld_ptr )
{	
	std::vector<TableInfo> DetectedTables;

	// Step 1 : Table z-Filter, remove NaNs
	pcl::PointCloud<PointT>::Ptr filtered_cld_ptr( new pcl::PointCloud<PointT>);
	pcl::PassThrough<PointT> pass_;
	pass_.setFilterFieldName ("z");
  	pass_.setFilterLimits (z_filter_min_, z_filter_max_);
	pass_.setInputCloud (cld_ptr);
  	pass_.filter (*filtered_cld_ptr);

	// If the remaining cloud is too small the table was not found
	if (filtered_cld_ptr->size() < static_cast<unsigned int>(min_table_cluster_size_))
	{
		ROS_INFO("[tabletop analyzer] Filtered cloud only has %d points", static_cast<int>(filtered_cld_ptr->size()));
		return DetectedTables;
	}
	
	// Table detection downsampling parameters
	pcl::PointCloud<PointT>::Ptr downsmp_cld_ptr( new pcl::PointCloud<PointT>);
	pcl::VoxelGrid<PointT> grid_;
	grid_.setLeafSize (table_detection_voxel_size_, table_detection_voxel_size_, table_detection_voxel_size_);
	grid_.setFilterFieldName ("z");
	grid_.setFilterLimits (z_filter_min_, z_filter_max_);
	grid_.setDownsampleAllData (false);
	grid_.setInputCloud( filtered_cld_ptr );
  	grid_.filter ( *downsmp_cld_ptr );

	if (downsmp_cld_ptr->size() < static_cast<unsigned int>(min_table_cluster_size_))
	{
   	ROS_INFO("[tabletop analyzer] Downsampled cloud only has %d points", static_cast<int>(downsmp_cld_ptr->size()));
		return DetectedTables;
	}

	// Cluster potential table points
	std::vector<pcl::PointIndices> table_clusters;
	SearchMethod::Ptr clusters_tree_ (new SearchMethod);
	pcl::EuclideanClusterExtraction<PointT> cluster_;
	cluster_.setClusterTolerance (table_cluster_distance_);
  	cluster_.setMinClusterSize (min_table_cluster_size_);
  	cluster_.setSearchMethod (clusters_tree_);
	cluster_.setInputCloud ( downsmp_cld_ptr );
  	cluster_.extract ( table_clusters );
	
	if(table_clusters.size() < 1)
  	{
   	ROS_INFO("Found no potential table clusters");
   	return DetectedTables;
  	}

	// --] Process all potential table clusters
	pcl::ExtractIndices<PointT> extract_;	// for extracting the clusters
	
	// Normal Estimation
	SearchMethod::Ptr normals_tree_ (new SearchMethod);
	pcl::NormalEstimation<PointT, pcl::Normal> n3d_;
	n3d_.setKSearch (10);
	n3d_.setSearchMethod ( normals_tree_ );

	// Table model fitting parameters
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg_;
	seg_.setDistanceThreshold (0.05);
	seg_.setMaxIterations (10000);
	seg_.setNormalDistanceWeight (0.1);
	seg_.setOptimizeCoefficients (true);
	seg_.setModelType (pcl::SACMODEL_NORMAL_PLANE);
	seg_.setMethodType (pcl::SAC_RANSAC);
	seg_.setProbability (0.99);

	// Table projection parameters
	pcl::ProjectInliers<PointT> proj_;
	proj_.setModelType (pcl::SACMODEL_PLANE);

	
	

	for(unsigned int i = 0; i < table_clusters.size(); ++i)
  	{
   	ROS_INFO_STREAM("[Processing table cluster "<<i<<" size: "<<table_clusters[i].indices.size()<<"]");
	
		// Extract the ith cluster
		pcl::PointCloud<PointT>::Ptr table_cluster_i(new pcl::PointCloud<PointT>);
		extract_.setInputCloud (downsmp_cld_ptr);
    	extract_.setIndices (boost::make_shared<const pcl::PointIndices> (table_clusters[i]));
    	extract_.setNegative (false);
    	extract_.filter (*table_cluster_i);

		if (table_cluster_i->size() < static_cast<unsigned int>(min_table_cluster_size_))
   	{
   		ROS_INFO("[tabletop analyzer] Table cluster only has %d points", static_cast<int>(table_cluster_i->size()));
   		continue;
   	}

		// Step 2 : Estimate normals
		pcl::PointCloud<pcl::Normal>::Ptr table_cluster_i_normals(new pcl::PointCloud<pcl::Normal>);
		n3d_.setInputCloud (table_cluster_i);
		n3d_.compute (*table_cluster_i_normals);

		// Step 3 : Perform planar segmentation		
		pcl::PointIndices::Ptr table_inliers (new pcl::PointIndices ());
		pcl::ModelCoefficients::Ptr table_coefficients (new pcl::ModelCoefficients ());
		seg_.setInputCloud ( table_cluster_i );
    	seg_.setInputNormals ( table_cluster_i_normals );
    	seg_.segment ( *table_inliers, *table_coefficients);
		
		if (table_coefficients->values.size () <=3)
		{
			ROS_INFO("[tabletop analyzer] Failed to detect table in scan");
			continue;
		}
		
		if ( table_inliers->indices.size() < static_cast<unsigned int>(table_inlier_threshold_))
		{
			ROS_INFO("[tabletop analyzer] Plane detection has %d inliers, below min threshold of %d", static_cast<int>(table_inliers->indices.size()), table_inlier_threshold_);
			continue;
		}

		if( fabs(table_coefficients->values[0]) > 0.3 || 
			 fabs(table_coefficients->values[1]) > 0.3 ||
			 fabs(table_coefficients->values[2]) < 0.8 )
		{
			ROS_INFO("[tabletop analyzer] Detected plane not perpendicular to z axis: [%f %f %f %f].", table_coefficients->values[0], table_coefficients->values[1],
       table_coefficients->values[2], table_coefficients->values[3]);
			continue;
		}
		
		ROS_INFO ("[tabletop_analyzer] Model found with %d inliers: [%f %f %f %f].", static_cast<int>(table_inliers->indices.size ()),
      table_coefficients->values[0], table_coefficients->values[1],
      table_coefficients->values[2], table_coefficients->values[3]);

		// Step 4 : Project the table inliers on the table
		pcl::PointCloud<PointT>::Ptr table_projected_ptr(new pcl::PointCloud<PointT>);
		proj_.setInputCloud ( table_cluster_i );
    	proj_.setIndices ( table_inliers );
    	proj_.setModelCoefficients ( table_coefficients );
    	proj_.filter( *table_projected_ptr );

		// Determine the tabletop objects
		DetectedTables.push_back( process_table( table_coefficients, filtered_cld_ptr, table_projected_ptr ) );
		
	}
	return DetectedTables;
}


tabletop_analyzer::TableInfo
tabletop_analyzer::process_table( pcl::ModelCoefficients::Ptr & table_coeffs,
											 const pcl::PointCloud<PointT>::ConstPtr & cld_ptr,
											 const pcl::PointCloud<PointT>::ConstPtr & tab_ptr )
{	
	TableInfo ti;
	ti.tab_coefs = table_coeffs;

	// ---[ Estimate the table convex hull
	//pcl::PointCloud<PointT>::Ptr table_hull_ptr( new pcl::PointCloud<PointT>);
	//pcl::ConvexHull<PointT, PointT> hull_;
	pcl::ConvexHull<PointT> hull_;
	hull_.setDimension(2);
	hull_.setInputCloud (tab_ptr);
  	hull_.reconstruct ( *(ti.table_hull_ptr) );
	ROS_ASSERT(ti.table_hull_ptr->points.size() > 2);

	// table_hull_ptr contains the boundary points of the table
	// The centroid of the table is:
	ti.table_centroid = getXYAreaCentroid<PointT>( ti.table_hull_ptr );

	// ---[ Get the objects on top of the table
	// Consider only objects in a given layer above the table
	pcl::PointIndices cloud_object_indices;
	pcl::ExtractPolygonalPrismData<PointT> prism_;
	prism_.setHeightLimits (table_z_filter_min_, table_z_filter_max_);
	prism_.setViewPoint(0.0, 0.0, 100.0);
  	prism_.setInputCloud ( cld_ptr );
  	prism_.setInputPlanarHull ( ti.table_hull_ptr );
  	prism_.segment ( cloud_object_indices );

	// Get a cloud containing only the objects
	//pcl::PointCloud<PointT>::Ptr obj_cld_ptr(new pcl::PointCloud<PointT>);
	pcl::ExtractIndices<PointT> extract_;
	extract_.setInputCloud (cld_ptr);
  	extract_.setIndices(boost::make_shared<const pcl::PointIndices> (cloud_object_indices));
  	extract_.filter( *ti.obj_cld_ptr );
	
	if ( ti.obj_cld_ptr->size() < static_cast<unsigned int>(min_object_cluster_size_))
	{
   	ROS_INFO("Not enough object points on table for object detection");
   	return ti;
	}
	
	// ---[ Downsample the object points
	pcl::PointCloud<PointT>::Ptr downsmp_obj_cld_ptr(new pcl::PointCloud<PointT>);
	if( object_detection_voxel_size_ == 0 )
		downsmp_obj_cld_ptr = ti.obj_cld_ptr;
	else{
		pcl::VoxelGrid<PointT> grid_;
		grid_.setLeafSize (object_detection_voxel_size_, object_detection_voxel_size_, object_detection_voxel_size_);
	  	grid_.setDownsampleAllData (false);
		grid_.setInputCloud (ti.obj_cld_ptr);
	  	grid_.filter( *downsmp_obj_cld_ptr );
	}
	
	if (downsmp_obj_cld_ptr->size() < static_cast<unsigned int>(min_object_cluster_size_))
	{
		ROS_INFO("Not enough object points on table after downsampling for object detection");
		return ti;
	}


	// ---[ Split the objects into Euclidean clusters
	obj_cld2clusters( ti.tab_coefs, downsmp_obj_cld_ptr, ti.obj_surf, 
							ti.obj_maxp, ti.obj_minp, ti.obj_centroids, ti.obj_proj_centroids );

	ROS_INFO_STREAM("Extracted " << ti.obj_centroids->size() <<" object cluster centroids");
	
	return ti;
	
	
	
	/*
	std::vector<pcl::PointIndices> obj_idx;
	SearchMethod::Ptr clusters_tree_ (new SearchMethod);
	pcl::EuclideanClusterExtraction<PointT> cluster_;
	cluster_.setClusterTolerance (object_cluster_distance_);
  	cluster_.setMinClusterSize (min_object_cluster_size_);
  	cluster_.setMaxClusterSize (max_object_cluster_size_);
  	cluster_.setSearchMethod (clusters_tree_);
	cluster_.setInputCloud ( downsmp_obj_cld_ptr );
  	cluster_.extract ( obj_idx );

	ROS_INFO ("Number of object clusters found on the table: %d.", static_cast<int>(obj_idx.size ()));

	//pcl::PointCloud<PointT>::Ptr obj_centroids( new pcl::PointCloud<PointT> );
	for(unsigned int i = 0; i < obj_idx.size(); ++i)
  	{
   	if(obj_idx[i].indices.size() >= static_cast<unsigned int>(min_object_cluster_size_))
    	{
      	pcl::PointCloud<PointT>::Ptr obj_idx_i(new pcl::PointCloud<PointT>);

      	extract_.setInputCloud ( downsmp_obj_cld_ptr );
      	extract_.setIndices (boost::make_shared<const pcl::PointIndices> (obj_idx[i]));
      	extract_.setNegative (false);
      	extract_.filter(*obj_idx_i);

			// save surface
			//pcl::PointCloud<pcl::PointXYZRGB>::Ptr obj_surf_i(new pcl::PointCloud<pcl::PointXYZRGB>);
			//copyPointCloud(obj_idx_i, *obj_surf_i);
			ti.obj_surf.push_back(obj_idx_i);

			

			// compute bounding box and centroid
			Eigen::Vector4f centroid, minp, maxp;
			pcl::getMinMax3D( *obj_idx_i, minp, maxp);	
      	pcl::compute3DCentroid<PointT> ( *obj_idx_i, centroid );

			//ROS_INFO_STREAM("centroid A = " << ((maxp + minp)/2).transpose() );
			//ROS_INFO_STREAM("centroid B = " << centroid.transpose() );

      	// Save the results
			ti.obj_maxp->push_back( pcl::PointXYZ( maxp(0), maxp(1), maxp(2) ));
			ti.obj_minp->push_back( pcl::PointXYZ( minp(0), minp(1), minp(2) ));
      	ti.obj_centroids->push_back(pcl::PointXYZ( centroid(0), centroid(1), centroid(2) ));
    	}
	}
	*/

}


void 
tabletop_analyzer::obj_cld2clusters(  pcl::ModelCoefficients::Ptr & table_coeffs,
												  pcl::PointCloud<pcl::PointXYZ>::Ptr & obj_cld,
												  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & obj_surf,
												  pcl::PointCloud<pcl::PointXYZ>::Ptr & obj_maxp,
												  pcl::PointCloud<pcl::PointXYZ>::Ptr & obj_minp,
												  pcl::PointCloud<pcl::PointXYZ>::Ptr & obj_centroids,
												  pcl::PointCloud<pcl::PointXYZ>::Ptr & obj_proj_centroids )
{
	std::vector<pcl::PointIndices> obj_idx;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr clusters_tree (new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_engine;
	cluster_engine.setClusterTolerance (object_cluster_distance_);
  	cluster_engine.setMinClusterSize (min_object_cluster_size_);
  	cluster_engine.setMaxClusterSize (max_object_cluster_size_);
  	cluster_engine.setSearchMethod (clusters_tree);
	cluster_engine.setInputCloud ( obj_cld );
  	cluster_engine.extract ( obj_idx );

	if( obj_idx.size() == 0)
		ROS_WARN("No clusters were found! Check the ClusterTolerance and Min/Max ClusterSize Parameters!!!");
	
	//pcl::PointCloud<PointT>::Ptr obj_centroids( new pcl::PointCloud<PointT> );
	pcl::ExtractIndices<pcl::PointXYZ> extract_;
	for(unsigned int i = 0; i < obj_idx.size(); ++i)
  	{
   	if(obj_idx[i].indices.size() >= static_cast<unsigned int>(min_object_cluster_size_))
    	{
      	pcl::PointCloud<pcl::PointXYZ>::Ptr obj_idx_i(new pcl::PointCloud<pcl::PointXYZ>);

      	extract_.setInputCloud ( obj_cld );
      	extract_.setIndices (boost::make_shared<const pcl::PointIndices> (obj_idx[i]));
      	extract_.setNegative (false);
      	extract_.filter(*obj_idx_i);

			// save surface
			//pcl::PointCloud<pcl::PointXYZRGB>::Ptr obj_surf_i(new pcl::PointCloud<pcl::PointXYZRGB>);
			//copyPointCloud(obj_idx_i, *obj_surf_i);
			obj_surf.push_back(obj_idx_i);

			

			// compute bounding box and centroid
			Eigen::Vector4f centroid, minp, maxp;
			pcl::getMinMax3D( *obj_idx_i, minp, maxp);	
      	pcl::compute3DCentroid<pcl::PointXYZ> ( *obj_idx_i, centroid );

			//ROS_INFO_STREAM("centroid A = " << ((maxp + minp)/2).transpose() );
			//ROS_INFO_STREAM("centroid B = " << centroid.transpose() );

      	// Save the results
			obj_maxp->push_back( pcl::PointXYZ( maxp(0), maxp(1), maxp(2) ));
			obj_minp->push_back( pcl::PointXYZ( minp(0), minp(1), minp(2) ));
      	obj_centroids->push_back(pcl::PointXYZ( centroid(0), centroid(1), centroid(2) ));
    	}
	}
	
	if( (obj_centroids -> size()) > 0 ){
		// Project the object centroids to the the table surface
		pcl::ProjectInliers<pcl::PointXYZ> proj_cntr_;
		proj_cntr_.setModelType (pcl::SACMODEL_PLANE);
		proj_cntr_.setInputCloud ( obj_centroids );
		proj_cntr_.setModelCoefficients ( table_coeffs );
		proj_cntr_.filter( *obj_proj_centroids );
	}
	else
		ROS_WARN("[tabletop_analyzer] obj_centroids is empty!"); 
}

void 
tabletop_analyzer::obj_cld2clusters(  pcl::ModelCoefficients::Ptr & table_coeffs,
												  pcl::PointCloud<pcl::PointXYZRGB>::Ptr & obj_cld,
												  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> & obj_surf,
												  pcl::PointCloud<pcl::PointXYZ>::Ptr & obj_maxp,
												  pcl::PointCloud<pcl::PointXYZ>::Ptr & obj_minp,
												  pcl::PointCloud<pcl::PointXYZ>::Ptr & obj_centroids,
												  pcl::PointCloud<pcl::PointXYZ>::Ptr & obj_proj_centroids )
{
	std::vector<pcl::PointIndices> obj_idx;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr clusters_tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> cluster_engine;
	cluster_engine.setClusterTolerance (object_cluster_distance_);
  	cluster_engine.setMinClusterSize (min_object_cluster_size_);
  	cluster_engine.setMaxClusterSize (max_object_cluster_size_);
  	cluster_engine.setSearchMethod (clusters_tree);
	cluster_engine.setInputCloud ( obj_cld );
  	cluster_engine.extract ( obj_idx );

	//pcl::PointCloud<PointT>::Ptr obj_centroids( new pcl::PointCloud<PointT> );
	pcl::ExtractIndices<pcl::PointXYZRGB> extract_;
	for(unsigned int i = 0; i < obj_idx.size(); ++i)
  	{
   	if(obj_idx[i].indices.size() >= static_cast<unsigned int>(min_object_cluster_size_))
    	{
      	pcl::PointCloud<pcl::PointXYZRGB>::Ptr obj_idx_i(new pcl::PointCloud<pcl::PointXYZRGB>);

      	extract_.setInputCloud ( obj_cld );
      	extract_.setIndices (boost::make_shared<const pcl::PointIndices> (obj_idx[i]));
      	extract_.setNegative (false);
      	extract_.filter(*obj_idx_i);

			// save surface
			//pcl::PointCloud<pcl::PointXYZRGB>::Ptr obj_surf_i(new pcl::PointCloud<pcl::PointXYZRGB>);
			//copyPointCloud(obj_idx_i, *obj_surf_i);
			obj_surf.push_back(obj_idx_i);

			
			// compute bounding box and centroid
			Eigen::Vector4f centroid, minp, maxp;
			pcl::getMinMax3D( *obj_idx_i, minp, maxp);	
      	pcl::compute3DCentroid<pcl::PointXYZRGB> ( *obj_idx_i, centroid );

			//ROS_INFO_STREAM("centroid A = " << ((maxp + minp)/2).transpose() );
			//ROS_INFO_STREAM("centroid B = " << centroid.transpose() );

      	// Save the results
			obj_maxp->push_back( pcl::PointXYZ( maxp(0), maxp(1), maxp(2) ));
			obj_minp->push_back( pcl::PointXYZ( minp(0), minp(1), minp(2) ));
      	obj_centroids->push_back(pcl::PointXYZ( centroid(0), centroid(1), centroid(2) ));
    	}
	}
	
	if( (obj_centroids -> size()) > 0 ){
		// Project the object centroids to the the table surface
		pcl::ProjectInliers<pcl::PointXYZ> proj_cntr_;
		proj_cntr_.setModelType (pcl::SACMODEL_PLANE);	
		proj_cntr_.setInputCloud ( obj_centroids );
		proj_cntr_.setModelCoefficients ( table_coeffs );
		proj_cntr_.filter( *obj_proj_centroids );
	}
	else
		ROS_WARN("[tabletop_analyzer] obj_centroids is empty!"); 
}

/*
int tabletop_analyzer_main(int argc, char **argv)
{
	tabletop_analyzer TA;
	return 0;
}

int main(int argc, char **argv)
{
	return tabletop_analyzer_main(argc, argv);
}
*/
