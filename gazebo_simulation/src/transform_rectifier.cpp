//
#include <iostream>
#include <string>

// Pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

// Ros
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl_ros/publisher.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr cam_cld_ptr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr head_cld_ptr(new pcl::PointCloud<pcl::PointXYZ>);


tf::StampedTransform cam2hand;
tf::StampedTransform head2hand;

bool head_cld_up = false;
bool cam_cld_up = false;

void cam_cld_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud)
{	
	ROS_INFO("Updating cam cloud");
	std::string arm_frm("/r_gripper_r_finger_link");
	std::string cam_frm("/arm_kinect_depth_optical_frame");
	tf::TransformListener listener(ros::Duration(100.0));
	while(!cam_cld_up)
	{
		try
		{
			//listener.waitForTransform( cam_frm, arm_frm, cloud->header.stamp, ros::Duration(5.0));
			//listener.lookupTransform( arm_frm, cam_frm, cloud->header.stamp, cam2hand);
			listener.lookupTransform( arm_frm, cam_frm, ros::Time(0), cam2hand);
		
			// update the cloud
			pcl::copyPointCloud(*cloud, *cam_cld_ptr);
			cam_cld_up = true;
			ROS_INFO("cam update success!");
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s", ex.what());
		}
	}
}

void head_cld_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud)
{	
	ROS_INFO("Updating head cloud");
	std::string arm_frm("/r_gripper_r_finger_link");
	std::string head_frm("/narrow_stereo_optical_frame");
	tf::TransformListener listener(ros::Duration(100.0));
	while(!head_cld_up)
	{
		try
		{
			//listener.waitForTransform( head_frm, arm_frm, cloud->header.stamp, ros::Duration(5.0));
			listener.lookupTransform( arm_frm, head_frm, ros::Time(0), head2hand);
		
			// update the cloud
			pcl::copyPointCloud(*cloud, *head_cld_ptr);
			head_cld_up = true;
			ROS_INFO("head update success!");
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s", ex.what());
		}
	}
}
	
int trec(int argc, char **argv)
{
	ros::init(argc, argv, "cloud_rectifier");
	ros::NodeHandle nh_;
	ros::Subscriber cam_cld_sub = nh_.subscribe("/arm_kinect/depth/points", 1, &cam_cld_callback);
	ros::Subscriber head_cld_sub = nh_.subscribe("/narrow_stereo/points2", 1, &head_cld_callback);
	
	ROS_INFO("Waiting for clouds...");
	while( !head_cld_up || !cam_cld_up)
		ros::spinOnce();
	ROS_INFO("Clouds received!");
	
	
	// transform the clouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr cam_Arm_cld_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr head_Arm_cld_ptr(new pcl::PointCloud<pcl::PointXYZ>);	
	pcl_ros::transformPointCloud( *cam_cld_ptr, *cam_Arm_cld_ptr, cam2hand);
	pcl_ros::transformPointCloud( *head_cld_ptr, *head_Arm_cld_ptr, head2hand);
	
	// remove nans
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cam_Arm_cld_ptr, *cam_Arm_cld_ptr, indices);
	pcl::removeNaNFromPointCloud(*head_Arm_cld_ptr, *head_Arm_cld_ptr, indices);
	
	// remove far away points
	//	pcl::PointCloud<pcl::PointXYZ>::Ptr cam_Arm_filt_ptr( new pcl::PointCloud<pcl::PointXYZ>);
	//	pcl::PointCloud<pcl::PointXYZ>::Ptr head_Arm_filt_ptr( new pcl::PointCloud<pcl::PointXYZ>);
	//	
	//	pcl::PassThrough<pcl::PointXYZ> pass_;
	//	pass_.setFilterFieldName ("x");
	//  	pass_.setFilterLimits (1.2, 2.2);
	//	pass_.setInputCloud (cam_Arm_cld_ptr);
	//  	pass_.filter (*cam_Arm_filt_ptr);
	//  	
	//  	pass_.setInputCloud (head_Arm_filt_ptr);
	//  	pass_.filter (*head_Arm_filt_ptr);
  	
	// icp
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setMaxCorrespondenceDistance(0.1);
	icp.setTransformationEpsilon(1e-6);	// transformation convergence epsilon	
	icp.setInputCloud(cam_Arm_cld_ptr);
	icp.setInputTarget(head_Arm_cld_ptr);
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
	icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;
	
	// Save clouds for viewing
	pcl::PCDWriter writer;
	writer.writeASCII( "/home/natanaso/Desktop/cam.pcd", *cam_Arm_cld_ptr );
	writer.writeASCII( "/home/natanaso/Desktop/head.pcd", *head_Arm_cld_ptr );
	writer.writeASCII( "/home/natanaso/Desktop/final.pcd", Final );
	Final += *head_Arm_cld_ptr;
	writer.writeASCII( "/home/natanaso/Desktop/combined.pcd", Final );
	return 0;
}

int main(int argc, char **argv)
{
	return trec(argc, argv);
}
