// Ros
#include <ros/ros.h>
#include <pcl_ros/publisher.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

// Pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>



pcl::PointCloud<pcl::PointXYZ>::Ptr omap_cld_ptr(new pcl::PointCloud<pcl::PointXYZ>);
bool omap_cld_up = false;

void omap_cld_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud)
{	
	if( !omap_cld_up )
	{
		ROS_INFO("Updating octomap cloud");
		omap_cld_up = true;
		pcl::copyPointCloud(*cloud, *omap_cld_ptr);
	}
}



int osave(int argc, char **argv)
{
	ros::init(argc, argv, "octomap_saver");
	ros::NodeHandle nh_;
	ros::Subscriber omap_cld_sub = nh_.subscribe("/octomap_point_cloud_centers", 1, &omap_cld_callback);

	ROS_INFO("Waiting for cloud...");
	while( !omap_cld_up )
		ros::spinOnce();
	ROS_INFO("Cloud received!");
	

	// Save cloud for viewing
	pcl::PCDWriter writer;
	writer.writeASCII( "/home/natanaso/Desktop/octomap.pcd", *omap_cld_ptr );
	
	
	return 0;
}

int main(int argc, char **argv)
{
	return osave(argc, argv);
}
