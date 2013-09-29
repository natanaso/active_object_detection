#ifndef OOD_VISUALIZATION_H
#define OOD_VISUALIZATION_H

// Standard
#include <vector>

// Ros
#include <ros/ros.h>
//#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

//#include <sensor_msgs/PointCloud2.h>

// PCL includes
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include <pcl/visualization/pcl_visualizer.h>

// pcl_ros
#include <pcl_ros/publisher.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>


//http://gt-ros-pkg.googlecode.com/svn/trunk/hrl/hrl_hardware_drivers/phantom_omni/src/omni.cpp



// Three versions
// 1. Call back version
// 2. Latched many topics
// 3. Latched 1 topic with growing data

class ood_visualization{

	public:
		// Typdefs
		typedef pcl::PointXYZ PointT;
		typedef pcl::PointXYZI PointL;
		typedef pcl::PointXYZRGB PointRGB;
		
		typedef pcl::PointCloud<PointT> PointCloud;
		typedef pcl::PointCloud<PointRGB> PointCloudRGB;
		
		typedef geometry_msgs::PoseStamped PoseS;
		typedef visualization_msgs::Marker PoseM;
		
	protected:
		ros::NodeHandle nh_;
		
	private:
		int ver;
		
		// Current pose publisher
		ros::Publisher curr_pose_pub;
		ros::Publisher curr_cloud_pub;
		
		// Version 1 & 2 Data
		std::vector<ros::Publisher> rgbcloud_pub_vec;
		std::vector<PointCloudRGB::Ptr> rgbcloud_vec;
		
		std::vector<PoseM> pose_vec;
		std::vector<ros::Publisher> pose_pub_vec;
		
		void connectCallback_pc(const ros::SingleSubscriberPublisher& pub);	// Cloud publisher callback
		void connectCallback_pp(const ros::SingleSubscriberPublisher& pub);	// Pose publisher callback
		void disconnectCallback(const ros::SingleSubscriberPublisher& pub);
		
		
		// Version 3 Data
		int hist_length;
		PoseM pose_hist;
		bool is_init_pose_hist_pub;
		ros::Publisher pose_hist_pub;
		
		PointCloudRGB cloud_hist;
		bool is_init_cloud_hist_pub;
		ros::Publisher cloud_hist_pub;
		

		// MarkerArray Display
		ros::Publisher marray_pub;		
		ros::Publisher bbox_pub;
		ros::Publisher id_pub;
		ros::Publisher curr_view_pub;
		
		
		// Additional clouds
		ros::Publisher tst_cld_pub1;
		ros::Publisher tst_cld_pub2;
		ros::Publisher tst_cld_pub3;
		
		// Private Functions
		void add_hist_cloud_v1(PointCloudRGB::Ptr cloud);

		template <typename T>
		void add_hist_pose_v1(Eigen::Matrix<T,3,1> const &position, Eigen::Matrix<T,4,1> const &orientation);

		void add_hist_cloud_v2(PointCloudRGB::Ptr cloud);

		template <typename T>
		void add_hist_pose_v2(Eigen::Matrix<T,3,1> const &position, Eigen::Matrix<T,4,1> const &orientation);
		
		void add_hist_cloud_v3(PointCloudRGB::Ptr cloud);

		template <typename T>
		void add_hist_pose_v3(Eigen::Matrix<T,3,1> const &position, Eigen::Matrix<T,4,1>const &orientation);

		void init_pose_hist_pub();
		void init_cloud_hist_pub();
		
	public:
		
		ood_visualization(ros::NodeHandle &anode, int version);
		~ood_visualization();
		
		
		void add_curr_cloud(PointCloudRGB::Ptr cloud);
		
		void add_hist_cloud(PointCloudRGB::Ptr cloud);
		
		template <typename T>
		void add_hist_pose(Eigen::Matrix<T,3,1> const & position, Eigen::Matrix<T,4,1> const & orientation)
		{
			switch(ver){
				case 1:
					add_hist_pose_v1(position,orientation);
					break;
			
				case 2:
					add_hist_pose_v2(position,orientation);
					break;
			
				case 3:
					if(!is_init_pose_hist_pub)
						init_pose_hist_pub();

					add_hist_pose_v3(position,orientation);
					break;
			}
		}
		
		template <typename T>
		void add_curr_pose(Eigen::Matrix<T,3,1> const & position, Eigen::Matrix<T,4,1> const & orientation)
		{
			ood_visualization::PoseS pose;
			
			pose.pose.position.x = position.x();
			pose.pose.position.y = position.y();
			pose.pose.position.z = position.z();

			pose.pose.orientation.x = orientation.x();
			pose.pose.orientation.y = orientation.y();
			pose.pose.orientation.z = orientation.z();
			pose.pose.orientation.w = orientation.w();
	
			pose.header.frame_id = "/map";
			//pose.header.stamp = ros::Time::now ();
			pose.header.stamp = ros::Time(0);
			curr_pose_pub.publish(pose);
		}
		
		void add_centroids( const Eigen::MatrixXd & mat_3xN );
		void add_bbox( const Eigen::MatrixXd & maxp_3xN,
							 const Eigen::MatrixXd & minp_3xN,
							 const Eigen::MatrixXf & colors );
		void add_ids( const Eigen::MatrixXd & maxp_3xN,
						  const Eigen::VectorXd & obj_ids );
						  
		void add_curr_view( pcl::PointCloud<pcl::PointXYZ>::Ptr & cld_ptr);
		
		void add_tst_cld1( pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cld_ptr)
		{
			cld_ptr->header.frame_id = "/map";
  			cld_ptr->header.stamp = ros::Time(0);	// ensure it does not get deleted by rviz
			tst_cld_pub1.publish(*cld_ptr);
		}
		
		void add_tst_cld2( pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cld_ptr)
		{
			cld_ptr->header.frame_id = "/map";
  			cld_ptr->header.stamp = ros::Time(0);	// ensure it does not get deleted by rviz
			tst_cld_pub2.publish(*cld_ptr);
		}
		
		void add_tst_cld3( pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cld_ptr)
		{
			cld_ptr->header.frame_id = "/map";
  			cld_ptr->header.stamp = ros::Time(0);	// ensure it does not get deleted by rviz
			tst_cld_pub3.publish(*cld_ptr);
		}
};

#endif
