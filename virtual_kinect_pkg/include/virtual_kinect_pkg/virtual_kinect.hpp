#ifndef VIRTUAL_KINECT_HPP_
#define VIRTUAL_KINECT_HPP_

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>

// PCL_ROS
#include <pcl_ros/publisher.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

// Actionlib
#include <actionlib/server/simple_action_server.h>
#include <virtual_kinect_pkg/GoToPoseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

// Pcl
//#include <pcl/io/pcd_io.h>

// CUSTOM
#include "virtual_kinect_pkg/vkin_offline.hpp"


class virtual_kinect : public vkin_offline
{

private:
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
	
	// communication
	ros::Publisher pose_pub;
	ros::Publisher cloud_pub;
	
protected:
	virtual ros::NodeHandle& getNodeHandle() { return nh_; }
  	virtual ros::NodeHandle& getPrivateNodeHandle() { return private_nh_; }
	
	// NodeHandle instance must be created before this line. Otherwise strange error occurs.
	std::string action_name_;
	actionlib::SimpleActionServer<virtual_kinect_pkg::GoToPoseAction> as_;
	virtual_kinect_pkg::GoToPoseResult result_;
	
	actionlib::SimpleActionClient<virtual_kinect_pkg::GoToPoseAction> ac_;
	
public:
	virtual_kinect( const Eigen::Vector3d position, const Eigen::Vector4d orientation,
					bool obj_coord, bool organized, bool add_noise );

	~virtual_kinect(){
		//ROS_INFO("Shutting down virtual_kinect node!");
		pose_pub.shutdown();
		cloud_pub.shutdown();
	}
	
	void spin_vkin();
	void actionCallback(const virtual_kinect_pkg::GoToPoseGoalConstPtr &goal);
	
	// For testing purposes:
	void send_goal(const Eigen::Vector3d & goal_position, 
			   	   const Eigen::Vector4d & goal_orientation);
			   	   
	void doneCb(const actionlib::SimpleClientGoalState& state,
	   const virtual_kinect_pkg::GoToPoseResultConstPtr& result);
	/**/   
private:
							 
	
	void publish_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);
	void publish_pose();
	

};

#endif
