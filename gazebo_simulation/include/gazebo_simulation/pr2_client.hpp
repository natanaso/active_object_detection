#ifndef _PR2_CLIENT_HPP_
#define _PR2_CLIENT_HPP_

// Standard
#include <iostream>
#include <vector>
#include <Eigen/Core>

// Pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

// Ros
#include <tf/transform_listener.h>

// Actionlib
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <gazebo_simulation/GoToPoseAction.h>

// Custom
#include <misc.hpp>					// quaternion operations
#include <ood_visualization.hpp>	// visualization of history data


class pr2_client
{
public:
	enum Proc_State{ CHOOSE,
					 DECIDE };
	
	enum Move_State{ WAIT_FOR_GOAL,
					 GOAL_REACHED };
					 
private:
	Proc_State ps;
	Move_State ms;

	// Current sensor info
	Eigen::Vector3d position;
	Eigen::Vector4d orientation;
	tf::StampedTransform kinect2map;
	tf::StampedTransform optical2map;
	pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cld_ptr;					// cloud continuously updated by the kinect
	pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cld_ptr_original;		// ICP-ed cloud since the beginning of time
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr disp_cld_ptr;				// ICP-ed cloud since the beginning of time

	// Communication
	bool cloud_updated;
	ros::Subscriber cloud_sub;
	ros::Time goal_completion_time;
	// The listener receives tf transformations and buffers them for up to 10 seconds
	tf::TransformListener listener;

	// display
	ood_visualization ov;

	// counting number of iterations
	int step_cntr;

	// Ros stuff
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
	actionlib::SimpleActionClient<gazebo_simulation::GoToPoseAction> ac_;
	
protected:
	virtual ros::NodeHandle& getNodeHandle() { return nh_; }
  	virtual ros::NodeHandle& getPrivateNodeHandle() { return private_nh_; }


public:
	// Constructor
	pr2_client( ros::NodeHandle & nh );
	
	// Destructor
	~pr2_client(){}	
	
	// NEEDS TO BE CALLED BEFORE ANY OTHER FUNCTIONS
	void pr2_client_init();
	void pr2_client_spin();

	
	void cloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud)
	{	
		if( ( !cloud_updated ) && ( goal_completion_time + ros::Duration(2.0) < cloud->header.stamp ) )
		{			
			try
			{
				// update the pose
				listener.waitForTransform( "/arm_kinect_frame", "/map", cloud->header.stamp, ros::Duration(5.0));
				listener.lookupTransform( "/map", "/arm_kinect_frame", cloud->header.stamp, kinect2map);
				
				listener.waitForTransform( "/arm_kinect_optical_frame", "/map", cloud->header.stamp, ros::Duration(5.0));
				listener.lookupTransform( "/map", "/arm_kinect_optical_frame", cloud->header.stamp, optical2map);

				tf::Vector3 position_( kinect2map.getOrigin() );
				position.x() = position_.x();
				position.y() = position_.y();
				position.z() = position_.z();

				tf::Quaternion orientation_( kinect2map.getRotation() );
				orientation.x() = orientation_.x();
				orientation.y() = orientation_.y();
				orientation.z() = orientation_.z();
				orientation.w() = orientation_.w();

				ROS_INFO_STREAM("position = " << position.transpose() );
				ROS_INFO_STREAM("orientation = " << orientation.transpose() );

				// update the cloud
				pcl::copyPointCloud(*cloud, *xyz_cld_ptr);	// Do I need to copy it?
				//xyz_cld_ptr = cloud;
				cloud_updated = true;
			}
			catch (tf::TransformException ex){
				ROS_ERROR("%s", ex.what());
			}
			
		}
	}
			
	// Goal
	void send_goal(const Eigen::Vector3d & goal_position, const Eigen::Vector4d & goal_orientation);
	
	// Called once when the goal becomes active
	void activeCb(){
		ROS_INFO("Goal just went active");
	}

	// Called every time feedback is received for the goal
	void feedbackCb(const gazebo_simulation::GoToPoseFeedbackConstPtr& feedback){
		ROS_INFO("Got Feedback");
	}
	
	void doneCb( const actionlib::SimpleClientGoalState& state,
		  		 	 const gazebo_simulation::GoToPoseResultConstPtr& result);
		  		 
	// Display
	void display_all_data( const Eigen::Vector3d & position, 
						   const Eigen::Vector4d & orientation, 
						   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cld_ptr)
	{		
		ov.add_curr_pose(position, orientation);
		ov.add_hist_pose(position, orientation);
		ov.add_curr_cloud(cld_ptr);
	}

private:
	bool process_fake( Eigen::Vector3d & proc_position, Eigen::Vector4d & proc_orientation, 
					  	pcl::PointCloud<pcl::PointXYZ>::Ptr proc_cloud_ptr,
					  	Eigen::Vector3d & goal_position, Eigen::Vector4d & goal_orientation);

	void update_disp_cloud_fake( pcl::PointCloud<pcl::PointXYZ>::Ptr & proc_cloud_ptr );

};

#endif
