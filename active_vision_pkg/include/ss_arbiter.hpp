#ifndef _SS_ARBITER_HPP_
#define _SS_ARBITER_HPP_

// Standard
#include <iostream>
#include <vector>
#include <map>
#include <Eigen/Core>
#include <boost/lexical_cast.hpp>

// Pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>

// Ros
#include <tf/transform_listener.h>

// Actionlib
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <active_vision_pkg/GoToPoseAction.h>


// Custom
#include <misc.hpp>					// quaternion operations
#include <metric_map.hpp>			// map of the table
#include <ood_visualization.hpp>	// visualization of history data
#include "obj.hpp"
#include "ss_decider.hpp"
#include <tabletop_analyzer.hpp>


class ss_arbiter
{
public:
	enum Proc_State{ CHOOSE,
					 	  DECIDE,
					 	  LOOK_AROUND };
	
	enum Move_State{ WAIT_FOR_GOAL,
					 GOAL_REACHED };
	

					 
private:
	Proc_State ps;
	Move_State ms;
	
	// Current sensor info
	Eigen::Vector3d position;
	Eigen::Vector4d orientation;
	//tf::StampedTransform kinect2map;
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
	std::string fx_frm;
	//std::string snsr_frm;
	std::string opt_frm;
	

	// Map of the table
	metric_map table_map;								// used to convert meters to cells
	std::map< std::pair<int,int>, boost::shared_ptr<obj> > hyp_map;	// used to store the objects
	pcl::PointCloud<pcl::PointXY>::Ptr hyp_map_centroids;
	std::map< int, boost::shared_ptr<obj> > obj_list;
	int obj_id_ptr_;
	tabletop_analyzer TA;								// used to segment the objects from the table
	
	// Planning
	ss_decider decider;
	int PROC_OBJ_ID;
	std::vector< std::pair<double,int> > objs_around;
	
	struct dec_memory{
		Eigen::Vector3d goal_position;
		Eigen::Vector4d goal_orientation;
	}DCM;
		
	// display
	ood_visualization ov;
	
	// counting number of iterations
	int step_cntr;
	
	// Ros stuff
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
	actionlib::SimpleActionClient<active_vision_pkg::GoToPoseAction> ac_;
	
protected:
	virtual ros::NodeHandle& getNodeHandle() { return nh_; }
  	virtual ros::NodeHandle& getPrivateNodeHandle() { return private_nh_; }
  

	
public:
	// Constructor
	ss_arbiter( ros::NodeHandle & nh );
	
	// Destructor
	~ss_arbiter(){}	
	
	// NEEDS TO BE CALLED BEFORE ANY OTHER FUNCTIONS
	void ss_arbiter_init();
	void ss_arbiter_spin();
	
	
	void cloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud)
	{	
		// We receive continous clouds from the kinect here but will not use them!
		//ROS_INFO("CLOUD CALLBACK CLOUD CALLBACK CLOUD CALLBACK !!!");
		if( ( !cloud_updated ) && ( goal_completion_time + ros::Duration(2.0) < cloud->header.stamp ) )
		{

			try
			{
				// update the pose
				listener.waitForTransform( opt_frm, fx_frm, cloud->header.stamp, ros::Duration(5.0));
				listener.lookupTransform( fx_frm, opt_frm, cloud->header.stamp, optical2map);
				
				/*
				// Add artificial noise to the optical2map transoform
				tf::Transform errorT;
				errorT.setOrigin( tf::Vector3(misc::uniform_cont(-0.1, 0.1),
														misc::uniform_cont(-0.01, 0.01),
														misc::uniform_cont(-0.1, 0.1)));
				tf::Quaternion errorQ;
				errorQ.setRPY( misc::uniform_cont(-0.1,0.1),
									misc::uniform_cont(-0.1,0.1),
									misc::uniform_cont(-0.1,0.1));
				errorT.setRotation(errorQ);
				optical2map *= errorT;
				
				listener.waitForTransform( snsr_frm, fx_frm, cloud->header.stamp, ros::Duration(5.0));
				listener.lookupTransform( fx_frm, snsr_frm, cloud->header.stamp, kinect2map);
				*/
				
				// ASSUMES optical2map and sensor2map have the same translation!!!
				tf::Vector3 position_( optical2map.getOrigin() );
				position.x() = position_.x();
				position.y() = position_.y();
				position.z() = position_.z();

				// We dont need to look up kinect2map
				tf::Quaternion opt_quat_snsr(0.5, -0.5, 0.5, 0.5);
				tf::Quaternion orientation_( optical2map * opt_quat_snsr );
				orientation.x() = orientation_.x();
				orientation.y() = orientation_.y();
				orientation.z() = orientation_.z();
				orientation.w() = orientation_.w();

				//ROS_INFO_STREAM("position = " << position.transpose() );
				//ROS_INFO_STREAM("orientation = " << orientation.transpose() );

				// update the cloud
				//xyz_cld_ptr = cloud;
				pcl::PointCloud<pcl::PointXYZ>::Ptr cld_tmp(new pcl::PointCloud<pcl::PointXYZ>);
				pcl::copyPointCloud(*cloud, *cld_tmp);	// Do I need to copy it?
				
				
				// cut far away points
				pcl::PassThrough<pcl::PointXYZ> pass_;
				pass_.setFilterFieldName ("z");
				pass_.setFilterLimits (0.1, 2.0);
				pass_.setInputCloud(cld_tmp);
				pass_.filter(*xyz_cld_ptr);

				/*
				// save for viewing
				pcl::PCDWriter writer;
				writer.writeASCII( "/home/natanaso/Desktop/cld_"+
						boost::lexical_cast<std::string>(goal_completion_time.toSec()) +
						".pcd", *xyz_cld_ptr );
				*/
								
				cloud_updated = true;
			}
			catch (tf::TransformException ex){
				ROS_ERROR("%s", ex.what());
			}
		}
	}
	
	/*	
	void poseCallback(const geometry_msgs::PoseStamped::ConstPtr & pose)
	{		
		//ROS_INFO("POSE CALLBACK POSE CALLBACK POSE CALLBACK POSE CALLBACK POSE CALLBACK POSE CALLBACK !!!");
		if( ( !pose_updated ) && ( goal_completion_time + ros::Duration(2.0) < pose->header.stamp ) )
		{
			position.x() = pose->pose.position.x;
			position.y() = pose->pose.position.y;
			position.z() = pose->pose.position.z;

			orientation.x() = pose->pose.orientation.x;
			orientation.y() = pose->pose.orientation.y;
			orientation.z() = pose->pose.orientation.z;
			orientation.w() = pose->pose.orientation.w;
			
			pose_updated = true;
		}
	}
	*/

	// Goal
	void send_goal(const Eigen::Vector3d & goal_position, const Eigen::Vector4d & goal_orientation);
	
	// Called once when the goal becomes active
	void activeCb(){
		ROS_INFO("Goal just went active");
	}

	// Called every time feedback is received for the goal
	void feedbackCb(const active_vision_pkg::GoToPoseFeedbackConstPtr& feedback){
		ROS_INFO("Got Feedback");
	}
	
	void doneCb( const actionlib::SimpleClientGoalState& state,
		  		 	 const active_vision_pkg::GoToPoseResultConstPtr& result);
		  		 
	// Display
	void display_all_data( const Eigen::Vector3d & position, 
						   const Eigen::Vector4d & orientation, 
						   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cld_ptr)
	{
		//ROS_INFO_STREAM("DISPALL DATA position " << position.transpose() << " to ood vis");
		//ROS_INFO_STREAM("DISPALL DATA orientation " << orientation.transpose() << " to ood vis");		
		ov.add_curr_pose(position, orientation);
		ov.add_hist_pose(position, orientation);
		ov.add_curr_cloud(cld_ptr);

		// Display the centroids and the bounding boxes
		int num_obj = obj_list.size();
		Eigen::MatrixXd mat_3xN(3, num_obj);
		Eigen::MatrixXd maxp_3xN(3, num_obj);
		Eigen::MatrixXd minp_3xN(3, num_obj);
		Eigen::MatrixXf color_3xN(3, num_obj);
		Eigen::VectorXd obj_ids(num_obj);
		{
			int k = 0;
			for( std::map< int, boost::shared_ptr<obj> >::iterator it = obj_list.begin();
					it != obj_list.end(); ++it, ++k)
			{
				mat_3xN.col(k) = ((it->second) -> get_position());
				maxp_3xN.col(k) = ((it->second) -> get_maxp());
				minp_3xN.col(k) = ((it->second) -> get_minp());
				color_3xN.col(k) = ((it->second) -> get_color());
				obj_ids(k) = ((it->second) -> get_id());
			}
		}

		ov.add_centroids( mat_3xN );
		ov.add_bbox( maxp_3xN, minp_3xN, color_3xN );
		ov.add_ids( maxp_3xN, obj_ids );
		ROS_WARN("Num boxes %d", minp_3xN.cols());
	}
	
private:
	//**************************************************************************************
	//** PROCESSING FUNCTIONS
	//**************************************************************************************
	bool process( 	Eigen::Vector3d & proc_position, Eigen::Vector4d & proc_orientation, 
					pcl::PointCloud<pcl::PointXYZ>::Ptr proc_cloud_ptr,
					Eigen::Vector3d & goal_position, Eigen::Vector4d & goal_orientation );

	bool process_fake( Eigen::Vector3d & proc_position, Eigen::Vector4d & proc_orientation, 
					  	pcl::PointCloud<pcl::PointXYZ>::Ptr proc_cloud_ptr,
					  	Eigen::Vector3d & goal_position, Eigen::Vector4d & goal_orientation);

	bool process_fake_2( Eigen::Vector3d & proc_position, Eigen::Vector4d & proc_orientation, 
					  	pcl::PointCloud<pcl::PointXYZ>::Ptr proc_cloud_ptr,
					  	Eigen::Vector3d & goal_position, Eigen::Vector4d & goal_orientation);

	bool process_choose_state( Eigen::Vector3d & proc_position, Eigen::Vector4d & proc_orientation, 
								pcl::PointCloud<pcl::PointXYZ>::Ptr proc_cloud_ptr,
								Eigen::Vector3d & goal_position, Eigen::Vector4d & goal_orientation);
								
	bool process_decide_state( Eigen::Vector3d & proc_position, Eigen::Vector4d & proc_orientation, 
										pcl::PointCloud<pcl::PointXYZ>::Ptr proc_cloud_ptr,
										Eigen::Vector3d & goal_position, Eigen::Vector4d & goal_orientation);
					
	bool process_decide_state_fake( Eigen::Vector3d & proc_position, Eigen::Vector4d & proc_orientation, 
				  							pcl::PointCloud<pcl::PointXYZ>::Ptr proc_cloud_ptr,
				  							Eigen::Vector3d & goal_position, Eigen::Vector4d & goal_orientation );

	bool process_look_state( Eigen::Vector3d & proc_position, Eigen::Vector4d & proc_orientation, 
		  							pcl::PointCloud<pcl::PointXYZ>::Ptr proc_cloud_ptr,
		  							Eigen::Vector3d & goal_position, Eigen::Vector4d & goal_orientation );
					  							

	ss_decider::Decider_State update_object( int obj_id, Eigen::Vector3d & proc_position, const ros::Time & tf_time,
														  Eigen::Vector3d & goal_position, Eigen::Vector4d & goal_orientation );
	
	bool transform_cloud( pcl::PointCloud<pcl::PointXYZ>::Ptr & cld_in_ptr, 
								 pcl::PointCloud<pcl::PointXYZ>::Ptr & cld_out_ptr, 
								 const ros::Time & tf_time );

	void update_original_cloud( pcl::PointCloud<pcl::PointXYZ>::Ptr & cld_ptr );

	void update_obj_map( tabletop_analyzer::TableInfo & ti );
	
	/*
	void update_obj_map( std::vector<tabletop_analyzer::TableInfo>::iterator ti_start,
								std::vector<tabletop_analyzer::TableInfo>::iterator ti_end );
	*/

	void get_test_surf( boost::shared_ptr<obj> & OBJ_ptr, 
							  pcl::PointCloud<pcl::PointXYZ>::Ptr & proc_cloud_ptr,
							  pcl::PointCloud<pcl::PointXYZ>::Ptr & test_surf_ptr );

	void renew_obj_map( pcl::ModelCoefficients::Ptr & table_coeffs );

	void update_disp_cloud( );
	void update_disp_cloud_fake( pcl::PointCloud<pcl::PointXYZ>::Ptr & proc_cloud_ptr );
};

#endif
