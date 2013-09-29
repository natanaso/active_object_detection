#include <pcl/filters/voxel_grid.h>

#include "gazebo_simulation/pr2_client.hpp"

// process_fake()
#include <ros/package.h>
#include <io_utils.hpp>



pr2_client::pr2_client( ros::NodeHandle & nh )
	: ps( CHOOSE ), ms( GOAL_REACHED ),
	  xyz_cld_ptr(new pcl::PointCloud<pcl::PointXYZ>),
	  xyz_cld_ptr_original(new pcl::PointCloud<pcl::PointXYZ>),
	  disp_cld_ptr(new pcl::PointCloud<pcl::PointXYZRGB>),
	  cloud_updated(false), goal_completion_time( ros::Time::now() ),
	  listener( ros::Duration(60.0) ),		// tf cache length in sec
	  ov(nh,3), step_cntr(0), nh_(nh), private_nh_("~"), ac_("go_to_pose_act", true)	  
{
	srand((unsigned)time(NULL));
}

void 
pr2_client::pr2_client_init()
{
	cloud_sub = nh_.subscribe( "/arm_kinect/arm_kinect/depth/points", 1, &pr2_client::cloudCallback, this);
			
	// wait for the action server to start
	ROS_INFO("Waiting for action server to start.");
	ac_.waitForServer(); //will wait for infinite time
	ROS_INFO("Action server started!");
}

void
pr2_client::pr2_client_spin()
{
	switch(ms){
		case WAIT_FOR_GOAL:
			ros::spinOnce();	// Waiting for goal get position and clouds
			break;

		case GOAL_REACHED:{

			// Timing
			clock_t tic;
			double toc;

			// Goal
			Eigen::Vector3d goal_position;
			Eigen::Vector4d goal_orientation;
			
			/*
			// Update pose and cloud
			ROS_INFO("Waiting for cloud...");
			while( !cloud_updated )
				ros::spinOnce();
			ROS_INFO("Cloud received!");
			*/
			
			// Convert cloud to /map frame.  
			pcl::PointCloud<pcl::PointXYZ>::Ptr map_cld_ptr( new pcl::PointCloud<pcl::PointXYZ>);
			pcl_ros::transformPointCloud( *xyz_cld_ptr, *map_cld_ptr, optical2map);
			map_cld_ptr->header.frame_id = "/map";
			map_cld_ptr->header.stamp = xyz_cld_ptr->header.stamp;

			
			// Get the processing cloud and ensure it has a correct time stamp!
			pcl::PointCloud<pcl::PointXYZ>::Ptr proc_cloud_ptr( map_cld_ptr );
			proc_cloud_ptr -> header.stamp = xyz_cld_ptr-> header.stamp;


			// Process
			ROS_INFO("Processing...");
			tic = clock();		
			bool need_to_move = process_fake( position, orientation, proc_cloud_ptr,
										 	  	  		 goal_position, goal_orientation );							
			toc = (clock()-tic)*1000/CLOCKS_PER_SEC;
			++step_cntr;
			std::cout << "It took " << toc << " msec to process!" << std::endl;

			//need_to_move = false;


			/*
			// Display
			tic = clock();
			update_disp_cloud_fake( proc_cloud_ptr );
			display_all_data( position, orientation, disp_cld_ptr );
			toc = (clock()-tic)*1000/CLOCKS_PER_SEC;
			std::cout << "It took " << toc << " msec to display!" << std::endl;
			*/
								
			if(need_to_move){
				send_goal( goal_position, goal_orientation );
				ms = WAIT_FOR_GOAL;
			}else{
				goal_completion_time = ros::Time::now();
				cloud_updated = false;
			}							

			break;
		}
		default:
			std::cout << "Unknown move state" << std::endl;
			break;
	}
}


bool
pr2_client::process_fake( Eigen::Vector3d & proc_position, Eigen::Vector4d & proc_orientation, 
					  			  pcl::PointCloud<pcl::PointXYZ>::Ptr proc_cloud_ptr,
					  			  Eigen::Vector3d & goal_position, Eigen::Vector4d & goal_orientation)
{
	ROS_INFO("Processing step %d!", step_cntr);	
	//ros::Duration(1).sleep(); // sleep for 2 seconds
	bool need_to_move;
	
	//static Eigen::Vector3d table_origin(0.98, 0, 0.585);
	static Eigen::Vector3d table_origin(1.475, 0.175, 0.724);
	static int rowid = 0;
	static Eigen::MatrixXd plan_vps;	
	static std::string avp_root_dir( ros::package::getPath("active_vision_pkg") );
	
	if(rowid == 0)
		io_utils::file2matrix( avp_root_dir + "/data/plan_vps.txt", plan_vps, 3 );

	if(( step_cntr % 1 == 0 ) && ( rowid < plan_vps.rows() ))
	{
		goal_position = table_origin + plan_vps.row(rowid).transpose();
		goal_orientation << misc::target2quat( goal_position, table_origin );
		++rowid;
		//goal_position = Eigen::Vector3d( misc::uniform_cont(-2.0, 2.0), 
		//			misc::uniform_cont(-2.0, 2.0), misc::uniform_cont(0.0, 1.5) );
		//goal_orientation = misc::target2quat( goal_position, Eigen::Vector3d(0.0,0.0,0.0) );
		need_to_move = true;
	}
	else{
		goal_position = proc_position;
		goal_orientation = proc_orientation;
		need_to_move = false;
	}

	/*
	switch(step_cntr){
		case 3:
			goal_position = table_origin + Eigen::Vector3d(1.5,1.5,0.25);
			goal_orientation << misc::target2quat( goal_position, table_origin );
			need_to_move = true;
			break;
		case 6:
			goal_position = table_origin + Eigen::Vector3d(-1.5,-1.5,0.25);
			goal_orientation << misc::target2quat( goal_position, table_origin );
			need_to_move = true;
			break;
		case 9:
			goal_position = table_origin + Eigen::Vector3d(-0.3,0,1);
			goal_orientation << misc::target2quat( goal_position, table_origin );
			need_to_move = true;
			break;
		default:
			goal_position = proc_position;
			goal_orientation = proc_orientation;
			need_to_move = false;
	}
	*/

	return need_to_move;
}



void 
pr2_client::update_disp_cloud_fake( pcl::PointCloud<pcl::PointXYZ>::Ptr & proc_cloud_ptr )
{
	//disp_cld_ptr->clear();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_cld_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud( *proc_cloud_ptr, *tmp_cld_ptr );
	*disp_cld_ptr += *tmp_cld_ptr;
	pcl::VoxelGrid<pcl::PointXYZRGB> grid_filter_;
	double cell_size = 0.05;	//leaf size in meters
	grid_filter_.setLeafSize (cell_size, cell_size, cell_size);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cld( new pcl::PointCloud<pcl::PointXYZRGB> );
	grid_filter_.setInputCloud( disp_cld_ptr );
	grid_filter_.filter ( *filtered_cld );
	disp_cld_ptr = filtered_cld;
}


void 
pr2_client::send_goal(const Eigen::Vector3d & goal_position, 
					  		 const Eigen::Vector4d & goal_orientation)
{
	Eigen::Vector3d target = misc::quat2target( goal_position, goal_orientation);
	Eigen::Vector3d dir_vec = target-goal_position;
	dir_vec = dir_vec / dir_vec.norm();
	Eigen::VectorXd offsets(11);
	offsets << 0, -0.1, 0.1, -0.2, 0.2, -0.3, 0.3, -0.4, 0.4, -0.5, 0.5;	// positive is closer to goal

	gazebo_simulation::GoToPoseGoal goal;
	goal.goal_pose_arr.resize(offsets.size());

	for(int k = 0; k < offsets.size(); ++k)
	{
		goal.goal_pose_arr[k].pose.position.x = goal_position.x() + offsets(k)*dir_vec.x();
		goal.goal_pose_arr[k].pose.position.y = goal_position.y() + offsets(k)*dir_vec.y();
		goal.goal_pose_arr[k].pose.position.z = goal_position.z() + offsets(k)*dir_vec.z();
		goal.goal_pose_arr[k].pose.orientation.x = goal_orientation.x();
		goal.goal_pose_arr[k].pose.orientation.y = goal_orientation.y();
		goal.goal_pose_arr[k].pose.orientation.z = goal_orientation.z();
		goal.goal_pose_arr[k].pose.orientation.w = goal_orientation.w();
		goal.goal_pose_arr[k].header.frame_id = "/map";
		goal.goal_pose_arr[k].header.stamp = ros::Time::now();
	}

	// send the goal to the action server
	ac_.sendGoal(goal, boost::bind(&pr2_client::doneCb, this, _1, _2),
            actionlib::SimpleActionClient<gazebo_simulation::GoToPoseAction>::SimpleActiveCallback(),
            actionlib::SimpleActionClient<gazebo_simulation::GoToPoseAction>::SimpleFeedbackCallback());

	// send tf transform for display purposes
	
}

void 
pr2_client::doneCb( const actionlib::SimpleClientGoalState& state,
	  			 		  const gazebo_simulation::GoToPoseResultConstPtr& result )
{
	ROS_INFO("Finished in state [%s]", state.toString().c_str());
			
	if(result->success){
		goal_completion_time = ros::Time::now();
		ms = GOAL_REACHED;
		cloud_updated = false;
	}
	else{
		ROS_INFO("Goal pose was not reached! Don't know what to do!");
	}
}

int pr2_client_main(int argc, char **argv)
{
	ros::init (argc, argv, "pr2_client");
	ros::NodeHandle nh;
	pr2_client p2c( nh );
	p2c.pr2_client_init();
	
	//ros::Rate loop_rate(0.1);
	while ( nh.ok() )
	{
		p2c.pr2_client_spin();
				
		ros::spinOnce();
		//loop_rate.sleep();
	}
	
	return 0;
}

int main(int argc, char **argv)
{
	return pr2_client_main(argc, argv);
}
