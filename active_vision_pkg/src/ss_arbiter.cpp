// Ros
#include <ros/package.h>
//#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>

// Boost
#include <boost/lexical_cast.hpp>

// PCL
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>

// Custom
#include "ss_arbiter.hpp"
#include <misc.hpp>
#include <pcd_utils.hpp>

ss_arbiter::ss_arbiter( ros::NodeHandle & nh )
	: ps( CHOOSE ), ms( GOAL_REACHED ),
	  xyz_cld_ptr(new pcl::PointCloud<pcl::PointXYZ>),
	  xyz_cld_ptr_original(new pcl::PointCloud<pcl::PointXYZ>),
	  disp_cld_ptr(new pcl::PointCloud<pcl::PointXYZRGB>),
	  cloud_updated(false), goal_completion_time( ros::Time::now() ),
	  listener( ros::Duration(180.0) ),		// tf cache length in sec
	  table_map(Eigen::Vector2d(-3,-3), Eigen::Vector2d(3,3), Eigen::Vector2d(0.03, 0.03)),
	  hyp_map_centroids( new pcl::PointCloud<pcl::PointXY> ), obj_id_ptr_(0),
	  TA(nh), PROC_OBJ_ID( -1 ), ov(nh,3), step_cntr(0), 
	  nh_(nh), private_nh_("~"), ac_("go_to_pose_act", true)	  
{
	srand((unsigned)time(NULL));
}

void 
ss_arbiter::ss_arbiter_init()
{	
	// Initialize directories for ss_decider
	std::string obj_list_path;
	std::string neg_obj_list_path;
	std::string hyp_path;
	std::string tree_vps_path;
	std::string omap_vps_path;
	std::string plan_vps_path;
	std::string policy_dir_path;
	std::string omap_file_path;
	std::string cmap_file_path;
	std::string database_dir;
	std::string clouds_dir;
	
	std::string pkg_root_dir( ros::package::getPath("active_vision_pkg") );
	std::string vision_module_dir( ros::package::getPath("vision_module") );
	std::string planning_module_dir( ros::package::getPath("planning_module") );
	
	private_nh_.param( "obj_list_path", obj_list_path, pkg_root_dir + "/data/obj_list.txt" );
	private_nh_.param( "neg_obj_list_path", neg_obj_list_path, pkg_root_dir + "/data/neg_obj_list.txt" );
	private_nh_.param( "hyp_path", hyp_path, pkg_root_dir + "/data/hypotheses.txt" );
	private_nh_.param( "tree_vps_path", tree_vps_path, vision_module_dir + "/data/tree_vps.txt" );
	private_nh_.param( "omap_vps_path", omap_vps_path, vision_module_dir + "/data/omap_vps.txt" );
	private_nh_.param( "plan_vps_path", plan_vps_path, pkg_root_dir + "/data/plan_vps.txt" );
	private_nh_.param( "policy_dir_path", policy_dir_path, pkg_root_dir + "/data/sarsop_data" );
	private_nh_.param( "omap_file_path", omap_file_path, pkg_root_dir + "/data/sarsop_data/oMap.txt" );
	private_nh_.param( "cmap_file_path", cmap_file_path, pkg_root_dir + "/data/sarsop_data/cMap.txt" );
	private_nh_.param( "database_dir", database_dir, vision_module_dir + "/data" );
	private_nh_.param( "clouds_dir", clouds_dir, vision_module_dir + "/../database/cloud_data" );
	
	//std::cout << "clouds_dir = " << clouds_dir << std::endl;
	
	decider.start_online( obj_list_path, hyp_path, tree_vps_path, omap_vps_path, plan_vps_path, policy_dir_path, omap_file_path, cmap_file_path, database_dir, clouds_dir );
				
	// subscribe to pose and cloud updates from the virtual kinect
	//pose_sub = nh_.subscribe( "vkinect_pose", 1, &ss_arbiter::poseCallback, this);
	std::string cld_topic_name;
	private_nh_.param( "cld_topic_name", cld_topic_name, std::string("vkinect_cloud") );
	private_nh_.param( "fx_frm", fx_frm, std::string("/map") );
	//private_nh_.param( "snsr_frm", snsr_frm, std::string("/sensor") );
	private_nh_.param( "opt_frm", opt_frm, std::string("/sensor_optical") );
	cloud_sub = nh_.subscribe( cld_topic_name, 1, &ss_arbiter::cloudCallback, this);
		
	// wait for the action server to start
	ROS_INFO("Waiting for action server to start.");
	ac_.waitForServer(); //will wait for infinite time
	ROS_INFO("Action server started!");
}

void
ss_arbiter::ss_arbiter_spin()
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


			//*****
			// 0. Get pose and cloud
			// Wait until time stamps are bigger than goal completion time
			tic = clock();
			while( !cloud_updated )
				ros::spinOnce();
			toc = (clock()-tic)*1000/CLOCKS_PER_SEC;
			std::cout << "It took " << toc << " msec to update the pose and cloud!" << std::endl;

			ROS_INFO("Objects observed so far: %d", static_cast<unsigned int>(obj_list.size()));
			//*****
			// convert cloud to /map frame, TODO: Ensure the transform is correct!
			pcl::PointCloud<pcl::PointXYZ>::Ptr map_cld_ptr( new pcl::PointCloud<pcl::PointXYZ>);
			pcl_ros::transformPointCloud( *xyz_cld_ptr, *map_cld_ptr, optical2map);
			map_cld_ptr->header.frame_id = fx_frm;
			map_cld_ptr->header.stamp = xyz_cld_ptr->header.stamp;
			
			// align map_cld_ptr with the cloud history using ICP
			tic = clock();
			update_original_cloud( map_cld_ptr );
			toc = (clock()-tic)*1000/CLOCKS_PER_SEC;
			std::cout << "It took " << toc << " msec to update the original cloud!" << std::endl;

			// Display original cloud
			ov.add_curr_view( xyz_cld_ptr_original );
			
			// Segment the table out and cluster the objects
			tic = clock();
			std::vector<tabletop_analyzer::TableInfo> ti_vec;			
			ti_vec = TA.detect_tabletop_objects( map_cld_ptr );
			bool table_found = (ti_vec.size() > 0);
			toc = (clock()-tic)*1000/CLOCKS_PER_SEC;
			std::cout << "It took " << toc << " msec to segment table!" << std::endl;

			std::cout << ti_vec.size() << " tables were detected!" << std::endl;
			
			int tab_idx = -1;
			if(table_found){
				int max_obj_num = 0;
				for( int k = 0; k < ti_vec.size(); ++k)
					if( static_cast<int>(ti_vec[k].obj_centroids->size()) > max_obj_num)
					{
						tab_idx = k;
						max_obj_num = ti_vec[k].obj_centroids->size();
					}
					
				if( tab_idx < 0 )
					table_found = false;
			}
			
			if( !table_found )
			{
				/*
				// Save cloud for viewing
				pcl::PCDWriter writer;
				writer.writeASCII( "/home/natanaso/Desktop/cld_"+
						boost::lexical_cast<std::string>(goal_completion_time.toSec()) +
						".pcd", *map_cld_ptr );
				*/
				
				
				// Set the current viewpoint as visited
				// TODO: figure out what to do if no table was found!
				Eigen::Vector3d temp_target = misc::quat2target( position, orientation );
				
				if( PROC_OBJ_ID >= 0 )
				{
					int vp = misc::find_closest_viewpoint( temp_target, decider.get_plan_vps(), position, goal_position, goal_orientation );
					obj_list[PROC_OBJ_ID] -> set_visited_vps(vp, decider.get_num_sta()-1);
				}
				
				goal_position = position + Eigen::Vector3d(0.0,0.0,0.25);
				goal_orientation = misc::target2quat( goal_position, temp_target );
				send_goal( goal_position, goal_orientation );
				ms = WAIT_FOR_GOAL;
				return;
			}

			// Display data
			std::cout << "The table has " 
						 << ti_vec[tab_idx].obj_centroids->size() 
						 << " objects on it!" << std::endl;
			//ov.add_curr_view( ti_vec[tab_idx].obj_cld_ptr );
			
			/*
			tabletop_analyzer::TableInfo & ti = ti_vec[0];
			std::cout << "obj_centroids = " << std::endl;
			for( pcl::PointCloud<pcl::PointXYZ>::iterator it = ti.obj_centroids->begin();
					it != ti.obj_centroids->end(); ++it)
				std::cout << *it << std::endl;

			std::cout << "obj_proj_centroids = " << std::endl;
			for( pcl::PointCloud<pcl::PointXYZ>::iterator it = ti.obj_proj_centroids->begin();
					it != ti.obj_proj_centroids->end(); ++it)
				std::cout << *it << std::endl;
			*/

			// Match the centroids with the existing ones using NNSearch3D
			// update the object centroids, bounding boxes, and surfaces (ti.obj_idx) in hyp_map
			// Run a voxelgrid for each object surface
			tic = clock();
			update_obj_map( ti_vec[tab_idx] );
			toc = (clock()-tic)*1000/CLOCKS_PER_SEC;
			std::cout << "It took " << toc << " msec to update the map!" << std::endl;
						
			renew_obj_map( ti_vec[tab_idx].tab_coefs );

			//*****
			// 3. Display (run through the obj_list and cat the obj_surf clouds into a big display cloud)
			tic = clock();
			update_disp_cloud( );
			//update_disp_cloud_fake( proc_cloud_ptr );
			display_all_data( position, orientation, disp_cld_ptr );
			toc = (clock()-tic)*1000/CLOCKS_PER_SEC;
			std::cout << "It took " << toc << " msec to display!" << std::endl;
			
			
			//*****
			// 2. Process			
			// Get the processing cloud and ensure it has a correct time stamp!
			pcl::PointCloud<pcl::PointXYZ>::Ptr proc_cloud_ptr( ti_vec[0].obj_cld_ptr );
			proc_cloud_ptr -> header.stamp = xyz_cld_ptr-> header.stamp;
			
			tic = clock();		
			bool need_to_move = process( position, orientation, proc_cloud_ptr,
										 	  	  goal_position, goal_orientation );							
			toc = (clock()-tic)*1000/CLOCKS_PER_SEC;
			++step_cntr;
			std::cout << "It took " << toc << " msec to process!" << std::endl;

			// Display again
			tic = clock();
			update_disp_cloud( );
			display_all_data( position, orientation, disp_cld_ptr );
			toc = (clock()-tic)*1000/CLOCKS_PER_SEC;
			std::cout << "It took " << toc << " msec to display!" << std::endl;
			
								
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



void 
ss_arbiter::send_goal(const Eigen::Vector3d & goal_position, 
					  const Eigen::Vector4d & goal_orientation)
{

	Eigen::Vector3d target = misc::quat2target( goal_position, goal_orientation);
	Eigen::Vector3d dir_vec = target-goal_position;
	dir_vec = dir_vec / dir_vec.norm();
	Eigen::VectorXd offsets(11);
	offsets << 0, -0.1, 0.1, -0.2, 0.2, -0.3, 0.3, -0.4, 0.4, -0.5, 0.5;
	
	//Eigen::VectorXd offsets(17);
	// positive is closer to goal
	//offsets << 0, -0.1, 0.1, -0.2, 0.2, -0.3, 0.3, 
	//			-0.4, 0.4, -0.5, 0.5, -0.6, 0.6, -0.7, -0.8, -0.9, -1.0;	

	active_vision_pkg::GoToPoseGoal goal;
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
	ac_.sendGoal(goal, boost::bind(&ss_arbiter::doneCb, this, _1, _2),
            actionlib::SimpleActionClient<active_vision_pkg::GoToPoseAction>::SimpleActiveCallback(),
            actionlib::SimpleActionClient<active_vision_pkg::GoToPoseAction>::SimpleFeedbackCallback());
}


void 
ss_arbiter::doneCb( const actionlib::SimpleClientGoalState& state,
	  			 	const active_vision_pkg::GoToPoseResultConstPtr& result )
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

//******************************************************************
//**************** PROCESSING FUNCTIONS ****************************
//******************************************************************

// true if new goal, false else
bool
ss_arbiter::process( Eigen::Vector3d & proc_position, Eigen::Vector4d & proc_orientation, 
					  		pcl::PointCloud<pcl::PointXYZ>::Ptr proc_cloud_ptr,
					  		Eigen::Vector3d & goal_position, Eigen::Vector4d & goal_orientation)
{
	bool need_to_move = false;
	switch(ps)
	{
		case CHOOSE:
			std::cout << "The sensor state is CHOOSE" << std::endl;
			need_to_move = process_choose_state( proc_position, proc_orientation, proc_cloud_ptr,
							   			  	   		 goal_position, goal_orientation );
			break;
			
		case DECIDE:
			std::cout << "The sensor state is DECIDE with object id " << PROC_OBJ_ID << std::endl;
			need_to_move = process_decide_state( proc_position, proc_orientation, proc_cloud_ptr,
							     				 goal_position, goal_orientation);			
			break;
			
		case LOOK_AROUND:
			std::cout << "The sensor state is LOOK_AROUND with object id " << PROC_OBJ_ID << std::endl;
			need_to_move = process_look_state( proc_position, proc_orientation, proc_cloud_ptr,
							     				 goal_position, goal_orientation);
			break;
			
		default:
			std::cout << "Unknown state" << std::endl;
			break;
	}

	return need_to_move;
}

bool
ss_arbiter::process_fake( Eigen::Vector3d & proc_position, Eigen::Vector4d & proc_orientation, 
					  			  pcl::PointCloud<pcl::PointXYZ>::Ptr proc_cloud_ptr,
					  			  Eigen::Vector3d & goal_position, Eigen::Vector4d & goal_orientation)
{
	ROS_INFO("Processing!");	
	//ros::Duration(1).sleep(); // sleep for 2 seconds
	bool need_to_move;
	
	Eigen::Vector3d table_origin(0, 0, 0);
	switch(step_cntr){
		case 10:
			goal_position = table_origin + Eigen::Vector3d(1.5,1.5,0.25);
			goal_orientation << misc::target2quat( goal_position, table_origin );
			need_to_move = true;
			break;
		case 20:
			goal_position = table_origin + Eigen::Vector3d(-1.5,-1.5,0.25);
			goal_orientation << misc::target2quat( goal_position, table_origin );
			need_to_move = true;
			break;
		case 30:
			goal_position = table_origin + Eigen::Vector3d(0,0,1);
			goal_orientation << misc::target2quat( goal_position, table_origin );
			need_to_move = true;
			break;
		default:
			goal_position = proc_position;
			goal_orientation = proc_orientation;
			need_to_move = false;
	}
	
	return need_to_move;
}

bool
ss_arbiter::process_fake_2( Eigen::Vector3d & proc_position, Eigen::Vector4d & proc_orientation, 
					  			  pcl::PointCloud<pcl::PointXYZ>::Ptr proc_cloud_ptr,
					  			  Eigen::Vector3d & goal_position, Eigen::Vector4d & goal_orientation)
{
	ROS_INFO("Processing!");	
	//ros::Duration(1).sleep(); // sleep for 2 seconds
	bool need_to_move;
	if( step_cntr % 1 == 0 )
	{
		goal_position = Eigen::Vector3d( misc::uniform_cont(-2.0, 2.0), 
					misc::uniform_cont(-2.0, 2.0), misc::uniform_cont(0.0, 1.5) );
		goal_orientation = misc::target2quat( goal_position, Eigen::Vector3d(0.0,0.0,0.0) );
		need_to_move = true;
	}
	else{
		goal_position = proc_position;
		goal_orientation = proc_orientation;
		need_to_move = false;
	}	
	return need_to_move;
}



bool
ss_arbiter::process_choose_state( Eigen::Vector3d & proc_position, Eigen::Vector4d & proc_orientation, 
					  							pcl::PointCloud<pcl::PointXYZ>::Ptr proc_cloud_ptr,
					  							Eigen::Vector3d & goal_position, Eigen::Vector4d & goal_orientation )
{
	// reset the previous choice
	PROC_OBJ_ID = -1;

	// Choose the largest object that is not decided !!
	Eigen::Vector3d target;
	{
		double max_vol = 0;
		int max_id = -1;
		double cl_dist = 1000;
		int cl_id = -1;
		int k = 0;
		for( std::map< int, boost::shared_ptr<obj> >::iterator it = obj_list.begin();
				it != obj_list.end(); ++it, ++k)
		{
			if( (it->second) -> get_decision( ) == -1)
			{
				double vol = (it->second)-> get_volume();
				if( vol > max_vol)
				{
					max_vol = vol;
					max_id = (it->second)-> get_id();
				}
				
				ROS_INFO("[arbiter] Obj vol = %f", vol);
				
				double dist_to_obj = ((it->second)->get_position() - proc_position).norm();
				if( ( dist_to_obj < cl_dist ) && (vol > 0.002) )
				{
					cl_dist = dist_to_obj;
					cl_id = (it->second)-> get_id();
				}
				
				/*
				// Get the object id and centroid projected to the table
				PROC_OBJ_ID = (it->second)-> get_id();
				std::cout << "obj_id = " << PROC_OBJ_ID << std::endl;
				std::cout << "obj_id_correct = " << k << std::endl;
				target = (it->second)-> get_table_position();

				// Color the chosen object yellow
				(it->second)-> set_color( Eigen::Vector3f(255,255,0) );
				break;
				*/
			}
		}
		
		// Choose the closest object with vol > 0.0025
		if( cl_id > -1 )
			PROC_OBJ_ID = cl_id;
		else
			PROC_OBJ_ID = max_id;
	}
	
	// if no objects left to process we are DONE!
	if( PROC_OBJ_ID == -1)
	{
		ROS_INFO("ALL OBJECTS HAVE BEEN PROCESSED!!!");
		// TODO: Move to a random location
		return false;
	}
	
	// If an object to process has been chosen, make it yellow
	std::cout << "obj_id = " << PROC_OBJ_ID << std::endl;
	target = obj_list[PROC_OBJ_ID]-> get_table_position();
	obj_list[PROC_OBJ_ID]->set_color( Eigen::Vector3f(255,255,0) );

	// Request a turn towards the chosen object and go to the decide state
	decider.initial_alignment( proc_position, target, goal_position, goal_orientation );
	ps = DECIDE;
	return true;
}

bool
ss_arbiter::process_decide_state( Eigen::Vector3d & proc_position, 
											 Eigen::Vector4d & proc_orientation, 
					  						pcl::PointCloud<pcl::PointXYZ>::Ptr proc_cloud_ptr,
					  						Eigen::Vector3d & goal_position, 
					  						Eigen::Vector4d & goal_orientation )
{	
	if( PROC_OBJ_ID < 0)
		throw std::runtime_error("Invalid object id in DECIDE state!\n");

	
	ss_decider::Decider_State dec_st ( update_object( PROC_OBJ_ID, proc_position, proc_cloud_ptr->header.stamp,
																	  goal_position, goal_orientation ) );

	
	bool need_to_move;	
	if( dec_st == ss_decider::DONE )
	{
		ps = CHOOSE;
		need_to_move = process_choose_state( proc_position, proc_orientation, proc_cloud_ptr, 
														  goal_position, goal_orientation );
	}
	else if( dec_st == ss_decider::MOVE )
	{
		need_to_move = true;
	}
	else
		need_to_move = false;
		
		
	//********* (START LOOK AROUND ADD ON) *********//
	if( dec_st != ss_decider::DONE )
	{
		// Save the PROC_OBJ goal pose
		DCM.goal_position = goal_position;
		DCM.goal_orientation = goal_orientation;
		
		// load the objs_around vector with all objects that have views
		for( std::map< int, boost::shared_ptr<obj> >::iterator it = obj_list.begin();
			it != obj_list.end(); ++it)
		{
		
			Eigen::Vector3d dir_vec = ((it->second)->get_table_position() - proc_position);
			
			if ( ( (it->second) -> get_decision( ) == -1 ) &&
				  ( ((it->second)->get_latest_view())->size() > 5000 ) && 
				  ( dir_vec.norm() < 1.4) &&
				  ( (it->second)->get_id() != PROC_OBJ_ID ) )
			{
				double yaw = atan2( dir_vec.y(), dir_vec.x() );	
				objs_around.push_back( std::make_pair( yaw, (it->second)->get_id()) );
				
				// set color to yellow
				(it->second)-> set_color( Eigen::Vector3f(200+misc::uniform_int(0,55),200+misc::uniform_int(0,55),0) );
			}
		}
		
		if( !objs_around.empty() )
		{
			// Sort the objs_around according to yaw
			std::sort ( objs_around.begin(), objs_around.end() );
		
			// Request a turn towards the smallest yaw
			goal_position = proc_position;
			goal_orientation = misc::target2quat( goal_position, obj_list[objs_around.back().second]->get_table_position() );
			ps = LOOK_AROUND;
		}
		
		need_to_move = true;
	}
	//********* (END LOOK AROUND ADD ON) *********//
	
	return need_to_move; 	
}


bool
ss_arbiter::process_look_state( Eigen::Vector3d & proc_position, Eigen::Vector4d & proc_orientation, 
				  							pcl::PointCloud<pcl::PointXYZ>::Ptr proc_cloud_ptr,
				  							Eigen::Vector3d & goal_position, Eigen::Vector4d & goal_orientation )
{
	assert( !objs_around.empty() );
	
	int obj_id = objs_around.back().second;
	ROS_INFO("LOOK_AROUND at obj %d", obj_id);
	
	update_object( obj_id, proc_position, proc_cloud_ptr->header.stamp,
						goal_position, goal_orientation );
						
	
	objs_around.pop_back();
		
	if( objs_around.empty() )
	{
		// If all objects have been looked at go back to the decide state
		goal_position = DCM.goal_position;
		goal_orientation = DCM.goal_orientation;
		ps = DECIDE;
	}
	else
	{
		// Continue looking at the next object
		goal_position = proc_position;
		goal_orientation = misc::target2quat( goal_position, obj_list[objs_around.back().second]->get_table_position() );
	}
	
	return true;
}
					  							
					  							



ss_decider::Decider_State 
ss_arbiter::update_object( int obj_id, Eigen::Vector3d & proc_position, const ros::Time & tf_time,
									Eigen::Vector3d & goal_position, Eigen::Vector4d & goal_orientation )
{
	// Get the a pointer to the object of interest
	boost::shared_ptr<obj> & OBJ_ptr = obj_list[obj_id];

	// Display object properties
	std::cout << *OBJ_ptr << std::endl;
	
	// Get the target and the belief
	const Eigen::Vector3d & target = OBJ_ptr-> get_table_position();
	//std::vector<double>::iterator bel_start;
	//std::vector<double>::iterator bel_end;
	//OBJ_ptr->get_belief( bel_start, bel_end);
	
	
	// Get the test surface
	pcl::PointCloud<pcl::PointXYZ>::Ptr & test_surf = OBJ_ptr-> get_latest_view();
	//pcl::PointCloud<pcl::PointXYZ>::Ptr test_surf(new pcl::PointCloud<pcl::PointXYZ>);
	//get_test_surf( OBJ_ptr, proc_cloud_ptr, test_surf);

	// Convert the test surface to the /sensor_optical frame
	pcl::PointCloud<pcl::PointXYZ>::Ptr test_surf_opt(new pcl::PointCloud<pcl::PointXYZ>);
	test_surf_opt -> header.frame_id = opt_frm;
	test_surf -> header.frame_id = fx_frm;		// just to ensure the frame_id is correct
	bool transform_success = transform_cloud( test_surf, test_surf_opt, tf_time );
	if( !transform_success )
		throw std::runtime_error("Transform for test surface could not be found! Increase tf cache size!\n");

	//ov.add_curr_view( test_surf_opt );
	

	// Get the best action from the decider
	std::pair<ss_decider::Decider_State,int> isDone_bestAct( ss_decider::MOVE, -1 );
	//isDone_bestAct = decider.decider_spin( proc_position, target, test_surf_opt, bel_start, bel_end, goal_position, goal_orientation );	
	isDone_bestAct = decider.decider_spin( proc_position, target, test_surf_opt, 
			obj_list, obj_id, goal_position, goal_orientation );
	
	if( isDone_bestAct.first == ss_decider::DONE )
	{
		// if we are done color the object based on the hypothesis and set ps to CHOOSE
		Hypothesis & chosenHyp = decider.get_hyp( isDone_bestAct.second );
		
		if( !chosenHyp.positive )
		{
			OBJ_ptr -> set_color(Eigen::Vector3f(255,0,0));		// red
			OBJ_ptr -> set_decision( isDone_bestAct.second, Eigen::Vector4d(0,0,0,1) ); // null hypothesis
		}
		else
		{
			OBJ_ptr -> set_color( Eigen::Vector3f(0,255,0) );	// green
			// TODO: perform orientation refinement!
			Eigen::Vector3d yaw_pitch_roll( chosenHyp.yaw, chosenHyp.pitch, chosenHyp.roll );
			OBJ_ptr -> set_decision( isDone_bestAct.second, misc::angle2quat( yaw_pitch_roll ) );
		}
		
		// write statistics to file
		std::ofstream outfile ( "/home/natanaso/Desktop/dec_stat.txt", std::ofstream::app );
		if(!outfile.is_open())
			throw std::runtime_error("Unable to open save file...\n");
		
		outfile << OBJ_ptr->to_file_str(decider.get_dMap());
		outfile.close();
		
	}
	
	ROS_INFO("OBJ after");
	std::cout << *OBJ_ptr << std::endl;
	return isDone_bestAct.first;
}



/**
 * IMPORTANT: Ensure that the frame_id of cld_out is set correctly to the target frame
 */
bool
ss_arbiter::transform_cloud( pcl::PointCloud<pcl::PointXYZ>::Ptr & cld_in_ptr, 
									  pcl::PointCloud<pcl::PointXYZ>::Ptr & cld_out_ptr, 
									  const ros::Time & tf_time )
{
	std::string from_frame( cld_in_ptr->header.frame_id);
	std::string to_frame( cld_out_ptr->header.frame_id);
	tf::StampedTransform transform;

	try
	{
		listener.waitForTransform( from_frame, to_frame, tf_time, ros::Duration(5.0));
		listener.lookupTransform( to_frame, from_frame, tf_time, transform);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s", ex.what());
		return false;
	}
	
	pcl_ros::transformPointCloud( *cld_in_ptr, *cld_out_ptr, transform);
	cld_out_ptr->header.frame_id = to_frame;
	cld_out_ptr->header.stamp = cld_in_ptr->header.stamp;
	return true;
}



// takes a cloud in the map frame
void
ss_arbiter::update_original_cloud( pcl::PointCloud<pcl::PointXYZ>::Ptr & cld_ptr )
{
	float orig_cld_voxel_size = 0.003f;// 0.0005f
	
	// remove NaNs
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cld_ptr, *cld_ptr, indices);
	
	pcl::PassThrough<pcl::PointXYZ> pass_;
	pass_.setFilterFieldName ("z");
	pass_.setFilterLimits (0.55, 2.0);	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cld_ptr_a(new pcl::PointCloud<pcl::PointXYZ>);
	pass_.setInputCloud(cld_ptr);
	pass_.filter(*cld_ptr_a);
	
	cld_ptr_a = pcd_utils::voxel_grid_subsample( cld_ptr_a, orig_cld_voxel_size );	
	
	if( xyz_cld_ptr_original->empty() )
	{
		/*
		// load the octomap into xyz_cld_ptr_original
		pcl::io::loadPCDFile<pcl::PointXYZ> ( ros::package::getPath("gazebo_simulation") + 
														"/maps/map-grasp-lab.pcd", *xyz_cld_ptr_original);
		cld_ptr = pcd_utils::align_clouds( cld_ptr, xyz_cld_ptr_original );
		//*xyz_cld_ptr_original = *cld_ptr + *xyz_cld_ptr_original;		// add the two clouds
		*/

		xyz_cld_ptr_original = cld_ptr_a;
	}else
	{	
	
	
		// Get an initial guess for the transform
		pass_.setFilterLimits (0.75, 2.0);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cld_ptr_filt(new pcl::PointCloud<pcl::PointXYZ>);
		pass_.setInputCloud(cld_ptr_a);
		pass_.filter(*cld_ptr_filt);
		pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cld_ptr_filt(new pcl::PointCloud<pcl::PointXYZ>);
		pass_.setInputCloud(xyz_cld_ptr_original);
		pass_.filter(*xyz_cld_ptr_filt);
		
		
		double fitness_score = 1000;
		Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
		Eigen::Matrix4f fTrans = Eigen::Matrix4f::Identity();
		pcd_utils::align_clouds( cld_ptr_filt, xyz_cld_ptr_filt, 0.2, guess, fTrans, fitness_score );
		
		guess = fTrans;
		pcd_utils::align_clouds( cld_ptr_a, xyz_cld_ptr_original, 0.2, guess, fTrans, fitness_score );
		
		
		Eigen::Matrix4f best_trans = fTrans;
		double best_score = fitness_score;
		if( fitness_score > 0.00025 )
		{
			const double arr[] = {0.2, 0.0, 0.0,
										-0.2, 0.0, 0.0, 
										 0.0, 0.2, 0.0, 
										 0.0, -0.2, 0.0,
										 0.0, 0.0, 0.2,
										 0.0, 0.0, -0.2};
			for(int k = 0; k < 6; ++k)
			{
				// generate the offset cloud above the table
				pcl::PointCloud<pcl::PointXYZ>::Ptr cld_ptr_off(new pcl::PointCloud<pcl::PointXYZ>);
				pcl::copyPointCloud(*cld_ptr_filt, *cld_ptr_off);
				for( pcl::PointCloud<pcl::PointXYZ>::iterator it = cld_ptr_off->begin();
					it != cld_ptr_off->end(); ++it)
				{
					(*it).x += arr[3*k];
					(*it).y += arr[3*k+1];
					(*it).z += arr[3*k+2];
				}
				guess = Eigen::Matrix4f::Identity();
				pcd_utils::align_clouds( cld_ptr_off, xyz_cld_ptr_filt, 0.2, guess, fTrans, fitness_score );	
			
			
				pcl::PointCloud<pcl::PointXYZ>::Ptr cld_ptr_a_off(new pcl::PointCloud<pcl::PointXYZ>);
				pcl::copyPointCloud(*cld_ptr_a, *cld_ptr_a_off);
				for( pcl::PointCloud<pcl::PointXYZ>::iterator it = cld_ptr_a_off->begin();
					it != cld_ptr_a_off->end(); ++it)
				{
					(*it).x += arr[3*k];
					(*it).y += arr[3*k+1];
					(*it).z += arr[3*k+2];
				}		
				guess = fTrans;
				pcd_utils::align_clouds( cld_ptr_a_off, xyz_cld_ptr_original, 0.2, guess, fTrans, fitness_score );
						
				if( fitness_score < best_score)
				{
					best_score = fitness_score;
				
					Eigen::Matrix4f T = Eigen::Matrix4f::Identity();				
					T(0,3) = arr[3*k];
					T(1,3) = arr[3*k+1];
					T(2,3) = arr[3*k+2];
					best_trans = fTrans * T;
					
					if( best_score < 0.00025)
						break;
				}	
			}
		}
		
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr best_ptr(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::transformPointCloud ( *cld_ptr, *best_ptr, best_trans );
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr best_ptr_a(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::transformPointCloud ( *cld_ptr_a, *best_ptr_a, best_trans );
		
		pcl::PCDWriter writer;
		writer.writeASCII( "/home/natanaso/Desktop/cld_experiment/orig_"+
						boost::lexical_cast<std::string>(goal_completion_time.toSec()) +
						".pcd", *xyz_cld_ptr_original );
		writer.writeASCII( "/home/natanaso/Desktop/cld_experiment/cld_"+
						boost::lexical_cast<std::string>(goal_completion_time.toSec()) +
						".pcd", *cld_ptr );
									
		cld_ptr = best_ptr;	
		*xyz_cld_ptr_original = *best_ptr_a + *xyz_cld_ptr_original;		// add the two clouds
		
		
		writer.writeASCII( "/home/natanaso/Desktop/cld_experiment/final_"+
						boost::lexical_cast<std::string>(goal_completion_time.toSec()) +
						"_" + boost::lexical_cast<std::string>(fitness_score)+
						".pcd", *xyz_cld_ptr_original );		
		
		/*	
		pcl::PointCloud<pcl::PointXYZ>::Ptr best_ptr(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr best_ptr_a(new pcl::PointCloud<pcl::PointXYZ>);
		double best_score = 1000;
		for(int k = 0; k < 1; ++k)
		{
			// Get an initial guess for the transform
			pass_.setFilterLimits (0.75 + k*0.12, 2.0);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cld_ptr_filt(new pcl::PointCloud<pcl::PointXYZ>);
			pass_.setInputCloud(cld_ptr_a);
			pass_.filter(*cld_ptr_filt);
			pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cld_ptr_filt(new pcl::PointCloud<pcl::PointXYZ>);
			pass_.setInputCloud(xyz_cld_ptr_original);
			pass_.filter(*xyz_cld_ptr_filt);
		
			double fitness_score = 1000;
			Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
			Eigen::Matrix4f fTrans = Eigen::Matrix4f::Identity();
			
			pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_ptr(new pcl::PointCloud<pcl::PointXYZ>);
			tmp_ptr = pcd_utils::align_clouds( cld_ptr_filt, xyz_cld_ptr_filt, 0.2, guess, fTrans, fitness_score );
			
			// Use the guess to align the large cloud	
			guess = fTrans;
			tmp_ptr = pcd_utils::align_clouds( cld_ptr_a, xyz_cld_ptr_original, 0.2, guess, fTrans, fitness_score );
		
			if( fitness_score < best_score )
			{
				best_score = fitness_score;
				pcl::transformPointCloud ( *cld_ptr, *best_ptr, fTrans );
				pcl::transformPointCloud ( *cld_ptr_a, *best_ptr_a, fTrans );
			}
		}
		*/
		
	}

	// Filter to reduce size and remove duplicates
	xyz_cld_ptr_original = pcd_utils::voxel_grid_subsample( xyz_cld_ptr_original, orig_cld_voxel_size );
	ROS_INFO("Size after voxelgrid filter: %d", static_cast<int>(xyz_cld_ptr_original->size()) );		
		
		
		/*
		if( fitness_score > 0.01 )
		{
			for(int k = 1; k <= 2; ++k)
			{
				pass_.setFilterLimits (0.75 + k*0.05, 2.0);
				pass_.setInputCloud(cld_ptr);
				pass_.filter(*cld_ptr_filt);
				pass_.setInputCloud(xyz_cld_ptr_original);
				pass_.filter(*xyz_cld_ptr_filt);			
				tmp_ptr = pcd_utils::align_clouds( cld_ptr_filt, xyz_cld_ptr_filt, 0.1 + k*0.1, Eigen::Matrix4f::Identity(), fTrans, fitness_score );
				
				pass_.setFilterLimits (0.4, 2.0);
				pass_.setInputCloud(cld_ptr);
				pass_.filter(*cld_ptr_filt);
				pass_.setInputCloud(xyz_cld_ptr_original);
				pass_.filter(*xyz_cld_ptr_filt);				
				tmp_ptr = pcd_utils::align_clouds( cld_ptr_filt, xyz_cld_ptr_filt, 0.1 + k*0.1, fTrans, fitness_score );
				
				if( fitness_score > 0.01 )
				{
					if( fitness_score < best_score )
					{
						best_score = fitness_score;
						pcl::transformPointCloud ( *cld_ptr, *best_ptr, fTrans );
					}
				}
				else
				{
					best_score = fitness_score;
					pcl::transformPointCloud ( *cld_ptr, *best_ptr, fTrans );
					break;
				}
				
			}
			
			
			
			double best_score = fitness_score;
			for(int k = 1; k <= 7; ++k)
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr cld_ptr_filta(new pcl::PointCloud<pcl::PointXYZ>);
				pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cld_ptr_filta(new pcl::PointCloud<pcl::PointXYZ>);
				pass_.setFilterLimits (0.2 + k*0.1, 2.0);
				pass_.setInputCloud(cld_ptr);
				pass_.filter(*cld_ptr_filta);
				pass_.setInputCloud(xyz_cld_ptr_original);
				pass_.filter(*xyz_cld_ptr_filta);
				
				tmp_ptr = pcd_utils::align_clouds( cld_ptr_filta, xyz_cld_ptr_filta, 0.2, Eigen::Matrix4f::Identity(), fTrans, fitness_score );
				tmp_ptr = pcd_utils::align_clouds( cld_ptr_filt, xyz_cld_ptr_filt, 0.2, fTrans, fitness_score );
				
				if( fitness_score > 0.01 )
				{
					if( fitness_score < best_score )
					{
						best_score = fitness_score;
						pcl::transformPointCloud ( *cld_ptr, *best_ptr, fTrans );
					}
				}
				else
				{
					pcl::transformPointCloud ( *cld_ptr, *best_ptr, fTrans );
					break;
				}
			}
			
		}
		*/
		/*
		pcl::PointCloud<pcl::PointXYZ>::Ptr best_ptr(new pcl::PointCloud<pcl::PointXYZ>);
		double best_score = 1000;
		for(int k = 0; k < 3; ++k)
		{
			// first align only the clouds above the table to get a guess for the transformation
			Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
			Eigen::Matrix4f fTrans = Eigen::Matrix4f::Identity();
		
			pass_.setFilterLimits (0.7+k*0.01, 2.5);
		
			pcl::PointCloud<pcl::PointXYZ>::Ptr cld_ptr_filt(new pcl::PointCloud<pcl::PointXYZ>);
			pass_.setInputCloud(cld_ptr);
			pass_.filter(*cld_ptr_filt);
		
			pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cld_ptr_filt(new pcl::PointCloud<pcl::PointXYZ>);
			pass_.setInputCloud(xyz_cld_ptr_original);
			pass_.filter(*xyz_cld_ptr_filt);
		
			double fitness_score = 1000;
			pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_ptr(new pcl::PointCloud<pcl::PointXYZ>);
			tmp_ptr = pcd_utils::align_clouds( cld_ptr_filt, 
								xyz_cld_ptr_filt, 0.15, guess, fTrans, fitness_score );
																											
			// Perform ICP
			tmp_ptr = pcd_utils::align_clouds( cld_ptr, 
								xyz_cld_ptr_original, 0.08, fTrans, fitness_score );
			
			if( fitness_score < best_score )
			{
				best_score = fitness_score;
				best_ptr = tmp_ptr;
			}
			
			if( best_score < 0.05 )
				break;
			else
			{
				pass_.setFilterLimits (0.2, 2.0);
				pass_.setInputCloud(cld_ptr);
				pass_.filter(*cld_ptr_filt);
				pass_.setInputCloud(xyz_cld_ptr_original);
				pass_.filter(*xyz_cld_ptr_filt);
				tmp_ptr = pcd_utils::align_clouds( cld_ptr_filt, xyz_cld_ptr_filt, 0.1, Eigen::Matrix4f::Identity(), fTrans, fitness_score );
				pcl::transformPointCloud ( *cld_ptr, *best_ptr, fTrans ); 
				break;
			}
		}
	  */
		
}



/*
std::vector<tabletop_analyzer::TableInfo>::iterator ti_start,
									 std::vector<tabletop_analyzer::TableInfo>::iterator ti_end )
	//std::cout << "HERE" << std::endl;
	// Choose the first table and assume it is perpendicular to the z-axis
	 = *ti_start;
*/	  
void
ss_arbiter::update_obj_map( tabletop_analyzer::TableInfo & ti )
{

	int num_obj = ti.obj_surf.size();	
	float dist_rad = 0.18f / 0.03f;	// two centroids are combined if they are less than this distance in [cells] NOT [meters]


	// Reset the latest views of all existing objects
	for( std::map< int, boost::shared_ptr<obj> >::iterator it = obj_list.begin();
			it != obj_list.end(); ++it)
	{
		(it->second) -> reset_latest_view();
	}
	
	 
	// loop thorugh hypmap and match the existing centroids to the newly detected projected centroids
	if( hyp_map_centroids->empty() ){

		// add the new objects
		for( int k=0; k < num_obj; ++k)
		{
			// obj_id
			int obj_id = obj_id_ptr_;

			// Object surface
			pcl::PointCloud<pcl::PointXYZ>::Ptr obj_surf_i( ti.obj_surf[k] );
			
			// Random Color exept red green 
			//Eigen::Vector3f obj_color_i(0,0,0);
			Eigen::Vector3f obj_color_i(misc::uniform_int(0,155),misc::uniform_int(0,155),misc::uniform_int(0,255));
						
			// Create the object
			std::vector<double>::iterator prior_beg;
			std::vector<double>::iterator prior_end;
			decider.get_prior(prior_beg, prior_end);
			boost::shared_ptr<obj> OBJ_ptr( new obj( ti.tab_coefs, obj_id, obj_surf_i,
															     obj_color_i, prior_beg, prior_end ) );
															     
			// convert centroid to cells
		  	Eigen::Vector2i obj_centroid_cell_i;
		  	table_map.meters2cells( (OBJ_ptr -> get_table_position()).topLeftCorner(2,1), obj_centroid_cell_i );

			// Save the object data
			pcl::PointXY obj_cc_i;
			obj_cc_i.x = obj_centroid_cell_i(0);
			obj_cc_i.y = obj_centroid_cell_i(1);
			
			hyp_map.insert(std::make_pair(
		 						std::make_pair( obj_centroid_cell_i(0), obj_centroid_cell_i(1)), OBJ_ptr));

			hyp_map_centroids->push_back(obj_cc_i);
			
			obj_list.insert( std::make_pair( obj_id, OBJ_ptr ) );
			++obj_id_ptr_;
		}
	}
	else{
		// Search through the existing centroids
		pcl::search::KdTree<pcl::PointXY>::Ptr stree_ (new pcl::search::KdTree<pcl::PointXY>);
		stree_->setInputCloud (hyp_map_centroids);
		
		for( int k=0; k < num_obj; ++k)
		{			
			// convert centroid to cells
			Eigen::Vector3d obj_centroid_proj_i( ti.obj_proj_centroids->at(k).x, 
															 ti.obj_proj_centroids->at(k).y,
															 ti.obj_proj_centroids->at(k).z );
			Eigen::Vector2i obj_centroid_cell_i;
			table_map.meters2cells( obj_centroid_proj_i.topLeftCorner(2,1), obj_centroid_cell_i );
			pcl::PointXY obj_cc_i;
			obj_cc_i.x = obj_centroid_cell_i(0);
			obj_cc_i.y = obj_centroid_cell_i(1);

			// Check if the object exists already			
			std::vector< int > idx;
			std::vector< float > sqr_dist;
			if( ( stree_->nearestKSearch( obj_cc_i, 1, idx, sqr_dist ) > 0 ) && ( sqr_dist[0] < dist_rad*dist_rad) )
			{
				// Object exists so update
				std::pair<int,int> match_table_loc = std::make_pair( hyp_map_centroids-> at(idx[0]).x,
																					  hyp_map_centroids-> at(idx[0]).y );
				
				boost::shared_ptr<obj> & OBJ_ptr = hyp_map[match_table_loc];

				// insert in hyp_map in case the new table loc is not associated with the object yet
				std::pair<int,int> new_table_loc = std::make_pair(obj_centroid_cell_i(0), obj_centroid_cell_i(1));
				if( hyp_map.count(new_table_loc) == 0 )
				{
					hyp_map.insert(std::make_pair(new_table_loc, OBJ_ptr));
					hyp_map_centroids->push_back(obj_cc_i);
				}

				// Update the surface
				pcl::PointCloud<pcl::PointXYZ>::Ptr obj_surf_i( ti.obj_surf[k] );
				OBJ_ptr -> add_latest_view( obj_surf_i );
				//ROS_WARN("New surface size = %d", obj_surf_i->size() );
			}
			else
			{
				ROS_INFO("New object detected with distance to closest centroid = %f", sqr_dist[0]); 
				std::cout << sqr_dist[0] << std::endl;

				// Object not found so create
				int obj_id = obj_id_ptr_;
				//Eigen::Vector3f obj_color_i(0,0,0);
				Eigen::Vector3f obj_color_i(misc::uniform_int(0,155),misc::uniform_int(0,155),misc::uniform_int(0,255));
				pcl::PointCloud<pcl::PointXYZ>::Ptr obj_surf_i( ti.obj_surf[k] );
				
				
				// Create the object
				std::vector<double>::iterator prior_beg;
				std::vector<double>::iterator prior_end;
				decider.get_prior(prior_beg, prior_end);
				boost::shared_ptr<obj> OBJ_ptr( new obj( ti.tab_coefs, obj_id, obj_surf_i,
															     obj_color_i, prior_beg, prior_end ) );

				// Save the object data
				hyp_map.insert(std::make_pair(
			 						std::make_pair( obj_centroid_cell_i(0), obj_centroid_cell_i(1)), OBJ_ptr));

				hyp_map_centroids->push_back(obj_cc_i);
			
				obj_list.insert( std::make_pair( obj_id, OBJ_ptr ) );
				++obj_id_ptr_;
			}
		}
	}
}



void
ss_arbiter::renew_obj_map( pcl::ModelCoefficients::Ptr & table_coeffs )
{
	//ROS_WARN("HERE1!");
	// determine the intersection objects
	std::set<int> intersect_ids;
	std::map< int, boost::shared_ptr<obj> >::iterator olist_preend = obj_list.end();
	--olist_preend;
	for( std::map< int, boost::shared_ptr<obj> >::iterator it1 = obj_list.begin();
			it1 != olist_preend; ++it1)	// up to obj_list.end() - 1
		for( std::map< int, boost::shared_ptr<obj> >::iterator it2 = it1;
			++it2 != obj_list.end(); /* advance before the loop */)
		{
			if( ( (it1->second) -> is_intersect( *(it2->second) ) ) &&	// objects intersect
				 ( ( (it1->second) -> get_surf_size_delta() < 0 ) ||		// their surfaces changed enough
				 	( (it1->second) -> get_surf_size_delta() >= 0 ) ||
				 	( (it2->second) -> get_surf_size_delta() < 0 ) ||
				 	( (it2->second) -> get_surf_size_delta() >= 0 ) ) )
			{
				//ROS_WARN("Object %d change in surf = %f", (it1->second)->get_id(), (it1->second) -> get_surf_size_delta());
				//ROS_WARN("Object %d change in surf = %f", (it2->second)->get_id(), (it2->second) -> get_surf_size_delta());
				intersect_ids.insert( (it1->second) -> get_id() );
				intersect_ids.insert( (it2->second) -> get_id() );
			}
		}
	
	// if no intersecting objects we do not need to update
	if(intersect_ids.empty())
		return;
	
	ROS_INFO("Intersect ids:");
	/*
	for(std::set< int >::iterator it = intersect_ids.begin();
			it != intersect_ids.end(); ++it)
	{
		std::cout << *it << " ";
	}
	*/
	
	/*
	for(std::map< int, boost::shared_ptr<obj> >::iterator it = obj_list.begin();
			it != obj_list.end(); ++it)
	{
		ROS_WARN("OBJ_id = %d", (it->second)->get_id());
	}
	*/
	
	// recluster the intersecting surface
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr intersect_surf( new pcl::PointCloud<pcl::PointXYZRGB> );
	for(std::set< int >::iterator it = intersect_ids.begin();
			it != intersect_ids.end(); ++it)
	{
			*intersect_surf += *(obj_list[*it]->get_surface());
	}
	
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> new_obj_surf;
	pcl::PointCloud<pcl::PointXYZ>::Ptr new_obj_maxp( new pcl::PointCloud<pcl::PointXYZ> );
	pcl::PointCloud<pcl::PointXYZ>::Ptr new_obj_minp( new pcl::PointCloud<pcl::PointXYZ> );
	pcl::PointCloud<pcl::PointXYZ>::Ptr new_obj_centroids( new pcl::PointCloud<pcl::PointXYZ> );
	pcl::PointCloud<pcl::PointXYZ>::Ptr new_obj_proj_centroids( new pcl::PointCloud<pcl::PointXYZ> );
	
	ROS_WARN("Reclustering %d intersected surfaces", static_cast<int>(intersect_surf->size()));
	TA.obj_cld2clusters( table_coeffs, intersect_surf, new_obj_surf,
								new_obj_maxp, new_obj_minp, new_obj_centroids, new_obj_proj_centroids );
	ROS_WARN("Done");
	
	// IF the reclustering failed...
	if( new_obj_centroids -> size() < 1)
		return;
	
	
	/*
	// Test problems!!
	ROS_WARN("new_obj_surf.size() %d", new_obj_surf.size());
	//1. Color new ojs randomly
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr reclustered_surf( new pcl::PointCloud<pcl::PointXYZRGB> );
		
	for(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::iterator it = new_obj_surf.begin();
		it != new_obj_surf.end(); ++it)
	{
	
		double rrrr = misc::uniform_int(0,255);
		double gggg = misc::uniform_int(0,255);
		double bbbb = misc::uniform_int(0,255);
		for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it1 = (*it)->begin();
			it1 != (*it)->end(); ++it1)
		{
			(*it1).r = rrrr;
			(*it1).g = gggg;
			(*it1).b = bbbb;
		}
		*reclustered_surf += (**it);
	}
	
	// 2. Display intersect surf and reclustered_surf
	ov.add_tst_cld1( intersect_surf );
	ov.add_tst_cld2( reclustered_surf );
	*/
	
	//ROS_WARN("new_obj_centroids->size() %d", new_obj_centroids->size());
						
	//ROS_WARN("HERE3!");
	// associate the intersect ids with the newly detected clusters
	pcl::search::KdTree<pcl::PointXYZ>::Ptr stree (new pcl::search::KdTree<pcl::PointXYZ>);
	stree->setInputCloud (new_obj_centroids);
	std::vector< std::vector<boost::shared_ptr<obj> > > obj_matches;		// maps new obj to a list of old objs
	obj_matches.resize( new_obj_centroids->size() );
	for( std::set<int>::iterator it = intersect_ids.begin(); 
			it != intersect_ids.end(); ++it)
	{

		// determine the object centroid
		boost::shared_ptr<obj> OBJ_ptr( obj_list[*it] );
		pcl::PointXYZ obj_cntr( OBJ_ptr->get_position().x(), OBJ_ptr->get_position().y(),
										OBJ_ptr->get_position().z()  );
		
		// match it against the reclustered centroids
		std::vector< int > idx;
		std::vector< float > sqr_dist;
		if( stree->nearestKSearch( obj_cntr, 1, idx, sqr_dist ) > 0 )
		{
			obj_matches[idx[0]].push_back( OBJ_ptr );
		}
	}
	
	ROS_INFO("obj_matches.size = %d", static_cast<int>(obj_matches.size()));
	for( std::vector< std::vector<boost::shared_ptr<obj> > >::iterator it1 = obj_matches.begin();
			it1 != obj_matches.end(); ++it1)
		{
			for( std::vector<boost::shared_ptr<obj> >::iterator it2 = it1->begin();
				it2 != it1->end(); ++it2)
				std::cout << (*it2)->get_id() << " ";
			std::cout << std::endl;
		}
	ROS_INFO("END");
	std::cout << std::endl;	
	
	// Create a map from old ids to new objects and update the obj_list
	std::map< int, boost::shared_ptr<obj> > match_map;
	for( unsigned int i = 0; i < new_obj_centroids->size(); ++i )
	{		
		// Create the new object
		Eigen::Vector3d obj_centroid_i( new_obj_centroids->at(i).x, new_obj_centroids->at(i).y, new_obj_centroids->at(i).z );
		Eigen::Vector3d obj_centroid_proj_i( new_obj_proj_centroids->at(i).x, 
														 new_obj_proj_centroids->at(i).y, 
														 new_obj_proj_centroids->at(i).z );
		Eigen::Vector3d obj_maxp_i( new_obj_maxp->at(i).x, new_obj_maxp->at(i).y, new_obj_maxp->at(i).z );
		Eigen::Vector3d obj_minp_i( new_obj_minp->at(i).x, new_obj_minp->at(i).y, new_obj_minp->at(i).z );
		
		
		Eigen::Vector3f new_color(misc::uniform_int(0,155),misc::uniform_int(0,155),misc::uniform_int(0,255));
		std::vector<double>::iterator bel_start;
		std::vector<double>::iterator bel_end;
		decider.get_prior(bel_start, bel_end);
						     
		Eigen::Vector4d new_r;
		int new_hid = -1;
		
		
		bool is_copied = false;
		std::vector<int>::iterator vp_hist_start;
		std::vector<int>::iterator vp_hist_end;
		std::vector<bool>::iterator visited_vps_start;
		std::vector<bool>::iterator visited_vps_end;
		pcl::PointCloud< pcl::PointXYZ >::Ptr latest_view_ptr( new pcl::PointCloud< pcl::PointXYZ >);
		
		// TODO: Copy the old object over instead of copying individual properties
		// retain properties of the old object
		if( obj_matches[i].size() > 0 ){
			boost::shared_ptr<obj> largest_OBJ = *max_element(obj_matches[i].begin(), obj_matches[i].end(), obj::comp_less);
			new_color = largest_OBJ->get_color();
			largest_OBJ -> get_belief( bel_start, bel_end );
			new_hid = largest_OBJ->get_decision( new_r );
			
			is_copied = true;
			largest_OBJ -> get_vp_hist( vp_hist_start, vp_hist_end, visited_vps_start, visited_vps_end );
			
			// get the latest views from all matched old objects
			for( std::vector<boost::shared_ptr<obj> >::iterator it = obj_matches[i].begin();
					it != obj_matches[i].end(); ++it)
				*latest_view_ptr += *((*it)-> get_latest_view());
		}
					 	  

		boost::shared_ptr<obj> new_OBJ_ptr( new obj( table_coeffs, obj_id_ptr_, new_obj_surf[i],
																new_color, bel_start, bel_end, obj_centroid_i,
															   obj_centroid_proj_i, obj_maxp_i, obj_minp_i ) );
															   
		// copy the old vp history
		if( is_copied )
			new_OBJ_ptr -> set_vp_hist( vp_hist_start, vp_hist_end, visited_vps_start, visited_vps_end );
		
		// ensure the latest view is empty because it is not high resolution now
		// But this makes the current view empty
		new_OBJ_ptr->set_latest_view( latest_view_ptr );
		new_OBJ_ptr->set_decision( new_hid, new_r );
		++obj_id_ptr_;
		

		// Insert the new object into the obj list
		obj_list.insert( std::make_pair( new_OBJ_ptr->get_id(), new_OBJ_ptr ) );
		
		// populate the match map
		for( std::vector<boost::shared_ptr<obj> >::iterator it2 = obj_matches[i].begin();
				it2 != obj_matches[i].end(); ++it2)
		{
			// accumulate the latest views
			new_OBJ_ptr->add_latest_view( (*it2)->get_latest_view() );
			
			// include the match
			match_map.insert( std::make_pair( (*it2)->get_id(), new_OBJ_ptr) );
		
			// delete the old object from obj_list
			obj_list.erase ((*it2)->get_id());
		}		
	}
	
	
	// update hyp_map
	// Go through the hyp_map
	for( std::map< std::pair<int,int>, boost::shared_ptr<obj> >::iterator it = hyp_map.begin();
			it != hyp_map.end(); ++it)
	{
		// If the obj's id is in intersect ids
		int obj_id = (it->second)->get_id();
		if( intersect_ids.find( obj_id ) != intersect_ids.end() )
		{
			// Associate its obj pointer with the new obj!
			(it->second) = match_map[obj_id];
		}
	}

	// Correct the proc_obj_id_vp
	if( (PROC_OBJ_ID != -1) && (intersect_ids.find( PROC_OBJ_ID ) != intersect_ids.end() ))
	{
		PROC_OBJ_ID = match_map[PROC_OBJ_ID] -> get_id();
	}
	
	// correct the look around obj ids
	for( std::vector< std::pair<double,int> >::iterator it = objs_around.begin();
			it != objs_around.end(); ++it )
	{
		if( intersect_ids.find( it->second ) != intersect_ids.end() )
			it->second = match_map[it->second] -> get_id();
	}
	
	/*
	ROS_WARN("DISP HYPMAP");
	for(std::map< int, boost::shared_ptr<obj> >::iterator it = obj_list.begin();
			it != obj_list.end(); ++it)
	{
		ROS_WARN("OBJ_id = %d", (it->second)->get_id());
	}
	*/	
}

void
ss_arbiter::update_disp_cloud( )
{
	disp_cld_ptr->clear();
	for(std::map< int, boost::shared_ptr<obj> >::iterator it = obj_list.begin();
			it != obj_list.end(); ++it)
		*disp_cld_ptr += *((it->second)->get_surface());
}


void 
ss_arbiter::update_disp_cloud_fake( pcl::PointCloud<pcl::PointXYZ>::Ptr & proc_cloud_ptr )
{
	//disp_cld_ptr->clear();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_cld_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud( *proc_cloud_ptr, *tmp_cld_ptr );
	*disp_cld_ptr += *tmp_cld_ptr;
	pcl::VoxelGrid<pcl::PointXYZRGB> grid_filter_;
	double cell_size = 0.008;	//leaf size in meters
	grid_filter_.setLeafSize (cell_size, cell_size, cell_size);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cld( new pcl::PointCloud<pcl::PointXYZRGB> );
	grid_filter_.setInputCloud( disp_cld_ptr );
	grid_filter_.filter ( *filtered_cld );
	disp_cld_ptr = filtered_cld;
}






void
ss_arbiter::get_test_surf( boost::shared_ptr<obj> & OBJ_ptr,
									pcl::PointCloud<pcl::PointXYZ>::Ptr & proc_cloud_ptr,
							 		pcl::PointCloud<pcl::PointXYZ>::Ptr & test_surf_ptr )
{
	// Get the object bounding box
	const Eigen::Vector3d & maxp = OBJ_ptr-> get_maxp();
	const Eigen::Vector3d & minp = OBJ_ptr-> get_minp();
	Eigen::Vector3d midpt = (maxp + minp)/2.0;

	// Determine a search point and a radius to be used with a kdtree
	// in order to extract the test surface
	pcl::PointXYZ kdSearchPt( midpt(0), midpt(1), midpt(2) );
	float kdRad = static_cast<float>((maxp - midpt).norm());

	// Use a kdtree to search the proc_cloud for the test surface
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_ (new pcl::search::KdTree<pcl::PointXYZ>);
	kdtree_->setInputCloud( proc_cloud_ptr );
	std::vector<int> match_idx;
	std::vector<float> match_sqdist;

	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_test_surf(new pcl::PointCloud<pcl::PointXYZ>);
	if ( kdtree_->radiusSearch (kdSearchPt, kdRad, match_idx, match_sqdist) > 0 )
	{
		// These are the indices of the test surface in the proc_cloud
		pcl::PointIndices::Ptr test_surf_idx(new pcl::PointIndices);
		test_surf_idx->indices = match_idx;
		
		// Extract the found surface from proc_cloud_ptr
		pcl::ExtractIndices<pcl::PointXYZ> extract_;
		extract_.setInputCloud ( proc_cloud_ptr );
		extract_.setIndices ( test_surf_idx );
		extract_.setNegative (false);
    	extract_.filter (*tmp_test_surf);
	}
	else
		throw std::runtime_error("No points found around the given centroid!!!\n");

	// Cluster the tmp_test_surf and choose only the largest cluster
	std::vector<pcl::PointIndices> test_surf_idx;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr clusters_tree_ (new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_;
	cluster_.setClusterTolerance (0.005);
  	cluster_.setMinClusterSize (100);
  	cluster_.setMaxClusterSize (25000);
  	cluster_.setSearchMethod (clusters_tree_);
	cluster_.setInputCloud ( tmp_test_surf );
  	cluster_.extract ( test_surf_idx );

	// choose the largest cluster
	int best_k = -1;
	{
	unsigned int k = 0, max_size = 0;
	for(std::vector<pcl::PointIndices>::iterator it = test_surf_idx.begin();
			it != test_surf_idx.end(); ++it, ++k)
	{
		if( it->indices.size() > max_size )
		{
			max_size = it->indices.size();
			best_k = static_cast<int>(k);
		}
	}
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr test_surf(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ExtractIndices<pcl::PointXYZ> extract_;
	extract_.setInputCloud ( tmp_test_surf );
  	extract_.setIndices(boost::make_shared<const pcl::PointIndices> ( test_surf_idx[best_k] ));
  	extract_.filter( *test_surf );

	test_surf_ptr = test_surf;
}


int ss_arbiter_main(int argc, char **argv)
{
	ros::init (argc, argv, "ss_arbiter");
	ros::NodeHandle nh;
	ss_arbiter ssa( nh );
	ssa.ss_arbiter_init();
	
	//ros::Rate loop_rate(0.1);
	while (ros::ok())
	{
		ssa.ss_arbiter_spin();
				
		ros::spinOnce();
		//loop_rate.sleep();
	}
	
	return 0;
}

int main(int argc, char **argv)
{
	return ss_arbiter_main(argc, argv);
}




/*
bool
ss_arbiter::process_decide_state_fake( Eigen::Vector3d & proc_position, Eigen::Vector4d & proc_orientation, 
					  							pcl::PointCloud<pcl::PointXYZ>::Ptr proc_cloud_ptr,
					  							Eigen::Vector3d & goal_position, Eigen::Vector4d & goal_orientation )
{

	if( proc_obj_id_vp.first < 0)
		throw std::runtime_error("Invalid object id in DECIDE state!\n");

	// Get the a pointer to the object of interest
	boost::shared_ptr<obj> OBJ_ptr( obj_vec[proc_obj_id_vp.first] );

	// Get the target and the belief
	const Eigen::Vector3d & target = OBJ_ptr-> get_table_position();
	std::vector<double>::iterator bel_start;
	std::vector<double>::iterator bel_end;
	OBJ_ptr->get_belief( bel_start, bel_end);

	// Get the object bounding box
	const Eigen::Vector3d & maxp = OBJ_ptr-> get_maxp();
	const Eigen::Vector3d & minp = OBJ_ptr-> get_minp();
	Eigen::Vector3d midpt = (maxp + minp)/2.0;

	
	// Determine a search point and a radius to be used with a kdtree
	// in order to extract the test surface
	pcl::PointXYZ kdSearchPt( midpt(0), midpt(1), midpt(2) );
	float kdRad = static_cast<float>((maxp - midpt).norm());

	// Use a kdtree to search the proc_cloud for the test surface
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_ (new pcl::search::KdTree<pcl::PointXYZ>);
	kdtree_->setInputCloud( proc_cloud_ptr );
	std::vector<int> match_idx;
	std::vector<float> match_sqdist;

	pcl::PointCloud<pcl::PointXYZ>::Ptr test_surf(new pcl::PointCloud<pcl::PointXYZ>);
	if ( kdtree_->radiusSearch (kdSearchPt, kdRad, match_idx, match_sqdist) > 0 )
	{
		// These are the indices of the test surface in the proc_cloud
		pcl::PointIndices::Ptr test_surf_idx(new pcl::PointIndices);
		test_surf_idx->indices = match_idx;
		
		// Extract the found surface from proc_cloud_ptr
		pcl::ExtractIndices<pcl::PointXYZ> extract_;
		extract_.setInputCloud ( proc_cloud_ptr );
		extract_.setIndices ( test_surf_idx );
		extract_.setNegative (false);
    	extract_.filter (*test_surf);
	}
	else
		throw std::runtime_error("No points found around the given centroid!!!\n");

	// Convert the test surface to the /sensor_optical frame
	pcl::PointCloud<pcl::PointXYZ>::Ptr test_surf_opt(new pcl::PointCloud<pcl::PointXYZ>);
	test_surf_opt -> header.frame_id = "/sensor_optical";
	test_surf -> header.frame_id = "/map";		// just to ensure the frame_id is correct
	bool transform_success = transform_cloud( test_surf, test_surf_opt, proc_cloud_ptr->header.stamp );

	if( !transform_success )
		throw std::runtime_error("Transform for test surface could not be found! Increase tf cache size!\n");



	std::pair<bool,int> isDone_bestAct( std::make_pair(false,-1) );
	isDone_bestAct = decider.decider_spin( proc_position, target, test_surf_opt, bel_start, bel_end, goal_position, goal_orientation );

	if( step_cntr % 5 == 0 ){
		// Simulate a decision
		OBJ_ptr -> set_color(Eigen::Vector3f(0,0,255));
		OBJ_ptr -> set_decision( 0, Eigen::Vector4d(0,0,0,1) ); // null hypothesis
		ps = CHOOSE;
		return false;
	}
	else{
		goal_position = Eigen::Vector3d( misc::uniform_cont(-2.0, 2.0), 
						misc::uniform_cont(-2.0, 2.0), misc::uniform_cont(0.0, 1.5) );
		goal_orientation = misc::target2quat( goal_position, Eigen::Vector3d(0.0,0.0,0.0) );
		return true;
	}
}
*/






/*
bool 
ss_arbiter::sensor_optical2map( pcl::PointCloud<pcl::PointXYZ>::Ptr & cld_ptr )
{
	try
	{
		// Lookup the transform from /sensor_optical to /map
		tf::StampedTransform transform;
		listener.waitForTransform("/sensor_optical", "/map", cld_ptr->header.stamp, ros::Duration(5.0));
		// pTc: child -> parent = lookupTransform("child", "parent", timestamp, transform)
		listener.lookupTransform("/map", "/sensor_optical", cld_ptr->header.stamp, transform);

		// Convert the cld_ptr to the /map frame
		pcl_ros::transformPointCloud( *cld_ptr, *cld_ptr, transform);
		cld_ptr->header.frame_id = "/map";
		return true;
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s", ex.what());
		return false;
	}
}

// takes a cloud in the map frame
void
ss_arbiter::update_disp_cloud( pcl::PointCloud<pcl::PointXYZ>::Ptr & cld_ptr )
{
	// ICP the xyz_cld_ptr and the past rgb clouds
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cld_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	copyPointCloud( *cld_ptr, *new_cld_ptr );
	
	if( disp_cld_ptr->empty() )
		disp_cld_ptr = new_cld_ptr;
	else{
		//ROS_INFO("Size before align: %d", static_cast<int>(disp_cld_ptr->size()) );
		//disp_cld_ptr = pcd_utils::align_clouds( new_cld_ptr, disp_cld_ptr );
		//ROS_INFO("Size after align: %d", static_cast<int>(disp_cld_ptr->size()) );
		*disp_cld_ptr = *new_cld_ptr + *disp_cld_ptr;


		// Shift the old indices
		std::vector<bool> cluster_shifted( hyp_map.size(),false );
		for(std::map<std::pair<int,int>, obj* >::iterator it = hyp_map.begin();
						it != hyp_map.end(); ++it)
		{
			obj *OBJ = hyp_map[it->first];
			
			if(!cluster_shifted[OBJ->get_id()])
			{
				OBJ->shift_idx( new_cld_ptr->size() );
				cluster_shifted[OBJ->get_id()] = true;
			}
		}

	}

	// Filter to reduce size and remove duplicates
	disp_cld_ptr = pcd_utils::voxel_grid_subsample( disp_cld_ptr, 0.01f );	
	ROS_INFO("Size after voxelgrid filter: %d", static_cast<int>(disp_cld_ptr->size()) );


	// Recolor the disp_cld_ptr
	std::vector<bool> cluster_colored( hyp_map.size(), false );
	for(std::map<std::pair<int,int>, obj* >::iterator it = hyp_map.begin();
					it != hyp_map.end(); ++it)
	{
		obj *OBJ = hyp_map[it->first];
		if(!cluster_colored[OBJ->get_id()])
		{

			pcd_utils::color_pointcloud( disp_cld_ptr, OBJ->get_idx(), OBJ->get_color() );
			cluster_colored[OBJ->get_id()] = true;
		}
	}
}
*/

