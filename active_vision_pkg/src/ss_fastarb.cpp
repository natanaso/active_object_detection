#include <stdexcept>
#include <map>
#include <vector>
#include <boost/filesystem.hpp>	//boost::filesystem::path(fpath).stem().string()

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>

#include <pcl_ros/publisher.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

//PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

//Boost
#include <boost/lexical_cast.hpp>

// Custom
#include <pcd_utils.hpp>
#include <misc.hpp>
#include <metric_map.hpp>			// map of the table
#include <io_utils.hpp>
#include "obj.hpp"
#include <virtual_kinect_pkg/vkin_offline.hpp>
#include <tabletop_analyzer.hpp>
#include "ss_decider.hpp"

// Global Variables
int PROC_OBJ_ID = -1;
int obj_id_ptr_ = 0;
std::vector<double> prior;
metric_map table_map(Eigen::Vector2d(-5,-5), Eigen::Vector2d(5,5), Eigen::Vector2d(0.1, 0.1));
std::map< std::pair<int,int>, boost::shared_ptr<obj> > hyp_map;
pcl::PointCloud<pcl::PointXY>::Ptr hyp_map_centroids( new pcl::PointCloud<pcl::PointXY> );
std::map< int, boost::shared_ptr<obj> > obj_list;
boost::shared_ptr<tabletop_analyzer> TA;

// Visualization
visualization_msgs::Marker pose_hist;

void init_pose_hist()
{
	// Set up the pose history
	pose_hist.header.frame_id = "/map";
	pose_hist.header.stamp = ros::Time(0);	// ensure it does not get deleted by rviz
	pose_hist.type = visualization_msgs::Marker::SPHERE_LIST;
	pose_hist.action = visualization_msgs::Marker::ADD;

	// scale
	pose_hist.scale.x = 0.075;
	pose_hist.scale.y = 0.075;
	pose_hist.scale.z = 0.075;

	// color
	pose_hist.color.r = 0.0f;
	pose_hist.color.g = 1.0f;
	pose_hist.color.b = 0.20f;
	pose_hist.color.a = 1.0;

	// lifetime
	pose_hist.lifetime = ros::Duration();
}

// GLOBAL VARS: obj_list, hyp_map, hyp_map_centroids, table_map
void
update_obj_map( std::vector<tabletop_analyzer::TableInfo>::iterator ti_start,
					 std::vector<tabletop_analyzer::TableInfo>::iterator ti_end )
{
	// Choose the first table and assume it is perpendicular to the z-axis
	tabletop_analyzer::TableInfo & ti = *ti_start;
	int num_obj = ti.obj_surf.size();	
	float dist_rad = 0.2f / 0.1f;	// two centroids are combined if they are less than this distance in [cells] NOT [meters]


	// Reset the latest views of all existing objects
	for( std::map< int, boost::shared_ptr<obj> >::iterator it = obj_list.begin();
			it != obj_list.end(); ++it)
	{
		(it->second) -> reset_latest_view();
	}
	
	 
	// loop thorugh hypmap and match the existing centroids to the newly detected projected centroids
	if( hyp_map_centroids->empty() )
	{

		// add the new objects
		for( int k=0; k < num_obj; ++k)
		{
			// obj_id
			int obj_id = obj_id_ptr_;

			// Object surface
			pcl::PointCloud<pcl::PointXYZ>::Ptr obj_surf_i( ti.obj_surf[k] );
			
			// Color
			//Eigen::Vector3f obj_color_i(0,0,0);
			Eigen::Vector3f obj_color_i(misc::uniform_int(0,255),misc::uniform_int(0,255),misc::uniform_int(0,255));
						
			// Create the object
			boost::shared_ptr<obj> OBJ_ptr( new obj( ti.tab_coefs, obj_id, obj_surf_i,
															     obj_color_i, prior.begin(), prior.end() ) );
															     
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
				Eigen::Vector3f obj_color_i(misc::uniform_int(0,255),misc::uniform_int(0,255),misc::uniform_int(0,255));
				pcl::PointCloud<pcl::PointXYZ>::Ptr obj_surf_i( ti.obj_surf[k] );
				
				
				// Create the object
				boost::shared_ptr<obj> OBJ_ptr( new obj( ti.tab_coefs, obj_id, obj_surf_i,
															     obj_color_i, prior.begin(), prior.end() ) );

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
renew_obj_map( pcl::ModelCoefficients::Ptr & table_coeffs )
{
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
				intersect_ids.insert( (it1->second) -> get_id() );
				intersect_ids.insert( (it2->second) -> get_id() );
			}
		}
	
	// if no intersecting objects we do not need to update
	if(intersect_ids.empty())
		return;
		
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
	
	TA->obj_cld2clusters( table_coeffs, intersect_surf, new_obj_surf,
								new_obj_maxp, new_obj_minp, new_obj_centroids, new_obj_proj_centroids );
	
	// IF the reclustering failed...
	if( new_obj_centroids -> size() < 1)
		return;
	
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
	
	//ROS_INFO("obj_matches.size = %d", static_cast<int>(obj_matches.size()));
	for( std::vector< std::vector<boost::shared_ptr<obj> > >::iterator it1 = obj_matches.begin();
			it1 != obj_matches.end(); ++it1)
		{
			for( std::vector<boost::shared_ptr<obj> >::iterator it2 = it1->begin();
				it2 != it1->end(); ++it2)
				std::cout << (*it2)->get_id() << " ";
			std::cout << std::endl;
		}
	//ROS_INFO("END");
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
		std::vector<double>::iterator bel_start( prior.begin());
		std::vector<double>::iterator bel_end( prior.end());		     
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
}


int fastarb(int argc, char **argv)
{
	if(argc < 5)
		throw std::runtime_error("vp needs to be specified\n");
	if(argc < 4)
		throw std::runtime_error("Experiment number is required as 3rd argument\n");
	if(argc < 3)
		throw std::runtime_error("A file to save the results is required as a second argument\n");
	if(argc < 2)
		throw std::runtime_error("A ply file is required as a first argument...\n");
	
	std::stringstream results_file_path;
	results_file_path << argv[2] << "_" << argv[3] << ".txt";	
	std::string ply_file_path( argv[1] );
	std::string ply_file_name( boost::filesystem::path(ply_file_path).stem().string() );
	std::string ply_info_path( boost::filesystem::path(ply_file_path).parent_path().string() + "/" + ply_file_name + "_info.txt" );
	
	
	// Determine true
	size_t underscore_pos = ply_file_name.find_first_of("_");
	std::string true_str(  ply_file_name.substr(1, underscore_pos-1 ) );
	int true_hyp = std::atoi(true_str.c_str());
	
	std::string pkg_root_dir( ros::package::getPath("active_vision_pkg") );
	std::string vision_module_dir( ros::package::getPath("vision_module") );
	std::string planning_module_dir( ros::package::getPath("planning_module") );
	
	std::string vp_file_path(pkg_root_dir + "/data/plan_vps.txt");
	Eigen::MatrixXd vp_positions;
	io_utils::file2matrix( vp_file_path, vp_positions, 3 );
	//int num_vp = vp_positions.rows();
	//int dim_vp = vp_positions.cols();

	Eigen::MatrixXd true_pose;
	io_utils::file2matrix( ply_info_path, true_pose, 4 );
	//std::cout << "True pose = " << std::endl;
	//std::cout << true_pose << std::endl;
	
	
	// get a virtual kinect object
	int coord = 1; // 0 = obj coord, 1 = optical sensor coord, 2 = standard sensor coord
	bool add_noise = true;
	vkin_offline vko( Eigen::Vector3d(0,0,0), Eigen::Vector4d(0,0,0,1),
					  		coord, false, add_noise );
	
	vko.init_vkin( ply_file_path );
		
	// Choose a starting point at random
	srand((unsigned)time(NULL));
	//int start_vp = misc::uniform_int(0,num_vp-1);
	int start_vp = atoi(argv[4]);
	Eigen::Vector3d snsr_position = vp_positions.row(start_vp).transpose();
	Eigen::Vector3d target(0,0,0);
	Eigen::Vector4d snsr_orientation = misc::target2quat( snsr_position, target );
	
	// Get a decider
	ss_decider sd;
	sd.start_online( pkg_root_dir + "/data/obj_list.txt",
						pkg_root_dir + "/data/hypotheses.txt",
						vision_module_dir + "/data/tree_vps.txt",
						vision_module_dir + "/data/omap_vps.txt",
						pkg_root_dir + "/data/plan_vps.txt",
						pkg_root_dir + "/data/sarsop_data",
						pkg_root_dir + "/data/sarsop_data/oMap.txt",
						pkg_root_dir + "/data/sarsop_data/cMap.txt",
						vision_module_dir + "/data",
						vision_module_dir + "/../database/cloud_data" );

	std::vector<double>::iterator prior_beg;
	std::vector<double>::iterator prior_end;
	sd.get_prior(prior_beg, prior_end);
	prior.assign(prior_beg, prior_end);
	
	// Start a node with a random name
	std::string node_name(argv[3]);	
	ros::init(argc,argv,"exp_"+node_name + "_" + argv[4] + "_" + ply_file_name);
	TA.reset( new tabletop_analyzer());

	
	// Visualization
	ros::NodeHandle nh;
	ros::Publisher pub1(nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ( "snsr_cld", 1, true ));
	ros::Publisher pub2(nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ( "map_cld", 1, true ));
	init_pose_hist();
	ros::Publisher pose_hist_pub(nh.advertise<visualization_msgs::Marker> ("pose_hist", 1, true));
	ros::Publisher curr_pose_pub(nh.advertise<geometry_msgs::PoseStamped> ("curr_pose", 1, true));
	ros::Publisher curr_view_pub(nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ("curr_view", 1, true));
	ros::Publisher obj_view_pub(nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("obj_view", 1, true));
	/**/
	
	// make a done action pair
	std::vector<int> vp_hist;
	std::pair<ss_decider::Decider_State,int> da(std::make_pair(ss_decider::MOVE,-1));
	
	int watchdog_cntr = 0;
	while( da.first != ss_decider::DONE )
	{
	
		// Determine the vp and save
		Eigen::Vector3d tmpa;
		Eigen::Vector4d tmpb;
		int vp = misc::find_closest_viewpoint( target, vp_positions, snsr_position, 
											tmpa, tmpb );						
		vp_hist.push_back(vp);
		
		// Get clound in optical sensor frame
		pcl::PointCloud<pcl::PointXYZ>::Ptr cld_ptr = vko.sense( snsr_position, target );
		cld_ptr->header.frame_id = "/optical";
		cld_ptr->header.stamp = ros::Time(0);
	
		// Convert cloud to /map frame, 
		tf::Transform sensor2map;
		sensor2map.setOrigin( tf::Vector3(snsr_position.x(), snsr_position.y(), snsr_position.z()) );
		sensor2map.setRotation( tf::Quaternion( snsr_orientation.x(), snsr_orientation.y(), 
														snsr_orientation.z(), snsr_orientation.w()) );
	
		tf::Transform optical2sensor;
		optical2sensor.setOrigin( tf::Vector3( 0.0, 0.0, 0.0 ) );
		optical2sensor.setRotation( tf::Quaternion( -0.5, 0.5, -0.5, 0.5 ) );
	
		tf::Transform optical2map = sensor2map * optical2sensor;
		//tf::StampedTransform optical2map_stmp(optical2map, ros::Time(0), "/map","/optical");

		pcl::PointCloud<pcl::PointXYZ>::Ptr map_cld_ptr( new pcl::PointCloud<pcl::PointXYZ>);
		pcl_ros::transformPointCloud( *cld_ptr, *map_cld_ptr, optical2map);
		map_cld_ptr->header.frame_id = "/map";
		map_cld_ptr->header.stamp = ros::Time(0);
	
	
		// Segment the table out and cluster the objects
		std::vector<tabletop_analyzer::TableInfo> ti_vec;
		ti_vec = TA->detect_tabletop_objects( map_cld_ptr );
		bool table_found = (ti_vec.size() > 0);
		if( !table_found ){
			
			// throw std::runtime_error("Table is not visible\n");
			// Use the old table coeffs
		}
		update_obj_map( ti_vec.begin(), ti_vec.end() );
		
		// Select the right object if not done yet
		if(PROC_OBJ_ID == -1)
		{
			// go through the obj list and find the one with table centroid closest to (0,0,0)
			double min_dist = 100;

			for( std::map< int, boost::shared_ptr<obj> >::iterator it = obj_list.begin();
					it != obj_list.end(); ++it)
			{
				Eigen::Vector3d const& tab_pos = (it->second) -> get_table_position();
				double dist = (tab_pos-Eigen::Vector3d(0,0,0)).norm();
				if( dist < min_dist )
				{
					min_dist = dist;
					PROC_OBJ_ID = (it->second)-> get_id();
				}
			}
		}
		else
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr & test_surf1 = obj_list[PROC_OBJ_ID]-> get_latest_view();
			std::cout << "Test surf1 size: " << test_surf1 -> size() << std::endl;
			renew_obj_map( ti_vec[0].tab_coefs );
		}
		
		
		
		// Get the a pointer to the object of interest
		boost::shared_ptr<obj> & OBJ_ptr = obj_list[PROC_OBJ_ID];
		OBJ_ptr -> set_visited_vps(vp, sd.get_num_sta()-1);

		// Display object properties
		//std::cout << *OBJ_ptr << std::endl;
	
		// Get the target and the belief
		target = OBJ_ptr-> get_table_position();
		std::vector<double>::iterator bel_start;
		std::vector<double>::iterator bel_end;
		OBJ_ptr->get_belief( bel_start, bel_end);
	
		// Get the test surface
		pcl::PointCloud<pcl::PointXYZ>::Ptr & test_surf = OBJ_ptr-> get_latest_view();
		test_surf -> header.frame_id = "/map";		// just to ensure the frame_id is correct
		test_surf -> header.stamp = ros::Time(0);
		
		// Convert the test surface to the /sensor_optical frame
		pcl::PointCloud<pcl::PointXYZ>::Ptr test_surf_opt(new pcl::PointCloud<pcl::PointXYZ>);
		test_surf_opt -> header.frame_id = "/optical";
		test_surf -> header.stamp = ros::Time(0);
		pcl_ros::transformPointCloud( *test_surf, *test_surf_opt, optical2map.inverse() );
			
		
		// Get the next decision
		Eigen::Vector3d goal_position;
		Eigen::Vector4d goal_orientation;
		//da = sd.decider_spin_v2( snsr_position, target, test_surf_opt, OBJ_ptr,
		//							 bel_start, bel_end, goal_position, goal_orientation );
		da = sd.decider_spin( snsr_position, target, test_surf_opt, obj_list, PROC_OBJ_ID,
									 goal_position, goal_orientation );
		
		// Display belief
		std::cout << *OBJ_ptr << std::endl;	
		
		
		/*
		// Visualize
		// Current cloud
		//pub1.publish(*cld_ptr);
		pub2.publish(*map_cld_ptr);
		
		// Pose History
		pose_hist.points.resize(vp_hist.size());
		pose_hist.points[vp_hist.size()-1].x = snsr_position.x();
		pose_hist.points[vp_hist.size()-1].y = snsr_position.y();
		pose_hist.points[vp_hist.size()-1].z = snsr_position.z();
		pose_hist_pub.publish(pose_hist);

		// Current Sensor Pose
		geometry_msgs::PoseStamped curr_pose;
		curr_pose.pose.position.x = snsr_position.x();
		curr_pose.pose.position.y = snsr_position.y();
		curr_pose.pose.position.z = snsr_position.z();

		curr_pose.pose.orientation.x = snsr_orientation.x();
		curr_pose.pose.orientation.y = snsr_orientation.y();
		curr_pose.pose.orientation.z = snsr_orientation.z();
		curr_pose.pose.orientation.w = snsr_orientation.w();

		curr_pose.header.frame_id = "/map";
		curr_pose.header.stamp = ros::Time(0);
		curr_pose_pub.publish(curr_pose);
		
		// Current view of the object
		curr_view_pub.publish(*test_surf);
		
		
		// Object surfaces
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr disp_cld_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
		for(std::map< int, boost::shared_ptr<obj> >::iterator it = obj_list.begin();
			it != obj_list.end(); ++it)
			*disp_cld_ptr += *((it->second)->get_surface());
		
		disp_cld_ptr->header.frame_id = "/map";
		disp_cld_ptr->header.stamp = ros::Time(0);		
		obj_view_pub.publish(*disp_cld_ptr);
		
		ros::spinOnce();
		*/
		
		// move
		snsr_position = goal_position;
		snsr_orientation = goal_orientation;
		
		
		// Correct for times when we get stuck
		++watchdog_cntr;
		if( watchdog_cntr > 26 )
			return 0;
	}

	/*
	// display hyp list
	std::cout << "Hypothesis list" << std::endl;
	for( int hid = 0; hid < sd.get_num_hid(); ++hid)
		std::cout << sd.get_hyp(hid) << std::endl;
	*/
	
		
	//std::cout << "True hyp is " << true_hyp << std::endl;
	int guess_hyp = da.second;
	Hypothesis & chosenHyp = sd.get_hyp(guess_hyp);
	

	
	
	double pos_error = std::numeric_limits<double>::infinity();
	double ori_error = std::numeric_limits<double>::infinity();
	double ori_error2 = std::numeric_limits<double>::infinity();
	Eigen::Vector3d guess_position;
	Eigen::Vector4d guess_orientation;
	Eigen::Vector4d guess_orientation2;
	Eigen::Vector4d true_orientation = true_pose.topLeftCorner(1,4).transpose();
	if( chosenHyp.positive )
	{
		boost::shared_ptr<obj> & OBJ_ptr = obj_list[PROC_OBJ_ID];
		

		OBJ_ptr->refine_orientation( chosenHyp, 
				vision_module_dir + "/../database/pcd_models",
				guess_position, guess_orientation );


		Eigen::Vector3d guess_position2;
		
		OBJ_ptr->refine_orientation_v2( chosenHyp, 
				vision_module_dir + "/../database/pcd_models",
				guess_position2, guess_orientation2 );
		
		/*								
		Eigen::Vector3d yaw_pitch_roll = misc::quat2angle(guess_orientation);
		std::cout << "Orientation: " << guess_orientation.transpose() << std::endl;
		std::cout << "YPR: " << (180.0/M_PI)*yaw_pitch_roll.transpose() << std::endl;
		std::cout << "Position: " << guess_position.transpose() << std::endl;
		*/
		
		/*
		Hypothesis & trueHyp = sd.get_hyp(true_hyp);
		Eigen::Vector3d true_YPR( trueHyp.yaw, trueHyp.pitch, trueHyp.roll );
		Eigen::Vector3d unaligned_YPR( chosenHyp.yaw, chosenHyp.pitch, chosenHyp.roll );
		*/
		
		
		pos_error = (Eigen::Vector3d::Zero() - guess_position).norm();
		ori_error = misc::SO3_metric(true_orientation, guess_orientation);
		ori_error2 = misc::SO3_metric(true_orientation, guess_orientation2);
		
		// ori_error = misc::SO3_metric(misc::angle2quat( true_YPR ), guess_orientation);
		//double ori_error_ua = misc::SO3_metric(misc::angle2quat( true_YPR ), misc::angle2quat( unaligned_YPR ));
		
		//std::cout << "Position Error = " << pos_error << std::endl;
		//std::cout << "Orientation Error = " << (180.0/M_PI)*ori_error << std::endl;
		//std::cout << "Unaligned Orientation Error = " << (180.0/M_PI)*ori_error_ua << std::endl;
	}
	
	bool mistake = true;
	if ( (true_hyp == guess_hyp) || 
		 ( true_hyp >= sd.get_num_pos_hyp() && guess_hyp >= sd.get_num_pos_hyp() ) )
		mistake = false;

	double mistake_cost = 0.0;
	if(mistake)
		mistake_cost = 75;
	
	int num_msrmnts = vp_hist.size();
	
	double move_cost = 0;
	boost::multi_array<double,2> const& dMap = sd.get_dMap();
	for(int vp = 1; vp < num_msrmnts; ++vp)
		move_cost += dMap[vp_hist[vp-1]][vp_hist[vp]];
	
	
	//std::cout << "Guessed hyp is " << guess_hyp << std::endl;
	std::cout << num_msrmnts << std::endl;
	std::cout << move_cost << std::endl;
	std::cout << mistake_cost << std::endl;
	
	/*
	std::cout << "VPS" << std::endl;
	for(int vp = 0; vp < num_msrmnts; ++vp)
		std::cout << vp_hist[vp] << std::endl;
	*/
		
	// record the results
	std::ofstream outfile ( results_file_path.str().c_str(), std::ofstream::app );
	if(!outfile.is_open())
		throw std::runtime_error("Unable to open save file...\n");
		
	outfile << start_vp << " " << num_msrmnts << " " 
			  << move_cost << " " << mistake_cost << " "
			  << true_hyp << " " << guess_hyp << " "
			  << ori_error << " " << pos_error << " "
			  << ori_error2 << " "
			  << true_orientation.transpose() << " "
			  << guess_orientation.transpose() << " "
			  << guess_orientation2.transpose() << " "
			  << guess_position.transpose() << std::endl;
	
	outfile.close();
	
	
	return 0;
}


int main(int argc, char **argv)
{
	return fastarb(argc, argv);
}
