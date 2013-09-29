#include <string>

// Boost
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>

#include <visualization_msgs/MarkerArray.h>

#include "ood_visualization.hpp"



// Constructor
ood_visualization::ood_visualization(ros::NodeHandle &anode, int version)
	: nh_(anode), ver(version), hist_length(0), 
	  is_init_pose_hist_pub(false), is_init_cloud_hist_pub(false)
{
	ROS_INFO("Creating an ood_visualization object with version %d!",version);
	curr_pose_pub = nh_.advertise<ood_visualization::PoseS> ("curr_pose", 1, true);
	curr_cloud_pub = nh_.advertise<ood_visualization::PointCloudRGB> ("curr_cloud", 1, true);
	marray_pub = nh_.advertise<visualization_msgs::Marker> ("centroids", 1, true);
	bbox_pub = nh_.advertise<visualization_msgs::MarkerArray> ("bboxes", 1, true);
	id_pub = nh_.advertise<visualization_msgs::MarkerArray> ("obj_ids", 1, true);
	curr_view_pub = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> > ("curr_view", 1, true);
	
	tst_cld_pub1 = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("tst_cld1", 1, true);
	tst_cld_pub2 = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("tst_cld2", 1, true);
	tst_cld_pub3 = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("tst_cld3", 1, true);
}

// Destructor
ood_visualization::~ood_visualization()
{
	ROS_INFO("Shutting down ood_visualization node!");
	
	if((ver == 1)||(ver == 2)){
		// Clear saved cloud data
		for(std::vector<ros::Publisher>::iterator it = rgbcloud_pub_vec.begin(); it != rgbcloud_pub_vec.end(); ++it)
			it->shutdown();
		rgbcloud_pub_vec.clear();

		rgbcloud_vec.clear();
	
	
		// Clear saved pose data
		for(std::vector<ros::Publisher>::iterator it = pose_pub_vec.begin(); it != pose_pub_vec.end(); ++it)
			it->shutdown();
		pose_pub_vec.clear();
		
		pose_vec.clear();
	}
}


void 
ood_visualization::add_curr_cloud(PointCloudRGB::Ptr cloud)
{
	//ROS_INFO("Publishing pointcloud of size %d...", cloud->size());
	cloud->header.frame_id = "/map";
  	cloud->header.stamp = ros::Time(0);	// ensure it does not get deleted by rviz
	curr_cloud_pub.publish(*cloud);
}
		

void 
ood_visualization::connectCallback_pc(const ros::SingleSubscriberPublisher& pub)
{
	const std::string& publisher_topic = pub.getTopic();
	std::string info("ood_connectCallback_pc on topic " + publisher_topic);
	ROS_INFO("%s",info.c_str());
	
	ROS_INFO("Publishing corresponding pointcloud...");
	
	// the index of the point cloud in rgbcloud_vec is publisher_topic.substr(underscore_loc)
	std::size_t underscore_loc = publisher_topic.find_last_of("_",publisher_topic.size());
	//ROS_INFO("The location of _ is: %d", underscore_loc);

	int cloud_idx = std::atoi((publisher_topic.substr(underscore_loc+1)).c_str());
	//ROS_INFO("The cloud index is: %d", cloud_idx);
	
	pub.publish(*(rgbcloud_vec[cloud_idx]));
	
	// we can erase the point cloud here
	rgbcloud_vec[cloud_idx] = PointCloudRGB::Ptr();
}

void
ood_visualization::connectCallback_pp(const ros::SingleSubscriberPublisher& pub)
{
	const std::string& publisher_topic = pub.getTopic();
	std::string info("ood_connectCallback_pp on topic " + publisher_topic);
	ROS_INFO("%s",info.c_str());
	
	ROS_INFO("Publishing corresponding pose...");
	
	// the index of the pose in pose_vec is publisher_topic.substr(underscore_loc)
	std::size_t underscore_loc = publisher_topic.find_last_of("_",publisher_topic.size());
	int pose_idx = std::atoi((publisher_topic.substr(underscore_loc+1)).c_str());
	pub.publish(pose_vec[pose_idx]);
}


void 
ood_visualization::disconnectCallback(const ros::SingleSubscriberPublisher& pub)
{
	ROS_INFO("ood_disconnectCallback");
}


void
ood_visualization::add_hist_cloud_v1(PointCloudRGB::Ptr cloud)
{
	// Set topic name and initialize the publisher
	char numstr[21];
	sprintf(numstr, "%d", static_cast<int>(rgbcloud_pub_vec.size()));
	
	std::string pub_name("pcp_");
	
	ROS_INFO("Initializing Point Cloud publisher...");
	ros::Publisher pc_pub = nh_.advertise<ood_visualization::PointCloudRGB> (pub_name + numstr, 1, boost::bind(&ood_visualization::connectCallback_pc, this, _1),
														  						 				boost::bind(&ood_visualization::disconnectCallback, this, _1));
														  						 				
	rgbcloud_pub_vec.push_back(pc_pub);
	  	
  	// Save a copy of the rgb cloud
  	cloud->header.frame_id = "/map";
  	cloud->header.stamp = ros::Time(0);	// ensure it does not get deleted by rviz
  	rgbcloud_vec.push_back(cloud);													  						 												  						 	
}

template <typename T>
void
ood_visualization::add_hist_pose_v1(Eigen::Matrix<T,3,1> const &position, Eigen::Matrix<T,4,1> const &orientation)
{
	//  ood_visualization::PoseS pose;
	//	pose.header.frame_id = "/my_frame";
	//	
	//	pose.pose.position.x = position.x();
	//	pose.pose.position.y = position.y();
	//	pose.pose.position.z = position.z();
	//	
	//	pose.pose.orientation.x = orientation.x();
	//	pose.pose.orientation.y = orientation.y();
	//	pose.pose.orientation.z = orientation.z();
	//	pose.pose.orientation.w = orientation.w();
	//		
	//	pose.header.stamp = ros::Time::now ();
	//	curr_pose_pub.publish(pose);
	
	// save the pose
	ood_visualization::PoseM pose_marker;
	pose_marker.header.frame_id = "/map";
	pose_marker.header.stamp = ros::Time(0);	// ensure it does not get deleted by rviz

	pose_marker.type = visualization_msgs::Marker::SPHERE;
	pose_marker.action = visualization_msgs::Marker::ADD;
	
	pose_marker.pose.position.x = position.x();
	pose_marker.pose.position.y = position.y();
	pose_marker.pose.position.z = position.z();

	pose_marker.pose.orientation.x = orientation.x();
	pose_marker.pose.orientation.y = orientation.y();
	pose_marker.pose.orientation.z = orientation.z();
	pose_marker.pose.orientation.w = orientation.w();

	// scale
	pose_marker.scale.x = 0.075;
	pose_marker.scale.y = 0.075;
	pose_marker.scale.z = 0.075;
	
	// color
	pose_marker.color.r = 0.0f;
	pose_marker.color.g = 1.0f;
	pose_marker.color.b = 0.50f;
	pose_marker.color.a = 1.0;
	
	// lifetime
	pose_marker.lifetime = ros::Duration();
	
	pose_vec.push_back(pose_marker);
	
	// create publisher for the history
	char numstr[21];
	sprintf(numstr, "%d", static_cast<int>(pose_pub_vec.size()));
	
	std::string pub_name("pp_");
	
	ROS_INFO("Initializing Pose publisher...");
	ros::Publisher pc_pub = nh_.advertise<ood_visualization::PoseM> (pub_name + numstr, 1, boost::bind(&ood_visualization::connectCallback_pp, this, _1),
														  						 			 boost::bind(&ood_visualization::disconnectCallback, this, _1));
	pose_pub_vec.push_back(pc_pub);
													  			
}

void
ood_visualization::add_hist_cloud_v2(PointCloudRGB::Ptr cloud)
{
	// Save a pointer to the point cloud
	cloud->header.frame_id = "/map";
  	cloud->header.stamp = ros::Time(0);	// ensure it does not get deleted by rviz
  	rgbcloud_vec.push_back(cloud);
  	
  	
	// Set topic name and initialize the publisher
	char numstr[21];
	sprintf(numstr, "%d", static_cast<int>(rgbcloud_pub_vec.size()));
	
	std::string pub_name("pcp_");
	
	ROS_INFO("Initializing Point Cloud publisher...");
															  						 				
	ros::Publisher pc_pub = nh_.advertise<ood_visualization::PointCloudRGB> (pub_name + numstr, 1, true);

	rgbcloud_pub_vec.push_back(pc_pub);
	pc_pub.publish(*cloud);	
}

template <typename T>
void
ood_visualization::add_hist_pose_v2(Eigen::Matrix<T,3,1> const &position, Eigen::Matrix<T,4,1> const &orientation)
{
	//	ood_visualization::PoseS pose;
	//	pose.header.frame_id = "/my_frame";
	//	
	//	pose.pose.position.x = position.x();
	//	pose.pose.position.y = position.y();
	//	pose.pose.position.z = position.z();
	//	
	//	pose.pose.orientation.x = orientation.x();
	//	pose.pose.orientation.y = orientation.y();
	//	pose.pose.orientation.z = orientation.z();
	//	pose.pose.orientation.w = orientation.w();
	//		
	//	pose.header.stamp = ros::Time::now ();
	//	curr_pose_pub.publish(pose);
	
	// save the pose
	ood_visualization::PoseM pose_marker;
	pose_marker.header.frame_id = "/map";
	pose_marker.header.stamp = ros::Time(0);	// ensure it does not get deleted by rviz

	pose_marker.type = visualization_msgs::Marker::SPHERE;
	pose_marker.action = visualization_msgs::Marker::ADD;
	
	pose_marker.pose.position.x = position.x();
	pose_marker.pose.position.y = position.y();
	pose_marker.pose.position.z = position.z();

	pose_marker.pose.orientation.x = orientation.x();
	pose_marker.pose.orientation.y = orientation.y();
	pose_marker.pose.orientation.z = orientation.z();
	pose_marker.pose.orientation.w = orientation.w();
	
	// scale
	pose_marker.scale.x = 0.075;
	pose_marker.scale.y = 0.075;
	pose_marker.scale.z = 0.075;
	
	// color
	pose_marker.color.r = 0.0f;
	pose_marker.color.g = 1.0f;
	pose_marker.color.b = 0.20f;
	pose_marker.color.a = 1.0;
	
	// lifetime
	pose_marker.lifetime = ros::Duration();
	
	// Save the pose marker
	pose_vec.push_back(pose_marker);
	
	// create publisher for the history
	char numstr[21];
	sprintf(numstr, "%d", static_cast<int>(pose_pub_vec.size()));
	
	std::string pub_name("pp_");
	
	ROS_INFO("Initializing Pose publisher...");
	ros::Publisher pc_pub = nh_.advertise<ood_visualization::PoseM> (pub_name + numstr, 1, true);
	pc_pub.publish(pose_marker);

	pose_pub_vec.push_back(pc_pub);	
}

void
ood_visualization::add_hist_cloud_v3(PointCloudRGB::Ptr cloud)
{
	ROS_INFO("Publishing a cloud and adding it to the cloud history...");
	// Combine old and new cloud
	cloud_hist += (*cloud);	// think about removing duplicates!
	
	cloud_hist_pub.publish(cloud_hist);
}

template <typename T>
void
ood_visualization::add_hist_pose_v3(Eigen::Matrix<T,3,1> const &position, Eigen::Matrix<T,4,1> const &orientation)
{
	//ROS_INFO("Publishing a pose and adding it to the pose history...");
	//	ood_visualization::PoseS pose;
	//	pose.header.frame_id = "/my_frame";
	//	
	//	pose.pose.position.x = position.x();
	//	pose.pose.position.y = position.y();
	//	pose.pose.position.z = position.z();
	//	
	//	pose.pose.orientation.x = orientation.x();
	//	pose.pose.orientation.y = orientation.y();
	//	pose.pose.orientation.z = orientation.z();
	//	pose.pose.orientation.w = orientation.w();
	//		
	//	//pose.header.stamp = ros::Time::now ();
	//	pose.header.stamp = ros::Time(0);
	//	curr_pose_pub.publish(pose);
	
	pose_hist.points.resize(hist_length+1);
	pose_hist.points[hist_length].x = position.x();
	pose_hist.points[hist_length].y = position.y();
	pose_hist.points[hist_length].z = position.z();
	++hist_length;
	
	pose_hist_pub.publish(pose_hist);
}

void
ood_visualization::add_hist_cloud(PointCloudRGB::Ptr cloud)
{
	switch(ver){
		case 1:
			add_hist_cloud_v1(cloud);
			break;
			
		case 2:
			add_hist_cloud_v2(cloud);
			break;
			
		case 3:
			if(!is_init_cloud_hist_pub)
				init_cloud_hist_pub();
			add_hist_cloud_v3(cloud);
			break;
	}
}

void
ood_visualization::init_pose_hist_pub()
{
	// Pose history stuff
	pose_hist_pub = nh_.advertise<visualization_msgs::Marker> ("pose_hist", 1, true);

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

void
ood_visualization::init_cloud_hist_pub()
{
	// Cloud history stuff
	cloud_hist_pub = nh_.advertise<ood_visualization::PointCloudRGB> ("cloud_hist", 1, true);
	cloud_hist.header.frame_id = "/map";
  	cloud_hist.header.stamp = ros::Time(0);	// ensure it does not get deleted by rviz
}




void 
ood_visualization::add_centroids( const Eigen::MatrixXd & mat_3xN )
{
	int num_pts = mat_3xN.cols();

	PoseM marray;

	marray.header.frame_id = "/map";
	marray.header.stamp = ros::Time(0);	// ensure it does not get deleted by rviz
	marray.type = visualization_msgs::Marker::SPHERE_LIST;
	marray.action = visualization_msgs::Marker::ADD;

	// scale
	marray.scale.x = 0.03;
	marray.scale.y = 0.03;
	marray.scale.z = 0.03;

	// color
	marray.color.r = 1.0f;
	marray.color.g = 0.0f;
	marray.color.b = 0.2f;
	marray.color.a = 1.0;

	// lifetime
	marray.lifetime = ros::Duration();

	marray.points.resize(num_pts);
	for( int k = 0; k < num_pts; ++k){
		marray.points[k].x = mat_3xN(0,k);
		marray.points[k].y = mat_3xN(1,k);
		marray.points[k].z = mat_3xN(2,k);
	}

	marray_pub.publish(marray);
}

void 
ood_visualization::add_bbox( const Eigen::MatrixXd & maxp_3xN,
										 const Eigen::MatrixXd & minp_3xN,
										 const Eigen::MatrixXf & colors )
{
	int num_bbox = maxp_3xN.cols();
	
	visualization_msgs::MarkerArray bboxes;
	bboxes.markers.resize( num_bbox );
	for( int i = 0; i < num_bbox; ++i)
	{
		bboxes.markers[i].header.frame_id = "/map";
		bboxes.markers[i].header.stamp = ros::Time(0);	// ensure it does not get deleted by rviz
		bboxes.markers[i].id = i;
		bboxes.markers[i].type = visualization_msgs::Marker::CUBE;
		bboxes.markers[i].action = visualization_msgs::Marker::ADD;
		
		// pose
		Eigen::Vector3d cntr = (maxp_3xN.col(i) + minp_3xN.col(i))/2.0;
		bboxes.markers[i].pose.position.x = cntr.x();
		bboxes.markers[i].pose.position.y = cntr.y();
		bboxes.markers[i].pose.position.z = cntr.z();
		bboxes.markers[i].pose.orientation.x = 0.0;
		bboxes.markers[i].pose.orientation.y = 0.0;
		bboxes.markers[i].pose.orientation.z = 0.0;
		bboxes.markers[i].pose.orientation.w = 1.0;
    
		// scale
		bboxes.markers[i].scale.x = maxp_3xN(0,i) - minp_3xN(0,i);
		bboxes.markers[i].scale.y = maxp_3xN(1,i) - minp_3xN(1,i);
		bboxes.markers[i].scale.z = maxp_3xN(2,i) - minp_3xN(2,i);

		// color
		bboxes.markers[i].color.r = colors(0,i);
		bboxes.markers[i].color.g = colors(1,i);
		bboxes.markers[i].color.b = colors(2,i);
		bboxes.markers[i].color.a = 0.5;

		// lifetime
		bboxes.markers[i].lifetime = ros::Duration();
	}
	
	bbox_pub.publish( bboxes );	
}

void 
ood_visualization::add_ids( const Eigen::MatrixXd & maxp_3xN,
									 const Eigen::VectorXd & obj_ids )
{
	int num_ids = obj_ids.size();
	visualization_msgs::MarkerArray ids;
	ids.markers.resize( num_ids );
	
	for( int i = 0; i < num_ids; ++i)
	{
		ids.markers[i].text = boost::lexical_cast<std::string>(obj_ids(i));
		ids.markers[i].header.frame_id = "/map";
		ids.markers[i].header.stamp = ros::Time(0);	// ensure it does not get deleted by rviz
		ids.markers[i].id = i;
		ids.markers[i].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		ids.markers[i].action = visualization_msgs::Marker::ADD;
		
		// pose
		ids.markers[i].pose.position.x = maxp_3xN(0,i);
		ids.markers[i].pose.position.y = maxp_3xN(1,i);
		ids.markers[i].pose.position.z = maxp_3xN(2,i);
		ids.markers[i].pose.orientation.x = 0.0;
		ids.markers[i].pose.orientation.y = 0.0;
		ids.markers[i].pose.orientation.z = 0.0;
		ids.markers[i].pose.orientation.w = 1.0;
    
		// scale
		ids.markers[i].scale.x = 0.0;
		ids.markers[i].scale.y = 0.0;
		ids.markers[i].scale.z = 0.05;

		// color
		ids.markers[i].color.r = 1.0;
		ids.markers[i].color.g = 0.0;
		ids.markers[i].color.b = 0.0;
		ids.markers[i].color.a = 1.0;

		// lifetime
		ids.markers[i].lifetime = ros::Duration();
	}	
	
	id_pub.publish( ids );
}


void 
ood_visualization::add_curr_view( pcl::PointCloud<pcl::PointXYZ>::Ptr & cld_ptr)
{
  	cld_ptr->header.stamp = ros::Time(0);	// ensure it does not get deleted by rviz
	curr_view_pub.publish(*cld_ptr);
}

// Explicit Template Instantiations
template void ood_visualization::add_hist_pose_v3(Eigen::Matrix<int,3,1> const &position, Eigen::Matrix<int,4,1> const &orientation);
template void ood_visualization::add_hist_pose_v3(Eigen::Matrix<float,3,1> const &position, Eigen::Matrix<float,4,1> const &orientation);
template void ood_visualization::add_hist_pose_v3(Eigen::Matrix<double,3,1> const &position, Eigen::Matrix<double,4,1> const &orientation);
template void ood_visualization::add_hist_pose_v2(Eigen::Matrix<int,3,1> const &position, Eigen::Matrix<int,4,1> const &orientation);
template void ood_visualization::add_hist_pose_v2(Eigen::Matrix<float,3,1> const &position, Eigen::Matrix<float,4,1> const &orientation);
template void ood_visualization::add_hist_pose_v2(Eigen::Matrix<double,3,1> const &position, Eigen::Matrix<double,4,1> const &orientation);
template void ood_visualization::add_hist_pose_v1(Eigen::Matrix<int,3,1> const &position, Eigen::Matrix<int,4,1> const &orientation);
template void ood_visualization::add_hist_pose_v1(Eigen::Matrix<float,3,1> const &position, Eigen::Matrix<float,4,1> const &orientation);
template void ood_visualization::add_hist_pose_v1(Eigen::Matrix<double,3,1> const &position, Eigen::Matrix<double,4,1> const &orientation);

/*
void connectCallback(const ros::SingleSubscriberPublisher&)
{
  ROS_INFO("connectCallback");
}

void disconnectCallback(const ros::SingleSubscriberPublisher&)
{
  ROS_INFO("disconnectCallback");
}
*/



