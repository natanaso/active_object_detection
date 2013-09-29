#include "virtual_kinect_pkg/virtual_kinect.hpp"
#include <misc.hpp>

int
run_virtual_kinect(int argc, char ** argv)
{
	ros::init (argc, argv, "virtual_kinect");
	
	// Set scene
	std::string ply_file_path("/media/natanaso/Data/Stuff/Research/git/icra_2013/ns_shared/penn_nbv_slam/data/test_scenes/test_scene1.ply");
	ros::param::get("~ply_file_path", ply_file_path );
		
	ROS_INFO("The ply file is set to: %s", ply_file_path.c_str());

	// Set initial pose
	double px = 1.0;
	double py = 1.0;
	double pz = 0.0;
	
	ros::param::get("~px", px );
	ros::param::get("~py", py );
	ros::param::get("~pz", pz );

	// Set target point
	double tx = 0.0;
	double ty = 0.0;
	double tz = 0.0;

	ros::param::get("~tx", tx );
	ros::param::get("~ty", ty );
	ros::param::get("~tz", tz );
	
	Eigen::Vector3d init_position( px, py, pz);
	Eigen::Vector3d target( tx, ty, tz );
	Eigen::Vector4d init_orientation = misc::target2quat( init_position, target );

	ROS_INFO_STREAM("The initial position set to: " << init_position.transpose());
	ROS_INFO_STREAM("The initial orientation set to: " << init_orientation.transpose());
	

	//Eigen::Vector3d init_position(0.0,0.0,0.0);
	//Eigen::Vector4d init_orientation(0,0,0,1);
	//Eigen::Vector4d init_orientation( 0.0, 0.0, 0.9239, 0.3827 );

	// Create virtual kinect and initialize
	int coord = 1;	// 0 = obj coor, 1 = optical, 2 = standard
	bool add_noise = true;
	virtual_kinect vk( init_position, init_orientation, coord, false, add_noise );
	vk.init_vkin( ply_file_path );
	
	// Set rate
	ros::Rate loop_rate(20);
	//int cnt = 0;
	//srand((unsigned)time(NULL));
	while ( ros::ok() )
	{
		vk.spin_vkin();
				
		ros::spinOnce();
		loop_rate.sleep();

		/*
		//ROS_INFO("virtual_kinect running... %d",cnt);
		if( cnt % 10 == 0 )
		{
			Eigen::Vector3d np( misc::uniform_cont(-2.0, 2.0), 
						misc::uniform_cont(-2.0, 2.0), misc::uniform_cont(0.0, 1.5) );
			Eigen::Vector4d no = misc::target2quat( np, Eigen::Vector3d(0.0,0.0,0.0) );
			vk.send_goal( np, no );
		}
		cnt++;
		*/
	}
	
	return 0;
}

int
main(int argc, char ** argv)
{
	return run_virtual_kinect(argc, argv);	
}
