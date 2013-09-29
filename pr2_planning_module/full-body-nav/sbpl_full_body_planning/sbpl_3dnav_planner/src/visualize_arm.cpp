/* \author Ben Cohen */

#include <sbpl_3dnav_planner/visualize_arm.h>
#include <time.h>

namespace sbpl_full_body_planner
{

void HSVtoRGB( double *r, double *g, double *b, double h, double s, double v )
{
	int i;
	double f, p, q, t;
	if( s == 0 ) {
		// achromatic (grey)
		*r = *g = *b = v;
		return;
	}
	h /= 60;        // sector 0 to 5
	i = floor(h);
	f = h - i;			// factorial part of h
	p = v * ( 1 - s );
	q = v * ( 1 - s * f );
	t = v * ( 1 - s * ( 1 - f ) );
	switch( i ) {
		case 0:
			*r = v;
			*g = t;
			*b = p;
			break;
		case 1:
			*r = q;
			*g = v;
			*b = p;
			break;
		case 2:
			*r = p;
			*g = v;
			*b = t;
			break;
		case 3:
			*r = p;
			*g = q;
			*b = v;
			break;
		case 4:
			*r = t;
			*g = p;
			*b = v;
			break;
    default:
			*r = v;
			*g = p;
			*b = q;
			break;
	}
}

VisualizeArm::VisualizeArm(std::string arm_name) : ph_("~"), arm_name_(arm_name)
{
  num_joints_ = 7;
  reference_frame_ = "torso_lift_link";
  chain_root_name_ = "torso_lift_link";
  chain_tip_name_ = "_gripper_r_finger_tip_link";
  chain2_tip_name_ = "_gripper_l_finger_tip_link";

  srand (time(NULL));

  joint_names_.push_back("_shoulder_pan_joint");
  joint_names_.push_back("_shoulder_lift_joint");
  joint_names_.push_back("_upper_arm_roll_joint");
  joint_names_.push_back("_elbow_flex_joint");
  joint_names_.push_back("_forearm_roll_joint");
  joint_names_.push_back("_wrist_flex_joint");
  joint_names_.push_back("_wrist_roll_joint");

  link_names_.push_back("_shoulder_pan_link");
  link_names_.push_back("_shoulder_lift_link");
  link_names_.push_back("_upper_arm_roll_link");
  link_names_.push_back("_elbow_flex_link");
  link_names_.push_back("_forearm_roll_link");
  link_names_.push_back("_wrist_flex_link");
  link_names_.push_back("_wrist_roll_link");

  if(arm_name == "left_arm")
  {
    side_ = "l";
    side_full_ = "left";
    i_arm_ = 1;
  }
  else
  {
    side_ = "r";
    side_full_ = "right";
    i_arm_ = 0;
  }

  for(size_t i = 0; i < joint_names_.size(); ++i)
    joint_names_[i] = side_ + joint_names_[i];

  for(size_t i = 0; i < link_names_.size(); ++i)
    link_names_[i] = side_ + link_names_[i];

  fk_service_name_ = "pr2_" + side_full_ + "_arm_kinematics/get_fk";
  ik_service_name_ = "pr2_" + side_full_ + "_arm_kinematics/get_ik";
  chain_tip_name_ = side_ + chain_tip_name_;
  chain2_tip_name_ = side_ + chain2_tip_name_;

  // arm meshes
  pr2_arm_meshes_.push_back("package://pr2_description/meshes/shoulder_v0/shoulder_yaw.stl");
  pr2_arm_meshes_.push_back("package://pr2_description/meshes/shoulder_v0/shoulder_lift.stl");
  pr2_arm_meshes_.push_back("package://pr2_description/meshes/upper_arm_v0/upper_arm.stl");
  pr2_arm_meshes_.push_back("package://pr2_description/meshes/upper_arm_v0/elbow_flex.stl");
  //pr2_arm_meshes_.push_back("package://pr2_description/meshes/upper_arm_v0/forearm_roll.mesh");  
  pr2_arm_meshes_.push_back("package://pr2_description/meshes/forearm_v0/forearm.stl");
  pr2_arm_meshes_.push_back("package://pr2_description/meshes/forearm_v0/wrist_flex.stl");
  pr2_arm_meshes_.push_back("package://pr2_description/meshes/forearm_v0/wrist_roll.stl");
  pr2_arm_meshes_.push_back("package://pr2_description/meshes/gripper_v0/gripper_palm.stl");

  // gripper meshes
  pr2_gripper_meshes_.push_back("package://pr2_description/meshes/gripper_v0/upper_finger_r.stl");
  pr2_gripper_meshes_.push_back("package://pr2_description/meshes/gripper_v0/finger_tip_r.stl");
  pr2_gripper_meshes_.push_back("package://pr2_description/meshes/gripper_v0/upper_finger_l.stl");
  pr2_gripper_meshes_.push_back("package://pr2_description/meshes/gripper_v0/finger_tip_l.stl");

  initKDLChain();

  marker_array_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 500);
  marker_publisher_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
  display_trajectory_publisher_ = nh_.advertise<arm_navigation_msgs::DisplayTrajectory>("arm_viz", 200);
}

VisualizeArm::~VisualizeArm()
{
  delete fk_solver_;
  delete gripper_l_fk_solver_;
}

bool VisualizeArm::parsePoseFile(std::string filename, std::vector<std::vector<double> > &poses)
{
  std::ifstream input_file(filename.c_str());
  if(!input_file.good())
  {
    printf("[parseCSVFile] Unable to open '%s' for reading.\n",filename.c_str());
    return false;
  }

  int row(0), col(0);
  char line[256];
  input_file.seekg(0);

  row = -1;
  poses.clear();

  while (input_file.getline(line, 256, ','))
  {   
    poses.resize(poses.size()+1);
    poses[++row].resize(6);
    poses[row][0] = atof(line);
    printf("%d poses_size: %d\n", row, (int)poses.size());
    printf("%0.3f ", poses[row][0]);

    for(col = 1; col < 6; col++)
    {
      input_file.getline(line, 256, ',');
      poses[row][col] = atof(line);
      printf("%0.3f ",poses[row][col]);
    }
//    input_file.getline(line, 256);
//    poses[row][col] = atof(line);
//    printf("%0.3f ",poses[row][col]);
    printf("\n");
  }
  printf("num rows: %d\n",(int)poses.size());
  return true;
}

bool VisualizeArm::parseCSVFile(std::string filename, int num_cols, std::vector<std::vector<double> > &data)
{
  std::ifstream input_file(filename.c_str());
  if(!input_file.good())
  {
    printf("[parseCSVFile] Unable to open '%s' for reading.\n",filename.c_str());
    return false;
  }

  int row(0), col(0);
  char line[256];
  input_file.seekg(0);

  std::vector<std::vector<double> > raw_data;

  row = -1;
  raw_data.clear();

  while (input_file.getline(line, 256, ','))
  {
    raw_data.resize(raw_data.size()+1);
    raw_data[++row].resize(num_cols);
    raw_data[row][0] = atof(line);

    for(col = 1; col < num_cols; col++)
    {
      input_file.getline(line, 256, ',');
      raw_data[row][col] = atof(line);
    }
  }

  std::vector<double> zero_line(num_cols,0);
  for(int i = 0; i < int(raw_data.size()); i++)
  {
    if(raw_data[i] != zero_line)
      data.push_back(raw_data[i]);
  }

  ROS_DEBUG("[parseCSVFile] raw_data: num rows: %d",(int)raw_data.size());
  ROS_DEBUG("[parseCSVFile] data: num rows: %d",(int)data.size());
  return true;
}

bool VisualizeArm::parseEnvironmentFile(std::string filename)
{
  std::ifstream input_file(filename.c_str());
  if(!input_file.good())
  {
    printf("[parseEnvironmentFile] Unable to open %s for reading.\n",filename.c_str());
    return false;
  }

  char line[256];
  input_file.seekg(0);
  std::vector<double> cube_temp(6,0);

  goal_pose_.resize(6,0);
  start_config_.resize(7,0);

  while (input_file.getline(line, 256,' ') && strlen(line) > 0)
  {
    if(strcmp(line,"linkstartangles(radians):") == 0)
    {
      for(int i=0; i < num_joints_; i++)
      {
        if(input_file.getline(line, 256, ' '))
          start_config_[i] = atof(line);
      }
      input_file.getline(line,256);
    }
    else if(strcmp(line,"endeffectorgoal(meters):") == 0)
    {
      if(input_file.getline(line, 256))
      {
        for(int i=0; i < atoi(line); i++)
        {
          for(int j = 0; j < 6; j++)
            if(input_file.getline(line, 256, ' '))
              goal_pose_[j] = atof(line);

          if(input_file.getline(line, 256))
            goal_is_6dof_ = atoi(line);
        }
      }
    }
    else if(strcmp(line,"goal_tolerance(meters,radians):") == 0)
    {
      if(input_file.getline(line, 256, ' '))
        position_tolerance_ = atof(line);

      if(input_file.getline(line, 256, ' '))
        orientation_tolerance_ = atof(line);
      input_file.getline(line,256);
    }
    else if(strcmp(line,"cube:") == 0)
    {
      for(int i=0; i < 6; i++)
      {
        if(input_file.getline(line, 256, ' '))
          cube_temp[i] = atof(line);
      }
      cubes_.push_back(cube_temp);
      input_file.getline(line,256);
    }
    else
    {
      ROS_ERROR("[parseEnvironmentFile] Error: unrecognized text in environment file.");
      input_file.getline(line,256);
    }
  }
  return true;
}

void VisualizeArm::displayArmConfiguration(std::vector<double> angles)
{
  //requires motion planning rviz plugin - not working right now
  while(display_trajectory_publisher_.getNumSubscribers() < 1 && nh_.ok())
  {
    ROS_INFO("Waiting for subscriber");
    ros::Duration(0.3).sleep();
  }

  arm_navigation_msgs::DisplayTrajectory display_trajectory;

  display_trajectory.model_id = "pr2";
  display_trajectory.trajectory.joint_trajectory.header.frame_id = "base_link";
  display_trajectory.trajectory.joint_trajectory.header.stamp = ros::Time::now();
  display_trajectory.trajectory.joint_trajectory.joint_names = joint_names_;
  display_trajectory.trajectory.joint_trajectory.points.resize(2);
  display_trajectory.trajectory.joint_trajectory.points[0].positions.resize(joint_names_.size());
  display_trajectory.trajectory.joint_trajectory.points[1].positions.resize(joint_names_.size());


  for(unsigned int i=0; i < joint_names_.size(); i++)
  {
    display_trajectory.trajectory.joint_trajectory.points[0].positions[i] = angles[i];
    display_trajectory.trajectory.joint_trajectory.points[1].positions[i] = angles[i];
  }

  display_trajectory.robot_state.joint_state = joint_state_monitor_.getJointStateRealJoints();
  ROS_INFO("Publishing path for display");
  display_trajectory_publisher_.publish(display_trajectory);
  joint_state_monitor_.stop();
}

void VisualizeArm::printEnvironmentInfo(FILE *fid)
{
  fprintf(fid, "start configuration: ");
  for(int i = 0; i < (int)start_config_.size(); i++)
    fprintf(fid, "%0.3f  ",start_config_[i]);
  fprintf(fid, "\n");

  fprintf(fid, "goal pose: ");
  for(int i = 0; i < (int)goal_pose_.size(); i++)
    fprintf(fid, "%0.3f  ",goal_pose_[i]);
  fprintf(fid, "\n");

  fprintf(fid,"position tolerance (m): %0.3f\n", position_tolerance_);
  fprintf(fid,"orientation tolerance (m): %0.3f\n", orientation_tolerance_);
  fprintf(fid,"goal is 6dof constraint: %d\n", goal_is_6dof_);

  fprintf(fid, "obstacles: %d\n", int(cubes_.size()));
  for(int j = 0; j < int(cubes_.size()); j++)
  {
    for(int i = 0; i < int(cubes_[j].size()); i++)
      fprintf(fid, "%0.3f  ",cubes_[j][i]);
    fprintf(fid, "\n");
  }
}

bool VisualizeArm::computeFKforVisualization(const std::vector<double> &jnt_pos, std::vector<geometry_msgs::PoseStamped> &poses)
{
  kinematics_msgs::GetPositionFK::Request  request;
  kinematics_msgs::GetPositionFK::Response response;

  request.header.stamp = ros::Time();
  request.header.frame_id = reference_frame_;

  request.robot_state.joint_state.name = joint_names_;

  ROS_DEBUG("%d joints", (int)jnt_pos.size());

  for(unsigned int j = 0 ; j < jnt_pos.size(); ++j)
    request.robot_state.joint_state.position.push_back(jnt_pos[j]);

  ROS_DEBUG("%d joints, %d positions",int(request.robot_state.joint_state.name.size()),int(request.robot_state.joint_state.position.size()));

  request.fk_link_names.push_back(link_names_[0]);
  request.fk_link_names.push_back(link_names_[1]);
  request.fk_link_names.push_back(link_names_[2]);
  request.fk_link_names.push_back(link_names_[3]);
  request.fk_link_names.push_back(link_names_[4]);
  request.fk_link_names.push_back(link_names_[5]);
  request.fk_link_names.push_back(link_names_[6]);
  request.fk_link_names.push_back(link_names_[6]);

  ros::service::waitForService(fk_service_name_);
  ros::ServiceClient client = nh_.serviceClient<kinematics_msgs::GetPositionFK>(fk_service_name_);

  ROS_DEBUG("[computeFKforVisualization] about to call service...");
  if(client.call(request, response))
  {
    if(response.error_code.val == response.error_code.SUCCESS)
    {
      poses = response.pose_stamped;
      return true;
    }
    else
    {
      ROS_ERROR("[computeFKforVisualization] FK service failed with error code %d", response.error_code.val);
      return false;
    }
  }
  else
  {
    ROS_ERROR("FK service failed");
    return false;
  }
}

bool VisualizeArm::initKDLChain()
{
  std::string robot_description;
  std::string robot_param;
  nh_.searchParam("robot_description",robot_param);
  nh_.param<std::string>(robot_param,robot_description,"robot_description");

  if (!kdl_parser::treeFromString(robot_description, kdl_tree_))
  {
    printf("Failed to parse tree from manipulator description file.\n");
    return false;
  }

  if (!kdl_tree_.getChain(chain_root_name_, chain_tip_name_, chain_))
  {
    printf("Error: could not fetch the KDL chain for the desired manipulator. Exiting.\n"); 
    return false;
  }

  fk_solver_ = new KDL::ChainFkSolverPos_recursive(chain_);
  jnt_pos_in_.resize(chain_.getNrOfJoints());
  jnt_pos_out_.resize(chain_.getNrOfJoints());

  double jnt_pos = 0;
  for(unsigned int j = 0; j < chain_.getNrOfSegments(); ++j)
  {
    ROS_DEBUG("frame %2d: segment: %0.3f %0.3f %0.3f  joint: %0.3f %0.3f %0.3f   joint_type: %s",j,
        chain_.getSegment(j).pose(jnt_pos).p.x(),
        chain_.getSegment(j).pose(jnt_pos).p.y(),
        chain_.getSegment(j).pose(jnt_pos).p.z(),
        chain_.getSegment(j).getJoint().pose(jnt_pos).p.x(),
        chain_.getSegment(j).getJoint().pose(jnt_pos).p.y(),
        chain_.getSegment(j).getJoint().pose(jnt_pos).p.z(),
        chain_.getSegment(j).getJoint().getTypeName().c_str());
  }

  ROS_DEBUG("[initKDLChain] the arm chain has %d segments with %d joints", chain_.getNrOfSegments(), chain_.getNrOfJoints());

  //added 7/4/2010
  if (!kdl_tree_.getChain(chain_root_name_, chain2_tip_name_, gripper_l_chain_))
  {
    printf("Error: could not fetch the KDL chain for the desired manipulator. Exiting.\n"); 
    return false;
  }
  ROS_DEBUG("left finger KDL chain:\n");
  for(unsigned int j = 0; j < gripper_l_chain_.getNrOfSegments(); ++j)
  {
    ROS_DEBUG("frame %2d: segment: %0.3f %0.3f %0.3f  joint: %0.3f %0.3f %0.3f   joint_type: %s\n",j,
        gripper_l_chain_.getSegment(j).pose(jnt_pos).p.x(),
        gripper_l_chain_.getSegment(j).pose(jnt_pos).p.y(),
        gripper_l_chain_.getSegment(j).pose(jnt_pos).p.z(),
        gripper_l_chain_.getSegment(j).getJoint().pose(jnt_pos).p.x(),
        gripper_l_chain_.getSegment(j).getJoint().pose(jnt_pos).p.y(),
        gripper_l_chain_.getSegment(j).getJoint().pose(jnt_pos).p.z(),
        gripper_l_chain_.getSegment(j).getJoint().getTypeName().c_str());
  }

  gripper_l_fk_solver_ = new KDL::ChainFkSolverPos_recursive(gripper_l_chain_);

  return true;
}

bool VisualizeArm::computeFKwithKDL(const std::vector<double> angles, int frame_num, geometry_msgs::Pose &pose)
{
  KDL::Frame frame_out;
  for(int i = 0; i < num_joints_; ++i)
    jnt_pos_in_(i) = angles[i];

  if(frame_num > 0)
  {
    if(fk_solver_->JntToCart(jnt_pos_in_, frame_out, frame_num) < 0)
    {
      printf("JntToCart returned < 0. Exiting.\n");
      return false;
    }
  }
  else
  {
    if(gripper_l_fk_solver_->JntToCart(jnt_pos_in_, frame_out, -1*frame_num) < 0)
    {
      printf("JntToCart returned < 0. Exiting.\n");
      return false;
    }
  }

  pose.position.x = frame_out.p[0];
  pose.position.y = frame_out.p[1];
  pose.position.z = frame_out.p[2];

  frame_out.M.GetQuaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);

  return true;
}

bool VisualizeArm::computeFKforVisualizationWithKDL(const std::vector<double> &jnt_pos, std::vector<geometry_msgs::PoseStamped> &poses)
{
  //output is PoseStamped only so it has same interface as computeFKforVisualization
  
  geometry_msgs::Pose pose;

  poses.resize(pr2_arm_meshes_.size());
  for(int i=0; i < (int)pr2_arm_meshes_.size(); i++)
  {
    if(!computeFKwithKDL(jnt_pos, i, poses[i].pose))
      return false;
    ROS_DEBUG("pose: %d: %0.2f %0.2f %0.2f",i, poses[i].pose.position.x, poses[i].pose.position.y,poses[i].pose.position.z);
  }
  return true;
}

void VisualizeArm::visualizeObstacles(const std::vector<std::vector<double> > &obstacles)
{
  marker_array_.markers.clear();
  marker_array_.markers.resize(obstacles.size());

  ROS_INFO("Displaying %d obstaclesin the %s frame", (int)obstacles.size(), reference_frame_.c_str());

  std::string ns = "obstacles"+boost::lexical_cast<std::string>(rand());

  for(int i = 0; i < int(obstacles.size()); i++)
  {
    if(obstacles[i].size() < 6)
    {
      ROS_INFO("Obstacle description doesn't have length = 6");
      continue;
    }

    //TODO: Change this to use a CUBE_LIST
    marker_array_.markers[i].header.stamp = ros::Time::now();
    marker_array_.markers[i].header.frame_id = reference_frame_;
    marker_array_.markers[i].ns = ns;
    marker_array_.markers[i].id = rand();
    marker_array_.markers[i].type = visualization_msgs::Marker::CUBE;
    marker_array_.markers[i].action = visualization_msgs::Marker::ADD;
    marker_array_.markers[i].pose.position.x = obstacles[i][0];
    marker_array_.markers[i].pose.position.y = obstacles[i][1];
    marker_array_.markers[i].pose.position.z = obstacles[i][2];
    marker_array_.markers[i].scale.x = obstacles[i][3];
    marker_array_.markers[i].scale.y = obstacles[i][4];
    marker_array_.markers[i].scale.z = obstacles[i][5];
    marker_array_.markers[i].color.r = 0.0;
    marker_array_.markers[i].color.g = 0.0;
    marker_array_.markers[i].color.b = 0.5;
    marker_array_.markers[i].color.a = 0.9;
    marker_array_.markers[i].lifetime = ros::Duration(500.0);
  }

  marker_array_publisher_.publish(marker_array_);
}

void VisualizeArm::visualizeEnvironment(std::string filename)
{
  parseEnvironmentFile(filename);
  visualizePose(goal_pose_, filename);
  visualizeArmConfiguration(10, start_config_);
  visualizeObstacles(cubes_);
}

void VisualizeArm::visualizeTrajectoryFile(std::string filename, int throttle)
{
  std::vector<std::vector<double> > traj;
  if(!parseCSVFile(filename, num_joints_, traj))
    printf("[aviz] Parsing %s failed.\n",filename.c_str());

  visualizeArmConfigurations(traj,throttle);
}

void VisualizeArm::visualizeJointTrajectoryMsg(trajectory_msgs::JointTrajectory traj_msg, int throttle)
{
  std::vector<std::vector<double> > traj;

  if(traj_msg.points.empty())
  {
    ROS_WARN("Trajectory message is empty. Not visualizing anything.");
    return;
  }

  traj.resize(traj_msg.points.size());
  for(size_t i = 0; i < traj_msg.points.size(); i++)
  {
    traj[i].resize(traj_msg.points[i].positions.size());
    for(size_t j = 0; j < traj_msg.points[i].positions.size(); j++)
      traj[i][j] = traj_msg.points[i].positions[j];
  }

  ROS_INFO("[aviz] Visualizing trajectory of length %d with the throttle set to %d.",int(traj.size()),throttle);

  visualizeArmConfigurations(traj,throttle);
}

void VisualizeArm::visualizeArmConfigurations(const std::vector<std::vector<double> > &traj, int throttle)
{
  double color_inc = 260/traj.size();   //260 is blue
  std::vector<double> zero_config(num_joints_,0);

  //always print out first & last configs
  visualizeArmConfiguration(color_inc*(1), traj[0]);
  visualizeArmConfiguration(color_inc*((traj.size()-1)+1), traj[traj.size()-1]);
  
  for(int i = 1; i < (int)traj.size(); i++)
  {
    //hack: some trajectories get parsed with an additional vector of zeroes 
    if(traj[i] == zero_config)
    {
      ROS_WARN("[visualizeArmConfigurations] Not displaying arm configurations of all zeroes.");
      continue;
    }

    if(i % throttle != 0)
      continue;

    visualizeArmConfiguration(color_inc*(i+1), traj[i]);
  }
}

void VisualizeArm::visualizePoses(const std::vector<std::vector<double> > &poses)
{
  marker_array_.markers.clear();
  marker_array_.markers.resize(poses.size()*3);
  tf::Quaternion pose_quaternion;
  geometry_msgs::Quaternion quaternion_msg;

  int mind = -1;

  ros::Time time = ros::Time::now();

  for(int i = 0; i < (int)poses.size(); ++i)
  {
    pose_quaternion.setRPY(poses[i][3],poses[i][4],poses[i][5]);
    tf::quaternionTFToMsg(pose_quaternion, quaternion_msg);

    mind++;
    marker_array_.markers[mind].header.stamp = time;
    marker_array_.markers[mind].header.frame_id = reference_frame_;
    marker_array_.markers[mind].ns = "pose_arrows";
    marker_array_.markers[mind].type = visualization_msgs::Marker::ARROW;
    marker_array_.markers[mind].id = i;
    marker_array_.markers[mind].action = visualization_msgs::Marker::ADD;
    marker_array_.markers[mind].pose.position.x = poses[i][0];
    marker_array_.markers[mind].pose.position.y = poses[i][1];
    marker_array_.markers[mind].pose.position.z = poses[i][2];
    marker_array_.markers[mind].pose.orientation = quaternion_msg;
    marker_array_.markers[mind].scale.x = 0.1;
    marker_array_.markers[mind].scale.y = 0.1;
    marker_array_.markers[mind].scale.z = 0.1;
    marker_array_.markers[mind].color.r = 0.0;
    marker_array_.markers[mind].color.g = 0.7;
    marker_array_.markers[mind].color.b = 0.6;
    marker_array_.markers[mind].color.a = 0.7;
    marker_array_.markers[mind].lifetime = ros::Duration(500.0);

    mind++;
    marker_array_.markers[mind].header.stamp = time;
    marker_array_.markers[mind].header.frame_id = reference_frame_;
    marker_array_.markers[mind].ns = "pose_spheres";
    marker_array_.markers[mind].id = i;
    marker_array_.markers[mind].type = visualization_msgs::Marker::SPHERE;
    marker_array_.markers[mind].action = visualization_msgs::Marker::ADD;
    marker_array_.markers[mind].pose.position.x = poses[i][0];
    marker_array_.markers[mind].pose.position.y = poses[i][1];
    marker_array_.markers[mind].pose.position.z = poses[i][2];
    marker_array_.markers[mind].pose.orientation = quaternion_msg;
    marker_array_.markers[mind].scale.x = 0.07;
    marker_array_.markers[mind].scale.y = 0.07;
    marker_array_.markers[mind].scale.z = 0.07;
    marker_array_.markers[mind].color.r = 1.0;
    marker_array_.markers[mind].color.g = 0.0;
    marker_array_.markers[mind].color.b = 0.6;
    marker_array_.markers[mind].color.a = 0.6;
    marker_array_.markers[mind].lifetime = ros::Duration(500.0);

    mind++;
    marker_array_.markers[mind].header.stamp = time;
    marker_array_.markers[mind].header.frame_id = reference_frame_;
    marker_array_.markers[mind].ns = "pose_text_blocks";
    marker_array_.markers[mind].id = i;
    marker_array_.markers[mind].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_array_.markers[mind].action = visualization_msgs::Marker::ADD;
    marker_array_.markers[mind].pose.position.x = poses[i][0];
    marker_array_.markers[mind].pose.position.y = poses[i][1];
    marker_array_.markers[mind].pose.position.z = poses[i][2];
    marker_array_.markers[mind].scale.x = 0.03;
    marker_array_.markers[mind].scale.y = 0.03;
    marker_array_.markers[mind].scale.z = 0.03;
    marker_array_.markers[mind].color.r = 1.0;
    marker_array_.markers[mind].color.g = 1.0;
    marker_array_.markers[mind].color.b = 1.0;
    marker_array_.markers[mind].color.a = 0.9;
    marker_array_.markers[mind].text = boost::lexical_cast<std::string>(i+1);
    marker_array_.markers[mind].lifetime = ros::Duration(500.0);
  }

  ROS_INFO("%d markers in the array",(int)marker_array_.markers.size());
  marker_array_publisher_.publish(marker_array_);
}

void VisualizeArm::visualizePose(const std::vector<double> &pose, std::string text)
{
  tf::Quaternion pose_quaternion;
  geometry_msgs::Pose pose_msg;

  pose_msg.position.x = pose[0];
  pose_msg.position.y = pose[1];
  pose_msg.position.z = pose[2];

  pose_quaternion.setRPY(pose[3],pose[4],pose[5]);
  tf::quaternionTFToMsg(pose_quaternion, pose_msg.orientation);

  ROS_DEBUG("[visualizing: %s] position: %0.3f %0.3f %0.3f quaternion: %0.3f %0.3f %0.3f %0.3f (frame: %s)", text.c_str(), pose[0], pose[1], pose[2], pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w, reference_frame_.c_str());

  visualizePose(pose_msg, text);
}

void VisualizeArm::visualizePose(const geometry_msgs::Pose &pose, std::string text)
{
  int mind = -1;
  marker_array_.markers.clear();
  marker_array_.markers.resize(2);
  ros::Time time = ros::Time::now();

  ROS_INFO("[aviz] %s: position: %0.3f %0.3f %0.3f quaternion: %0.3f %0.3f %0.3f %0.3f (frame: %s)", text.c_str(), pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w, reference_frame_.c_str());
  
  mind++;
  marker_array_.markers[mind].header.stamp = time;
  marker_array_.markers[mind].header.frame_id = reference_frame_;
  marker_array_.markers[mind].ns = text + "_arrow";
  marker_array_.markers[mind].type = visualization_msgs::Marker::ARROW;
  marker_array_.markers[mind].id = 0;
  marker_array_.markers[mind].action = visualization_msgs::Marker::ADD;
  marker_array_.markers[mind].pose = pose;
  marker_array_.markers[mind].scale.x = 0.1; //0.125;
  marker_array_.markers[mind].scale.y = 0.1; //0.125;
  marker_array_.markers[mind].scale.z = 0.1; //0.125;
  marker_array_.markers[mind].color.r = 1.0;
  marker_array_.markers[mind].color.g = 0.0;
  marker_array_.markers[mind].color.b = 0.9;
  marker_array_.markers[mind].color.a = 1.0;
  marker_array_.markers[mind].lifetime = ros::Duration(500.0);

  mind++;
  marker_array_.markers[mind].header.stamp = time;
  marker_array_.markers[mind].header.frame_id = reference_frame_;
  marker_array_.markers[mind].ns = text + "_sphere";
  marker_array_.markers[mind].id = 1;
  marker_array_.markers[mind].type = visualization_msgs::Marker::SPHERE;
  marker_array_.markers[mind].action = visualization_msgs::Marker::ADD;
  marker_array_.markers[mind].pose = pose;
  marker_array_.markers[mind].scale.x = 0.08; //0.1
  marker_array_.markers[mind].scale.y = 0.08; //0.1
  marker_array_.markers[mind].scale.z = 0.08; //0.1
  marker_array_.markers[mind].color.r = 1.0; //1.0;
  marker_array_.markers[mind].color.g = 165.0/255.0; //0.0;
  marker_array_.markers[mind].color.b = 79.0/255.0; //0.6;
  marker_array_.markers[mind].color.a = 1.0;
  marker_array_.markers[mind].lifetime = ros::Duration(500.0);

  /*
    mind++;
    marker_array_.markers[mind].header.stamp = time;
    marker_array_.markers[mind].header.frame_id = reference_frame_;
    marker_array_.markers[mind].ns = text;
    marker_array_.markers[mind].id = 2;
    marker_array_.markers[mind].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_array_.markers[mind].action = visualization_msgs::Marker::ADD;
    marker_array_.markers[mind].pose = pose;
    marker_array_.markers[mind].scale.x = 0.03;
    marker_array_.markers[mind].scale.y = 0.03;
    marker_array_.markers[mind].scale.z = 0.03;
    marker_array_.markers[mind].color.r = 1.0;
    marker_array_.markers[mind].color.g = 1.0;
    marker_array_.markers[mind].color.b = 1.0;
    marker_array_.markers[mind].color.a = 0.9;
    marker_array_.markers[mind].text = text;
    marker_array_.markers[mind].lifetime = ros::Duration(180.0);
  */

  marker_array_publisher_.publish(marker_array_);
}

void VisualizeArm::visualizeSphere(std::vector<double> pose, int color, std::string text, double radius)
{
  double r=0,g=0,b=0;
  visualization_msgs::Marker marker;

  HSVtoRGB(&r, &g, &b, color, 1.0, 1.0);

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = reference_frame_;
  marker.ns = text + "-sphere";
  marker.id = 1;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = pose[0];
  marker.pose.position.y = pose[1];
  marker.pose.position.z = pose[2];
  marker.scale.x = radius*2;
  marker.scale.y = radius*2;
  marker.scale.z = radius*2;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 0.6;
  marker.lifetime = ros::Duration(1000.0);

  marker_publisher_.publish(marker);
}

void VisualizeArm::visualizeSpheres(const std::vector<std::vector<double> > &pose, int color, std::string text, double radius)
{
  double r=0,g=0,b=0;
  visualization_msgs::Marker marker;

  HSVtoRGB(&r, &g, &b, color, 1.0, 1.0);

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = reference_frame_;
  marker.ns = "spheres-" + text;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = radius*2.0;
  marker.scale.y = radius*2.0;
  marker.scale.z = radius*2.0;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 0.8;
  marker.lifetime = ros::Duration(1000.0);
  marker.id = 1;

  marker.points.resize(pose.size());
  for(size_t i = 0; i < pose.size(); i++)
  {
    marker.points[i].x = pose[i][0];
    marker.points[i].y = pose[i][1];
    marker.points[i].z = pose[i][2];
  }

  marker_publisher_.publish(marker);
  ROS_INFO("[aviz] Visualizing %d spheres.", int(marker.points.size()));
}

void VisualizeArm::visualizeSpheres(const std::vector<std::vector<double> > &pose, int color, std::string text, std::vector<double> &radius)
{
  double r=0,g=0,b=0;
  visualization_msgs::Marker marker;

  HSVtoRGB(&r, &g, &b, color, 1.0, 1.0);

  for(size_t i = 0; i < pose.size(); ++i)
  {
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = reference_frame_;
    marker.ns = text;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = radius[i]*2.0;
    marker.scale.y = radius[i]*2.0;
    marker.scale.z = radius[i]*2.0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 0.6;
    marker.lifetime = ros::Duration(500.0);
    marker.id = i;

    marker.pose.position.x = pose[i][0];
    marker.pose.position.y = pose[i][1];
    marker.pose.position.z = pose[i][2];

    marker_publisher_.publish(marker);
    usleep(100);
  }
}

void VisualizeArm::visualizeArmMeshes(double color_num, std::vector<geometry_msgs::PoseStamped> &poses)
{
  double r,g,b;
  marker_array_.markers.clear();
  marker_array_.markers.resize(pr2_arm_meshes_.size());
  ros::Time time = ros::Time();

  HSVtoRGB(&r, &g, &b, color_num, 1.0, 1.0);

  for(int i = 0; i < (int)marker_array_.markers.size(); ++i)
  {
    marker_array_.markers[i].header.stamp = time;
    marker_array_.markers[i].header.frame_id = reference_frame_;
    marker_array_.markers[i].ns = side_full_ + "_arm_" + boost::lexical_cast<std::string>(color_num);
    marker_array_.markers[i].type = visualization_msgs::Marker::MESH_RESOURCE;
    marker_array_.markers[i].id = i;
    marker_array_.markers[i].action = visualization_msgs::Marker::ADD;
    marker_array_.markers[i].pose = poses.at(i).pose;
    marker_array_.markers[i].scale.x = 1.0;  
    marker_array_.markers[i].scale.y = 1.0;
    marker_array_.markers[i].scale.z = 1.0;

    marker_array_.markers[i].color.r = r;
    marker_array_.markers[i].color.g = g;
    marker_array_.markers[i].color.b = b;
    marker_array_.markers[i].color.a = 0.4;
    marker_array_.markers[i].lifetime = ros::Duration(500.0);
    marker_array_.markers[i].mesh_resource = pr2_arm_meshes_[i];
  }

  marker_array_publisher_.publish(marker_array_);
}

void VisualizeArm::visualizeGripperConfiguration(double color_num, const std::vector<double> &jnt_pos)
{
  std::vector<geometry_msgs::PoseStamped> poses;

  poses.resize(4);

  //HACK: Trying to make sure the gripper is always closed
  std::vector<double> jnt_pos2(jnt_pos);
  jnt_pos2.push_back(0.0);
  jnt_pos2.push_back(0.0);
  jnt_pos2.push_back(0.0);

  //r_upper_finger
  computeFKwithKDL(jnt_pos2,11,poses[0].pose);

  //r_gripper_finger_tip_r
  computeFKwithKDL(jnt_pos2,12,poses[1].pose);

  //l_upper_finger
  computeFKwithKDL(jnt_pos2,-11,poses[2].pose);

  //r_gripper_finger_tip_l
  computeFKwithKDL(jnt_pos2,-12,poses[3].pose);

  visualizeGripperMeshes(color_num, poses);
}

void VisualizeArm::visualizeGripperMeshes(double color_num, std::vector<geometry_msgs::PoseStamped> &poses)
{
  double r,g,b;
  marker_array_.markers.clear();
  marker_array_.markers.resize(pr2_gripper_meshes_.size());

  HSVtoRGB(&r, &g, &b, color_num, 1.0, 1.0);

  for(int i = 0; i < (int)marker_array_.markers.size(); ++i)
  {
    marker_array_.markers[i].header.stamp = ros::Time();
    marker_array_.markers[i].header.frame_id = chain_root_name_;
    marker_array_.markers[i].ns = side_full_ + "_gripper_" + boost::lexical_cast<std::string>(color_num);
    marker_array_.markers[i].type = visualization_msgs::Marker::MESH_RESOURCE;
    marker_array_.markers[i].id = i;
    marker_array_.markers[i].action = visualization_msgs::Marker::ADD;
    marker_array_.markers[i].pose = poses.at(i).pose;

    marker_array_.markers[i].scale.x = 1.0;  
    marker_array_.markers[i].scale.y = 1.0;
    marker_array_.markers[i].scale.z = 1.0;

    marker_array_.markers[i].color.r = r;
    marker_array_.markers[i].color.g = g;
    marker_array_.markers[i].color.b = b;
    marker_array_.markers[i].color.a = 0.4;
    marker_array_.markers[i].lifetime = ros::Duration(500.0);
    marker_array_.markers[i].mesh_resource = pr2_gripper_meshes_[i];
  }

  marker_array_publisher_.publish(marker_array_);
}

void VisualizeArm::visualizeArmConfiguration(double color_num, const std::vector<double> &jnt_pos)
{
  std::vector<geometry_msgs::PoseStamped> poses;
  if(!computeFKforVisualization(jnt_pos, poses))
    ROS_INFO("[visualizeArmConfiguration] Unable to compute forward kinematics.");
  else
    visualizeArmMeshes(color_num, poses);

  visualizeGripperConfiguration(color_num,jnt_pos);
}

void VisualizeArm::visualizeCollisionModel(const std::vector<std::vector<double> > &path, sbpl_full_body_planner::PR2CollisionSpace &cspace, int throttle)
{
  std::vector<std::vector<double> > cylinders;
  visualization_msgs::MarkerArray marker_array;
  
  for(int i = 0; i < int(path.size()); ++i)
  {
    if(i % throttle != 0)
    {
      if(i != 0 && i != int(path.size())-1)
        continue;
    }
 
    ROS_ERROR("[aviz] getCollisionCylinders is not functioning right now. Please try again later.");
    return;
  
    /* 
    cylinders.resize(0);
    if(!cspace.getCollisionCylinders(path[i], i_arm_, cylinders))
      ROS_WARN("[visualizeCollisionModel] Cannot display arm collision model.");
    */
    ROS_DEBUG("[visualizeCollisionModel] waypoint #%d contains %d cylinders.", i, (int)cylinders.size());
    marker_array.markers.resize(cylinders.size());

    for(int j = 0; j < int(cylinders.size()); ++j)
    {
      marker_array.markers[j].header.frame_id = reference_frame_;
      marker_array.markers[j].header.stamp = ros::Time::now();
      marker_array.markers[j].ns = side_full_ + "_arm_model_" + boost::lexical_cast<std::string>(i);
      marker_array.markers[j].id = j;
      marker_array.markers[j].type = visualization_msgs::Marker::SPHERE;
      marker_array.markers[j].action =  visualization_msgs::Marker::ADD;
      marker_array.markers[j].scale.x = cylinders[j][3] * 2;
      marker_array.markers[j].scale.y = cylinders[j][3] * 2;
      marker_array.markers[j].scale.z = cylinders[j][3] * 2;
      

      marker_array.markers[j].color.r = 0.6;
      marker_array.markers[j].color.g = 0.6;
      marker_array.markers[j].color.b = 0.0;
      marker_array.markers[j].color.a = 0.3;
      marker_array.markers[j].lifetime = ros::Duration(120.0);

      marker_array.markers[j].pose.position.x = cylinders[j][0];
      marker_array.markers[j].pose.position.y = cylinders[j][1];
      marker_array.markers[j].pose.position.z = cylinders[j][2];

      ROS_DEBUG("[visualizeCollisionModel]           center: %0.3f %0.3f %0.3f radius: %0.3fm", cylinders[j][0], cylinders[j][1], cylinders[j][2],cylinders[j][3]);
    }
    
    if(cylinders.size() > 0)
      marker_array_publisher_.publish(marker_array);
  }
}

void VisualizeArm::visualizeCollisionModel(const std::vector<std::vector<double> > &path, sbpl_full_body_planner::PR2CollisionSpace &cspace, int throttle, int hue)
{
  std::vector<std::vector<double> > cylinders;
  visualization_msgs::MarkerArray marker_array;
  double r,g,b;
  HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);

  for(int i = 0; i < int(path.size()); ++i)
  {
    if(i % throttle != 0)
    {
      if(i != 0 && i != int(path.size())-1)
        continue;
    }
   
    ROS_ERROR("[aviz] getCollisionCylinders is not functioning right now. Please try again later.");
    return;
    /*
    cylinders.resize(0);
    if(!cspace.getCollisionCylinders(path[i], i_arm_, cylinders))
      ROS_WARN("[visualizeCollisionModel] Cannot display arm collision model.");
    */
    ROS_DEBUG("[visualizeCollisionModel] waypoint #%d contains %d cylinders.", i, (int)cylinders.size());
    marker_array.markers.resize(cylinders.size());

    for(int j = 0; j < int(cylinders.size()); ++j)
    {
      marker_array.markers[j].header.frame_id = reference_frame_;
      marker_array.markers[j].header.stamp = ros::Time::now();
      marker_array.markers[j].ns = side_full_ + "_arm_model_" + boost::lexical_cast<std::string>(i);
      marker_array.markers[j].id = j;
      marker_array.markers[j].type = visualization_msgs::Marker::SPHERE;
      marker_array.markers[j].action =  visualization_msgs::Marker::ADD;
      marker_array.markers[j].scale.x = cylinders[j][3] * 2;
      marker_array.markers[j].scale.y = cylinders[j][3] * 2;
      marker_array.markers[j].scale.z = cylinders[j][3] * 2;
      

      marker_array.markers[j].color.r = r;
      marker_array.markers[j].color.g = g;
      marker_array.markers[j].color.b = b;
      marker_array.markers[j].color.a = 0.9;
      marker_array.markers[j].lifetime = ros::Duration(360.0);

      marker_array.markers[j].pose.position.x = cylinders[j][0];
      marker_array.markers[j].pose.position.y = cylinders[j][1];
      marker_array.markers[j].pose.position.z = cylinders[j][2];

      ROS_DEBUG("[visualizeCollisionModel]           center: %0.3f %0.3f %0.3f radius: %0.3fm", cylinders[j][0], cylinders[j][1], cylinders[j][2],cylinders[j][3]);
    }
    
    if(cylinders.size() > 0)
      marker_array_publisher_.publish(marker_array);
  }
}

void VisualizeArm::deleteVisualizations(std::string ns, int max_id)
{
  marker_array_.markers.clear();
  marker_array_.markers.resize(max_id);

  for(int j = 0; j < max_id; j++)
  {
    marker_array_.markers[j].header.stamp = ros::Time::now();
    marker_array_.markers[j].header.frame_id = reference_frame_;
    marker_array_.markers[j].ns = ns;
    marker_array_.markers[j].action = visualization_msgs::Marker::DELETE;
    marker_array_.markers[j].id = j;
  }
  marker_array_publisher_.publish(marker_array_);
}

void VisualizeArm::visualizeCollisionModelFromJointTrajectoryMsg(trajectory_msgs::JointTrajectory &traj_msg, sbpl_full_body_planner::PR2CollisionSpace &cspace, int throttle)
{
  std::vector<std::vector<double> > traj;

  if(traj_msg.points.empty())
  {
    ROS_WARN("Trajectory message is empty. Not visualizing anything.");
    return;
  }

  traj.resize(traj_msg.points.size());
  for(size_t i = 0; i < traj_msg.points.size(); i++)
  {
    traj[i].resize(traj_msg.points[i].positions.size());
    for(size_t j = 0; j < traj_msg.points[i].positions.size(); j++)
      traj[i][j] = traj_msg.points[i].positions[j];
  }

  ROS_DEBUG("[visualizeCollisionModelFromJointTrajectoryMsg] Visualizing collision model of trajectory with length %d ",int(traj.size()));

  visualizeCollisionModel(traj, cspace, throttle);
}

void VisualizeArm::visualize3DPath(std::vector<std::vector<double> > &dpath)
{
  if(dpath.empty())
  {
    ROS_INFO("[visualizeShortestPath] The shortest path is empty.");
    return;
  }
  else
    ROS_INFO("[visualizeShortestPath] There are %i waypoints in the shortest path.",int(dpath.size()));

  visualization_msgs::Marker obs_marker;
  obs_marker.header.frame_id = reference_frame_;
  obs_marker.header.stamp = ros::Time();
  obs_marker.header.seq = 0;
  obs_marker.ns = "path";
  obs_marker.id = 0;
  obs_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  obs_marker.action = 0;
  obs_marker.scale.x = 3*0.02;
  obs_marker.scale.y = 3*0.02;
  obs_marker.scale.z = 3*0.02;
  obs_marker.color.r = 0.45;
  obs_marker.color.g = 0.3;
  obs_marker.color.b = 0.4;
  obs_marker.color.a = 0.8;
  obs_marker.lifetime = ros::Duration(360.0);

  obs_marker.points.resize(dpath.size());

  for (int k = 0; k < int(dpath.size()); k++)
  {
    if(int(dpath[k].size()) < 3)
      continue;

    obs_marker.points[k].x = dpath[k][0];
    obs_marker.points[k].y = dpath[k][1];
    obs_marker.points[k].z = dpath[k][2];
  }

  marker_publisher_.publish(obs_marker);
}

void VisualizeArm::visualizeBasicStates(const std::vector<std::vector<double> > &states, const std::vector<double> &color, std::string name, double size)
{
  unsigned int inc = 1;
  visualization_msgs::Marker marker;
  
  //check if the list is empty
  if(states.empty())
  {
    ROS_DEBUG("[visualizeBasicStates] There are no states in the %s states list.", name.c_str());
    return;
  }

  //if there are too many states, rviz will crash and burn when drawing
  if(states.size() > 50000)
    inc = 4;
  else if(states.size() > 10000)
    inc = 1;   //2
  else
    inc = 1;

  marker.points.resize(states.size()/inc + 1);

  marker.header.seq = 0;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = reference_frame_;

  marker.ns = name;
  marker.id = 1;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = size;
  marker.scale.y = size;
  marker.scale.z = size;
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = color[3];
  marker.lifetime = ros::Duration(500.0);

  unsigned int m_ind = 0;
  for(unsigned int i = 0; i < states.size(); i=i+inc)
  {
    if(states[i].size() >= 3)
    {
      marker.points[m_ind].x = states[i][0];
      marker.points[m_ind].y = states[i][1];
      marker.points[m_ind].z = states[i][2];
      ++m_ind;
    }
  }

  marker_publisher_.publish(marker);
  ROS_DEBUG("[visualizeBasicStates] published %d markers for %s states", int(marker.points.size()), name.c_str());
}

void VisualizeArm::visualizeDetailedStates(const std::vector<std::vector<double> > &states, const std::vector<std::vector<double> >&color, std::string name, double size)
{
  unsigned int inc = 1;
  std::vector<double> scaled_color(4,0);
  visualization_msgs::MarkerArray marker_array;

  //check if the list is empty
  if(states.empty())
  {
    ROS_INFO("[aviz] There are no states in the %s states list", name.c_str());
    return;
  } 
  else
    ROS_INFO("[aviz] There are %i states in the %s states list.",int(states.size()),name.c_str());
    
  if(color.size()<2)
  {
    ROS_INFO("[aviz] Not enough colors specified.");
    return;
  } 
  
  if(color[0].size() < 4 || color[1].size() < 4)
  {
    ROS_INFO("[aviz] RGBA must be specified for each color.");
    return;
  } 
  
  //if there are too many states, rviz will crash and burn when drawing
  /*
  if(states.size() > 50000)
    inc = 20;
  else if(states.size() > 5000)
    inc = 10;
  else if(states.size() > 500)
    inc = 1;   //changed  8/31/11
  else
    inc = 1;
  */

  unsigned int mind = 0;
  for(unsigned int i = 0; i < states.size(); i=i+inc)
  {
    marker_array.markers.resize(marker_array.markers.size()+1);
    marker_array.markers[mind].header.frame_id = reference_frame_;
    marker_array.markers[mind].header.stamp = ros::Time::now();
    marker_array.markers[mind].ns = "expanded_states";
    marker_array.markers[mind].id = mind;
    marker_array.markers[mind].type = visualization_msgs::Marker::CUBE;
    marker_array.markers[mind].action =  visualization_msgs::Marker::ADD;
    marker_array.markers[mind].scale.x = size;
    marker_array.markers[mind].scale.y = size;
    marker_array.markers[mind].scale.z = size;
    
    for(unsigned int j = 0; j < 4; ++j)
      scaled_color[j] = color[0][j] - ((color[0][j] - color[1][j]) * (double(i)/double(states.size()/inc)));
      
    marker_array.markers[mind].color.r = scaled_color[0];
    marker_array.markers[mind].color.g = scaled_color[1];
    marker_array.markers[mind].color.b = scaled_color[2];
    marker_array.markers[mind].color.a = 1;
    marker_array.markers[mind].lifetime = ros::Duration(500.0);
    
    marker_array.markers[mind].pose.position.x = states[i][0];
    marker_array.markers[mind].pose.position.y = states[i][1];
    marker_array.markers[mind].pose.position.z = states[i][2];
    
    ++mind;
  } 
  
  ROS_DEBUG("[aviz] published %d markers for %s states", (int)marker_array.markers.size(), name.c_str());
  marker_array_publisher_.publish(marker_array);
}

void VisualizeArm::visualizeLine(const std::vector<geometry_msgs::Point> points, std::string ns, int id, int hue, double thickness)
{
  double r=0,g=0,b=0;
  visualization_msgs::Marker marker;

  HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = reference_frame_;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.points = points;
  marker.scale.x = thickness;
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 0.5;
  marker.lifetime = ros::Duration(500.0);

  ROS_INFO("Visualizing a line with %d points", int(points.size()));
  marker_publisher_.publish(marker);
}

void VisualizeArm::visualizeText(geometry_msgs::Pose pose, std::string text, std::string ns, int id, int hue)
{
  double r=0,g=0,b=0;
  visualization_msgs::Marker marker;

  HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = reference_frame_;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.pose = pose;
  
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;
  marker.text = text;
  marker.lifetime = ros::Duration(500.0);

  marker_publisher_.publish(marker);
}

void VisualizeArm::visualizeText(geometry_msgs::Pose pose, std::string text, std::string ns, int id, std::vector<double> color, double size)
{
  visualization_msgs::Marker marker;

  if(color.size() < 4)
    color.resize(4,1);

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = reference_frame_;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = size;
  marker.scale.y = size;
  marker.scale.z = size;
  marker.pose = pose;
  
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = color[3];
  marker.text = text;
  marker.lifetime = ros::Duration(360.0);

  marker_publisher_.publish(marker);
}

void VisualizeArm::visualizeCube(geometry_msgs::PoseStamped pose, int color, std::string ns, int id, std::vector<double> dim)
{
  double r=0,g=0,b=0;
  visualization_msgs::Marker marker;

  if(dim.size() < 3)
  {
    ROS_INFO("[aviz] Three dimensions are needed to visualize a cube.");
    if(dim.size() > 1)
      dim.resize(3,dim[0]);
    else
      return;
  }

  HSVtoRGB(&r, &g, &b, color, 1.0, 1.0);

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = pose.header.frame_id;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = pose.pose;
  marker.scale.x = dim[0];
  marker.scale.y = dim[1];
  marker.scale.z = dim[2];
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration(800.0);

  marker_publisher_.publish(marker);
}

void VisualizeArm::visualizeCube(geometry_msgs::PoseStamped pose, std::vector<double> &color, std::string ns, int id, std::vector<double> dim)
{
  visualization_msgs::Marker marker;

  if(dim.size() < 3)
  {
    ROS_INFO("[aviz] Three dimensions are needed to visualize a cube.");
    if(dim.size() > 1)
      dim.resize(3,dim[0]);
    else
      return;
  }
  if(color.size() < 4)
  {
    ROS_ERROR("[aviz] No color specified.");
    return;
  }

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = pose.header.frame_id;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = pose.pose;
  marker.scale.x = dim[0];
  marker.scale.y = dim[1];
  marker.scale.z = dim[2];
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = color[3];
  marker.lifetime = ros::Duration(1000.0);

  marker_publisher_.publish(marker);
}

}
