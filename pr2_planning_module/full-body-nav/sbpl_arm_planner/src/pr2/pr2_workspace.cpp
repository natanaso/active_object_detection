/* \author Gokul Subramanian */

#include <sbpl_arm_planner/pr2/pr2_workspace.h>

double lower_corner_x = -0.6;
double lower_corner_y = -1.25;
double lower_corner_z = -0.05;
double grid_size_x = 1.6;
double grid_size_y = 1.6;
double grid_size_z = 1.4;

double resolution_x = 0.02;
double resolution_y = 0.02;
double resolution_z = 0.02;
double pan_res = 0.06981;

double shoulder_pan_x = -0.050;
double shoulder_pan_y = -0.188;
double shoulder_pan_z =  0.743;

double L_up_arm = 0.410;
double L_fo_arm = 0.321;
double L_sh = 0.100;

double jnt_min[] = {-2.285, -0.523, -3.899, -2.299};
double jnt_max[] = { 0.714,  1.396,  0.850,  0.000};

double fo_arm_up_arm_angle_offset = 0.0;
#define OFFSET fo_arm_up_arm_angle_offset
#define PI 3.14159

//Converting grid coordinates to (x,y,z) coordinates
void grid2world(long i, long j, long k, double& x, double& y, double& z) {
  x = lower_corner_x + (double(i)+0.5)*resolution_x;
  y = lower_corner_y + (double(j)+0.5)*resolution_y;
  z = lower_corner_z + (double(k)+0.5)*resolution_z;
}

//Converting (x,y,z) coordinates to grid coordinates
void world2grid(double x, double y, double z, long& i, long& j, long& k) {
  double temp;
  double a, b, c;
  temp = modf(x/resolution_x, &a);
  temp = modf(y/resolution_y, &b);
  temp = modf(z/resolution_z, &c);
  i = long(a);
  j = long(b);
  k = long(c);
}

//Function which returns the 2 possible elbow positions for a given shoulder pan
//such that the end effector can be at (x2, y2, z2) and the shoulder lift axis 
//at (x1, y1, z1) with 'vect' indicating a horizontal vector from the shoulder pan
//axis to the shoulder lift axis.
vector<vector<double> > elbow_positions_given_pan(double x1, double y1, double z1, double r1, double x2, double y2, double z2, double r2, double vect[3]) {
	
	//Shoulder lift (SL)
	//End effector (EE)	
	
  vector<vector<double> > list_of_points;
  double sphere1_cent[] = {x1, y1, z1}; //referred to as cent1
  double sphere2_cent[] = {x2, y2, z2}; //referred to as cent2
  double cent12cent2_dist = distance_between(sphere1_cent, sphere2_cent, 3); 
	
	//If the SL axis and EE position are too far away,
 	//then there can be no possible locations for the EE
	//even geometrically  
  if ( cent12cent2_dist >= (r1+r2) ) {
    return list_of_points; //an empty list
  }
	
	//The vector from SL axis to EE
  double cent12cent2_vect[] = {x2-x1, y2-y1, z2-z1};
	
	//Some crazy mathematics given 2 pieces of information
	//1) The dot_product of the upperarm vector with the vector from SL to EE
	//2) The projection of the desired upperarm vector be along 'vect'.
  double dot_prod = (pow(r1,2) + pow(cent12cent2_dist, 2) - pow(r2, 2))/2;
  double A = r1*vect[0]*cent12cent2_vect[0] + r1*vect[1]*cent12cent2_vect[1];
  double B = r1*cent12cent2_vect[2];

  double alpha1 = asin((2*B*dot_prod + sqrt(pow(2*B*dot_prod, 2) - 4*(pow(A,2) + pow(B,2))*(pow(dot_prod,2) - pow(A,2))))/(2*(pow(A,2) + pow(B,2))));
  double alpha2 = asin((2*B*dot_prod - sqrt(pow(2*B*dot_prod, 2) - 4*(pow(A,2) + pow(B,2))*(pow(dot_prod,2) - pow(A,2))))/(2*(pow(A,2) + pow(B,2))));
	
	//Checking if alpha1 or alpha2 are NaN
  if (alpha1 != alpha1 || alpha2 != alpha2) return list_of_points;

  vector<double> solution1;
  vector<double> solution2;

  solution1.push_back(x1 + r1*cos(alpha1)*vect[0]);
  solution1.push_back(y1 + r1*cos(alpha1)*vect[1]);
  solution1.push_back(z1 + r1*sin(alpha1));

  solution2.push_back(x1 + r1*cos(alpha2)*vect[0]);
  solution2.push_back(y1 + r1*cos(alpha2)*vect[1]);
  solution2.push_back(z1 + r1*sin(alpha2));
  
	//Since a dot product of two vectors is invariant with simultaneous direction
	//inversion of both vectors, it is necessary to check the resultant solution
	//to see if it satisfies the distance constraint.
  double tolerance = 1/pow(10,2);	//A tolerance is necessary for double precision equality checking
  if ((fabs(distance_between(solution1, sphere2_cent, 3) - r2)) > tolerance || fabs(distance_between(solution2, sphere2_cent, 3) - r2) > tolerance)
    return list_of_points;

  list_of_points.push_back(solution1);
  list_of_points.push_back(solution2);
  return list_of_points;
}

//Function checks if the arm with the given poses for the shoulder pan axis,
//shoulder lift axis, elbow position and end effector position is within
//joint limits. It is assumed that shoulder pan is already checked for its
//limits. There are 2 versions of this function, one that appends all
//acceptable joint angles into the elbow_pose vector, and another version
//that doesn't do that
bool check_joint_limits_and_append_joint_angles(double pan, double shlift_wrt_shpan_uvect[3], double shlift[3], vector<double>& elbow_pose, double endeff_pose[3]) {
  
  //Pushing the shoulder pan joint angle into the elbow_pos vector
  elbow_pose.push_back(pan);

  //Checking for the upperarm lift joint limits
  double up_arm_uvect[] = {elbow_pose[0] - shlift[0], elbow_pose[1] - shlift[1], elbow_pose[2] - shlift[2]};
  double nm = vect_norm(up_arm_uvect, 3);
  up_arm_uvect[0] /= nm;
  up_arm_uvect[1] /= nm;
  up_arm_uvect[2] /= nm;
  
  double dot_prod = dot_product(shlift_wrt_shpan_uvect, up_arm_uvect, 3);
  if ((elbow_pose[2] > shlift[2] && dot_prod < cos(jnt_min[1])) || (elbow_pose[2] <= shlift[2] && dot_prod < cos(jnt_max[1]))) {
    return 0;
  }

  //Pushing the upper arm lift joint angle into the elbow_pos vector
  double lift = (elbow_pose[2] >= shlift[2] ) ? -acos(dot_prod) : acos(dot_prod);
  elbow_pose.push_back(lift);

  //Checking for the elbow flex joint limits
  double fo_arm_uvect[] = {endeff_pose[0] - elbow_pose[0], endeff_pose[1] - elbow_pose[1], endeff_pose[2] - elbow_pose[2]};
  nm = vect_norm(fo_arm_uvect, 3);
  fo_arm_uvect[0] /= nm;
  fo_arm_uvect[1] /= nm;
  fo_arm_uvect[2] /= nm;
  
  dot_prod = dot_product(up_arm_uvect, fo_arm_uvect, 3);
  if((dot_prod > 0 && dot_prod > cos(OFFSET)) ||  (dot_prod < 0 && dot_prod < cos(-jnt_min[3]+OFFSET))) {
    return 0;
  }

  //Pushing the fore arm flex joint angle into the elbow_pos vector
  double flex = acos(dot_prod);
  elbow_pose.push_back(flex);

  //Checking for upperarm roll joint limits
  double up_arm_proj[] = {up_arm_uvect[0], up_arm_uvect[1], 0};
  double up_arm_proj_norm = vect_norm(up_arm_proj, 3);

  up_arm_proj[0] /= up_arm_proj_norm;
  up_arm_proj[1] /= up_arm_proj_norm;

  double up_arm_proj_perp[] = {-up_arm_proj[1], up_arm_proj[0], 0};
  double ref_uvect[3];
  cross_product(ref_uvect, up_arm_uvect, up_arm_proj_perp);
  double fo_arm_proj_uvect[] = {fo_arm_uvect[0] - dot_prod*up_arm_uvect[0], fo_arm_uvect[1] - dot_prod*up_arm_uvect[1], fo_arm_uvect[2] - dot_prod*up_arm_uvect[2]};
  double fo_arm_proj_norm = vect_norm(fo_arm_proj_uvect, 3);

  fo_arm_proj_uvect[0] /= fo_arm_proj_norm;
  fo_arm_proj_uvect[1] /= fo_arm_proj_norm;
  fo_arm_proj_uvect[2] /= fo_arm_proj_norm;

  double ref_min_uvect[3], ref_max_uvect[3];
  rotate_vector(ref_min_uvect, ref_uvect, up_arm_uvect, jnt_min[2]);
  rotate_vector(ref_max_uvect, ref_uvect, up_arm_uvect, jnt_max[2]);
  double tolerance = 1/pow(10, 2);
  double direction[3] ;
  cross_product(direction, ref_min_uvect, ref_max_uvect);
  bool roll_acceptable = 0;
  if (fabs(acos(dot_product(ref_min_uvect, ref_max_uvect, 3)) - acos(dot_product(ref_min_uvect, fo_arm_proj_uvect, 3)) - acos(dot_product(ref_max_uvect, fo_arm_proj_uvect, 3))) < tolerance) {
    if (vect_divide(direction, up_arm_uvect, 3) > 0){
      roll_acceptable = 1;
    }
  }
  else {
    if (vect_divide(direction, up_arm_uvect, 3) < 0) {
      roll_acceptable = 1;
    }
  }
  //If roll is acceptable, then push the forearm roll joint angle into the
  //elbow_pos vector
  if (!roll_acceptable) {
    return 0;
  }
  else {//Roll angle is very specific to the PR2 robot joint limits
    dot_prod = dot_product(ref_uvect, fo_arm_proj_uvect, 3);
    double roll;
    cross_product(direction, ref_uvect, fo_arm_proj_uvect);
    if (vect_divide(direction, up_arm_uvect, 3) >= 0 && dot_prod > 0) {
      roll = acos(dot_prod);
    }
    else {
      if (vect_divide(direction, up_arm_uvect, 3) < 0) {
        roll = -acos(dot_prod);
      }
      else {
        roll = -(2*PI - acos(dot_prod));
      }
    }
    elbow_pose.push_back(roll);
    return 1;
  }
}

bool check_joint_limits(double shlift_wrt_shpan_uvect[3], double shlift[3], vector<double> elbow_pose, double endeff_pose[3]) {
  //Checking for the upperarm lift joint limits
  double up_arm_uvect[] = {elbow_pose[0] - shlift[0], elbow_pose[1] - shlift[1], elbow_pose[2] - shlift[2]};

  up_arm_uvect[0] /= L_up_arm;
  up_arm_uvect[1] /= L_up_arm;
  up_arm_uvect[2] /= L_up_arm;
  
  double dp = dot_product(shlift_wrt_shpan_uvect, up_arm_uvect, 3);
  if ((elbow_pose[2] > shlift[2] && dp < cos(jnt_min[1])) || (elbow_pose[2] <= shlift[2] && dp < cos(jnt_max[1]))) {
    return 0;
  }

  //Checking for the elbow flex joint limits
  double fo_arm_uvect[] = {endeff_pose[0] - elbow_pose[0], endeff_pose[1] - elbow_pose[1], endeff_pose[2] - elbow_pose[2]};

  fo_arm_uvect[0] /= L_fo_arm;
  fo_arm_uvect[1] /= L_fo_arm;
  fo_arm_uvect[2] /= L_fo_arm;
  
  double dot_prod = dot_product(up_arm_uvect, fo_arm_uvect, 3);
  if((dot_prod > 0 && dot_prod > cos(OFFSET)) ||  (dot_prod < 0 && dot_prod < cos(-jnt_min[3]+OFFSET))) {
    return 0;
  }

  //Checking for upperarm roll joint limits
  double up_arm_proj[] = {up_arm_uvect[0], up_arm_uvect[1], 0};
  double up_arm_proj_norm = vect_norm(up_arm_proj, 3);

  up_arm_proj[0] /= up_arm_proj_norm;
  up_arm_proj[1] /= up_arm_proj_norm;

  double up_arm_proj_perp[] = {-up_arm_proj[1], up_arm_proj[0], 0};
  double ref_uvect[3];
  cross_product(ref_uvect, up_arm_uvect, up_arm_proj_perp);

  double fo_arm_proj_uvect[] = {fo_arm_uvect[0] - L_fo_arm*dot_prod*up_arm_uvect[0], fo_arm_uvect[1] - L_fo_arm*dot_prod*up_arm_uvect[1], fo_arm_uvect[2] - L_fo_arm*dot_prod*up_arm_uvect[2]};
  double fo_arm_proj_norm = vect_norm(fo_arm_proj_uvect, 3);

  fo_arm_proj_uvect[0] /= fo_arm_proj_norm;
  fo_arm_proj_uvect[1] /= fo_arm_proj_norm;
  fo_arm_proj_uvect[2] /= fo_arm_proj_norm;

  double ref_min_uvect[3], ref_max_uvect[3];
  rotate_vector(ref_min_uvect, ref_uvect, up_arm_uvect, jnt_min[2]);
  rotate_vector(ref_max_uvect, ref_uvect, up_arm_uvect, jnt_max[2]);

  double tolerance = 1/pow(10, 2);
  double direction[3];
  cross_product(direction, ref_min_uvect, ref_max_uvect);
  if (fabs(acos(dot_product(ref_min_uvect, ref_max_uvect, 3)) - acos(dot_product(ref_min_uvect, fo_arm_proj_uvect, 3)) - acos(dot_product(ref_max_uvect, fo_arm_proj_uvect, 3))) < tolerance) {
    if (vect_divide(direction, up_arm_uvect, 3) < 0){
      return 0;
    }
    else 
      return 1;
  }
  else {
    if (vect_divide(direction, up_arm_uvect, 3) > 0) {
      return 0;
    }
    else 
      return 1;
  }
}

//Function computes possible elbow locations for an end effector pose.
//Additionally has all the joint angles corresponding to each pose.
//There are 2 versions of this function. One for output of only accepted
//points and other for that of rejected points as well
vector<vector<double> > elbow_positions_given_endeff_pose(double pan_start, double resolution, double x, double y, double z) { 
  vector<vector<double> > list_of_accepted_points;

  double endeff_pose[] = {x, y, z};
  vector<vector<double> > elbow_positions_possible;

  double shlift_wrt_shpan_uvect[3]; shlift_wrt_shpan_uvect[2] = 0;
  double shlift_cent[3];

  //added 9/4/2010 -- Since the pan_start is in between the pan joint limit
  //angles, two for loops are used for going in opposite directions from
  //pan_start, one towards jnt_min[0] and the other towards jnt_max[0]
  pan_res = resolution;

  for (double pan = pan_start; pan <= jnt_max[0]; pan += pan_res) {
    shlift_wrt_shpan_uvect[0] = cos(pan);
    shlift_wrt_shpan_uvect[1] = sin(pan);

    shlift_cent[0] = shoulder_pan_x + L_sh*shlift_wrt_shpan_uvect[0];  
    shlift_cent[1] = shoulder_pan_y + L_sh*shlift_wrt_shpan_uvect[1];
    shlift_cent[2] = shoulder_pan_z;  
    elbow_positions_possible = elbow_positions_given_pan(shlift_cent[0], shlift_cent[1], shlift_cent[2], L_up_arm, x, y, z, L_fo_arm, shlift_wrt_shpan_uvect);


    if (!elbow_positions_possible.empty()) {
      for (unsigned int i = 0; i < elbow_positions_possible.size(); i++) {
        if (check_joint_limits_and_append_joint_angles(pan, shlift_wrt_shpan_uvect, shlift_cent, elbow_positions_possible[i], endeff_pose)) {
          list_of_accepted_points.push_back(elbow_positions_possible[i]);
        }
      }
    }
  }
  for (double pan = pan_start-pan_res; pan >= jnt_min[0]; pan -= pan_res) {
    shlift_wrt_shpan_uvect[0] = cos(pan);
    shlift_wrt_shpan_uvect[1] = sin(pan);

    shlift_cent[0] = shoulder_pan_x + L_sh*shlift_wrt_shpan_uvect[0];  
    shlift_cent[1] = shoulder_pan_y + L_sh*shlift_wrt_shpan_uvect[1];
    shlift_cent[2] = shoulder_pan_z;  
    elbow_positions_possible = elbow_positions_given_pan(shlift_cent[0], shlift_cent[1], shlift_cent[2], L_up_arm, x, y, z, L_fo_arm, shlift_wrt_shpan_uvect);


    if (!elbow_positions_possible.empty()) {
      for (unsigned int i = 0; i < elbow_positions_possible.size(); i++) {
        if (check_joint_limits_and_append_joint_angles(pan, shlift_wrt_shpan_uvect, shlift_cent, elbow_positions_possible[i], endeff_pose)) {
          list_of_accepted_points.push_back(elbow_positions_possible[i]);
        }
      }
    }
  }
  return list_of_accepted_points;
}

void elbow_positions_given_endeff_pose(vector<vector<double> >& list_of_accepted_points, vector<vector<double> >& list_of_rejected_points, double pan_start, double resolution, double x, double y, double z) {
  double endeff_pose[] = {x, y, z};
  vector<vector<double> > elbow_positions_possible;

  double shlift_wrt_shpan_uvect[3]; shlift_wrt_shpan_uvect[2] = 0;
  double shlift_cent[3];
  
  //added 9/4/2010 -- Since the pan_start is in between the pan joint limit
  //angles, two for loops are used for going in opposite directions from
  //pan_start, one towards jnt_min[0] and the other towards jnt_max[0]
  pan_res = resolution;
  for (double pan = pan_start; pan <= jnt_max[0]; pan += pan_res) {
    shlift_wrt_shpan_uvect[0] = cos(pan);
    shlift_wrt_shpan_uvect[1] = sin(pan);

    shlift_cent[0] = shoulder_pan_x + L_sh*shlift_wrt_shpan_uvect[0];  
    shlift_cent[1] = shoulder_pan_y + L_sh*shlift_wrt_shpan_uvect[1];
    shlift_cent[2] = shoulder_pan_z;  
    elbow_positions_possible = elbow_positions_given_pan(shlift_cent[0], shlift_cent[1], shlift_cent[2], L_up_arm, x, y, z, L_fo_arm, shlift_wrt_shpan_uvect);


    if (!elbow_positions_possible.empty()) {
      for (unsigned int i = 0; i < elbow_positions_possible.size(); i++) {
        if (check_joint_limits_and_append_joint_angles(pan, shlift_wrt_shpan_uvect, shlift_cent, elbow_positions_possible[i], endeff_pose)) {
          list_of_accepted_points.push_back(elbow_positions_possible[i]);
        }
        else {
          list_of_rejected_points.push_back(elbow_positions_possible[i]);
        }
      }
    }
  }
  for (double pan = pan_start-pan_res; pan >= jnt_min[0]; pan -= pan_res) {
    shlift_wrt_shpan_uvect[0] = cos(pan);
    shlift_wrt_shpan_uvect[1] = sin(pan);

    shlift_cent[0] = shoulder_pan_x + L_sh*shlift_wrt_shpan_uvect[0];  
    shlift_cent[1] = shoulder_pan_y + L_sh*shlift_wrt_shpan_uvect[1];
    shlift_cent[2] = shoulder_pan_z;  
    elbow_positions_possible = elbow_positions_given_pan(shlift_cent[0], shlift_cent[1], shlift_cent[2], L_up_arm, x, y, z, L_fo_arm, shlift_wrt_shpan_uvect);


    if (!elbow_positions_possible.empty()) {
      for (unsigned int i = 0; i < elbow_positions_possible.size(); i++) {
        if (check_joint_limits_and_append_joint_angles(pan, shlift_wrt_shpan_uvect, shlift_cent, elbow_positions_possible[i], endeff_pose)) {
          list_of_accepted_points.push_back(elbow_positions_possible[i]);
        }
        else {
          list_of_rejected_points.push_back(elbow_positions_possible[i]);
        }
      }
    }
  }
}

//Function takes a list_of_rejected_points output by the
//elbow_positions_given_endeff_pose function and separates the points based
//on why they were rejected i.e. due to either lift, flex or roll
void separate_rejected_points(vector<vector<double> > list_of_rejected_points, vector<vector<double> >& rejected_due_to_lift, vector<vector<double> >& rejected_due_to_flex, vector<vector<double> >& rejected_due_to_roll) {
  int len = list_of_rejected_points.size();
  vector<double> point;
  int size_of_point;
  for (int i = 0; i < len; i++) { 
    point.clear();
    point = list_of_rejected_points[i];
    size_of_point = point.size();
    switch (size_of_point) {
      case 4: {
                rejected_due_to_lift.push_back(point);
                break;
              }
      case 5: {
                rejected_due_to_flex.push_back(point);
                break;
              }
      case 6: {
                rejected_due_to_roll.push_back(point);
                break;
              }
      default: {
                 break;
               }
    }
  }
}

//Function checks if the cell (i,j,k) in the grid is reachable by the robot
//arm given joint limits
bool position(long i, long j, long k) {
  double x, y, z;
  grid2world(i, j, k, x, y, z);
  double endeff_pose[] = {x, y, z};

  vector<vector<double> > elbow_positions_possible;
	
  double shlift_wrt_shpan_uvect[3];	  shlift_wrt_shpan_uvect[2] = 0;
  double shlift_cent[3];

  bool possible_at_least_once = 0;

  for (double pan = jnt_min[0]; pan <=jnt_max[0]; pan += pan_res) {
    shlift_wrt_shpan_uvect[0] = cos(pan);
    shlift_wrt_shpan_uvect[1] = sin(pan);

    shlift_cent[0] = shoulder_pan_x + L_sh*shlift_wrt_shpan_uvect[0];
    shlift_cent[1] = shoulder_pan_y + L_sh*shlift_wrt_shpan_uvect[1];
    shlift_cent[2] = shoulder_pan_z;

    elbow_positions_possible = elbow_positions_given_pan(shlift_cent[0], shlift_cent[1], shlift_cent[2], L_up_arm, x, y, z, L_fo_arm, shlift_wrt_shpan_uvect);

    if (!elbow_positions_possible.empty()) {
      for (unsigned int i = 0; i < elbow_positions_possible.size(); i++) {
        if (check_joint_limits(shlift_wrt_shpan_uvect, shlift_cent, elbow_positions_possible[i], endeff_pose)) {
          possible_at_least_once = 1;
          break;
        }
      }
    }
    if (possible_at_least_once) break;
  }
  return possible_at_least_once;
}
