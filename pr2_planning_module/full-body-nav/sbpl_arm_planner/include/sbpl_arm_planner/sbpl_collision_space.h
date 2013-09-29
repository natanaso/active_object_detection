/*
 * Copyright (c) 2010, Maxim Likhachev
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Pennsylvania nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <vector>
#include <resource_retriever/retriever.h>
#include <sbpl_arm_planner/bresenham.h>
#include <sbpl_arm_planner/sbpl_arm_model.h>
#include <sbpl_arm_planner/occupancy_grid.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/bodies.h>
#include <geometric_shapes/shape_operations.h>
#include <tf_conversions/tf_kdl.h>

#include <tf/tf.h>
#include <arm_navigation_msgs/CollisionObject.h>

using namespace std;

#ifndef _SBPL_COLLISION_SPACE_
#define _SBPL_COLLISION_SPACE_

/** @brief coords - used to pass around lists of valid cells */
typedef struct
{
  short unsigned int x;
  short unsigned int y;
  short unsigned int z;
  bool bIsObstacle;
} CELL3V;

namespace sbpl_arm_planner
{

class SBPLCollisionSpace
{
  public:
    /** @brief default constructor 
     * @param a pointer to the arm object used for planning
     * @param a pointer to an occupancy grid used for planning
    */
    SBPLCollisionSpace(SBPLArmModel* arm, OccupancyGrid* grid);

    /** \brief destructor */
    ~SBPLCollisionSpace(){};

    /** \brief choose the file to output debug information */
    void setDebugFile(FILE* file_ptr);

    /** \brief check joint configuration for collision (0: collision) */
    bool checkCollision(const std::vector<double> &angles, bool verbose, bool check_mesh, unsigned char &dist);

    /** \brief check if a specific link is in collision (0: collision) */
    bool checkLinkForCollision(const std::vector<double> &angles, int link_num, bool verbose, unsigned char &dist);

    /** \brief check if the cell's distance to nearest obstacle > radius */
    inline bool isValidCell(const int x, const int y, const int z, const int radius);

    void addArmCuboidsToGrid();

    bool getCollisionCylinders(const std::vector<double> &angles, std::vector<std::vector<double> > &cylinders);

    /** \brief get coordinates of cells that a line segment intersects */
    void getLineSegment(const std::vector<int> a,const std::vector<int> b,std::vector<std::vector<int> > &points);

    void getInterpolatedPath(const std::vector<double> &start, const std::vector<double> &end, double inc, std::vector<std::vector<double> > &path);

    void getInterpolatedPath(const std::vector<double> &start, const std::vector<double> &end, std::vector<double> &inc, std::vector<std::vector<double> > &path);

    /** \brief check linearly interpolated path for collisions */
    bool checkPathForCollision(const std::vector<double> &start, const std::vector<double> &end, bool verbose, unsigned char &dist);

    /** \brief check linearly interpolated path for collision of a specific link */
    bool checkLinkPathForCollision(const std::vector<double> &start, const std::vector<double> &end, int link_num, bool verbose, unsigned char &dist);

    /** \brief check if a line segment lies on an invalid cell (0: collision) */
    unsigned char isValidLineSegment(const std::vector<int> a,const std::vector<int> b,const short unsigned int radius);

    /* \brief remove all attached objects from gripper */
    void removeAttachedObject();

    /* \brief attach a sphere to the gripper */
    void attachSphereToGripper(std::string frame, geometry_msgs::Pose pose, double radius);

    /* \brief attach a cylinder to the gripper */
    void attachCylinderToGripper(std::string frame, geometry_msgs::Pose pose, double radius, double length);

    /* \brief attach a mesh to the gripper (attach a bounding cylinder of that mesh)*/
    void attachMeshToGripper(const std::string frame, const geometry_msgs::Pose pose, const bodies::BoundingCylinder &cyl);
    
    /* \brief attach a mesh to the gripper (attach a bounding cylinder of that mesh)*/
    void attachMeshToGripper(const std::string frame, const geometry_msgs::Pose pose, const std::vector<int32_t> &triangles, const std::vector<geometry_msgs::Point> &vertices);

    /** \brief get the voxels that the attached object occupies */
    bool getAttachedObject(const std::vector<double> &angles, std::vector<std::vector<double> > &xyz);

    /** \brief get the radius of the attached object (sphere or cylinder) */
    double getAttachedObjectRadius();
   
    /** \brief add a collision object to the environment */ 
    void addCollisionObject(const arm_navigation_msgs::CollisionObject &object);

    /** \brief transform a pose from one frame to another */
    void transformPose(std::string current_frame, std::string desired_frame, geometry_msgs::Pose &pose_in, geometry_msgs::Pose &pose_out);

    void transformPose(const std::string &current_frame, const std::string &desired_frame, const geometry_msgs::Pose &pose_in, geometry_msgs::Pose &pose_out);

    /** \brief process a collision object message */
    void processCollisionObjectMsg(const arm_navigation_msgs::CollisionObject &object);

    void removeCollisionObject(const arm_navigation_msgs::CollisionObject &object);

    void removeAllCollisionObjects();

    void putCollisionObjectsInGrid();

    void getCollisionObjectVoxelPoses(std::vector<geometry_msgs::Pose> &points);

    /** @brief get the xyz coords of each joint in the arm */ 
    bool getJointPosesInGrid(std::vector<double> angles, std::vector<std::vector<int> > &jnts, KDL::Frame &f_out, bool verbose);

    void printObjectMaps();

  private:


    /** @brief arm model used by planner */
    SBPLArmModel* arm_;

    /** @brief occupancy grid used by planner */
    OccupancyGrid* grid_;

    /** @brief the file for dumping debug output */
    FILE* fOut_;

    /** \brief map from object id to object details */
    std::map<std::string, arm_navigation_msgs::CollisionObject> object_map_;
    
    /** \brief map from object id to list of occupying voxels */
    std::map<std::string, std::vector<tf::Vector3> > object_voxel_map_;

    std::vector<std::string> known_objects_;

    tf::TransformListener tf_;

    std::vector<double> inc_;

    bool object_attached_;
    int attached_object_frame_num_;
    short unsigned int attached_object_radius_;
    std::vector<KDL::Frame> object_points_;


     /** @brief get the shortest distance between two 3D line segments */
    double distanceBetween3DLineSegments(std::vector<int> l1a, std::vector<int> l1b,std::vector<int> l2a,std::vector<int> l2b);

    bool getBoundingCylinderOfMesh(std::string mesh_file, shapes::Shape *mesh, bodies::BoundingCylinder &cyl);
    void getBoundingCylinderOfMesh(const std::vector<int32_t> &triangles, const std::vector<geometry_msgs::Point> &vertices, bodies::BoundingCylinder &cyl);

    bool isValidPoint(double &x, double &y, double &z, short unsigned int &radius, unsigned char &dist);
    bool isValidAttachedObject(const std::vector<double> &angles, unsigned char &dist);
    bool isValidAttachedObject(const std::vector<double> &angles, unsigned char &dist, std::vector<int> j1, std::vector<int> j2);

};

inline bool SBPLCollisionSpace::isValidCell(const int x, const int y, const int z, const int radius)
{
  if(grid_->getCell(x,y,z) <= radius)
    return false;
  return true;
}

} 
#endif

