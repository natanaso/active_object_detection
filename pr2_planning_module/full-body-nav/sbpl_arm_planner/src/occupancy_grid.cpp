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

 /** \author Benjamin Cohen */

#include <sbpl_arm_planner/occupancy_grid.h>

using namespace std;

namespace sbpl_arm_planner {

OccupancyGrid::OccupancyGrid()
{
  origin_[0] = 0.0;
  origin_[1] = 0.0;
  origin_[2] = 0.0;

  grid_resolution_ = 0.01;

  world_size_[0] = 2.0;
  world_size_[1] = 2.0;
  world_size_[2] = 2.0;

  grid_size_[0] = world_size_[0] / grid_resolution_;
  grid_size_[1] = world_size_[1] / grid_resolution_;
  grid_size_[2] = world_size_[2] / grid_resolution_;

  prop_distance_ = 0.60;

  grid_ = new distance_field::PropagationDistanceField(world_size_[0], world_size_[1], world_size_[2], grid_resolution_, origin_[0], origin_[1], origin_[2], prop_distance_);

  grid_->reset();
  
  reference_frame_="map";
}

OccupancyGrid::OccupancyGrid(double dim_x, double dim_y, double dim_z, double resolution, double origin_x, double origin_y, double origin_z)
{
  origin_[0] = origin_x;
  origin_[1] = origin_y;
  origin_[2] = origin_z;

  grid_resolution_ = resolution;

  world_size_[0] = dim_x;
  world_size_[1] = dim_y;
  world_size_[2] = dim_z;

  grid_size_[0] = world_size_[0] / grid_resolution_;
  grid_size_[1] = world_size_[1] / grid_resolution_;
  grid_size_[2] = world_size_[2] / grid_resolution_;

  prop_distance_ = 0.60;
 
  grid_ = new distance_field::PropagationDistanceField(world_size_[0], world_size_[1], world_size_[2], grid_resolution_, origin_[0], origin_[1], origin_[2], prop_distance_);
  grid_->reset();

  reference_frame_="map";
}

OccupancyGrid::~OccupancyGrid()
{
  delete grid_;
}

void OccupancyGrid::getGridSize(int &dim_x, int &dim_y, int &dim_z)
{
  dim_x = grid_size_[0];
  dim_y = grid_size_[1];
  dim_z = grid_size_[2];
}

void OccupancyGrid::setWorldSize(double dim_x, double dim_y, double dim_z)
{
  world_size_[0] = dim_x;
  world_size_[1] = dim_y;
  world_size_[2] = dim_z;

  SBPL_INFO("[OccupancyGrid] Set internal world dimensions but not distance field\n");
}

void OccupancyGrid::getWorldSize(double &dim_x, double &dim_y, double &dim_z)
{
  dim_x = grid_->getSize(distance_field::PropagationDistanceField::DIM_X);
  dim_y = grid_->getSize(distance_field::PropagationDistanceField::DIM_Y);
  dim_z = grid_->getSize(distance_field::PropagationDistanceField::DIM_Z);
}

void OccupancyGrid::setOrigin(double wx, double wy, double wz)
{
  origin_[0] = wx;
  origin_[1] = wy;
  origin_[2] = wz;
}

void OccupancyGrid::getOrigin(double &wx, double &wy, double &wz)
{
  wx = grid_->getOrigin(distance_field::PropagationDistanceField::DIM_X);
  wy = grid_->getOrigin(distance_field::PropagationDistanceField::DIM_Y);
  wz = grid_->getOrigin(distance_field::PropagationDistanceField::DIM_Z);
}

void OccupancyGrid::setResolution(double resolution_m)
{
  grid_resolution_ = resolution_m;
}

double OccupancyGrid::getResolution()
{
  return grid_->getResolution(distance_field::PropagationDistanceField::DIM_X);
}

void OccupancyGrid::updateFromCollisionMap(const arm_navigation_msgs::CollisionMap &collision_map)
{
  if(collision_map.boxes.empty())
  {
    SBPL_DEBUG("[grid] collision map received is empty.");
    return;
  }

  if(reference_frame_.compare(collision_map.header.frame_id) != 0)
  { 
    ROS_DEBUG_ONCE("[grid] Collision map is in frame %s but is expected to be in %s. Using anyway.",collision_map.header.frame_id.c_str(),reference_frame_.c_str());
    reference_frame_ = collision_map.header.frame_id;
  }
  
  SBPL_DEBUG("[grid] Resetting grid and updating from collision map");
  grid_->reset();
  //grid_->addPointsToField(cuboid_points_);
  grid_->addCollisionMapToField(collision_map);
}

void OccupancyGrid::addCollisionCuboid(double origin_x, double origin_y, double origin_z, double size_x, double size_y, double size_z)
{
  int num_points=0;
  //cuboid_points_.clear();
  
  for (double x=origin_x-size_x/2.0; x<=origin_x+size_x/2.0; x+=grid_resolution_)
  {
    for (double y=origin_y-size_y/2.0; y<=origin_y+size_y/2.0; y+=grid_resolution_)
    {
      for (double z=origin_z-size_z/2.0; z<=origin_z+size_z/2.0; z+=grid_resolution_)
      {
        cuboid_points_.push_back(tf::Vector3(x,y,z));
        ++num_points;
      }
    }
  }
 
  grid_->addPointsToField(cuboid_points_);

  ROS_DEBUG("[addCollisionCuboid] Added %d points for collision cuboid (origin: %0.2f %0.2f %0.2f  size: %0.2f %0.2f %0.2f).", num_points, origin_x, origin_y, origin_z, size_x, size_y, size_z);
}

void OccupancyGrid::getVoxelsInBox(const geometry_msgs::Pose &pose, const std::vector<double> &dim, std::vector<tf::Vector3> &voxels)
{
  tf::Vector3 vin, vout, v(pose.position.x, pose.position.y, pose.position.z);
  tf::Matrix3x3 m(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));

  for (double x=0-dim[0]/2.0; x<=dim[0]/2.0; x+=grid_resolution_)
  {
    for (double y=0-dim[1]/2.0; y<=dim[1]/2.0; y+=grid_resolution_)
    {
      for (double z=0-dim[2]/2.0; z<=dim[2]/2.0; z+=grid_resolution_)
      {
        vin.setX(x);
        vin.setY(y);
        vin.setZ(z);
        vout = m*vin;
        vout += v;

        voxels.push_back(vout);
      }
    }
  }
}

void OccupancyGrid::visualize()
{
  tf::Transform trans; 
  trans.setIdentity();
  
  // TODO: visualize has been removed from distance_field::PropagationDistanceField
  //grid_->visualize(0.01, 0.02, reference_frame_, trans, ros::Time::now());
}

const distance_field::PropagationDistanceField* OccupancyGrid::getDistanceFieldPtr()
{
  return grid_;
}

void OccupancyGrid::printGridFromBinaryFile(std::string filename)
{
  ifstream fin;
  int dimx,dimy,dimz;
  unsigned char temp = 0;
  struct stat file_stats;

  const char *name  = filename.c_str();
  fin.open(name, ios_base::in | ios_base::binary);

  if (fin.is_open())
  {
    fin.read( (char*) &dimx, sizeof(int));
    fin.read( (char*) &dimy, sizeof(int));
    fin.read( (char*) &dimz, sizeof(int));

    for(int x = 0; x < dimx; x++)
    {
      for(int y = 0; y < dimy; y++)
      {
        for(int z = 0; z < dimz; z++)
        {
          fin.read( (char*) &temp, sizeof(unsigned char));
        }
      }
    }
  }
  else
  {
    SBPL_ERROR("[printGridFromBinaryFile] Failed to open file for reading.");
    return;
  }

  if(stat(name, &file_stats) == 0)
    SBPL_DEBUG("[printGridFromBinaryFile] The size of the file read is %d kb.", int(file_stats.st_size)/1024);
  else
    SBPL_DEBUG("[printGridFromBinaryFile] An error occurred when retrieving size of file read.");

  fin.close();
}

bool OccupancyGrid::saveGridToBinaryFile(std::string filename)
{
  ofstream fout;
  int dimx,dimy,dimz;
  unsigned char val = 0;
  struct stat file_stats;

  getGridSize(dimx, dimy, dimz);

  const char *name  = filename.c_str();
  fout.open(name, ios_base::out | ios_base::binary | ios_base::trunc);

  if (fout.is_open())
  {
    fout.write( (char *) &dimx, sizeof(int));
    fout.write( (char *) &dimy, sizeof(int));
    fout.write( (char *) &dimz, sizeof(int));

    for(int x = 0; x < dimx; x++)
    {
      for(int y = 0; y < dimy; y++)
      {
        for(int z = 0; z < dimz; z++)
        {
          val = getCell(x,y,z);
          fout.write( (char *) &val, sizeof(unsigned char));
        }
      }
    }
  }
  else
  {
    SBPL_ERROR("[saveGridToBinaryFile] Failed to open file for writing.\n");
    return false;
  }

  fout.close();

  //verify writing to file was successful
  if(stat(name, &file_stats) == 0)
  {
    SBPL_INFO("[saveGridToBinaryFile] The size of the file created is %d kb.\n", int(file_stats.st_size)/1024);
    if(file_stats.st_size == 0)
    {
      SBPL_ERROR("[saveGridToBinaryFile] File created is empty. Exiting.\n");
      return false;
    }
  }
  else
  {
    SBPL_ERROR("[saveGridToBinaryFile] An error occurred when retrieving size of file created. Exiting.\n");
    return false;
  }

  return true;
}

}
