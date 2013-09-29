/*
 * Copyright (c) 2011, University of Pennsylvania
 * All rights reserved.
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
/**
 * \author Bharath Sankaran
 *
 * @b comprises all the forward declarations and headers for the project
 */
#ifndef FORWARD_DECLARATIONS_H
#define FORWARD_DECLARATIONS_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <limits>
#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <list>
#include <ctime>
#include <functional>
#include <numeric>
#include <dirent.h>
#include <time.h>
#include <math.h>
#include <sys/stat.h>
#include "ros/ros.h"
#include "rospack/rospack.h"

//PCL includes
#include <pcl/pcl_base.h>
#include <pcl/common/eigen.h>
#include "pcl/common/common.h"
#include "pcl/common/angles.h"
#include <pcl/common/common_headers.h>
#include <pcl/common/pca.h>
#include <pcl/common/centroid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/sample_consensus/sac_model_parallel_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/segmentation/extract_polygonal_prism_data.h"
#include <pcl/surface/marching_cubes.h>
#include <pcl/surface/reconstruction.h>
#include <pcl/surface/marching_cubes_greedy.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/Vertices.h>
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/voxel_grid.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include "pcl/features/normal_3d.h"
#include "pcl/features/fpfh.h"
#include <pcl/features/vfh.h>
#include "pcl/features/pfh.h"
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/registration/transforms.h>
#include "pcl/registration/ia_ransac.h"
#include "pcl/registration/icp.h"
#include <pcl/range_image/range_image.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_nodes.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree_iterator.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>

//Vocabulary Tree includes
#include <vocabulary_tree/simple_kmeans.h>
#include <vocabulary_tree/vocabulary_tree.h>
#include <vocabulary_tree/database.h>
#include <vocabulary_tree/tree_builder.h>
#include <vocabulary_tree/simple_kmeans.h>

//ANN includes
#include <ANN/ANN.h>

// Messages
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <vector>

// Boost includes
#include <boost/foreach.hpp>
#include <boost/thread/thread.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>
#include <boost/random.hpp>

//pcl_ros
#include <pcl_ros/publisher.h>
#include "pcl_ros/transforms.h"
#include <pcl_ros/point_cloud.h>

//namespace using
using namespace Eigen;

// A bit of shorthand
typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZI PointL; // to store labels
typedef pcl::Normal PointNT;
typedef pcl::search::KdTree<PointL> SearchMethodI;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointNT> SurfaceNormals;
typedef pcl::PointIndices PointIndices;
typedef pcl::VFHSignature308 Keypoint;
typedef pcl::PFHSignature125 Keypoint_p;
typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
typedef pcl::PointCloud<pcl::PFHSignature125> LocalFeatures_p;
typedef pcl::PointCloud<pcl::VFHSignature308> GlobalFeatures;
typedef pcl::search::KdTree<PointT> SearchMethod;
typedef pcl::search::KdTree<PointNT> SearchMethodL;
typedef Eigen::Matrix<double, Dynamic, 3> MatrixNx3;

#endif //_FORWARD_DECLARATIONS_H
