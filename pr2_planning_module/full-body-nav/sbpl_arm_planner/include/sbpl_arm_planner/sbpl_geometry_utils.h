/** /author Andrew Dornbush */

#ifndef _SBPL_GEOMETRY_UTILS_
#define _SBPL_GEOMETRY_UTILS_

#include <vector>
#include <math.h>
#include <string>
#include <geometry_msgs/Point.h>
#include <geometric_shapes/shape_operations.h>
#include <tf/LinearMath/Vector3.h>

namespace sbpl_geometry_utils
{

struct Point
{
	Point() { x = 0.0; y = 0.0; z = 0.0; }
	Point(double x, double y, double z) { this->x = x; this->y = y; this->z = z; }

  double x;
  double y;
  double z;
};

struct Triangle
{
	Point p1;
	Point p2;
	Point p3;
};

struct Sphere
{
  Point p;
  double radius;
};

void getEnclosingSpheresOfCube(double xSize, double ySize, double zSize, double radius, std::vector<Sphere>& spheres);

void getEnclosingSpheresOfCube(double xSize, double ySize, double zSize, double radius, std::vector<std::vector<double> >& spheres);

////////////////////////////////////////////////////////////////////////////////
/// computeEnclosingCollisionSpheres2
///
/// Generates a set of spheres that completely enclose a triangle mesh
///
/// @param vertices The list of vertices
/// @param triangles The list of indices into the vertex array (Triangle k is
///                  represented by by vertices[3*k + 0], vertices[3*k + 1] and
///                  vertices[3*k + 2]
/// @param radius The desired radius of the generated spheres
/// @param spheres The returned spheres
/// @param fillMesh A flag denoting whether there should also be spheres that
///                 fill in the mesh interior; false denotes only spheres
///                 enclosing the surface of the mesh are generated
////////////////////////////////////////////////////////////////////////////////
void getEnclosingSpheresOfMesh(const std::vector<geometry_msgs::Point>& vertices,
                               const std::vector<int>& triangles,
                               double radius, std::vector<Sphere>& spheres,
                               bool fillMesh = false);

void getEnclosingSpheresOfMesh(const std::vector<geometry_msgs::Point>& vertices,
                               const std::vector<int>& triangles,
                               double radius, std::vector<std::vector<double> >& spheres,
                               bool fillMesh = false);

bool getTrianglesFromMeshFile(std::string mesh_file, std::vector<int32_t> &triangles, std::vector<geometry_msgs::Point> &vertices);

void voxelizeMesh(const std::vector<geometry_msgs::Point>& vertices,
                  const std::vector<int>& triangles,
                  double radius, std::vector<tf::Vector3>& voxels,
                  bool fillMesh = false);

} // namespace sbpl_geometry_utils
#endif
