#ifndef VKIN_OFFLINE_HPP_
#define VKIN_OFFLINE_HPP_
#include <string>
#include <vtkSmartPointer.h>
#include <vtkCellLocator.h>
#include <vtkPolyData.h>
#include <vtkPLYReader.h>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// BOOST
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

#define EPS 0.00001


// Coordinate Frame Convensions:
// Standard: x = forward, y = left, z = up
// Optical: x = right, y = down, z = forward
// Color code: x = RED, y = GREEN, z = BLUE

class vkin_offline
{
public:	
	// For generating normal distribution
	typedef boost::mt19937                      ENG;    // Mersenne Twister
   typedef boost::normal_distribution<double> DIST;    // Normal Distribution
   typedef boost::variate_generator<ENG&,DIST> GEN;    // Variate generator

protected:
    // Pose in world frame
	Eigen::Vector3d position_;
	Eigen::Vector4d orientation_;
	
private:
	// scene files
	vtkSmartPointer<vtkCellLocator> tree;
	double bounds[6];
	
	// noise
	ENG rng;
	DIST normal_distrib;
	GEN gaussian_rng;
	void addNoise( const Eigen::Vector3d position,
				   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
				   GEN &generator );
				  
	// Scan parameters
	struct ScanParameters
	{
	  int coord;		// 0 = obj_coord, 1 = optical camera coord, 2 = standard camera coord
	  bool organized;
	  bool add_noise;
	  
	  int nr_scans;             // number of steps for sweep movement (900) (550)
	  int nr_points_in_scans;   // number of laser beam measurements per scan (900) (740)
	  double vert_res;          // vertical resolution (res. of sweep) in degrees (0.25) (0.0818)
	  double hor_res;           // horizontal resolution (of laser beam) in degrees (0.25) (0.0784)
	  double max_dist;          // maximum distance in units. (30000 mm)
	  
	  double vert_start;		// - (vert FOV)/2 (degrees)
	  double vert_end;		    // (vert FOV)/2 (degrees)
	  double hor_start;			// - (horiz FOV)/2 (degrees)
	  double hor_end;			// - (horiz FOV)/2 (degrees)
	  
	  ScanParameters( int coord, bool org, bool add_noise, 
	  				  int nr_sc, int nr_pt, double v_res, double h_res,
	  				  double max_dist )
	  	: coord( coord ), organized(org), add_noise(add_noise),
	  	  nr_scans(nr_sc), nr_points_in_scans(nr_pt), vert_res(v_res),
	  	  hor_res(h_res), max_dist(max_dist), 
	  	  vert_start( -(static_cast<double> (nr_scans - 1) / 2.0) * vert_res ),
	  	  vert_end( (nr_scans-1) * vert_res + vert_start ),
	  	  hor_start( -(static_cast<double> (nr_points_in_scans - 1) / 2.0) * hor_res ),
	  	  hor_end( (nr_points_in_scans-1) * hor_res + hor_start )
	  {}
	};
	
	// tools
	ScanParameters sp;

public:
	vkin_offline(const Eigen::Vector3d position, const Eigen::Vector4d orientation,
				 int coord, bool organized, bool add_noise)
		: position_(position), orientation_(orientation), rng(time(0)), 
		  normal_distrib(0.0, 1.0), gaussian_rng( rng, normal_distrib ),
	  	  sp( coord, organized, add_noise, 480, 640, (45.0/480.0), (58.0/640.0), 5.0 )
	{}
				 
	~vkin_offline(){}
	
	void init_vkin( const std::string ply_file_path );
	
	void set_pose( const Eigen::Vector3d position, const Eigen::Vector4d orientation )
	{
		position_ = position;
		orientation_ = orientation;
	}
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr sense();
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr sense( const Eigen::Vector3d & position, 
											   const Eigen::Vector3d & target );

	pcl::PointCloud<pcl::PointXYZ>::Ptr sense( const Eigen::Vector3d & position, 
											   const Eigen::Vector4d & orientation );
											   

	
	
private:
	/** \brief Loads a 3D point cloud from a given PLY fileName, and returns: a
	* vtkDataSet object containing the point cloud.
	* \param file_name the name of the file containing the PLY dataset
	*/
	vtkPolyData* loadPLYAsDataSet (const char* file_name)
	{
	  vtkPLYReader* reader = vtkPLYReader::New ();
	  reader->SetFileName (file_name);
	  reader->Update ();
	  return (reader->GetOutput ());
	}					  
};
#endif
