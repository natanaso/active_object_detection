#ifndef _OBJ_HPP_
#define _OBJ_HPP_

#include <iostream>
#include <vector>
#include <Eigen/Core>



// ros
#include <ros/package.h>

// Pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <pcl/PointIndices.h>
#include <pcl/common/centroid.h>		// compute3DCentroid()
#include <pcl/io/pcd_io.h>				// copyPointCloud()
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

// Boost
#include <boost/multi_array.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>

// Custom
#include "hypothesis.hpp"
#include <pcd_utils.hpp>
#include <io_utils.hpp>
#include <misc.hpp>

class obj
{
private:
	// Position
	int obj_id;
	Eigen::Vector3d position;				// position of the centroid
	Eigen::Vector3d table_position;		// position of the centroid projected to the table surface

	// Bounding box
	Eigen::Vector3d maxp;
	Eigen::Vector3d minp;

	// Surface
	pcl::ProjectInliers<pcl::PointXYZ> proj_cntr_;
	Eigen::Vector3f color;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr obj_surf;
	pcl::PointCloud<pcl::PointXYZ>::Ptr latest_view;
	double surf_size_delta;		// percentage change in surface size

	//Eigen::Vector2d position_cell;
	//std::pair<int, int> table_map_position;
	//pcl::PointIndices::Ptr cluster_idx;
	// Color
	
	// Belief
	std::vector< std::vector<double> > bel_hist;
	std::vector<bool> visited_vps;
	std::vector<int> vp_hist;
	//std::vector<double> rad_hist;
	
	// Decision
	int hid;
	Eigen::Vector4d orientation;

	// filtering
	float grid_cell_size_;
	pcl::VoxelGrid<pcl::PointXYZRGB> grid_filter_;
	
	
public:
	// Constructors
	/* ============================================ */
	obj( pcl::ModelCoefficients::Ptr & table_coeffs,
		  int id, 
		  pcl::PointCloud<pcl::PointXYZ>::Ptr & obj_view,
		  Eigen::Vector3f color,
		  std::vector<double>::iterator prior_start,
		  std::vector<double>::iterator prior_end )
	:		  
		obj_id(id), color(color),
		obj_surf( new pcl::PointCloud<pcl::PointXYZRGB> ), 
		latest_view( obj_view ),
		bel_hist(1, std::vector<double>(prior_start, prior_end)),
		hid(-1), grid_cell_size_(0)
	{
		init( table_coeffs, latest_view, obj_surf);
		
		/*
		// centroid projection
		proj_cntr_.setModelType (pcl::SACMODEL_PLANE);
		proj_cntr_.setModelCoefficients ( table_coeffs );
		
		// subsampling
		double cell_size = 0.001;	//leaf size in meters
		grid_filter_.setLeafSize (cell_size, cell_size, cell_size);
				
		// color surface
		pcl::copyPointCloud( *latest_view, *obj_surf );
		color_surf();
		surf_size_delta = static_cast<double>(obj_surf -> size());
		*/
		
		// position and bounding box
		compute_position();
		compute_bbox();
	}
	
	/* ============================================ */
	obj( pcl::ModelCoefficients::Ptr & table_coeffs,
		  int id, 
		  pcl::PointCloud<pcl::PointXYZRGB>::Ptr & obj_view,
		  Eigen::Vector3f color,
		  std::vector<double>::iterator prior_start,
		  std::vector<double>::iterator prior_end )
	:		  
		obj_id(id), color(color),
		obj_surf( obj_view ), 
		latest_view( new pcl::PointCloud<pcl::PointXYZ> ),
		bel_hist(1, std::vector<double>(prior_start, prior_end)),
		hid(-1), grid_cell_size_(0)
	{
	
		init( table_coeffs, obj_surf, latest_view );
		
		/*
		// centroid projection
		proj_cntr_.setModelType (pcl::SACMODEL_PLANE);
		proj_cntr_.setModelCoefficients ( table_coeffs );
		
		// subsampling
		double cell_size = 0.001;	//leaf size in meters
		grid_filter_.setLeafSize (cell_size, cell_size, cell_size);
				
		// color surface
		pcl::copyPointCloud( *obj_surf, *latest_view );
		color_surf();
		surf_size_delta = static_cast<double>(obj_surf -> size());
		*/
		// position and bounding box
		compute_position();
		compute_bbox();
	}
		
	/* ============================================ */
	obj( pcl::ModelCoefficients::Ptr & table_coeffs,
		  int id,
		  pcl::PointCloud<pcl::PointXYZ>::Ptr & obj_view,
		  Eigen::Vector3f color,
		  std::vector<double>::iterator prior_start,
		  std::vector<double>::iterator prior_end,  
		  Eigen::Vector3d position,
		  Eigen::Vector3d table_position,
		  Eigen::Vector3d maxp,
		  Eigen::Vector3d minp
		)
		: obj_id(id), position(position),
		  table_position(table_position), maxp(maxp), minp(minp),
		  color(color), obj_surf( new pcl::PointCloud<pcl::PointXYZRGB> ), 
		  latest_view( obj_view ),
		  bel_hist(1, std::vector<double>(prior_start, prior_end)),
		  hid(-1), grid_cell_size_(0)
	{
		init( table_coeffs, latest_view, obj_surf);
		/*
		// centroid projection
		proj_cntr_.setModelType (pcl::SACMODEL_PLANE);
		proj_cntr_.setModelCoefficients ( table_coeffs );
		
		double cell_size = 0.001;	//leaf size in meters
		grid_filter_.setLeafSize (cell_size, cell_size, cell_size);	
		
		pcl::copyPointCloud( *latest_view, *obj_surf );
		color_surf();
		surf_size_delta = static_cast<double>(obj_surf -> size());
		*/
	}

	/* ============================================ */
	obj( pcl::ModelCoefficients::Ptr & table_coeffs,
		  int id,
		  pcl::PointCloud<pcl::PointXYZRGB>::Ptr & obj_view,
		  Eigen::Vector3f color,
		  std::vector<double>::iterator prior_start,
		  std::vector<double>::iterator prior_end,  
		  Eigen::Vector3d position,
		  Eigen::Vector3d table_position,
		  Eigen::Vector3d maxp,
		  Eigen::Vector3d minp
		)
		: obj_id(id), position(position),
		  table_position(table_position), maxp(maxp), minp(minp),
		  color(color), obj_surf( obj_view ), 
		  latest_view( new pcl::PointCloud<pcl::PointXYZ> ),
		  bel_hist(1, std::vector<double>(prior_start, prior_end)),
		  hid(-1), grid_cell_size_(0)
	{
		init( table_coeffs, obj_surf, latest_view );
		/*
		// centroid projection
		proj_cntr_.setModelType (pcl::SACMODEL_PLANE);
		proj_cntr_.setModelCoefficients ( table_coeffs );
		
		double cell_size = 0.001;	//leaf size in meters
		grid_filter_.setLeafSize (cell_size, cell_size, cell_size);	
		
		pcl::copyPointCloud( *obj_surf, *latest_view );
		color_surf();
		surf_size_delta = static_cast<double>(obj_surf -> size());
		*/
	}
	
	// Destructor
	~obj(){}
	
	//****************************
	//******** Mutators
	//****************************
	void set_id( int id ){
		obj_id = id;
	}
	
	int get_id() const{
		return obj_id;
	}

	const Eigen::Vector3d & get_position() const{
		return position;
	}
	
	const Eigen::Vector3d & get_table_position() const{
		return table_position;
	}	

	const Eigen::Vector3d & get_maxp() const{
		return maxp;
	}

	const Eigen::Vector3d & get_minp() const{
		return minp;
	}
		
	void set_orientation(Eigen::Vector4d r){
		orientation = r;
	}
	
	const Eigen::Vector4d & get_orientation() const{
		return orientation;
	}
			
	// Pose
	/*
	void set_position(Eigen::Vector3d p){
		position = p;
	}
	
	void update_position(Eigen::Vector3d p){
		position = (p + pos_cnt*position)/(1 + pos_cnt);
		++pos_cnt;
	}
	
	void set_table_position(Eigen::Vector3d p){
		table_position = p;
	}
	
	void update_table_position(Eigen::Vector3d p){
		table_position = (p + tab_pos_cnt*table_position)/(1 + tab_pos_cnt);
		++tab_pos_cnt;
	}
	
	void set_maxp(Eigen::Vector3d maxp){
		this->maxp = maxp;
	}

	void update_maxp(Eigen::Vector3d p){
		maxp = (p + maxp_cnt * maxp)/(1 + maxp_cnt);
		++maxp_cnt;
	}
	


	void set_minp(Eigen::Vector3d minp){
		this->minp = minp;
	}

	void update_minp(Eigen::Vector3d p){
		minp = (p + minp_cnt * minp)/(1 + minp_cnt);
		++minp_cnt;
	}

	*/
	/*
	void set_map_position( std::pair<int,int> mp ){
		table_map_position.first = mp.first;
		table_map_position.second = mp.second;
	}
	
	const std::pair<int,int> & get_map_position(){
		return table_map_position;
	}
	*/
	
	// Color
	void set_color(Eigen::Vector3f c){
		color = c;
		color_surf();
	}
	
	const Eigen::Vector3f & get_color() const{
		return color;
	}
	

	void set_latest_view( pcl::PointCloud<pcl::PointXYZ>::Ptr & new_view )
	{
		latest_view = new_view;
	}
	
	void reset_latest_view( )
	{
		latest_view.reset( new pcl::PointCloud<pcl::PointXYZ> );
	}
	
	void add_latest_view( pcl::PointCloud<pcl::PointXYZ>::Ptr & new_view )
	{
		*latest_view += *new_view;
		update_surface( );
	}
		
	pcl::PointCloud<pcl::PointXYZ>::Ptr & get_latest_view()
	{
		return latest_view;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr & get_surface()
	{
		return obj_surf;
	}
	
	double get_surf_size_delta()
	{
		return surf_size_delta;
	}
	
	/*
	// Cluster idx	
	void cat_idx( std::vector<int>::iterator cat_idx_begin,
				  std::vector<int>::iterator cat_idx_end )
	{
		cluster_idx->indices.insert( cluster_idx->indices.end(), cat_idx_begin, cat_idx_end );
	}
		
	void shift_idx(int shift_amt){
		if(!cluster_idx->indices.empty()){
			std::transform( cluster_idx->indices.begin(), cluster_idx->indices.end(), 
							cluster_idx->indices.begin(), std::bind2nd(std::plus<int>(), shift_amt));
		
		}
	}
		
	pcl::PointIndices::Ptr get_idx(){
		return cluster_idx;
	}
	*/

	// Belief
	void set_belief( std::vector<double>::iterator bel_start,
					 std::vector<double>::iterator bel_end )
	{
		bel_hist.push_back( std::vector<double>(bel_start, bel_end) );
	}
	
	void get_belief( std::vector<double>::iterator & bel_start,
					 	  std::vector<double>::iterator & bel_end )
	{
		if( !bel_hist.empty() ){
			bel_start = bel_hist.back().begin();
			bel_end = bel_hist.back().end();
		}
	}
	
	//*************************************************
	// History of visited vps
	void set_visited_vps(int vp, int num_vps)
	{
		if(visited_vps.empty())
			visited_vps.resize(num_vps, false);
		
		visited_vps[vp] = true;
		
		// store the history of vps
		vp_hist.push_back(vp);
	}
	
	bool is_vp_visited(int vp)
	{
		if( (visited_vps.empty()) || !visited_vps[vp] )
			return false;
		return true;
	}
	
	void clear_visited_vps(int vp)
	{
		if( !visited_vps.empty() && visited_vps[vp])
			visited_vps[vp] = false;
	}
	
	void clear_visited_vps()
	{
		visited_vps.clear();
	}
	
	void get_vp_hist( std::vector<int>::iterator & vp_hist_start,
					 	  std::vector<int>::iterator & vp_hist_end,
					 	  std::vector<bool>::iterator & visited_vps_start,
					 	  std::vector<bool>::iterator & visited_vps_end )
	{
		vp_hist_start = vp_hist.begin();
		vp_hist_end = vp_hist.end();
		visited_vps_start = visited_vps.begin();
		visited_vps_end = visited_vps.end();
	}
	
	void set_vp_hist( std::vector<int>::iterator vp_hist_start,
							std::vector<int>::iterator vp_hist_end,
							std::vector<bool>::iterator visited_vps_start,
					 	   std::vector<bool>::iterator visited_vps_end )
	{
		vp_hist.clear();
		vp_hist.assign(vp_hist_start, vp_hist_end);
		
		visited_vps.clear();
		visited_vps.assign(visited_vps_start, visited_vps_end);
	}
	
	//*************************************************
	// Decision
	void set_decision( int hid, const Eigen::Vector4d & r){
		this->hid = hid;
		orientation = r;
	}
	
	int get_decision( ) const
	{
		return hid;
	}

	int get_decision( Eigen::Vector4d & r ) const
	{
		r = orientation;
		return hid;
	}

	// check if this object intersects with another
	bool is_intersect( const obj &o2 ) const
	{	
		/*
		// Three interval tests
		if( ( minp.x() > o2.get_maxp().x() ) ||
			 ( maxp.x() < o2.get_minp().x() ) ||
			 ( minp.y() > o2.get_maxp().y() ) ||
			 ( maxp.y() < o2.get_minp().y() ) ||
			 ( minp.z() > o2.get_maxp().z() ) ||
			 ( maxp.z() < o2.get_minp().z() ) )			 
			return false;
		else
			return true;
		*/
		
		return( maxp.x() > o2.get_minp().x() &&
				  minp.x() < o2.get_maxp().x() &&
				  maxp.y() > o2.get_minp().y() &&
				  minp.y() < o2.get_maxp().y() &&
				  maxp.z() > o2.get_minp().z() &&
				  minp.z() < o2.get_maxp().z() );
	}
	
	/*
	 * check if a ray starting from position and pointing at target
	 * intersects this object
	 * returns true if there is an intersection
	 */
	bool ray_intersect( Eigen::Vector3d const& position,
							  Eigen::Vector3d const& target )
	{
		// tranform from cube to world frame. 
		Eigen::Matrix4d T;
		T << Eigen::Matrix3d((maxp - minp).asDiagonal()), (minp+maxp)/2,
		  	  Eigen::MatrixXd::Zero(1,3), 1.0;
		  	  
		// Call ray_cube_intersect
		double res = ray_cube_intersect( position, target-position, T );
		
		return (res >= 0);
	}

	double get_volume()
	{
		return (maxp - minp).prod();
	}
	
	
	void refine_orientation(const Hypothesis & h, std::string pcd_model_dir,
									Eigen::Vector3d & obj_position, Eigen::Vector4d & obj_orientation)
	{
		// load the views from the visited vps and combine them into a model
		std::string avision_dir( ros::package::getPath("active_vision_pkg") );
		std::string vision_dir( ros::package::getPath("vision_module") );
		
		
		std::string plan_vps_path( avision_dir + "/data/plan_vps.txt");
		std::string omap_vps_path( vision_dir + "/data/omap_vps.txt");
		std::string views_path( avision_dir + "/../database/cloud_data_omap_vps/"
														   + h.name + "_views" );		
		
		// read the plan vps
		Eigen::MatrixXd plan_vps;
		io_utils::file2matrix( plan_vps_path, plan_vps, 3 );
		int num_vps = plan_vps.rows();
		
		// read the omap_vps
		Eigen::MatrixXd omap_vps;
		io_utils::file2matrix( omap_vps_path, omap_vps, 3 );
		
		Eigen::Matrix<double,3,3> wRo = misc::rotz(h.yaw) * 
							  misc::roty(h.pitch) * 
							  misc::rotx(h.roll);
							  
		// assemble the model
		pcl::PointCloud<pcl::PointXYZ>::Ptr model_ptr( new pcl::PointCloud<pcl::PointXYZ>);
		for( int vp = 0 ; vp < num_vps; ++vp )
		{
			// get the surface if it has been visited
			if(visited_vps[vp])
			{
				// load the correct view view
				Eigen::Vector3d position_obj_frame = wRo.transpose() * plan_vps.row(vp).transpose();
				
				// find the closest omap_vp
				Eigen::Matrix<int,1,1> vp_id;
				Eigen::Matrix<double,1,1> vp_dist;
				misc::NNsearch_3D( omap_vps, position_obj_frame.transpose(),
									vp_id, vp_dist );
				int ovp = vp_id(0,0);
								
				
				pcl::PointCloud<pcl::PointXYZ>::Ptr view_ptr( new pcl::PointCloud<pcl::PointXYZ>);
				std::string view_path( views_path + "/" + h.name + "_" 
												+ boost::lexical_cast<std::string>(ovp) +".pcd" );
				pcl::io::loadPCDFile<pcl::PointXYZ> ( view_path, *view_ptr );
		
				// rotate to world frame
				Eigen::Vector3d camera_position = omap_vps.row(ovp).transpose();
				Eigen::Vector4d camera_orientation = misc::target2quat( camera_position, 
																						  Eigen::Vector3d(0,0,0) );
		
				tf::Transform map_T_snsr;
				map_T_snsr.setOrigin( tf::Vector3(camera_position.x(), 
															 camera_position.y(), 
															 camera_position.z()) );
				map_T_snsr.setRotation( tf::Quaternion( camera_orientation.x(), 
																	 camera_orientation.y(), 
																	 camera_orientation.z(), 
																	 camera_orientation.w()) );
		
				tf::Transform snsr_T_optical;
				snsr_T_optical.setOrigin( tf::Vector3( 0.0, 0.0, 0.0 ) );
				snsr_T_optical.setRotation( tf::Quaternion( -0.5, 0.5, -0.5, 0.5 ) );
		
				tf::Transform map_T_optical = map_T_snsr * snsr_T_optical;

				pcl_ros::transformPointCloud( *view_ptr, *view_ptr, map_T_optical);
		
				// add to pcd_model
				*model_ptr +=  *view_ptr;
			}
		}

		//std::string model_path( pcd_model_dir + "/" + h.name + "_complete.pcd" );
		//pcl::io::loadPCDFile<pcl::PointXYZ> ( model_path, *model_ptr );
		
		// voxelgrid
		pcl::PointCloud<pcl::PointXYZ>::Ptr model_filt_ptr(new pcl::PointCloud<pcl::PointXYZ>);
		if( grid_cell_size_ > 0 )
		{
			pcl::VoxelGrid<pcl::PointXYZ> grid_filter_xyz;
			grid_filter_xyz.setLeafSize( grid_cell_size_, grid_cell_size_, grid_cell_size_ );
			grid_filter_xyz.setInputCloud( model_ptr );
			grid_filter_xyz.filter ( *model_filt_ptr );
		}
		
				
		// translate and rotate
		tf::Transform TT;
		tf::Quaternion q;
		q.setRPY( h.roll, h.pitch, h.yaw );
		TT.setRotation( q );
		pcl_ros::transformPointCloud( *model_filt_ptr, *model_filt_ptr, TT);


		Eigen::Vector4f centroid;
		pcl::compute3DCentroid<pcl::PointXYZ> ( *model_filt_ptr, centroid );	
		//std::cout << "orig_cntr = " << centroid.transpose() << std::endl;
			
		TT.setOrigin( tf::Vector3(position.x() - static_cast<double>(centroid.x()), 
										  position.y() - static_cast<double>(centroid.y()), 
										  position.z() - static_cast<double>(centroid.z())) );

		TT.setRotation( tf::Quaternion(0,0,0,1) );
		pcl_ros::transformPointCloud( *model_filt_ptr, *model_filt_ptr, TT);
		TT.setRotation( q );
		
		// compute new centroid for comparison
		//pcl::compute3DCentroid<pcl::PointXYZ> ( *model_filt_ptr, centroid );
		//std::cout << "new_cntr = " << centroid.transpose() << std::endl;
		//std::cout << "position = " << position.transpose() << std::endl;
		
		// icp with object surface to get final transform
		pcl::PointCloud<pcl::PointXYZ>::Ptr obj_surf_xyz_ptr( new pcl::PointCloud<pcl::PointXYZ>);
		pcl::copyPointCloud(*obj_surf, *obj_surf_xyz_ptr);
				
		// Save for viewing
		//pcl::PCDWriter writer;
		//writer.writeASCII( "/home/natanaso/Desktop/model.pcd", *model_filt_ptr );
		//writer.writeASCII( "/home/natanaso/Desktop/object.pcd", *obj_surf_xyz_ptr );
		
		double fitness_score = 1000;
		Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
		Eigen::Matrix4f fTrans = Eigen::Matrix4f::Identity();
		pcd_utils::align_clouds( model_filt_ptr, obj_surf_xyz_ptr, 0.4, guess, fTrans, fitness_score );
		
		Eigen::Matrix4f best_trans = fTrans;
		double best_score = fitness_score;
		{
			double dev = 0.1;
			const double arr[] = {dev, 0.0, 0.0,
										-dev, 0.0, 0.0, 
										 0.0, dev, 0.0, 
										 0.0, -dev, 0.0,
										 0.0, 0.0, dev,
										 0.0, 0.0, -dev};
			for(int k = 0; k < 6; ++k)
			{
				// generate the offset cloud above the table
				pcl::PointCloud<pcl::PointXYZ>::Ptr cld_ptr_off(new pcl::PointCloud<pcl::PointXYZ>);
				pcl::copyPointCloud(*model_filt_ptr, *cld_ptr_off);
				for( pcl::PointCloud<pcl::PointXYZ>::iterator it = cld_ptr_off->begin();
					it != cld_ptr_off->end(); ++it)
				{
					(*it).x += arr[3*k];
					(*it).y += arr[3*k+1];
					(*it).z += arr[3*k+2];
				}
				pcd_utils::align_clouds( cld_ptr_off, obj_surf_xyz_ptr, 0.4, guess, fTrans, fitness_score );	
								
				if( fitness_score < best_score)
				{
					best_score = fitness_score;
			
					Eigen::Matrix4f T = Eigen::Matrix4f::Identity();				
					T(0,3) = arr[3*k];
					T(1,3) = arr[3*k+1];
					T(2,3) = arr[3*k+2];
					best_trans = fTrans * T;
				
				}	
			}					
		}
		
		// Save for viewing
		pcl::transformPointCloud ( *model_filt_ptr, *model_filt_ptr, best_trans );
		//writer.writeASCII( "/home/natanaso/Desktop/aligned_model.pcd", *model_filt_ptr );
		
		
		//std::cout << "Final transform:" << std::endl;
		//std::cout << best_trans << std::endl;
		
		tf::Vector3 t( best_trans(0,3), best_trans(1,3), best_trans(2,3) );
		tf::Matrix3x3 R( best_trans(0,0), best_trans(0,1), best_trans(0,2),
							  best_trans(1,0), best_trans(1,1), best_trans(1,2),
							  best_trans(2,0), best_trans(2,1), best_trans(2,2) );
							  
		tf::Transform CorrectionT( R, t );
		
		CorrectionT *= TT;
		
		t = CorrectionT.getOrigin();
		obj_position << t.getX(), t.getY(), t.getZ();
		
		q = CorrectionT.getRotation();
		obj_orientation << q.getX(), q.getY(), q.getZ(), q.getW(); 
	}




	
	void refine_orientation_v2(const Hypothesis & h, std::string pcd_model_dir,
									Eigen::Vector3d & obj_position, Eigen::Vector4d & obj_orientation)
	{

		pcl::PointCloud<pcl::PointXYZ>::Ptr model_ptr( new pcl::PointCloud<pcl::PointXYZ>);
		std::string model_path( pcd_model_dir + "/" + h.name + "_complete.pcd" );
		pcl::io::loadPCDFile<pcl::PointXYZ> ( model_path, *model_ptr );
		
		// voxelgrid
		pcl::PointCloud<pcl::PointXYZ>::Ptr model_filt_ptr(new pcl::PointCloud<pcl::PointXYZ>);
		if( grid_cell_size_ > 0 )
		{
			pcl::VoxelGrid<pcl::PointXYZ> grid_filter_xyz;
			grid_filter_xyz.setLeafSize( grid_cell_size_, grid_cell_size_, grid_cell_size_ );
			grid_filter_xyz.setInputCloud( model_ptr );
			grid_filter_xyz.filter ( *model_filt_ptr );
		}
		
				
		// translate and rotate
		tf::Transform TT;
		tf::Quaternion q;
		q.setRPY( h.roll, h.pitch, h.yaw );
		TT.setRotation( q );
		pcl_ros::transformPointCloud( *model_filt_ptr, *model_filt_ptr, TT);


		Eigen::Vector4f centroid;
		pcl::compute3DCentroid<pcl::PointXYZ> ( *model_filt_ptr, centroid );	
			
		TT.setOrigin( tf::Vector3(position.x() - static_cast<double>(centroid.x()), 
										  position.y() - static_cast<double>(centroid.y()), 
										  position.z() - static_cast<double>(centroid.z())) );

		TT.setRotation( tf::Quaternion(0,0,0,1) );
		pcl_ros::transformPointCloud( *model_filt_ptr, *model_filt_ptr, TT);
		TT.setRotation( q );
		

		
		// icp with object surface to get final transform
		pcl::PointCloud<pcl::PointXYZ>::Ptr obj_surf_xyz_ptr( new pcl::PointCloud<pcl::PointXYZ>);
		pcl::copyPointCloud(*obj_surf, *obj_surf_xyz_ptr);
				
		
		double fitness_score = 1000;
		Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
		Eigen::Matrix4f fTrans = Eigen::Matrix4f::Identity();
		pcd_utils::align_clouds( model_filt_ptr, obj_surf_xyz_ptr, 0.4, guess, fTrans, fitness_score );
		
		Eigen::Matrix4f best_trans = fTrans;
		double best_score = fitness_score;
		{
			double dev = 0.1;
			const double arr[] = {dev, 0.0, 0.0,
										-dev, 0.0, 0.0, 
										 0.0, dev, 0.0, 
										 0.0, -dev, 0.0,
										 0.0, 0.0, dev,
										 0.0, 0.0, -dev};
			for(int k = 0; k < 6; ++k)
			{
				// generate the offset cloud above the table
				pcl::PointCloud<pcl::PointXYZ>::Ptr cld_ptr_off(new pcl::PointCloud<pcl::PointXYZ>);
				pcl::copyPointCloud(*model_filt_ptr, *cld_ptr_off);
				for( pcl::PointCloud<pcl::PointXYZ>::iterator it = cld_ptr_off->begin();
					it != cld_ptr_off->end(); ++it)
				{
					(*it).x += arr[3*k];
					(*it).y += arr[3*k+1];
					(*it).z += arr[3*k+2];
				}
				pcd_utils::align_clouds( cld_ptr_off, obj_surf_xyz_ptr, 0.4, guess, fTrans, fitness_score );	
								
				if( fitness_score < best_score)
				{
					best_score = fitness_score;
			
					Eigen::Matrix4f T = Eigen::Matrix4f::Identity();				
					T(0,3) = arr[3*k];
					T(1,3) = arr[3*k+1];
					T(2,3) = arr[3*k+2];
					best_trans = fTrans * T;
				
				}	
			}					
		}
		
		// Save for viewing
		pcl::transformPointCloud ( *model_filt_ptr, *model_filt_ptr, best_trans );
		
		tf::Vector3 t( best_trans(0,3), best_trans(1,3), best_trans(2,3) );
		tf::Matrix3x3 R( best_trans(0,0), best_trans(0,1), best_trans(0,2),
							  best_trans(1,0), best_trans(1,1), best_trans(1,2),
							  best_trans(2,0), best_trans(2,1), best_trans(2,2) );
							  
		tf::Transform CorrectionT( R, t );
		
		CorrectionT *= TT;
		
		t = CorrectionT.getOrigin();
		obj_position << t.getX(), t.getY(), t.getZ();
		
		q = CorrectionT.getRotation();
		obj_orientation << q.getX(), q.getY(), q.getZ(), q.getW(); 
	}
	
	
	
	bool operator<( const obj &o2) const
	{
		return ((maxp - minp).prod() < (o2.get_maxp() - o2.get_minp()).prod());
	}

	
	static bool comp_less( boost::shared_ptr<obj> & o1, boost::shared_ptr<obj> & o2)
	{
		return ((o1->get_maxp() - o1->get_minp()).prod() < (o2->get_maxp() - o2->get_minp()).prod());
	}
	
	std::string to_str() const
   {
    	std::stringstream ss;
    	ss << std::endl;
		ss << "OBJ ID = " << obj_id << std::endl;
		ss << "Position = " << position.transpose() << std::endl;
		ss << "Projected position = " << table_position.transpose() << std::endl;
		ss << "Maxp = " << maxp.transpose() << std::endl;
		ss << "Minp = " << minp.transpose() << std::endl;
		ss << "Color = " << color.transpose() << std::endl;
		ss << "Surface size = " << obj_surf->size() << std::endl;
		ss << "Latest view size = " << latest_view->size() << std::endl;
		
		ss << "Belief = ";
		for( std::vector<double>::const_iterator it = bel_hist.back().begin();
				it != bel_hist.back().end(); ++it )
			ss << *it << " ";
		ss << std::endl;
		
		ss << "Visited vps = ";
		for( std::vector<bool>::const_iterator it = visited_vps.begin();
				it != visited_vps.end(); ++it )
			ss << *it << " ";
		ss << std::endl;
				
		ss << "Decision ID = " << hid << std::endl;
		ss << "Orientation = " << orientation.transpose() << std::endl;
   	return ss.str();
   }
   
   std::string to_file_str(boost::multi_array<double,2> const& dMap) const
   {
   	int num_msrmnts = vp_hist.size();
   	double move_cost = 0.0;
   	for(int vp = 1; vp < num_msrmnts; ++vp)
			move_cost += dMap[vp_hist[vp-1]][vp_hist[vp]];
		
   	std::stringstream ss;
   	// ID position table_position decision num_msrmnts move_cost
   	ss << obj_id << " (" << position.transpose() << ") ("
   		<< table_position.transpose() << ") "
   		<< hid << " " << " " << num_msrmnts << " "
   		<< move_cost << std::endl;
   		
   	return ss.str();	
   }
   
private:

	template <typename P1, typename P2>
	void init( pcl::ModelCoefficients::Ptr & table_coeffs,
				  boost::shared_ptr<pcl::PointCloud<P1> > & s1,
				  boost::shared_ptr<pcl::PointCloud<P2> > & s2 )
	{
		// centroid projection
		proj_cntr_.setModelType (pcl::SACMODEL_PLANE);
		proj_cntr_.setModelCoefficients ( table_coeffs );
		
		// subsampling
		grid_cell_size_ = 0.004;	//leaf size in meters
		
		if( grid_cell_size_ > 0)
			grid_filter_.setLeafSize (grid_cell_size_, grid_cell_size_, grid_cell_size_);
				
		// color surface
		pcl::copyPointCloud( *s1, *s2 );
		color_surf();
		surf_size_delta = static_cast<double>(obj_surf -> size());
	}
	
	friend std::ostream & operator<<(std::ostream &os, const obj &o1)
	{
		return os << o1.to_str();
	}

	void update_surface()
	{		
		// if the latest view is empty we are done
		if( latest_view -> size() < 50 )
			return;
			
		// Create an rgb cloud
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_color_view(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::copyPointCloud( *latest_view, *new_color_view );

		// add the two surfaces together
		double tmp = static_cast<double>( obj_surf->size() );
		*obj_surf += *new_color_view;

		// Filter to remove duplicates and reduce size
		// XXX: it is necessary to create a new cloud for the output; otherwise the filter does not work right!!!
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cld( new pcl::PointCloud<pcl::PointXYZRGB> );
		if( grid_cell_size_ > 0 ){
			grid_filter_.setInputCloud( obj_surf );
			grid_filter_.filter ( *filtered_cld );
			obj_surf = filtered_cld;
		}
		
		color_surf();
		
		// TODO: Think about this a little more
		surf_size_delta = (static_cast<double>( obj_surf->size() ) - tmp) / tmp;	// Change_size / Old_size
		
		// Update position and bounding box
		compute_position();
		compute_bbox();
	}
		
	void color_surf()
	{
		for( pcl::PointCloud<pcl::PointXYZRGB>::iterator it = obj_surf->begin();
				it != obj_surf->end(); ++it)
		{
			it->r = color.x();
			it->g = color.y();
			it->b = color.z();
		}
	}
	
	void compute_position()
	{
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid<pcl::PointXYZRGB> ( *obj_surf, centroid );
		position.x() = static_cast<double>(centroid.x());
		position.y() = static_cast<double>(centroid.y());
		position.z() = static_cast<double>(centroid.z());
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr cntr_cld(new pcl::PointCloud<pcl::PointXYZ>);
		cntr_cld->push_back( pcl::PointXYZ( position.x(), position.y(), position.z() ) );
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr proj_cntr_cld(new pcl::PointCloud<pcl::PointXYZ>);
		
		proj_cntr_.setInputCloud ( cntr_cld );
		proj_cntr_.filter( *proj_cntr_cld );
		table_position.x() = proj_cntr_cld->at(0).x;
		table_position.y() = proj_cntr_cld->at(0).y;
		table_position.z() = proj_cntr_cld->at(0).z;
	}
	
	void compute_bbox()
	{
		Eigen::Vector4f minp_tmp, maxp_tmp;
		pcl::getMinMax3D( *obj_surf, minp_tmp, maxp_tmp);
		minp.x() = static_cast<double>(minp_tmp.x());
		minp.y() = static_cast<double>(minp_tmp.y());
		minp.z() = static_cast<double>(minp_tmp.z());
		maxp.x() = static_cast<double>(maxp_tmp.x());
		maxp.y() = static_cast<double>(maxp_tmp.y());
		maxp.z() = static_cast<double>(maxp_tmp.z());  
	}
	
		// T is a transform (scale, rotate, translate), which takes a point in the cube frame to the world frame: [R t; 0 1] [sx 0 0 0; 0 sy 0 0; 0 0 sz 0; 0 0 0 1] 	
	static double ray_cube_intersect( Eigen::Vector3d const& P0,
												 Eigen::Vector3d const& V0,
												 Eigen::Matrix4d const& T )
	{
		double err_const = 0.0001;
		typedef Eigen::Vector3d vec3;
		typedef Eigen::Vector4d vec4;
		typedef Eigen::Matrix4d mat4;
	
		std::pair<double, vec3> p;
		p.first = -1.0;
		p.second << 0.0, 0.0, 0.0;
	
		//Start with front and back faces
		vec3 N1(0.0, 0.0, 1.0);
		vec3 N2(0.0, 0.0, -1.0);
	
		mat4 Tinv = T.inverse();
	
		vec4 temp = Tinv * (vec4() << V0.normalized(), 0.0 ).finished();
		vec3 V( temp[0], temp[1], temp[2]);
	
		temp = Tinv * (vec4() << P0, 1.0).finished();
		vec3 P( temp[0], temp[1], temp[2]);
	
		if( ( N1.dot(V) != 0) && !(P.z() < 0.5 && P.z() > -0.5))
		{
		   double t1 = (N1.dot(vec3(0.0, 0.0, 0.5) - P)) / (N1.dot(V));
			double t2 = (N2.dot(vec3(0.0, 0.0, -0.5) - P)) / (N2.dot(V));

			double tempT = p.first;
			vec3 tempN = p.second;

			if(t2 > 0 && t1 > 0)
			{
				tempT = (t2 > t1) ? t1 : t2;
				tempN = (t2 > t1) ? N1 : N2;
			}
			else if(t2 < 0 && t1 > 0) 
			{
				tempT = t1;
				tempN = N1;
			}
			else if(t2 > 0 && t1 < 0)
			{
				tempT = t2;
				tempN = N2;
			}

			if( (P.x() + tempT*V.x() >= -0.5 - err_const) && 
				 (P.x() + tempT*V.x() <= 0.5 + err_const) &&
				 (P.y() + tempT*V.y() >= -0.5 - err_const) &&
				 (P.y() + tempT*V.y() <= 0.5 + err_const) )
			{
				p.first = tempT;
				p.second = tempN;
			}
		}
	
		//side faces
		vec3 N3(-1.0, 0.0, 0.0);
		vec3 N4(1.0, 0.0, 0.0);
		
		if( (N3.dot(V) != 0) && !(P.x() < 0.5 && P.x() > -0.5) )
		{
		   double t1 = (N3.dot(vec3(-0.5, 0.0, 0.0) - P)) / (N3.dot(V));
			double t2 = (N4.dot(vec3(0.5, 0.0, 0.0) - P)) / (N4.dot(V));

			double tempT = p.first;
			vec3 tempN = p.second;

			if(t2 > 0 && t1 > 0)
			{
				double temp = (t2 > t1) ? t1 : t2;
				vec3 Ntemp = (t2 > t1) ? N3 : N4;

				if (temp > p.first)
				{
					tempT = temp;
					tempN = Ntemp;
				}
				
			}
			else if(t2 < 0 && t1 > 0)
			{
				tempT = t1;
				tempN = N3;
			}
			else if(t2 > 0 && t1 < 0)
			{
				tempT = t2;
				tempN = N4;
			}

			if( (P.z() + tempT*V.z() >= -0.5 - err_const) && 
				 (P.z() + tempT*V.z() <= 0.5 + err_const) &&
				 (P.y() + tempT*V.y() >= -0.5 - err_const) && 
				 (P.y() + tempT*V.y() <= 0.5 + err_const) )
			{
				p.first = tempT;
				p.second = tempN;
			}
		}
	
		//bases
		vec3 N5(0.0, 1.0, 0.0);
		vec3 N6(0.0, -1.0, 0.0);

		if( (N5.dot(V) != 0) && !(P.y() < 0.5 && P.y() > -0.5))
		{
			double t1 = (N5.dot(vec3(0.0, 0.5, 0.0) - P)) / (N5.dot(V));
			double t2 = (N6.dot(vec3(0.0, -0.5, 0.0) - P)) / (N6.dot(V));

			double tempT = p.first;
			vec3 tempN = p.second;

			if(t2 > 0 && t1 > 0)
			{
				double temp = (t2 > t1) ? t1 : t2;
				vec3 Ntemp = (t2 > t1) ? N5 : N6;

				if (temp > p.first)
				{
					tempT = temp;
					tempN = Ntemp;
				}
			}
			else if(t2 < 0 && t1 > 0) 
			{
				tempT = t1;
				tempN = N5;
			}
			else if(t2 > 0 && t1 < 0)
			{
				tempT = t2;
				tempN = N6;
			}

			if( (P.x() + tempT*V.x() >= -0.5) && 
				 (P.x() + tempT*V.x() <= 0.5) &&
				 (P.z() + tempT*V.z() >= -0.5) && 
				 (P.z() + tempT*V.z() <= 0.5) )
			{
				p.first = tempT;
				p.second = tempN;
			}
		}
		
		return p.first;	
	}
	
};

#endif
