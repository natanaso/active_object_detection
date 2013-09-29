// STANDARD
#include <iostream>
#include <cmath>
#include <cassert>

// VTK
#include <vtkGeneralTransform.h>
#include <vtkMath.h>

// CUSTOM
#include "virtual_kinect_pkg/vkin_offline.hpp"
#include "misc.hpp"

void 
vkin_offline::init_vkin( const std::string ply_file_path )
{
	vtkSmartPointer<vtkPolyData> scene_ = loadPLYAsDataSet( ply_file_path.c_str() );
	
	// xmin, xmax, ymin, ymax, zmin, zmax 
	scene_->GetBounds (bounds);
	
	/*
	std::cout << "The bounds are "
			  << bounds[0] << " "
			  << bounds[1] << " "
			  << bounds[2] << " "
			  << bounds[3] << " "
			  << bounds[4] << " "
			  << bounds[5] << std::endl;
	*/		  
	tree = vtkSmartPointer<vtkCellLocator>::New ();
	tree->SetDataSet (scene_);
	tree->CacheCellBoundsOn ();
	tree->SetTolerance (0.0);
	tree->SetNumberOfCellsPerBucket (1);
	tree->AutomaticOn ();
	tree->BuildLocator ();
	tree->Update ();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
vkin_offline::sense()
{
	/*
	std::cout << "Sensing at position " << position_.transpose()
			  << " and orientation " << orientation_.transpose() << std::endl;
	*/		  
	return sense( position_, orientation_ );
}


pcl::PointCloud<pcl::PointXYZ>::Ptr
vkin_offline::sense( const Eigen::Vector3d & position, const Eigen::Vector3d & target )
{
	// Determine the orientation	
	Eigen::Vector4d orientation = misc::target2quat(position, target);
	
	/*	
	std::cout << "Sensing at position " << position_.transpose()
			  << " and orientation " << orientation.transpose() << std::endl;
	*/
	return sense( position, orientation );
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
vkin_offline::sense( const Eigen::Vector3d & position, const Eigen::Vector4d & orientation )
{
	double up[3] = {0.0, 0.0, 0.0};
	double right[3] = {0.0, 0.0, 0.0};
	double x_axis[3] = {1.0, 0.0, 0.0};
	double z_axis[3] = {0.0, 0.0, 1.0};
	double eye[3], viewray[3];

	// Camera position
	eye[0] = position.x();
	eye[1] = position.y();
	eye[2] = position.z();

	// Viewray, right, and up
	// In Standard coordinate system (x = forward, y = left, z = up)
	// wRs = [viewray | left | up]
	Eigen::Matrix<double,3,3> wRs = misc::quat2rot( orientation );
	
	viewray[0] = wRs(0,0);
	viewray[1] = wRs(1,0);
	viewray[2] = wRs(2,0);	
	if (fabs(viewray[0]) < EPS) viewray[0] = 0;
	if (fabs(viewray[1]) < EPS) viewray[1] = 0;
	if (fabs(viewray[2]) < EPS) viewray[2] = 0;

	right[0] = -wRs(0,1);
	right[1] = -wRs(1,1);
	right[2] = -wRs(2,1);	
	if (fabs(right[0]) < EPS) right[0] = 0;
	if (fabs(right[1]) < EPS) right[1] = 0;
	if (fabs(right[2]) < EPS) right[2] = 0;

	up[0] = wRs(0,2);
	up[1] = wRs(1,2);
	up[2] = wRs(2,2);	
	if (fabs(up[0]) < EPS) up[0] = 0;
	if (fabs(up[1]) < EPS) up[1] = 0;
	if (fabs(up[2]) < EPS) up[2] = 0;


   // Prepare the point cloud data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

   double temp_beam[3], beam[3], p[3];
  	double p_coords[3], x[3], t;
  	int subId;
  	
	// Create a transformation
	vtkGeneralTransform* tr1 = vtkGeneralTransform::New ();
	vtkGeneralTransform* tr2 = vtkGeneralTransform::New ();
	 
	 // Sweep vertically    
	for (double vert = sp.vert_start; vert <= sp.vert_end; vert += sp.vert_res)
	{
		
		tr1->Identity ();
		tr1->RotateWXYZ (vert, right);
		tr1->InternalTransformPoint (viewray, temp_beam);

		// Sweep horizontally
		for (double hor = sp.hor_start; hor <= sp.hor_end; hor += sp.hor_res)
		{
		
		  // Create a beam vector with (lat,long) angles (vert, hor) with the viewray
			tr2->Identity ();
			tr2->RotateWXYZ (hor, up);
			tr2->InternalTransformPoint (temp_beam, beam);
			vtkMath::Normalize (beam);

			// Find point at max range: p = eye + beam * max_dist
			for (int d = 0; d < 3; d++)
				p[d] = eye[d] + beam[d] * sp.max_dist;

			// Put p_coords into laser scan at packet id = vert, scan id = hor
			/*
			std::cout <<"eye = "<< eye[0] << " " << eye[1] << " " << eye[2] << std::endl;
			std::cout <<"p = " << p[0] << " " << p[1] << " " << p[2] << std::endl;
			std::cout <<"x = " << x[0] << " " << x[1] << " " << x[2] << std::endl;
			std::cout <<"t = " << t << ", subId = " << subId << std::endl;
			*/

			// Determine if the ray between eye (camera position) and p (max range)
			// intersects with the tree given a tolerance of 0
			// return the intersection coordinates in x in the WORLD frame
			// return the cell which was intersected by the ray in cellId
			vtkIdType cellId;
			if (tree->IntersectWithLine (eye, p, 0, t, x, p_coords, subId, cellId))
			{ 
				// x are the coordinates in the world frame
				pcl::PointXYZ pt;
				switch( sp.coord )
				{
					case 0:{		// object coordinates
						pt.x = static_cast<float> (x[0]); 
						pt.y = static_cast<float> (x[1]); 
						pt.z = static_cast<float> (x[2]);
						//pt.vp_x = static_cast<float> (eye[0]); 
						//pt.vp_y = static_cast<float> (eye[1]); 
						//pt.vp_z = static_cast<float> (eye[2]);
						break;
					}
					case 2:{		// camera coordinates: x = forward, y = left, z = up
						// Translate the origin to the sensor position
						x[0] -= eye[0];
						x[1] -= eye[1];
						x[2] -= eye[2];	
						
						// sRw = wRs^T = [viewray ; left ; up]
						pt.x = static_cast<float> ( viewray[0]*x[0] + viewray[1]*x[1] + viewray[2]*x[2] );
						pt.y = static_cast<float> ( -right[0]*x[0] - right[1]*x[1] - right[2]*x[2] );
						pt.z = static_cast<float> ( up[0]*x[0] + up[1]*x[1] + up[2]*x[2] );
						//pt.vp_x = pt.vp_y = pt.vp_z = 0.0f;
						break;
					}
					default:{	// optical coordinates: z = forward, x = right, y = down
					
						// Translate the origin to the sensor position
						x[0] -= eye[0];
						x[1] -= eye[1];
						x[2] -= eye[2];
					
						pt.x = static_cast<float> ( right[0]*x[0] + right[1]*x[1] + right[2]*x[2] );
						pt.y = static_cast<float> ( -up[0]*x[0] - up[1]*x[1] - up[2]*x[2] );
						pt.z = static_cast<float> ( viewray[0]*x[0] + viewray[1]*x[1] + viewray[2]*x[2] ); 
						//pt.vp_x = pt.vp_y = pt.vp_z = 0.0f;
					}
				}
			 	
			 	cloud_ptr->points.push_back (pt);
			}
			else if (sp.organized)
			{
				pcl::PointXYZ pt;
				pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN ();
				//pt.vp_x = static_cast<float> (eye[0]);
				//pt.vp_y = static_cast<float> (eye[1]);
				//pt.vp_z = static_cast<float> (eye[2]);
				
				cloud_ptr->points.push_back (pt);
			 }
		} // Horizontal
	} // Vertical
	 
	// Add noise
	if(sp.add_noise)
	{
		if ( sp.coord == 0 )
	 		addNoise(position, cloud_ptr, gaussian_rng);
	 	else
	 		addNoise(Eigen::Vector3d(0.0,0.0,0.0), cloud_ptr, gaussian_rng);
	}
	 
	if (sp.organized)
	{
		cloud_ptr->height = 1 + static_cast<uint32_t> ((sp.vert_end - sp.vert_start) / sp.vert_res);
		cloud_ptr->width = 1 + static_cast<uint32_t> ((sp.hor_end - sp.hor_start) / sp.hor_res);
	}
	else
	{
		cloud_ptr->width = static_cast<uint32_t> (cloud_ptr->points.size ());
		cloud_ptr->height = 1;
	}

	return cloud_ptr;
}


void 
vkin_offline::addNoise( const Eigen::Vector3d position, 
					    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
					    vkin_offline::GEN &generator )
{		
	Eigen::Vector3d ray_vec;
	double noise_param = 0.0025;		// TRUE IS 0.0005
	double noise_std;
	double sq_norm;
	for (std::size_t cp = 0; cp < cloud->points.size(); cp++)
	{
		// get a vector pointing from the camera to the point
		ray_vec.x() = cloud->points[cp].x - position.x();
		ray_vec.y() = cloud->points[cp].y - position.y();
		ray_vec.z() = cloud->points[cp].z - position.z();
		
		// normalize it
		sq_norm = ray_vec.squaredNorm(); 
		ray_vec = ray_vec / sqrt(sq_norm);
		
		// get the noise vector magnitude
		noise_std = noise_param * sq_norm;
		
		//set the correct size
		ray_vec = generator()*noise_std*ray_vec;
		
		// add the noise
		cloud->points[cp].x += ray_vec.x();
		cloud->points[cp].y += ray_vec.y();
		cloud->points[cp].z += ray_vec.z();
	}
}

/*
pcl::PointCloud<pcl::PointXYZ>::Ptr
vkin_offline::sense( const Eigen::Vector3d position, const Eigen::Vector4d orientation )
{
	// Determine the target point		
	Eigen::Vector3d target = misc::quat2target(position, orientation);

	//std::cout << "Sensing at position " << position_.transpose()
	//		  << " and target " << target.transpose() << std::endl;
	
	return sense( position, target );
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
vkin_offline::sense( const Eigen::Vector3d position, const Eigen::Vector3d target )
{		
	double up[3] = {0.0, 0.0, 0.0};
	double right[3] = {0.0, 0.0, 0.0};
	double x_axis[3] = {1.0, 0.0, 0.0};
	double z_axis[3] = {0.0, 0.0, 1.0};
	double eye[3], viewray[3];

	// Camera position
	eye[0] = position.x();
	eye[1] = position.y();
	eye[2] = position.z();

	// camera viewray ( forward axis )
	viewray[0] = target.x() - position.x();
	viewray[1] = target.y() - position.y();
	viewray[2] = target.z() - position.z();
	
	if (fabs(viewray[0]) < EPS) viewray[0] = 0;
	if (fabs(viewray[1]) < EPS) viewray[1] = 0;
	if (fabs(viewray[2]) < EPS) viewray[2] = 0;
	
	double len = sqrt (viewray[0]*viewray[0] + viewray[1]*viewray[1] + viewray[2]*viewray[2]);
	assert( len != 0);      
   
	viewray[0] /= len;
  	viewray[1] /= len;
  	viewray[2] /= len;
  	
	// Determine right axis
	if ((viewray[0] == 0) && (viewray[1] == 0))
		vtkMath::Cross (viewray, x_axis, right);
	else
		vtkMath::Cross (viewray, z_axis, right);
		
	if (fabs(right[0]) < EPS) right[0] = 0;
	if (fabs(right[1]) < EPS) right[1] = 0;
	if (fabs(right[2]) < EPS) right[2] = 0;

	// Determine up axis
	vtkMath::Cross (viewray, right, up);
	if (fabs(up[0]) < EPS) up[0] = 0;
	if (fabs(up[1]) < EPS) up[1] = 0;
	if (fabs(up[2]) < EPS) up[2] = 0;
	
	// normalize the axes if we need to save in camera coordinates 
	if (!sp.object_coordinates)
    {
      // Normalization
      double right_len = sqrt (right[0]*right[0] + right[1]*right[1] + right[2]*right[2]);
      right[0] /= right_len;
      right[1] /= right_len;
      right[2] /= right_len;
      double up_len = sqrt (up[0]*up[0] + up[1]*up[1] + up[2]*up[2]);
      up[0] /= up_len;
      up[1] /= up_len;
      up[2] /= up_len;

    }

    // right = viewray x up
    vtkMath::Cross (viewray, up, right);
    // IN FACT: right is left and up is down!
    
	// Output resulting vectors
	std::cout << "Viewray Right Up:" << std::endl;
	std::cout << viewray[0] << " " << viewray[1] << " " << viewray[2] << " " << std::endl;
	std::cout << right[0] << " " << right[1] << " " << right[2] << " " << std::endl;
	std::cout << up[0] << " " << up[1] << " " << up[2] << " " << std::endl;
	std::cout << std::endl;

	std::cout << "CORRECT: Viewray Right Up:" << std::endl;
	Eigen::Matrix<double,3,3> wRs = misc::quat2rot( orientation_ );
	std::cout << (wRs * Eigen::Vector3d(1,0,0)).transpose() << std::endl;
	std::cout << (wRs * Eigen::Vector3d(0,-1,0)).transpose() << std::endl;
	std::cout << (wRs * Eigen::Vector3d(0,0,1)).transpose() << std::endl;
	std::cout << std::endl;

	std::cout << "POSE:" << std::endl;
	std::cout << position_.transpose() << std::endl;
	std::cout << position.transpose() << std::endl;
	std::cout << orientation_.transpose() << std::endl;
	std::cout << misc::quat2angle( orientation_ ) << std::endl;
    
   // Prepare the point cloud data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	
	
   double temp_beam[3], beam[3], p[3];
  	double p_coords[3], x[3], t;
  	int subId;
  	
	 // Create a transformation
    vtkGeneralTransform* tr1 = vtkGeneralTransform::New ();
    vtkGeneralTransform* tr2 = vtkGeneralTransform::New ();
    
    // Sweep vertically    
    for (double vert = sp.vert_start; vert <= sp.vert_end; vert += sp.vert_res)
    {

      tr1->Identity ();
      tr1->RotateWXYZ (vert, right);
      tr1->InternalTransformPoint (viewray, temp_beam);

      // Sweep horizontally
      for (double hor = sp.hor_start; hor <= sp.hor_end; hor += sp.hor_res)
      {
      
        // Create a beam vector with (lat,long) angles (vert, hor) with the viewray
        tr2->Identity ();
        tr2->RotateWXYZ (hor, up);
        tr2->InternalTransformPoint (temp_beam, beam);
        vtkMath::Normalize (beam);

        // Find point at max range: p = eye + beam * max_dist
        for (int d = 0; d < 3; d++)
				p[d] = eye[d] + beam[d] * sp.max_dist;

        // Put p_coords into laser scan at packetid = vert, scan id = hor
        //std::cout <<"eye = "<< eye[0] << " " << eye[1] << " " << eye[2] << std::endl;
        //std::cout <<"p = " << p[0] << " " << p[1] << " " << p[2] << std::endl;
        //std::cout <<"x = " << x[0] << " " << x[1] << " " << x[2] << std::endl;
        //std::cout <<"t = " << t << ", subId = " << subId << std::endl;


		  // Determine if the ray between eye (camera position) and p (max range)
		  // intersects with the tree give a tolerance of 0
		  // return the intersection coordinates in x (i.e. x is the intersection point in the WORLD frame)
		  // return the cell which was intersected by the ray in cellId
        vtkIdType cellId;
        if (tree->IntersectWithLine (eye, p, 0, t, x, p_coords, subId, cellId))
        { 
			 // x are the coordinates in the world frame
          pcl::PointXYZ pt;
          if (sp.object_coordinates)
          {
            pt.x = static_cast<float> (x[0]); 
            pt.y = static_cast<float> (x[1]); 
            pt.z = static_cast<float> (x[2]);
            //pt.vp_x = static_cast<float> (eye[0]); 
            //pt.vp_y = static_cast<float> (eye[1]); 
            //pt.vp_z = static_cast<float> (eye[2]);
          }
          else
          {
				// Translate the origin to the sensor position
				x[0] -= eye[0];
				x[1] -= eye[1];
				x[2] -= eye[2];

				// Determine sRw
				// In world frame:
				// x is forward, y is left, z is up
				//
				// In sensor frame:
				// right = viewray x up
				//
				// x_axis is right (red) : [1 0 0] = sRw * (right)
				// y_axis is down (green) : [0 1 0] = sRw * (-up)
				//	z_axis is forward (blue) : [0 0 1] = sRw * viewray 
				//
				// Therefore: sRw * [right | -up | viewray] = Identity 
				//	==> wRs = [right | -up | viewray]
				// ==> sRw = wRs^T = [ right; -up; viewray ]

				pt.x = static_cast<float> ( viewray[0]*x[0] + viewray[1]*x[1] + viewray[2]*x[2] );
				pt.y = static_cast<float> ( -right[0]*x[0] - right[1]*x[1] - right[2]*x[2] );
				pt.z = static_cast<float> ( up[0]*x[0] + up[1]*x[1] + up[2]*x[2] );

				//pt.x = static_cast<float> ( -right[0]*x[0] - right[1]*x[1] - right[2]*x[2] );
				//pt.y = static_cast<float> ( up[0]*x[0] + up[1]*x[1] + up[2]*x[2] );
				//pt.z = static_cast<float> ( viewray[0]*x[0] + viewray[1]*x[1] + viewray[2]*x[2] );


				//pt.x = static_cast<float> (-right[0]*x[1] + up[0]*x[2] + viewray[0]*x[0] );
            //pt.y = static_cast<float> (-right[1]*x[1] + up[1]*x[2] + viewray[1]*x[0] );
            //pt.z = static_cast<float> (-right[2]*x[1] + up[2]*x[2] + viewray[2]*x[0] );
 				//pt.vp_x = pt.vp_y = pt.vp_z = 0.0f;


				// Convert from world to sensor coordinates!
				// sRw * [1 0 0] = viewray		// X axis is forward
				// sRw * [0 1 0] = -right		// Y axis is left
				// sRw * [0 0 1] = up			// Z axis is up
				// 
				// sRw = [viewray | -right | up]; 
				// sRw_optical = [right | -up | viewray];
				// Let p_w be the position of the sensor in the world frame
				// x_s = sRw * (x_w - p_w)

				// pt.Z IS FORWARD ( BLUE )
				// pt.X IS RIGHT ( RED )
				// pt.Y IS DOWN ( GREEN )

            //pt.x = static_cast<float> (right[0]*x[0] - up[0]*x[1] + viewray[0]*x[2] );
            //pt.y = static_cast<float> (right[1]*x[0] - up[1]*x[1] + viewray[1]*x[2] );
            //pt.z = static_cast<float> (right[2]*x[0] - up[2]*x[1] + viewray[2]*x[2] );
            // z axis is the viewray
            // y axis is up	(ROS_OPTICAL: down)
            // x axis is -right (negative because z*y=-x but viewray*up=right)	(ROS_OPTICAL: right)
				// sensor_T_optical = (r,p,y) = (-90, 0, -90);
				

				
				// A rotation matrix [a|b|c] = [a1 b1 c1; a2 b2 c2; a3 b3 c3] has determinant 1.
				// Therefore the inverse is:
			   // [  (b2*c3 - b3*c2), -(b1*c3 - b3*c1),  (b1*c2 - b2*c1)] = [b x c]
			   // [ -(a2*c3 - a3*c2),  (a1*c3 - a3*c1), -(a1*c2 - a2*c1)] = [-a x c] 
			   // [  (a2*b3 - a3*b2), -(a1*b3 - a3*b1),  (a1*b2 - a2*b1)] = [a x b]
				//
				
				// sRw = [-up x viewray; - right x viewray; right x -up] = 
				//		 = [viewray x up; viewray x right; up x right] = [right; 


				// Given [u v w] in world frame we get sRw * [u v w] in sensor frame, i.e.:
				// 
				// sRw * [x y z] = x * sRw * [1 0 0]
				// right = 
				// We want [1 0 0] to become [0 0 1] i.e. viewray, to become [0 0 1]: [0 0 1] = sRw * viewray
				
				// viewray(z_s) is forward, right (-y_s) is right, and up (z_s) is up, i.e.:
				// [x_s, y_s, z_s] = sRw * [x_w, y_w, z_w] = x_w * sRw * [1 0 0] + y_w * sRw * [0 1 0] 
				// z_s = viewray = sRw * [1 0 0], y_s = -right = sRw * [0 1 0], z_s = up = sRw * [0 0 1]
				
				// z is up

           
          }
          
          //std::cout << "Pushing a point..." << std::endl; 
          cloud_ptr->points.push_back (pt);
        }
        else
          if (sp.organized)
          {
            pcl::PointXYZ pt;
            pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN ();
            //pt.vp_x = static_cast<float> (eye[0]);
            //pt.vp_y = static_cast<float> (eye[1]);
            //pt.vp_z = static_cast<float> (eye[2]);
            
            //std::cout << "Pushing a point..." << std::endl;
            cloud_ptr->points.push_back (pt);
          }
      } // Horizontal
    } // Vertical
    
    // Add noise
    if(sp.add_noise)
    {
    	if (sp.object_coordinates)
    		addNoise(position, cloud_ptr, gaussian_rng);
    	else
    		addNoise(Eigen::Vector3d(0,0,0), cloud_ptr, gaussian_rng);
    }
    // Downsample and remove silly point duplicates
	 //pcl::VoxelGrid<pcl::PointWithViewpoint> grid;
	 //grid.setLeafSize (2.5, 2.5, 2.5);    // @note: this value should be given in mm!
    //pcl::PointCloud<pcl::PointWithViewpoint> cloud_downsampled;
    //grid.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointWithViewpoint> > (cloud));
    //grid.filter (cloud_downsampled);
    
	if (sp.organized)
	{
		cloud_ptr->height = 1 + static_cast<uint32_t> ((sp.vert_end - sp.vert_start) / sp.vert_res);
		cloud_ptr->width = 1 + static_cast<uint32_t> ((sp.hor_end - sp.hor_start) / sp.hor_res);
	}
	else
	{
		cloud_ptr->width = static_cast<uint32_t> (cloud_ptr->points.size ());
		cloud_ptr->height = 1;
	}
	
	return cloud_ptr;
}
*/
