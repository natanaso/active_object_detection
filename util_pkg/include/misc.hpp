#ifndef _MISC_HPP_
#define _MISC_HPP_

#include <vector>
#include <Eigen/Core>
#include <boost/multi_array.hpp>
#include <stdlib.h>	// rand()

// Template Implementation Includes
#include <boost/bind.hpp>
#include <limits>
#include <cmath>


// XXX: assume w (4th component) is the magnitude!
class misc
{
	private:
		
	public:
		misc(){}
		~misc(){}
		///////////////////////////////////////////////////////
		// Typedefs
		///////////////////////////////////////////////////////
		typedef boost::multi_array<double,2> array2D;
		typedef boost::multi_array<double,3> array3D;
		//typedef array3D::index index3D;
		typedef boost::multi_array<int,2> array2I;
		typedef boost::multi_array<int,3> array3I;
    	//typedef array3I::index index3I;
		

		// call the following once:
		// srand((unsigned)time(NULL));
		// before calling a loop of this:
		// Returns a random integer in [a, b] 
		static int uniform_int(int a, int b){
			return rand() % (b - a + 1) + a;
		}
		
		// call the following once:
		// srand((unsigned)time(NULL));
		// before calling a loop of this:
		// Returns a random double in [a, b] 
		static double uniform_cont(double a, double b){
			return a + (b-a) * ((double)rand()/(double)RAND_MAX);
		}
		
		template <typename T> static T round_down(T x){
			return ceil(x - static_cast<T>(0.5));	
		}
		
		static double deg2rad(double deg){
			return (M_PI / 180.0) * deg;
		}
	
		static double rad2deg(double rad){
			return (180.0 / M_PI) * rad;
		}

		// Rotation matrices		
		static Eigen::Matrix<double,3,3> rotx(double roll);
		static Eigen::Matrix<double,3,3> roty(double pitch);
		static Eigen::Matrix<double,3,3> rotz(double yaw);

		// XXX: ASSUMES THE SCALAR IS THE LAST COMPONENT!
		static Eigen::Matrix<double,3,3> quat2rot( Eigen::Vector4d q );
		
		template <typename Derived>
		static bool is_finite(const Eigen::MatrixBase<Derived>& x)
		{
		   return ( (x - x).array() == (x - x).array()).all();
		}


		template <typename Derived>
		static bool is_nan(const Eigen::MatrixBase<Derived>& x)
		{
		   return ((x.array() == x.array())).all();
		}
			
		// sgn function
		template <typename T> static int sgn(T val) {
    		return (T(0) < val) - (val < T(0));
		}				  
		//========= Signum Function ====================//
		// Requires: -std=c++0x
		// #include <type_traits>
		/*
		template <typename T> inline constexpr
		int signum(T x, std::false_type is_signed) {
    		return T(0) < x;
		}

		template <typename T> inline constexpr
		int signum(T x, std::true_type is_signed) {
    		return (T(0) < x) - (x < T(0));
		}

		template <typename T> inline constexpr
		int signum(T x) {
    		return signum(x, std::is_signed<T>());
		}
		*/
		//==============================================//
		
    	template <typename T> static T quatmod(Eigen::Matrix<T,4,1>  const & quat){
			return sqrt(quatnorm(quat));
		}
		
		template <typename T> static T quatnorm(Eigen::Matrix<T,4,1> const & quat){
			return (pow(quat.w(),2) + pow(quat.x(),2) + pow(quat.y(),2) + pow(quat.z(),2));
		}

		///////////////////////////////////////////////////////
		// Function Declarations
		///////////////////////////////////////////////////////    	
		// Assumes ZYX rotation
		template <typename T> static Eigen::Matrix<T,4,1> angle2quat(Eigen::Matrix<T,3,1> const & angles);	// yaw pitch roll
		
		// Assumes ZYX rotation
		template <typename T> static Eigen::Matrix<T,3,1> quat2angle(Eigen::Matrix<T,4,1> quat);
		
		
		template <typename T> static Eigen::Matrix<T,4,1> quatnormalize(Eigen::Matrix<T,4,1> quat);
		
		// [x,y,z] = sph2cart(az,elev,r)
		template <typename T> static Eigen::Matrix<T,3,1> sph2cart(Eigen::Matrix<T,3,1> az_elev_rad);
		
		// [az,elev,r] = cart2sph(x,y,z)
		template <typename T> static Eigen::Matrix<T,3,1> cart2sph(Eigen::Matrix<T,3,1> x_y_z);
		
		// Takes a quaternion with zero roll  and returns a target point on the z = 0 plane
		template <typename T> static Eigen::Matrix<T,3,1> quat2target(Eigen::Matrix<T,3,1> const & position, Eigen::Matrix<T,4,1> const & orientation);
		template <typename T> static Eigen::Matrix<T,4,1> target2quat(Eigen::Matrix<T,3,1> const & position, Eigen::Matrix<T,3,1> const & target);
		
		// compute the minimum distance for each of the query points to the base points		
		template <typename D1, typename D2, typename D3, typename D4>
		static void NNsearch_3D( Eigen::MatrixBase<D1> const & base,
								  Eigen::MatrixBase<D2> const & query,
								  Eigen::MatrixBase<D3> const & match,
								  Eigen::MatrixBase<D4> const & mindist);

		template <typename D1, typename D2>
		static void meters2cells( Eigen::MatrixBase<D1> const & datam,
								   Eigen::MatrixXi const & datac,
								   Eigen::MatrixBase<D2> const & dim_min,
								   Eigen::MatrixBase<D2> const & res );
		
						   		
		template <typename D1, typename D2>
		static void cells2meters( Eigen::MatrixXi const & datac,
								   Eigen::MatrixBase<D1> const & datam,
								   Eigen::MatrixBase<D2> const & dim_min,
								   Eigen::MatrixBase<D2> const & res );		
		
		template <typename D>
		static void dim_min_max( Eigen::MatrixBase<D> const & devmin,
								  Eigen::MatrixBase<D> const & devmax,
								  Eigen::MatrixBase<D> const & res,
								  Eigen::MatrixBase<D> const & dim_min,
								  Eigen::MatrixBase<D> const & dim_max );

		// returns the first odd integer bigger than x
		template <typename D1, typename D2>
		static void odd_ceil( Eigen::MatrixBase<D1> const & x, Eigen::MatrixBase<D2> const & ocx);
		 
		static void update_belief( std::vector<double> const & prior,
							int vp,
							std::vector<int> const & vp_hist,
							Eigen::VectorXf const & scores,
							Eigen::MatrixXf const & views,
							array3D const & oMap,			//const double *oMap,
							std::vector<double> & posterior );
		
		template <typename Derived>
		static double get_alpha(int vp, std::vector<int> const & vp_hist, Eigen::MatrixBase<Derived> const & views);
		
		template <typename Derived, typename T>
		static int find_closest_viewpoint( Eigen::Matrix<T,3,1> const & centroid, 
									Eigen::MatrixBase<Derived> const & views,
									Eigen::Matrix<T,3,1> const & position, 
									Eigen::Matrix<T,3,1> & vp_position,
									Eigen::Matrix<T,4,1> & vp_orientation );
		
		static double great_circle_dist(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d cntr);
		
		static double SO3_metric( const Eigen::Vector4d & q1, const Eigen::Vector4d & q2 );
};



///////////////////////////////////////////////////////////////////////////////////
// Template Functions implementations
//////////////////////////////////////////////////////////////////////////////////

// Works for both double and float
template <typename T> 
Eigen::Matrix<T,4,1>
misc::angle2quat(Eigen::Matrix<T,3,1>  const & angles){
	T cyaw = cos(angles.x()/2);
	T cpitch = cos(angles.y()/2);
	T croll = cos(angles.z()/2);
	
	T syaw = sin(angles.x()/2);
	T spitch = sin(angles.y()/2);
	T sroll = sin(angles.z()/2);	
	Eigen::Matrix<T,4,1> quat(
		cyaw*cpitch*sroll - syaw*spitch*croll,
		cyaw*spitch*croll + syaw*cpitch*sroll,
		syaw*cpitch*croll - cyaw*spitch*sroll,
		cyaw*cpitch*croll + syaw*spitch*sroll);

	return quat;
}


template <typename T> 
Eigen::Matrix<T,3,1>
misc::quat2angle(Eigen::Matrix<T,4,1> quat)
{
	quat = quatnormalize(quat);
	T r11 = 2*(quat.x() * quat.y() + quat.w() * quat.z());
	T r12 = pow(quat.w(),2) + pow(quat.x(),2) - pow(quat.y(),2) - pow(quat.z(),2);
	T r21 = -2*(quat.x() * quat.z() - quat.w()*quat.y());
	T r31 = 2*(quat.y() * quat.z() + quat.w() * quat.x());
	T r32 = pow(quat.w(),2) + pow(quat.z(),2) - pow(quat.x(),2) - pow(quat.y(),2);

	// truncate r21 if above the allowed range
	if(abs(r21) >= static_cast<T>(1.0))
		r21 = sgn(r21) * static_cast<T>(1.0);
	
	Eigen::Matrix<T,3,1> yaw_pitch_roll( atan2( r11, r12 ), asin( r21 ), atan2( r31, r32 ));
	
	return yaw_pitch_roll;
}

template <typename T>
Eigen::Matrix<T,4,1>
misc::quatnormalize(Eigen::Matrix<T,4,1> quat){
	T mod = quatmod(quat);
	quat.w() = quat.w()/mod;
	quat.x() = quat.x()/mod;
	quat.y() = quat.y()/mod;
	quat.z() = quat.z()/mod; 
	return quat;
}

template <typename T>
Eigen::Matrix<T,3,1>
misc::sph2cart(Eigen::Matrix<T,3,1> az_elev_rad){
	Eigen::Matrix<T,3,1> x_y_z (
		az_elev_rad.z() * cos(az_elev_rad.y()) * cos(az_elev_rad.x()),
		az_elev_rad.z() * cos(az_elev_rad.y()) * sin(az_elev_rad.x()),
		az_elev_rad.z() * sin(az_elev_rad.y()));
	return x_y_z;
}

template <typename T>
Eigen::Matrix<T,3,1>
misc::cart2sph(Eigen::Matrix<T,3,1> x_y_z){
	Eigen::Vector3d az_elev_rad (
		atan2( x_y_z.y(), x_y_z.x()),
		atan2( x_y_z.z(), hypot(x_y_z.x(),x_y_z.y()) ),
		hypot(hypot(x_y_z.x(),x_y_z.y()), x_y_z.z()));
	return az_elev_rad;
}


template <typename T>
Eigen::Matrix<T,3,1>
misc::quat2target(Eigen::Matrix<T,3,1> const & position, Eigen::Matrix<T,4,1> const & orientation)
{
	// Convert orientation quat to euler angles
	Eigen::Matrix<T,3,1> yaw_pitch_roll = quat2angle(orientation);
	
	// azimuth = yaw; elevation = pitch;
	// Set the radius so that we cross the z = 0 plane:
	yaw_pitch_roll.z() = position.z() / sin(yaw_pitch_roll.y());
	
	
	// set the radius to 1 if we never cross the z = 0 plane!
	if(isnan(yaw_pitch_roll.z()) || isinf(yaw_pitch_roll.z()) || (yaw_pitch_roll.z() <= 0))
		yaw_pitch_roll.z() = 1;
		
	
	// pitch goes in the opposite direction from the spherical coordinate angel when the coordinate system is:
	// x = front, y = left, z = up
	// az = yaw; elev = -pitch;
	yaw_pitch_roll.y() = -yaw_pitch_roll.y();
	
	// get cartesian coordinates
	yaw_pitch_roll = sph2cart(yaw_pitch_roll);
	
	// Get the target point
	Eigen::Matrix<T,3,1> target = yaw_pitch_roll + position;
	
	return target;
}

template <typename T>
Eigen::Matrix<T,4,1>
misc::target2quat(Eigen::Matrix<T,3,1> const & position, Eigen::Matrix<T,3,1> const & target)
{
	// Get vector pointing towards the target
	Eigen::Matrix<T,3,1> v = target - position;
	
	// Get az_elev_rad
	v = cart2sph(v);
	
	// yaw = az; pitch = -elev; roll = 0;
	v.y() = -v.y();
	v.z() = 0;
	
	//std::cout << "ypr =" << v << std::endl;
	
	// Conver yaw_pitch_roll to quaternion
	Eigen::Matrix<T,4,1> quat = angle2quat(v);
	return quat;
}


// compute the minimum distance for each of the query points to the base points
template <typename D1, typename D2, typename D3, typename D4>
void
misc::NNsearch_3D(Eigen::MatrixBase<D1> const & base,
				  Eigen::MatrixBase<D2> const & query,
				  Eigen::MatrixBase<D3> const & match,
				  Eigen::MatrixBase<D4> const & mindist)
{

	//typedef typename D1::Scalar Scalar;
	//typedef typename Eigen::internal::plain_row_type<D2>::type RowVectorType;
	//typedef typename Eigen::internal::plain_col_type<D2>::type ColVectorType;
	
	//const int num_bpts = static_cast<int>(base.rows());
	const int num_qpts = static_cast<int>(query.rows());

	//std::cout << "num_bpts = " << num_bpts << std::endl;
	//std::cout << "num_qpts = " << num_qpts << std::endl;


	// Cast away the constantness of the outputs
	Eigen::MatrixBase<D3>& match_ = const_cast< Eigen::MatrixBase<D3>& >(match);
	Eigen::MatrixBase<D4>& mindist_ = const_cast< Eigen::MatrixBase<D4>& >(mindist);

	// Populate the output:
	// For each query point find the minimum distance to the base points 
	// and record the matched point
	for(int i = 0; i < num_qpts; ++i){
		mindist_(i) = (base.rowwise() - query.row(i)).rowwise().norm().minCoeff(&match_(i));
	}	
	
	/*
	Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic> xDiff(num_bpts,num_qpts), 
			yDiff(num_bpts,num_qpts), zDiff(num_bpts,num_qpts), dMat(num_bpts,num_qpts);
			
	// compute differences
	for(int i = 0; i < num_bpts; ++i){
		xDiff.row(i) = (query.col(0).array() - base(i,0)).matrix().transpose();
		yDiff.row(i) = (query.col(1).array() - base(i,1)).matrix().transpose();
		zDiff.row(i) = (query.col(2).array() - base(i,2)).matrix().transpose();		
	}

	dMat = (xDiff.array().square() + yDiff.array().square() + zDiff.array().square()).matrix();
	
	mindist_ = dMat.colwise().minCoeff().cwiseSqrt().transpose();
	*/

}


template <typename D1, typename D2>
void
misc::meters2cells( Eigen::MatrixBase<D1> const & datam,
				    Eigen::MatrixXi const & datac,
				    Eigen::MatrixBase<D2> const & dim_min,
				    Eigen::MatrixBase<D2> const & res )
{	
	// Define the type of the scalar that is used
	typedef typename D1::Scalar Scalar;
	
	// cast away the constantness of the output
	Eigen::MatrixXi& datac_ = const_cast< Eigen::MatrixXi& >(datac);
	
	// datam = [num_dim x num_pts]
	const int num_pts = static_cast<int>(datam.cols());
	
	Eigen::Array<Scalar,Eigen::Dynamic,1> temp(datam.rows());
		
	for(int i = 0; i < num_pts; ++i){
		temp = (datam.col(i) - dim_min.template cast<Scalar>()).array() / (res.template cast<Scalar>()).array();
		datac_.col(i) = temp.matrix().unaryExpr(std::ptr_fun(ceil)).template cast<int>();
	}
	
	// convert zeros to ones and subtract 1 to ensure indexing starts at 0
	datac_ += ((datac_.array() == 0).template cast<int>() - 1).matrix();
	
}

template <typename D1, typename D2>
void 
misc::cells2meters( Eigen::MatrixXi const & datac,
				    Eigen::MatrixBase<D1> const & datam,
				    Eigen::MatrixBase<D2> const & dim_min,
				    Eigen::MatrixBase<D2> const & res )
{
	// Define the type of the scalar that is used
	typedef typename D1::Scalar Scalar;
	
	// cast away the constantness of the output
	Eigen::MatrixBase<D1>& datam_ = const_cast< Eigen::MatrixBase<D1>& >(datam);
	
	// datam = [num_dim x num_pts]
	const int num_pts = static_cast<int>(datac.cols());
	
	Eigen::Array<Scalar,Eigen::Dynamic,1> temp(datac.rows());
	
	for(int i = 0; i < num_pts; ++i){
		temp = (datac.col(i).array().template cast<Scalar>() + 0.5) * (res.array().template cast<Scalar>());
		datam_.col(i) = (temp + (dim_min.array().template cast<Scalar>())).matrix();
	}	
}

template <typename D>
void 
misc::dim_min_max( Eigen::MatrixBase<D> const & devmin,
				   Eigen::MatrixBase<D> const & devmax,
				   Eigen::MatrixBase<D> const & res,
				   Eigen::MatrixBase<D> const & dim_min,
				   Eigen::MatrixBase<D> const & dim_max )
{
	typedef typename D::Scalar Scalar;
	
	// cast away the constantness of the outputs
	Eigen::MatrixBase<D>& dim_min_ = const_cast< Eigen::MatrixBase<D>& >(dim_min);
	Eigen::MatrixBase<D>& dim_max_ = const_cast< Eigen::MatrixBase<D>& >(dim_max);

	// Get the number of half pixels
	Eigen::Matrix<int,Eigen::Dynamic,Eigen::Dynamic> num_half_pix_min(devmin.rows(),devmin.cols());
	
	odd_ceil( ((2*devmin.cwiseAbs()).array() / res.array()).matrix(), num_half_pix_min );
	
	Eigen::Matrix<int,Eigen::Dynamic,Eigen::Dynamic> num_half_pix_max(devmax.rows(),devmax.cols());
	
	odd_ceil( ((2*devmax).array() / res.array()).matrix(), num_half_pix_max );
	
	dim_min_ = (-num_half_pix_min.template cast<Scalar>().array()*res.array()/2).matrix();
	dim_max_ = (num_half_pix_max.template cast<Scalar>().array()*res.array()/2).matrix();

}

template <typename D1, typename D2>
void 
misc::odd_ceil( Eigen::MatrixBase<D1> const & x, Eigen::MatrixBase<D2> const & ocx )
{

	typedef typename D1::Scalar Scalar1;
	typedef typename D2::Scalar Scalar2;
	
	// cast away the constantness of the output
	Eigen::MatrixBase<D2>& ocx_ = const_cast< Eigen::MatrixBase<D2>& >(ocx);
	
	Eigen::Array<Scalar1, Eigen::Dynamic, Eigen::Dynamic> rndmask(x.rows(),x.cols());
	
	rndmask = ((x.unaryExpr(std::ptr_fun(floor)) - x).cwiseAbs().array() <= std::numeric_limits<Scalar1>::epsilon( )).template cast<Scalar1>();
		
	ocx_ = (x.array() * (1 - rndmask) + (x.array()*rndmask).unaryExpr(std::ptr_fun(floor))).unaryExpr(std::ptr_fun(ceil)).matrix().template cast<int>();
	
	
	// Add one to all even elements
	int(*mod_ptr)(int) = [](int x) { return x % 2; };
	
	Eigen::Array<Scalar2, Eigen::Dynamic, Eigen::Dynamic> evenmask(ocx.rows(),ocx.cols());
	evenmask = (ocx_.unaryExpr(mod_ptr).array() == 0).template cast<Scalar2>();
	
	ocx_ = (ocx_.array() * (1 - evenmask) + (1 + ocx_.array())*(evenmask)).matrix();
}
		
							  

template <typename Derived>
double
misc::get_alpha(int vp, std::vector<int> const & vp_hist, Eigen::MatrixBase<Derived> const & views)
{		
	double alpha = 0.0;
	
	if(!vp_hist.empty())
	{
	
		const double dmax = 0.5;
		double dmin = std::numeric_limits<double>::infinity();
	
		double dist;
		for(std::vector<int>::const_iterator it = vp_hist.begin(); it != vp_hist.end(); ++it){
			dist = static_cast<double>((views.block(*it,0,1,2) - views.block(vp,0,1,2)).array().pow(2).sum());
			if(dist < dmin)
				dmin = dist;
		}
		
		std::cout << "dmin = " << dmin << std::endl;
		
		if(dmin < dmax)
			alpha = 1.0 - dmin/dmax;
	}
	
	return alpha;
}


template<typename Derived, typename T>
int
misc::find_closest_viewpoint( Eigen::Matrix<T,3,1> const & centroid, 
							  Eigen::MatrixBase<Derived> const & views,
							  Eigen::Matrix<T,3,1> const & position,
							  Eigen::Matrix<T,3,1> & vp_position,
							  Eigen::Matrix<T,4,1> & vp_orientation )
{	
	// find the radius of the view sphere
	T radius = (position - centroid).norm();
	
	Eigen::Matrix<int,1,1> vp_id;
	Eigen::Matrix<T,1,1> vp_dist;
	
	
	// Center the viewsphere around the centroid and enlarge it by the radius	
	Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic> view_sphere(views.rows(),3);
	
	view_sphere = (radius*(views.template cast<T>())).rowwise() + centroid.transpose();
	
	
	// find the viewpoint position and id:
	NNsearch_3D( view_sphere, position.transpose(), vp_id, vp_dist );
	
	// get the view point position
	vp_position = view_sphere.row(vp_id(0)).transpose();
			  
	// get the orientation when we are looking towards the target point:
	vp_orientation = target2quat(vp_position, centroid);
	
	return vp_id(0);
}
#endif
