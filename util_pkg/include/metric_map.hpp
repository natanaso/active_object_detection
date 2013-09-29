#ifndef METRIC_MAP_H_
#define METRIC_MAP_H_
#include <Eigen/Core>
#include <iostream>
#include <cmath>

class metric_map
{
private:
	Eigen::VectorXd res;
	Eigen::VectorXd dim_min;
	Eigen::VectorXd dim_max;
	
	
public:
	metric_map( Eigen::VectorXd const & devmin,
		 		Eigen::VectorXd const & devmax,
		 		const Eigen::VectorXd resolution )
		: res(resolution.rows()), dim_min(devmin.rows()), dim_max(devmax.rows())
	{
		set_range(devmin, devmax, resolution);	
	}
	
	metric_map( double devmin, double devmax, double resolution )
		: res(1), dim_min(1), dim_max(1)
	{
		set_range(devmin, devmax, resolution);	
	}
	
		 
	~metric_map(){}
	
	void set_res( double res ){
		(this->res).x() = res; 
	}
	
	void set_res( const Eigen::VectorXd res ){
		this->res = res;	
	}
	
	void set_range( double devmin, double devmax, double res)
	{	
		Eigen::VectorXd devmin_mat(1);
		devmin_mat << devmin;
					
		Eigen::VectorXd devmax_mat(1);
		devmax_mat << devmax;

		(this->res).x() = res;
		
		dim_min_max( devmin_mat, devmax_mat, this->res, dim_min, dim_max );			 
	}
	
	
	void set_range( Eigen::VectorXd const & devmin, Eigen::VectorXd const & devmax, const Eigen::VectorXd res )
	{
		this->res = res;
		dim_min_max( devmin, devmax, res, dim_min, dim_max );	 				   
	}
	
	template <typename D1, typename D2>
	void meters2cells( Eigen::MatrixBase<D1> const & datam, Eigen::MatrixBase<D2> const & datac )
	{	
		meters2cells1( datam, datac, dim_min, res );						   
	}
	
	template <typename D1, typename D2>
	void cells2meters( Eigen::MatrixBase<D2> const & datac, Eigen::MatrixBase<D1> const & datam )
	{
		cells2meters<D1, D2, Eigen::VectorXd>( datac, datam, dim_min, res );	 
	}
	
	
	std::string to_str() const
   {
    	std::stringstream ss;
    	ss << std::endl;
		ss << "dim_min = " << std::endl;
		ss << dim_min << std::endl;
		ss << "dim_max = " << std::endl;
		ss << dim_max << std::endl;
		ss << "res = " << std::endl;
		ss << res << std::endl;
		ss << std::endl;
        
   	return ss.str();
   }
        

	
private:
	friend std::ostream & operator<<(std::ostream &os, const metric_map &m)
	{
		return os << m.to_str();
	}

	// Methods
	template <typename D1, typename D2, typename D3>
	void meters2cells1( Eigen::MatrixBase<D1> const & datam,
					   Eigen::MatrixBase<D2> const & datac,
					   Eigen::MatrixBase<D3> const & dim_min,
					   Eigen::MatrixBase<D3> const & res );
	
	template <typename D1, typename D2, typename D3>
	void cells2meters( Eigen::MatrixBase<D2> const & datac,
				   Eigen::MatrixBase<D1> const & datam,
				   Eigen::MatrixBase<D3> const & dim_min,
				   Eigen::MatrixBase<D3> const & res );		   
	
	template <typename D>
	void dim_min_max( Eigen::MatrixBase<D> const & devmin,
				  Eigen::MatrixBase<D> const & devmax,
				  Eigen::MatrixBase<D> const & res,
				  Eigen::MatrixBase<D> const & dim_min,
				  Eigen::MatrixBase<D> const & dim_max );
				  
	// returns the first odd integer bigger than x
	template <typename D1, typename D2>
	void odd_ceil( Eigen::MatrixBase<D1> const & x, Eigen::MatrixBase<D2> const & ocx);
};







//*********************************************
//*** IMPLEMENTATIONS *************************
//*********************************************

template <typename D1, typename D2, typename D3>
void 
metric_map::meters2cells1( Eigen::MatrixBase<D1> const & datam,
				   Eigen::MatrixBase<D2> const & datac,
				   Eigen::MatrixBase<D3> const & dim_min,
				   Eigen::MatrixBase<D3> const & res )
{	
	// Define the type of the scalar that is used
	typedef typename D1::Scalar Scalar;
	typedef typename D2::Scalar Scalar_int;
	// cast away the constantness of the output
	Eigen::MatrixBase<D2>& datac_ = const_cast< Eigen::MatrixBase<D2>& >(datac);

	// datam = [num_dim x num_pts]
	const int num_pts = static_cast<int>(datam.cols());

	Eigen::Array<Scalar,Eigen::Dynamic,1> temp(datam.rows());

	for(int i = 0; i < num_pts; ++i){
		temp = (datam.col(i) - dim_min.template cast<Scalar>()).array() / (res.template cast<Scalar>()).array();
		datac_.col(i) = temp.matrix().unaryExpr(std::ptr_fun(ceil)).template cast<Scalar_int>();
	}

	// convert zeros to ones and subtract 1 to ensure indexing starts at 0
	datac_ += ((datac_.array() == 0).template cast<Scalar_int>() - 1).matrix();

	//std::cout << datac_ << std::endl;
}
				   		
template <typename D1, typename D2, typename D3>
void 
metric_map::cells2meters( Eigen::MatrixBase<D2> const & datac,
				   Eigen::MatrixBase<D1> const & datam,
				   Eigen::MatrixBase<D3> const & dim_min,
				   Eigen::MatrixBase<D3> const & res )
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
metric_map::dim_min_max( Eigen::MatrixBase<D> const & devmin,
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

// returns the first odd integer bigger than x
template <typename D1, typename D2>
void 
metric_map::odd_ceil( Eigen::MatrixBase<D1> const & x, Eigen::MatrixBase<D2> const & ocx)
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


#endif
