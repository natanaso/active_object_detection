
#include <stdlib.h>	// rand()
#include <algorithm> // sort()

#include <misc.hpp>

#include "planning_module/voi_planner.hpp"

int
voi_planner::getEJSAction( double eps, boost::multi_array<double,3> const& oMap,
						  std::vector<double>::iterator curr_bel_start, 
					  	  std::vector<double>::iterator curr_bel_end )
{
	int num_vp = oMap.shape()[1] - 1;
	
	
	// Check if we can make a decision
	{
	int hid = 0;
	for( std::vector<double>::iterator it = curr_bel_start;
		it != curr_bel_end; ++it, ++hid)
		if( *it >= 1 - eps)
			return (num_vp + hid);
	}
	
	
	int best_act = -1;
	double max_EJS = 0.0;
	for( int vp = 0; vp < num_vp; ++vp)
	{
		double ejs = EJS( vp, oMap, curr_bel_start, curr_bel_end);
		if( ejs >= max_EJS )
		{
			max_EJS = ejs;
			best_act = vp;
		}
	}
	
	return best_act;
}

void
voi_planner::get_EJS_sequence( double eps,
									  const boost::multi_array<double,3> & oMap,
									  std::vector<double>::iterator curr_bel_start, 
								  	  std::vector<double>::iterator curr_bel_end,
								  	  std::vector<int> & vp_seq )
{
	int num_vp = oMap.shape()[1] - 1;
	
	
	// Check if we can make a decision
	{
		int hid = 0;
		for( std::vector<double>::iterator it = curr_bel_start;
			it != curr_bel_end; ++it, ++hid)
			if( *it >= 1 - eps)
			{
				vp_seq.push_back(num_vp + hid);
				break;
			}
	}
	
	
	// rank the rest of the actions according to EJS
	std::vector< std::pair<double, int> > ejs_vp_vec;
	for( int vp = 0; vp < num_vp; ++vp)
		ejs_vp_vec.push_back( std::make_pair( EJS( vp, oMap, curr_bel_start, curr_bel_end), vp ) ); 

	// sort in descending EJS order
	std::sort ( ejs_vp_vec.rbegin(), ejs_vp_vec.rend() );

	for( int vp = 0; vp < num_vp; ++vp)
	{
		vp_seq.push_back( ejs_vp_vec[vp].second );
	}
}

					  	  
double
voi_planner::EJS( int vp, boost::multi_array<double,3> const& oMap,
				 std::vector<double>::iterator curr_bel_start, 
				 std::vector<double>::iterator curr_bel_end )
{
	int num_obs = oMap.shape()[0];
	
	typedef boost::multi_array_types::index_range range;
	double ejs = 0.0;
	int hid1 = 0;
	for( std::vector<double>::iterator it1 = curr_bel_start;
		it1 != curr_bel_end; ++it1, ++hid1)
	{

		boost::multi_array<double,1> obs_vec_sum(boost::extents[num_obs]);
		std::fill( obs_vec_sum.data(), 
				   obs_vec_sum.data() + obs_vec_sum.num_elements(), 0.0);
		
		int hid2 = 0;
		for( std::vector<double>::iterator it2 = curr_bel_start;
			it2 != curr_bel_end; ++it2, ++hid2)
			if( it2 != it1 )
			{
				for(int obs = 0; obs < num_obs; ++obs)
					obs_vec_sum[obs] += ((*it2) / (1 - (*it1)) ) * oMap[obs][vp][hid2];
			}
		
		boost::multi_array<double,3>::const_array_view<1>::type obs_vec = 
			oMap[ boost::indices[range()][vp][hid1] ];
		
		ejs += (*it1) * kl_div( obs_vec.begin(), obs_vec.end(),
							   obs_vec_sum.begin(), obs_vec_sum.end() );
	}
	
	return ejs;
}


double
voi_planner::kl_div( boost::multi_array<double,1>::const_iterator p_start,
					boost::multi_array<double,1>::const_iterator p_end,
					boost::multi_array<double,1>::const_iterator q_start,
					boost::multi_array<double,1>::const_iterator q_end )
{
	double kldiv = 0.0;
	boost::multi_array<double,1>::const_iterator it1 = p_start;
	boost::multi_array<double,1>::const_iterator it2 = q_start;
	while( ( it1 != p_end ) && ( it2 != q_end) )
	{
		kldiv += (*it1) * (log( (*it1) / (*it2) ) / log( 2 ));
		++it1;
		++it2;
	}
	return kldiv;
}

					
double
voi_planner::kl_div( std::vector<double>::iterator p_start,
					std::vector<double>::iterator p_end,
					std::vector<double>::iterator q_start,
					std::vector<double>::iterator q_end )
{
	double kldiv = 0.0;
	std::vector<double>::iterator it1 = p_start;
	std::vector<double>::iterator it2 = q_start;
	while( ( it1 != p_end ) && ( it2 != q_end) )
	{
		kldiv += (*it1) * (log( (*it1) / (*it2) ) / log( 2 ));
		++it1;
		++it2;
	}
	return kldiv;
}

/*
int voi_planner_main(int argc, char **argv)
{
	boost::multi_array<double,3> oMap( boost::extents[2][3][2] );
	int num_obs = oMap.shape()[0];
	int num_sta = oMap.shape()[1];
	int num_vp = num_sta - 1;
	int num_hid = oMap.shape()[2];
	int num_act = num_vp + num_hid;
	
	// Initialize
	srand((unsigned)time(NULL));
	for( int obs = 0; obs < num_obs; ++obs)
		for( int sta = 0; sta < num_sta; ++sta)
			for( int hid = 0; hid < num_hid; ++hid)
			{
				oMap[obs][sta][hid] = misc::uniform_int( 1, 50 );
			}
	
	// compute the denominators
	typedef boost::multi_array_types::index_range range;
	boost::multi_array<double,2> denum(boost::extents[num_sta][num_hid]);
	for(int hid = 0; hid < num_hid; ++hid)
		for (int sta = 0; sta < num_sta; ++sta)
		{
			// find the sum of the observations
			boost::multi_array<double,3>::array_view<1>::type obs_vec = 
					oMap[ boost::indices[range()][sta][hid] ];
		
			denum[sta][hid] = std::accumulate(obs_vec.begin(), obs_vec.end(), 0.0);
		}
	
	// normalize
	for (int obs = 0; obs < num_obs; ++obs)
		for (int sta = 0; sta < num_sta; ++sta)
			for(int hid = 0; hid < num_hid; ++hid)
				oMap[obs][sta][hid] = oMap[obs][sta][hid] / denum[sta][hid];
	
	// display
	for(int hid = 0; hid < num_hid; ++hid)
	{
		for (int sta = 0; sta < num_sta; ++sta)
		{
			for (int obs = 0; obs < num_obs; ++obs)
				std::cout << oMap[obs][sta][hid] << " ";

			std::cout << std::endl;
		}
		std::cout << std::endl << std::endl << std::endl;
	}
	
		
	const double arr[] = {0.5, 0.5};
	std::vector<double> bel;
	bel.assign(arr, arr + sizeof(arr) / sizeof(arr[0]));
	
	double eps = 0.05;
	int best_act = voi_planner::getEJSAction( eps, oMap, bel.begin(), bel.end() );
	
	return 0;
}


int main( int argc, char **argv)
{
	return voi_planner_main( argc, argv );
}
*/

