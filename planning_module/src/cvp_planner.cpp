#include "planning_module/cvp_planner.hpp"

#include <algorithm> // sort()

void
cvp_planner::get_CVP_sequence( double eps, int curr_vp,
									  boost::multi_array<double,2> const& cMap,
									  std::vector<double>::iterator curr_bel_start, 
								  	  std::vector<double>::iterator curr_bel_end,
								  	  std::vector<int> & vp_seq )
{
	int num_vp = cMap.shape()[1];
	
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
	
	// rank the rest of the actions according to closeness
	std::vector< std::pair<double, int> > dist_vp_vec;
	for( int vp = 0; vp < num_vp; ++vp)
		dist_vp_vec.push_back( std::make_pair( cMap[curr_vp][vp], vp ) );

	// sort in ascending order
	std::sort ( dist_vp_vec.begin(), dist_vp_vec.end() );

	for( int vp = 0; vp < num_vp; ++vp)
	{
		vp_seq.push_back( dist_vp_vec[vp].second );
	}
}

