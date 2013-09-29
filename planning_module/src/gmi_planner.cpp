#include "planning_module/gmi_planner.hpp"

#include <algorithm> // sort()

int
gmi_planner::get_GMI_action( double eps, int curr_vp,
						  boost::multi_array<double,3> const& oMap,
						  boost::multi_array<double,2> const& cMap,
						  std::vector<double>::iterator curr_bel_start, 
					  	  std::vector<double>::iterator curr_bel_end )
{
	std::vector<int> vp_seq;
	
	get_GMI_sequence( eps, curr_vp, oMap, cMap,
							curr_bel_start, curr_bel_end, vp_seq );
							
	return vp_seq[0];
}

void
gmi_planner::get_GMI_sequence( double eps, int curr_vp,
									  boost::multi_array<double,3> const& oMap,
									  boost::multi_array<double,2> const& cMap,
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
	
	// rank the rest of the actions according to MI
	std::vector< std::pair<double, int> > mi_vp_vec;
	for( int vp = 0; vp < num_vp; ++vp)
		mi_vp_vec.push_back( std::make_pair( 
				MI( vp, oMap, curr_bel_start, curr_bel_end) / cMap[curr_vp][vp]
				, vp ) );

	// sort in ascending order
	//std::sort ( mi_vp_vec.begin(), mi_vp_vec.end() );
	
	// sort in descending MI order
	std::sort ( mi_vp_vec.rbegin(), mi_vp_vec.rend() );
	
	for( int vp = 0; vp < num_vp; ++vp)
	{
		vp_seq.push_back( mi_vp_vec[vp].second );
	}
}

double 
gmi_planner::MI( int vp, boost::multi_array<double,3> const& oMap,
						   std::vector<double>::iterator curr_bel_start, 
						   std::vector<double>::iterator curr_bel_end )
{
	int num_obs = oMap.shape()[0];
	double mi = 0.0;
	
	for(int obs = 0; obs < num_obs; ++obs)
	{
		double qTp = 0.0;
		int hid = 0;
		for( std::vector<double>::iterator it = curr_bel_start;
			it != curr_bel_end; ++it, ++hid)
		{
			qTp += oMap[obs][vp][hid] * (*it);
		}
		
		hid = 0;
		for( std::vector<double>::iterator it = curr_bel_start;
			it != curr_bel_end; ++it, ++hid)
		{
			mi += (oMap[obs][vp][hid] * (*it)) * log2( qTp / (oMap[obs][vp][hid] * (*it)));
		}
	}
	
	return mi;
}


