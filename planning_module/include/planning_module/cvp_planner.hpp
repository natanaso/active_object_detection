#ifndef _CVP_PLANNER_HPP_
#define _CVP_PLANNER_HPP_

#include <vector>
#include <boost/multi_array.hpp>

class cvp_planner
{	
public:
	static void get_CVP_sequence( double eps, int curr_vp,
						  boost::multi_array<double,2> const& cMap,
						  std::vector<double>::iterator curr_bel_start, 
					  	  std::vector<double>::iterator curr_bel_end,
					  	  std::vector<int> & vp_seq );
};

#endif
