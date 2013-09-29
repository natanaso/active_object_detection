#ifndef _GMI_PLANNER_HPP_
#define _GMI_PLANNER_HPP_

#include <vector>
#include <boost/multi_array.hpp>

class gmi_planner
{	
public:

	static int get_GMI_action( double eps, int curr_vp,
							boost::multi_array<double,3> const& oMap,
						   boost::multi_array<double,2> const& cMap,
							std::vector<double>::iterator curr_bel_start, 
					  	  	std::vector<double>::iterator curr_bel_end );

	static void get_GMI_sequence( double eps, int curr_vp,
						  const boost::multi_array<double,3> & oMap,
						  boost::multi_array<double,2> const& cMap,
						  std::vector<double>::iterator curr_bel_start, 
					  	  std::vector<double>::iterator curr_bel_end,
					  	  std::vector<int> & vp_seq );
					  	  
	static double MI( int vp, boost::multi_array<double,3> const& oMap,
						   std::vector<double>::iterator curr_bel_start, 
						   std::vector<double>::iterator curr_bel_end );
	
};

#endif
