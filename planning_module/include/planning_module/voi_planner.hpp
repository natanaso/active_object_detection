#ifndef _VOI_PLANNER_HPP_
#define _VOI_PLANNER_HPP_

#include <vector>
#include <boost/multi_array.hpp>

class voi_planner
{
public:
	static int getEJSAction( double eps, boost::multi_array<double,3> const& oMap,
						  std::vector<double>::iterator curr_bel_start, 
					  	  std::vector<double>::iterator curr_bel_end );
					  	  
	static double EJS( int vp, boost::multi_array<double,3> const& oMap,
						 std::vector<double>::iterator curr_bel_start, 
						 std::vector<double>::iterator curr_bel_end );
				 
	static double kl_div( boost::multi_array<double,1>::const_iterator p_start,
						boost::multi_array<double,1>::const_iterator p_end,
						boost::multi_array<double,1>::const_iterator q_start,
						boost::multi_array<double,1>::const_iterator q_end );
	
	static double kl_div( std::vector<double>::iterator p_start,
						std::vector<double>::iterator p_end,
						std::vector<double>::iterator q_start,
						std::vector<double>::iterator q_end );
						
	static void get_EJS_sequence( double eps, 
						  const boost::multi_array<double,3> & oMap,
						  std::vector<double>::iterator curr_bel_start, 
					  	  std::vector<double>::iterator curr_bel_end,
					  	  std::vector<int> & vp_seq );	
};

#endif
