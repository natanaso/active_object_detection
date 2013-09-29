#ifndef _EJS_PLANNER_HPP_
#define _EJS_PLANNER_HPP_

#include <vector>
#include <boost/multi_array.hpp>

class ejs_planner
{
public:
	static int get_EJS_action( double eps, int curr_vp,
						  boost::multi_array<double,3> const& oMap,
						  boost::multi_array<double,2> const& cMap,
						  std::vector<double>::iterator curr_bel_start, 
					  	  std::vector<double>::iterator curr_bel_end );

	static void get_EJS_sequence( double eps, int curr_vp,
						  const boost::multi_array<double,3> & oMap,
						  boost::multi_array<double,2> const& cMap,
						  std::vector<double>::iterator curr_bel_start, 
					  	  std::vector<double>::iterator curr_bel_end,
					  	  std::vector<int> & vp_seq );
					  	  					  	  
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

};

#endif
