#ifndef _BOOST_DISCRETE_DISTRIBUTION_SAMPLER_HPP_
#define _BOOST_DISCRETE_DISTRIBUTION_SAMPLER_HPP_

#include <ctime>
#include <boost/random/mersenne_twister.hpp>
//#include <boost/random/discrete_distribution.hpp>

// Add to avoid discrete_distribution.hpp
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <algorithm>
#include <numeric>
// ************




class boost_discrete_distribution_sampler
{
	
private:
	boost::mt19937 generator;
	//boost::random::discrete_distribution<> dist;
	
	// Add to avoid discrete_distribution.hpp
	std::vector<double> cumulative;
	boost::shared_ptr< boost::variate_generator<boost::mt19937&, boost::uniform_real<> > > disc_dist_ptr;
	// ************
	

public:
	boost_discrete_distribution_sampler( std::vector<double>::iterator dist_begin,
													 std::vector<double>::iterator dist_end )
		: generator( std::time(0) )
	{
		std::partial_sum(dist_begin, dist_end, std::back_inserter(cumulative));
		boost::uniform_real<> dist(0, cumulative.back());
		disc_dist_ptr.reset( new boost::variate_generator<boost::mt19937&, boost::uniform_real<> >(generator, dist) );
   }
	
	int sample()
	{
		return (std::lower_bound(cumulative.begin(), cumulative.end(), disc_dist_ptr->operator()()) - cumulative.begin());
	}
	
	/*
	boost_discrete_distribution_sampler( ){}
	boost_discrete_distribution_sampler( std::vector<double>::iterator dist_begin,
											 std::vector<double>::iterator dist_end )
		: generator( std::time(0) ),
		  dist( dist_begin, dist_end )
	{}
	
											 
	int sample()
	{
		return dist( generator );
	}
	*/
	
};
#endif
