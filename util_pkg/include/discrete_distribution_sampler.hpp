// REQUIRED: -std=c++0x
#ifndef _DISCRETE_DISTRIBUTION_SAMPLER_HPP_
#define _DISCRETE_DISTRIBUTION_SAMPLER_HPP_

#include <iostream>
#include <random>
#include <chrono>
#include <sstream>
//#include <functional>

class discrete_distribution_sampler
{
	
private:
	std::default_random_engine generator;
	std::discrete_distribution<int> dist;

public:
	discrete_distribution_sampler(){}
	discrete_distribution_sampler( std::vector<double>::iterator dist_begin,
											 std::vector<double>::iterator dist_end )
		: generator( std::chrono::system_clock::now().time_since_epoch().count() ),
		  dist( dist_begin, dist_end)
	{}
											 
	int sample()
	{
		return dist( generator );
	}
	
		
	std::string to_str() const
   {
    	std::stringstream ss;
    	ss << "[discrete_distribution_sampler] Probabilities: " << std::endl;
    	for (double x:dist.probabilities()) ss << x << " ";
  		ss << std::endl;
   	return ss.str();
   }
   
private:
	friend std::ostream & operator<<(std::ostream &os, const discrete_distribution_sampler &d1)
	{
		return os << d1.to_str();
	}
};
#endif
