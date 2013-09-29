#include <iostream>
#include "boost_discrete_distribution_sampler.hpp"

int test_dd_samp(int argc, char **argv)
{
	const double prob_a[] = {0.125,0.125,0.0625,0.0625,0.125,0.125,0.0625,0.0625,0.125,0.125};
	std::vector<double> probs_v( prob_a, prob_a + sizeof(prob_a) / sizeof(prob_a[0]) );
	
	const int nrolls = 10000; // number of experiments
  	const int nstars = 100;   // maximum number of stars to distribute
	
	boost_discrete_distribution_sampler bdds( probs_v.begin(), probs_v.end() );
	
	int p[10]={};

	for (int i=0; i<nrolls; ++i) 
	{
   	int number = bdds.sample();
   	++p[number];
  	}

  	std::cout << "a discrete_distribution:" << std::endl;
  	for (int i=0; i<10; ++i)
   	std::cout << i << ": " << std::string(p[i]*nstars/nrolls,'*') << std::endl;
	
	return 0;
}

int main(int argc, char **argv)
{
	return test_dd_samp( argc, argv );
}
