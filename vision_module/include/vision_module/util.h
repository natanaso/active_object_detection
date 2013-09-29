/*
 * Copyright (c) 2011, University of Pennsylvania
 * All rights reserved.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * \author Bharath Sankaran and Nikolay Atanasov
 *
 * @b comprises all the utility functions
 */
#ifndef UTIL_H
#define UTIL_H
#include "vision_module/forward_declarations.h"
#include "vision_module/lookup_tables.h"

class NErr {
public:
   NErr() {};
   ~NErr() {};
   const char *ShowReason() const {
      return "Exception: The discretization needs to be >= 1";
   }
};
const double infty = std::numeric_limits<double>::infinity();
std::vector<Vector3d> discretize_sphere(const Vector3d& cntr, const double rad, const int N) throw(NErr){

   	if(N < 1)
   		throw NErr();
	  
	std::vector<Vector3d> pts;
	
	double R, X, Y, Z, alpha;
	int M, idx;
	
	// subdivision angle
	double beta = M_PI/2/N;
	
	// line segment length
	double A = 2*sin(beta/2);
	
	pts.push_back(Vector3d(cntr[0],cntr[1],cntr[2]+rad));
	pts.push_back(Vector3d(cntr[0],cntr[1],cntr[2]+rad));
	
	for(int i = 1; i<=N; i++){
		R = sin(i*beta);
		Z = cos(i*beta);
		M = round(R*2*M_PI/A);		
		for(int j = 0; j<M;j++){
			alpha = (double) j/M*2*M_PI;
			X = cos(alpha)*R;
			Y = sin(alpha)*R;
			
			if(i < N){
				pts.push_back(Vector3d(cntr[0]+rad*X,cntr[1]+rad*Y,cntr[2]+rad*Z));
				pts.push_back(Vector3d(cntr[0]+rad*X,cntr[1]+rad*Y,cntr[2]-rad*Z));
			}
			else{
				pts.push_back(Vector3d(cntr[0]+rad*X,cntr[1]+rad*Y,cntr[2]+rad*Z));
			}			
		}//ENDFOR
	}//ENDFOR	
	return pts;
}


double CalculateAvg(const std::vector<double> &list)
{
	if (!list.empty())
	   return(std::accumulate(list.begin(),list.end(),0.0) / list.size());
	else
           return(-1);
}

//Utility function for adding noise
void addNoise(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double noise_std)
{

  boost::mt19937 rng(static_cast<unsigned int> (std::time(0)));
  boost::normal_distribution<float> normal_distrib(0.0f, noise_std * noise_std);
  boost::variate_generator<boost::mt19937&, boost::normal_distribution<float> > gaussian_rng(rng, normal_distrib);

  for (size_t cp = 0; cp < cloud->points.size(); cp++)
  {
    cloud->points[cp].z += gaussian_rng();
  }

}
// utility class for Sequential hypothesis testing
class util{
	public:
	int states_;
	std::vector<double>f0_probs_,f1_probs_;
	std::vector<float>costs_;
	double f0_0_init[22],f0_1_init[22],f1_0_init[22],f1_1_init[22];

	util(int state,std::vector<float> &cost){
		states_ = state;
		costs_ = cost;
		for(int i = 0; i<22; i++)
			{
			f0_0_init[i] = heavyranch[i];
			f0_1_init[i] = not_heavyranch[i];
			f1_0_init[i] = axe[i];
			f1_1_init[i] = not_axe[i];
			}
		}

	~util(){
		}

	int movef(int state, int action){

		state = state + 1;
		action = action + 1;

		if(state+action > states_)
			return state+action-states_-1;
		else
			return state+action-1;
		}

	void init_f(){

		for(int i= 0;i<states_;i++)
			{
			f0_probs_.push_back(f0_1_init[i]/(f0_1_init[i]+f0_0_init[i]));
			f1_probs_.push_back(f1_1_init[i]/(f1_1_init[i]+f1_0_init[i]));
			}
		}

	double f0_sphere(int z, int x) {

		// probability of seeing a f0 given that the object is a f1
		if (z == 0)
			return 1 - f0_probs_[x];
		else
			return f0_probs_[x];

		}

	double f1_sphere(int z, int x) {

		// probability of seeing a f1 given that the object is a f0
		if (z == 0)
			return 1 - f1_probs_[x];
		else
			return f1_probs_[x];

		}

	double costf(int x, int u){

		int idx = movef(x,u);
		return (double)costs_[x];

		}
	};

#endif

