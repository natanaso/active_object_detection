#ifndef SARSOP_USER_H_
#define SARSOP_USER_H_

// Standard
#include <vector>
#include <string>
#include <boost/multi_array.hpp>

// Sarsop
//#include "solverUtils.h"
//#include "MOMDP.h"

#define MaxStr 2048

class sarsop_user{
	private:
		//bool state;	// indicates that the pomdpx model has been written
		std::string sarsop_path;
		//std::string pomdpx_filepath;
		//std::string policy_filepath;
		//std::vector< std::vector <double> > polTable;
		//std::vector<int> actVec;
		//std::vector<int> staVec;
		std::vector< std::vector< std::vector <double> > > polTables;
		std::vector< std::vector<int> > actVecs;
		std::vector< std::vector<int> > staVecs;
		
		//bool fileToPolicy();
	public:
		// Constructor
		sarsop_user( ): sarsop_path( "pomdpsol" ){}
		sarsop_user(const std::string& sarsopPath): sarsop_path(sarsopPath){}
		
		void set_sarsop_path( std::string const & sarsopPath )
		{
			sarsop_path = sarsopPath;
		}
		
		// Typedefs
		typedef boost::multi_array<double,2> array2D;
		typedef boost::multi_array<double,3> array3D;
    	//typedef array3D::index index3D;
    
    	typedef boost::multi_array<int,2> array2I;
    	typedef boost::multi_array<int,3> array3I;
    	//typedef array3I::index index3I;
		
		// Methods		
		bool modelToFile(int num_sta, int num_hid, int num_act, int num_obs,
						 const int *fMap, const double *cMap, 
						 const double *oMap, const std::string& pomdpxFilePath,
						 int init_sta, std::vector<double>::iterator init_bel);

				  
		bool modelToFile( array2I const & fMap,
						  array3D const & cMap,
						  array3D const & oMap,
						  std::string const & pomdpxFilePath,
						  int init_sta,
						  std::vector<double>::iterator init_bel );

						  
		bool computePolicy( double precision, double time_limit, 
							const std::string &pomdpxFilePath,
						   const std::string &policyFilePath );
		
		/*
		bool fileToPolicy( std::string const & policyFilePath ){
			policy_filepath = policyFilePath;
			state = fileToPolicy();
			return state;
		}
		*/
		
		void loadPolicy( int init_sta, std::string const& base_path );
		void loadAllPolicies( int num_sta, std::string const& base_path );
						    
		int getBestAction( int sta,
						   std::vector<double>::iterator bel_start,
						   std::vector<double>::iterator bel_end );
						   
	private:
		void fileToPolicy( std::string const &policyFilePath,
								std::vector< std::vector <double> > &polTable,
								std::vector<int> &actVec,
								std::vector<int> &staVec );
		
		/*						
		struct OutputParams {
			double timeoutSeconds;
			double interval;
			OutputParams(void)
				: timeoutSeconds(-1), interval(-1) {}
		};
		*/
		
		/*
		#ifdef _MSC_VER
			BOOL CtrlHandler( DWORD fdwCtrlType );
			void registerCtrlHanler();
		#else
			void sigIntHandler(int sig);
			void setSignalHandler(int sig, void (*handler)(int));
		#endif
		*/
		
		/*
		void usage(const char* cmdName);
		int QMDPSolution(applSharedPointer<MOMDP> problem, SolverParams* p);
		int FIBSolution(applSharedPointer<MOMDP> problem, SolverParams* p);
		int MDPSolution(applSharedPointer<MOMDP> problem, SolverParams* p);
		int sarsop_gateway(int argc, char **argv);
		*/
};
#endif
