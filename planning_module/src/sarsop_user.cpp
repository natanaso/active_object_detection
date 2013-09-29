// cpp standard
#include <iostream>
#include <fstream>
#include <sstream>
#include <limits>
#include <numeric>
#include <string>		// strings
#include <boost/filesystem.hpp>	  //boost::filesystem::path(fpath).stem().string()
#include <boost/lexical_cast.hpp> //boost::lexical_cast<std::string>
#include <boost/scoped_array.hpp>
#include <assert.h>
#include <stdexcept>
//#include <signal.h>


/*
#ifdef _MSC_VER
	#include "getopt.h"
	#define NOMINMAX 
	#include <windows.h> 
#else
	#include <getopt.h>
	#include <sys/time.h>
#endif
*/

// c standard
#include <cfloat>		// DBL_MAX
//#include <cstdlib>		// system()
//#include <stdio.h>
//#include <stdlib.h>


/*
// SARSOP
#include "CPTimer.h"
#include "GlobalResource.h"
#include "Parser.h"
#include "POMDP.h"
#include "ParserSelector.h"
#include "SARSOP.h"
#include "BackupAlphaPlaneMOMDP.h"
#include "BackupBeliefValuePairMOMDP.h"
#include "FullObsUBInitializer.h"
#include "FastInfUBInitializer.h"
*/

// custom
#include "planning_module/sarsop_user.hpp"
#include "xml_parse_lib.c"

using std::string; using std::vector;
using std::cout; using std::cerr; using std::endl;


/**
 * Takes the problem data and generates a pomdpx model
 */
bool 
sarsop_user::modelToFile(int num_sta, int num_hid, int num_act, int num_obs,
						 const int *fMap, const double *cMap, 
						 const double *oMap, const string& pomdpxFilePath,
						 int init_sta, vector<double>::iterator init_bel)
{
	//pomdpx_filepath = pomdpxFilePath;
	const string file_name = boost::filesystem::path(pomdpxFilePath).stem().string();
	
	std::ofstream outfile ( pomdpxFilePath.c_str() );
	
	if(outfile.is_open()){
		
		// Header
		outfile << "<?xml version=\"1.0\" encoding=\"ISO-8859-1\"?>\n";
    	outfile << "\t<pomdpx version=\"1.0\" id=\"genByMATLAB\"\n";
    	outfile << "\t\txmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"\n";
    	outfile << "\t\txsi:noNamespaceSchemaLocation=\""
    			<< file_name <<".xsd\">\n";
    	outfile << "\t\t\t<Description>Automatically generated"
    			<< " POMDPX file</Description>\n";
    	
    	// Discount
    	outfile << "\t\t\t<Discount>0.99</Discount>\n";
    	
    	// Variables
    	outfile << "\t\t\t<Variable>\n";
    	outfile << "\t\t\t\t<StateVar vnamePrev=\"sta_0\" vnameCurr=\"sta_1\""
    			<< " fullyObs=\"true\">\n";
    	outfile << "\t\t\t\t\t<NumValues>" << num_sta << "</NumValues>\n";
    	outfile << "\t\t\t\t</StateVar>\n";
    	
    	outfile << "\t\t\t\t<StateVar vnamePrev=\"hid_0\" vnameCurr=\"hid_1\">\n";
    	outfile << "\t\t\t\t\t<NumValues>" << num_hid << "</NumValues>\n";
    	outfile << "\t\t\t\t</StateVar>\n";
    	
    	outfile << "\t\t\t\t<ObsVar vname=\"obs\">\n";
    	outfile << "\t\t\t\t\t<NumValues>" << num_obs << "</NumValues>\n";
    	outfile << "\t\t\t\t</ObsVar>\n";
    	
    	outfile << "\t\t\t\t<ActionVar vname=\"act\">\n";
    	outfile << "\t\t\t\t\t<NumValues>" << num_act << "</NumValues>\n";
    	outfile << "\t\t\t\t</ActionVar>\n";
    	
    	outfile << "\t\t\t\t<RewardVar vname=\"rwrd\" />\n";
    	outfile << "\t\t\t</Variable>\n";
    	
    	// Initial State Belief
    	outfile << "\t\t\t<InitialStateBelief>\n";
    	outfile << "\t\t\t\t<CondProb>\n";
    	outfile << "\t\t\t\t\t<Var>sta_0</Var>\n";
    	outfile << "\t\t\t\t\t<Parent>null</Parent>\n";
    	outfile << "\t\t\t\t\t<Parameter type=\"TBL\">\n";
    	outfile << "\t\t\t\t\t\t<Entry>\n";
    	outfile << "\t\t\t\t\t\t\t<Instance>s"<< init_sta <<"</Instance>\n";
    	outfile << "\t\t\t\t\t\t\t<ProbTable>1.0</ProbTable>\n";
    	outfile << "\t\t\t\t\t\t</Entry>\n";
    	outfile << "\t\t\t\t\t</Parameter>\n";
    	outfile << "\t\t\t\t</CondProb>\n";
    	
    	outfile << "\t\t\t\t<CondProb>\n";
    	outfile << "\t\t\t\t\t<Var>hid_0</Var>\n";
    	outfile << "\t\t\t\t\t<Parent>null</Parent>\n";
    	outfile << "\t\t\t\t\t<Parameter type=\"TBL\">\n";
    	outfile << "\t\t\t\t\t\t<Entry>\n";
    	outfile << "\t\t\t\t\t\t\t<Instance>-</Instance>\n";
    	outfile << "\t\t\t\t\t\t\t<ProbTable>";
    	for(int hid = 0; hid < num_hid; ++hid)
    		outfile << *init_bel++ << " ";
    	outfile << "</ProbTable>\n";
    	outfile << "\t\t\t\t\t\t</Entry>\n";
    	outfile << "\t\t\t\t\t</Parameter>\n";
    	outfile << "\t\t\t\t</CondProb>\n";    	
    	outfile << "\t\t\t</InitialStateBelief>\n";
    	
    	// State Transition Function
    	outfile << "\t\t\t<StateTransitionFunction>\n";
    	outfile << "\t\t\t\t<CondProb>\n";
    	outfile << "\t\t\t\t\t<Var>sta_1</Var>\n";
    	outfile << "\t\t\t\t\t<Parent>act sta_0</Parent>\n";
    	outfile << "\t\t\t\t\t<Parameter type=\"TBL\">\n";
    	
    	for(int act = 0; act < num_act; ++act)
    		for(int sta = 0; sta < num_sta; ++sta){
				outfile << "\t\t\t\t\t\t<Entry>\n";
				outfile << "\t\t\t\t\t\t\t<Instance>a"
						<< act << " s" << sta << " s"
						<< fMap[act*num_sta + sta]
						<< "</Instance>\n";
				outfile << "\t\t\t\t\t\t\t<ProbTable>1.0</ProbTable>\n"; 
				outfile << "\t\t\t\t\t\t</Entry>\n";
    		}
    	
    	outfile << "\t\t\t\t\t</Parameter>\n";
    	outfile << "\t\t\t\t</CondProb>\n";
    	
    	outfile << "\t\t\t\t<CondProb>\n";
    	outfile << "\t\t\t\t\t<Var>hid_1</Var>\n";
    	outfile << "\t\t\t\t\t<Parent>act sta_0 hid_0</Parent>\n";
    	outfile << "\t\t\t\t\t<Parameter type=\"TBL\">\n";
    	outfile << "\t\t\t\t\t\t<Entry>\n";
    	outfile << "\t\t\t\t\t\t\t<Instance>* * - - </Instance>\n";
    	outfile << "\t\t\t\t\t\t\t<ProbTable>identity</ProbTable>\n";
    	outfile << "\t\t\t\t\t\t</Entry>\n";
    	outfile << "\t\t\t\t\t</Parameter>\n";
    	outfile << "\t\t\t\t</CondProb>\n";    	
    	outfile << "\t\t\t</StateTransitionFunction>\n";
    	
    	// Obs Function
    	outfile << "\t\t\t<ObsFunction>\n";
    	outfile << "\t\t\t\t<CondProb>\n";
    	outfile << "\t\t\t\t\t<Var>obs</Var>\n";
    	outfile << "\t\t\t\t\t<Parent>sta_1 hid_1</Parent>\n";
    	outfile << "\t\t\t\t\t<Parameter type=\"TBL\">\n";
    	
    	int tmp_idx = 0;
    	for(int sta = 0; sta < num_sta; ++sta)
    		for(int hid = 0; hid < num_hid; ++hid){
    			outfile << "\t\t\t\t\t\t<Entry>\n";
    			outfile << "\t\t\t\t\t\t\t<Instance>s"
    					<< sta << " s" << hid <<" - </Instance>\n";
    			outfile << "\t\t\t\t\t\t\t<ProbTable>";
    			
    			for(int obs = 0; obs < num_obs; ++obs){
    				tmp_idx = obs * num_sta * num_hid + sta * num_hid + hid;
    				outfile << oMap[tmp_idx] << " ";
    			}
    			outfile << "</ProbTable>\n";		
    			outfile << "\t\t\t\t\t\t</Entry>\n";
    		}
    	
    	outfile << "\t\t\t\t\t</Parameter>\n";
    	outfile << "\t\t\t\t</CondProb>\n"; 
    	outfile << "\t\t\t</ObsFunction>\n";    	
    	
    	// Cost Function
    	outfile << "\t\t\t<RewardFunction>\n";
    	outfile << "\t\t\t\t<Func>\n";
    	outfile << "\t\t\t\t\t<Var>rwrd</Var>\n";
    	outfile << "\t\t\t\t\t<Parent>act sta_0 hid_0</Parent>\n";
    	outfile << "\t\t\t\t\t<Parameter type=\"TBL\">\n";
    	
    	for(int act = 0; act < num_act; ++act)
    		for(int sta = 0; sta < num_sta; ++sta)
    			for(int hid = 0; hid < num_hid; ++hid){
					outfile << "\t\t\t\t\t\t<Entry>\n";
					outfile << "\t\t\t\t\t\t\t<Instance>a"
							<< act << " s" << sta 
							<< " s" << hid <<"</Instance>\n";
							
					tmp_idx = act * num_sta * num_hid + sta * num_hid + hid;
					outfile << "\t\t\t\t\t\t\t<ValueTable>"
							<< (-cMap[tmp_idx]) << "</ValueTable>\n";		
					outfile << "\t\t\t\t\t\t</Entry>\n";
    			}

    	outfile << "\t\t\t\t\t</Parameter>\n";
    	outfile << "\t\t\t\t</Func>\n";
    	outfile << "\t\t\t</RewardFunction>\n"; 
    	
    	// End    	    	
    	outfile << "\t</pomdpx>\n";
		outfile.close();
		return true;
	}
	else{
		std::cout << "Unable to open " << file_name << std::endl;
		return false;
	}
}


/**
 * Takes the problem data and generates a pomdpx model
 */
bool 
sarsop_user::modelToFile( array2I const & fMap,
						  array3D const & cMap,
						  array3D const & oMap,
						  std::string const & pomdpxFilePath,
						  int init_sta,
						  std::vector<double>::iterator init_bel )
{
	// get dimensions:
	// fMap[num_act][num_sta];
	// cMap[num_act][num_sta][num_hid];
	// oMap[num_obs][num_sta][num_hid];	
	int num_act = fMap.shape()[0];
	int num_sta = fMap.shape()[1];
	int num_obs = oMap.shape()[0];
	int num_hid = oMap.shape()[2];
	
	// Open the file and start writing
	//pomdpx_filepath = pomdpxFilePath;
	const string file_name = boost::filesystem::path(pomdpxFilePath).stem().string();
	
	std::ofstream outfile ( pomdpxFilePath.c_str() );
	
	if(outfile.is_open()){
		
		// Header
		outfile << "<?xml version=\"1.0\" encoding=\"ISO-8859-1\"?>\n";
    	outfile << "\t<pomdpx version=\"1.0\" id=\"genByMATLAB\"\n";
    	outfile << "\t\txmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"\n";
    	outfile << "\t\txsi:noNamespaceSchemaLocation=\""
    			<< file_name <<".xsd\">\n";
    	outfile << "\t\t\t<Description>Automatically generated"
    			<< " POMDPX file</Description>\n";
    	
    	// Discount
    	outfile << "\t\t\t<Discount>0.99</Discount>\n";
    	
    	// Variables
    	outfile << "\t\t\t<Variable>\n";
    	outfile << "\t\t\t\t<StateVar vnamePrev=\"sta_0\" vnameCurr=\"sta_1\""
    			<< " fullyObs=\"true\">\n";
    	outfile << "\t\t\t\t\t<NumValues>" << num_sta << "</NumValues>\n";
    	outfile << "\t\t\t\t</StateVar>\n";
    	
    	outfile << "\t\t\t\t<StateVar vnamePrev=\"hid_0\" vnameCurr=\"hid_1\">\n";
    	outfile << "\t\t\t\t\t<NumValues>" << num_hid << "</NumValues>\n";
    	outfile << "\t\t\t\t</StateVar>\n";
    	
    	outfile << "\t\t\t\t<ObsVar vname=\"obs\">\n";
    	outfile << "\t\t\t\t\t<NumValues>" << num_obs << "</NumValues>\n";
    	outfile << "\t\t\t\t</ObsVar>\n";
    	
    	outfile << "\t\t\t\t<ActionVar vname=\"act\">\n";
    	outfile << "\t\t\t\t\t<NumValues>" << num_act << "</NumValues>\n";
    	outfile << "\t\t\t\t</ActionVar>\n";
    	
    	outfile << "\t\t\t\t<RewardVar vname=\"rwrd\" />\n";
    	outfile << "\t\t\t</Variable>\n";
    	
    	// Initial State Belief
    	outfile << "\t\t\t<InitialStateBelief>\n";
    	outfile << "\t\t\t\t<CondProb>\n";
    	outfile << "\t\t\t\t\t<Var>sta_0</Var>\n";
    	outfile << "\t\t\t\t\t<Parent>null</Parent>\n";
    	outfile << "\t\t\t\t\t<Parameter type=\"TBL\">\n";
    	outfile << "\t\t\t\t\t\t<Entry>\n";
    	outfile << "\t\t\t\t\t\t\t<Instance>s"<< init_sta <<"</Instance>\n";
    	outfile << "\t\t\t\t\t\t\t<ProbTable>1.0</ProbTable>\n";
    	outfile << "\t\t\t\t\t\t</Entry>\n";
    	outfile << "\t\t\t\t\t</Parameter>\n";
    	outfile << "\t\t\t\t</CondProb>\n";
    	
    	outfile << "\t\t\t\t<CondProb>\n";
    	outfile << "\t\t\t\t\t<Var>hid_0</Var>\n";
    	outfile << "\t\t\t\t\t<Parent>null</Parent>\n";
    	outfile << "\t\t\t\t\t<Parameter type=\"TBL\">\n";
    	outfile << "\t\t\t\t\t\t<Entry>\n";
    	outfile << "\t\t\t\t\t\t\t<Instance>-</Instance>\n";
    	outfile << "\t\t\t\t\t\t\t<ProbTable>";
    	for(int hid = 0; hid < num_hid; ++hid)
    		outfile << *init_bel++ << " ";
    	outfile << "</ProbTable>\n";
    	outfile << "\t\t\t\t\t\t</Entry>\n";
    	outfile << "\t\t\t\t\t</Parameter>\n";
    	outfile << "\t\t\t\t</CondProb>\n";    	
    	outfile << "\t\t\t</InitialStateBelief>\n";
    	
    	// State Transition Function
    	outfile << "\t\t\t<StateTransitionFunction>\n";
    	outfile << "\t\t\t\t<CondProb>\n";
    	outfile << "\t\t\t\t\t<Var>sta_1</Var>\n";
    	outfile << "\t\t\t\t\t<Parent>act sta_0</Parent>\n";
    	outfile << "\t\t\t\t\t<Parameter type=\"TBL\">\n";
    	
    	for(int act = 0; act < num_act; ++act)
    		for(int sta = 0; sta < num_sta; ++sta){
				outfile << "\t\t\t\t\t\t<Entry>\n";
				outfile << "\t\t\t\t\t\t\t<Instance>a"
						<< act << " s" << sta << " s"
						<< fMap[act][sta]	//fMap[act*num_sta + sta]
						<< "</Instance>\n";
				outfile << "\t\t\t\t\t\t\t<ProbTable>1.0</ProbTable>\n"; 
				outfile << "\t\t\t\t\t\t</Entry>\n";
    		}
    	
    	outfile << "\t\t\t\t\t</Parameter>\n";
    	outfile << "\t\t\t\t</CondProb>\n";
    	
    	outfile << "\t\t\t\t<CondProb>\n";
    	outfile << "\t\t\t\t\t<Var>hid_1</Var>\n";
    	outfile << "\t\t\t\t\t<Parent>act sta_0 hid_0</Parent>\n";
    	outfile << "\t\t\t\t\t<Parameter type=\"TBL\">\n";
    	outfile << "\t\t\t\t\t\t<Entry>\n";
    	outfile << "\t\t\t\t\t\t\t<Instance>* * - - </Instance>\n";
    	outfile << "\t\t\t\t\t\t\t<ProbTable>identity</ProbTable>\n";
    	outfile << "\t\t\t\t\t\t</Entry>\n";
    	outfile << "\t\t\t\t\t</Parameter>\n";
    	outfile << "\t\t\t\t</CondProb>\n";    	
    	outfile << "\t\t\t</StateTransitionFunction>\n";
    	
    	// Obs Function
    	outfile << "\t\t\t<ObsFunction>\n";
    	outfile << "\t\t\t\t<CondProb>\n";
    	outfile << "\t\t\t\t\t<Var>obs</Var>\n";
    	outfile << "\t\t\t\t\t<Parent>sta_1 hid_1</Parent>\n";
    	outfile << "\t\t\t\t\t<Parameter type=\"TBL\">\n";
    	
    	//int tmp_idx = 0;
    	for(int sta = 0; sta < num_sta; ++sta)
    		for(int hid = 0; hid < num_hid; ++hid){
    			outfile << "\t\t\t\t\t\t<Entry>\n";
    			outfile << "\t\t\t\t\t\t\t<Instance>s"
    					<< sta << " s" << hid <<" - </Instance>\n";
    			outfile << "\t\t\t\t\t\t\t<ProbTable>";
    			
    			for(int obs = 0; obs < num_obs; ++obs){
    				//tmp_idx = obs * num_sta * num_hid + sta * num_hid + hid;
    				outfile << oMap[obs][sta][hid] << " ";	//oMap[tmp_idx]
    			}
    			outfile << "</ProbTable>\n";		
    			outfile << "\t\t\t\t\t\t</Entry>\n";
    		}
    	
    	outfile << "\t\t\t\t\t</Parameter>\n";
    	outfile << "\t\t\t\t</CondProb>\n"; 
    	outfile << "\t\t\t</ObsFunction>\n";    	
    	
    	// Cost Function
    	outfile << "\t\t\t<RewardFunction>\n";
    	outfile << "\t\t\t\t<Func>\n";
    	outfile << "\t\t\t\t\t<Var>rwrd</Var>\n";
    	outfile << "\t\t\t\t\t<Parent>act sta_0 hid_0</Parent>\n";
    	outfile << "\t\t\t\t\t<Parameter type=\"TBL\">\n";
    	
    	for(int act = 0; act < num_act; ++act)
    		for(int sta = 0; sta < num_sta; ++sta)
    			for(int hid = 0; hid < num_hid; ++hid){
					outfile << "\t\t\t\t\t\t<Entry>\n";
					outfile << "\t\t\t\t\t\t\t<Instance>a"
							<< act << " s" << sta 
							<< " s" << hid <<"</Instance>\n";
							
					//tmp_idx = act * num_sta * num_hid + sta * num_hid + hid;
					outfile << "\t\t\t\t\t\t\t<ValueTable>"
							<< (-cMap[act][sta][hid]) << "</ValueTable>\n";		//-cMap[tmp_idx]
					outfile << "\t\t\t\t\t\t</Entry>\n";
    			}

    	outfile << "\t\t\t\t\t</Parameter>\n";
    	outfile << "\t\t\t\t</Func>\n";
    	outfile << "\t\t\t</RewardFunction>\n"; 
    	
    	// End    	    	
    	outfile << "\t</pomdpx>\n";
		outfile.close();
		return true;
	}
	else{
		std::cout << "Unable to open " << file_name << std::endl;
		return false;
	}
}
						  
/**
 * Calls SARSOP to compute a policy
 */
bool 
sarsop_user::computePolicy( double precision, double time_limit,
							const std::string &pomdpxFilePath,
						   const std::string &policyFilePath)
{	
	
	// Format a command to call SARSOP
	std::ostringstream command;
	command << sarsop_path << " " << pomdpxFilePath << " -o "
			<< policyFilePath << " -p " << precision
			<< " --timeout " << time_limit;
	
	// Display for debugging purposes		
	cout << command.str() << endl;
	int result = system(command.str().c_str());
	
	
	/*
	// Call SARSOP
	int argc = 8;
	char *argv[argc];
	boost::scoped_array<char> sarsop_path_char(new char[sarsop_path.size() + 1]);
	std::copy(sarsop_path.begin(), sarsop_path.end(), sarsop_path_char.get());
	sarsop_path_char[sarsop_path.size()] = '\0'; // don't forget the terminating 0
	argv[0] = sarsop_path_char.get();
	
	boost::scoped_array<char> pomdpx_filepath_char(new char[pomdpxFilePath.size() + 1]);
	std::copy(pomdpxFilePath.begin(), pomdpxFilePath.end(), pomdpx_filepath_char.get());
	pomdpx_filepath_char[pomdpxFilePath.size()] = '\0'; // don't forget the terminating 0
	argv[1] = pomdpx_filepath_char.get();
	
	boost::scoped_array<char> policy_filepath_char(new char[policyFilePath.size() + 1]);
	std::copy(policyFilePath.begin(), policyFilePath.end(), policy_filepath_char.get());
	policy_filepath_char[policyFilePath.size()] = '\0'; // don't forget the terminating 0	
	argv[2] = "-o";
	argv[3] = policy_filepath_char.get();
	
	std::string precision_str = boost::lexical_cast<std::string>(precision);
	boost::scoped_array<char> precision_char(new char[precision_str.size() + 1]);
	std::copy(precision_str.begin(), precision_str.end(), precision_char.get());
	precision_char[precision_str.size()] = '\0'; // don't forget the terminating 0		
	argv[4] = "-p";
	argv[5] = precision_char.get();
	
	std::string time_limit_str = boost::lexical_cast<std::string>(time_limit);
	boost::scoped_array<char> time_limit_char(new char[time_limit_str.size() + 1]);
	std::copy(time_limit_str.begin(), time_limit_str.end(), time_limit_char.get());
	time_limit_char[time_limit_str.size()] = '\0'; // don't forget the terminating 0	
	argv[6] = "--timeout";
	argv[7] = time_limit_char.get();
		
	int result = sarsop_gateway(argc, argv);
	*/
	
	if(result == 0)
		return true;
	else
		return false;
	
}

/**
 * Extracts the info from an xml policy file 
 */
void
sarsop_user::fileToPolicy( std::string const &policyFilePath,
									std::vector< std::vector <double> > &polTable,
									std::vector<int> &actVec,
									std::vector<int> &staVec )
{
	// IMPORTANT: clear the old information!
	polTable.clear();
	actVec.clear();
	staVec.clear();
	
	char tag[MaxStr], contents[MaxStr], tagname[MaxStr], attrname[MaxStr], value[MaxStr];
   //float x1, y1, z1, x2, y2, z2, t0, t1;
   int linum=0;
   FILE *infile=0;//, *outfile=0;
    
    infile = fopen(policyFilePath.c_str(),"r");    
    if(infile==0)
    {
		cerr << "ERROR: couldn't open " << policyFilePath << " for reading " << endl;
		exit(EXIT_FAILURE);
    }
    xml_parse( infile, tag, contents, MaxStr, &linum );
    xml_grab_tag_name( tag, tagname, MaxStr );	/* Get tag name. */
    xml_grab_attrib( tag, attrname, value, MaxStr );	/* Get next attribute, if any. */
    while(value[0] != '\0'){
		xml_grab_attrib( tag, attrname, value, MaxStr );	/* Get next attribute, if any. */
    }

    xml_parse( infile, tag, contents, MaxStr, &linum );
    xml_grab_tag_name( tag, tagname, MaxStr );	/* Get tag name. */
    if(string(tagname)!="Policy"){
		cerr << "ERROR:\n\tExpected Policy tag as root" << endl;
		exit(EXIT_FAILURE);
    }
    
    xml_grab_attrib( tag, attrname, value, MaxStr );	/* Get any attributes within tag. */
    while(value[0] != '\0'){
		if(string(attrname)=="type" && string(value)!="value"){
			cerr << "ERROR:\n\tOnly policy of type \"value\" is supported" << endl;
			exit(EXIT_FAILURE);
		}
		xml_grab_attrib( tag, attrname, value, MaxStr );	/* Get next attribute, if any. */
    }

    xml_parse( infile, tag, contents, MaxStr, &linum );
    xml_grab_tag_name( tag, tagname, MaxStr );	/* Get tag name. */
    if(string(tagname)!="AlphaVector"){
		cerr << "ERROR:\n\tExpected AlphaVector tag" << endl;
		exit(EXIT_FAILURE);
    }
    int vectorLength = -1;

    xml_grab_attrib( tag, attrname, value, MaxStr );	/* Get any attributes within tag. */
    while(value[0] != '\0'){
		if(string(attrname)=="vectorLength")
		{
			vectorLength = atoi(value);
		}
		xml_grab_attrib( tag, attrname, value, MaxStr );
    }

    if(vectorLength == -1){
		cerr << "ERROR:\n\tCannot find integer attribute vectorLength in AlphaVector Tag" << endl;
		exit(EXIT_FAILURE);
    }

    vector<double> polTableLine(vectorLength);
    xml_parse_tag_only( infile, tag, MaxStr, &linum );		//read the vector tag
    xml_grab_tag_name( tag, tagname, MaxStr);
    while( tag[0]!='\0'){
		bool foundAct = false;	bool foundObs = false;
		xml_grab_attrib( tag, attrname, value, MaxStr );
		while(value[0] != '\0'){
			if(string(attrname)=="action"){
				actVec.push_back(atoi(value));
				foundAct = true;
			}
			else if(string(attrname)=="obsValue"){
				staVec.push_back(atoi(value));
				foundObs = true;
			}
			xml_grab_attrib( tag, attrname, value, MaxStr );
		}

		if(!foundAct){
			cerr << "ERROR:\n\tCannot find integer attribute action in Vector tag" << endl;
			exit(EXIT_FAILURE);
		}
		if(!foundObs){
			cerr << "ERROR:\n\tCannot find integer attribute obsValue in Vector tag" << endl;
			exit(EXIT_FAILURE);
		}

		//check if vector is dense or sparse
		if(string(tagname)=="Vector"){ 	    
			for(int i = 0; i < vectorLength; ++i){
				char  dvalue[200];
				if(fscanf(infile, "%s", &dvalue)==EOF){
					cerr << "ERROR:\n\tVector is too short, are you using the correct policy file?" << endl;
					exit(EXIT_FAILURE);
				}
				polTableLine[i] = atof(dvalue);
			}
			polTable.push_back(polTableLine);
		}
		else if(string(tagname)=="SparseVector"){
			xml_parse( infile, tag, contents, MaxStr, &linum );
			xml_grab_tag_name( tag, tagname, MaxStr);
			while(string(tagname)=="Entry"){
				double value;int index;
				sscanf(contents, "%d %f", &index, &value);
				
				polTableLine[index] = value;

				xml_parse( infile, tag, contents, MaxStr, &linum );
				xml_parse( infile, tag, contents, MaxStr, &linum );
				xml_grab_tag_name( tag, tagname, MaxStr);
			}
			polTable.push_back(polTableLine);
		}
		xml_parse(infile, tag,contents, MaxStr, &linum );
		xml_parse_tag_only( infile, tag, MaxStr, &linum );		//read the vector tag
		xml_grab_tag_name( tag, tagname, MaxStr);
    }
    fclose(infile);
}

void
sarsop_user::loadPolicy( int init_sta, std::string const& base_path )
{
	if( polTables.size() < init_sta+1 )
	{
		polTables.resize(init_sta+1);
		actVecs.resize(init_sta+1);
		staVecs.resize(init_sta+1);	
	}
	fileToPolicy( base_path + "_" + boost::lexical_cast<std::string>(init_sta) + ".policy",
				  polTables[init_sta], actVecs[init_sta], staVecs[init_sta] );
}

void 
sarsop_user::loadAllPolicies( int num_sta, std::string const& base_path )
{
	if( polTables.size() < num_sta )
	{
		polTables.resize(num_sta);
		actVecs.resize(num_sta);
		staVecs.resize(num_sta);
	}
		
	for( int sta = 0; sta < num_sta; ++sta)
	{

		fileToPolicy( base_path + "_" + boost::lexical_cast<std::string>(sta) + ".policy",
						  polTables[sta], actVecs[sta], staVecs[sta] );
	}
	
}

/**
 * Returns the best action for the current state and belief
 */
int
sarsop_user::getBestAction(int sta, vector<double>::iterator bel_start,
				  	  	   vector<double>::iterator bel_end)
{
	int best_act = -1;
	double best_val = -std::numeric_limits<double>::infinity();
	
	int pol_id;
	if( sta < polTables.size())
		pol_id = sta;
	else if( polTables.size() == 1 )
		pol_id = 0;
	else
		throw std::runtime_error("[sarsop user] No policy was found for the provided state...\n");

	
	double alphaVal = 0.0;

	int numAlphaVecs = polTables[pol_id].size();

	for(int a = 0; a < numAlphaVecs; ++a){
		if(staVecs[pol_id][a] == sta){
			alphaVal = inner_product(bel_start, bel_end,
									 polTables[pol_id][a].begin(),0.0);
			if(alphaVal > best_val){
				best_val = alphaVal;
				best_act = actVecs[pol_id][a];
			}
		}
	}

	
	std::cout << "[sarsop_user] The best value is " << best_val << std::endl;
	//std::cout << "The best action is " << best_act << std::endl;	
	return best_act;		  
}

/*
void 
sarsop_user::usage(const char* cmdName)
{
	cerr <<
		"Usage: " << cmdName << " POMDPModelFileName [--fast] [--precison targetPrecision] [--randomization]\n" 
"	[--timeout timeLimit] [--memory memoryLimit] [--output policyFileName]\n" 
"	[--policy-interval timeInterval]\n"
		"    or " <<cmdName << " --help (or -h)	Print this help\n"
		"    or " <<cmdName << " --version		Print version information\n"
		"\n"
		"Solver options:\n"
        "  -f or --fast		Use fast (but very picky) alternate parser for .pomdp files.\n"
	"  -p or --precision targetPrecision\n"    
"			Set targetPrecision as the target precision in solution \n"
"			quality; run ends when target precision is reached. The target\n" 
"			precision is 1e-3 by default.\n"
	"  --randomization	Turn on randomization for the sampling algorithm.\n"
"			Randomization is off by default.\n"
	"  --timeout timeLimit	Use timeLimit as the timeout in seconds.  If running time\n" 
"			exceeds the specified value, the solver writes out a policy and\n" 
"			terminates. There is no time limit by default.\n"
	"  --memory memoryLimit	Use memoryLimit as the memory limit in MB. No memory limit\n" 
"			by default.  If memory usage exceeds the specified value,\n" 
"			ofsol writes out a policy and terminates. Set the value to be\n" 
"			less than physical memory to avoid swapping.\n"
	"  --trial-improvement-factor improvementConstant\n"
"			Use improvementConstant as the trial improvement factor in the\n"
"			sampling algorithm. At the default of 0.5, a trial terminates at\n" 
"			a belief when the gap between its upper and lower bound is 0.5 of\n" 
"			the current precision at the initial belief.\n" 
		"\n"
		"Policy output options:\n"
       "  -o or --output policyFileName\n"        
"			Use policyFileName as the name of policy output file. The\n" 
"			file name is 'out.policy' by default.\n"
	"  --policy-interval timeInterval\n"       
"			Use timeInterval as the time interval between two consecutive\n" 
"			write-out of policy files. If this is not specified, the solver\n" 
"			only writes out a policy file upon termination.\n"
		"\n"
		"Examples:\n"
		"  " << cmdName << " Hallway.pomdp\n"
		"  " << cmdName << " --timeout 100 --output hallway.policy Hallway.pomdp\n"
		"\n"
		;

	exit(-1);
}

int 
sarsop_user::QMDPSolution(applSharedPointer<MOMDP> problem, SolverParams* p)
{
	cout << "Generate QMDP Policy" << endl;
	double targetPrecision = MDP_RESIDUAL;
	// no need to invoke POMDP solver
	// solve MDP
	FullObsUBInitializer m;
	if(problem->XStates->size() != 1 && problem->hasPOMDPMatrices())
	{
		DEBUG_LOG(cout << "Calling FullObsUBInitialize::QMDPSolution_unfac()" << endl;);
		// un-factored 
		// only does this if convert fast is called to produce pomdp version of the matrices
		// need pomdp matrix
		m.QMDPSolution_unfac(problem, targetPrecision); // SYL030909 prevly: m.QValueIteration_unfac(problem, targetPrecision);
		int numActions  = problem->actions->size();
		int numXstates = problem->XStates->size();
		int numYstates = problem->YStates->size();
		m.actionAlphaByState.resize(numActions);
		FOR(a, numActions)
		{
			m.actionAlphaByState[a].resize(numXstates);
			FOR (state_idx, numXstates) 
			{
				m.actionAlphaByState[a][state_idx].resize(problem->getBeliefSize());
			}

		}

		FOR(a, numActions)
		{
			m.UnfacPostProcessing(m.actionAlphas[a], m.actionAlphaByState[a]);
		}
	}
	else
	{
		DEBUG_LOG(cout << "Calling FullObsUBInitialize::QMDPSolution()" << endl;);
		// factored
		m.QMDPSolution(problem, targetPrecision); // SYL030909 prevly: m.QValueIteration(problem, targetPrecision);
		FOR(a, problem->actions->size())
		{
			m.FacPostProcessing(m.actionAlphaByState[a]);
		}
	}

	AlphaPlanePoolSet alphaPlanePoolSet(NULL);
	alphaPlanePoolSet.setProblem(problem);
	alphaPlanePoolSet.setSolver(NULL);
	alphaPlanePoolSet.initialize();
	//addAlphaPlane(alphaPlane);
	
	FOR(a, problem->actions->size())
	{
		for(int stateidx = 0; stateidx < alphaPlanePoolSet.set.size() ; stateidx ++)
		{
			applSharedPointer<AlphaPlane> plane (new AlphaPlane());
			copy(*plane->alpha, m.actionAlphaByState[a][stateidx]);
			plane->action = a;
			plane->sval = stateidx;

			alphaPlanePoolSet.set[stateidx]->addAlphaPlane(plane);
		}
	}
	string outFileName (p->outPolicyFileName);
	alphaPlanePoolSet.writeToFile(outFileName, p->problemName);
	return 0;	
}

int 
sarsop_user::FIBSolution(applSharedPointer<MOMDP> problem, SolverParams* p)
{
	cout << "Generate FIB Policy" << endl;
	double targetPrecision = MDP_RESIDUAL;
	// no need to invoke POMDP solver

	FastInfUBInitializer f(problem);
	DEBUG_LOG(cout << "Calling FastInfUBInitializer::getFIBsolution()" << endl;);		f.getFIBsolution(targetPrecision);

	AlphaPlanePoolSet alphaPlanePoolSet(NULL);
	alphaPlanePoolSet.setProblem(problem);
	alphaPlanePoolSet.setSolver(NULL);
	alphaPlanePoolSet.initialize();
	//addAlphaPlane(alphaPlane);
	
	FOR(a, problem->actions->size())
	{
		for(int stateidx = 0; stateidx < alphaPlanePoolSet.set.size() ; stateidx ++)
		{
			applSharedPointer<AlphaPlane> plane (new AlphaPlane());
			copy(*plane->alpha, f.actionAlphaByState[a][stateidx]);
			plane->action = a;
			plane->sval = stateidx;

			alphaPlanePoolSet.set[stateidx]->addAlphaPlane(plane);
		}
	}
	string outFileName (p->outPolicyFileName);
	alphaPlanePoolSet.writeToFile(outFileName, p->problemName); 
	return 0;	
}

int 
sarsop_user::MDPSolution(applSharedPointer<MOMDP> problem, SolverParams* p)
{
    cout << "Generate MDP Policy" << endl;
    double targetPrecision = MDP_RESIDUAL;
    // no need to invoke POMDP solver
    // solve MDP
    FullObsUBInitializer m;
    if(problem->XStates->size() != 1 && problem->hasPOMDPMatrices())
    {
	// un-factored 
	// only does this if convert fast is called to produce pomdp version of the matrices
	// need pomdp matrix
	m.alphaByState.resize(problem->XStates->size());
	DEBUG_LOG(cout << "Calling FullObsUBInitialize::valueIteration_unfac()" << endl;);
	m.valueIteration_unfac(problem, targetPrecision);
	m.UnfacPostProcessing(m.alpha, m.alphaByState);
    }
    else
    {
	// factored
	DEBUG_LOG(cout << "Calling FullObsUBInitialize::valueIteration()" << endl;);
	m.valueIteration(problem, targetPrecision);
	m.FacPostProcessing(m.alphaByState);
    }

    AlphaPlanePoolSet alphaPlanePoolSet(NULL);
    alphaPlanePoolSet.setProblem(problem);
    alphaPlanePoolSet.setSolver(NULL);
    alphaPlanePoolSet.initialize();
    //addAlphaPlane(alphaPlane);

    
    //do one step lookahead if problem is pure MDP
    if(problem->YStates->size() == 1)
    {
	for(int stateidx = 0; stateidx < alphaPlanePoolSet.set.size() ; stateidx ++)
	{
	    applSharedPointer<AlphaPlane> plane (new AlphaPlane());
	    int maxAction = 0;
	    double maxActionLB = -DBL_MAX;

	    //search for the best action for this state
	    applSharedPointer<BeliefWithState> b = applSharedPointer<BeliefWithState>(new BeliefWithState); 
	    b->bvec = new SparseVector(); b->bvec->resize(1);
	    b->bvec->push_back(0,1.0); b->sval=stateidx;
	    //initialise the MDP belief to current state
	    obsState_prob_vector spv;  // outcome probability for values of observed state
	    for(Actions::iterator aIter = problem->actions->begin(); aIter != problem->actions->end(); aIter ++) 
	    {
		int a = aIter.index();

		double sum = 0.0;
		double immediateReward = problem->rewards->getReward(*b, a);
		problem->getObsStateProbVector(spv, *b, a);

		FOR(Xn, spv.size()) 
		{
		    double sprob = spv(Xn);
		    if (sprob > OBS_IS_ZERO_EPS) 
		    {
			double childLB =  m.alphaByState[Xn](0);
			sum += childLB * sprob;
		    }
		}
		sum *= problem->getDiscount();
		sum += immediateReward;

		if(sum > maxActionLB)
		{
		    maxActionLB = sum;
		    maxAction = a;
		}
		assert(maxActionLB !=  -DBL_MAX);
	    }

	    copy(*plane->alpha, m.alphaByState[stateidx]);
	    plane->action = maxAction;
	    plane->sval = stateidx;

	    alphaPlanePoolSet.set[stateidx]->addAlphaPlane(plane);
	}
    }
    else{
	for(int stateidx = 0; stateidx < alphaPlanePoolSet.set.size() ; stateidx ++)
	{
		applSharedPointer<AlphaPlane> plane (new AlphaPlane());
		copy(*plane->alpha, m.alphaByState[stateidx]);
		plane->action = -1;
		plane->sval = stateidx;

		alphaPlanePoolSet.set[stateidx]->addAlphaPlane(plane);
	}
    }

    string outFileName (p->outPolicyFileName);
    alphaPlanePoolSet.writeToFile(outFileName, p->problemName);
    return 0;	
}


int 
sarsop_user::sarsop_gateway(int argc, char **argv) 
{

	//try
	{
		optind = 1;
		opterr = 1;
		optopt = '?';
		
		SolverParams* p = &GlobalResource::getInstance()->solverParams;
		bool parseCorrect = SolverParams::parseCommandLineOption(argc, argv, *p);
		if(!parseCorrect)
		{
			usage(p->cmdName);
			exit(EXIT_FAILURE);
		}

		OutputParams op;
		if(GlobalResource::getInstance()->benchmarkMode)
		{
			if(GlobalResource::getInstance()->simNum == 0|| GlobalResource::getInstance()->simLen == 0)
			{
				cout << "Benchmark Length and/or Number not set, please set them using option --simLen and --simNum" << endl;
				exit(-1);
			}
		}


		GlobalResource::getInstance()->init();
		string baseName = GlobalResource::getInstance()->parseBaseNameWithoutPath(p->problemName);
		GlobalResource::getInstance()->setBaseName(baseName);

		//*************************
		//TODO: parse the problem
		//	long int clk_tck = sysconf(_SC_CLK_TCK);
		//	struct tms now1, now2;
		//	float utime, stime;

		printf("\nLoading the model ...\n  ");

		//Parser* parser = new Parser();  

		GlobalResource::getInstance()->PBSolverPrePOMDPLoad();
		applSharedPointer<MOMDP> problem (NULL);
		if(p->hardcodedProblem.length() ==0 )
		{
			problem = ParserSelector::loadProblem(p->problemName, *p);
		}
		else
		{
            cout << "Unknown hard coded problem type : " << p->hardcodedProblem << endl;
            exit(0);
		}

		double pomdpLoadTime = GlobalResource::getInstance()->PBSolverPostPOMDPLoad();
		printf("  loading time : %.2fs \n", pomdpLoadTime);
		GlobalResource::getInstance()->problem = problem;

		//Getting a MDP solutions
		if(p->MDPSolution == true)
		{
			MDPSolution(problem, p);
			return 0;
		}

		if(p->QMDPSolution == true)
		{
			QMDPSolution(problem, p);
			return 0;
		}

		if(p->FIBSolution == true)
		{
			FIBSolution(problem, p);
			return 0;
		}

		if(GlobalResource::getInstance()->benchmarkMode)
		{
			srand(GlobalResource::getInstance()->randSeed);
			GlobalResource::getInstance()->expRewardRecord.resize(GlobalResource::getInstance()->simNum);
		}
		//decide which solver to create
		PointBasedAlgorithm* solver;

		switch (p->strategy)
		{
		case S_SARSOP:
			{
				SARSOP* sarsopSolver = NULL;
				BackupAlphaPlaneMOMDP* lbBackup = new BackupAlphaPlaneMOMDP();
				BackupBeliefValuePairMOMDP* ubBackup = new BackupBeliefValuePairMOMDP();

				sarsopSolver = new SARSOP(problem, p);

				lbBackup->problem = problem;
				sarsopSolver->lowerBoundBackup = lbBackup;

				((BackupAlphaPlaneMOMDP* )(sarsopSolver->lowerBoundBackup))->solver = sarsopSolver;

				ubBackup->problem = problem;
				sarsopSolver->upperBoundBackup = ubBackup;
				solver = sarsopSolver;
			}
			break;

			//case S_FSVI:
			//	solver = new FSVI(problem, p);
			//	break;

			//case S_GES:
			//	if(GlobalResource::getInstance()->migsPathFile != NULL)
			//	{
			//		if(GlobalResource::getInstance()->migsPathFileNum < 0 )
			//		{
			//			GlobalResource::getInstance()->migsPathFileNum = 10;
			//		}
			//		solver = new GES(problem, p, true);
			//	}
			//	else
			//	{
			//		solver = new GES(problem, p);
			//	}
			//	break;

		default:
			assert(0);// should never reach this point
		};

		//solve the problem
		solver->solve(problem);

		cout << endl;

	}

	// Commented out during merge 02102009
	//	catch(bad_alloc &e)
	//	{
	//		if(GlobalResource::getInstance()->solverParams.memoryLimit == 0)
	//		{
	//			cout << "Memory allocation failed. Exit." << endl;
	//		}
	//		else
	//		{
	//			cout << "Memory limit reached. Please try increase memory limit" << endl;
	//		}

	//	}
	//	catch(exception &e)
	//	{
	//		cout << "Exception: " << e.what() << endl ;
	//	}
	

	return 0;
}

*/

/*
#ifdef _MSC_VER
BOOL CtrlHandler( DWORD fdwCtrlType ) 
{ 
	switch( fdwCtrlType ) 
	{ 
		// Handle the interrupt signal. 
	case CTRL_C_EVENT: 
	case CTRL_CLOSE_EVENT: 
	case CTRL_BREAK_EVENT: 
	case CTRL_SHUTDOWN_EVENT: 
	case CTRL_LOGOFF_EVENT:
		if(GlobalResource::getInstance()->solving)
		{
			GlobalResource::getInstance()->userTerminatedG = true;
		}
		else

		{
			exit(1);
		}
		printf("*** Received SIGINT. User pressed control-C. ***\n");

		printf("\nTerminating ...\n");
		fflush(stdout);
		GlobalResource::getInstance()->userTerminatedG = true;
		return( TRUE );

	default: 
		return FALSE; 
	} 
} 

void registerCtrlHanler()
{
	if( SetConsoleCtrlHandler( (PHANDLER_ROUTINE) CtrlHandler, TRUE ) ) 
	{ 
		// Success
	} 
	else 
	{
		// Failed to register... but continue anyway
		printf( "\nERROR: Could not set control handler"); 

	}
}

#else

void sigIntHandler(int sig) 
{

	if(GlobalResource::getInstance()->solving)
	{
		GlobalResource::getInstance()->userTerminatedG = true;
	}
	else
	{
		exit(1);
	}


	printf("*** Received SIGINT. User pressed control-C. ***\n");
	printf("\nTerminating ...\n");
	fflush(stdout);
}

void setSignalHandler(int sig, void (*handler)(int)) 
{
	struct sigaction act;
	memset (&act, 0, sizeof(act));
	act.sa_handler = handler;
	act.sa_flags = SA_RESTART;
	if (-1 == sigaction (sig, &act, NULL)) {
		cerr << "ERROR: unable to set handler for signal "
			<< sig << endl;
		exit(EXIT_FAILURE);
	}


}

#endif
*/

