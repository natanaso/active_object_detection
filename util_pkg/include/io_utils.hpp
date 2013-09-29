#ifndef IO_UTILS_HPP_
#define IO_UTILS_HPP_


#include <fstream>
#include <string>

#include <Eigen/Core>


class io_utils
{
public:
	static std::ofstream& open_out_file( std::ofstream &fsr, const std::string &file)
	{
		// put file in valid state first
		fsr.close();	// close in case it was already open
		fsr.clear();	// clear any existing errors
		fsr.open( file.c_str() );
		return fsr;
	}
	
	static std::ifstream& open_in_file( std::ifstream &fsr, const std::string &file)
	{
		// put file in valid state first
		fsr.close();	// close in case it was already open
		fsr.clear();	// clear any existing errors
		fsr.open( file.c_str() );
		return fsr;
	}
	
	static void file2matrix( const std::string &file, Eigen::MatrixXd &mat, int cols );
};

#endif
