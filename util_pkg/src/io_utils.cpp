#include <iostream>
#include <stdexcept>
#include <vector>

#include "io_utils.hpp"

void
io_utils::file2matrix(const std::string &file, Eigen::MatrixXd &mat, int cols )
{
	std::ifstream in;

	open_in_file( in, file );

	if(!in){
		std::cout << "[io_utils] Cannot open file: " << file << std::endl;
		throw std::runtime_error("Runtime error...\n");
	}
	
	std::vector<double> data;
	double val;
	while ( in >> val ){
		data.push_back( val );
	}

	// copy the vector to an eigen matrix
	int rows = data.size() / cols;
	mat.resize( rows, cols );
	std::copy( data.data(), data.data()+data.size(), mat.data() );
	
	for(int r = 0; r < rows; ++r)
		for(int c = 0; c < cols; ++c)
		{
			mat(r,c) = data[cols*r + c];
		} 

	in.close();
}

int io_utils_test(int argc, char **argv)
{
	return 0;
}

/*
int main(int argc, char **argv)
{
	return io_utils_test(argc, argv);
}
*/
