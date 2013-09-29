#include "vision_module/vtree_user.hpp"

/**
 * This file depends on the ros parameter server and expects that the following parameters are set:
 * 	<param  name="command" type="string" value="/init" />
 *	<param  name="clouds_folder" type="string" value="path/to/traindata" />
 *	<param  name="database_location" type="string" value="path/to/database" />
 *	<param  name="tree_k" type="int" value="5" />
 *  <param  name="tree_levels" type="int" value="5" />
 */
int build_tree_main(int argc, char **argv)
{
	ros::init(argc, argv, "build_tree");
	ros::NodeHandle nh;
	/*
	// For debugging purposes
	std::string command;
	node_handle.param ("command", command, std::string("/load"));
	std::cout << "command = " << command << std::endl;
	*/
	
	ros_vtree_user vt_user( nh );
	//std::cout << "tree object constructed!" << std::endl;
	
	return vt_user.start();
}

int main(int argc, char **argv)
{
	return build_tree_main( argc, argv );
}
