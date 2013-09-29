#include <string>
#include <stdexcept>
#include <boost/filesystem.hpp>
#include <io_utils.hpp>

int
tmp_fxr(int argc, char **argv)
{
	std::ifstream f1;
	std::ofstream f2;
	io_utils::open_in_file( f1, "/media/natanaso/Data/Stuff/Research/git/icra_2013/ns_shared/penn_nbv_slam/vision_module/data/omap/occ1/oMap_bigbox.txt");
	io_utils::open_out_file( f2, "/media/natanaso/Data/Stuff/Research/git/icra_2013/ns_shared/penn_nbv_slam/vision_module/data/omap/occ1/oMap_bigbox_fx.txt" );
	
						
	if( !f1 )
		throw std::runtime_error("Cannot open file 1\n");
	if( !f2 )
		throw std::runtime_error("Cannot open file 2\n");
		
	int cntr = 0;
	int vp1 = -1;
	std::string st1;
	double sc1;
	
	int dcntr = -1;
	while( (f1 >> vp1) && (f1 >> st1) && (f1 >> sc1) )
	{
		if(( vp1 > dcntr ) && (cntr < 16 ))
		{
			f2 << vp1 << " " << st1 << " " << sc1 << std::endl;
			++cntr;
		}
		
		if( cntr == 16)
		{	
			++dcntr;
			cntr = 0;
		}
	}
	f1.close();
	f2.close();
	
	return 0;
}

int
main(int argc, char **argv)
{
	return tmp_fxr(argc, argv);
}
