#include <string>
#include <stdexcept>
#include <boost/filesystem.hpp>
#include <io_utils.hpp>

int
fix_omap(int argc, char **argv)
{
	typedef boost::filesystem3::directory_iterator directory_iterator;
	// Get file paths
	if(argc < 4)
		throw std::runtime_error("Path to second occluded oMap directory is required as third arg\n");
			
	if(argc < 3)
		throw std::runtime_error("Path to occluded oMap directory is required as second arg\n");
		
	if(argc < 2)
		throw std::runtime_error("Path to oMap directory is required as first arg\n");
	
	std::string dir_omap(argv[1]);
	std::string dir_omap_occ(argv[2]);
	std::string dir_omap_occ_2(argv[3]);
	
	for (directory_iterator itr(dir_omap); itr!=directory_iterator(); ++itr)
   {
   	// Get first txt file
   	std::string curr_ext(itr->path().extension().string());
   	if( curr_ext.compare(".txt") == 0 )
   	{
   		std::string curr_stem( itr->path().stem().string() );
   		std::string curr_name( itr->path().filename().string() );
   		
   		for (directory_iterator itr2(dir_omap_occ); itr2!=directory_iterator(); ++itr2)
   		{
   			std::string curr_occ_name( itr2->path().filename().string() );
   			if( curr_occ_name.compare(curr_name) == 0)
   			{
   				for (directory_iterator itr3(dir_omap_occ_2); itr3!=directory_iterator(); ++itr3)
   				{
   					std::string curr_occ_2_name( itr3->path().filename().string() );
   					if( curr_occ_2_name.compare(curr_name) == 0)
   					{
   					
   					   std::cout << curr_occ_name << std::endl;
   				
							// Open files
							std::ifstream f1, f2, f3;
							std::ofstream f4;
							io_utils::open_in_file( f1, itr->path().string() );
							io_utils::open_in_file( f2, itr2->path().string() );
							io_utils::open_in_file( f3, itr3->path().string() );
							io_utils::open_out_file( f4, "./" + curr_stem + "_combined.txt" );
						
							if( !f1 )
								throw std::runtime_error("Cannot open oMap file " + curr_stem + "\n");
							if( !f2 )
								throw std::runtime_error("Cannot open occ file " + curr_stem + "\n");
							if( !f2 )
								throw std::runtime_error("Cannot open occ2 file " + curr_stem + "\n");								
							if( !f4 )
								throw std::runtime_error("Cannot open out file...\n");
							
							// Combine them
							int cntr = 0;
							int vp1 = -1, vp2 = -1, vp3 = -1;
							std::string st1, st2, st3;
							double sc1, sc2, sc3;
					
							while( (f1 >> vp1) && (f1 >> st1) && (f1 >> sc1) )
							{
								if( vp1 == cntr )
									f4 << vp1 << " " << st1 << " " << sc1 << std::endl;
								else
								{
									if(vp2 != -1)
									{
										f4 << vp2 << " " << st2 << " " << sc2 << std::endl;
									}

									while( (f2 >> vp2) && (f2 >> st2) && (f2 >> sc2) )
									{
									
										if(vp2 == cntr)
											f4 << vp2 << " " << st2 << " " << sc2 << std::endl;
										else
										{
											if(vp3 != -1)
											{
												f4 << vp3 << " " << st3 << " " << sc3 << std::endl;
											}
											
											while( (f3 >> vp3) && (f3 >> st3) && (f3 >> sc3) )
											{
												if(vp3 == cntr)
													f4 << vp3 << " " << st3 << " " << sc3 << std::endl;
												else
													break;
											}
											
											break;
										}
									}
							
									f4 << vp1 << " " << st1 << " " << sc1 << std::endl;					
									++cntr;
								}
							}
					
					
							f4 << vp2 << " " << st2 << " " << sc2 << std::endl;

							while( (f2 >> vp2) && (f2 >> st2) && (f2 >> sc2) )
							{						
								f4 << vp2 << " " << st2 << " " << sc2 << std::endl;
							}
							
							f4 << vp3 << " " << st3 << " " << sc3 << std::endl;
							while( (f3 >> vp3) && (f3 >> st3) && (f3 >> sc3) )
							{						
								f4 << vp3 << " " << st3 << " " << sc3 << std::endl;
							}							
					
							// close the files
							f1.close();
							f2.close();
							f3.close();
							f4.close();
   						
   					}
   					
   				}
   				

   			}
   		}
   	}
  	}
  	  	

	return 0;
}

int
main(int argc, char **argv)
{
	return fix_omap(argc, argv);
}
