// Benjamin Cohen
// Test file for BFS3D class

#include <iostream>
#include <sbpl_arm_planner/bfs_3d.h>

using namespace std;

int main(int argc, char *argv[])
{
  if(argc < 4)
  {
    printf("Error: Goal must be passed as parameter\n");
    return 0;
  }

  std::vector<short unsigned int> goal(3,0);
  BFS3D bfs(100,100,100,4,10);

  bfs.init();

  goal[0] = atoi(argv[1]);
  goal[1] =  atoi(argv[2]);
  goal[2] = atoi(argv[3]);


  if(!bfs.setGoal(goal))
  {
    printf("Error setting goal. Please input a valid goal location.\n");
    return 0;
  }
  
  if(!bfs.runBFS())
  {
    printf("Running BFS failed.\n");
    return 0;
  }

//  bfs.printCostToGoal();


  std::vector<short unsigned int> start(3,0);
  start[0] = 0;
  start[1] = 3;
  start[2] = 5;
  std::vector<std::vector<int> > path;
  printf("Calling getShortestPath()\n");

  bfs.getShortestPath(start,path);
  
  printf("Returned from getShortestPath()\n");
  
  return 1;
}

