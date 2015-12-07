#include <unordered_map>
#include "ros/ros.h"
#include <limits>
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Path.h"
#include "math.h"
#include "nav_msgs/OccupancyGrid.h"
#include "map_tools/GetMap.h"
#include "path_planning/GetPath.h"


ros::ServiceClient *map_client;

using std::vector;

double resolution;

int map_height;
int map_width;

//the size of "cells"
int rowSize;
int colSize;

int cells[100][100];
int reversedCells[100][100];

int cellSize = 10;

nav_msgs::OccupancyGrid *map;//, *explored;

nav_msgs::Path path;

void validation(int upRight,int upLeft,int downRight,int downLeft)
{
    int totalZero = 0;
    for(int i = 0; i < rowSize; i++)
    {
        for(int j = 0; j < colSize; j++)
        {
            if(reversedCells[0][0] == 0)
            {
                totalZero++;
            }
        }
    }

    std::cout<<"Total Zero is :" << totalZero << std::endl;
    std::cout<<"Result is :"<<upRight+upLeft+downRight+downLeft<<std::endl;
}


void reverseUpAndDown()
{
    //for(int i = rowSize - 1; i > 0; i--)
    for(int i = 0; i < rowSize ; i++)
    {
        for(int j = 0; j < colSize; j++)
        {
            reversedCells[i][j] = cells[rowSize - i][j];
        }
    }
}

void nextExplore(int upRight,int upLeft,int downRight,int downLeft, int rowIndex, int colIndex)
{
    ROS_INFO("Next Explore");
   bool upright = false;
   bool upleft = false;
   bool downright = false;
   bool downleft = false;
   if(upRight > upLeft && upRight > downLeft && upRight > downRight)
   {
       upright = true;
   }
   else if(upLeft > upRight && upLeft > downLeft && upLeft > downRight)
   {
       upleft = true;
   }
   else if(upLeft > upRight && upLeft > downLeft && upLeft > downRight)
   {
       downright = true;
   }
   else if(upLeft > upRight && upLeft > downLeft && upLeft > downRight)
   {
       downleft = true;
   }
   //Default UpRight
   else
   {
        upright = true;
   }

   int maxDistance = 0;
   int maxPosition = 0;
   int maxRow = 0;
   int maxCol = 0;
   if(upright)
   {
       //find the furtherest cell in the up-Right direction
        for(int i = rowIndex; i > 0; i--)
        {
            for(int j = colIndex; j < colSize; j++)
            {
                if(cells[i][j] == 0)
                {
                    maxPosition = abs(rowIndex - i) + abs(j - colIndex);
                    if(maxPosition > maxDistance)
                    {
                        maxDistance = maxPosition;
                        maxRow = i;
                        maxCol = j;
                    }
                }
            }
        }
   }

   if(upLeft)
   {
       //find the furtherest cell in the up-Right direction
        for(int i = rowIndex; i > 0; i--)
        {
            for(int j = colIndex; j > 0; j--)
            {
                if(cells[i][j] == 0)
                {
                    maxPosition = abs(rowIndex - i) + abs(j - colIndex);
                    if(maxPosition > maxDistance)
                    {
                        maxDistance = maxPosition;
                        maxRow = i;
                        maxCol = j;
                    }
                }
            }
        }
   }

   if(downRight)
   {
       //find the furtherest cell in the up-Right direction
        for(int i = rowIndex; i < rowSize; i++)
        {
            for(int j = colIndex; j < colSize; j++)
            {
                if(cells[i][j] == 0)
                {
                    maxPosition = abs(rowIndex - i) + abs(j - colIndex);
                    if(maxPosition > maxDistance)
                    {
                        maxDistance = maxPosition;
                        maxRow = i;
                        maxCol = j;
                    }
                }
            }
        }
   }

   if(downLeft)
   {
       //find the furtherest cell in the up-Right direction
        for(int i = rowIndex; i < rowSize; i++)
        {
            for(int j = colIndex; j > 0; j--)
            {
                if(cells[i][j] == 0)
                {
                    maxPosition = abs(rowIndex - i) + abs(j - colIndex);
                    if(maxPosition > maxDistance)
                    {
                        maxDistance = maxPosition;
                        maxRow = i;
                        maxCol = j;
                    }
                }
            }
        }
   }
   std::cout<<"Up Right furtherest ROW and COL : "<< maxRow << " , "<<maxCol<<std::endl;

}


void printCell()
{
    for(int i = 0; i < rowSize; i++)
    {
        for(int j = 0; j < colSize; j++)
        {
            std::cout<< cells[i][j] <<" ";
        }
        std::cout<<""<<std::endl;
    }
}


bool updateMap(void)
{
  map_tools::GetMap srv2;
  srv2.request.type.data = "inflated";  //Check the name
  if (map_client->call(srv2)){
    *map = srv2.response.map;
    map_height = map->info.height;
    map_width = map->info.width;
    resolution = map->info.resolution;
  }else{
    ROS_ERROR("Failed to call service GetMap (inflated).");
    return false;
  }
  return true;
}

//if this point is movable, then return True
bool isCanMove(int row, int col)
{
    // check the "Central Point" is moveable or not
    int compareRow = (cellSize / 2) + row * cellSize;
    int compareCol = (cellSize / 2) + col * cellSize;
//    std::cout<< "ComePareRow : "<< compareRow<< std::endl;
//    std::cout<< "ComePareCol : "<< compareCol<< std::endl;
    if(map->data[compareRow*map->info.width + compareCol] > 15)
        return true;
    else
        return false;
}

void divieFourSections()
{
    ROS_INFO("Divid Four Sections");
    int finalPointX =  path.poses.back().pose.position.x / resolution;
    int finalPointY =  path.poses.back().pose.position.y / resolution;
    int rowIndex;
    int colIndex;
    //get cell index
    for(int i = 0; i < rowSize; i++)
    {
        if(cellSize * i > finalPointY)
        {
            rowIndex = rowSize - i;
            break;
        }
    }
    for(int i = 0; i < colSize; i++)
    {
        if(cellSize * i > finalPointX)
        {
            colIndex = i;
            break;
        }
    }

    //cells[rowIndex][colIndex] = 2;
    //printCell();

    std::cerr<<"Final Point Cell Index: row & col :"<< rowIndex << " , "<< colIndex<<std::endl;

    //count the empty cells
    int upRight = 0;
    int upLeft = 0;
    int downRight = 0;
    int downLeft = 0;
    //Down right section
    for(int i = rowIndex; i < rowSize ; i++)
    {
        for(int j = colIndex; j < colSize; j++)
        {
            if(cells[i][j] == 0)
                downRight++;
        }
    }
    //Down left section
    for(int i = rowIndex; i < rowSize ; i++)
    {
        for(int j = colIndex; j > 0; j--)
        {
            if(cells[i][j] == 0)
                downLeft++;
        }
    }
    //Up Left section
    for(int i = rowIndex; i > 0 ; i--)
    {
        for(int j = colIndex; j > 0; j--)
        {
            if(cells[i][j] == 0)
                upLeft++;
        }
    }
    //Up right section
    for(int i = rowIndex; i > 0 ; i--)
    {
        for(int j = colIndex; j < colSize; j++)
        {
            if(cells[i][j] == 0)
                upRight++;
        }
    }


    std::cout<<"Amount of UpRight cells : " << upRight <<std::endl;
    std::cout<<"Amount of UpLeft cells : " << upLeft <<std::endl;
    std::cout<<"Amount of DownRight cells : " << downRight <<std::endl;
    std::cout<<"Amount of DownLeft cells : " << downLeft <<std::endl;
    nextExplore(upRight,upLeft,downRight,downLeft, rowIndex, colIndex);
    validation(upRight,upLeft,downRight,downLeft);
}

// Use the path to update the "Cells"
void updateCells(geometry_msgs::Pose first, geometry_msgs::Pose second)
{
    ROS_INFO("Update Cells");
    // If this line is at the Row (horzontal), then set changeRowcells to be true
    // changeRowCells TRUE means we only changes the row cells values, !!horizontially
    bool changeRowCells;

    if(first.position.x != second.position.x && first.position.y == second.position.y)
    {
        ROS_INFO("The path is on the Horizontal");
        changeRowCells = true;
    }
    else if(first.position.x == second.position.x && first.position.y != second.position.y)
    {
        ROS_INFO("The path is on the vertical");
        changeRowCells = false;
    }
    else
    {
        ROS_ERROR("ERROR: Cannot use this, the path is not stictly Vertical or Hortizal");
        changeRowCells = false;
    }

    //Chaneg col cell value
    if(changeRowCells)
    {
        // startCellIndex is the first cell of the first.position.x
        int startColCellIndex;
        int endColCellIndex;

        int rowCellIndex;

        std::cout<<"First Position X : "<< first.position.x / resolution <<std::endl;
        std::cout<<"Second Position X : "<< second.position.x / resolution <<std::endl;

        // Get col cells indice
        for(int i = 0; i < colSize; i++)
        {
            if(cellSize * i > first.position.x / resolution)
            {
                startColCellIndex = i;
                break;
            }
        }
        for(int i = 0; i < colSize; i++)
        {
            if(cellSize * i > second.position.x / resolution)
            {
                endColCellIndex = i;
                break;
            }
        }

        // Get col cell index
        for(int i = 0; i < rowSize; i++)
        {
            if(cellSize * i > first.position.y / resolution)
            {
                rowCellIndex = i;
                break;
            }
        }
//        std::cout<<"StartColCellIndex : "<< startColCellIndex <<std::endl;
//        std::cout<<"EndColCellIndex : "<< endColCellIndex <<std::endl;
//        std::cout<<"RowCellIndex : "<< rowCellIndex <<std::endl;

        // If start value is larger than the end value, reverse them
        int middle = 0;
        if(startColCellIndex > endColCellIndex)
        {
            middle = endColCellIndex;
            endColCellIndex = startColCellIndex;
            startColCellIndex = middle;
        }

        while(startColCellIndex < endColCellIndex)
        {
            cells[rowSize - rowCellIndex][startColCellIndex] = 1;
            startColCellIndex++;
        }
    }
    else
    {
        // startCellIndex is the first cell of the first.position.y
        int startRowCellIndex;
        int endRowCellIndex;

        int colCellIndex;

        std::cout<<"First Position Y : "<< first.position.y / resolution <<std::endl;
        std::cout<<"Second Position Y : "<< second.position.y / resolution <<std::endl;

        // Get col cells indice
        for(int i = 0; i < rowSize; i++)
        {
            if(cellSize * i > first.position.y / resolution)
            {
                startRowCellIndex = i;
                break;
            }
        }
        for(int i = 0; i < rowSize; i++)
        {
            if(cellSize * i > second.position.y / resolution)
            {
                endRowCellIndex = i;
                break;
            }
        }

        // Get col cell index
        for(int i = 0; i < colSize; i++)
        {
            if(cellSize * i > first.position.x / resolution)
            {
                colCellIndex = i;
                break;
            }
        }


        std::cout<<"StartRowCellIndex : "<< startRowCellIndex <<std::endl;
        std::cout<<"EndRowCellIndex : "<< endRowCellIndex <<std::endl;
        std::cout<<"ColCellIndex : "<< colCellIndex <<std::endl;

        // If start value is larger than the end value, reverse them
        int middle = 0;
        if(startRowCellIndex > endRowCellIndex)
        {
            middle = endRowCellIndex;
            endRowCellIndex = startRowCellIndex;
            startRowCellIndex = middle;
        }


        while(startRowCellIndex < endRowCellIndex)
        {
            cells[rowSize - startRowCellIndex][colCellIndex] = 1;
            startRowCellIndex++;
        }
    }
    //reverseUpAndDown();
    printCell();
    divieFourSections();
}

//Receives the path
void setPath(const nav_msgs::Path::ConstPtr& msg)
{
    path.poses = msg.get()->poses;
    int pathSize = msg.get()->poses.size();
    std::cout<< "Path size :"<< pathSize << std::endl;
    //TODO - when pathSize is one
    for(int i = 0; i < pathSize - 1; i++)
    {
        int j = i + 1;
        updateCells(msg.get()->poses.at(i).pose, msg.get()->poses.at(j).pose);
    }
}

void dividMap(nav_msgs::OccupancyGrid map)
{
    ROS_INFO("Divid Map");
    // divide Map into several cells
    // size plus one, right?
//    colSize = 1 + map_width / cellSize;
//    rowSize = 1 + map_height / cellSize;
    colSize = map_width / cellSize;
    rowSize = map_height / cellSize;
    std::cout<<"Row Cell Size : " << rowSize << std::endl;
    std::cout<<"Col Cell Size : " << colSize << std::endl;
    //Intialize the map, assign 5 to every cell
    for(int i = 0; i < rowSize; i++)
    {
        for(int j = 0; j < colSize; j++)
        {
            //
            cells[i][j] = 5;
            reversedCells[i][j] = 5;
        }
    }
    //cells[0][0] is the cell in the left-top cell, row-col value is (map_height,0)
    //Check if this cell is movable or not
    //int visitedCount = 0;
    for(int row = 0; row < rowSize; row++)
    {
        for(int col = 0; col < colSize; col++)
        {
            // Check the wrong "Position" actually

            //The reason why we use (rowSize - row) is because
            if(!isCanMove(rowSize - row, col))
            {
                cells[row][col] = 0;
            }
        }
    }
    printCell();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_explorer");
  ros::NodeHandle handle;

  //Set map;
  map = new nav_msgs::OccupancyGrid();

  map->info.height = 0;
  map->info.width = 0;

  //it should publish the goal pose
  ros::Publisher pub_goal_pose = handle.advertise<geometry_msgs::Pose>("/goal_pose",10);
  ros::Subscriber sub_path = handle.subscribe<nav_msgs::Path>("/Astar/Simplepath",10,setPath);

  //Wait 8 s for the map service.
  if(!ros::service::waitForService("/map_node/get_map", 8000))
    {
      ROS_ERROR("Map service unreachable.");
      return -1;
    }

  map_client = new ros::ServiceClient();
  *map_client = handle.serviceClient<map_tools::GetMap>("/map_node/get_map");
  updateMap();
  dividMap(*map);
  ros::Rate loop_rate(2);


  while (ros::ok())
    {
      updateMap();
      ros::spinOnce();
      loop_rate.sleep();
    }

  return 0;
}
