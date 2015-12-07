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
#include "tf/tf.h"
#include <tf/transform_listener.h>
#include <random>
#include "path_planning/NextGoal.h"



ros::ServiceClient *map_client;
tf::TransformListener *tf_listener;
geometry_msgs::PoseStamped currentPose;
using std::vector;

double resolution;

int map_height;
int map_width;

//the size of "cells"
int rowSize;
int colSize;

int rowIndex;
int colIndex;

int cells[100][100];

int cellSize = 10;

nav_msgs::OccupancyGrid *map;//, *explored;

nav_msgs::Path path;

//if this point is movable, then return True
bool isCanMove(int row, int col)
{
    // check the "Central Point" is moveable or not
    int compareRow = (cellSize / 2) + row * cellSize;
    int compareCol = (cellSize / 2) + col * cellSize;
    if(map->data[compareRow*map->info.width + compareCol] > 15)
        return true;
    else
        return false;
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

bool isPointFree(int row,int col)
{
  if(row < 0)
    {
  return false;
    }
  if(row >= map->info.height)
    {
  return false;
    }
  if(col < 0)
    {
  return false;
    }
  if(col >= map->info.width)
    {
  return false;
    }
  if(map->data[row*map->info.width + col] <= 15)//  15 is the one
    {
  return false;
    }
  else
    {
  //explored->data[row*explored->info.width + col] = 120;
  return true;
    }
}


geometry_msgs::Pose getPoint(int maxRow, int maxCol)
{
    int mapYsz = abs(maxRow - rowIndex) * cellSize;
    int mapXsz = abs(maxCol - colIndex) * cellSize;

    geometry_msgs::Pose rs;

    bool up,down,right,left;
    if(maxRow - rowIndex < 0)
    {
        up = true;
    }
    else
    {
        down = true;
    }
    if(maxCol - colIndex > 0)
    {
        right = true;
    }
    else
    {
        left = true;
    }


    if(up&right)
    {
        do{
            rs.position.x = currentPose.pose.position.x + mapXsz * (rand()/(RAND_MAX));
            rs.position.y = currentPose.pose.position.y + mapYsz * (rand()/(RAND_MAX));
        }while(!isPointFree((int)(rs.position.x / resolution),(int)(rs.position.y / resolution))); // Make sure this part
    }
    if(up&left)
    {
        do{
            rs.position.x = currentPose.pose.position.x + mapXsz * (rand()/(RAND_MAX));
            rs.position.y = currentPose.pose.position.y - mapYsz * (rand()/(RAND_MAX));
        }while(!isPointFree((int)(rs.position.x / resolution),(int)(rs.position.y / resolution)));
    }
    if(down&right)
    {
        do{
            rs.position.x = currentPose.pose.position.x - mapXsz * (rand()/(RAND_MAX));
            rs.position.y = currentPose.pose.position.y + mapYsz * (rand()/(RAND_MAX));
        }while(!isPointFree((int)(rs.position.x / resolution),(int)(rs.position.y / resolution)));
    }
    if(down&left)
    {
        do{
            rs.position.x = currentPose.pose.position.x - mapXsz * (rand()/(RAND_MAX));
            rs.position.y = currentPose.pose.position.y - mapYsz * (rand()/(RAND_MAX));
        }while(!isPointFree((int)(rs.position.x / resolution),(int)(rs.position.y / resolution)));
    }

    return rs;
}


geometry_msgs::Pose nextExplore(int upRight,int upLeft,int downRight,int downLeft, int rowIndex, int colIndex)
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
   else if(upLeft)
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
   else if(downRight)
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
   else if(downLeft)
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

   geometry_msgs::Pose goal = getPoint(maxRow,maxCol);
   return goal;
}

geometry_msgs::Pose divieFourSections()
{
    //std::cerr<<"Current Point Cell Index: row & col :"<< rowIndex << " , "<< colIndex<<std::endl;
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

    //std::cout<<"Amount of UpRight cells : " << upRight <<std::endl;
    //std::cout<<"Amount of UpLeft cells : " << upLeft <<std::endl;
    //std::cout<<"Amount of DownRight cells : " << downRight <<std::endl;
    //std::cout<<"Amount of DownLeft cells : " << downLeft <<std::endl;
    geometry_msgs::Pose goal = nextExplore(upRight,upLeft,downRight,downLeft, rowIndex, colIndex);
    return goal;
    //printCell();
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


void dividMap(nav_msgs::OccupancyGrid map)
{
    ROS_INFO("Divide Map");

    colSize = map_width / cellSize;
    rowSize = map_height / cellSize;

    //Intialize the map, assign 5 to every cell
    for(int i = 0; i < rowSize; i++)
    {
        for(int j = 0; j < colSize; j++)
        {
            cells[i][j] = 5;
        }
    }

    for(int row = 0; row < rowSize; row++)
    {
        for(int col = 0; col < colSize; col++)
        {

            if(!isCanMove(rowSize - row, col))
            {
                cells[row][col] = 0;
            }
        }
    }
}

void updateCells()
{
    std::string TargetFrameName = "/map";
    std::string CurrentFrame = "/base_link";

    geometry_msgs::PoseStamped p;
    p.header.frame_id = CurrentFrame;
    p.header.stamp = ros::Time(0);
    p.pose.position.x = 0.0;
    p.pose.position.y = 0.0;
    p.pose.position.z = 0.0;
    p.pose.orientation.x = 0.0;
    p.pose.orientation.y = 0.0;
    p.pose.orientation.z = 0.0;
    p.pose.orientation.w = 1.0;

    try
      {
        tf_listener->waitForTransform(TargetFrameName, CurrentFrame, ros::Time(0), ros::Duration(1.0) );
        tf_listener->transformPose(TargetFrameName,p,currentPose);
      }
    catch(tf::TransformException &ex)
      {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }

    int currentCOl = currentPose.pose.position.x / resolution;
    int currentROW = currentPose.pose.position.y / resolution;

    //get cell index
    for(int i = 0; i < rowSize; i++)
    {
        if(cellSize * (i+1) > currentROW)
        {
            rowIndex = rowSize - i;
            break;
        }
    }
    for(int i = 0; i < colSize; i++)
    {
        if(cellSize * (i+1) > currentCOl)
        {
            colIndex = i;
            break;
        }
    }
    cells[rowIndex][colIndex] = 1;
    //printCell();
}

geometry_msgs::Pose getGoalPose()
{
    geometry_msgs::Pose goal = divieFourSections();
    return goal;
}

bool NextGoal(path_planning::NextGoal::Request &req, path_planning::NextGoal::Response &res)
{
  res.goal = getGoalPose();
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_explorer");
  ros::NodeHandle handle;

  //Set map;
  map = new nav_msgs::OccupancyGrid();

  map->info.height = 0;
  map->info.width = 0;

  //Wait 8 s for the map service.
  if(!ros::service::waitForService("/map_node/get_map", 8000))
    {
      ROS_ERROR("Map service unreachable.");
      return -1;
    }

  map_client = new ros::ServiceClient();
  *map_client = handle.serviceClient<map_tools::GetMap>("/map_node/get_map");
  ros::ServiceServer goal_srv = handle.advertiseService("/nextExplorePose", NextGoal);
  updateMap();
  tf_listener = new tf::TransformListener();
  dividMap(*map);
  ros::Rate loop_rate(2);

  while (ros::ok())
    {
      updateMap();
      updateCells();
      ros::spinOnce();
      loop_rate.sleep();
    }

  return 0;
}
