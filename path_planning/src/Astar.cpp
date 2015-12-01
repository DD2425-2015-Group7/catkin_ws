#include "vector"
#include <queue>
#include <unordered_map>
#include "ros/ros.h"
#include <limits>
#include "std_msgs/String.h"
#include <tf/transform_listener.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Path.h"
#include "math.h"
#include "nav_msgs/OccupancyGrid.h"
#include "map_tools/GetMap.h"
#include "path_planning/GetPath.h"
#include <tf/transform_listener.h>
#include "tf/tf.h"

ros::ServiceClient *map_client;

using std::vector;

double cell_size;

int turnningCost = 300;

int G_OFFSET_4 = 1;

nav_msgs::OccupancyGrid *map, *explored;
geometry_msgs::PoseStamped startPose;
tf::TransformListener *tf_listener;

bool updateMap(void)
{
    map_tools::GetMap srv2;
    srv2.request.type.data = "distance_obj";
    if (map_client->call(srv2)){
        *map = srv2.response.map;
        cell_size = map->info.resolution;
    }else{
        ROS_ERROR("Failed to call service GetMap (distance_obj).");
        return false;
    }
    return true;
}


struct Node
{
    int row;
    int col;
    int parent;

    double g; //past road cost
    double h;

    double oAcc;
    double o; // O stands for the distance between the current point and the nearest obstacle
    double t; // T stands for the turn cost. This value will be very high, if the node has to turn.

    double f; //heuristic, f = g + h + o

    Node(int row, int col, int parent){
        this->parent = parent;
        this->row = row;
        this->col = col;
    }
    
    bool operator==(const Node &other) const
      { 
          return (row == other.row && col == other.col);
      }
};

namespace std {
    template <>
    struct hash<Node> {
        public:
        std::size_t operator()(const Node& a) const
        {
            std::size_t hash = 13;
            hash += a.row * 7;
            hash *= 3;
            hash += a.col * 17;
            hash *= 5;
            return hash;
        }
    };
}

class CompareNodes {
    public:
    bool operator()(Node& a, Node& b) // Returns true if a is before b
    {
       return (a.f > b.f);
    }
};



class PathFinder
{
    private:
        Node *start;
        Node *goal;
        vector<Node> closeSet;

    public:
    PathFinder(Node node1, Node node2)
    {
        start = new Node(node1.row,node1.col,node1.parent);
        goal = new Node(node2.row,node2.col,node2.parent);
        //ROS_INFO("PathFinder Intialized");
    }

    int hValue(Node current)
    {
        int h = abs(goal->col - current.col) + abs(goal->row - current.row);

        return h;
    }

    int oValue(Node current)
    {
        // As for the toal f value, the smaller, the better, while the obstacle of distance should be the larger the better, so I reverse this value
        int o = (50 - (map->data[current.row*map->info.width + current.col]));
        assert(o >= 0);
        assert(o <= 50);
        o = o * 10;
        //int o = - map->data[current.row*map->info.width + current.col];
        //double o = (50 - (map->data[current.row*map->info.width + current.col]))/5;
        //std::cout<< "O value is  : "<< o << std::endl;
        return o;
    }

    int getBestChild(vector<Node> nodelist)
    {
        //std::cout<< "getBestChild size  : "<< nodelist.size() << std::endl;
        assert(nodelist.size()>0);
        int bestValue = nodelist[0].f;
        int bestNum = 0;
        for(int i = 0; i < nodelist.size(); i++)
        {
            //std::cout<< " F value : "<< nodelist.at(i).f << std::endl;
            if(nodelist.at(i).f < bestValue)
            {
                bestValue = nodelist.at(i).f;
                bestNum = i;
                //std::cout<< "Best F value : "<< bestValue << std::endl;
                //std::cout<< "Best Numb  : "<< bestNum << std::endl;
            }
        }
        //std::cout << "Best Child : ( "<< nodelist.at(bestNum).row << " , "<<nodelist.at(bestNum).col <<" )" <<std::endl;
        //std::cout << " ********   Best Child G value "<< nodelist.at(bestNum).g <<std::endl;
        return bestNum;
    }
    //if this point is passable, then return TRUE, if it is occupied, return false
    bool isCanMove(int row,int col)
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
            explored->data[row*explored->info.width + col] = 120;
            return true;
        }
    }

    vector<Node> getNeighbours(Node &current, int index)
    {
        int row = current.row;
        int col = current.col;

        vector<Node> neighbours;

        Node *previousNode;
        // "turnToUpDown" means this node want to move to UP or DOWN.
        // If the previous two points move in horiztal direction, then this should be true
        // And it this value is true, when this value want to turn up or down, this will return a very high cost
        bool turnToUpDown;
        bool turnToRightLeft;
        assert(index < closeSet.size());

        if(current.parent >= 0)
        {
            assert(current.parent < closeSet.size());
            previousNode = &(closeSet.at(current.parent)); //come_from[current.row][current.col];
            // Move in horiztal means only col changes
            if(previousNode->col != current.col && previousNode->row == current.row)
            {
                //If it wants to move Up or Down, then return a very high cost
                turnToUpDown = true;
            }
            else
            {
                turnToUpDown = false;
            }

            if(previousNode->col == current.col && previousNode->row != current.row)
            {
                //If it wants to move Right or Left, then return a very high cost
                turnToRightLeft = true;
            }
            else
            {
                turnToRightLeft = false;
            }
        }
        else
        {
            //The start point.
            turnToRightLeft = false;
            turnToUpDown = false;
        }


        //std::cout << "Current Row & Col is : "<< current.row <<" & "<<current.col<<std::endl;
        //std::cout << "Previous Row & Col is : "<< previousNode.row <<" & "<<previousNode.col<<std::endl;

        //std::cout << "turnToUpDown : "<< turnToUpDown<<std::endl;
        //std::cout << "turnToRightLeft : "<< turnToRightLeft<<std::endl;

        //Up
        if(isCanMove(row+1,col))
        {            
            Node newNode(row+1, col, index);
            newNode.t = current.t;
            if(turnToUpDown)
            {
                newNode.t += turnningCost;
            }
            neighbours.push_back(newNode);
        }
        //Down
        if(isCanMove(row-1,col))
        {
            Node newNode(row-1, col, index);
            newNode.t = current.t;
            if(turnToUpDown)
            {
                newNode.t += turnningCost;
            }
            neighbours.push_back(newNode);
        }
        //Right
        if(isCanMove(row,col+1))
        {
            Node newNode(row, col+1, index);
            newNode.t = current.t;
            if(turnToRightLeft)
            {
                newNode.t += turnningCost;
            }
            neighbours.push_back(newNode);
        }
        //Left
        if(isCanMove(row,col-1))
        {
            Node newNode(row, col - 1, index);
            newNode.t = current.t;
            if(turnToRightLeft)
            {
                newNode.t += turnningCost;
            }
            neighbours.push_back(newNode);
        }
        return neighbours;
    }

    //if this nodeList has this value, then return "true"
    bool itHas(vector<Node> nodeList, Node node)
    {
        for(int i = 0; i < nodeList.size(); i++)
        {
            if(node.row == nodeList.at(i).row && node.col == nodeList.at(i).col)
            {
                return true;
            }
        }
        return false;
    }

    int getPosition(vector<Node> nodeList, Node node)
    {
        for(int i = 0; i < nodeList.size(); i++)
        {
            if(node.row == nodeList.at(i).row && node.col == nodeList.at(i).col)
            {
                return i;
            }
        }
        return -10;
    }

    //if the position of start or goal is invalid, then we return TRUE
    bool inValid(Node start, Node goal)
    {
        //std::cout << "Start Row & Col is : "<< start.row <<" & "<<start.col<<std::endl;
        //std::cout << "Goal Row & Col is : "<< goal.row <<" & "<<goal.col<<std::endl;
        //std::cout << "Map Height & Width is : "<< map->info.width <<" & "<<map->info.height<<std::endl;
        if(start.row >= map->info.height || start.col >= map->info.width ||
                goal.row >= map->info.height || goal.col >= map->info.width ||
                !isCanMove(start.row,start.col) || !isCanMove(goal.row,goal.col))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    nav_msgs::Path getPath()
    {
        //if this two point is invalid, then break
        if(inValid(*start,*goal))
        {
            ROS_INFO("Start or Goal is not valid");
            nav_msgs::Path nu;
            return nu;
        }
        
        std::priority_queue <Node, std::vector<Node>, CompareNodes > openSet;
        std::unordered_map<Node, Node> openMap;
        
        std::unordered_map<Node, Node> realCloseMap;
        closeSet.clear();

        start->g = 0;
        //std::cout<< "start g : "<< start.g <<std::endl;
        start->h = hValue(*start);
        //std::cout<< "start h : "<< start.h <<std::endl;
        start->oAcc = oValue(*start);
        start->o = oValue(*start);
        //std::cout<< "start o : "<< start.o <<std::endl;
        start->t = 0;
        start->f = start->g + start->o + start->h + start->t;// start.t + start->h;
        //std::cout<< "start f : "<< start.f <<std::endl;
        //come_from[start.row][start.col] = start;

        openSet.push(*start);
        std::pair<Node,Node> node (*start, *start);
        openMap.insert(node);

        while(!openSet.empty())
        {
            Node current = openSet.top();
            openSet.pop();
            openMap.erase(current);

            closeSet.push_back(current);
            std::pair<Node,Node> node (current, current);
            realCloseMap.insert(node);
            
            if(current.row == goal->row && current.col == goal->col)
            {
                    ROS_INFO("Astar Destinaltion reached");
                    realCloseMap.erase(realCloseMap.begin(), realCloseMap.end());
                    openMap.erase(openMap.begin(), openMap.end());
                    //openSet.erase(openSet.begin(), openMap.end());
                    //realCloseMap.~unordered_map();
                    //openMap.~unordered_map();
                    //openSet.~priority_queue();
                    return reconstruct(current);
            }
            
            assert(closeSet.size() > 0);
            vector<Node> neighbours = getNeighbours(closeSet.back(),closeSet.size() -1 );
            
            for(int i = 0; i < neighbours.size(); i++)
            {
                Node child = neighbours.at(i);
                child.g = current.g + G_OFFSET_4;
                child.h = hValue(child);
                child.oAcc = current.oAcc + oValue(child);
                child.o =  child.oAcc; //(child.oAcc / child.g) * 10; //child.oAcc;
                child.f = child.g + child.o + child.h + child.t;
                
                std::unordered_map<Node,Node>::const_iterator gotClose = realCloseMap.find (child);
                if(gotClose != realCloseMap.end()){
                    if(child.f < gotClose->second.f){
                        realCloseMap.erase(gotClose);
                    }else{
                        continue;
                    }
                }
                
                std::unordered_map<Node,Node>::const_iterator got = openMap.find (child);
                if(got != openMap.end()){
                    //got->first
                    //TODO
                }else{
                    openSet.push(child);
                    std::pair<Node,Node> node (child, child);
                    openMap.insert(node);
                }
                
                
         }
     }
                    
        nav_msgs::Path nu;
        return nu;
        
    }
    nav_msgs::Path reconstruct(Node current)
    {
        vector<Node> path;
        nav_msgs::Path finalPath;

        //path.push_back(current);
        std::cout<<"Reconstruct Starts "<<std::endl;
        while(current.parent >= 0 )
        {
            path.push_back(closeSet.at(current.parent));
            current = closeSet.at(current.parent);
            //std::cout << "Parent's Point Positionv is : ( "<< current.row << " , "<< current.col << " )"<<std::endl;
        }
        std::cout<<"Reconstruct Ends "<<std::endl;

        // Should this be just larger than 0 or it could equal 0
        for(int i = path.size() - 1; i >= 0; i--)
        {
            //std::cout<< "Original Path No." << i <<" point is X: "<< path.at(i).col << " Y :" << path.at(i).row<< std::endl;
           std::cout << "Path No."<< i << " Node G & H & T & O & F: ( "<< path.at(i).g << " , "<<path.at(i).h << " , "<<path.at(i).t << " , "<<path.at(i).o << " , "<<path.at(i).f <<" )" <<std::endl;
           geometry_msgs::PoseStamped p;
           //path.x should be the col value, right? TODO: check how the pub_points works. The coordinate must stay the same
           p.pose.position.x = path.at(i).col * cell_size;
           p.pose.position.y = path.at(i).row * cell_size;
           p.pose.orientation.x = 0;
           p.pose.orientation.y = 0;
           p.pose.orientation.z = 0;
           p.pose.orientation.w = 1;

           finalPath.poses.push_back(p);
        }


        finalPath.header.stamp = ros::Time::now();
        finalPath.header.frame_id = "/map";
        /*
        std::cout<<"Original Path Size is: "<< finalPath.poses.size()<<std::endl;
        //Show the Original path

        for(int i = 0; i < finalPath.poses.size(); i++)
        {
            //std::cout<< "Original Path No." << i <<" point is X: "<< finalPath.poses.at(i).pose.position.x << " Y :" << finalPath.poses.at(i).pose.position.y << std::endl;
        }
        */


        return finalPath;
    }

    nav_msgs::Path simpilifyPath(nav_msgs::Path path)
    {
        nav_msgs::Path newPath;

        //This loop is used to store the "corner" point(either x or y value changes)
        //checkY means we will check the y difference for the two points
        bool checkY = true;
        for(int i = 0; i < path.poses.size() - 1; i++)
        {
           int j = i + 1;
           if(checkY)
           {
               if(path.poses.at(i).pose.position.y != path.poses.at(j).pose.position.y )
               {
                   //ROS_INFO("X value changes");
                   geometry_msgs::PoseStamped p;
                   p.pose.position.x = path.poses.at(i).pose.position.x;
                   p.pose.position.y = path.poses.at(i).pose.position.y;
                   p.pose.orientation.x = 0;
                   p.pose.orientation.y = 0;
                   p.pose.orientation.z = 0;
                   p.pose.orientation.w = 1;

                   newPath.poses.push_back(p);
                   checkY = false;
                }
           }
           else
           {
               if(path.poses.at(i).pose.position.x != path.poses.at(j).pose.position.x )
               {
                   //ROS_INFO("X value changes");
                   geometry_msgs::PoseStamped p;
                   p.pose.position.x = path.poses.at(i).pose.position.x;
                   p.pose.position.y = path.poses.at(i).pose.position.y;
                   p.pose.orientation.x = 0;
                   p.pose.orientation.y = 0;
                   p.pose.orientation.z = 0;
                   p.pose.orientation.w = 1;

                   newPath.poses.push_back(p);
                   checkY = true;
                }
           }
        }

            geometry_msgs::PoseStamped p;
            p.pose.position.x = goal->col * cell_size;
            p.pose.position.y = goal->row * cell_size;
            p.pose.orientation.x = 0;
            p.pose.orientation.y = 0;
            p.pose.orientation.z = 0;
            p.pose.orientation.w = 1;
            newPath.poses.push_back(p);

        //Show the new path(corner Point)
        //std::cout<<"Simple Path Size is: "<< newPath.poses.size()<<std::endl;
        //std::cout<<"Simple Path Last point : "<< newPath.poses.back().pose.position.x<< " " <<newPath.poses.back().pose.position.y<<std::endl;


        /*
        nav_msgs::Path finalPath;
        //Simplify the path further (ignore the point that is too close)
        int start = 0;
        int end = start + 1;
        //Add the firstPoint (start Point) to the path
        geometry_msgs::PoseStamped p;
        p.pose.position.x = newPath.poses.at(0).pose.position.x;
        p.pose.position.y = newPath.poses.at(0).pose.position.y;
        p.pose.orientation.x = 0;
        p.pose.orientation.y = 0;
        p.pose.orientation.z = 0;
        p.pose.orientation.w = 1;

        finalPath.poses.push_back(p);

        while(end < newPath.poses.size())
        {
            //std::cout<< "End Value : "<< end <<std::endl;
            //std::cout<< "Start Value : "<< start <<std::endl;
            double distanceBetweenPoints = sqrt((newPath.poses.at(start).pose.position.y - newPath.poses.at(end).pose.position.y)*(newPath.poses.at(start).pose.position.y - newPath.poses.at(end).pose.position.y) +
                    (newPath.poses.at(start).pose.position.x - newPath.poses.at(end).pose.position.x)*(newPath.poses.at(start).pose.position.x - newPath.poses.at(end).pose.position.x));
            //std::cout<< "Distance : "<< distanceBetweenPoints <<std::endl;

            //If the distance between those two points are less than 0.1 meters
            if(distanceBetweenPoints < 0.1)
            {
                end++;
            }
            else
            {
                //std::cout<< "The point we add : "<< "( " << newPath.poses.at(end).pose.position.x<< " , "<<newPath.poses.at(end).pose.position.y <<" )"<<std::endl;
                //std::cout<< "End Value : "<< end <<std::endl;
                //std::cout<< "Path size is : "<< newPath.poses.size() << std::endl;
                p.pose.position.x = newPath.poses.at(end).pose.position.x;
                p.pose.position.y = newPath.poses.at(end).pose.position.y;
                p.pose.orientation.x = 0;
                p.pose.orientation.y = 0;
                p.pose.orientation.z = 0;
                p.pose.orientation.w = 1;

                finalPath.poses.push_back(p);

                start = end;
                end = start + 1;
            }
        }
        // TODO - There is some risk that the destination will be removed. If this happens, how to solve this
        // 1. Remove the last point and add the destination point; <-- I personally prefer this one
        // 2. Just add the destination Point.
        // And here we could use the "newPath last point" as the goal value
        if(finalPath.poses.back().pose.position.x != newPath.poses.back().pose.position.x &&
               finalPath.poses.back().pose.position.y != newPath.poses.back().pose.position.y )
        {
            finalPath.poses.pop_back(); // Delete the last point
            finalPath.poses.push_back(newPath.poses.back()); // Add the last point of the newPath
        }


        std::cout<< "FinalPath size is : "<< finalPath.poses.size() << std::endl;
    //    for(int i = 0; i < finalPath.poses.size(); i++)
    //    {
    //        std::cout<< "Final Path No." << i <<" point is X: "<< finalPath.poses.at(i).pose.position.x << " Y :" << finalPath.poses.at(i).pose.position.y << std::endl;
    //    }


        //This is for the final Path
        finalPath.header.stamp = ros::Time::now();
        finalPath.header.frame_id = "/map";
        return finalPath;
            */

        newPath.header.stamp = ros::Time::now();
        newPath.header.frame_id = "/map";
        return newPath;

    }
};





nav_msgs::Path servicePath(geometry_msgs::Pose &msg)
{
    //The Pose msg is in meters, so multiply this by 100
    int goalCol = msg.position.y / cell_size;
    int goalRow = msg.position.x / cell_size;

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
    p.pose.orientation.y = 0.0;
    p.pose.orientation.z = 1.0;

    try
    {
        tf_listener->waitForTransform(TargetFrameName, CurrentFrame, ros::Time(0), ros::Duration(1.0) );
        tf_listener->transformPose(TargetFrameName,p,startPose);
    }
    catch(tf::TransformException &ex)
    {
      ROS_ERROR("%s",ex.what());
      nav_msgs::Path nu;
      return nu;
      ros::Duration(1.0).sleep();
    }
    int startRow = startPose.pose.position.y / cell_size;
    int startCol = startPose.pose.position.x / cell_size;

    Node start(startRow,startCol,-1);
    Node goal(goalRow,goalCol,-1);

    PathFinder pf(start,goal);
    nav_msgs::Path originalPath = pf.getPath();
    nav_msgs::Path simplePath = pf.simpilifyPath(originalPath);
    return simplePath;
}



bool GetPath(path_planning::GetPath::Request  &req, path_planning::GetPath::Response &res)
{
    res.path = servicePath(req.goal);
    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Astar");
    ros::NodeHandle handle;

    // function for calculate the Current Position: baselink (0.0) to the map frame (TF)
    //ros::Subscriber path_sub_goal = handle.subscribe<geometry_msgs::Pose>("/goalPosition",1000,setGoalPosition);

    //Set map;
    map = new nav_msgs::OccupancyGrid();
    explored = new nav_msgs::OccupancyGrid();
    map->info.height = 0;
    map->info.width = 0;

    //ros::Publisher path_pub = handle.advertise<nav_msgs::Path>("/Astar/path",10);
    //ros::Publisher path_pub_simple = handle.advertise<nav_msgs::Path>("/Astar/Simplepath",10);
    //ros::Publisher explored_pub = handle.advertise<nav_msgs::OccupancyGrid>("/Astar/explored",1);


    //Wait 8 s for the map service.
    if(!ros::service::waitForService("/map_node/get_map", 8000))
    {
        ROS_ERROR("Map service unreachable.");
        return -1;
    }

    map_client = new ros::ServiceClient();
    *map_client = handle.serviceClient<map_tools::GetMap>("/map_node/get_map");
    updateMap();


    ros::ServiceServer path_srv = handle.advertiseService("/Astar/pathTest", GetPath);

    /*
    int StartRow = 23;//211;//138;//25;//92;//20;
    int StartCol = 205;//195;//220;//195;//200;//38;//193;//20;
    // row & col : start -> 211,195  || goal : 73,186

    int GoalRow = 70;//73;//211;//145;//216;//216;//216;//211;//90;//73;//25;//20;//40;
    int GoalCol = 205;//186;//195;//227;//220;//186;//104;//200;//36;//186;//200;//220;//20;
    
    Node start1(23, 205, -1);
    Node goal1(70, 205, -1);
    //Node goal1(50, 220, -1);
    Node start2(20, 20, -1);
    Node goal2(25, 25, -1);
    Node goal3(70, 50, -1);
    Node goal4(70, 110, -1);

    Node start(StartRow, StartCol, -1);
    Node goal(GoalRow,GoalCol, -1);
    
    start = start2;
    goal = goal3;
    


    PathFinder pf(start,goal);



    nav_msgs::Path path;
    nav_msgs::Path simpilifiedPath;

    */

    ros::Rate loop_rate(2);

    while (ros::ok())
    {
        updateMap();
        
        /*
        if(map->data.size() < 1000)
        {
            ROS_INFO("Map is not yet intialized");
        }
        else
        {
          ROS_INFO("Planning a path...");
          *explored = *map;
          path = pf.getPath();
          //std::cout<<"path size : "<<path.poses.size()<<std::endl;

          if(path.poses.size() > 0)
          {
             simpilifiedPath = pf.simpilifyPath(path);
          }
           path_pub_simple.publish(simpilifiedPath);
          path_pub.publish(path);
          explored_pub.publish(*explored);
        }
        */
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
