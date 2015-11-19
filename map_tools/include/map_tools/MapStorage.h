#ifndef _MAPSTORAGE_H
#define _MAPSTORAGE_H

#include "ros/ros.h"
#include <cairo.h>
#include <fstream>
#include <iostream>
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"


class MapStorage{
    public:
        MapStorage(int wc, int hc, double cellSize, int fullyOccupied = 100, double inflationRadius = 0.01);
        void addWall(double x0, double y0, double x1, double y1, double thickness);
        void addEllipse(double x, double y, double a, double b, double th);

        void loadWalls(std::string fn, double thickness);
        void renderGrid(void);
        void getMap(std::string type, nav_msgs::OccupancyGrid& _map);
    private:
        double cellSz, inflationRadius;
        int fullyOccupied;
        cairo_surface_t *surface;
        cairo_t *cr;
        nav_msgs::OccupancyGrid *map, *distMap, *inflMap;
        
        int min(int a, int b);
        int max(int a, int b);
        int boundedDist(int x, int y, int bound);
        int nnDist(int xi, int yi);
        void renderDistMap(void);
        void renderInflMap(void);

};

#endif
