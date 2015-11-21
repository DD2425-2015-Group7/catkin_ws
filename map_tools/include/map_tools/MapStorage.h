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
        MapStorage(double cellSize, int fullyOccupied = 100, double inflationRadius = 0.01);
        void stackWall(double x0, double y0, double x1, double y1, double thickness);
        void stackEllipse(double x, double y, double a, double b, double th);
        

        void loadWalls(std::string fn, double thickness);
        void renderGrid(void);
        void getMap(std::string type, nav_msgs::OccupancyGrid& _map);
    private:
        struct Ellipse{
            double x;
            double y;
            double a;
            double b;
            double th;
            Ellipse(double x=0, double y=0, double a=0, double b=0, double th=0)
                : x(x), y(y), a(b), b(b), th(th)
            {
            }
        };
        struct Wall{
            double x0;
            double y0;
            double x1;
            double y1;
            double thickness;
            Wall(double x0=0, double y0=0, double x1=0, double y1=0, double thickness=0)
                : x0(x0), y0(y0), x1(x1), y1(y1), thickness(thickness)
            {
            }
        };
        double cellSz, inflationRadius, xMax, yMax;
        std::vector<Wall> wallSt;
        std::vector<Ellipse> ellipseSt;
        int fullyOccupied;
        cairo_surface_t *surface;
        cairo_t *cr;
        nav_msgs::OccupancyGrid *map, *distMap, *inflMap;
        
        void drawWall(double x0, double y0, double x1, double y1, double thickness);
        void drawEllipse(double x, double y, double a, double b, double th);
        
        int min(int a, int b);
        int max(int a, int b);
        int boundedDist(int x, int y, int bound);
        int nnDist(int xi, int yi);
        void renderDistMap(void);
        void renderInflMap(void);

};

#endif
