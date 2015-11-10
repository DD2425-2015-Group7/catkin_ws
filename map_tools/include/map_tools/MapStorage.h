#ifndef _MAPSTORAGE_H
#define _MAPSTORAGE_H

#include "ros/ros.h"
//#include "map_tools/Drawing2D.h"
#include <cairo.h>
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"


class MapStorage{
    public:
        MapStorage(int wc, int hc, double cellSize, int fullyOccupied = 100);
        void addWall(double x0, double y0, double x1, double y1, double thickness);
        void addEllipse(double x, double y, double a, double b, double th);

        void renderGrid(void);
        void getMap(nav_msgs::OccupancyGrid& _map);
    private:
        double cellSz;
        int fullyOccupied;
        //Drawing2D<int8_t> mapStack;
        cairo_surface_t *surface;
        cairo_t *cr;
        nav_msgs::OccupancyGrid *map;

        // TODO: Drawing2D<int8_t> mapInfStack;

};

#endif
