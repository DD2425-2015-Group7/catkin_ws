#include "map_tools/MapStorage.h"


MapStorage::MapStorage(int wc, int hc, double cellSize, int fullyOccupied, double inflationRadius)
{
    assert(cellSize>1e-10);
    assert(wc>0);
    assert(hc>0);
    this->cellSz = cellSize;
    this->fullyOccupied = fullyOccupied;
    this->inflationRadius = inflationRadius;
    map = new nav_msgs::OccupancyGrid();
    distMap = new nav_msgs::OccupancyGrid();
    inflMap = new nav_msgs::OccupancyGrid();
    map->info.map_load_time = ros::Time::now();
    map->info.resolution = cellSize;
    map->info.width = wc;
    map->info.height = hc;
    map->info.origin.position.x = 0.0;
    map->info.origin.position.y = 0.0;
    map->info.origin.position.z = 0.0;
    map->info.origin.orientation.x = 0.0;
    map->info.origin.orientation.y = 0.0;
    map->info.origin.orientation.z = 0.0;
    map->info.origin.orientation.w = 1.0;
    map->data.resize(wc*hc);
    *distMap = *map;
    *inflMap = *map;

    surface = cairo_image_surface_create (CAIRO_FORMAT_A8, wc, hc);
    cr = cairo_create (surface);
}

void MapStorage::addWall(double x0, double y0, double x1, double y1, double thickness)
{
    x0 = x0/cellSz;
    y0 = y0/cellSz;
    x1 = x1/cellSz;
    y1 = y1/cellSz;
    cairo_save (cr);
    cairo_set_line_width(cr, thickness/cellSz);
    cairo_set_source_rgba(cr, 0.0, 0.0, 0.0, (double)fullyOccupied/255.0);
    cairo_move_to(cr, x0, y0);
    cairo_move_to(cr, x0, y0);
    cairo_line_to(cr, x1, y1);
    cairo_stroke(cr);
    cairo_restore (cr);
}

void MapStorage::addEllipse(double x, double y, double a, double b, double th)
{
    x = x/cellSz;
    y = y/cellSz;
    a = a/cellSz;
    b = b/cellSz;
    cairo_save (cr);
    cairo_set_source_rgba(cr, 0.0, 0.0, 0.0, (double)fullyOccupied/255.0);
    cairo_translate (cr, x + a / 2., y + b / 2.);
    cairo_rotate(cr, th);
    cairo_scale (cr, a / 2., b / 2.);
    cairo_arc (cr, 0., 0., 1., 0., 2 * M_PI);
    cairo_fill (cr);
    cairo_restore (cr);
}

int MapStorage::max(int a, int b)
{
    if(a>b)
        return a;
    else
        return b;
}

int MapStorage::min(int a, int b)
{
    if(a<b)
        return a;
    else
        return b;
}

int MapStorage::boundedDist(int x, int y, int bound)
{
    int dist;
    dist = x*x + y*y;
    dist = (int)sqrt((double)dist);
    if(dist > bound)
        dist = bound;
    return dist;
}

int MapStorage::nnDist(int xi, int yi)
{
    int w = map->info.width;
    int h = map->info.height;
    if(map->data[yi*w + xi] >= fullyOccupied)
        return 0;
    const int maxLevel = 50;
    int level = 1, dist = maxLevel, x = xi, y = yi;
    while(level<maxLevel){
        y = max(yi - level, 0); 
        for(x = max(xi-level, 0); x < min(xi+level, w); x++){
            if(map->data[y*w + x] >= fullyOccupied)
                return boundedDist(x-xi, y-yi, maxLevel);
        }
        y = min(yi + level, h); 
        for(x = max(xi-level, 0); x < min(xi+level, w); x++){
            if(map->data[y*w + x] >= fullyOccupied)
                return boundedDist(x-xi, y-yi, maxLevel);
        }
        x = max(xi - level, 0); 
        for(y = max(yi-level, 0); y < min(yi+level, h); y++){
            if(map->data[y*w + x] >= fullyOccupied)
                return boundedDist(x-xi, y-yi, maxLevel);
        }
        x = min(xi + level, w); 
        for(y = max(yi-level, 0); y < min(yi+level, h); y++){
            if(map->data[y*w + x] >= fullyOccupied)
                return boundedDist(x-xi, y-yi, maxLevel);
        }
        level++;
    }
    return maxLevel;
}

void MapStorage::renderDistMap(void)
{
    int w = map->info.width;
    int h = map->info.height;
    for(int yi = 0; yi < h; yi++){
        for(int xi = 0; xi < w; xi++){
            distMap->data[yi*w + xi] = nnDist(xi, yi);
        }
    }
}

void MapStorage::renderInflMap(void)
{
    int w = map->info.width;
    int h = map->info.height;
    for(int yi = 0; yi < h; yi++){
        for(int xi = 0; xi < w; xi++){
            if(distMap->data[yi*w + xi]*cellSz > inflationRadius){
                inflMap->data[yi*w + xi] = 0;
            }else{
                inflMap->data[yi*w + xi] = fullyOccupied;
            }
        }
    }
}

void MapStorage::renderGrid(void)
{
    unsigned char *data = cairo_image_surface_get_data (surface);
    map->data.assign(data, data+(map->info.width*map->info.height));
    renderDistMap();
    renderInflMap();
}

void MapStorage::getMap(std::string type, nav_msgs::OccupancyGrid& _map)
{
    if(type.compare("default") == 0){
        _map.info = map->info;
        _map.data = map->data;
        return;
    }
    if(type.compare("inflated") == 0)
    {
        _map.info = inflMap->info;
        _map.data = inflMap->data;
        return;
    }
    if(type.compare("distance") == 0)
    {
        _map.info = distMap->info;
        _map.data = distMap->data;
        return;
    }
    
}

void MapStorage::loadWalls(std::string fn, double thickness)
{
    std::ifstream ifs(fn);
    assert(ifs.is_open());

    std::string line;
    int wall_id = 0;
    while (getline(ifs, line)){

        if (line[0] == '#') {
            // comment -> skip
            continue;
        }

        double max_num = std::numeric_limits<double>::max();
        double x1= max_num,
               x2= max_num,
               y1= max_num,
               y2= max_num;

        std::istringstream line_stream(line);

        line_stream >> x1 >> y1 >> x2 >> y2;

        if ((x1 == max_num) || ( x2 == max_num) || (y1 == max_num) || (y2 == max_num)){
            ROS_WARN("Segment error. Skipping line: %s",line.c_str());
            continue;
        }

        addWall(x1, y1, x2, y2, thickness);
        wall_id++;
    }
    ROS_INFO_STREAM("Read "<<wall_id<<" walls from map file.");
}

