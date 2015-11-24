#include "map_tools/MapStorage.h"


MapStorage::MapStorage(double cellSize, int fullyOccupied, int minOccupied, double inflationRadius)
{
    assert(cellSize>1e-10);
    this->cellSz = cellSize;
    this->fullyOccupied = fullyOccupied;
    this->minOccupied = minOccupied;
    this->inflationRadius = inflationRadius;
    map = new nav_msgs::OccupancyGrid();
    distMap = new nav_msgs::OccupancyGrid();
    inflMap = new nav_msgs::OccupancyGrid();
    map->info.map_load_time = ros::Time::now();
    map->info.resolution = cellSize;
    map->info.origin.position.x = 0.0;
    map->info.origin.position.y = 0.0;
    map->info.origin.position.z = 0.0;
    map->info.origin.orientation.x = 0.0;
    map->info.origin.orientation.y = 0.0;
    map->info.origin.orientation.z = 0.0;
    map->info.origin.orientation.w = 1.0;
    this->xMax = 0.0;
    this->yMax = 0.0;
    
    surface = nullptr;
    cr = nullptr;
}

void MapStorage::drawWall(double x0, double y0, double x1, double y1, double thickness)
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

void MapStorage::drawEllipse(double x, double y, double a, double b, double th)
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
    if(map->data[yi*w + xi] >= minOccupied)
        return 0;
    const int maxLevel = 50;
    int level = 1, dist = maxLevel, x = xi, y = yi, d;
    while(level<maxLevel){
        y = max(yi - level, 0); 
        for(x = max(xi-level, 0); x < min(xi+level, w); x++){
            if(map->data[y*w + x] >= minOccupied){
                d = boundedDist(x-xi, y-yi, maxLevel);
                if(d<dist)
                    dist = d;
            }
        }
        y = min(yi + level, h); 
        for(x = max(xi-level, 0); x < min(xi+level, w); x++){
            if(map->data[y*w + x] >= minOccupied){
                d = boundedDist(x-xi, y-yi, maxLevel);
                if(d<dist)
                    dist = d;
            }
        }
        x = max(xi - level, 0); 
        for(y = max(yi-level, 0); y < min(yi+level, h); y++){
            if(map->data[y*w + x] >= minOccupied){
                d = boundedDist(x-xi, y-yi, maxLevel);
                if(d<dist)
                    dist = d;
            }
        }
        x = min(xi + level, w); 
        for(y = max(yi-level, 0); y < min(yi+level, h); y++){
            if(map->data[y*w + x] >= minOccupied){
                d = boundedDist(x-xi, y-yi, maxLevel);
                if(d<dist)
                    dist = d;
            }
        }
        if(dist <= level)
            return dist;
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

void MapStorage::stackWall(double x0, double y0, double x1, double y1, double thickness)
{
    if(x0 > xMax)
        xMax = x0;
    if(x1 > xMax)
        xMax = x1;
    if(y0 > yMax)
        yMax = y0;
    if(y1 > yMax)
        yMax = y1;
    wallSt.push_back(MapStorage::Wall(x0,y0,x1,y1,thickness));
}

void MapStorage::stackEllipse(double x, double y, double a, double b, double th)
{
    ellipseSt.push_back(MapStorage::Ellipse(x,y,a,b,th));
}

void MapStorage::clearEllipses(void)
{
    ellipseSt.clear();
}


void MapStorage::renderGrid(void)
{
    int wc, hc;
    wc = (int)(xMax/cellSz);
    hc = (int)(yMax/cellSz);
    assert(wc > 0);
    assert(hc > 0);
    map->info.width = wc;
    map->info.height = hc;
    map->data.resize(wc*hc);
    *distMap = *map;
    *inflMap = *map;

    if(cr!=nullptr)
        cairo_destroy(cr);
    if(surface!=nullptr)
        cairo_surface_destroy(surface);
    
    surface = cairo_image_surface_create (CAIRO_FORMAT_A8, wc, hc);
    cr = cairo_create (surface);
    
    for(int i = 0; i<wallSt.size(); i++)
        drawWall(wallSt[i].x0, wallSt[i].y0, wallSt[i].x1, wallSt[i].y1, wallSt[i].thickness);
    for(int i = 0; i<ellipseSt.size(); i++)
        drawEllipse(ellipseSt[i].x, ellipseSt[i].y, ellipseSt[i].a, ellipseSt[i].b, ellipseSt[i].th);
    
    unsigned char *data = cairo_image_surface_get_data (surface);
    map->data.assign(data, data+(wc*hc));
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

        stackWall(x1, y1, x2, y2, thickness);
        wall_id++;
    }
    ROS_INFO_STREAM("Read "<<wall_id<<" walls from map file.");
}

