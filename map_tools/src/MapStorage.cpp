#include "map_tools/MapStorage.h"


MapStorage::MapStorage(int wc, int hc, double cellSize, int fullyOccupied)
{
    assert(cellSize>1e-10);
    assert(wc>0);
    assert(hc>0);
    this->cellSz = cellSize;
    this->fullyOccupied = fullyOccupied;
    map = new nav_msgs::OccupancyGrid();
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

void MapStorage::renderGrid(void)
{
    unsigned char *data = cairo_image_surface_get_data (surface);
    map->data.assign(data, data+(map->info.width*map->info.height));
}

void MapStorage::getMap(nav_msgs::OccupancyGrid& _map)
{
    _map.info = map->info;
    _map.data = map->data;
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

