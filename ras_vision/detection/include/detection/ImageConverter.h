#include <iostream>
#include <vector>
#include <thread>
#include <assert.h>

#define cimg_display 0
#include "CImg.h"
using namespace cimg_library;

#include "sensor_msgs/Image.h"


typedef double my_float;

void rosImg2CImg(const sensor_msgs::Image::ConstPtr& rim, CImg<my_float> &_img);
void CImg2rosImg(const CImg<my_float> &_img, sensor_msgs::Image& rim);
