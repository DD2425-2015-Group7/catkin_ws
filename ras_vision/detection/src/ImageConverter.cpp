#include "detection/ImageConverter.h"

void rosImg2CImg(const sensor_msgs::Image::ConstPtr& rim, CImg<my_float> &_img)
{
    int _width = rim->width;
    int _height = rim->height;
    int _channels = rim->step/rim->width;
    assert(_channels==3);

    int size = _width*_height*_channels;
    int s1 = _width*_height;
    char * dataF2 = (char*) malloc(size);
    int k = 0;

    for(int i = 0; i<size; i+=_channels){
        dataF2[k+0*s1] = rim->data[i+0];
        dataF2[k+1*s1] = rim->data[i+1];
        dataF2[k+2*s1] = rim->data[i+2];
        k++;
    }

    _img = CImg<char> (dataF2, _width, _height, 1, _channels, false); //false for is_shared
    free(dataF2);
}

void CImg2rosImg(const CImg<my_float> &_img, sensor_msgs::Image& rim)
{
    int _width = _img.width();
    int _height = _img.height();
    int _channels = _img.spectrum();
    rim.width = _width;
    rim.height = _height;
    rim.step = _channels * _width;
    assert(_channels==3);

    int size = _width*_height*_channels;
    int s1 = _width*_height;
    rim.data.resize(size);
    int k = 0;

    for(int i = 0; i<size; i+=_channels){
        rim.data[i+0] = _img.data()[k+0*s1];
        rim.data[i+1] = _img.data()[k+1*s1];
        rim.data[i+2] = _img.data()[k+2*s1];
        k++;
    }
}
