#include <iostream>
#include <vector>
#include <thread>
#include <assert.h>

#define cimg_display 0
#include "CImg.h"
using namespace cimg_library;

enum ReqOutput {IMAGE_PATCHES, DATA_PATCHES, BOTH};
typedef double my_float;

class MovingWindow{

public:
    MovingWindow(int windowSizeX, int windowSizeY, int stepX, int stepY, std::vector<int> scales);
    void runWindow(CImg<my_float> img, ReqOutput po); 
    std::vector< CImg<my_float> > getImagePatches(void);
    std::vector< std::vector<my_float> > getDataPatches(void);
    std::vector<int> getBoundingBox(int index);
    void freeMemory(void);
    
private:
    std::vector< CImg<my_float> > imagePatches;
    std::vector< std::vector<my_float> > dataPatches;
    std::vector<std::vector<int>> boundingBoxes;
    
    ReqOutput currentOutput;
    int windowSizeX, windowSizeY, stepX, stepY;
    std::vector<int> scales;
};
