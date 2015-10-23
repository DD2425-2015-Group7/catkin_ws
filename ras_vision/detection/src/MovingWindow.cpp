#include "detection/MovingWindow.h"

MovingWindow::MovingWindow(int windowSizeX, int windowSizeY, int stepX, int stepY, std::vector<int> scales)
{
    assert(windowSizeX > 0);
    assert(windowSizeY > 0);
    assert(stepX > 0);
    assert(stepY > 0);
    assert(scales.size()%2 == 0);
    this->windowSizeX = windowSizeX;
    this->windowSizeY = windowSizeY;
    this->stepX = stepX;
    this->stepY = stepY;
    this->scales = scales;
}

void MovingWindow::runWindow(CImg<my_float> img, ReqOutput ro)
{
    currentOutput = ro;
    freeMemory();
    int width = img.width();
    int height = img.height();
    
    CImg<double> imgCrop = CImg<double>();
    
    for(int s = 0; s < scales.size(); s+=2){
        img.resize(scales[s], scales[s+1], img.depth(), img.spectrum());
        
        for (int y = windowSizeY; y<scales[s+1]; y+=stepY){
        for (int x = windowSizeX; x<scales[s]; x+=stepX){
            imgCrop = img.get_crop(x-windowSizeX+1,y-windowSizeY+1,0,0,x,y,0,2);
            if(ro == BOTH || ro == IMAGE_PATCHES){
                imagePatches.push_back(imgCrop);
            }
            if(ro == BOTH || ro == DATA_PATCHES){
                std::vector<my_float> vcrop;
                vcrop.assign(imgCrop.data(), imgCrop.data() + imgCrop.size());
                dataPatches.push_back(vcrop);
            }
            int ksx = width/scales[s];
            int ksy = height/scales[s+1];
            std::vector<int> bbt;
            bbt.push_back(ksx*(x-windowSizeX) + 1);
            bbt.push_back(ksy*(y-windowSizeY) + 1);
            bbt.push_back(ksx*x);
            bbt.push_back(ksy*y);
            boundingBoxes.push_back(bbt);
        }
        }
    }
}
 
std::vector< CImg<my_float> > MovingWindow::getImagePatches(void)
{
    assert(currentOutput == BOTH || currentOutput == IMAGE_PATCHES);
    return imagePatches;
}

std::vector< std::vector<my_float> > MovingWindow::getDataPatches(void)
{
    assert(currentOutput == BOTH || currentOutput == DATA_PATCHES);
    return dataPatches;
}

std::vector<int> MovingWindow::getBoundingBox(int index)
{
    assert(index >= 0);
    assert(index < boundingBoxes.size());
    return boundingBoxes[index];
}

void MovingWindow::freeMemory(void)
{
    imagePatches.clear();
    dataPatches.clear();
    boundingBoxes.clear();
}
