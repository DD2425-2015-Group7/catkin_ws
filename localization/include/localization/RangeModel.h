#ifndef _RANGE_MODEL_H
#define _RANGE_MODEL_H

#include "localization/SensorInterface.h"
#include <vector>
#include <cmath>

class RangeModel : public SensorInterface
{
    public:
        virtual double likelihood(struct PoseState s);
        RangeModel(double (*nearestObstacleDist)(double, double), double sigmaHit, double zHit, double zrm);
        void updateMeasurements(std::vector<struct PoseState> m);
        
    private:
        std::vector<struct PoseState> measured;
        double (*getDist)(double, double);
        double sigmaHit, zHit, zrm;
        
        double prob(double dist2);
};

#endif
