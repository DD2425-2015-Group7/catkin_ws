#ifndef _RANGE_MODEL_H
#define _RANGE_MODEL_H

#include "localization/SensorInterface.h"
#include <vector>
#include <cmath>

class RangeModel : public SensorInterface
{
    public:
        struct Reading{
            double x;
            double y;
            double sigma;
            Reading(double x=0, double y=0, double sigma=0)
                    : x(x), y(y), sigma(sigma)
            {
            }
        };
        virtual double likelihood(struct PoseState s);
        RangeModel(double (*nearestObstacleDist)(double, double), double zHit, double zrm);
        void updateMeasurements(std::vector<struct Reading> m);
        
    private:
        std::vector<struct Reading> measured;
        double (*getDist)(double, double);
        double zHit, zrm;
        
        double prob(double dist2, double sigmaHit);
};

#endif
