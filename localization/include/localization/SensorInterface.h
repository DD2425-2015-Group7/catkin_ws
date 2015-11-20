#ifndef _SENSOR_INTERFACE_H
#define _SENSOR_INTERFACE_H

#include "localization/PoseState.h"

class SensorInterface
{
    public:
        virtual ~SensorInterface() {}
        virtual double likelihood(const struct PoseState s) = 0;
};

#endif
