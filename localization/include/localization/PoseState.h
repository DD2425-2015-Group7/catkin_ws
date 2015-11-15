#ifndef _POSESTATE_H
#define _POSESTATE_H

#include <cmath>

struct PoseState{
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
    PoseState(double x=0, double y=0, double z=0, double roll=0, double pitch=0, double yaw=0)
            : x(x), y(y), z(z), roll(roll), pitch(pitch), yaw(yaw)
    {
    }
    PoseState operator +(const PoseState& a) {
        return PoseState(a.x+x, a.y+y, a.z+z, a.roll+roll, a.pitch+pitch, a.yaw+yaw);
    }
    PoseState operator *(const double& a) {
        return PoseState(a*x, a*y, a*z, a*roll, a*pitch, a*yaw);
    }
    PoseState pow2(void){
        return PoseState(x*x, y*y, z*z, roll*roll, pitch*pitch, yaw*yaw);
    }
    PoseState sqrt(void){
        return PoseState(std::sqrt(x), std::sqrt(y), std::sqrt(z), std::sqrt(roll), std::sqrt(pitch), std::sqrt(yaw));
    }
    bool operator >(const PoseState& a) {
        bool res;
        res = (x > a.x) && (y > a.y) && (z > a.z);
        res = res && (roll > a.roll) && (pitch > a.pitch) && (yaw > a.yaw);
        return res;
    }

    void set(double num){
        x = num;
        y = num;
        z = num;
        roll = num;
        pitch = num;
        yaw = num;
    }
};
#endif
