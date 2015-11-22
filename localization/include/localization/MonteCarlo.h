#ifndef _MONTE_CARLO_H
#define _MONTE_CARLO_H

#include <iostream>
#include <fstream>
#include <vector>
#include <thread>
#include <assert.h>
#include <cmath>
#include <mutex>
#include <random>

#include "localization/PoseState.h"
#include "localization/SensorInterface.h"
#include "localization/OdometryModel.h"


class MonteCarlo{
    public:
        struct StateW{
            struct PoseState s;
            double weight;
            StateW(struct PoseState s = PoseState(), double weight = 0)
                    : s(s), weight(weight)
            {
            }
        };
        MonteCarlo(OdometryModel *om, bool (*isFree)(double, double),
            const int nParticles, double minDelta, double aslow, double afast,
            double crashRadius, double crashYaw, struct PoseState goodStd);
        void addSensor(SensorInterface* si);
        void removeSensors(void);
        void init(double mapXsz, double mapYsz);
        //void init(struct PoseState pose, double coneRadius, double yawVar);
        void init(struct PoseState pose, double coneRadius, double yawVar, double mapXsz, double mapYsz);
        bool run(struct PoseState odom, double mapXsz, double mapYsz);
        struct PoseState getState(void);
        struct PoseState getStd(void);
        std::vector<struct StateW> getParticles(void);

        bool test(void);
    
    private:
        bool first;
        std::mutex mtx;
        
        struct PoseState stateAvg;
        struct PoseState stateStd, goodStd;
        std::vector<struct StateW> belief;
        OdometryModel *om;
        struct PoseState odom0;
        int nParticles;
        double crashRadius, crashYaw;
        double minDelta, mapXsz, mapYsz;
        double wavg, wslow, wfast, aslow, afast;
        bool (*isFree)(double, double);
        std::vector<SensorInterface*> sensors;
        
        struct PoseState randUniform(void);
        bool randNear(struct PoseState centre, struct PoseState &rs, double coneRadius, double yawVar);
        //struct PoseState randNear(struct PoseState centre, double coneRadius, double yawVar);
        void initRandom(std::vector<struct StateW>& particles);
        void motionUpdate(const struct PoseState odom);
        double max(double a, double b);
        double sensorUpdate(std::vector<StateW>& particles);
        void lowVarSampleOne(int &i, double &c, double r, int m, std::vector<MonteCarlo::StateW>& particles);
        void sample(void);
        void avgAndStd(void);
        
        void initTest(void);
    
};

#endif
