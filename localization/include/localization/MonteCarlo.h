#include <iostream>
#include <fstream>
#include <vector>
#include <thread>
#include <assert.h>
#include <cmath>
#include <mutex>
#include <random>

#include "localization/PoseState.h"
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
        MonteCarlo(OdometryModel *om, bool (*isFree)(double, double), const int nParticles);
        //int addSensor(void);
        //void removeSensors(void);
        //bool removeSensor(int index);
        //void initRandom(void);
        void init(struct PoseState pose, double coneRadius, double yawVar);
        void run(const struct PoseState odom);
        struct PoseState getState(void);
        struct PoseState getStd(void);
        std::vector<struct StateW> getParticles(void);

        bool test(void);
    
    private:
        bool first;
        std::mutex mtx;
        
        struct PoseState stateAvg;
        struct PoseState stateStd;
        std::vector<struct StateW> belief;
        OdometryModel *om;
        struct PoseState odom0;
        int nParticles;
        bool (*isFree)(double, double);
        
        void motionUpdate(const struct PoseState odom);
        void sensorUpdate(void);
        void sample(void);
        void avgAndStd(void);
        
        void initTest(void);
    
};
