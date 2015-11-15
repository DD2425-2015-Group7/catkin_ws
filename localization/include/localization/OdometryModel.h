#include <iostream>
#include <fstream>
#include <vector>
#include <thread>
#include <assert.h>
#include <cmath>
#include <mutex>
#include <random>
#include <time.h>

#include "localization/PoseState.h"

class OdometryModel{
    public:
        OdometryModel(const double a1, const double a2, const double a3, const double a4);
        void setOdometry(const struct PoseState odom0, const struct PoseState odom1);
        struct PoseState sample(const struct PoseState prev);
    private:
        double orot1, otrans, orot2;
        double a1, a2, a3, a4;
        double sampleTriangular(const double variance);
};
