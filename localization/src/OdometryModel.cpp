#include "localization/OdometryModel.h"

OdometryModel::OdometryModel(const double a1, const double a2, const double a3, const double a4)
{
    this->a1 = a1;
    this->a2 = a2;
    this->a3 = a3;
    this->a4 = a4;
    srand (time(NULL));
}

void OdometryModel::setOdometry(const struct PoseState odom0, const struct PoseState odom1)
{
    double dx = odom1.x-odom0.x;
    double dy = odom1.y-odom0.y;
    orot1 = atan2(dy, dx) - odom0.yaw;
    otrans = std::sqrt(dx*dx + dy*dy);
    orot2 = odom1.yaw - odom0.yaw - orot1;
}
struct PoseState OdometryModel::sample(struct PoseState prev)
{
    double rot1, trans, rot2;
    rot1 = orot1 - sampleTriangular(a1*orot1 + a2*otrans);
    trans = otrans - sampleTriangular(a3*otrans + a4*(orot1 + orot2));
    rot2 = orot2 - sampleTriangular(a1*orot2 + a2*otrans);
    
    prev.x += trans * cos(rot1 + prev.yaw); 
    prev.y += trans * sin(rot1 + prev.yaw); 
    prev.yaw += rot1 + rot2;
    
    return prev;
}

double OdometryModel::sampleTriangular(const double variance)
{
    double r1 = (double)rand()/(double)(RAND_MAX/2);
    r1 = r1 - 1.0;
    double r2 = (double)rand()/(double)(RAND_MAX/2);
    r2 = r2 - 1.0;
    return variance * r1 * r2;
}
