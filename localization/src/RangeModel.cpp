#include "localization/RangeModel.h"

RangeModel::RangeModel(double (*nearestObstacleDist)(double, double), double zHit, double zrm)
{
    this->getDist = nearestObstacleDist;
    this->zHit = zHit;
    this->zrm = zrm;
}

void RangeModel::updateMeasurements(std::vector<RangeModel::Reading> m)
{
    this->measured = m;
}

double RangeModel::prob(double dist2, double sigmaHit)
{
    double p;
    p = (1.0/(std::sqrt(2.0*M_PI)*sigmaHit));
    p = p * zHit * std::exp(-dist2/(2*sigmaHit*sigmaHit));
    p = p + zrm;
    return p;
}

double RangeModel::likelihood(struct PoseState state)
{
    //Model unexplored free space as unknown -> sensor reading has const. prob. 1/zmax
    //Probablility calculation based on page 155. or extended symmetric algorithm? (page 157)
    //Summary 166
    double q = 1;
    const double et = 0.001;
    double x, y, dist;
    
    for(int i = 0; i < measured.size(); i++){
        // Ignore max/invalid measurements.
        if(measured[i].x>-et && measured[i].x<et && measured[i].y>-et && measured[i].y<et)
            continue;
        x = state.x + measured[i].x*cos(state.yaw) - measured[i].y*sin(state.yaw);
        y = state.y + measured[i].y*cos(state.yaw) + measured[i].x*sin(state.yaw);
        dist = getDist(x, y);
        dist = dist*dist;
        q = q * prob(dist, measured[i].sigma);
    }
    
    return q;
}
