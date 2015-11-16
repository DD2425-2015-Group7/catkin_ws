#include "localization/MonteCarlo.h"

MonteCarlo::MonteCarlo(OdometryModel *om, const int nParticles)
{
    this->om = om;
    this->nParticles = nParticles;
    this->first = true;
    srand (time(NULL));
}

void MonteCarlo::run(const struct PoseState odom)
{
    motionUpdate(odom);
    sensorUpdate();
    sample();
    avgAndStd();
}

void MonteCarlo::init(struct PoseState pose, double coneRadius, double yawVar)
{
    double r, th;
    double r1, r2;
    struct PoseState rs;
    rs.set(0.0);
    belief.clear();
    for(int i=0; i < nParticles; i++){
        th = (((double)rand()/(double)(RAND_MAX/2))-1.0)*M_PI;
        r1 = ((double)rand()/(double)(RAND_MAX/2))-1.0;
        r2 = ((double)rand()/(double)(RAND_MAX/2))-1.0;
        r = coneRadius * r1 * r2;
        rs.x = r*cos(th);
        rs.y = r*sin(th);
        r1 = ((double)rand()/(double)(RAND_MAX/2))-1.0;
        r2 = ((double)rand()/(double)(RAND_MAX/2))-1.0;
        rs.yaw = yawVar * r1 * r2;
        belief.push_back(rs);
    }
}

void MonteCarlo::motionUpdate(const struct PoseState odom)
{
    if(first){
        this->odom0 = odom;
        first = false;
        return;
    }
    om->setOdometry(this->odom0, odom);
    for(int i = 0; i < belief.size(); i++){
        belief[i].s = om->sample(belief[i].s);
        //TODO: repeat unless consistent with the inflated map.
    }
    this->odom0 = odom;
}

void MonteCarlo::sensorUpdate(void)
{
    //TODO: use the likelihood field
    //Receive sensor readings in the map frame. (x,y)
    //Precompute distances to the closest obstacle for each grid cell. (nearest neightbour)
    //Model unexplored free space as unknown -> sensor reading has const. prob. 1/zmax
    //Probablility calculation based on page 155. or extended symmetric algorithm? (page 157)
    //Summary 166
}

void MonteCarlo::sample(void)
{
    //TODO
    //Sample only when high weight variance.
    //Low-variance sampler. Page 98.
}

void MonteCarlo::avgAndStd(void)
{
    stateAvg.set(0.0);
    for(int i = 0; i < belief.size(); i++){
        stateAvg = stateAvg + belief[i].s;
    }
    stateAvg = stateAvg * (1.0/belief.size());

    stateStd.set(0.0);
    for(int i = 0; i < belief.size(); i++){
        stateStd = stateStd + (belief[i].s + (stateAvg*(-1))).pow2();
    }
    stateStd = stateStd * (1.0/(belief.size()-1));
    stateStd = stateStd.sqrt();
}

struct PoseState MonteCarlo::getState(void)
{
    return this->stateAvg;
}

struct PoseState MonteCarlo::getStd(void)
{
    return this->stateStd;
}

std::vector<MonteCarlo::StateW> MonteCarlo::getParticles(void)
{
    return this->belief;
}

bool MonteCarlo::test(void)
{
    /*
    struct PoseState z;
    z.set(0.0);
    struct PoseState s = PoseState(0.4, 0.6, -0.8, -1.0, 1.2, 1.4);
    struct PoseState ra = PoseState(0.533333, 0.966666, -0.966666, 0.266666, 4.266666, 1.5);
    struct PoseState rd = PoseState(0.208167, 0.1527525, 1.357694, 1.101514, 3.002221, 0.7);
    struct PoseState eth;
    eth.set(1e-4);
    initTest();
    motionUpdate(s, z);
    avgAndStd();

    s = getState();
    assert((s > (ra+(eth*(-1)))) && ((ra+eth) > s));
    s = getStd();
    assert((s.x > (rd.x-1e-4)) && (s.x < (rd.x+1e-4)));
    assert((s > (rd+(eth*(-1)))) && ((rd+eth) > s));
    return true;
    */
    return false;
}

void MonteCarlo::initTest(void)
{
    belief.clear();
    belief.push_back(MonteCarlo::StateW(PoseState(0.2, 0.5, 1.1, 2.0, 3.2, 0.4), 0.2));
    belief.push_back(MonteCarlo::StateW(PoseState(0.3, 0.4, -1.6, 1.8, 6.0, -0.7), 0.3));
    belief.push_back(MonteCarlo::StateW(PoseState(-0.1, 0.2, 0.0, 0.0, 0.0, 0.6), 0.5));
}
