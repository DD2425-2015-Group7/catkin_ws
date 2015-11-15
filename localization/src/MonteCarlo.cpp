#include "localization/MonteCarlo.h"

MonteCarlo::MonteCarlo(OdometryModel *om)
{
    this->om = om;
    this->first = true;
}

void MonteCarlo::run(const struct PoseState odom)
{
    motionUpdate(odom);
    sensorUpdate();
    sample();
    avgAndStd();
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
    }
    this->odom0 = odom;
}

void MonteCarlo::sensorUpdate(void)
{
    //TODO
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
