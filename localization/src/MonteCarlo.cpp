#include "localization/MonteCarlo.h"

MonteCarlo::MonteCarlo(OdometryModel *om, bool (*isFree)(double, double), const int nParticles, double minDelta)
{
    this->om = om;
    this->nParticles = nParticles;
    this->minDelta = minDelta;
    this->first = true;
    this->isFree = isFree;
    srand (time(NULL));
    wavg = wslow = wfast = 0.0;
    aslow = 0.02;
    afast = 0.3;
}

bool MonteCarlo::run(struct PoseState odom)
{
    // Do not do anything when we are not moving.
    if(odom.magnitude() < minDelta)
        return false;
    motionUpdate(odom);
    sensorUpdate();
    sample();
    avgAndStd();
    return true;
}

void MonteCarlo::init(struct PoseState pose, double coneRadius, double yawVar)
{
    double r, th;
    double r1, r2;
    struct PoseState rs;
    rs.set(0.0);
    belief.clear();
    for(int i=0; i < nParticles; i++){
        do{
            th = (((double)rand()/(double)(RAND_MAX/2))-1.0)*M_PI;
            r1 = ((double)rand()/(double)(RAND_MAX/2))-1.0;
            r2 = ((double)rand()/(double)(RAND_MAX/2))-1.0;
            r = coneRadius * r1 * r2;
            rs.x = r*cos(th);
            rs.y = r*sin(th);
            rs = rs + pose;
        }while(!isFree(rs.x, rs.y));
        r1 = ((double)rand()/(double)(RAND_MAX/2))-1.0;
        r2 = ((double)rand()/(double)(RAND_MAX/2))-1.0;
        rs.yaw += yawVar * r1 * r2;
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
    struct PoseState st;
    for(int i = 0; i < belief.size(); i++){
        int count = 0;
        do{
            st = om->sample(belief[i].s);
            count++;
            if(count > 10){
                //If hitting a wall, keep the old particle. The move is not possible.
                st = belief[i].s;
                break;
            }
        }while(!isFree(st.x, st.y));
        belief[i].s = st;
    }
    this->odom0 = odom;
}

void MonteCarlo::sensorUpdate(void)
{
    double wsum = 0.0;
    for(int j = 0; j < belief.size(); j++){
        belief[j].weight = 1;
        for(int i = 0; i < sensors.size(); i++){
            belief[j].weight *= sensors[i]->likelihood(belief[j].s);
        }
        wsum += belief[j].weight;
    }
    wavg = wsum / belief.size();
    for(int j = 0; j < belief.size(); j++)
        belief[j].weight = belief[j].weight / wsum;
}

void MonteCarlo::sample(void)
{
    //Sample only when high weight variance.
    //Low-variance sampler. Page 98.
    //Further on sampling hacks, random particle MCL: page 217.
    //The particle filter: 90.
    if(belief.size()<1)
        return;
    wslow += aslow * (wavg - wslow);
    wfast += afast * (wavg - wfast);
    std::vector<struct StateW> nb;
    double r = ((double)rand()/(double)(RAND_MAX))/((double)belief.size());
    int sz, i = 0;
    double c = belief[0].weight;
    sz = belief.size();
    for(int m = 0; m < sz; m++){
        if(0){ //with probability max(0, 1 - wfast/wslow)
            //add random pose
        }//otherwise sample a pose normally
        
        double u = r + m/((double)sz);
        while(u > c){
            i = i + 1;
            assert(i<sz);
            c = c + belief[i].weight;
        }
        nb.push_back(belief[i]);
    }
    belief = nb;
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

void MonteCarlo::addSensor(SensorInterface* si)
{
    sensors.push_back(si);
}

void MonteCarlo::removeSensors(void)
{
    sensors.clear();
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
