#include "localization/MonteCarlo.h"

MonteCarlo::MonteCarlo(OdometryModel *om, bool (*isFree)(double, double),
    const int nParticles, double minDelta, double aslow, double afast,
    double crashRadius, double crashYaw, struct PoseState goodStd)
{
    this->om = om;
    this->nParticles = nParticles;
    assert(nParticles > 0);
    this->minDelta = minDelta;
    this->first = true;
    this->isFree = isFree;
    srand (time(NULL));
    wavg = wslow = wfast = 0.0;
    this->aslow = aslow;
    this->afast = afast;
    assert(aslow >= 0.0);
    assert(aslow < afast);
    stateAvg.set(0.0);
    this->crashRadius = crashRadius;
    this->crashYaw = crashYaw;
    this->goodStd = goodStd;
}

bool MonteCarlo::run(struct PoseState odom, double mapXsz, double mapYsz)
{
    // Do not do anything when we are not moving.
    avgAndStd();
    if(std::sqrt(odom.x * odom.x + odom.y * odom.y) < minDelta)
        return false;
    this->mapXsz = mapXsz;
    this->mapYsz = mapYsz;
    motionUpdate(odom); 
    wavg = sensorUpdate(belief);
    sample();
    return true;
}

bool MonteCarlo::randNear(struct PoseState centre, struct PoseState &rs, double coneRadius, double yawVar)
{
    double r, th;
    double r1, r2;
    int count = 0;
    do{
        rs.set(0.0);
        count++;
        if(count%10 == 0)
            coneRadius = coneRadius * 1.3;
        th = (((double)rand()/(double)(RAND_MAX/2))-1.0)*M_PI;
        r1 = ((double)rand()/(double)(RAND_MAX/2))-1.0;
        r2 = ((double)rand()/(double)(RAND_MAX/2))-1.0;
        r = coneRadius * r1 * r2;
        rs.x = r*cos(th);
        rs.y = r*sin(th);
        rs = rs + centre;
        
        if(count>70){ // WARNING: this fails (rarely). TODO: a better solution.
            return false;
        }
    }while(!isFree(rs.x, rs.y));
    r1 = ((double)rand()/(double)(RAND_MAX/2))-1.0;
    r2 = ((double)rand()/(double)(RAND_MAX/2))-1.0;
    rs.yaw += yawVar * r1 * r2;
    return true;
}

struct PoseState MonteCarlo::randUniform(void)
{
    struct PoseState rs;
    assert(mapXsz > 0);
    assert(mapYsz > 0);
    rs.yaw = (((double)rand()/(double)(RAND_MAX/2))-1.0)*M_PI;
    do{
        rs.x = mapXsz * ((double)rand()/(double)(RAND_MAX));
        rs.y = mapYsz * ((double)rand()/(double)(RAND_MAX));
    }while(!isFree(rs.x, rs.y));
    return rs;
}

void MonteCarlo::init(struct PoseState pose, double coneRadius, double yawVar, double mapXsz, double mapYsz)
{
    this->mapXsz = mapXsz;
    this->mapYsz = mapYsz;
    struct PoseState rs;
    stateAvg = pose;
    belief.clear();
    for(int i=0; i < nParticles; i++){
        if(randNear(pose, rs, coneRadius, yawVar)){
            belief.push_back(rs);   //TODO How come this works ?????
        }else{
            belief.push_back(randUniform());
        }
    }
}

void MonteCarlo::init(double mapXsz, double mapYsz)
{
    this->mapXsz = mapXsz;
    this->mapYsz = mapYsz;
    stateAvg.set(0.0);
    initRandom(belief);
}

void MonteCarlo::initRandom(std::vector<MonteCarlo::StateW>& particles)
{
    struct StateW rsw;
    rsw.s.set(0.0);
    particles.clear();
    assert(mapXsz > 0);
    assert(mapYsz > 0);
    
    for(int i=0; i < nParticles; i++){
        /*
        rsw.s.yaw = (((double)rand()/(double)(RAND_MAX/2))-1.0)*M_PI;
        do{
            rsw.s.x = mapXsz * ((double)rand()/(double)(RAND_MAX));
            rsw.s.y = mapYsz * ((double)rand()/(double)(RAND_MAX));
        }while(!isFree(rsw.s.x, rsw.s.y));
        */
        rsw.s = randUniform();
        particles.push_back(rsw);
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
    struct PoseState st, sb;
    for(int i = 0; i < belief.size(); i++){
        int c1 = 0, c2 = 0;
        sb = belief[i].s;
        do{
            st = om->sample(sb);
            c1++;
            //If still having problems.
            if(c2 > 2){
                st = belief[i].s;
                break;
            }
            //If hitting a wall.
            if(c1 > 4){
                // TODO: What if stateStd is low but our particle is an outlier?
                if(stateStd > goodStd){
                    if(!randNear(belief[i].s, sb, crashRadius, crashYaw)){
                        st = belief[i].s;
                        break;
                    }
                }else{
                    if(!randNear(stateAvg, sb, crashRadius, crashYaw)){
                        st = belief[i].s;
                        break;
                    }
                }
                c2++;
                c1 = 0;
                continue;
            }
        }while(!isFree(st.x, st.y));
        belief[i].s = st;
    }
    this->odom0 = odom;
}

double MonteCarlo::sensorUpdate(std::vector<MonteCarlo::StateW>& particles)
{
    double wsum = 0.0;
    for(int j = 0; j < particles.size(); j++){
        particles[j].weight = 1;
        for(int i = 0; i < sensors.size(); i++){
            particles[j].weight *= sensors[i]->likelihood(particles[j].s);
        }
        wsum += particles[j].weight;
    }
    for(int j = 0; j < particles.size(); j++)
        particles[j].weight = particles[j].weight / wsum;
    
    return wsum / particles.size();
}

double MonteCarlo::max(double a, double b)
{
    if(a>b)
        return a;
    else
        return b;
}

void MonteCarlo::lowVarSampleOne(int &i, double &c, double r, int m, std::vector<MonteCarlo::StateW>& particles)
{
    int sz = particles.size();
    double u = r + ((double)m)/((double)sz);
    while(u > c){
        i = i + 1;
        assert(i<sz);
        c = c + particles[i].weight;
    }
}

void MonteCarlo::sample(void)
{
    //Sample only when high weight variance.
    //Low-variance sampler. Page 98.
    //Further on sampling hacks, random particle MCL: page 217.
    //The particle filter: 90.
    std::vector<struct StateW> nb, randPool;
    double r1, r2, c1, c2;
    int sz, i1, i2, m2;
    
    initRandom(randPool);
    sensorUpdate(randPool);
    
    if(belief.size()<1 || randPool.size()<1)
        return;
    
    wslow += aslow * (wavg - wslow);
    wfast += afast * (wavg - wfast);
    r1 = ((double)rand()/(double)(RAND_MAX))/((double)belief.size());
    r2 = ((double)rand()/(double)(RAND_MAX))/((double)randPool.size());
    i1 = i2 = 0;
    c1 = belief[0].weight;
    c2 = randPool[0].weight;
    sz = belief.size();
    
    m2 = 0;
    for(int m1 = 0; m1 < sz; m1++){
        //with probability max(0, 1 - wfast/wslow)
        if(((double)rand()/(double)(RAND_MAX)) < max(0.0, 1.0 - wfast/wslow)){ 
            lowVarSampleOne(i2, c2, r2, m2, randPool);
            nb.push_back(randPool[i2]);
            m2++;
            continue;
        }
        lowVarSampleOne(i1, c1, r1, m1, belief);
        nb.push_back(belief[i1]);
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
