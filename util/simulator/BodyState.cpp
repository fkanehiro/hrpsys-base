#include <hrpModel/Link.h>
#include <hrpModel/Sensor.h>
#include "BodyState.h"

using namespace hrp;

void BodyState::set(BodyPtr i_body)
{
    Link *root = i_body->rootLink();
    p = root->p;
    R = root->attitude();
    q.resize(i_body->numLinks());
    for (unsigned int i=0; i<i_body->numLinks(); i++){
        q[i] = i_body->link(i)->q;
    }
    int n;
    n = i_body->numSensors(Sensor::FORCE);
    force.resize(n);
    for(int id = 0; id < n; ++id){
        ForceSensor* sensor = i_body->sensor<ForceSensor>(id);
        setVector3(sensor->f,   force[id], 0);
        setVector3(sensor->tau, force[id], 3);
    }

    n = i_body->numSensors(Sensor::RATE_GYRO);
    rate.resize(n);
    for(int id=0; id < n; ++id){
        RateGyroSensor* sensor = i_body->sensor<RateGyroSensor>(id);
        setVector3(sensor->w, rate[id]);
    }
    
    n = i_body->numSensors(Sensor::ACCELERATION);
    acc.resize(n);
    for(int id=0; id < n; ++id){
        AccelSensor* sensor = i_body->sensor<AccelSensor>(id);
        setVector3(sensor->dv, acc[id]);
    }

    n = i_body->numSensors(Sensor::RANGE);
    range.resize(n);
    for(int id=0; id < n; ++id){
        RangeSensor* sensor = i_body->sensor<RangeSensor>(id);
        range[id] = sensor->distances;
    }
}
