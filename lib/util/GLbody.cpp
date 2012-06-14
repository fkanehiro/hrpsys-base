#include <iostream>
#include "GLlink.h"
#include "GLbody.h"

using namespace hrp;

bool GLbody::m_useAbsTransformToDraw = false;

void GLbody::useAbsTransformToDraw()
{
    m_useAbsTransformToDraw = true;
}

GLbody::GLbody()
{
}

GLbody::~GLbody(){
}

void GLbody::setPosture(const double *i_angles){
    for (unsigned int i=0; i<numLinks(); i++){
        ((GLlink *)link(i))->setQ(i_angles[i]);
    }
}

void GLbody::setPosition(double x, double y, double z)
{
    ((GLlink *)rootLink())->setPosition(x,y,z);
}

void GLbody::setRotation(double r, double p, double y)
{
    ((GLlink *)rootLink())->setRotation(r,p,y);
}

void GLbody::setRotation(const double *R)
{
    ((GLlink *)rootLink())->setRotation(R);
}

void GLbody::setPosture(const double *i_angles, double *i_pos, double *i_rpy){
    setPosition(i_pos[0], i_pos[1], i_pos[2]); 
    setRotation(i_rpy[0], i_rpy[1], i_rpy[2]); 
    setPosture(i_angles);
}

void GLbody::setPosture(const dvector& i_q, const Vector3& i_p,
                        const Matrix33& i_R)
{
    double *tform = ((GLlink *)rootLink())->getTransform();
    tform[ 0]=i_R(0,0);tform[ 1]=i_R(1,0);tform[ 2]=i_R(2,0);tform[ 3]=0;
    tform[ 4]=i_R(0,1);tform[ 5]=i_R(1,1);tform[ 6]=i_R(2,1);tform[ 7]=0;
    tform[ 8]=i_R(0,2);tform[ 9]=i_R(1,2);tform[10]=i_R(2,2);tform[11]=0;
    tform[12]=i_p[0];tform[13]=i_p[1];tform[14]=i_p[2];tform[15]=1;
    setPosture(i_q.data());
}

void GLbody::draw(){
    if (m_useAbsTransformToDraw){
        for (unsigned int i=0; i<numLinks(); i++){
            ((GLlink *)link(i))->draw();
        }
    }else{
        ((GLlink *)rootLink())->draw(); // drawn recursively
    }
}

GLcamera *GLbody::findCamera(const char *i_name){
    for (unsigned int i=0; i<numLinks(); i++){
        GLcamera *camera = ((GLlink *)link(i))->findCamera(i_name);
        if (camera) return camera;
    }
    return NULL;
}

void GLbody::drawSensor(Sensor *sensor)
{
    if (m_sensorDrawCallback) m_sensorDrawCallback(this, sensor);
}

void GLbody::setSensorDrawCallback(boost::function2<void, hrp::Body *, hrp::Sensor *> f)
{
    m_sensorDrawCallback = f;
}

boost::function2<void, hrp::Body *, hrp::Sensor *> GLbody::getSensorDrawCallback()
{
    return m_sensorDrawCallback;
}
