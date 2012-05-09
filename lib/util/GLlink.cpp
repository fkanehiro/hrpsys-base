#include <iostream>
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#include "GLutil.h"
#include "GLcamera.h"
#include "GLlink.h"
#include "GLshape.h"

using namespace OpenHRP;
using namespace hrp;

bool GLlink::m_useAbsTransformToDraw = false;

void GLlink::useAbsTransformToDraw()
{
    m_useAbsTransformToDraw = true;
}

GLlink::GLlink()
{
}

void GLlink::setDrawInfo(const LinkInfo &i_li, ShapeSetInfo_ptr i_ssinfo){
    Vector3 axis;
    Matrix33 R;
    
    for (int i=0; i<3; i++){
        axis[i] = i_li.rotation[i];
    }
    setQ(0);
    
    hrp::calcRodrigues(R, axis, i_li.rotation[3]);
    
    m_trans[ 0]=R(0,0);m_trans[ 1]=R(1,0);m_trans[ 2]=R(2,0);m_trans[3]=0; 
    m_trans[ 4]=R(0,1);m_trans[ 5]=R(1,1);m_trans[ 6]=R(2,1);m_trans[7]=0; 
    m_trans[ 8]=R(0,2);m_trans[ 9]=R(1,2);m_trans[10]=R(2,2);m_trans[11]=0; 
    m_trans[12]=i_li.translation[0];m_trans[13]=i_li.translation[1];
    m_trans[14]=i_li.translation[2];m_trans[15]=1; 

    computeAbsTransform(m_absTrans);

    GLshape *shape = new GLshape();
    m_shapes.push_back(shape);
    shape->setDrawInfo(i_ssinfo, i_li.shapeIndices);

    const SensorInfoSequence& sensors = i_li.sensors;
    for (unsigned int i=0; i<sensors.length(); i++){
        const SensorInfo& si = sensors[i];
        std::string type(si.type);
        if (type == "Vision"){
            //std::cout << si.name << std::endl;
            m_cameras.push_back(new GLcamera(si,i_ssinfo, this));
        }
    }
}

GLlink::~GLlink()
{
    for (size_t i=0; i<m_shapes.size(); i++){
        delete m_shapes[i];
    }
    for (unsigned int i=0; i<m_cameras.size(); i++){
        delete m_cameras[i];
    }
}
        
void GLlink::draw(){
    glPushMatrix();
    if (m_useAbsTransformToDraw){
        glMultMatrixd(m_absTrans);
    }else{
        glMultMatrixd(m_trans);
        glMultMatrixd(m_T_j);
    }
    for (size_t i=0; i<m_shapes.size(); i++){
        m_shapes[i]->draw();
    }
    for (size_t i=0; i<m_cameras.size(); i++){
        m_cameras[i]->draw();
    }
    if (!m_useAbsTransformToDraw){
        hrp::Link *l = child;
        while (l){
            ((GLlink *)l)->draw();
            l = l->sibling;
        }
    }
    glPopMatrix();
}

void GLlink::setQ(double i_q){
    Matrix33 R;
    hrp::calcRodrigues(R, a, i_q);
    m_T_j[ 0]=R(0,0);m_T_j[ 1]=R(1,0);m_T_j[ 2]=R(2,0);m_T_j[3]=0; 
    m_T_j[ 4]=R(0,1);m_T_j[ 5]=R(1,1);m_T_j[ 6]=R(2,1);m_T_j[7]=0; 
    m_T_j[ 8]=R(0,2);m_T_j[ 9]=R(1,2);m_T_j[10]=R(2,2);m_T_j[11]=0;
    m_T_j[12]=0;     m_T_j[13]=0;     m_T_j[14]=0;     m_T_j[15]=1;    
    //printf("m_T_j:\n");
    //printMatrix(m_T_j);
}

void GLlink::setTransform(double i_trans[16]){
    memcpy(m_trans, i_trans, sizeof(double)*16);
}

void GLlink::setAbsTransform(double i_trans[16]){
    memcpy(m_absTrans, i_trans, sizeof(double)*16);
}

GLcamera *GLlink::findCamera(const char *i_name){
    std::string name(i_name);
    for (unsigned int i=0; i<m_cameras.size(); i++){
        if (m_cameras[i]->name() == name) return m_cameras[i];
    }
    return NULL;
}

void GLlink::computeAbsTransform(double o_trans[16]){
    if (parent){
        double trans1[16], trans2[16];
        mulTrans(m_T_j, m_trans, trans1);
        ((GLlink *)parent)->computeAbsTransform(trans2);
        mulTrans(trans1, trans2, o_trans);
    }else{
        memcpy(o_trans, m_trans, sizeof(double)*16);
    }
}

hrp::Link *GLlinkFactory()
{
    return new GLlink();
}
