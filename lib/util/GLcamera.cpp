#ifdef __APPLE__
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#endif
#include <hrpUtil/Eigen3d.h>
#include "GLutil.h"
#include "GLlink.h"
#include "GLcamera.h"

using namespace OpenHRP;
using namespace hrp;

GLcamera::GLcamera(const SensorInfo &i_si, OpenHRP::BodyInfo_var i_binfo,
                   GLlink *i_link) : m_name(i_si.name), m_link(i_link) {
    
    Matrix33 R;
    Vector3 axis;
    axis[0] = i_si.rotation[0];
    axis[1] = i_si.rotation[1];
    axis[2] = i_si.rotation[2];
    
    hrp::calcRodrigues(R, axis, i_si.rotation[3]);
    
    m_trans[ 0]=R(0,0);m_trans[ 1]=R(1,0);m_trans[ 2]=R(2,0);m_trans[3]=0; 
    m_trans[ 4]=R(0,1);m_trans[ 5]=R(1,1);m_trans[ 6]=R(2,1);m_trans[7]=0; 
    m_trans[ 8]=R(0,2);m_trans[ 9]=R(1,2);m_trans[10]=R(2,2);m_trans[11]=0; 
    m_trans[12]=i_si.translation[0];m_trans[13]=i_si.translation[1];
    m_trans[14]=i_si.translation[2];m_trans[15]=1; 

    // display list
    glPushMatrix();
    glMultMatrixd(m_trans);
    compileShape(i_binfo, i_si.shapeIndices);
    glPopMatrix();
    

    m_near = i_si.specValues[0];
    m_far  = i_si.specValues[1];
    m_fovy = i_si.specValues[2];
    m_width  = i_si.specValues[4];
    m_height = i_si.specValues[5];
}

GLcamera::GLcamera(int i_width, int i_height, double i_near, double i_far, double i_fovy) : m_near(i_near), m_far(i_far), m_fovy(i_fovy), m_width(i_width), m_height(i_height)
{
}

const std::string& GLcamera::name() const {
    return m_name;
}

void GLcamera::computeAbsTransform(double o_trans[16]){
    if (m_link){
        double trans[16];
        m_link->computeAbsTransform(trans);
        mulTrans(m_trans, trans, o_trans);
    }else{
        memcpy(o_trans, m_trans, sizeof(double)*16);
    }
}

void GLcamera::setView()
{
    computeAbsTransform(m_absTrans);
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fovy()*180/M_PI, 
                   (double)width() / (double)height(), 
                   near(), far());
    gluLookAt(m_absTrans[12], m_absTrans[13], m_absTrans[14], 
              m_absTrans[12]-m_absTrans[8], 
              m_absTrans[13]-m_absTrans[9], 
              m_absTrans[14]-m_absTrans[10],
              m_absTrans[4], m_absTrans[5], m_absTrans[6]);
}

void GLcamera::setTransform(double i_trans[16]){
    memcpy(m_trans, i_trans, sizeof(double)*16);
}

void GLcamera::getAbsTransform(double o_trans[16]){
    memcpy(o_trans, m_absTrans, sizeof(double)*16);
}

void GLcamera::getDepthOfLine(int i_row, float *o_depth)
{
    glReadPixels(0, i_row, width(), 1, GL_DEPTH_COMPONENT, GL_FLOAT, o_depth);
}
