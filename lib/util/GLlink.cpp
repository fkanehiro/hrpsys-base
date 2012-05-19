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

GLlink::GLlink() : m_showAxes(false)
{
    Rs = hrp::Matrix33::Identity();
    R  = hrp::Matrix33::Identity();
    setQ(0);
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
    if (m_showAxes){
        glDisable(GL_LIGHTING);
        glBegin(GL_LINES);
        glColor3f(1,0,0);
        glVertex3f(0,0,0); glVertex3f(0.5, 0, 0);
        glColor3f(0,1,0);
        glVertex3f(0,0,0); glVertex3f(0, 0.5, 0);
        glColor3f(0,0,1);
        glVertex3f(0,0,0); glVertex3f(0, 0, 0.5);
        glEnd();
        glEnable(GL_LIGHTING);
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

void GLlink::computeAbsTransform(){
    computeAbsTransform(m_absTrans);
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

void GLlink::addShape(GLshape *shape)
{
    m_shapes.push_back(shape);
}

void GLlink::addCamera(GLcamera *camera)
{
    m_cameras.push_back(camera);
}

void GLlink::showAxes(bool flag)
{
    m_showAxes = flag;
}

hrp::Link *GLlinkFactory()
{
    return new GLlink();
}

const std::vector<GLcamera *>& GLlink::cameras()
{
    return m_cameras;
}
