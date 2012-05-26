#ifdef __APPLE__
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#endif
#include <hrpUtil/Eigen3d.h>
#include "GLutil.h"
#include "GLlink.h"
#include "GLcamera.h"
#include "GLshape.h"

using namespace OpenHRP;
using namespace hrp;

GLcamera::GLcamera(const SensorInfo &i_si, OpenHRP::ShapeSetInfo_ptr i_ssinfo,
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

    for (size_t i=0; i<i_si.shapeIndices.length(); i++){
        GLshape *shape = new GLshape();
        loadShape(shape, i_ssinfo, i_si.shapeIndices[i]);
        m_shapes.push_back(shape);
    }

    m_near = i_si.specValues[0];
    m_far  = i_si.specValues[1];
    m_fovy = i_si.specValues[2];
    m_width  = i_si.specValues[4];
    m_height = i_si.specValues[5];
}

GLcamera::GLcamera(int i_width, int i_height, double i_near, double i_far, double i_fovy) : m_near(i_near), m_far(i_far), m_fovy(i_fovy), m_width(i_width), m_height(i_height), m_link(NULL)
{
}

GLcamera::~GLcamera()
{
    for (size_t i=0; i<m_shapes.size(); i++){
        delete m_shapes[i];
    }
}

void GLcamera::draw()
{
    // display list
    glPushMatrix();
    glMultMatrixd(m_trans);
    for (size_t i=0; i<m_shapes.size(); i++){
        m_shapes[i]->draw();
    }
#if 0
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    double t = tan(m_fovy/2);
    double xf = t*m_far*m_width/m_height, yf = t*m_far;
    glVertex3f( xf,  yf, -m_far); glVertex3f(-xf,  yf, -m_far);
    glVertex3f(-xf,  yf, -m_far); glVertex3f(-xf, -yf, -m_far);
    glVertex3f(-xf, -yf, -m_far); glVertex3f( xf, -yf, -m_far);
    glVertex3f( xf, -yf, -m_far); glVertex3f( xf,  yf, -m_far); 
    double xn = t*m_near*m_width/m_height, yn = t*m_near;
    glVertex3f( xn,  yn, -m_near); glVertex3f(-xn,  yn, -m_near);
    glVertex3f(-xn,  yn, -m_near); glVertex3f(-xn, -yn, -m_near);
    glVertex3f(-xn, -yn, -m_near); glVertex3f( xn, -yn, -m_near);
    glVertex3f( xn, -yn, -m_near); glVertex3f( xn,  yn, -m_near);
    glVertex3f( xn,  yn, -m_near); glVertex3f( xf,  yf, -m_far);
    glVertex3f(-xn,  yn, -m_near); glVertex3f(-xf,  yf, -m_far);
    glVertex3f(-xn, -yn, -m_near); glVertex3f(-xf, -yf, -m_far);
    glVertex3f( xn, -yn, -m_near); glVertex3f( xf, -yf, -m_far);
    glEnd();
    glEnable(GL_LIGHTING);
#endif
    glPopMatrix();
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
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fovy()*180/M_PI, 
                   (double)width() / (double)height(), 
                   near(), far());
    if (m_link){
        computeAbsTransform(m_absTrans);
        gluLookAt(m_absTrans[12], m_absTrans[13], m_absTrans[14], 
                  m_absTrans[12]-m_absTrans[8], 
                  m_absTrans[13]-m_absTrans[9], 
                  m_absTrans[14]-m_absTrans[10],
                  m_absTrans[4], m_absTrans[5], m_absTrans[6]);
    }else{
        gluLookAt(m_viewPoint[0], m_viewPoint[1], m_viewPoint[2], 
                  m_viewTarget[0], m_viewTarget[1], m_viewTarget[2], 
                  0,0,1);
    }
}

void GLcamera::setViewPoint(double x, double y, double z)
{
    m_viewPoint[0] = x; m_viewPoint[1] = y; m_viewPoint[2] = z;
}

void GLcamera::setViewTarget(double x, double y, double z)
{
    m_viewTarget[0] = x; m_viewTarget[1] = y; m_viewTarget[2] = z;
}

double *GLcamera::getAbsTransform(){
    return m_absTrans;
}

void GLcamera::getDepthOfLine(int i_row, float *o_depth)
{
    glReadPixels(0, i_row, width(), 1, GL_DEPTH_COMPONENT, GL_FLOAT, o_depth);
}

void GLcamera::setViewSize(int w, int h)
{
    m_width = w;
    m_height = h;
}

GLlink *GLcamera::link()
{
    return m_link;
}
