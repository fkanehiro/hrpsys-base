#include <iostream>
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#include <hrpModel/Body.h>
#include <hrpModel/Sensor.h>
#include <hrpModel/Light.h>
#include "GLutil.h"
#include "GLbody.h"
#include "GLcamera.h"
#include "GLlink.h"
#include "GLshape.h"

using namespace OpenHRP;
using namespace hrp;

bool GLlink::m_useAbsTransformToDraw = false;
int GLlink::m_drawMode = GLlink::DM_SOLID;

void GLlink::useAbsTransformToDraw()
{
    m_useAbsTransformToDraw = true;
}

GLlink::GLlink() : m_showAxes(false), m_highlight(false)
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
    if (m_drawMode != DM_COLLISION){
        for (size_t i=0; i<m_shapes.size(); i++){
            m_shapes[i]->draw(m_drawMode);
        }
        for (size_t i=0; i<m_cameras.size(); i++){
            m_cameras[i]->draw(m_drawMode);
        }
    }else{
        if (coldetModel && coldetModel->getNumTriangles()){
            Eigen::Vector3f n, v[3];
            int vindex[3];
            if (m_highlight){
                float red[] = {1,0,0,1};
                glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, red);
            }else{
                float gray[] = {0.8,0.8,0.8,1};
                glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, gray);
            }
            glBegin(GL_TRIANGLES);
            for (int i=0; i<coldetModel->getNumTriangles(); i++){
                coldetModel->getTriangle(i, vindex[0], vindex[1], vindex[2]);
                for (int j=0; j<3; j++){
                    coldetModel->getVertex(vindex[j], v[j][0], v[j][1], v[j][2]);
                }
                n = (v[1]-v[0]).cross(v[2]-v[0]);
                n.normalize();
                glNormal3fv(n.data());
                for (int j=0; j<3; j++){
                    glVertex3fv(v[j].data());
                }
            }
            glEnd();
        }
    }
    for (size_t i=0; i<sensors.size(); i++){
        Sensor *s = sensors[i];
        double T[16];
        for (int i=0; i<3; i++){
            for (int j=0; j<3; j++){
                T[i*4+j] = s->localR(j,i);
            }
        }
        T[12] = s->localPos[0]; T[13] = s->localPos[1]; T[14] = s->localPos[2];
        T[3] = T[7] = T[11] = 0.0; T[15] = 1.0;
        glPushMatrix();
        glMultMatrixd(T);
        GLbody *glb = dynamic_cast<GLbody *>(body);
        glb->drawSensor(s);
        glPopMatrix();
    }
    for (size_t i=0; i<lights.size(); i++){
        Light *l = lights[i];
        if (!l->on) continue;
        double T[16];
        for (int j=0; j<3; j++){
            for (int k=0; k<3; k++){
                T[j*4+k] = l->localR(k,j);
            }
        }
        T[12] = l->localPos[0]; T[13] = l->localPos[1]; T[14] = l->localPos[2];
        T[3] = T[7] = T[11] = 0.0; T[15] = 1.0;
        glPushMatrix();
        glMultMatrixd(T);
        glEnable(GL_LIGHTING);
        int lid = GL_LIGHT0+l->id; 
        glEnable(lid);
        GLfloat pos[] = {0,0,0,1};
        glLightfv(lid, GL_POSITION, pos);
        GLfloat color[] = {l->color[0], l->color[1], l->color[2], 1};
        glLightfv(lid, GL_DIFFUSE,  color);
        if (l->type == POINT){
            glLightf(lid, GL_CONSTANT_ATTENUATION,  l->attenuation[0]);
            glLightf(lid, GL_LINEAR_ATTENUATION,    l->attenuation[1]);
            glLightf(lid, GL_QUADRATIC_ATTENUATION, l->attenuation[2]);
        }else if(l->type == SPOT){
            glLightf(lid, GL_CONSTANT_ATTENUATION,  l->attenuation[0]);
            glLightf(lid, GL_LINEAR_ATTENUATION,    l->attenuation[1]);
            glLightf(lid, GL_QUADRATIC_ATTENUATION, l->attenuation[2]);
            //glLightf(lid, GL_SPOT_EXPONENT, 20);
            glLightf(lid, GL_SPOT_CUTOFF, l->cutOffAngle*180/M_PI);
            GLfloat dir[] = {l->direction[0], l->direction[1], l->direction[2]};
            glLightfv(lid, GL_SPOT_DIRECTION, dir);
        }else if(l->type == DIRECTIONAL){
            GLfloat dir[] = {l->direction[0], l->direction[1], l->direction[2]};
            glLightfv(lid, GL_SPOT_DIRECTION, dir);
        }
        glPopMatrix();
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
    Vector3 aLocal(Rs.transpose()*a);
    hrp::calcRodrigues(R, aLocal, i_q);
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
    if (m_useAbsTransformToDraw){
        memcpy(o_trans, m_absTrans, sizeof(double)*16);
        return;
    }
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

int GLlink::drawMode()
{
    return m_drawMode;
}

void GLlink::drawMode(int i_mode)
{
    m_drawMode = i_mode;
}

void GLlink::highlight(bool flag)
{
    m_highlight = flag;
    for (size_t i=0; i<m_shapes.size(); i++){
        m_shapes[i]->highlight(flag);
    }
    for (size_t i=0; i<m_cameras.size(); i++){
        m_cameras[i]->highlight(flag);
    }
}
