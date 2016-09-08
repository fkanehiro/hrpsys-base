#include <cstdio>
#include <iostream>
#include <fstream>
#include <sys/time.h>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include <boost/bind.hpp>
#include <hrpModel/Sensor.h>
#include "hrpsys/util/GLcamera.h"
#include "hrpsys/util/GLlink.h"
#include "hrpsys/util/GLbody.h"
#include "hrpsys/util/LogManager.h"
#include "SceneState.h"
#include "GLscene.h"

using namespace OpenHRP;
using namespace hrp;

GLscene::GLscene(LogManagerBase *i_log) : 
    GLsceneBase(i_log), 
    m_showSensors(false),
    m_showCollision(true)
{
}

void GLscene::updateScene()
{ 
    if (m_log->index()<0) return;

    LogManager<SceneState> *lm 
        = (LogManager<SceneState> *)m_log;
    SceneState &state = lm->state();
    
    for (unsigned int i=0; i<state.bodyStates.size(); i++){
        const BodyState& bstate = state.bodyStates[i];
        GLbody *glbody = dynamic_cast<GLbody *>(body(i).get());
        glbody->setPosture(bstate.q, bstate.p, bstate.R);
        if (m_showSensors){
            glbody->setSensorDrawCallback(
                boost::bind(&GLscene::drawSensorOutput, this, _1, _2));
        }else{
            glbody->setSensorDrawCallback(NULL);
        }
    }
}

static void drawString2(const char *str)
{
    for (unsigned int i=0; i<strlen(str); i++){
        glutBitmapCharacter(GLUT_BITMAP_8_BY_13, str[i]);
    }
}

void GLscene::drawAdditionalLines()
{
    if (!m_showCollision || m_log->index()<0) return;

    LogManager<SceneState> *lm 
        = (LogManager<SceneState> *)m_log;
    SceneState &state = lm->state();

    glBegin(GL_LINES);
    glColor3f(1,0,0);
    double e[3];
    const std::vector<CollisionInfo> &cs = state.collisions;
    for (unsigned int i=0; i<cs.size(); i++){
        for (int k=0; k<3; k++){
            e[k] = cs[i].position[k] + cs[i].normal[k]*(cs[i].idepth*10+0.1);
        }
        glVertex3dv(cs[i].position);
        glVertex3dv(e);
    }
    glEnd();
}

void GLscene::showStatus()
{
    if (m_log->index()<0) return;

    LogManager<SceneState> *lm 
        = (LogManager<SceneState> *)m_log;
    SceneState &state = lm->state();

    if (m_showingStatus){
        GLbody *glbody = NULL;
        BodyState *bstate = NULL;
        for (unsigned int i=0; i<numBodies(); i++){
            if (body(i)->numJoints()){
                glbody = dynamic_cast<GLbody *>(body(i).get());
                bstate = &state.bodyStates[i];
                break;
            }
        }
        if (!glbody) return;
#define HEIGHT_STEP 12
        int width = m_width - 350;
        int height = m_height-HEIGHT_STEP;
        char buf[256];
        double q[glbody->numJoints()];
        for (unsigned int i=0; i<glbody->numLinks(); i++){
            Link* l = glbody->link(i);
            if (l->jointId >= 0) q[l->jointId] = bstate->q[i];
        }
        for (unsigned int i=0; i<glbody->numJoints(); i++){
            GLlink *l = (GLlink *)glbody->joint(i);
            if (l){
                sprintf(buf, "%2d %15s %8.3f", i, l->name.c_str(),
                        q[i]*180/M_PI);
                glRasterPos2f(width, height);
                height -= HEIGHT_STEP;
                drawString2(buf);
            }
        }
        if (bstate->acc.size()){
            glRasterPos2f(width, height);
            height -= HEIGHT_STEP;
            drawString2("acc:");
            for (unsigned int i=0; i<bstate->acc.size(); i++){
                sprintf(buf, "  %8.4f %8.4f %8.4f",
                        bstate->acc[i][0], bstate->acc[i][1], bstate->acc[i][2]);
                glRasterPos2f(width, height);
                height -= HEIGHT_STEP;
                drawString2(buf);
            }
        }
        if (bstate->rate.size()){
            glRasterPos2f(width, height);
            height -= HEIGHT_STEP;
            drawString2("rate:");
            for (unsigned int i=0; i<bstate->rate.size(); i++){
                sprintf(buf, "  %8.4f %8.4f %8.4f",
                        bstate->rate[i][0], bstate->rate[i][1], bstate->rate[i][2]);
                glRasterPos2f(width, height);
                height -= HEIGHT_STEP;
                drawString2(buf);
            }
        }
        if (bstate->force.size()){
            glRasterPos2f(width, height);
            height -= HEIGHT_STEP;
            drawString2("force/torque:");
            for (unsigned int i=0; i<bstate->force.size(); i++){
                sprintf(buf, "  %6.1f %6.1f %6.1f %6.2f %6.2f %6.2f",
                        bstate->force[i][0], 
                        bstate->force[i][1], 
                        bstate->force[i][2],
                        bstate->force[i][3], 
                        bstate->force[i][4], 
                        bstate->force[i][5]);
                glRasterPos2f(width, height);
                height -= HEIGHT_STEP;
                drawString2(buf);
            }
        }
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_BLEND);
        glColor4f(0.0,0.0,0.0, 0.5);
        if (m_showSlider){
            glRectf(width,SLIDER_AREA_HEIGHT,m_width,m_height);
        }else{
            glRectf(width,0,m_width,m_height);
        }
        glDisable(GL_BLEND);
    }
}


void GLscene::drawSensorOutput(Body *body, Sensor *sensor)
{
    if (m_log->index()<0) return;

    LogManager<SceneState> *lm 
        = (LogManager<SceneState> *)m_log;
    SceneState &sstate = lm->state();
    if (bodyIndex(body->name())<0){
        std::cerr << "invalid bodyIndex(" << bodyIndex(body->name()) 
                  << ") for " << body->name() << std::endl;
        return;
    } 
    const BodyState &state = sstate.bodyStates[bodyIndex(body->name())];

    if (sensor->type == Sensor::RANGE){
        RangeSensor *range = dynamic_cast<RangeSensor *>(sensor);
        const std::vector<double> distances = state.range[sensor->id];
        if (distances.empty()) return;
        int scan_half = (int)(range->scanAngle/2/range->scanStep);
        double th;
        Vector3 v;
        v[1] = 0.0;
        glDisable(GL_LIGHTING);
        glBegin(GL_LINES);
        glColor3f(1,0,0);
        for (int i = -scan_half,j=0; i<= scan_half; i++,j++){
            th = i*range->scanStep;
            double d = distances[j] ? distances[j] : range->maxDistance;
            v[0] = -d*sin(th); 
            v[2] = -d*cos(th); 
            glVertex3f(0,0,0); glVertex3f(v[0], v[1], v[2]);
        }
        glEnd();
        glEnable(GL_LIGHTING);
    }else if(sensor->type == Sensor::VISION){
        VisionSensor *v = dynamic_cast<VisionSensor *>(sensor);
        double far = v->far, near = v->near;
        glDisable(GL_LIGHTING);
        glColor3f(1,1,1);
        glBegin(GL_LINES);
        double t = tan(v->fovy/2);
        double xf = t*far*v->width/v->height, yf = t*far;
        glVertex3f( xf,  yf, -far); glVertex3f(-xf,  yf, -far);
        glVertex3f(-xf,  yf, -far); glVertex3f(-xf, -yf, -far);
        glVertex3f(-xf, -yf, -far); glVertex3f( xf, -yf, -far);
        glVertex3f( xf, -yf, -far); glVertex3f( xf,  yf, -far); 
        double xn = t*near*v->width/v->height, yn = t*near;
        glVertex3f( xn,  yn, -near); glVertex3f(-xn,  yn, -near);
        glVertex3f(-xn,  yn, -near); glVertex3f(-xn, -yn, -near);
        glVertex3f(-xn, -yn, -near); glVertex3f( xn, -yn, -near);
        glVertex3f( xn, -yn, -near); glVertex3f( xn,  yn, -near);
        glVertex3f( xn,  yn, -near); glVertex3f( xf,  yf, -far);
        glVertex3f(-xn,  yn, -near); glVertex3f(-xf,  yf, -far);
        glVertex3f(-xn, -yn, -near); glVertex3f(-xf, -yf, -far);
        glVertex3f( xn, -yn, -near); glVertex3f( xf, -yf, -far);
        glEnd();
        if (v->imageType == VisionSensor::DEPTH 
            || v->imageType == VisionSensor::COLOR_DEPTH 
            || v->imageType == VisionSensor::MONO_DEPTH){
            bool colored = v->imageType == VisionSensor::COLOR_DEPTH;
            glBegin(GL_POINTS);
            float *ptr = (float *)&v->depth[0];
            for (unsigned int i=0; i<v->depth.size()/16; i++){
                glVertex3f(ptr[0], ptr[1], ptr[2]);
                if (colored){
                    ptr += 3;
                    unsigned char *rgb = (unsigned char *)ptr;
                    glColor3f(rgb[0]/255.0, rgb[1]/255.0, rgb[2]/255.0);
                    ptr++;
                }else{
                    ptr += 4;
                }
            }
            glEnd();
        } 

        glEnable(GL_LIGHTING);
    }
}

void GLscene::showSensors(bool flag)
{
    m_showSensors = flag;
}

bool GLscene::showSensors()
{
    return m_showSensors;
}

void GLscene::showCollision(bool flag)
{
    m_showCollision = flag;
}

bool GLscene::showCollision()
{
    return m_showCollision;
}
