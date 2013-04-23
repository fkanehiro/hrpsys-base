#include <cstdio>
#include <iostream>
#include <fstream>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include <sys/time.h>
#include "util/GLcamera.h"
#include "util/GLlink.h"
#include "util/GLbody.h"
#include "util/LogManager.h"
#include "TimedPosture.h"
#include "GLscene.h"
#include "CollisionDetectorService.hh"


using namespace OpenHRP;
using namespace hrp;
using namespace CollisionDetectorComponent;

static void drawString(const char *str)
{
    for (unsigned int i=0; i<strlen(str); i++){
        glutBitmapCharacter(GLUT_BITMAP_9_BY_15, str[i]);
    }
}

void GLscene::updateScene()
{
    if (m_log->index()<0) return;

#ifdef USE_COLLISION_STATE
    LogManager<OpenHRP::CollisionDetectorService::CollisionState> *lm
        = (LogManager<OpenHRP::CollisionDetectorService::CollisionState> *)m_log;
    GLbody *glbody = dynamic_cast<GLbody *>(body(0).get());
    OpenHRP::CollisionDetectorService::CollisionState &co = lm->state();
    if (co.angle.length() == glbody->numJoints()){
        for (int i=0; i<glbody->numJoints(); i++){
            GLlink *j = (GLlink *)glbody->joint(i);
            if (j){
                j->setQ(co.angle[i]);
            }
        }
    }
#else
    LogManager<OpenHRP::CollisionDetectorService::CollisionState> *lm 
        = (LogManager<TimedPosture> *)m_log;
    GLbody *glbody = dynamic_cast<GLbody *>(body(0).get());
    TimedPosture &ts = lm->state();
    if (ts.posture.size() == glbody->numJoints()){
        for (int i=0; i<glbody->numJoints(); i++){
            GLlink *j = (GLlink *)glbody->joint(i);
            if (j){
                j->setQ(ts.posture[i]);
            }
        }
    }
#endif
}

void GLscene::drawAdditionalLines()
{
    if (m_log->index()<0) return;

#ifdef USE_COLLISION_STATE
    LogManager<OpenHRP::CollisionDetectorService::CollisionState> *lm
        = (LogManager<OpenHRP::CollisionDetectorService::CollisionState> *)m_log;
    OpenHRP::CollisionDetectorService::CollisionState &co = lm->state();

    glBegin(GL_LINES);
    glColor3f(1,0,0);
    for (unsigned int i=0; i<co.lines.length(); i++){
        GLdouble p0[3], p1[3];
        p0[0] = co.lines[i][0][0];
        p0[1] = co.lines[i][0][1];
        p0[2] = co.lines[i][0][2];
        p1[0] = co.lines[i][1][0];
        p1[1] = co.lines[i][1][1];
        p1[2] = co.lines[i][1][2];
        glVertex3dv(p0);
        glVertex3dv(p1);
    }
    glEnd();

    glPointSize(4.0);
    glBegin(GL_POINTS);
    glColor3f(1,0,0);
    for (unsigned int i=0; i<co.lines.length(); i++){
        GLdouble p0[3], p1[3];
        p0[0] = co.lines[i][0][0];
        p0[1] = co.lines[i][0][1];
        p0[2] = co.lines[i][0][2];
        p1[0] = co.lines[i][1][0];
        p1[1] = co.lines[i][1][1];
        p1[2] = co.lines[i][1][2];
        glVertex3dv(p0);
        glVertex3dv(p1);
    }
    glEnd();
#else
    LogManager<TimedPosture> *lm 
        = (LogManager<TimedPosture> *)m_log;
    TimedPosture &tp = lm->state();

    glBegin(GL_LINES);
    glColor3f(1,0,0);
    for (unsigned int i=0; i<tp.lines.size(); i++){
        const std::pair<hrp::Vector3, hrp::Vector3>& line = tp.lines[i];
        glVertex3dv(line.first.data());
        glVertex3dv(line.second.data());
    }
    glEnd();

    glPointSize(4.0);
    glBegin(GL_POINTS);
    glColor3f(1,0,0);
    for (unsigned int i=0; i<tp.lines.size(); i++){
        const std::pair<hrp::Vector3, hrp::Vector3>& line = tp.lines[i];
        glVertex3dv(line.first.data());
        glVertex3dv(line.second.data());
    }
    glEnd();
#endif
}

void GLscene::showStatus()
{
    char buf[256];

    GLbody *glbody = dynamic_cast<GLbody *>(body(0).get());
    int width = m_width - 220;
#define HEIGHT_STEP 12
    int height = m_height-HEIGHT_STEP;
    int x = width;

    for (int i=0; i<glbody->numLinks(); i++){
        hrp::Link *l = glbody->link(i);
        if (l){
            sprintf(buf, "%13s %4d tris",
                    l->name.c_str(),
                    l->coldetModel->getNumTriangles());
            glRasterPos2f(x, height);
            drawString(buf);
            height -= HEIGHT_STEP;
        }
    }

    if (m_log->index()<0) return;

    LogManager<OpenHRP::CollisionDetectorService::CollisionState> *lm
        = (LogManager<OpenHRP::CollisionDetectorService::CollisionState> *)m_log;
    OpenHRP::CollisionDetectorService::CollisionState &co = lm->state();

    height -= HEIGHT_STEP;

    x = width - 34;
    sprintf(buf, "Number of pair     %8d",  co.lines.length());
    glRasterPos2f(x, height);
    drawString(buf);
    height -= HEIGHT_STEP;

    sprintf(buf, "Calc Time [msec]   %8.3f",  co.computation_time);
    glRasterPos2f(x, height);
    drawString(buf);
    height -= HEIGHT_STEP;

    sprintf(buf, "Recover Time[msec] %8.3f",  co.recover_time);
    glRasterPos2f(x, height);
    drawString(buf);
    height -= HEIGHT_STEP;

    sprintf(buf, "Safe Posture       %8s",  co.safe_posture?"true":"false");
    glRasterPos2f(x, height);
    drawString(buf);
    height -= HEIGHT_STEP;

    sprintf(buf, "Loop for check     %8d",  co.loop_for_check);
    glRasterPos2f(x, height);
    drawString(buf);
    height -= HEIGHT_STEP;

#if 0

    if (m_showingStatus){
        GLbody *glbody = dynamic_cast<GLbody *>(body(0).get());
#define HEIGHT_STEP 12
        int width = m_width - 410;
        int height = m_height-HEIGHT_STEP;
        char buf[256];
        for (int i=0; i<glbody->numJoints(); i++){
            hrp::Link *l = glbody->joint(i);
            if (l){
                int ss = rstate.servoState[i][0];
                int x = width;
                // joint ID
                sprintf(buf, "%2d",i);
                if (!isCalibrated(ss)){
                    yellow();
                }else if(isServoOn(ss)){
                    red();
                }
                glRasterPos2f(x, height);
                drawString2(buf);
                white();
                x += 8*3;
                // power status
                if (isPowerOn(ss)) blue();
                glRasterPos2f(x, height);
                drawString2("o");
                if (isPowerOn(ss)) white();
                x += 8*2;
                // joint name, current angle, command angle and torque
                sprintf(buf, "%13s %8.3f %8.3f %6.1f", 
                        l->name.c_str(), 
                        rstate.angle[i]*180/M_PI,
                        rstate.command[i]*180/M_PI,
                        rstate.torque[i]*180/M_PI);
                glRasterPos2f(x, height);
                drawString2(buf);
                x += 8*(14+9+9+7);
                // servo alarms
                sprintf(buf, "%03x", servoAlarm(ss));
                glRasterPos2f(x, height);
                drawString2(buf);
                x += 8*4;
                // driver temperature
                int temp = temperature(ss);
                if (!temp){
                    sprintf(buf, "--", temp);
                }else{
                    sprintf(buf, "%2d", temp);
                }
                if (temp >= 60) red();
                glRasterPos2f(x, height);
                drawString2(buf);
                if (temp >= 60) white();
                x += 8*3;
                
                height -= HEIGHT_STEP;
            }
        }
        if (rstate.accel.length()){
            glRasterPos2f(width, height);
            height -= HEIGHT_STEP;
            drawString2("acc:");
            for (unsigned int i=0; i<rstate.accel.length(); i++){
                sprintf(buf, "  %8.4f %8.4f %8.4f",
                        rstate.accel[i][0], rstate.accel[i][1], rstate.accel[i][2]);
                glRasterPos2f(width, height);
                height -= HEIGHT_STEP;
                drawString2(buf);
            }
        }
        if (rstate.rateGyro.length()){
            glRasterPos2f(width, height);
            height -= HEIGHT_STEP;
            drawString2("rate:");
            for (unsigned int i=0; i<rstate.rateGyro.length(); i++){
                sprintf(buf, "  %8.4f %8.4f %8.4f",
                        rstate.rateGyro[i][0], rstate.rateGyro[i][1], rstate.rateGyro[i][2]);
                glRasterPos2f(width, height);
                height -= HEIGHT_STEP;
                drawString2(buf);
            }
        }
        if (rstate.force.length()){
            glRasterPos2f(width, height);
            height -= HEIGHT_STEP;
            drawString2("force/torque:");
            for (unsigned int i=0; i<rstate.force.length(); i++){
                sprintf(buf, "  %6.1f %6.1f %6.1f %6.2f %6.2f %6.2f",
                        rstate.force[i][0], 
                        rstate.force[i][1], 
                        rstate.force[i][2],
                        rstate.force[i][3], 
                        rstate.force[i][4], 
                        rstate.force[i][5]);
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
    }else{
        // !m_showingRobotState
        bool servo=false, power=false;
        for (unsigned int i=0; i<rstate.servoState.length(); i++){
            if (isServoOn(rstate.servoState[i][0])) servo = true;
            if (isPowerOn(rstate.servoState[i][0])) power = true;
        }
        struct timeval tv;
        gettimeofday(&tv, NULL);
        double dt = tv.tv_sec + tv.tv_usec/1e6 - lm->time(m_log->length()-1);
        if (dt < 1.0) green(); else black();
        glRectf(m_width-115,m_height-45,m_width-85,m_height-15);
        if (power) blue(); else black();
        glRectf(m_width- 80,m_height-45,m_width-50,m_height-15);
        if (servo) red(); else black();
        glRectf(m_width- 45,m_height-45,m_width-15,m_height-15);
    }
#endif
}


