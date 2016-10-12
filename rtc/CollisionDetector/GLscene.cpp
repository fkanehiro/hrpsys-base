#include <cstdio>
#include <iostream>
#include <fstream>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include <sys/time.h>
#include "hrpsys/util/GLcamera.h"
#include "hrpsys/util/GLlink.h"
#include "hrpsys/util/GLbody.h"
#include "hrpsys/util/LogManager.h"
#include "TimedPosture.h"
#include "GLscene.h"
#include "hrpsys/idl/CollisionDetectorService.hh"


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
        for (unsigned int i=0; i<glbody->numJoints(); i++){
            GLlink *j = (GLlink *)glbody->joint(i);
            if (j){
                j->setQ(co.angle[i]);
            }
        }
    }
#else
    LogManager<TimedPosture> *lm 
        = (LogManager<TimedPosture> *)m_log;
    GLbody *glbody = dynamic_cast<GLbody *>(body(0).get());
    TimedPosture &ts = lm->state();
    if (ts.posture.size() == glbody->numJoints()){
        for (unsigned int i=0; i<glbody->numJoints(); i++){
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

    for (unsigned int i=0; i<glbody->numLinks(); i++){
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

}


