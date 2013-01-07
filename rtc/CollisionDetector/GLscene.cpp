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

using namespace OpenHRP;
using namespace hrp;
using namespace CollisionDetectorComponent;

void GLscene::updateScene()
{
    if (m_log->index()<0) return;

    LogManager<TimedPosture> *lm 
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
}

void GLscene::drawAdditionalLines()
{
    if (m_log->index()<0) return;

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
}
