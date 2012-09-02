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
