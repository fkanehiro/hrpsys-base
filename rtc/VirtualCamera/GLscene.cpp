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
#include "hrpsys/idl/HRPDataTypes.hh"
#include "GLscene.h"

using namespace OpenHRP;
using namespace hrp;

void drawString2(const char *str)
{
    for (unsigned int i=0; i<strlen(str); i++){
        glutBitmapCharacter(GLUT_BITMAP_8_BY_13, str[i]);
    }
}

void white(){
    glColor3d(1.0,1.0,1.0);
}

void red(){
    glColor3d(1.0,0.0,0.0);
}

void yellow(){
    glColor3d(1.0,1.0,0.0);
}

void green(){
    glColor3d(0.0,1.0,0.0);
}

void blue(){
    glColor3d(0.0,0.0,1.0);
}

void black(){
    glColor3d(0.0,0.0,0.0);
}

void GLscene::updateScene()
{
    if (m_log->index()<0) return;

    LogManager<OpenHRP::SceneState> *lm 
        = (LogManager<OpenHRP::SceneState> *)m_log;
    OpenHRP::SceneState &ss = lm->state();
    for (unsigned int i=0; i<ss.states.length(); i++){
        OpenHRP::RobotState &rs = ss.states[i];
        double pos[] = {rs.basePose.position.x,
                        rs.basePose.position.y,
                        rs.basePose.position.z};
        double rpy[] = {rs.basePose.orientation.r,
                        rs.basePose.orientation.p,
                        rs.basePose.orientation.y};
        GLbody *glbody = dynamic_cast<GLbody *>(body(i).get());
        glbody->setPosture(rs.q.get_buffer(), pos, rpy);
    }
}

void GLscene::showStatus()
{
    if (m_log->index()<0) return;

    LogManager<OpenHRP::SceneState> *lm 
        = (LogManager<OpenHRP::SceneState> *)m_log;
    OpenHRP::SceneState &sstate = lm->state();

    if (m_showingStatus){
        GLbody *glbody = NULL;
        OpenHRP::RobotState *rstate = NULL;
        for (unsigned int i=0; i<numBodies(); i++){
            if (body(i)->numJoints()){
                glbody = dynamic_cast<GLbody *>(body(i).get());
                rstate = &sstate.states[i];
                break;
            }
        }
#define HEIGHT_STEP 12
        int width = m_width - 410;
        int height = m_height-HEIGHT_STEP;
        char buf[256];
        for (unsigned int i=0; i<glbody->numJoints(); i++){
            hrp::Link *l = glbody->joint(i);
            if (l){
                int x = width;
                // joint ID
                sprintf(buf, "%2d",i);
                glRasterPos2f(x, height);
                drawString2(buf);
                white();
                x += 8*3;
                // joint name, current angle
                sprintf(buf, "%13s %8.3f", 
                        l->name.c_str(), 
                        rstate->q[i]*180/M_PI);
                glRasterPos2f(x, height);
                drawString2(buf);
                x += 8*(14+9);
                
                height -= HEIGHT_STEP;
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

void printMatrix(double mat[16])
{
    for (int i=0; i<4; i++){
        for (int j=0; j<4; j++){
            printf("%6.3f ", mat[j*4+i]);
        }
        printf("\n");
    }
}

