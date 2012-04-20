#include <cstdio>
#include <iostream>
#include <fstream>
#include <sys/time.h>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include "util/GLcamera.h"
#include "util/GLlink.h"
#include "util/GLbody.h"
#include "util/LogManager.h"
#include "SceneState.h"
#include "GLscene.h"

using namespace OpenHRP;
using namespace hrp;

void GLscene::updateScene()
{ 
    if (m_log->index()<0) return;

    LogManager<SceneState> *lm 
        = (LogManager<SceneState> *)m_log;
    SceneState &state = lm->state();
    
    for (unsigned int i=0; i<state.bodyStates.size(); i++){
        const BodyState& bstate = state.bodyStates[i];
        GLbody *body = m_bodies[i];
        body->setPosture(bstate.q, bstate.p, bstate.R);
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
    if (m_log->index()<0) return;

    LogManager<SceneState> *lm 
        = (LogManager<SceneState> *)m_log;
    SceneState &state = lm->state();

    glColor3f(1,0,0);
    double e[3];
    const CollisionSequence &cs = state.collisions;
    for (unsigned int i=0; i<cs.length(); i++){
        const CollisionPointSequence& cps = cs[i].points; 
        for (unsigned int j=0; j<cps.length(); j++){
            glVertex3dv(cps[j].position);
            for (int k=0; k<3; k++){
                e[k] = cps[j].position[k] + cps[j].normal[k]*(cps[j].idepth*10+0.1);
            }
            glVertex3dv(e);
        }
    }
}

void GLscene::showStatus()
{
    LogManager<SceneState> *lm 
        = (LogManager<SceneState> *)m_log;
    SceneState &state = lm->state();

    if (m_showingStatus){
        GLbody *body = NULL;
        BodyState *bstate = NULL;
        for (unsigned int i=0; i<m_bodies.size(); i++){
            if (m_bodies[i]->numJoints()){
                body = m_bodies[i];
                bstate = &state.bodyStates[i];
                break;
            }
        }
#define HEIGHT_STEP 12
        int width = m_width - 350;
        int height = m_height-HEIGHT_STEP;
        char buf[256];
        for (int i=0; i<body->numJoints(); i++){
            GLlink *l = body->joint(i);
            if (l){
                sprintf(buf, "%2d %15s %8.3f", i, l->name().c_str(),
                        bstate->q[i]*180/M_PI);
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


