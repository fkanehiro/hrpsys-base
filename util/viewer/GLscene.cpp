#include <cstdio>
#include <iostream>
#include <fstream>
#include <sys/time.h>
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#include "util/GLcamera.h"
#include "util/GLlink.h"
#include "util/GLbody.h"
#include "util/LogManager.h"
#include "GLscene.h"

using namespace OpenHRP;
using namespace hrp;

void GLscene::updateScene()
{ 
    if (m_log->index()<0) return;

    LogManager<OpenHRP::WorldState> *lm 
        = (LogManager<OpenHRP::WorldState> *)m_log;
    OpenHRP::WorldState &state = lm->state();
    for (unsigned int i=0; i<state.characterPositions.length(); i++){
        const CharacterPosition& cpos = state.characterPositions[i];
        std::string cname(cpos.characterName);
        GLbody *glbody = dynamic_cast<GLbody *>(body(cname).get());
        if (!glbody) {
            //std::cerr << "can't find a body named " << cname << std::endl;
            continue;
        }
        for (unsigned int j=0; j<cpos.linkPositions.length(); j++){
            const LinkPosition &lp = cpos.linkPositions[j];
            double T[] = {lp.R[0], lp.R[3], lp.R[6],0,
                          lp.R[1], lp.R[4], lp.R[7],0,
                          lp.R[2], lp.R[5], lp.R[8],0,
                          lp.p[0], lp.p[1], lp.p[2],1};
#if 0
            for (int i=0; i<4; i++){
                for (int j=0; j<4; j++){
                    printf("%6.3f ", T[i*4+j]);
                }
                printf("\n");
            }
            printf("\n");
#endif
            ((GLlink *)glbody->link(j))->setAbsTransform(T);
        }
    }
}

void GLscene::drawAdditionalLines()
{
    if (m_log->index()<0) return;

    LogManager<OpenHRP::WorldState> *lm 
        = (LogManager<OpenHRP::WorldState> *)m_log;
    OpenHRP::WorldState &state = lm->state();

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

GLscene::GLscene(LogManagerBase *i_log) 
  : GLsceneBase(i_log)
{
}

GLscene::~GLscene()
{
}
