#ifndef __GLSCENE_H__
#define __GLSCENE_H__

#include <SDL/SDL_thread.h>
#include <hrpCorba/ModelLoader.hh>
#include "util/GLsceneBase.h"

class LogManagerBase;

class GLscene : public GLsceneBase
{
public:
    GLscene(LogManagerBase *i_log);
    ~GLscene();
private:
    void updateScene();
    void drawAdditionalLines();
};

#endif
