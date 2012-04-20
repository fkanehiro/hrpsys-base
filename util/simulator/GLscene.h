#ifndef __GLSCENE_H__
#define __GLSCENE_H__

#include "util/GLsceneBase.h"

class GLscene : public GLsceneBase
{
public:
    GLscene(LogManagerBase *i_log) : GLsceneBase(i_log) {}
private:
    void drawAdditionalLines();
    void showStatus();
    void updateScene();
};

#endif
