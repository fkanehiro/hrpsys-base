#ifndef __GLMODEL_H__
#define __GLMODEL_H__

#include "util/GLsceneBase.h"

class LogManagerBase;

class GLscene : public GLsceneBase
{
public:
    GLscene(LogManagerBase *i_log) : GLsceneBase(i_log) {}
private:
    void updateScene();
    void showStatus();
};
#endif
