#ifndef __GLSCENE_H__
#define __GLSCENE_H__

#include <hrpModel/ColdetLinkPair.h>
#include "util/GLsceneBase.h"

class LogManagerBase;

namespace CollisionDetectorComponent{

class GLscene : public GLsceneBase
{
public:
    GLscene(LogManagerBase *i_log) : GLsceneBase(i_log) {}
private:
    void drawAdditionalLines();
    void updateScene();
    void showStatus();
};

};
#endif
