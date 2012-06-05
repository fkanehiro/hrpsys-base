#ifndef __GLSCENE_H__
#define __GLSCENE_H__

#include <hrpModel/ColdetLinkPair.h>
#include "util/GLsceneBase.h"

class LogManagerBase;

class GLscene : public GLsceneBase
{
public:
    GLscene(LogManagerBase *i_log) : GLsceneBase(i_log) {}
    void setCollisionCheckPairs(const std::vector<hrp::ColdetLinkPairPtr> &i_pairs);
private:
    void updateScene();
    void showStatus();
    void drawAdditionalLines();
    std::vector<hrp::ColdetLinkPairPtr> m_pairs;
};
#endif
