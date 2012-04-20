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
    void addBody(const std::string &i_name, OpenHRP::BodyInfo_var i_binfo);
private:
    void updateScene();
    void drawAdditionalLines();

    SDL_sem *m_sem;
    std::string m_newBodyName;
    OpenHRP::BodyInfo_var m_newBodyInfo;
    bool m_isNewBody;
};

#endif
