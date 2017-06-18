#ifndef __GLSCENE_H__
#define __GLSCENE_H__

#include "hrpsys/util/GLsceneBase.h"

class LogManagerBase;

class GLscene : public GLsceneBase {
 public:
  GLscene(LogManagerBase *i_log) : GLsceneBase(i_log) {}

 private:
  void updateScene();
  void showStatus();
};
#endif
