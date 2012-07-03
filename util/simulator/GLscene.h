#ifndef __GLSCENE_H__
#define __GLSCENE_H__

#include "util/GLsceneBase.h"

class GLscene : public GLsceneBase
{
public:
    GLscene(LogManagerBase *i_log);
    void showSensors(bool flag);
    bool showSensors();
    void showCollision(bool flag);
    bool showCollision();
private:
    void drawAdditionalLines();
    void showStatus();
    void updateScene();
    void drawSensorOutput(hrp::Body *i_body, hrp::Sensor *i_sensor);

    bool m_showSensors, m_showCollision;
};

#endif
