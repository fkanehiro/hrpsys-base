#ifndef __GLBODYRTC_H__
#define __GLBODYRTC_H__

#include "util/BodyRTC.h"
#include "util/GLbody.h"

class GLbodyRTC : public BodyRTC, public GLbody
{
public:
    GLbodyRTC(RTC::Manager* manager = &RTC::Manager::instance());
    static void moduleInit(RTC::Manager*);
private:
    static const char* glbodyrtc_spec[];
};

typedef boost::intrusive_ptr<GLbodyRTC> GLbodyRTCPtr;

#endif
