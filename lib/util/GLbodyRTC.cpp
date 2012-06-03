#include "util/GLbodyRTC.h"

const char* GLbodyRTC::glbodyrtc_spec[] =
{
    "implementation_id", "GLbodyRTC",
    "type_name",         "GLbodyRTC",
    "description",       "GLbodyRTC component",
    "version",           "0.1",
    "vendor",            "AIST",
    "category",          "Generic",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables

    ""
};

GLbodyRTC::GLbodyRTC(RTC::Manager* manager) : BodyRTC(manager)
{
}

template <class _Delete>
void DummyDelete(RTC::RTObject_impl* rtc)
{
}

void GLbodyRTC::moduleInit(RTC::Manager* manager)
{
    coil::Properties profile(glbodyrtc_spec);
    manager->registerFactory(profile,
                             RTC::Create<GLbodyRTC>,
                             DummyDelete<GLbodyRTC>
                             //RTC::Delete<GLbodyRTC>
        );
}
