#ifndef __BODY_STATE_H__
#define __BODY_STATE_H__

#include <hrpModel/Body.h>

class BodyState
{
public:
    void set(hrp::BodyPtr i_body);
    hrp::dvector q;
    hrp::Vector3 p;
    hrp::Matrix33 R;
};

#endif
