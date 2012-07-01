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
    std::vector<hrp::Vector3> acc;
    std::vector<hrp::Vector3> rate;
    std::vector<hrp::dvector6, Eigen::aligned_allocator<hrp::dvector6> > force;
    std::vector<std::vector<double> > range;
};

#endif
