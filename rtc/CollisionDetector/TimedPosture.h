// -*- coding:utf-8-unix; mode: c++; indent-tabs-mode: nil; c-basic-offset: 2; -*-
#include <vector>
#include <hrpUtil/EigenTypes.h>

class TimedPosture{
public:
    double time;
    std::vector<double> posture;
    std::vector<std::pair<hrp::Vector3, hrp::Vector3> > lines;
};
