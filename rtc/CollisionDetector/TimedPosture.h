#include <vector>
#include <hrpUtil/EigenTypes.h>

class TimedPosture{
public:
    double time;
    std::vector<double> posture;
    std::vector<std::pair<hrp::Vector3, hrp::Vector3> > lines;
};
