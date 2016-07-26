#include <hrpModel/Body.h>
#include <hrpCollision/ColdetModelPair.h>

namespace hrp{

class SelfCollisionChecker
{
public:
    SelfCollisionChecker(hrp::BodyPtr body);
    std::vector<std::pair<std::string, std::string> > check(const double *q);
    unsigned int numOfCheckPairs() const { return m_checkPairs.size(); }
private:
    hrp::BodyPtr m_robot;
    std::vector<hrp::ColdetModelPair> m_checkPairs;
};

}
