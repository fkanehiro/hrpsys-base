#include <hrpModel/Body.h>
#include <hrpCollision/ColdetModelPair.h>

namespace hrp{

typedef std::vector<std::pair<std::string, std::string> > LinkNamePairList;

class SelfCollisionChecker
{
public:
    SelfCollisionChecker(hrp::BodyPtr body, 
                         const LinkNamePairList& pairs=LinkNamePairList());
    LinkNamePairList check(const double *q);
    unsigned int numOfCheckPairs() const { return m_checkPairs.size(); }
private:
    hrp::BodyPtr m_robot;
    std::vector<hrp::ColdetModelPair> m_checkPairs;
};

}
