#include <hrpModel/Link.h>
#include "vclip_1.0/include/vclip.h"
#include "CollisionLibraryLinkPair.h"

class VclipLinkPair  : public CollisionLibraryLinkPair {
public:
    VclipLinkPair(hrp::Link* link0, Vclip::Polyhedron* pqp_model0, hrp::Link* link1, Vclip::Polyhedron* pqp_model1, double tolerance=0);
    ~VclipLinkPair();
    bool checkCollision();
    double computeDistance(double *q1, double *q2);

private:
    Vclip::Polyhedron *Vclip_Model1, *Vclip_Model2;
    Vclip::FeaturePair Feature_Pair;
};

typedef boost::intrusive_ptr<VclipLinkPair> VclipLinkPairPtr;

