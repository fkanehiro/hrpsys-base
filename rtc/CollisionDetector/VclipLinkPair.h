#include <hrpModel/Link.h>
#include "vclip_1.0/include/vclip.h"

class VclipLinkPair  : public hrp::Referenced {
public:
    VclipLinkPair(hrp::Link* link0, Vclip::Polyhedron* pqp_model0, hrp::Link* link1, Vclip::Polyhedron* pqp_model1, double tolerance=0);
    ~VclipLinkPair();
    bool checkCollision();
    double computeDistance(double *q1, double *q2);
    hrp::Link* link(int index) { return links_[index]; }
    double getTolerance() { return tolerance_; }
    void setTolerance(double t) { tolerance_ = t; }

private:
    hrp::Link *links_[2];
    Vclip::Polyhedron *Vclip_Model1, *Vclip_Model2;
    Vclip::FeaturePair Feature_Pair;
    double tolerance_;
};

typedef boost::intrusive_ptr<VclipLinkPair> VclipLinkPairPtr;

