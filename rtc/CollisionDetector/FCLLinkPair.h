#include <hrpModel/Link.h>

#include <fcl/distance.h>
#include <fcl/shape/geometric_shape_to_BVH_model.h>

typedef fcl::OBBRSS FCLCollisionModel;
typedef fcl::BVHModel<FCLCollisionModel> FCLModel;

class FCLLinkPair  : public hrp::Referenced {
public:
    FCLLinkPair(hrp::Link* link0, FCLModel *fcl_model0,
                hrp::Link* link1, FCLModel *fcl_model1, double tolerance=0);
    ~FCLLinkPair();
    bool checkCollision();
    double computeDistance(double *q1, double *q2);
    hrp::Link* link(int index) { return links_[index]; }
    double getTolerance() { return tolerance_; }
    void setTolerance(double t) { tolerance_ = t; }

private:
    hrp::Link *links_[2];
    FCLModel *model1, *model2;

    double tolerance_;
};

typedef boost::intrusive_ptr<FCLLinkPair> FCLLinkPairPtr;
