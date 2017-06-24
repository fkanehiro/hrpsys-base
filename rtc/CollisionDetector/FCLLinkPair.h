#ifdef USE_FCL
#include <hrpModel/Link.h>

#include <fcl/distance.h>
#include <fcl/shape/geometric_shape_to_BVH_model.h>

//typedef fcl::OBBRSS FCLCollisionModel; // moderate
//typedef fcl::RSS FCLCollisionModel; // moderate
typedef fcl::AABB FCLCollisionModel; // fast
//typedef fcl::kIOS FCLCollisionModel; // slow
typedef fcl::BVHModel<FCLCollisionModel> FCLModel;

#include "CollisionLibraryLinkPair.h"

class FCLLinkPair  : public CollisionLibraryLinkPair {
public:
    FCLLinkPair(hrp::Link* link0, FCLModel *fcl_model0,
                hrp::Link* link1, FCLModel *fcl_model1, double tolerance=0);
    ~FCLLinkPair();
    bool checkCollision();
    double computeDistance(double *q1, double *q2);

private:
    FCLModel *model1, *model2;
};

typedef boost::intrusive_ptr<FCLLinkPair> FCLLinkPairPtr;
#endif // USE_FCL
