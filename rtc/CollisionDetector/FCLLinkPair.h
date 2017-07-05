#ifdef USE_FCL
#ifndef FCL_LINK_PAIR_H
#define FCL_LINK_PAIR_H
#include <hrpModel/Link.h>

#include <fcl/distance.h>
#include <fcl/shape/geometric_shape_to_BVH_model.h>

#ifdef USE_FCL_MESH
//typedef fcl::OBBRSS FCLCollisionModel; // moderate
//typedef fcl::RSS FCLCollisionModel; // moderate
typedef fcl::AABB FCLCollisionModel; // fast
//typedef fcl::kIOS FCLCollisionModel; // slow
typedef fcl::BVHModel<FCLCollisionModel> FCLModel;
#else
typedef boost::shared_ptr <fcl::CollisionGeometry> FCLModel;
#endif

// FIX https://github.com/flexible-collision-library/fcl/pull/74
// https://stackoverflow.com/questions/5894940/calling-the-constructor-of-the-base-class-after-some-other-instructions-in-c
class ConvexFixed;
int Convex_PR_74(int num_points_, ConvexFixed& this_);
class ConvexFixed : public fcl::Convex
{
public:
  ConvexFixed(fcl::Vec3f* plane_normals_,
              fcl::FCL_REAL* plane_dis_,
              int num_planes_,
              fcl::Vec3f* points_,
              int num_points_,
              int* polygons_) : Convex(plane_normals_, plane_dis_, num_planes_, points_, Convex_PR_74(num_points_, *this), polygons_)
  {
  }
};


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
#endif // FCL_LINK_PAIR_H
