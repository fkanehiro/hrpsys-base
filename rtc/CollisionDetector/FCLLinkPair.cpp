#include "FCLLinkPair.h"

FCLLinkPair::FCLLinkPair(hrp::Link* link0, FCLModel *fcl_model0,
                         hrp::Link* link1, FCLModel *fcl_model1, double tolerance)
{
    links_[0] = link0;
    links_[1] = link1;
    model1 = fcl_model0;
    model2 = fcl_model1;
    tolerance_ = tolerance;
}

FCLLinkPair::~FCLLinkPair()
{
}

bool FCLLinkPair::checkCollision()
{
    double p1[3], p2[3];
    double len = computeDistance(p1,p2);
    if ( len < tolerance_ ) {
        return true;
    }
    return false;
}

double FCLLinkPair::computeDistance(double *q1, double *q2)
{
    const hrp::Vector3&  p1 = links_[0]->p;
    hrp::Matrix33 r1 = links_[0]->attitude();
    const hrp::Vector3&  p2 = links_[1]->p;
    hrp::Matrix33 r2 = links_[1]->attitude();

    fcl::Vec3f fT1(p1(0), p1(1), p1(2));
    fcl::Vec3f fT2(p2(0), p2(1), p2(2));
    fcl::Matrix3f fR1(r1(0, 0), r1(0, 1), r1(0, 2),
                      r1(1, 0), r1(1, 1), r1(1, 2),
                      r1(2, 0), r1(2, 1), r1(2, 2));
    fcl::Matrix3f fR2(r2(0, 0), r2(0, 1), r2(0, 2),
                      r2(1, 0), r2(1, 1), r2(1, 2),
                      r2(2, 0), r2(2, 1), r2(2, 2));

    fcl::Transform3f pose1(fR1, fT1);
    fcl::Transform3f pose2(fR2, fT2);

    // calc distance
    bool enable_nearest_points = true;
    fcl::DistanceRequest request(enable_nearest_points);
    fcl::DistanceResult local_result;

    double len = fcl::distance(model1, pose1, model2, pose2, request, local_result);

    q1[0] = local_result.nearest_points[0][0];
    q1[1] = local_result.nearest_points[0][1];
    q1[2] = local_result.nearest_points[0][2];
    q2[0] = local_result.nearest_points[1][0];
    q2[1] = local_result.nearest_points[1][1];
    q2[2] = local_result.nearest_points[1][2];

#if 0
    // may not need to transform points
    fcl::Vec3f iq1(local_result.nearest_points[0][0], local_result.nearest_points[0][1], local_result.nearest_points[0][2]);
    fcl::Vec3f iq2(local_result.nearest_points[1][0], local_result.nearest_points[1][1], local_result.nearest_points[1][2]);

    fcl::Vec3f oq1 = pose1.transform(iq1);
    fcl::Vec3f oq2 = pose2.transform(iq2);
    q1[0] = oq1[0]; q1[1] = oq1[1]; q1[2] = oq1[2];
    q2[0] = oq2[0]; q2[1] = oq2[1]; q2[2] = oq2[2];
    std::cerr << "FCLCollision: compute: " << len << "/" << local_result.min_distance << " ";
    std::cerr << q1[0] << " " << q1[1] << " " << q1[2] << " / ";
    std::cerr << q2[0] << " " << q2[1] << " " << q2[2] << std::endl;
#endif

    return local_result.min_distance;
}
