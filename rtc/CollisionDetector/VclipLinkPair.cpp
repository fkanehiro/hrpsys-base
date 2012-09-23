#include "VclipLinkPair.h"

VclipLinkPair::VclipLinkPair(hrp::Link* link0, Vclip::Polyhedron* vclip_model0, hrp::Link* link1, Vclip::Polyhedron* vclip_model1, double tolerance)
{
    links_[0] = link0;
    links_[1] = link1;
    Vclip_Model1 = vclip_model0;
    Vclip_Model2 = vclip_model1;
    tolerance_ = tolerance;
    Feature_Pair.first  = (const Vclip::Feature *)new Vclip::Vertex(Vclip_Model1->verts().front());
    Feature_Pair.second = (const Vclip::Feature *)new Vclip::Vertex(Vclip_Model2->verts().front());
}
VclipLinkPair::~VclipLinkPair()
{
}

bool VclipLinkPair::checkCollision()
{
    double p1[3], p2[3];
    double len = computeDistance(p1,p2);
    if ( len < tolerance_ ) {
        return true;
    }
    return false;
}

double VclipLinkPair::computeDistance(double *q1, double *q2)
{
    Vclip::Mat3 R1, R2;
    Vclip::Vect3 T1, T2;
    Vclip::VclipPose P1, P2;
    const hrp::Vector3&  p1 = links_[0]->p;
    hrp::Matrix33 r1 = links_[0]->attitude();
    const hrp::Vector3&  p2 = links_[1]->p;
    hrp::Matrix33 r2 = links_[1]->attitude();
    R1.xrow().set(r1(0,0), r1(0,1), r1(0,2));
    R1.yrow().set(r1(1,0), r1(1,1), r1(1,2));
    R1.zrow().set(r1(2,0), r1(2,1), r1(2,2));
    R2.xrow().set(r2(0,0), r2(0,1), r2(0,2));
    R2.yrow().set(r2(1,0), r2(1,1), r2(1,2));
    R2.zrow().set(r2(2,0), r2(2,1), r2(2,2));
    T1.set(p1(0), p1(1), p1(2));
    T2.set(p2(0), p2(1), p2(2));
    P1.set(R1, T1);
    P2.set(R2, T2);
    Vclip::VclipPose X12, X21;
    X12.invert(P2);
    X12.postmult(P1);
    X21.invert(X12);
    Vclip::Vect3 cp1, cp2;
    double len = Vclip::Polyhedron::vclip(Vclip_Model1, Vclip_Model2, X12, X21, Feature_Pair.first, Feature_Pair.second, cp1, cp2, 0);
    Vclip::Vect3 cp1g, cp2g;
    P1.xformPoint(cp1, cp1g);
    P2.xformPoint(cp2, cp2g);
    q1[0] = cp1g.x; q1[1] = cp1g.y; q1[2] = cp1g.z;
    q2[0] = cp2g.x; q2[1] = cp2g.y; q2[2] = cp2g.z;

    return len;
}


