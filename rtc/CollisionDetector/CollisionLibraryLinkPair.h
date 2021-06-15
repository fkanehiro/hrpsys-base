// -*- C++ -*-
/*!
 * @file  CollisionLibraryLinkPair.h
 * @brief CollisionLibraryLinkPair class definition
 * @date  $Date$
 *
 * $Id$
 */

#ifndef COLLISION_LIBRARY_LINK_PAIR_H
#define COLLISION_LIBRARY_LINK_PAIR_H

#include <hrpModel/Body.h>

class CollisionLibraryLinkPair  : public hrp::Referenced {
public:
    virtual bool checkCollision() {};
    virtual double computeDistance(double *q1, double *q2) {};
    hrp::Link* link(int index) { return links_[index]; }
    double getTolerance() { return tolerance_; }
    void setTolerance(double t) { tolerance_ = t; }

protected:
    hrp::Link *links_[2];
    double tolerance_;
};

#endif // COLLISION_DETECTOR_LINK_PAIR_H
