////////////////////////////////////////////////////////////////////////////////
//
//  Copyright 1997 Mitsubishi Electric Information Technology Center
//  America (MEITCA).  All Rights Reserved.
//
//  Permission to use, copy, modify and distribute this software and
//  its documentation for educational, research and non-profit
//  purposes, without fee, and without a written agreement is hereby
//  granted, provided that the above copyright notice and the
//  following three paragraphs appear in all copies.
//
//  Permission to incorporate this software into commercial products
//  may be obtained from MERL - A Mitsubishi Electric Research Lab, 201
//  Broadway, Cambridge, MA 02139.
//
//  IN NO EVENT SHALL MEITCA BE LIABLE TO ANY PARTY FOR DIRECT,
//  INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING
//  LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
//  DOCUMENTATION, EVEN IF MEITCA HAS BEEN ADVISED OF THE POSSIBILITY
//  OF SUCH DAMAGES.
//
//  MEITCA SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS ON
//  AN "AS IS" BASIS, AND MEITCA HAS NO OBLIGATIONS TO PROVIDE
//  MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
//
//  Author:
//    Brian Mirtich
//    mirtich@merl.com
//    617.621.7573
//    www.merl.com/people/mirtich
//
////////////////////////////////////////////////////////////////////////////////



#include <iostream>
#include <iomanip>
#include <fstream>
#include <limits.h>
#include <stdlib.h>
#include <vector>

#include "vclip.h"
using namespace Vclip;

///////////////////////////////////////////////////////////////////////////////
//  constants, data strucutres, macros, globals
///////////////////////////////////////////////////////////////////////////////


#define CONTINUE     0
#define DISJOINT     1
#define PENETRATION -1

#define V(v) ((Vertex *) (v))
#define E(e) ((Edge *)   (e))
#define F(f) ((Face *)   (f))


#define xformVertex(X, v, xg) \
if ((xg).feat != (v)) { \
  X.xformPoint(V(v)->coords_, (xg).coords); \
  (xg).feat = (v); \
}

#define xformEdge(X, e, xg) \
if ((xg).feat != (e)) { \
  X.xformPoint(E(e)->tail->coords_, (xg).tail); \
  X.xformPoint(E(e)->head->coords_, (xg).head); \
  (xg).seg.sub((xg).head, (xg).tail); \
  (xg).feat = (e); \
}


///////////////////////////////////////////////////////////////////////////////
//  closest feature tests
///////////////////////////////////////////////////////////////////////////////


int Polyhedron::vertVertTest(const Feature *&v1, const Feature *&v2, 
			     XformedGeom &xv1, XformedGeom &xv2,
			     const VclipPose &X12, const VclipPose &X21,
			     Vect3 &cp1, Vect3 &cp2, Real &dist)
{
  const list<VertConeNode> *cone;
  list<VertConeNode>::const_iterator cni;
  Vect3 xcoords;

  // check if v2 lies in v1's cone
  xformVertex(X21, v2, xv2);
  cone = &V(v1)->cone;
  FOR_EACH(*cone, cni)
    if (cni->plane->dist(xv2.coords) < 0) {
      v1 = cni->nbr;
      return CONTINUE;
    }

  // check if v1 lies in v2's cone
  xformVertex(X12, v1, xv1);
  cone = &V(v2)->cone;
  FOR_EACH(*cone, cni)
    if (cni->plane->dist(xv1.coords) < 0) {
      v2 = cni->nbr;
      return CONTINUE;
    }

  cp1 = V(v1)->coords_;
  cp2 = V(v2)->coords_;
  dist = cp2.distance(xv1.coords);
  return (dist > 0) ? DISJOINT : PENETRATION;  // (dist could be 0)
}


int Polyhedron::vertFaceTest(const Feature *&v, const Feature *&f, 
			     XformedGeom &xv, 
			     const VclipPose &Xvf, const list<Face> &allFaces,
			     Vect3 &cpv, Vect3 &cpf, Real &dist)
{
  int update;
  const Edge *e;
  const list<VertConeNode> *vcone;
  const list<FaceConeNode> *fcone;
  list<VertConeNode>::const_iterator vcni;
  list<FaceConeNode>::const_iterator fcni;
  list<Face>::const_iterator facei;
  Real d, d2, dmin;
  Vect3 xother;


  // check if v lies in f's cone
  xformVertex(Xvf, v, xv);
  update = 0;
  dmin = 0.0;
  fcone = &F(f)->cone;
  FOR_EACH(*fcone, fcni) 
    if ((d = fcni->plane->dist(xv.coords)) < dmin) {
      f = fcni->nbr;
      dmin = d;
      update = 1;
    }
  if (update) return CONTINUE;


  // check that none of the edges of v point toward f
  if ((d = F(f)->plane.dist(xv.coords)) == 0) {
    cpv = V(v)->coords_;
    cpf = xv.coords;
    return PENETRATION;
  }
  vcone = &V(v)->cone;
  FOR_EACH(*vcone, vcni) {
    e = vcni->nbr;
    Xvf.xformPoint((e->tail==v) ? e->head->coords_ : e->tail->coords_, xother);
    d2 = F(f)->plane.dist(xother);
    if (d < 0 && d2 > d || d > 0 && d2 < d) {
      if (e->tail == v) {xv.tail = xv.coords; xv.head = xother;}
      else {xv.tail = xother; xv.head = xv.coords;}
      xv.seg.sub(xv.head, xv.tail);
      xv.feat = e;
      v = e; 
      return CONTINUE;
    }
  }
  if (d > 0) {
    dist = d;
    cpv = V(v)->coords_;
    cpf.displace(xv.coords, F(f)->plane.normal(), -d);
    return DISJOINT;
  }

  // v is in local min on back side of f's cone
  FOR_EACH(allFaces, facei)
    if ((d2 = facei->plane.dist(xv.coords)) > d) {
      d = d2;
      f = &*facei;
    }
  if (d > 0) return CONTINUE;
  dist = d;
  cpv = V(v)->coords_;
  cpf.displace(xv.coords, F(f)->plane.normal(), -d);
  return PENETRATION;
}


int Polyhedron::vertEdgeTest(const Feature *&v, const Feature *&e, 
			     XformedGeom &xv, XformedGeom &xe,
			     const VclipPose &Xve, const VclipPose &Xev,
			     Vect3 &cpv, Vect3 &cpe, Real &dist)
{
  const Feature *minNbr, *maxNbr;
  list<VertConeNode>::const_iterator cni;
  Real min, max, lambda, dt, dh;
  Vect3 offset;
  Vect3 h;

  // check if v lies within edge cone planes
  xformVertex(Xve, v, xv);
  if (E(e)->tplane.dist(xv.coords) > 0) {e = E(e)->tail; return CONTINUE;}
  if (E(e)->hplane.dist(xv.coords) > 0) {e = E(e)->head; return CONTINUE;}
  if (E(e)->lplane.dist(xv.coords) > 0) {e = E(e)->left; return CONTINUE;}
  if (E(e)->rplane.dist(xv.coords) > 0) {e = E(e)->right; return CONTINUE;}


  // clip e against v's cone
  xformEdge(Xev, e, xe);
  min = 0;
  max = 1;
  minNbr = maxNbr = NULL;
  FOR_EACH(V(v)->cone, cni) {

    dt = cni->plane->dist(xe.tail);
    dh = cni->plane->dist(xe.head);

    if (dt >= 0) {
      if (dh >= 0) continue;
      if ((lambda = dt / (dt - dh)) < max)
	{max = lambda; maxNbr = cni->nbr; if (max < min) break;}
    }
    else { // dt < 0
      if (dh < 0) {minNbr = maxNbr = cni->nbr; break;}
      if ((lambda = dt / (dt - dh)) > min)
	{min = lambda; minNbr = cni->nbr; if (min > max) break;}
    }
  }

  if (cni != V(v)->cone.end() && minNbr == maxNbr) {
    v = minNbr;
    return CONTINUE;
  }

  // analyze derivatives at boundaries
  if (minNbr || maxNbr) {
    if (minNbr) {
      offset.displace(xe.tail, xe.seg, min);
      offset.sub(V(v)->coords_);
      if (offset == Vect3::ZERO) {
	cpv = V(v)->coords_;
	cpe = xv.coords;
	return PENETRATION;
      }
      if (offset.dot(xe.seg) > 0) {v = minNbr; return CONTINUE;}
    }
    if (maxNbr) {
      offset.displace(xe.tail, xe.seg, max);
      offset.sub(V(v)->coords_);
      if (offset == Vect3::ZERO) {
	cpv = V(v)->coords_;
	cpe = xv.coords;
	return PENETRATION;
      }
      if (offset.dot(xe.seg) < 0) {v = maxNbr; return CONTINUE;}
    }
  }

  cpv = V(v)->coords_;
  h.sub(xv.coords, E(e)->tail->coords_);
  cpe.displace(E(e)->tail->coords_, E(e)->dir, h.dot(E(e)->dir));
  dist = cpe.distance(xv.coords);
  return DISJOINT;
}


int Polyhedron::edgeEdgeSubtest(const Feature *&e, XformedGeom &xe, Vect3 &cp)
{
  int i;
  const Feature *nbr, *minNbr, *maxNbr, *vminNbr, *vmaxNbr;
  const Plane *plane;
  Real dt, dh, lambda, min, max, vmin, vmax, dmin, dmax;
  Vect3 seg, point, minPoint, maxPoint;

  min = 0;
  max = 1;
  minNbr = maxNbr = NULL;

  // clip against tail vertex plane
  dt = - E(e)->tplane.dist(xe.tail);
  dh = - E(e)->tplane.dist(xe.head);
  if (dt < 0) {
    if (dh < 0) {e = E(e)->tail; return CONTINUE;}
    min = dt / (dt - dh);
    minNbr = E(e)->tail;
  }
  else if (dh < 0) {
    max = dt / (dt - dh);
    maxNbr = E(e)->tail;
  }

  // clip against head vertex plane
  dt = - E(e)->hplane.dist(xe.tail);
  dh = - E(e)->hplane.dist(xe.head);
  if (dt < 0) {
    if (dh < 0) {e = E(e)->head; return CONTINUE;}
    min = dt / (dt - dh);
    minNbr = E(e)->head;
  }
  else if (dh < 0) {
    max = dt / (dt - dh);
    maxNbr = E(e)->head;
  }

  if (vminNbr = minNbr) vmin = min;
  if (vmaxNbr = maxNbr) vmax = max;

  // clip against left & right face planes
  for (i = 0; i < 2; i++) {
    if (i) {plane = &E(e)->rplane; nbr = E(e)->right;}
    else   {plane = &E(e)->lplane; nbr = E(e)->left;}
    dt = - plane->dist(xe.tail);
    dh = - plane->dist(xe.head);
    if (dt < 0) {
      if (dh < 0) {
	// completely clipped by a face plane - must check vertex derivs
	if (vminNbr) {
	  point.displace(xe.tail, xe.seg, vmin);
	  point.sub(V(vminNbr)->coords_);
	  if (point == Vect3::ZERO) {cp= V(minNbr)->coords_; return PENETRATION;}
	  if (point.dot(xe.seg) > 0) {e = vminNbr; return CONTINUE;}
	}
	if (vmaxNbr) {
	  point.displace(xe.tail, xe.seg, vmax);
	  point.sub(V(vmaxNbr)->coords_);
	  if (point == Vect3::ZERO) {cp= V(maxNbr)->coords_; return PENETRATION;}
	  if (point.dot(xe.seg) < 0) {e = vmaxNbr; return CONTINUE;}
	}
	e = nbr;
	return CONTINUE;
      }
      else if ((lambda = dt / (dt - dh)) > min)
	{min = lambda; minNbr = nbr; if (min > max) break;}
    }
    else if (dh < 0)
      if ((lambda = dt / (dt - dh)) < max)
	{max = lambda; maxNbr = nbr; if (max < min) break;}
  }

  if (i < 2) {
    // edge lies outside the voronoi region
    if (minNbr->type() == Feature::VERTEX) {
      point.displace(xe.tail, xe.seg, min);
      point.sub(V(minNbr)->coords_);
      if (point == Vect3::ZERO) {cp = V(minNbr)->coords_; return PENETRATION;}
      e = (point.dot(xe.seg) >= 0) ? minNbr : maxNbr;
      return CONTINUE;
    }
    if (maxNbr->type() == Feature::VERTEX) {
      point.displace(xe.tail, xe.seg, max);
      point.sub(V(maxNbr)->coords_);
      if (point == Vect3::ZERO) {cp = V(maxNbr)->coords_; return PENETRATION;}
      e = (point.dot(xe.seg) <= 0) ? maxNbr : minNbr;
      return CONTINUE;
    }
    // complete clipping by combination of both face planes
    dt = F(minNbr)->plane.dist(xe.tail);
    dh = F(minNbr)->plane.dist(xe.head);
    dmin = dt + min * (dh - dt);
    if (dmin == 0) {cp.displace(xe.tail, xe.seg, min); return PENETRATION;}
    //e = ((dmin > 0 && dt < dh) || (dmin < 0 && dt > dh)) ? minNbr : maxNbr;
    e = (dmin > 0) ? ((dt < dh) ? minNbr : maxNbr)
                   : ((dt > dh) ? minNbr : maxNbr);


    return CONTINUE;
  }

  // edge intersects V-region; analyze derivs
  
  if (minNbr) {
    if (minNbr->type() == Feature::FACE) {
      dt = F(minNbr)->plane.dist(xe.tail);
      dh = F(minNbr)->plane.dist(xe.head);
      dmin =             dt + min * (dh - dt);
      dmax = (maxNbr) ? (dt + max * (dh - dt)) : dh;
      if (dmin == 0) {cp.displace(xe.tail, xe.seg, min); return PENETRATION;}
      if ((dmin > 0 && dmin < dmax) || (dmin < 0 && dmin > dmax)) 
	{e = minNbr; return CONTINUE;}
    }
    else {
      point.displace(xe.tail, xe.seg, min);
      point.sub(V(minNbr)->coords_);
      if (point == Vect3::ZERO) {cp = V(minNbr)->coords_; return PENETRATION;}
      if (point.dot(xe.seg) > 0) {e = minNbr; return CONTINUE;}
    }
  }

  if (maxNbr) {
    if (maxNbr->type() == Feature::FACE) {
      dt = F(maxNbr)->plane.dist(xe.tail);
      dh = F(maxNbr)->plane.dist(xe.head);
      dmin = (minNbr) ? (dt + min * (dh - dt)) : dt;
      dmax =             dt + max * (dh - dt);
      if (dmax == 0) {cp.displace(xe.tail, xe.seg, max); return PENETRATION;}
      if ((dmax > 0 && dmax < dmin) || (dmax < 0 && dmax > dmin)) 
	{e = maxNbr; return CONTINUE;}
    }
    else {
      point.displace(xe.tail, xe.seg, max);
      point.sub(V(maxNbr)->coords_);
      if (point == Vect3::ZERO) {cp = V(maxNbr)->coords_; return PENETRATION;}
      if (point.dot(xe.seg) < 0) {e = maxNbr; return CONTINUE;}
    }
  }

  return DISJOINT;
}


int Polyhedron::edgeEdgeTest(const Feature *&e1, const Feature *&e2, 
			     XformedGeom &xe1, XformedGeom &xe2,
			     const VclipPose &X12, const VclipPose &X21,
			     Vect3 &cp1, Vect3 &cp2, Real &dist)
{
  int res;
  Real k, lambda, num, denom;
  Vect3 xdir, h, h2, xcoords;

  // clip e1 against e2's cone
  xformEdge(X12, e1, xe1);
  if ((res = edgeEdgeSubtest(e2, xe1, cp2)) == PENETRATION)
    X21.xformPoint(cp2, cp1);
  if (res != DISJOINT) return res;
  // clip e2 against e1's cone
  xformEdge(X21, e2, xe2);
  if ((res = edgeEdgeSubtest(e1, xe2, cp1)) == PENETRATION)
    X12.xformPoint(cp1, cp2);
  if (res != DISJOINT) return res;

  // disjoint - compute closest points & distance
  
  X21.xformVect(E(e2)->dir, xdir);
  k = xdir.dot(E(e1)->dir);
  h.sub(xe2.tail, E(e1)->tail->coords_);
  h2.displace(E(e1)->dir, xdir, -k);
  num = h.dot(h2);
  denom = 1 - k * k;
  if (denom == 0.0) {
    if (num > 0) cp1 = E(e1)->head->coords_;
    else cp1 = E(e1)->tail->coords_;
  }
  else {
    lambda = num / denom;
    if (lambda < 0) lambda = 0;
    else if (lambda > E(e1)->len) lambda = E(e1)->len;
    cp1.displace(E(e1)->tail->coords_, E(e1)->dir, lambda);
  }
  // now compute cp2
  X12.xformPoint(cp1, xcoords);
  h.sub(xcoords, E(e2)->tail->coords_);
  lambda = h.dot(E(e2)->dir);
  cp2.displace(E(e2)->tail->coords_, E(e2)->dir, lambda);

  dist = cp2.distance(xcoords);
  return DISJOINT;
}


int Polyhedron::edgeFaceTest(const Feature *&e, const Feature *&f, 
			     XformedGeom &xe, const VclipPose &Xef, 
			     Vect3 &cpe, Vect3 &cpf, Real &dist)
{
  enum Code {INSIDE, OUTSIDE, MIN, MAX};

  int i, intersect;
  list<FaceConeNode>::const_iterator cni;
  const FaceConeNode *cn, *prev, *next, *maxCn, *minCn, *chopCn;
  const Edge *s;
  const Vertex *minv, *maxv;
  Real lambda, min, max, dt, dh, dmin, dmax;
  Vect3 point;
  vector<int>::iterator c;
  vector<Real>::iterator l;
  static vector<int> code(MAX_VERTS_PER_FACE); //(template can't use local type)
  static vector<Real> lam(MAX_VERTS_PER_FACE);

  if (F(f)->sides > (int)code.capacity()) {
    code.reserve(F(f)->sides);
    lam.reserve(F(f)->sides);
  }

  xformEdge(Xef, e, xe);

  min = 0;
  max = 1;
  minCn = maxCn = chopCn = NULL;
  for (cni = F(f)->cone.begin(), l = lam.begin(), c = code.begin(); 
       cni != F(f)->cone.end(); ++cni, ++l, ++c) {
    dt = cni->plane->dist(xe.tail);
    dh = cni->plane->dist(xe.head);
    if (dt >= 0)
      if (dh >= 0) *c = INSIDE;
      else { // dh < 0
	*c = MAX;
	if ((*l = dt / (dt - dh)) < max) {max = *l; maxCn = &*cni;}
      }
    else  // dt < 0
      if (dh >= 0) {
	*c = MIN;
	if ((*l = dt / (dt - dh)) > min) {min = *l; minCn = &*cni;}
      }
      else { // dh < 0
	*c = OUTSIDE;
	chopCn = &*cni;
      }
  }

  if (chopCn || min > max) {

    if (chopCn) cn = chopCn;
    // heuristic:  choose minCn or maxCn, based on which
    // corresponding region contains more of edge being clipped.
    else cn = (min + max > 1.0) ? minCn : maxCn;
    
    prev = NULL; next = cn;
    intersect = 0;
    while (next != prev) {
      prev = cn;
      cn = next;
      s = cn->nbr;
      minv = maxv = NULL;

      // test edge plane
      i = cn->idx;
      if (code[i] == INSIDE) break;
      else if (code[i] == OUTSIDE) {min = 0; max = 1;}
      else if (code[i] == MIN) {min = 0; max = lam[i];}
      else if (code[i] == MAX) {min = lam[i]; max = 1;}

      // test tail plane
      dt = - s->tplane.dist(xe.tail);
      dh = - s->tplane.dist(xe.head);
      if (dt >= 0) {
	if (dh < 0)
	  if ((lambda = dt / (dt - dh)) < max) {
	    max = lambda; maxv = s->tail; 
	    if (min > max) {
	      if (intersect) break; 
	      next = (s->left == f) ? cn->cw : cn->ccw; 
	      continue;
	    }
	  }
      }
      else { // dt < 0
	if (dh < 0) {next = (s->left == f) ? cn->cw  : cn->ccw; continue;}
	if ((lambda = dt / (dt - dh)) > min) {
	  min = lambda; minv = s->tail; 
	  if (min > max) {
	    if (intersect) break; 
	    next = (s->left == f) ? cn->cw : cn->ccw; 
	    continue;
	  }
	}
      }

      // test head plane
      dt = - s->hplane.dist(xe.tail);
      dh = - s->hplane.dist(xe.head);
      if (dt >= 0) {
	if (dh < 0)
	  if ((lambda = dt / (dt - dh)) < max) {
	    max = lambda; maxv = s->head; 
	    if (min > max) {
	      if (intersect) break; 
	      next = (s->left == f) ? cn->ccw : cn->cw; 
	      continue;
	    }
	  }
      }
      else { // dt < 0
	if (dh < 0) {next = (s->left == f) ? cn->ccw : cn->cw;  continue;}
	if ((lambda = dt / (dt - dh)) > min) {
	  min = lambda; minv = s->head;
	  if (min > max) {
	    if (intersect) break; 
	    next = (s->left == f) ? cn->ccw : cn->cw; 
	    continue;
	  }
	}
      }

      intersect = 1; // we've found an edge Voronoi region that's intersected

      if (minv) {
	point.displace(xe.tail, xe.seg, min);
	point.sub(minv->coords_);
	if (point.dot(xe.seg) > 0) {
	  next = (s->left == f) ? ((s->tail == minv) ? cn->cw  : cn->ccw) :
	                          ((s->tail == minv) ? cn->ccw : cn->cw);
	  continue;
	}
      }

      if (maxv) {
	point.displace(xe.tail, xe.seg, max);
	point.sub(maxv->coords_);
	if (point.dot(xe.seg) < 0) {
	  next = (s->left == f) ? ((s->head == maxv) ? cn->ccw : cn->cw) :
	                          ((s->head == maxv) ? cn->cw  : cn->ccw);
	  continue;
	}
      }

      f = s;
      return CONTINUE;
    }

    f = (cn->ccw == prev) ? ((s->left == f) ? s->head : s->tail) :
                             ((s->left == f) ? s->tail : s->head) ;
    return CONTINUE;
  }

  // edge intersects faces cone - check derivatives

  dt = F(f)->plane.dist(xe.tail);
  dh = F(f)->plane.dist(xe.head);
  dmin = (minCn) ? (dt + min * (dh - dt)) : dt;
  dmax = (maxCn) ? (dt + max * (dh - dt)) : dh;
  if (dmin <= 0) {
    if (dmax >= 0) {
      dist = dmin; 
      cpe.displace(E(e)->tail->coords_, E(e)->dir, min * E(e)->len);
      cpf.displace(xe.tail, xe.seg, min);
      cpf.displace(F(f)->plane.normal(), -dmin);
      return PENETRATION;
    }
  }
  else if (dmax <= 0) {
    dist = dmax; 
    cpe.displace(E(e)->tail->coords_, E(e)->dir, max * E(e)->len);
    cpf.displace(xe.tail, xe.seg, max);
    cpf.displace(F(f)->plane.normal(), -dmax);
    return PENETRATION;
  }

  // at this point, dmin & dmax are both +ve or both -ve
  if (dmin > 0 && dt <= dh || dmin < 0 && dt >= dh)
    if (minCn) f = minCn->nbr; 
    else {
      xe.coords = xe.tail;
      xe.feat = e = E(e)->tail;
    }
  else
    if (maxCn) f = maxCn->nbr; 
    else {
      xe.coords = xe.head;
      xe.feat = e = E(e)->head;
    }
  
  return CONTINUE;
}



///////////////////////////////////////////////////////////////////////////////
//  vclip for Polyhedra
///////////////////////////////////////////////////////////////////////////////


#define MAX_ITERS 5000

const char *ptree1name="", *ptree2name="";

Real Polyhedron::vclip(const Polyhedron *const poly1, 
		       const Polyhedron *const poly2, 
		       const VclipPose &X12, const VclipPose &X21, 
		       const Feature *&feat1, const Feature *&feat2,
		       Vect3 &cp1, Vect3 &cp2, int oneStep)
{
  int result;
  Real dist;
  XformedGeom xg1, xg2;

  int loop = 0;

  xg1.feat = xg2.feat = NULL;  // initialized xformed geometries

  dist = 0.0;
  do {

    switch ((feat1->type() << 2) + feat2->type()) {

    case (Feature::VERTEX<<2)+Feature::VERTEX: 
      result = vertVertTest(feat1,feat2,xg1,xg2,X12,X21,cp1,cp2,dist); break;

    case (Feature::VERTEX<<2)+Feature::EDGE:   
      result = vertEdgeTest(feat1,feat2,xg1,xg2,X12,X21,cp1,cp2,dist); break;

    case (Feature::EDGE  <<2)+Feature::VERTEX: 
      result = vertEdgeTest(feat2,feat1,xg2,xg1,X21,X12,cp2,cp1,dist); break;

    case (Feature::VERTEX<<2)+Feature::FACE:   
      result = vertFaceTest(feat1,feat2,xg1,X12,poly2->faces_,cp1,cp2,dist); 
      break;

    case (Feature::FACE  <<2)+Feature::VERTEX: 
      result = vertFaceTest(feat2,feat1,xg2,X21,poly1->faces_,cp2,cp1,dist); 
      break;

    case (Feature::EDGE  <<2)+Feature::EDGE:   
      result = edgeEdgeTest(feat1,feat2,xg1,xg2,X12,X21,cp1,cp2,dist); break;

    case (Feature::EDGE  <<2)+Feature::FACE: 
      result = edgeFaceTest(feat1,feat2,xg1,X12,cp1,cp2,dist); break;

    case (Feature::FACE  <<2)+Feature::EDGE:   
      result = edgeFaceTest(feat2,feat1,xg2,X21,cp2,cp1,dist); break;

    default: 
      cerr << "\ninvalid feature pair combination in vclip" << endl;
      exit(1);
    }

    if (loop++ == MAX_ITERS) break;

  } while (result == CONTINUE && !oneStep);

  // uh oh...
  if (loop > MAX_ITERS) {
    int i;
    ofstream ofs("vclipCrash", ios::app);
    ofs << "(" << ptree1name << "," << ptree2name << ")" << endl;
    ofs << feat1->name() << '\n' << feat2->name() << '\n' << '*';
    //for (i = 0; i < sizeof(VclipPose); i++) ofs << ((char *) &X12)[i];
    //for (i = 0; i < sizeof(VclipPose); i++) ofs << ((char *) &X21)[i];
    ofs << '\n' << "X12\n" << X12 << '\n' << "X21\n" << X21 << endl;
    ofs << " ***** " << endl;
    ofs.close();
    cerr << "vclip cycle detected! \a"  << endl;
  }

  return dist;
}



///////////////////////////////////////////////////////////////////////////////
//  vclip for PolyTrees
///////////////////////////////////////////////////////////////////////////////


Real PolyTree::vclip_(const PolyTree *const ptree1, 
		      const PolyTree *const ptree2,
		      const VclipPose &Xr1r2, const VclipPose &Xr2r1,
		      ClosestFeaturesHT &ht,
		      Vect3 &cp1, Vect3 &cp2)
{
  PolyTreePair ptrees;
  list< Handle<PolyTree> >::const_iterator compi;
  Real dist, compDist;
  Vect3 compCp1, compCp2;
  VclipPose Xp1p2, Xp2p1;

  // find closest features
  ptrees.first  = ptree1;
  ptrees.second = ptree2;
  FeaturePair &features = ht[ptrees];
  if (!features.first) {
    // initialization:  set feature to first vertices on each PolyTree
    features.first  = &ptree1->poly_->verts_.front();
    features.second = &ptree2->poly_->verts_.front();
  }

  // call atomic algorithm on current pair
#if VCLIP_MATRIX_POSE
#define POLY_TO_REF Xpr_
#define REF_TO_POLY Xrp_
#else
#define POLY_TO_REF Tpr_
#define REF_TO_POLY Trp_
#endif
  Xp1p2.mult(Xr1r2, ptree1-> POLY_TO_REF);
  Xp1p2.premult(ptree2-> REF_TO_POLY);
  Xp2p1.invert(Xp1p2);
  ptree1name = ptree1->name; ptree2name = ptree2->name;
  dist = Polyhedron::vclip(&*ptree1->poly_, &*ptree2->poly_, Xp1p2, Xp2p1, 
			   features.first, features.second, cp1, cp2);
  ptree1-> POLY_TO_REF .xformPoint(cp1);  // xform cp's to reference frames
  ptree2-> POLY_TO_REF .xformPoint(cp2);
#undef POLY_TO_REF
#undef REF_TO_POLY

  if (dist > 0) return dist;  // disjoint
  
  if (ptree1->components.empty() && ptree2->components.empty())  
    return dist;  // penetration

  dist = HUGE_VAL;

  if (!ptree1->components.empty())
    FOR_EACH(ptree1->components, compi) {
      compDist = vclip_(&**compi, ptree2, Xr1r2, Xr2r1, ht, compCp1, compCp2);
      if (compDist <= 0) return compDist;
      if (compDist < dist) {
	dist = compDist;
	cp1 = compCp1;
	cp2 = compCp2;
      }
    }
  else
    FOR_EACH(ptree2->components, compi) {
      compDist = vclip_(ptree1, &**compi, Xr1r2, Xr2r1, ht, compCp1, compCp2);
      if (compDist <= 0) return compDist;
      if (compDist < dist) {
	dist = compDist;
	cp1 = compCp1;
	cp2 = compCp2;
      }
    }

  return dist;
}


void PolyTree::vclipFeatures(const PolyTree *const ptree1, 
			     const PolyTree *const ptree2,
			     ClosestFeaturesHT &ht,
			     const Feature *&feat1, const Feature *&feat2)
{
  PolyTreePair ptrees;
  ptrees.first  = ptree1;
  ptrees.second = ptree2;
  FeaturePair &features = ht[ptrees];
  feat1 = features.first;
  feat2 = features.second;
}

