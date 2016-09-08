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



#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <ctype.h>
#include <string.h>
#include <vector>

#include "vclip.h"

using namespace std;
using namespace Vclip;

#if INVENTOR
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>
#include <Inventor/nodes/SoShapeHints.h>
#endif

#if QHULL
extern "C" {
#include "qhull/qhull_a.h"
}
//char qh_version[] = "vclip 1.0";
#endif

///////////////////////////////////////////////////////////////////////////////
//  constants
///////////////////////////////////////////////////////////////////////////////

#define LONG_STR_SZ 1000  // for reading lines from files, etc.


///////////////////////////////////////////////////////////////////////////////
//  globals
///////////////////////////////////////////////////////////////////////////////

// Used to hold Edge names, which are not stored explicitly but
// derived from stored Vertex names.  If application calls
// Edge::name() twice, first result will be overwritten by second.
// Copy first result to another character array if both are needed
// simultaneously!
char edgeName[2 * VF_NAME_SZ];  // space for 2 Vertex names, :, and \0.


///////////////////////////////////////////////////////////////////////////////
//  low level Polyhedron support
///////////////////////////////////////////////////////////////////////////////

// Output a Plane
ostream& Plane::print(ostream &os) const
{
  int oldFlags;

  oldFlags = os.setf(ios::showpos);
  os << normal_.x << " x " << normal_.y << " y  "
     << normal_.z << " z " << offset_ << " >= 0";
  os.flags((std::_Ios_Fmtflags)oldFlags);
  return os;
}


const char *Vertex::name() const 
{
  return name_;
}


const char *Face::name() const 
{
  return name_;
}


const char *Edge::name() const 
{
  sprintf(edgeName, "%s:%s", tail->name(), head->name());
  return edgeName;
}


ostream& VertConeNode::print(ostream &os) const 
{
  return os << setw(16) << (nbr ? nbr->name() : "?") << "]  " << *plane;
}


ostream& FaceConeNode::print(ostream &os) const 
{
  return os << setw(16) << (nbr ? nbr->name() : "?") << "]  " << *plane;
}



///////////////////////////////////////////////////////////////////////////////
//  class Polyhedron
///////////////////////////////////////////////////////////////////////////////


// This is called when the vertex sequence (tail, head) is encountered
// in Face f's boundary walk.  Either a new edge is created and added
// to the edgeList, or the previously existing one from head to tail
// is updated.

void Polyhedron::processEdge(Face *f, Vertex *tail, Vertex *head)
{
  list<VertConeNode>::iterator vci;
  list<FaceConeNode>::iterator fci;;
  VertConeNode vcn;
  FaceConeNode fcn;

  Edge *e, e0;
  Vect3 v;

  // check if the reverse edge (from head to tail) already exists
  FOR_EACH(head->cone, vci) {
    if ((e = vci->nbr)->head == tail) {
      // set pointer to right, rplane
      e->right = f;
      v.cross(e->dir, f->plane.normal());
      v.normalize();
      e->rplane.set(v, head->coords_);
      // tell right about e
      fcn.nbr = e;
      fcn.plane = &e->rplane;
      f->cone.push_back(fcn);
      return;
    }
  }

  // set direction, length
  e0.dir.sub(head->coords_, tail->coords_);
  e0.len = e0.dir.norm();
  e0.dir.normalize();
  // set pointers to tail, head, left
  e0.tail = tail;
  e0.head = head;
  e0.left = f;
  // set tplane, hplane, and lplane
  v.negate(e0.dir);
  e0.tplane.set(v, tail->coords_);
  e0.hplane.set(e0.dir, head->coords_);
  v.cross(f->plane.normal(), e0.dir);
  v.normalize();
  e0.lplane.set(v, tail->coords_);
  // link into list of edges
  edges_.push_back(e0);
  e = &edges_.back();
  // tell tail about e
  vcn.nbr = e;
  vcn.plane = &e->tplane;
  tail->cone.push_back(vcn);
  // tell head about e
  vcn.plane = &e->hplane;
  head->cone.push_back(vcn);
  // tell left about e
  fcn.nbr = e;
  fcn.plane = &e->lplane;
  f->cone.push_back(fcn);
}


// No checking done, but numVerts better be at least 3!
void Polyhedron::addFace(const char *name, 
			 vector<Vertex *> &verts, int clockwise)
{
  int i;
  Face f0, *f;
  vector<Vertex *>::iterator vi;
  list<FaceConeNode>::iterator cni;
  FaceConeNode *last;
  Vect3 u, v, normal;

  f0.sides = verts.size();
  strcpy(f0.name_, name);
  // compute face support plane
  u.sub(verts[1]->coords_, verts[0]->coords_);
  v.sub(verts[2]->coords_, verts[1]->coords_);
  normal.cross(u, v);
  normal.normalize();
  if (clockwise) normal.negate();
  f0.plane.set(normal, verts[0]->coords_);
  // add to list of faces
  faces_.push_back(f0);
  f = &faces_.back();

  // build edges around face
  if (clockwise) {
    for (vi = verts.end()-1; vi != verts.begin(); --vi)
      processEdge(f, *vi, *(vi-1));
    processEdge(f, verts.front(), verts.back());  // close the loop
  }
  else {
    for (vi = verts.begin(); vi != verts.end()-1; ++vi)
      processEdge(f, *vi, *(vi+1));
    processEdge(f, verts.back(), verts.front());  // close the loop
  }

  // compute ccw and cw links around FaceConeNodes, cn indices, and f->sides
  for (cni = f->cone.begin(), last = &f->cone.back(), i = 0;
       cni != f->cone.end(); last = &*cni, ++cni) {
    cni->cw = last;
    last->ccw = &*cni;
    cni->idx = i++;
  }
}  


ostream& Polyhedron::print(ostream &os) const
{
  const Vertex *v;
  const Edge *e;
  const Face *f;
  list<VertConeNode>::const_iterator vcni;
  list<FaceConeNode>::const_iterator fcni;
  list<Vertex>::const_iterator vi;
  list<Edge  >::const_iterator ei;
  list<Face  >::const_iterator fi;
  //typename faces_.const_iterator fi;

  os << verts_.size() << " verts :  ";
  FOR_EACH(verts_, vi) os << vi->name() << " ";
  os << endl;

  os << edges_.size() << " edges :  ";
  FOR_EACH(edges_, ei) os << ei->name() << " ";
  os << endl;

  os << faces_.size() << " faces :  ";
  FOR_EACH(faces_, fi) os << fi->name() << " ";
  os << endl << endl;

  FOR_EACH(verts_, vi) {
    v = &*vi;
    os << "vertex " << v->name() << " "  << v->coords_ << "   "
       << v->cone.size() << " neighbors" << endl;
    FOR_EACH(v->cone, vcni) os << *vcni << endl;
    os << endl;
  }

  FOR_EACH(edges_, ei) {
    e = &*ei;
    os << "edge " << e->name() << " " <<  e->dir << endl;
    os << "tail: " << setw(16) << e->tail->name() << "]  " 
       << e->tplane << endl;
    os << "head: " << setw(16) << e->head->name() << "]  " 
       << e->hplane << endl;
    os << "left: " << setw(16) << e->left->name() << "]  " 
       << e->lplane << endl;
    os << "rght: " << setw(16) << e->right->name() << "]  " 
       << e->rplane << endl;
    os << endl;
  }

  FOR_EACH(faces_, fi) {
    f = &*fi;
    os << "face " << f->name() << " " << f->plane << "   "
       << f->sides << " neighbors" << endl;
    FOR_EACH(f->cone, fcni) os << *fcni << endl;
    os << endl;
  }

  return os;
}

#if INVENTOR
SoShapeKit *Polyhedron::buildInvModel() const
{
  int i, vertCounter;
  int numVerts, numEdges, numFaces;
  const Vertex *v;
  const Edge *e;
  list<Vertex>::const_iterator vi;
  list<Face  >::const_iterator fi;
  list<FaceConeNode>::const_iterator cni;
  SbVec3f sbv;
  // map:  Vertex *  -->  int, (we cast Vertex * to int for default hasher)
  //static hash_map<int, int> vertIndices(1000);
  static map<int, int> vertIndices(1000);  
  
  numVerts = verts_.size();
  numEdges = edges_.size();
  numFaces = faces_.size();

  SoShapeKit *kit = new SoShapeKit;

  SoShapeHints *shapeHints= (SoShapeHints *) kit->getPart("shapeHints", TRUE);
  shapeHints->vertexOrdering.setValue(SoShapeHints::COUNTERCLOCKWISE);
  shapeHints->shapeType.setValue(SoShapeHints::SOLID);
  shapeHints->faceType.setValue(SoShapeHints::CONVEX);
  // set 20 degree crease angle, see Inventor manual, p. 132
  shapeHints->creaseAngle.setValue(20.0 * M_PI/180); 

  SoCoordinate3 *vertCoords = 
    (SoCoordinate3 *) kit->getPart("coordinate3", TRUE);
  vertCoords->point.setNum(numVerts);

  vertIndices.erase(vertIndices.begin(), vertIndices.end());
  vertCounter = 0;
  FOR_EACH(verts_, vi) {
    vi->coords_.toSbVec3f(sbv);
    vertCoords->point.set1Value(vertCounter, sbv);
    vertIndices.insert(pair<const int, int>((int) &*vi, vertCounter++));
  }

  SoIndexedFaceSet *indexedFaces = new SoIndexedFaceSet;
  indexedFaces->coordIndex.setNum(2 * numEdges + numFaces);
  i = 0;
  FOR_EACH(faces_, fi) {
    FOR_EACH(fi->cone, cni) {
      e = cni->nbr;
      v = (e->left == &*fi) ? e->tail : e->head;
      indexedFaces->coordIndex.
	set1Value(i++, vertIndices.find((int) v)->second);
    }
    indexedFaces->coordIndex.set1Value(i++, -1);
  }

  kit->setPart("shape", indexedFaces);

  return kit;
}
#endif

///////////////////////////////////////////////////////////////////////////////
//  class PolyTree
///////////////////////////////////////////////////////////////////////////////


// constructor
PolyTree::PolyTree()
{
  //poly_.set(NULL);
  Tpr_ = Trp_ = Se3::ID; 
  Xpr_ = Xrp_ = MatX::ID;
  vol_ = 0.0;
  mov1_ = mov2_ = pov_ = Vect3::ZERO;
}

// copy constructor
PolyTree::PolyTree(const PolyTree &orig)
{
  list< Handle<PolyTree> >::const_iterator compi;

  poly_ = orig.poly_;
  vol_ = orig.vol_;
  mov1_ = orig.mov1_;
  mov2_ = orig.mov2_;
  pov_ = orig.pov_;
  rad_ = orig.rad_;
  Tpr_ = orig.Tpr_;
  Trp_ = orig.Trp_;
  Xpr_ = orig.Xpr_;
  Xrp_ = orig.Xrp_;
  strcpy(name, orig.name);

  // copy tree recursively
  Handle<PolyTree> h;
  FOR_EACH(orig.components, compi) {
    h.set(new PolyTree(**compi));
    components.push_back(h);
  }
  h.set(NULL);
}    
  
    
// Premultiply the Tpr_ field of a PolyTree by T, and recompute Trp_,
// Xpr_, and Xrp_ based on the new value.  Then repeat, recursively
// descending the PolyTree.  This routine sacrifices efficiency for
// accuracy, since these calculations should be performed only once.
void PolyTree::xform(const Se3 &T)
{
  list< Handle<PolyTree> >::iterator compi;

  Tpr_.premult(T);
  Tpr_.rot().normalize();
  Trp_.invert(Tpr_);
  Trp_.rot().normalize();
  Xpr_.set(Tpr_);
  Xrp_.set(Trp_);

  FOR_EACH(components, compi) (*compi)->xform(T);
}



void PolyTree::printRecur(ostream &os, int level) const
{
  list< Handle<PolyTree> >::const_iterator compi;

  os << setw(level * 4) << " " << Tpr_ << "  " << name << endl;

  FOR_EACH(components, compi) (*compi)->printRecur(os, level+1);
}


// Compute total number of nodes of the PolyTree rooted at this
int PolyTree::numNodes() const
{
  int i;
  list< Handle<PolyTree> >::const_iterator compi;

  if (components.empty()) return 1;
  else {
    i = 1;
    FOR_EACH(components, compi) i += (*compi)->numNodes();
    return i;
  }
}
    

// Compute total number of leaves of the PolyTree rooted at this
int PolyTree::numLeaves() const
{
  int i;
  list< Handle<PolyTree> >::const_iterator compi;

  if (components.empty()) return 1;
  else {
    i = 0;
    FOR_EACH(components, compi) i += (*compi)->numLeaves();
    return i;
  }
}


ostream& PolyTree::print(ostream &os) const
{
  os << "polytree " << name << endl;
  if (!components.empty())
    os << "compound:  " << components.size() << " children, " << numLeaves() 
       << " leaves, " << numNodes() << " total nodes" << endl;
  else os << "atomic" << endl;
  os << "volume            " << vol_  << endl;
  os << "1st moment of vol " << mov1_ << endl;
  os << "2nd moment of vol " << mov2_ << endl;
  os << "product of vol    " << pov_  << endl;
  os << "radius            " << rad_  << endl;

  if (!components.empty()) {
    printRecur(os, 0);
    os << endl;
  }
  else {
    os << &*poly_ << flush;
  }
  return os;
}

    

#if INVENTOR
// returns either an SoShapeKit (for atomic Poly) 
// or an SoSeparator (for a nontrivial tree)

SoNode *PolyTree::buildInvModel() const
{
  SoNode *node;
  SoTransform *xform;
  SoSeparator *root;
  list< Handle<PolyTree> >::const_iterator compi;
  Se3 T;

  if (components.empty()) return poly_->buildInvModel();    // atomic poly

  root = new SoSeparator;

  FOR_EACH(components, compi) {
    xform = new SoTransform;
    T.mult(Trp_, (*compi)->Tpr_);  // T = transform from component to this
    T.toSoTransform(xform);
    node = (*compi)->buildInvModel();
    if (node->getTypeId() == SoShapeKit::getClassTypeId())
      ((SoShapeKit *) node)->setPart("transform", xform);
    else ((SoSeparator *) node)->insertChild(xform, 0);
    root->addChild(node);
  }
  
  return root;
}
#endif


// Check convexity of Polyhedron and verify Euler formula.  Return 1
// on problem.
int Polyhedron::check() const
{
  int error;
  int nv, ne, nf;
  const Edge *e, *e1, *e2;
  const Face *f;
  list<FaceConeNode>::const_iterator cni;
  list<Edge>::const_iterator ei;
  list<Face>::const_iterator fi;
  Real dp;
  Vect3 v;

  error = 0;

  // check to make sure all edges are convex
  FOR_EACH(edges_, ei) {
    e = &*ei;
    v.cross(e->lplane.normal(), e->rplane.normal());
    if ((dp = Vect3::dot(e->dir, v)) >= 0) {
      error = 1;
      cerr << "\anonconvex edge:  "
	   << " tail=" << e->tail->name_
	   << " head=" << e->head->name_
	   << " left=" << e->left->name_
	   << " rght=" << e->right->name_
	   << " angle=" << asin(-dp) << endl;
    }
  }

  // check to make sure all faces are convex polygons
  FOR_EACH(faces_, fi) {
    f = &*fi;
    FOR_EACH(f->cone, cni) {
      e1 = (Edge *) cni->nbr;
      e2 = (Edge *) cni->ccw->nbr;
      v.cross(e1->dir, e2->dir);
      if ((e1->tail == e2->tail || e1->head == e2->head)) v.negate();
      if ((dp = Vect3::dot(f->plane.normal(), v)) <= 0) {
	error = 1;
	cerr << "\anonconvex face:  " << f->name_
	     << "  vertex=" << (e1->left == f ? e1->head : e1->tail)->name_
	     << "  angle=" << asin(-dp) << endl;
      }
    }
  }


  // Check if Euler formula (#V - #E + #F - 2 = 0) is satisfied
  nv = verts_.size();
  ne = edges_.size();
  nf = faces_.size();
  if (nv - ne + nf - 2) {
    error = 1;
    cout << "\apolyhedral Euler formula failure: "
	 << "nv=" << nv << " ne=" << ne << " nf=" << nf << endl;
  }

  return error;
}

#if QHULL
// Given the Vertex list of a Polyhedron, compute the convex hull.
// Use this to determine and build the Edges and Faces lists.  If a
// Vertex does not lie on the convex hull, it will be removed
// from the Vertex list.
int Polyhedron::buildHull()
{
  int nf, nv, npts, idx, i;
  Vertex *v;
  //List<Vertex> origVerts;
  Vect3 xcoords;

  boolT ismalloc;
  int curlong, totlong, exitcode;
  facetT *facet;
  vertexT *vertex;
  vertexT **vertexp;
  setT *vertices;
  coordT *qhullData, *qhd;
  list<Vertex>::iterator vi, vi0;
  char options [200];
  char name[LONG_STR_SZ];

  static vector<Vertex *> facelist(MAX_VERTS_PER_FACE);
#define MAX_VERTS_PER_HULL 1000  // initial size; exceeding it breaks nothing
  static vector<coordT> array      (MAX_VERTS_PER_HULL * 3);
  static vector<Vertex *> hullVerts(MAX_VERTS_PER_HULL);
  static vector<int> vertUsed      (MAX_VERTS_PER_HULL);
#undef MAX_VERTS_PER_HULL

  //cout << "invoking qhull...   " << flush;

  npts = verts_.size();


  if (npts > (int)hullVerts.capacity()) {
    array.reserve(3 * npts);
    hullVerts.reserve(npts);
    vertUsed.reserve(npts);
  }
  qhullData = &array.front();
    


  // build array of vertex coordinates
  i = 0;
  qhd = qhullData;
  FOR_EACH(verts_, vi) {
    v = &*vi;
#if OPCOUNTS
    *qhd++ = v->coords_.x.toDouble();
    *qhd++ = v->coords_.y.toDouble();
    *qhd++ = v->coords_.z.toDouble();
#else	 
    *qhd++ = v->coords_.x;
    *qhd++ = v->coords_.y;
    *qhd++ = v->coords_.z;
#endif
    vertUsed[i] = 0;
    hullVerts[i] = v;
    i++;
  }

  ismalloc= False; 	// True if qh_freeqhull should 'free(qhullData)'
  qh_init_A (stdin, stdout, stderr, 0, NULL);
  exitcode= setjmp (qh errexit);
  if (exitcode) goto error;
  sprintf(options, "qhull Qx i s Tcv C-0");
  //sprintf(options, "qhull A0.9999 i s Tcv C-0");
  qh_initflags (options);
  qh_init_B (qhullData, npts, 3, ismalloc);
  qh_qhull();
  qh_check_output();

  // build hull Polyhedron

  nf = 0;
  FORALLfacets {
    sprintf(name, "f%d", nf++);
    vertices= qh_facet3vertex (facet);
    facelist.clear();
    FOREACHvertex_(vertices) {
      idx = qh_pointid(vertex->point);
      vertUsed[idx] = 1;
      facelist.push_back(hullVerts[idx]);
    }
    addFace(name, facelist, 1); // qhull generates a clockwise list
    qh_settempfree(&vertices);
  }

  // strip out unused vertices, count remaining ones
  for (vi = verts_.begin(), i = 0; vi != verts_.end(); i++)
    if (vertUsed[i]) vi++;
    else {
      vi0 = vi;
      vi++;
      verts_.erase(vi0);
    }
  nv = verts_.size();

  //cout << nf << " faces, " << nv << " vertices " << endl;

  qh NOerrexit= True;
  qh_freeqhull (!qh_ALL);
  qh_memfreeshort (&curlong, &totlong);

  return 0;

error:
  cerr << "error building convex hull of Polyhedron \a" << endl;
  cerr << "exitcode: " << exitcode << endl;

  qh NOerrexit= True;
  qh_freeqhull (!qh_ALL);
  qh_memfreeshort (&curlong, &totlong);

  return 1;

}
#else
// QHULL not available - crash and burn
int Polyhedron::buildHull()
{
  cerr << "fatal error:" << endl;
  cerr << "attempt to compute convex hull of vertex set\n "
       << "but no QHULL library linked" << endl;
  exit(1);
  return 1;
}
#endif



// Compute the convex hull of a PolyTree's children (non-recursive)
// using Qhull.  Set poly field to point to this new Polyhedron.  Return
// 0 if no problems, and 1 on error.

int PolyTree::buildHull()
{
  int nv;
  const PolyTree *comp;
  list< Handle<PolyTree> >::const_iterator compi;
  list<Vertex>::const_iterator cvi;
  list<Vertex>::iterator vi;
  Vect3 xcoords;

  setPoly(new Polyhedron);


  FOR_EACH(components, compi) {
    comp = &**compi;
    FOR_EACH(comp->poly_->verts_, cvi) {
      comp->Tpr_.xformPoint(cvi->coords_, xcoords);
      poly_->addVertex("", xcoords);
    }
  }
  int status = poly_->buildHull();

  // name vertices sequentially
  nv = 0;
  FOR_EACH(poly_->verts_, vi) sprintf(vi->name_, "v%d", nv++);

  return status;
}

// Compute volume integrals of PolyTree and the rad field.  For info
// on algo, see "Fast and Accurate Computation of Polyhedral Mass
// Properties," Brian Mirtich, journal of graphics tools, volume 1,
// number 2, 1996.

void PolyTree::compVolInts()
{
  Mat3 M, Rt;

  int a, b, c;
  const PolyTree *comp;
  //Vertex *vert;
  Edge *e;
  const Face *f;
  list< Handle<PolyTree> >::const_iterator compi;
  list<Face>::const_iterator fi;
  list<Vertex>::const_iterator vi;
  list<FaceConeNode>::const_iterator cni;
  Real a0, a1, da;
  Real b0, b1, db;
  Real a0_2, a0_3, a0_4, b0_2, b0_3, b0_4;
  Real a1_2, a1_3, b1_2, b1_3;
  Real d, na, nb, nc, inv;
  Real I, Ia, Ib, Iaa, Iab, Ibb, Iaaa, Iaab, Iabb, Ibbb;
  Real Icc, Iccc, Ibbc, Icca;
  Real C0, Ca, Caa, Caaa, Cb, Cbb, Cbbb;
  Real Cab, Kab, Caab, Kaab, Cabb, Kabb;
  Vect3 h, w, v, cov;
  MatX X;

  vol_ = 0.0;
  mov1_ = mov2_ = pov_ = Vect3::ZERO;

  if (!components.empty()) { // compound PolyTree
    FOR_EACH(components, compi) {
      comp = &**compi;
      X.mult(Xrp_, comp->Xpr_);  // X = xform:  comp poly -> this poly
      X.xformVect(comp->mov1_, h);
      w.scale(X.trans(), comp->vol_);
      vol_ += comp->vol_;
      mov1_.add(h);
      mov1_.add(w);
      M.set(comp->mov2_, comp->pov_);
      Rt.xpose(X.rot());
      M.premult(X.rot());
      M.postmult(Rt);
      mov2_.add(M.diag());
      mov2_.add(Vect3(X.trans().x * (2 * h.x + w.x),
		     X.trans().y * (2 * h.y + w.y),
		     X.trans().z * (2 * h.z + w.z)));
      pov_.add(M.sym());
      pov_.add(Vect3(h.y * X.trans().z + X.trans().y * (h.z + w.z),
		    h.z * X.trans().x + X.trans().z * (h.x + w.x),
		    h.x * X.trans().y + X.trans().x * (h.y + w.y)));
    }
  }

  else { // atomic PolyTree


    FOR_EACH(poly_->faces_, fi) {
      f = &*fi;

      // compute projection direction
      v.set(fabs(f->plane.normal().x),
	    fabs(f->plane.normal().y),
	    fabs(f->plane.normal().z));
      c = (v.x >= v.y) ? ((v.x >= v.z) ? 0 : 2) 
                       : ((v.y >= v.z) ? 1 : 2);
      a = (c + 1) % 3;
      b = (c + 2) % 3;

      I = Ia = Ib = Iaa = Iab = Ibb = Iaaa = Iaab = Iabb = Ibbb = 0.0;

      /* walk around face */
  
      FOR_EACH(f->cone, cni) {
	e = (Edge *) cni->nbr;
	if (e->left == f) {  // CCW edge
	  a0 = e->tail->coords_[a];
	  b0 = e->tail->coords_[b];
	  a1 = e->head->coords_[a];
	  b1 = e->head->coords_[b];
	}
	else {  // CW edge
	  a0 = e->head->coords_[a];
	  b0 = e->head->coords_[b];
	  a1 = e->tail->coords_[a];
	  b1 = e->tail->coords_[b];
	}

	da = a1 - a0;
	db = b1 - b0;
	a0_2 = a0 * a0;
	a0_3 = a0_2 * a0;
	a0_4 = a0_3 * a0;
	b0_2 = b0 * b0;
	b0_3 = b0_2 * b0;
	b0_4 = b0_3 * b0;
	a1_2 = a1 * a1;
	a1_3 = a1_2 * a1;
	b1_2 = b1 * b1;
	b1_3 = b1_2 * b1;
	C0 = a1 + a0;
	Ca = a1*C0 + a0_2;
	Caa = a1*Ca + a0_3;
	Caaa = a1*Caa + a0_4;
	Cb = b1*(b1 + b0) + b0_2;
	Cbb = b1*Cb + b0_3;
	Cbbb = b1*Cbb + b0_4;
	Cab = 3*a1_2 + 2*a1*a0 + a0_2;
	Kab = a1_2 + 2*a1*a0 + 3*a0_2;
	Caab = a0*Cab + 4*a1_3;
	Kaab = a1*Kab + 4*a0_3;
	Cabb = 4*b1_3 + 3*b1_2*b0 + 2*b1*b0_2 + b0_3;
	Kabb = b1_3 + 2*b1_2*b0 + 3*b1*b0_2 + 4*b0_3;
	I += db*C0;
	Ia += db*Ca;
	Iaa += db*Caa;
	Iaaa += db*Caaa;
	Ib += da*Cb;
	Ibb += da*Cbb;
	Ibbb += da*Cbbb;
	Iab += db*(b1*Cab + b0*Kab);
	Iaab += db*(b1*Caab + b0*Kaab);
	Iabb += da*(a1*Cabb + a0*Kabb);
      }

      I /= 2.0;
      Ia /= 6.0;
      Iaa /= 12.0;
      Iaaa /= 20.0;
      Ib /= -6.0;
      Ibb /= -12.0;
      Ibbb /= -20.0;
      Iab /= 24.0;
      Iaab /= 60.0;
      Iabb /= -60.0;

      d =  f->plane.offset();
      v = f->plane.normal();
      na = v[a];
      nb = v[b];
      nc = v[c];
      inv = 1.0 / nc;

      if (a == 0)      vol_ += inv * na * Ia;
      else if (b == 0) vol_ += inv * nb * Ib;
      else             vol_ -= ((d*I + na*Ia + nb*Ib)/nc);

#define SQR(x) ((x) * (x))
#define CUBE(x) ((x) * (x) * (x))

      Icc = (SQR(na)*Iaa + 2*na*nb*Iab + SQR(nb)*Ibb
	     + d*(2*(na*Ia + nb*Ib) + d*I)) * SQR(inv);
      mov1_[a] += inv * na * Iaa;
      mov1_[b] += inv * nb * Ibb;
      mov1_[c] += Icc;
      
      Iccc = -(CUBE(na)*Iaaa + 3*SQR(na)*nb*Iaab 
	       + 3*na*SQR(nb)*Iabb + CUBE(nb)*Ibbb
	       + 3*(SQR(na)*Iaa + 2*na*nb*Iab + SQR(nb)*Ibb)*d
	       + d*d*(3*(na*Ia + nb*Ib) + d*I)) * CUBE(inv);
      mov2_[a] += inv * na * Iaaa;
      mov2_[b] += inv * nb * Ibbb;
      mov2_[c] += Iccc;
      
      Ibbc = -(d*Ibb + na*Iabb + nb*Ibbb) * inv;
      Icca = (SQR(na)*Iaaa + 2*na*nb*Iaab + SQR(nb)*Iabb
	      + d*(2*(na*Iaa + nb*Iab) + d*Ia)) * SQR(inv);
      pov_[c] += inv * na * Iaab;
      pov_[a] += inv * nb * Ibbc;
      pov_[b] += Icca;

#undef SQR
#undef CUBE

    }

    mov1_.scale(0.5);
    mov2_.scale(1.0/3.0);
    pov_.scale(0.5);
  }


  // Compute radius, defined as the maximum distance of any vertex on
  // the PolyTree's convex hull  from the center of volume.
  cov.scale(mov1_, 1.0 / vol_);  // center of volume
  rad_ = 0.0;
  FOR_EACH(poly_->verts_, vi) {
    d = cov.distance2(vi->coords_);
    if (d > rad_) rad_ = d;
  }
  rad_ = sqrt(rad_);
}




///////////////////////////////////////////////////////////////////////////////
//  PolyTreeLibrary
///////////////////////////////////////////////////////////////////////////////

// The lookup methods are for accessing PolyTrees in the library for
// informational purposes only.  Do not directly use pointers returned
// for V-Clip!  Instead, create instances of the library PolyTrees
// using the create() method below.  The lookup methods return NULL if
// a suitable PolyTree is not found in the library.

const PolyTree *PolyTreeLibrary::lookup(const char *name) const
{
  list< Handle<PolyTree> >::const_iterator libi;

  FOR_EACH(lib, libi) 
    if (!strcmp((*libi)->name, name)) break;
  return libi == lib.end() ? NULL : &**libi;
}


const PolyTree *PolyTreeLibrary::lookup(int i) const
{
  list< Handle<PolyTree> >::const_iterator libi;

  if (i < 0 || i >= (int)lib.size()) return NULL;
  for (libi = lib.begin(); i-- > 0; ++libi);
  return &**libi;
}


///////////////////////////////////////////////////////////////////////////////
//  default geometry readers 
///////////////////////////////////////////////////////////////////////////////

// Read and return a Polyhedron; return NULL on error.  This reads in
// the verts, edges, and faces for a Polyhedron, links features
// together, and builds the Voronoi structure.

Polyhedron *readPolyhedron(istream &is)
{
  char c, next;
  int vertCounter, faceCounter, i;
  Vertex *v;
  Polyhedron *p;
  list<Vertex>::iterator vi;
  list<Vertex>::const_iterator cvi;
  char s[LONG_STR_SZ]; 
  Vect3 coords;
  VertFaceName vertName, faceName;
  static vector<Vertex *> facelist(MAX_VERTS_PER_FACE);
  //static hash_map<int, Vertex *> indexedVerts(1000); // map:  int -> Vertex *
  static map<int, Vertex *> indexedVerts; // map:  int -> Vertex *


  indexedVerts.clear();

  p = new Polyhedron;

  // read vertices
  vertCounter = 0;
  while (1) {
    is >> ws;  // eat whitespace
    next = is.peek();
    if (next == '*') {
      is >> s;
      break;
    }
    if (next == '+' || next == '-' || isdigit(next)) 
      sprintf(vertName, "v%d", vertCounter);
    else is >> vertName;
    is >> coords;
    v = p->addVertex(vertName, coords);
    indexedVerts.insert(pair<const int, Vertex *>(vertCounter++, v));
  }

  // read faces
  faceCounter = 0;
  while (1) {
    is >> faceName;
    if (*faceName == '*') break;
    if (*faceName == '-') sprintf(faceName, "f%d", faceCounter++);

    is.get(s, LONG_STR_SZ, '\n'); is.get(c);  // read line & ending '\n'
    istringstream line(s);                       // create input stream
    facelist.clear();
    while (1) {
      if ((line >> vertName).fail()) break;
      if (isdigit(*vertName)) {
	i = atoi(vertName);
	if (i < 0 || i >= vertCounter) {
	  cerr << "no vertex " << i << " on Polyhedron\a" << endl;
	  goto error;
	}
	facelist.push_back(indexedVerts.find(i)->second);
      }
      else {
        //hash_map<int, Vertex *>::const_iterator hmi;
        map<int, Vertex *>::const_iterator hmi;
	FOR_EACH(indexedVerts, hmi)
	  if (!strcmp(hmi->second->name(), vertName)) break;
	if (hmi == indexedVerts.end()) {
	  cerr << "no vertex " << vertName << " on Polyhedron\a" << endl;
	  goto error;
	}
	facelist.push_back(hmi->second);
      }
    }
    p->addFace(faceName, facelist);
  }
  if (p->faces().empty()) p->buildHull(); // no faces given - build convex hull

  p->check();  // check for a properly constructed Polyhedron

  return p;
  
error:
  delete p;
  return NULL;
}


// Read and return an atomic PolyTree, or NULL on error.  Volume
// properties are automatically computed.

PolyTree *readAtomicPolyTree(istream &is)
{
  Polyhedron *p;
  PolyTree *ptree;

  if (!(p = readPolyhedron(is))) return NULL;
  ptree = new PolyTree;
  ptree->setPoly(p);
  ptree->compVolInts();
  return ptree;
}


// Read in a compound PolyTree.  Return NULL on error.  Volume
// properties are also computed.

PolyTree *readCompoundPolyTree(istream &is, const PolyTreeLibrary &library)
{
  char c;
  PolyTree *pt, *comp;
  Se3 T;
  char name[PTREE_NAME_SZ];

  is >> ws >> c;
  if (c != '{') {
    // base case
    is.putback(c);
    is >> name;
    return library.create(name);
  }

  // read a list of "{ pose } ptree" pairs, terminating upon closing }
  pt = new PolyTree;
  strcpy(pt->name, "<internal node>");
  while (1) {
    is >> ws >> c;
    if (c == '}') break;
    is.putback(c);
    is >> T;
    if (!(comp = readCompoundPolyTree(is, library))) goto error;
    comp->xform(T);
    pt->addComponent(comp);
  }
  if (pt->buildHull()) goto error;
  pt->compVolInts();
  return pt;

error:
  cerr << "error reading polyTree \a" << endl;
  delete pt;
  return NULL;
}



// Read in an entire PolyTree specification file, appending all
// PolyTrees that are read onto library.  When compound PolyTrees,
// references may be made to any PolyTrees that preceded it in the
// file, or any PolyTrees that were in library upon the initial call
// to this function.  This function returns the number of new
// PolyTrees that were loaded.  When not in the middle of an atomic or
// compound description, Everything from a '#' to the end of line is
// interpreted as comment.

int loadPolyTreeFile(const char *fname, PolyTreeLibrary &library)
{
  PolyTree *pt;
  int n;
  char token[200], name[PTREE_NAME_SZ];

  ifstream ifs(fname);
  if (!ifs) return 0;

  n = 0;
  while (!(ifs >> token).fail()) {

    if (*token == '#') {
      ifs.get(token, 200, '\n');  // eat comment
      continue;
    }

    if (strcmp(token, "atomic") && strcmp(token, "compound")) {
      cerr << "bad token: " << token << " \a" << endl;
      break;
    }

    ifs >> name;
    if (library.lookup(name))
      cerr << "warning: PolyTree " <<name<< " already in library \a" << endl;
    
    pt = (!strcmp(token, "compound")) ? 
      readCompoundPolyTree(ifs, library) : readAtomicPolyTree(ifs);
	
    if (pt) {
      strcpy(pt->name, name);
      library.add(pt);
      n++;
    }
  }

  return n;
}
      


