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



#ifndef VCLIP_H
#define VCLIP_H

#include <list>
#include <map>
#include <string.h>

#include "mv.h"

#if INVENTOR
#include <Inventor/nodekits/SoShapeKit.h>
#endif

using namespace std;
namespace Vclip {
///////////////////////////////////////////////////////////////////////////////
// constants
///////////////////////////////////////////////////////////////////////////////


#define VF_NAME_SZ  15   // max # of chars in a Face or Vertex name (incl. \0)
#define PTREE_NAME_SZ 80 // max # of chars in a PolyTree name (including \0)

typedef char VertFaceName[VF_NAME_SZ];

// Used as the initial size for certain resizable vectors.  Generally,
// the number defined below is plenty big, however exceeding this
// limit at runtime won't break anything.
#define MAX_VERTS_PER_FACE 100


///////////////////////////////////////////////////////////////////////////////
// data structures
///////////////////////////////////////////////////////////////////////////////

template<class T>
class Handle {
  T *ptr;
public:
  void set(T *p) {ptr = p;}
  Handle() {ptr = NULL;}
  Handle(T *p) {set(p);}
  ~Handle() {delete ptr;}
  T* operator->() {return ptr;}
  T& operator*() {return *ptr;}
  const T* operator->() const {return ptr;}
  const T& operator*() const {return *ptr;}
};


template<class T>
class ShareHandle {
  T *ptr;
public:
  void set(T *p) {ptr = p; p->handleCount++;}
  ShareHandle() {ptr = NULL;}
  ShareHandle(T *p) {set(p);}
  ShareHandle(const ShareHandle &orig) {set(orig.ptr);}
  ~ShareHandle() {if (ptr && --ptr->handleCount == 0) delete ptr;}
  ShareHandle &operator=(const ShareHandle &orig) {
    if (ptr && --ptr->handleCount == 0) delete ptr;
    set(orig.ptr);
    return *this;
  };
  T *operator->() {return ptr;}
  T &operator*() {return *ptr;}
  const T* operator->() const {return ptr;}
  const T& operator*() const {return *ptr;}
};



///////////////////////////////////////////////////////////////////////////////
// Extensions to SGI's standard library
///////////////////////////////////////////////////////////////////////////////

// If using this macro, make sure the list variable is not changed
// inside the loop, e.g.:

// FOR_EACH(num.primeFactors, itor) {
//   ...
//   num = otherNum;  // error!
//   ...
// } 

#ifndef FOR_EACH 
#define FOR_EACH(list, iterator) \
  for(iterator = (list).begin(); iterator != (list).end(); ++iterator)
#endif


///////////////////////////////////////////////////////////////////////////////
// typedef VclipPose
///////////////////////////////////////////////////////////////////////////////

// V-Clip can use either class MatX or class Se3 as the basis for pose
// specification (see mv.h).  Class MatX requires fewer floating point
// operations to transform points and vectors.  However the Se3
// representation may be numerically robust (further study is needed).
// During testing, cases were found where V-Clip returned an incorrect
// disjoint result for penetrating Polyhedra when using the MatX
// representation; this behavior went away when the Se3 representation
// was used.  These cases were extremely contrived and unlikely to
// occur in normal use.  The MatX representation is initially
// recommended for all applications.  If problems occur, consider
// switching to the Se3 representation.  The interface is the same for
// either option, except for the type of the relative pose parameter
// Xr1r2 passed to PolyTree::vclip().

// define as 1 for MatX pose representation and 0 for Se3 pose
// representation.  
#define VCLIP_MATRIX_POSE 0

#if VCLIP_MATRIX_POSE
typedef MatX VclipPose;
#else
typedef Se3 VclipPose;
#endif


// used in V-Clip to cache transforme geometry
struct XformedGeom {
  const class Feature *feat;
  Vect3 coords;  
  Vect3 tail;
  Vect3 head;
  Vect3 seg;
};


///////////////////////////////////////////////////////////////////////////////
// class Plane
///////////////////////////////////////////////////////////////////////////////


class Plane {

  Vect3 normal_;   // plane = { p | < p , normal_ > + offset_ = 0 }
  Real offset_;

public:

  const Vect3 &normal() const {return normal_;}
  const Real  &offset() const {return offset_;}

  void set(const Vect3 &normal, const Vect3 &thruPoint)
    {normal_ = normal; offset_ = - Vect3::dot(thruPoint, normal);}

  // Compute signed distance from point to plane; assumes unit length normal_
  Real dist(const Vect3 &point) const 
    {return Vect3::dot(normal_, point) + offset_;}

  ostream& print(ostream &os) const;
};



///////////////////////////////////////////////////////////////////////////////
// struct VertConeNode & FaceConeNode
///////////////////////////////////////////////////////////////////////////////


struct VertConeNode
{
  const Plane *plane;
  class Edge *nbr;     // neighboring edge when plane violated

  ostream& print(ostream &os) const;

};


struct FaceConeNode
{
  const Plane *plane;
  const class Edge *nbr;     // neighboring edge when plane violated

  const FaceConeNode *ccw, *cw;
  int idx;  // ranges from 0 to n-1, where n = number of edges on face

  ostream& print(ostream &os) const;

};



///////////////////////////////////////////////////////////////////////////////
// class Feature
///////////////////////////////////////////////////////////////////////////////

class Feature {

public:
  enum Type {VERTEX, EDGE, FACE};

protected:
  Type type_;

public:
  Type type() const {return type_;}
  virtual const char *name() const = 0;
};



///////////////////////////////////////////////////////////////////////////////
// closest feature pairs and hash table
///////////////////////////////////////////////////////////////////////////////

struct PolyTreePair
{
  const class PolyTree *first, *second;
};

inline int operator==(const PolyTreePair &ptree1, const PolyTreePair &ptree2)
{return ptree1.first == ptree2.first && ptree1.second == ptree2.second;}

inline int operator<(const PolyTreePair &ptree1, const PolyTreePair &ptree2)
{return ptree1 < ptree2;}


struct FeaturePair
{
  const Feature  *first, *second;
  FeaturePair() {first = second = NULL;}
};

#if 0
struct PolyTreePairHasher : unary_function<PolyTreePair, size_t> {
  size_t operator() (const PolyTreePair &ptrees) const
    {return ((size_t) ptrees.first) 
       ^ (((unsigned int) ptrees.second) << 16) 
       ^ (((unsigned int) ptrees.second) >> 16);}
};
#endif

typedef
std::map<PolyTreePair, FeaturePair>
ClosestFeaturesHT;

///////////////////////////////////////////////////////////////////////////////
// class Vertex
///////////////////////////////////////////////////////////////////////////////


class Vertex : private Feature
{
  friend class Polyhedron;
  friend class PolyTree;

  Vect3 coords_;
  list<VertConeNode> cone;
  VertFaceName name_;

public:
  Vertex() {type_ = VERTEX;}
  const char *name() const;
  const Vect3 &coords() const {return coords_;}

};



///////////////////////////////////////////////////////////////////////////////
// class Edge
///////////////////////////////////////////////////////////////////////////////


class Edge : private Feature
{
  friend class Polyhedron;
  friend class PolyTree;

  const Vertex *tail, *head;
  const class Face *left, *right;
  Real len;
  Vect3 dir;
  Plane tplane, hplane, lplane, rplane;

  Edge() {type_ = EDGE;}

public:
  const char *name() const;

};



///////////////////////////////////////////////////////////////////////////////
// class Face
///////////////////////////////////////////////////////////////////////////////


class Face : private Feature
{
  friend class Polyhedron;
  friend class PolyTree;

  int sides;     // number of edges around boundary
  Plane plane;
  list<FaceConeNode> cone;
  VertFaceName name_;

  Face() {type_ = FACE;}

public:
  const char *name() const;

};



///////////////////////////////////////////////////////////////////////////////
// class Polyhedron
///////////////////////////////////////////////////////////////////////////////


class Polyhedron
{
  friend class PolyTree;
  friend class ShareHandle<Polyhedron>;

  int handleCount;  // number of PolyTrees pointing to thie Polyhedron

  list<Vertex> verts_;
  list<Edge  > edges_;
  list<Face  > faces_;

  void processEdge(Face *f, Vertex *tail, Vertex *head);

  static int vertVertTest(const Feature *&v1, const Feature *&v2, 
			  XformedGeom &xv1, XformedGeom &xv2,
			  const VclipPose &X12, const VclipPose &X21,
			  Vect3 &cp1, Vect3 &cp2, Real &dist);

  static int vertFaceTest(const Feature *&v, const Feature *&f, XformedGeom &xv,
			  const VclipPose &Xvf, const list<Face> &allFaces,
			  Vect3 &cpv, Vect3 &cpf, Real &dist);

  static int vertEdgeTest(const Feature *&v, const Feature *&e, 
			  XformedGeom &xv, XformedGeom &xe,
			  const VclipPose &Xve, const VclipPose &Xev,
			  Vect3 &cpv, Vect3 &cpe, Real &dist);

  static int edgeEdgeSubtest(const Feature *&e, XformedGeom &xe, Vect3 &cp);

  static int edgeEdgeTest(const Feature *&e1, const Feature *&e2, 
			  XformedGeom &xe1, XformedGeom &xe2,
			  const VclipPose &X12, const VclipPose &X21,
			  Vect3 &cp1, Vect3 &cp2, Real &dist);

  static int edgeFaceTest(const Feature *&e, const Feature *&f, 
			  XformedGeom &xe, const VclipPose &Xef, 
			  Vect3 &cpe, Vect3 &cpf, Real &dist);

  public: static Real vclip(const Polyhedron *const poly1,
		    const Polyhedron *const poly2, 
		    const VclipPose &X12, const VclipPose &X21, 
		    const Feature *&feat1, const Feature *&feat2,
		    Vect3 &cp1, Vect3 &cp2, int oneStep = 0);

#if INVENTOR
  SoShapeKit *buildInvModel() const;
#endif

public:

  Polyhedron() {handleCount = 0;}

  // construction
  inline Vertex *addVertex(const char *name, const Vect3 &coords);
  void addFace(const char *name, 
			   vector<Vertex *> &verts, int clockwise = 0);
  int buildHull();
  int check() const;

  // examination
  ostream& print(ostream &os) const;
  const list<Vertex> &verts() const {return verts_;}
  const list<Edge  > &edges() const {return edges_;}
  const list<Face  > &faces() const {return faces_;}
};



///////////////////////////////////////////////////////////////////////////////
//  class PolyTree
///////////////////////////////////////////////////////////////////////////////

// Since the same Polytree's structure may be used multiple times,
// possibly even within the same rigid object, we need a way to refer
// to a specific instance of a given PolyTree.  The Inventor solution
// is to use paths to refer to specific instances of nodes in a scene
// graph, so the node doesn't need to be stored twice.  Our solution
// is simpler: we replicate instances of the PolyTree structures, that
// is, every occurrence of the same PolyTree is uniquely stored.
// Thus, addresses of PolyTree nodes can unambiguously specify
// instances.  The Polyhedron structures remain shared: different
// PolyTree nodes can point to the same Polyhedron.  Since most of the
// storage is in the Polyhedron structures, not the PolyTree nodes,
// this is not too wasteful.  It is also much easier to point to
// sepcific instances of PolyTrees than with the path method.

// Replication of PolyTree nodes allows us to have different names for
// the different instances of the same PolyTree.  It also allows the
// Tpm field (see below).  If PolyTrees were not replicated, there
// could be no such field since the same PolyTree node could be shared
// by different geometry hierarchies.  In this situation, the only
// alternative is to store a transformation to the parent's frame, and
// accumulate these transformations along a specific path.  The
// replication approach saves time because the Tpm's can be
// precomputed.

class PolyTree 
{

  friend class PolyTreeLibrary;

  // Pointer to a Polyhedron.  For an atomic PolyTree, this is the
  // geometry of the PolyTree itself; for a compound PolyTree, this is
  // the geometry of the convex hull.
  ShareHandle<Polyhedron> poly_;   

  // Volume integrals, relative to this PolyTree's reference frame
  Real vol_;   // volume:                                vol    = int(dV)
  Vect3 mov1_; // 1st moment of volume:                  mov1.x = int(x dV)
  Vect3 mov2_; // undiagonalized 2nd moment of volume:   mov2.x = int(x^2 dV)
  Vect3 pov_;  // product of volume:                     pov.x  = int(yz dV)

  Real rad_;   // "radius" of PolyTree, relative to center of volume


  // An entire PolyTree shares a common reference frame (r).  Tpr_ and
  // Trp_ are the transformations between each PolyTree's local frame
  // (p) and the PolyTree reference frame.  These fields are
  // recomputed each time the PolyTree is included (replicated) into
  // another hierarchy.  By default, the reference frame is simply the
  // local frame of the root PolyTree, so for the root node Tpr_ and
  // Trp_ are identity transformations.  In some cases, however, it is
  // advantageous to store transformations with respect to a different
  // frame.  For example, in rigid body simulation, an object's body
  // frame is the most useful frame, so all of the PolyTree nodes in
  // it's geometry use that as the reference frame.  The reference
  // frame of a hierarchy is updated using the xform() method.  Xpr_
  // and Xrp_ are the MatX_ equivalents of Tpr_ and Trp_.
  Se3 Tpr_, Trp_;
  MatX Xpr_, Xrp_;

  list< Handle<PolyTree> > components;  // children in convex decomp'n, if any

public:
  char name[PTREE_NAME_SZ];
  
private:

  void printRecur(ostream &os, int level) const;

  // copy constructor (perform a deep copy of this)
  PolyTree(const PolyTree &orig);

public:

  PolyTree();

  // construction
  void setPoly(Polyhedron *p) {poly_ = ShareHandle<Polyhedron>(p);}
  void addComponent(PolyTree *comp)
    {Handle<PolyTree> h(comp); components.push_back(h); h.set(NULL);}
  int buildHull();
  void xform(const Se3 &T);

  // examination
  const Polyhedron  *poly() const {return &*poly_;}
  int numNodes() const;
  int numLeaves() const;
  ostream& print(ostream &os) const;
  const Se3   &Tpr()  const {return Tpr_;}
#if INVENTOR
  SoNode *buildInvModel() const;
#endif

  // volume integrals
  void compVolInts();
  const Real  &vol()  const {return vol_;}
  const Vect3 &mov1() const {return mov1_;}
  const Vect3 &mov2() const {return mov2_;}
  const Vect3 &pov()  const {return pov_;}
  const Real  &rad()  const {return rad_;}

  // V-Clip
private:
  static Real vclip_(
		     const PolyTree *const ptree1, const PolyTree *const ptree2,
		     const VclipPose &Xr1r2, const VclipPose &Xr2r1,
		     ClosestFeaturesHT &ht,
		     Vect3 &cp1, Vect3 &cp2);
public:

  // Application callpoint for V-Clip.  Xr1r2 is the transformation
  // from the reference frame of ptree1 to the reference frame of
  // ptree2.  Cp1 and cp2 are also returned in these frames.
  inline static Real vclip(const PolyTree *const ptree1, 
			   const PolyTree *const ptree2,
			   const VclipPose &Xr1r2, 
			   ClosestFeaturesHT &ht,
			   Vect3 &cp1, Vect3 &cp2)
    {
      VclipPose Xr2r1;
      Xr2r1.invert(Xr1r2);
      return vclip_(ptree1, ptree2, Xr1r2, Xr2r1, ht, cp1, cp2);
    }

  // V-Clip does not return closest features to the caller.  If they're
  // needed, this function returns the most recent feature pair for the
  // given PolyTrees, or NULL is no hash table entry can be found.
  static void vclipFeatures(
			    const PolyTree *const ptree1, 
			    const PolyTree *const ptree2,
			    ClosestFeaturesHT &ht,
			    const Feature *&feat1, const Feature *&feat2);
};



///////////////////////////////////////////////////////////////////////////////
//  class PolyTreeLibrary
///////////////////////////////////////////////////////////////////////////////


class PolyTreeLibrary
{
  list< Handle<PolyTree> > lib;

public:

  void clear() {lib.clear();}                      // clear out the library
  int size() {return lib.size();}                  // return # of entries in lib

  void add(PolyTree *pt)                           // add PolyTree to library
    {Handle<PolyTree> h(pt); lib.push_front(h); h.set(NULL);}

  const PolyTree *lookup(const char *name) const;  // lookup by name
  const PolyTree *lookup(int i) const;             // lookup by index
  
  PolyTree *create(const char *name) const         // instantiate by name
    {const PolyTree *pt; return (pt = lookup(name)) ? new PolyTree(*pt) : NULL;}

  PolyTree *create(int i) const                    // instantiate by index
    {const PolyTree *pt; return (pt = lookup(i)) ? new PolyTree(*pt) : NULL;}
};
  

///////////////////////////////////////////////////////////////////////////////
//  stream operators
///////////////////////////////////////////////////////////////////////////////


inline ostream &operator<<(ostream &os, const Plane &p) {return p.print(os);}
inline ostream &operator<<(ostream &os, const VertConeNode &vcn) 
  {return vcn.print(os);}
inline ostream &operator<<(ostream &os, const FaceConeNode &fcn) 
  {return fcn.print(os);}
inline ostream &operator<<(ostream &os, const Polyhedron *poly)
  {return poly->print(os);}
inline ostream &operator<<(ostream &os, const PolyTree *pt) 
  {return pt->print(os);}



///////////////////////////////////////////////////////////////////////////////
//  inline methods
///////////////////////////////////////////////////////////////////////////////


Vertex *Polyhedron::addVertex(const char *name, const Vect3 &coords)
{
  Vertex v;
  v.coords_ = coords;
  strcpy(v.name_, name);
  verts_.push_back(v);
  return &verts_.back();
}



///////////////////////////////////////////////////////////////////////////////
//  default goemetry readers
///////////////////////////////////////////////////////////////////////////////

// Provided for convenience; not part of API.

int loadPolyTreeFile(const char *fname, PolyTreeLibrary &library);
};
#endif  // #ifndef VCLIP_H
