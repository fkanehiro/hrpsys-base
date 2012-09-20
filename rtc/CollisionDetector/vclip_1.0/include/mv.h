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



#ifndef MV_H
#define MV_H

#include <iostream>
#include <math.h>

#if INVENTOR
#include <Inventor/SbLinear.h>
#include <Inventor/nodes/SoTransform.h>
#endif

#if OPCOUNTS
#include "OpCounter.h"
#endif

using namespace std;
namespace Vclip {
///////////////////////////////////////////////////////////////////////////////
//
//  Matrix-Vector library 
//
///////////////////////////////////////////////////////////////////////////////

/*

  SGI Inventor Interface
  ----------------------
  The compiler option -DINVENTOR causes compilation of functions
  for translating to and from the Inventor Library's vector and matrix
  types.


  Counting Operations
  -------------------
  To count floating point operations using the OpCounter package, use
  the compiler flag -DOPCOUNTS.  This will cause type Real to be
  compiled as type Double, which is like type double, but also has
  mechanisms for counting operations.  See OpCounter.h for details.
  Note: This functionality is not included with the V-Clip
  distribution.


  Implied Operands
  ----------------
  For most operations, the result is returned in 'this'.  For example:
 
  Vect3 a,b,c;
  ...
  a.add(b,c);  // a <- vector sum of b & c

  In the case of vector add, it does not matter if one of the operands
  is also the destination:  a.add(a,b) simply adds b to a, as
  expected.  However, for other operations, the destination can not be
  passed as an operand, as this will invalidate the computation:

  Mat3 M, N;
  ...
  M.mult(M, N);  // WRONG!  M will not be matrix product M N upon return!

  To fix this problem, such functions have versions that assume the
  destination as an implied operand, and take care to compute the
  result correctly in this situation.  The above incorrect call coan
  be replaced with

  M.postmult(N);  // M <- matrix product M N

  For consistency, and because it reduces parameter passing, implied
  operand versions of most operations exist, even when the destination
  could be passed as an operand without a problem.  For example:

  Vect3 a,b,c;
  ...
  a.add(b);  // a <- vector sum of a & b

  All problems associated with using destinations as operands can be
  avoided by following one simple rule: 

       *** NEVER EXPLICITLY PASS THE DESTINATION AS AN OPERAND. ***

  If you are trying to do this, chances are there is a different
  version of the function which does not require the destination to be
  explicitly passed.  For example,

  MatX M, N;
  ...
  M.mult(N, M);  // No good!  Desination M passed as explicit operand
  M.premult(N);  // Ok.  M <- N M  (M is an implicit operand)
  ...
  M.invert(M);   // No good!  Desination M passed as explicit operand
  M.invert();    // Ok.  M <- M^-1 (M is an implicit operand)

  Violating the rule is sometimes ok, but adhere to it, and never go
  wrong.  There are a few cases which still pose problems.  For
  instance, one can't square a matrix M in place.  M.mult(M,M)
  violates the rule, but so does M.premult(M).  Here, one must compute
  the square into a new matrix: S.mult(M,M).


  Transforming Vectors and Points
  -------------------------------
  The transformation operations differ slightly from the other
  operations in that the result is not returned via 'this'.  Instead,
  'this' is the transformation description (e.g. a rotation matrix),
  and the source and destination vectors/points are passed as
  operands, in that order.  There are also implied operand versions
  that only take a single vector/point argument, transforming it and
  storing the result in the same place.  For example:

  Se3 T;
  Vect3 v, xv;
  ...
  T.xformVect(v, xv);  // xv <- transformed version of v
  T.xformVect(v);      // v  <- transformed version of v

  For some transformation objects, calls like xformVect(v, v) are ok,
  but for others, they are not - xformVect(v) must be used.  
  To ensure correct usage:

  *** NEVER PASS THE SAME SOURCE AND DESTINATION TO A TRANSFORM FUNCTION ***

*/

#if OPCOUNTS
typedef Double Real;
#else
typedef double Real;
#endif


class Vect3;
class Mat3;
class MatX;
class Quat;
class Se3;


///////////////////////////////////////////////////////////////////////////////
//
//  class Vect3
//
///////////////////////////////////////////////////////////////////////////////


class Vect3 {

  friend class Mat3;
  friend class MatX;
  friend class Quat;
  friend class Se3;

public:

  Real x, y, z;

  // constructors //////////////////////////////////////////////////////////////

  Vect3() {}
  Vect3(Real x_, Real y_, Real z_) {set(x_, y_, z_);}

  // setters / accessors / translators /////////////////////////////////////////

  void set(Real x_, Real y_, Real z_) {x = x_; y = y_; z = z_;}

  //Real &coord(int i) {return *(&x + i);} // index-based access:  0=x, 1=y, 2=z.

  // index-based access:  0=x, 1=y, 2=z.
  const Real &operator[](int i) const {return *(&x + i);} 
        Real &operator[](int i)       {return *(&x + i);} 


#if INVENTOR
  inline void set(const SbVec3f &v);
  inline void toSbVec3f(SbVec3f &v) const;
#endif

  // input / output ////////////////////////////////////////////////////////////

  ostream &print(ostream &os) const;

  // Read vector from stream If the next string read is the single
  // character i, j, or k, the appropriate unit vector is returned.
  // Plus and minus signs may be optionally placed in front of these
  // codes, e.g. +i or -k
  istream &read(istream &is);

  // operations not returning a Vect3 //////////////////////////////////////////

  //inline int operator==(const Vect3 &u, const Vect3 &v);
  inline int operator==(const Vect3 &other) const;
  inline Real dot(const Vect3 &other) const;
  inline Real norm()  const;
  inline Real norm2() const;  // norm^2
  inline Real distance (const Vect3 &other) const;
  inline Real distance2(const Vect3 &other) const;  // distance^2
  inline Real min() const;
  inline Real max() const;
  inline Real minAbs() const;
  inline Real maxAbs() const;
  inline void swap(Vect3 &other);
  // for symmetric invocations:
  static Real dot      (const Vect3 &u, const Vect3 &v) {return u.dot      (v);}
  static Real distance (const Vect3 &u, const Vect3 &v) {return u.distance (v);}
  static Real distance2(const Vect3 &u, const Vect3 &v) {return u.distance2(v);}
  static void swap     (      Vect3 &u,       Vect3 &v) {       u.swap     (v);}
  
  // operations returning result via this //////////////////////////////////////

  // The operation result indicated by the comments is always returned
  // in this.  The symbol [!] indicates that this must be distinct
  // from all of the operands.  The three-argument crossAdd() is
  // slightly different: this must be distinct from u and v, but not
  // necessarily from w.

  inline void normalize(const Vect3 &v);            // v/|v|
  inline void normalize();                          // this/|this|
  inline void negate(const Vect3 &v);               // -v
  inline void negate();                             // -this
  inline void add(const Vect3 &u, const Vect3 &v);  // u + v
  inline void add(const Vect3 &v);                  // this + v
  inline void sub(const Vect3 &u, const Vect3 &v);  // u - v
  inline void sub(const Vect3 &v);                  // this - v
  inline void mult(const Vect3 &u, const Vect3 &v); // u * v (component-wise) 
  inline void mult(const Vect3 &v);                 // this * v (component-wise)
  inline void scale(const Vect3 &v, Real s);        // s * v
  inline void scale(Real s);                        // s * this
  inline void cross(const Vect3 &u, const Vect3 &v);// u x v  [!]
  inline void precross(const Vect3 &v);             // v x this  [!]
  inline void postcross(const Vect3 &v);            // this x v  [!]
  inline void crossAdd(const Vect3 &u, const Vect3 &v, const Vect3 &w);
                                                    // u x v + w [!]
  inline void crossAdd(const Vect3 &u, const Vect3 &v);
                                                    //  u x v + this [!]
  inline void displace(const Vect3 &v, const Vect3 &u, Real lambda); 
                                                    // v + lambda * u
  inline void displace(const Vect3 &u, Real lambda);// this + lambda * u
  inline void interpolate(const Vect3 &u, const Vect3 &v, Real lambda);  
                                                    // (1-lambda)*u + lambda*v


  // Vect3 constants ///////////////////////////////////////////////////////////

  static const Vect3 ZERO;
  static const Vect3 I;     // unit vector along +x axis
  static const Vect3 J;     // unit vector along +y axis
  static const Vect3 K;     // unit vector along +z axis
  static const Vect3 I_;    // unit vector along -x axis
  static const Vect3 J_;    // unit vector along -y axis
  static const Vect3 K_;    // unit vector along -z axis

};



///////////////////////////////////////////////////////////////////////////////
//
//  class Mat3
//
///////////////////////////////////////////////////////////////////////////////


class Mat3 {

  friend class Quat;
  friend class MatX;

private:

  // (stored in row-major order)
  Real xx, xy, xz,
       yx, yy, yz,
       zx, zy, zz;

public:

  // constructors //////////////////////////////////////////////////////////////

  Mat3() {}
  Mat3(const Vect3 &diag, const Vect3 &sym) {set(diag, sym);}
  Mat3(const Vect3 &axis, Real angle, int normalizeAxis = 1)
    {set(axis, angle, normalizeAxis);}
  Mat3(const Quat &q) {set(q);}


  // setters / accessors ///////////////////////////////////////////////////////

  // make a symmetric matrix, given the diagonal and symmetric
  // (off-diagonal) elements in canonical order
  inline void set(const Vect3 &diag, const Vect3 &sym);
  // set Mat3 as a rotation of 'angle' radians about 'axis'
  // axis is automatically normalized unless normalizeAxis = 0
  inline void set(const Vect3 &axis, Real angle, int normalizeAxis = 1);
  void set(const Quat &q);

  // index-based access:  0=xrow, 1=yrow, 2=zrow.
  const Vect3 &operator[](int i) const {return *(((Vect3 *) &xx) + i);}
        Vect3 &operator[](int i)       {return *(((Vect3 *) &xx) + i);}

  // set matrix to the skew symmetric matrix corresponding to 'v X'
  inline void setSkew(const Vect3 &v);

  // for reading rows
  const Vect3 &xrow() const {return *((Vect3 *) &xx);}
  const Vect3 &yrow() const {return *((Vect3 *) &yx);}
  const Vect3 &zrow() const {return *((Vect3 *) &zx);}
  // for writing to rows
  Vect3 &xrow()  {return *((Vect3 *) &xx);}
  Vect3 &yrow()  {return *((Vect3 *) &yx);}
  Vect3 &zrow()  {return *((Vect3 *) &zx);}

  // for reading columns
  Vect3 xcol() const {return Vect3(xx, yx, zx);}
  Vect3 ycol() const {return Vect3(xy, yy, zy);}
  Vect3 zcol() const {return Vect3(xz, yz, zz);}
  // for writing to columns
  inline void setXcol(const Vect3 &v);
  inline void setYcol(const Vect3 &v);
  inline void setZcol(const Vect3 &v);

  // for reading a symmetric matrix
  Vect3 diag() const {return Vect3(xx, yy, zz);}
  Vect3 sym()  const {return Vect3(yz, zx, xy);}


  // input / output ////////////////////////////////////////////////////////////

  ostream& print(ostream &os) const;

  // operations not returning a Mat3 ///////////////////////////////////////////

  inline Real det() const;

  // operations returning result via this //////////////////////////////////////

  // The operation result indicated by the comments is always returned
  // in this.  The symbol [!] indicates that this must be distinct
  // from all of the operands.  The invert() methods are based on the
  // explicit inversion formula from determinants; there are faster
  // and more accurate ways.  The invert() methods return one if the
  // matrix was not invertible, otherwise zero.

  inline void xpose(const Mat3 &M);                   // M^T       [!]
  inline void xpose();                                // this^T
  inline void symmetrize(const Mat3 &M);              // M + M^T
  inline void symmetrize();                           // this + this^T
         int  invert(const Mat3 &M);                  // M^-1      [!]
         int  invert();                               // this^-1
  inline void negate(const Mat3 &M);                  // -M
  inline void negate();                               // -this
  inline void add(const Mat3 &M, const Mat3 &N);      // M + N
  inline void add(const Mat3 &M);                     // this + M
  inline void sub(const Mat3 &M, const Mat3 &N);      // M - N
  inline void sub(const Mat3 &M);                     // this - M
  inline void scale(const Mat3 &M, Real s);           // s * M
  inline void scale(Real s);                          // s * this
         void mult(const Mat3 &M, const Mat3 &N);     // M * N     [!]
         void premult(const Mat3 &M);                 // M * this  [!]
         void postmult(const Mat3 &M);                // this * M  [!]

  // Transforming Vect3s ///////////////////////////////////////////////////////

  inline void xform(const Vect3 &v, Vect3 &xv) const; // (this)(v) => xv; 
                                                      // v & xv must be distinct
  inline void xform(Vect3 &v) const;                  // (this)(v) => v

  // These are exactly like the above methods, except the inverse
  // transform this^-1 (= this^T) is used.  This can be thought of as
  // a row vector transformation, e.g.: (v^T)(this) => xv^T
  inline void invXform(const Vect3 &v, Vect3 &xv) const;
  inline void invXform(Vect3 &v) const;


  // Mat3 constants ////////////////////////////////////////////////////////////

  static const Mat3 ZERO;    // zero matrix
  static const Mat3 ID;      // identity matrix

};


///////////////////////////////////////////////////////////////////////////////
//
//  class MatX
//
///////////////////////////////////////////////////////////////////////////////


class MatX {

  friend class Se3;

private:

  Mat3 R;
  Vect3 d;

public:

  // constructors //////////////////////////////////////////////////////////////

  MatX()                                {}
  MatX(const Mat3 &R_, const Vect3 &d_) {set(R_, d_);}
  MatX(const Se3 &T)                    {set(T);}


  // setters / accessors / translators /////////////////////////////////////////
  
  inline void set(const Mat3 &R_, const Vect3 &d_) {R = R_; d = d_;}
  inline void set(const Se3 &T);

  const Mat3  &rot()   const {return R;}
  const Vect3 &trans() const {return d;}
        Mat3  &rot()         {return R;}
        Vect3 &trans()       {return d;}
  
  // input / output ////////////////////////////////////////////////////////////

  inline ostream& print(ostream &os) const;
  inline istream& read(istream &is);

  // operations returning result via this //////////////////////////////////////

  // The operation result indicated by the comments is always returned
  // in this.  The symbol [!] indicates that this must be distinct
  // from all of the operands.

  void mult(const MatX &M, const MatX &N);    // M * N     [!]
  void premult(const MatX &M);                // M * this  [!]
  void postmult(const MatX &M);               // this * M  [!]
  void invert(const MatX &M);                 // M^-1      [!]
  void invert();                              // this^-1

  // Transforming Vect3s ///////////////////////////////////////////////////////

  // MatXs can transform elements of R^3 either as vectors or as
  // points.  The [!] indicates that the operands must be distinct.

  inline void xformVect(const Vect3 &v, Vect3 &xv) const; // this*(v 0)=>xv  [!]
  inline void xformVect(Vect3 &v) const;                  // this*(v 0)=>v
  inline void xformPoint(const Vect3 &p, Vect3 &xp) const;// this*(p 1)=>xp  [!]
  inline void xformPoint(Vect3 &p) const;                 // this*(p 1)=>p

  // These are exactly like the above methods, except the inverse
  // transform this^-1 is used.
  inline void invXformVect(const Vect3 &v, Vect3 &xv) const;
  inline void invXformVect(Vect3 &v) const;                 
  inline void invXformPoint(const Vect3 &p, Vect3 &xp) const;
  inline void invXformPoint(Vect3 &p) const;                 


  // MatX constants ////////////////////////////////////////////////////////////

  static const MatX ID;      // identity matrix

};



///////////////////////////////////////////////////////////////////////////////
//
//  class Quat
//
///////////////////////////////////////////////////////////////////////////////


class Quat {

  friend class Mat3;
  friend class Se3;

private:

  Real s_, x_, y_, z_;


public:

  // constructors //////////////////////////////////////////////////////////////


  Quat() {}
  Quat(Real s, Real x, Real y, Real z) {set(s, x, y, z);}
  Quat(Real angle, const Vect3 &axis, int normalizeAxis = 1)
    {set(angle, axis, normalizeAxis);}
  Quat(const Mat3 &R) {set(R);}

  // setters / accessors / translators /////////////////////////////////////////

  void set(Real s, Real x, Real y, Real z) {s_=s; x_=x; y_=y; z_=z;}
  // set a quaternion to a rotation of 'angle' radians about 'axis'
  // the axis passed is automatically normalized unless normalizeAxis = 0
  void set(Real angle, const Vect3 &axis, int normalizeAxis = 1);
  void set(const Mat3 &R);
#if INVENTOR
  inline void set(const SbRotation &R);
  inline void toSbRotation(SbRotation &R) const;
#endif

  Real s() const {return s_;}
  Real x() const {return x_;}
  Real y() const {return y_;}
  Real z() const {return z_;}
  inline Vect3 axis() const;  // normalized axis of rotation
  inline Real angle() const;  // angle of rotation in radians, in range [0, 2pi)

  // input / output ////////////////////////////////////////////////////////////

  ostream& print(ostream &os) const;

  // operations not returning a Quat ///////////////////////////////////////////

  int operator==(const Quat &other)
    {return s_ == other.s_ && x_ == other.x_ 
         && y_ == other.y_ && z_ == other.z_;}


  // operations returning result via this //////////////////////////////////////

  // The operation result indicated by the comments is always returned
  // in this.  The symbol [!] indicates that this must be distinct
  // from all of the operands.

  inline void normalize(const Quat &q);               // q/|q|
  inline void normalize();                            // this/|this|
  inline void invert(const Quat &q);                  // q^-1
  inline void invert();                               // this^-1
  void mult(const Quat &p, const Quat &q);            // p * q     [!]
  void premult(const Quat &q);                        // q * this  [!]
  void postmult(const Quat &q);                       // this * q  [!]

  // Transforming Vect3s ///////////////////////////////////////////////////////

  void xform(const Vect3 &u, Vect3 &v) const;    // this (v 0) this^-1 => xv
  void xform(Vect3 &v) const;                    // this (v 0) this^-1 => v

  // These are exactly like the above methods, except the inverse
  // transform is used (i.e. the factors this and this^-1 are swapped).
  void invXform(const Vect3 &v, Vect3 &xv) const;
  void invXform(Vect3 &v) const;


  // miscellaneous /////////////////////////////////////////////////////////////

  // Return the quaternion derivatives in 'this', given a current
  // orientation q and body angular velocity w.  Note that what is
  // returned in 'this' is not a unit quaternion, but the derivative
  // of one!
  inline void deriv(const Quat &q, const Vect3 &w);

  // Quat constants ////////////////////////////////////////////////////////////

  static const Quat ID;   // identity quaternion
};



///////////////////////////////////////////////////////////////////////////////
//
//  class Se3
//
///////////////////////////////////////////////////////////////////////////////


class Se3 {

  friend class MatX;

private:

  Quat q;     // rotation component
  Vect3 d;    // translation component

public:

  // constructors //////////////////////////////////////////////////////////////


  Se3() {}
  Se3(const Quat &q_, const Vect3 &d_) {set(q_, d_);}
  Se3(const MatX &X) {set(X);}

  // setters / accessors / translators /////////////////////////////////////////

  void set(const Quat &q_, const Vect3 &d_) {q = q_; d = d_;}
  void set(const MatX &X) {q.set(X.R); d = X.d;}
#if INVENTOR
  inline void set(const SoTransform *T);
  inline void toSoTransform(SoTransform *T) const;
#endif

  const Quat  &rot()   const {return q;}
  const Vect3 &trans() const {return d;}
        Quat  &rot()         {return q;}
        Vect3 &trans()       {return d;}

  // input / output ////////////////////////////////////////////////////////////


  inline ostream& print(ostream &os) const;

  // Read Se3 from input stream.  The Se3 specification must be
  // enclosed in curly brackets { }.  Se3's are built up from a
  // sequence of translation and rotation operations.  A translation
  // opereration is specified by "trans v", where v is the translation
  // vector.  A rotation operation is specified by: "rot a v", where a
  // is the scalar rotation in *degrees*, and v is the axis of
  // rotation (a vector).

  istream& read(istream &is);

  // operations returning result via this //////////////////////////////////////

  // The operation result indicated by the comments is always returned
  // in this.  The symbol [!] indicates that this must be distinct
  // from all of the operands.

  inline void mult(const Se3 &T, const Se3 &U);    // T * U     [!]
  inline void premult(const Se3 &T);               // T * this  [!]
  inline void postmult(const Se3 &T);              // this * T  [!]
  inline void invert(const Se3 &T);                // T^-1
  inline void invert();                            // this^-1

  // Transforming Vect3s ///////////////////////////////////////////////////////

  // Se3s can transform elements of R^3 either as vectors or as
  // points.  Multiple operands need not be distinct.

  inline void xformVect(const Vect3 &v, Vect3 &xv) const;  // this * (v 0) => xv
  inline void xformVect(Vect3 &v) const;		   // this * (v 0) => v
  inline void xformPoint(const Vect3 &p, Vect3 &xp) const; // this * (p 1) => xp
  inline void xformPoint(Vect3 &p) const;		   // this * (p 1) => p

  // These are exactly like the above methods, except the inverse
  // transform this^-1 is used.
  inline void invXformVect(const Vect3 &v, Vect3 &xv) const;
  inline void invXformVect(Vect3 &v) const;                 
  inline void invXformPoint(const Vect3 &p, Vect3 &xp) const;
  inline void invXformPoint(Vect3 &p) const;                 


  // Se3 constants /////////////////////////////////////////////////////////////

  static const Se3 ID;      // identity Se3
};



///////////////////////////////////////////////////////////////////////////////
//
//  stream operators
//
///////////////////////////////////////////////////////////////////////////////


inline ostream &operator<<(ostream &os, const Vect3 &v) {return v.print(os);}
inline ostream &operator<<(ostream &os, const Mat3 &M)  {return M.print(os);}
inline ostream &operator<<(ostream &os, const MatX &X)  {return X.print(os);}
inline ostream &operator<<(ostream &os, const Quat &q)  {return q.print(os);}
inline ostream &operator<<(ostream &os, const Se3 &T)   {return T.print(os);}

inline istream &operator>>(istream &os, Vect3 &v)       {return v.read(os);}
inline istream &operator>>(istream &is, MatX &X)        {return X.read(is);}
inline istream &operator>>(istream &is, Se3 & T)        {return T.read(is);}



///////////////////////////////////////////////////////////////////////////////
//
//  inline function definitions
//
///////////////////////////////////////////////////////////////////////////////


#if INVENTOR
void Vect3::set(const SbVec3f &vec)
{
  const float *v = vec.getValue();
  x = v[0]; y = v[1]; z = v[2];
}
#endif


#if INVENTOR
void Vect3::toSbVec3f(SbVec3f &vec) const
{
#if OPCOUNTS
  vec.setValue(x.toDouble(), y.toDouble(), z.toDouble());
#else
  vec.setValue(x, y, z);
#endif
}
#endif


int Vect3::operator==(const Vect3 &other) const
{
  return x == other.x && y == other.y && z == other.z;
}


Real Vect3::dot(const Vect3 &other) const
{
  return x * other.x + y * other.y + z * other.z;
}


Real Vect3::norm() const
{
  return sqrt(x * x + y * y + z * z);
}


Real Vect3::norm2() const
{
  return (x * x + y * y + z * z);
}


Real Vect3::distance(const Vect3 &other) const
{
  Vect3 w;

  w.sub(other, *this);
  return w.norm();
}


Real Vect3::distance2(const Vect3 &other) const
{
  Vect3 w;

  w.sub(other, *this);
  return w.norm2();
}


Real Vect3::min() const
{
  return (x <= y) ? ((x <= z) ? x : z) : ((y <= z) ? y : z);
}


Real Vect3::max() const
{
  return (x >= y) ? ((x >= z) ? x : z) : ((y >= z) ? y : z);
}


Real Vect3::minAbs() const
{
  Real ax, ay, az;
  
  ax = fabs(x);
  ay = fabs(y);
  az = fabs(z);
  return (ax <= ay) ? ((ax <= az) ? ax : az) : ((ay <= az) ? ay : az);
}


Real Vect3::maxAbs() const
{
  Real ax, ay, az;
  
  ax = fabs(x);
  ay = fabs(y);
  az = fabs(z);
  return (ax >= ay) ? ((ax >= az) ? ax : az) : ((ay >= az) ? ay : az);
}


void Vect3::swap(Vect3 &other)
{
  Vect3 tmp;

  tmp = *this;
  *this = other;
  other = tmp;
}


void Vect3::normalize(const Vect3 &v)
{
  Real s;

  s = 1.0 / sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
  x = s * v.x;
  y = s * v.y;
  z = s * v.z;
}


void Vect3::normalize()
{
  Real s;

  s = 1.0 / sqrt(x * x + y * y + z * z);
  x *= s;
  y *= s;
  z *= s;
}


void Vect3::negate(const Vect3 &v)
{
  x = - v.x;
  y = - v.y;
  z = - v.z;
}


void Vect3::negate()
{
  x = - x;
  y = - y;
  z = - z;
}


void Vect3::add(const Vect3 &u, const Vect3 &v)
{
  x = u.x + v.x;
  y = u.y + v.y;
  z = u.z + v.z;
}


void Vect3::add(const Vect3 &v)
{
  x += v.x;
  y += v.y;
  z += v.z;
}


void Vect3::sub(const Vect3 &u, const Vect3 &v)
{
  x = u.x - v.x;
  y = u.y - v.y;
  z = u.z - v.z;
}


void Vect3::sub(const Vect3 &v)
{
  x -= v.x;
  y -= v.y;
  z -= v.z;
}


void Vect3::mult(const Vect3 &u, const Vect3 &v)
{
  x = u.x * v.x;
  y = u.y * v.y;
  z = u.z * v.z;
}


void Vect3::mult(const Vect3 &v)
{
  x *= v.x;
  y *= v.y;
  z *= v.z;
}


void Vect3::scale(const Vect3 &v, Real s)
{
  x = s * v.x;
  y = s * v.y;
  z = s * v.z;
}


void Vect3::scale(Real s)
{
  x *= s;
  y *= s;
  z *= s;
}



void Vect3::cross(const Vect3 &u, const Vect3 &v)
{
  x = u.y * v.z - u.z * v.y;
  y = u.z * v.x - u.x * v.z;
  z = u.x * v.y - u.y * v.x;
}


void Vect3::precross(const Vect3 &v)
{
  Real ox, oy;

  ox = x;
  oy = y;
  x = v.y * z - v.z * oy;
  y = v.z * ox - v.x * z;
  z = v.x * oy - v.y * ox;
}


void Vect3::postcross(const Vect3 &v)
{
  Real ox, oy;

  ox = x;
  oy = y;
  x = oy * v.z - z * v.y;
  y = z * v.x - ox * v.z;
  z = ox * v.y - oy * v.x;
}


void Vect3::crossAdd(const Vect3 &u, const Vect3 &v, const Vect3 &w)
{
  x = u.y * v.z - u.z * v.y + w.x;
  y = u.z * v.x - u.x * v.z + w.y;
  z = u.x * v.y - u.y * v.x + w.z;
}


void Vect3::crossAdd(const Vect3 &u, const Vect3 &v)
{
  x += u.y * v.z - u.z * v.y;
  y += u.z * v.x - u.x * v.z;
  z += u.x * v.y - u.y * v.x;
}


void Vect3::displace(const Vect3 &v, const Vect3 &u, Real lambda)
{
  x = v.x + lambda * u.x;
  y = v.y + lambda * u.y;
  z = v.z + lambda * u.z;
}


void Vect3::displace(const Vect3 &u, Real lambda)
{
  x += lambda * u.x;
  y += lambda * u.y;
  z += lambda * u.z;
}


void Vect3::interpolate(const Vect3 &u, const Vect3 &v, Real lambda)
{
  Real lambda2 = 1.0 - lambda;

  x = lambda2 * u.x + lambda * v.x;
  y = lambda2 * u.y + lambda * v.y;
  z = lambda2 * u.z + lambda * v.z;
}


void Mat3::set(const Vect3 &diag, const Vect3 &sym)
{
  xx = diag.x;
  yy = diag.y;
  zz = diag.z;
  yz = zy = sym.x;
  zx = xz = sym.y;
  xy = yx = sym.z;
}


void Mat3::set(const Vect3 &axis, Real angle, int normalizeAxis)
{
  Quat q;

  q.set(angle, axis, normalizeAxis);
  set(q);
}

void Mat3::setXcol(const Vect3 &v)
{
  xx = v.x;
  yx = v.y;
  zx = v.z;
}


void Mat3::setYcol(const Vect3 &v)
{
  xy = v.x;
  yy = v.y;
  zy = v.z;
}


void Mat3::setZcol(const Vect3 &v)
{
  xz = v.x;
  yz = v.y;
  zz = v.z;
}


void Mat3::setSkew(const Vect3 &v)
{
  xx = yy = zz = 0.0;
  zy =  v.x;
  yz = -v.x;
  xz =  v.y;
  zx = -v.y;
  yx =  v.z;
  xy = -v.z;
}


Real Mat3::det() const
{
  return  xx * (yy * zz - yz * zy)
        + xy * (yz * zx - yx * zz)
        + xz * (yx * zy - yy * zx);
}


void Mat3::xpose(const Mat3 &M)
{
  xx = M.xx;
  xy = M.yx;
  xz = M.zx;

  yx = M.xy;
  yy = M.yy;
  yz = M.zy;

  zx = M.xz;
  zy = M.yz;
  zz = M.zz;
}


void Mat3::xpose()
{
  Real tmp;

  tmp = xy;
  xy = yx;
  yx = tmp;;

  tmp = yz;
  yz = zy;
  zy = tmp;

  tmp = zx;
  zx = xz;
  xz = tmp;
}


void Mat3::symmetrize(const Mat3 &M)
{
  xx = 2 * M.xx;
  yy = 2 * M.yy;
  zz = 2 * M.zz;
  xy = yx = M.xy + M.yx;
  yz = zy = M.yz + M.zy;
  zx = xz = M.zx + M.xz;
}


void Mat3::symmetrize()
{
  xx = 2 * xx;
  yy = 2 * yy;
  zz = 2 * zz;
  xy = yx = xy + yx;
  yz = zy = yz + zy;
  zx = xz = zx + xz;
}


void Mat3::negate(const Mat3 &M)
{
  xx = - M.xx;
  xy = - M.xy;
  xz = - M.xz;

  yx = - M.yx;
  yy = - M.yy;
  yz = - M.yz;

  zx = - M.zx;
  zy = - M.zy;
  zz = - M.zz;
}  


void Mat3::negate()
{
  xx = - xx;
  xy = - xy;
  xz = - xz;

  yx = - yx;
  yy = - yy;
  yz = - yz;

  zx = - zx;
  zy = - zy;
  zz = - zz;
}  


void Mat3::add(const Mat3 &M, const Mat3 &N)
{
  xx = M.xx + N.xx;
  xy = M.xy + N.xy;
  xz = M.xz + N.xz;

  yx = M.yx + N.yx;
  yy = M.yy + N.yy;
  yz = M.yz + N.yz;

  zx = M.zx + N.zx;
  zy = M.zy + N.zy;
  zz = M.zz + N.zz;
}


void Mat3::add(const Mat3 &M)
{
  xx += M.xx;
  xy += M.xy;
  xz += M.xz;

  yx += M.yx;
  yy += M.yy;
  yz += M.yz;

  zx += M.zx;
  zy += M.zy;
  zz += M.zz;
}


void Mat3::sub(const Mat3 &M, const Mat3 &N)
{
  xx = M.xx - N.xx;
  xy = M.xy - N.xy;
  xz = M.xz - N.xz;

  yx = M.yx - N.yx;
  yy = M.yy - N.yy;
  yz = M.yz - N.yz;

  zx = M.zx - N.zx;
  zy = M.zy - N.zy;
  zz = M.zz - N.zz;
}


void Mat3::sub(const Mat3 &M)
{
  xx -= M.xx;
  xy -= M.xy;
  xz -= M.xz;

  yx -= M.yx;
  yy -= M.yy;
  yz -= M.yz;

  zx -= M.zx;
  zy -= M.zy;
  zz -= M.zz;
}


void Mat3::scale(const Mat3 &M, Real s)
{
  xx = s * M.xx; 
  xy = s * M.xy; 
  xz = s * M.xz;
  yx = s * M.yx; 
  yy = s * M.yy; 
  yz = s * M.yz;
  zx = s * M.zx; 
  zy = s * M.zy; 
  zz = s * M.zz;
}


void Mat3::scale(Real s)
{
  xx *= s; 
  xy *= s; 
  xz *= s;
  yx *= s; 
  yy *= s; 
  yz *= s;
  zx *= s; 
  zy *= s; 
  zz *= s;
}


void Mat3::xform(const Vect3 &v, Vect3 &xv) const
{
  xv.x = xx * v.x + xy * v.y + xz * v.z;
  xv.y = yx * v.x + yy * v.y + yz * v.z;
  xv.z = zx * v.x + zy * v.y + zz * v.z;
}


void Mat3::xform(Vect3 &v) const
{
  Real ox, oy;

  ox = v.x; oy= v.y;
  v.x = xx * ox + xy * oy + xz * v.z;
  v.y = yx * ox + yy * oy + yz * v.z;
  v.z = zx * ox + zy * oy + zz * v.z;
}


void Mat3::invXform(const Vect3 &v, Vect3 &xv) const
{
  xv.x = xx * v.x + yx * v.y + zx * v.z;
  xv.y = xy * v.x + yy * v.y + zy * v.z;
  xv.z = xz * v.x + yz * v.y + zz * v.z;
}


void Mat3::invXform(Vect3 &v) const
{
  Real ox, oy;

  ox = v.x; oy= v.y;
  v.x = xx * ox + yx * oy + zx * v.z;
  v.y = xy * ox + yy * oy + zy * v.z;
  v.z = xz * ox + yz * oy + zz * v.z;
}


void MatX::set(const Se3 &T)
{
  R.set(T.q); 
  d = T.d;
}

ostream& MatX::print(ostream &os) const
{
  return os << R << d << endl;
}


istream& MatX::read(istream &is)
{
  Se3 T;
  is >> T;
  set(T);
  return is;
}

void MatX::xformVect(const Vect3 &v, Vect3 &xv) const
{
  R.xform(v, xv);
}

  
void MatX::xformVect(Vect3 &v) const
{
  R.xform(v);
}

  
void MatX::xformPoint(const Vect3 &p, Vect3 &xp) const
{
  R.xform(p, xp);
  xp.add(d);
}


void MatX::xformPoint(Vect3 &p) const
{
  R.xform(p);
  p.add(d);
}


void MatX::invXformVect(const Vect3 &v, Vect3 &xv) const
{
  R.invXform(v, xv);
}

  
void MatX::invXformVect(Vect3 &v) const
{
  R.invXform(v);
}

  
void MatX::invXformPoint(const Vect3 &p, Vect3 &xp) const
{
  xp.sub(p, d);
  R.invXform(xp);
}


void MatX::invXformPoint(Vect3 &p) const
{
  p.sub(d);
  R.invXform(p);
}


#if INVENTOR
void Quat::set(const SbRotation &rot)
{
  const float *q = rot.getValue();
  s_ = q[3]; x_ = q[0]; y_ = q[1]; z_ = q[2];
}
#endif


#if INVENTOR
void Quat::toSbRotation(SbRotation &rot) const
{
#if OPCOUNTS
  rot.setValue(x_.toDouble(), y_.toDouble(), z_.toDouble(), s_.toDouble());
#else
  rot.setValue(x_, y_, z_, s_);
#endif
}
#endif


Vect3 Quat::axis() const
{
  Vect3 v(x_, y_, z_);
  if (v.norm() == 0.0) v = Vect3::I;  // axis is arbitrary here
  else v.normalize();
  return v;
}


Real Quat::angle() const
{
#if OPCOUNTS
  return 2 * acos(s_.toDouble());
#else
  return 2 * acos(s_);
#endif
}


void Quat::normalize(const Quat &q)
{
  Real scale;

  scale = 1.0 / sqrt(q.s_*q.s_ + q.x_*q.x_ + q.y_*q.y_ + q.z_*q.z_);
  s_ = scale * q.s_;
  x_ = scale * q.x_;
  y_ = scale * q.y_;
  z_ = scale * q.z_;
}


void Quat::normalize()
{
  Real scale;

  scale = 1.0 / sqrt(s_*s_ + x_*x_ + y_*y_ + z_*z_);
  s_ *= scale;
  x_ *= scale;
  y_ *= scale;
  z_ *= scale;
}


void Quat::invert(const Quat &q)
{
  s_ = -q.s_;
  x_ =  q.x_;
  y_ =  q.y_;
  z_ =  q.z_;
}


void Quat::invert()
{
  s_ = -s_;
}


void Quat::deriv(const Quat &q, const Vect3 &w)
{
  s_ = 0.5 * (-q.x_ * w.x - q.y_ * w.y - q.z_ * w.z);
  x_ = 0.5 * ( q.s_ * w.x - q.z_ * w.y + q.y_ * w.z);
  y_ = 0.5 * ( q.z_ * w.x + q.s_ * w.y - q.x_ * w.z);
  z_ = 0.5 * (-q.y_ * w.x + q.x_ * w.y + q.s_ * w.z);
}


#if INVENTOR
void Se3::set(const SoTransform *xform)
{
  const float *quat;
  const float *trans;

  quat = xform->rotation.getValue().getValue();
  q.x_ = quat[0]; q.y_ = quat[1]; q.z_ = quat[2]; q.s_ = quat[3];

  trans = xform->translation.getValue().getValue();
  d.x = trans[0];
  d.y = trans[1];
  d.z = trans[2];
}
#endif


#if INVENTOR
void Se3::toSoTransform(SoTransform *xform) const
{
#if OPCOUNTS
  xform->rotation.setValue
    (q.x_.toDouble(), q.y_.toDouble(), q.z_.toDouble(), q.s_.toDouble());
  xform->translation.setValue(d.x.toDouble(), d.y.toDouble(), d.z.toDouble());
#else
  xform->rotation.setValue(q.x_, q.y_, q.z_, q.s_);
  xform->translation.setValue(d.x, d.y, d.z);
#endif
}
#endif
  

ostream& Se3::print(ostream &os) const
{
  return os << q << d;
}


void Se3::mult(const Se3 &T, const Se3 &U)
{
  q.mult(T.q, U.q);
  T.q.xform(U.d, d);
  d.add(d, T.d);
}


void Se3::premult(const Se3 &T)
{
  q.premult(T.q);
  T.q.xform(d);
  d.add(T.d);
}


void Se3::postmult(const Se3 &T)
{
  Vect3 v;

  q.xform(T.d, v);
  d.add(v);
  q.postmult(T.q);
}


void Se3::invert(const Se3 &T)
{
  q.s_ = -T.q.s_;
  q.x_ =  T.q.x_;
  q.y_ =  T.q.y_;
  q.z_ =  T.q.z_;
  q.xform(T.d, d);
  d.negate(d);
}


void Se3::invert()
{
  q.s_ = -q.s_;
  q.xform(d);
  d.negate();
}


void Se3::xformVect(const Vect3 &v, Vect3 &xv) const
{
  q.xform(v, xv);
}


void Se3::xformVect(Vect3 &v) const
{
  q.xform(v);
}


void Se3::xformPoint(const Vect3 &p, Vect3 &xp) const
{
  q.xform(p, xp);
  xp.add(d);
}


void Se3::xformPoint(Vect3 &p) const
{
  q.xform(p);
  p.add(d);
}


void Se3::invXformVect(const Vect3 &v, Vect3 &xv) const
{
  q.invXform(v, xv);
}


void Se3::invXformVect(Vect3 &v) const
{
  q.invXform(v);
}


void Se3::invXformPoint(const Vect3 &p, Vect3 &xp) const
{
  xp.sub(p, d);
  q.invXform(xp);
}


void Se3::invXformPoint(Vect3 &p) const
{
  p.sub(d);
  q.invXform(p);
}


};
#endif   // #ifndef MV_H


