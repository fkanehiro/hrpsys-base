#include <stdlib.h>
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



#include <math.h>
#include <sstream>
#include <string.h>
#include "mv.h"

using namespace Vclip;

///////////////////////////////////////////////////////////////////////////////
//  constants
///////////////////////////////////////////////////////////////////////////////


static const double DEG_TO_RAD = M_PI / 180.0;

#define STR_LENGTH 1000  // for i/o operations

const Vect3 Vect3::ZERO(0, 0, 0);
const Vect3 Vect3::I   (1, 0, 0);
const Vect3 Vect3::J   (0, 1, 0);
const Vect3 Vect3::K   (0, 0, 1);
const Vect3 Vect3::I_  (-1,  0,  0);
const Vect3 Vect3::J_  ( 0, -1,  0);
const Vect3 Vect3::K_  ( 0,  0, -1);

const Mat3 Mat3::ZERO (Vect3::ZERO, Vect3::ZERO);
const Mat3 Mat3::ID   (Vect3(1, 1, 1), Vect3::ZERO);

const MatX MatX::ID   (Mat3::ID, Vect3::ZERO);

const Quat Quat::ID   (1, 0, 0, 0);

const Se3  Se3::ID    (Quat::ID, Vect3::ZERO);


///////////////////////////////////////////////////////////////////////////////
//
//  class Vect3
//
///////////////////////////////////////////////////////////////////////////////


ostream& Vect3::print(ostream &os) const
{
  int oldFlags = os.setf(ios::showpos);
  os << '(' << x << ' ' << y << ' ' << z << ')';
  os.flags((std::ios_base::fmtflags)oldFlags);
  return os;
}


istream& Vect3::read(istream &is)
{
  char tok[STR_LENGTH];
  char *code;

  is >> tok;

  if (tok[0] == '+' || tok[0] == '-') code = tok+1;
  else code = tok;

  if (*code >= 'i' && *code <= 'k' && *(code+1) == '\0') {
    switch (*code) {
    case 'i': *this = Vect3::I; break;
    case 'j': *this = Vect3::J; break;
    case 'k': *this = Vect3::K; break;
    default: break;
    }
    if (tok[0] == '-') negate();
    return is;
  }

  x = (Real) atof(tok);
  return (is >> y >> z);
}


///////////////////////////////////////////////////////////////////////////////
//
//  class Mat3
//
///////////////////////////////////////////////////////////////////////////////


void Mat3::set(const Quat &q)
{
  xx = 2.0 * (q.s_ * q.s_ + q.x_ * q.x_ - 0.5);
  yy = 2.0 * (q.s_ * q.s_ + q.y_ * q.y_ - 0.5);
  zz = 2.0 * (q.s_ * q.s_ + q.z_ * q.z_ - 0.5);

  xy = 2.0 * (q.y_ * q.x_ - q.z_ * q.s_);
  yx = 2.0 * (q.x_ * q.y_ + q.z_ * q.s_);


  yz = 2.0 * (q.z_ * q.y_ - q.x_ * q.s_);
  zy = 2.0 * (q.y_ * q.z_ + q.x_ * q.s_);

  zx = 2.0 * (q.x_ * q.z_ - q.y_ * q.s_);
  xz = 2.0 * (q.z_ * q.x_ + q.y_ * q.s_);
}


ostream& Mat3::print(ostream &os) const
{
  int oldFlags = os.setf(ios::showpos);
  os << '[' << xx << ' ' << xy << ' ' << xz << ']' << endl;
  os << '[' << yx << ' ' << yy << ' ' << yz << ']' << endl;
  os << '[' << zx << ' ' << zy << ' ' << zz << ']' << endl;
  os.flags((std::ios_base::fmtflags)oldFlags);
  return os;
}


int Mat3::invert(const Mat3 &M)
{
  Real D, oneOverDet;

  if (fabs(D = M.det()) < 1.0e-12) return 1; // not invertible
  oneOverDet = 1 / D;

  xx = (M.yy * M.zz - M.yz * M.zy) * oneOverDet;
  xy = (M.xz * M.zy - M.xy * M.zz) * oneOverDet;
  xz = (M.xy * M.yz - M.xz * M.yy) * oneOverDet;
  yx = (M.yz * M.zx - M.yx * M.zz) * oneOverDet;
  yy = (M.xx * M.zz - M.xz * M.zx) * oneOverDet;
  yz = (M.xz * M.yx - M.xx * M.yz) * oneOverDet;
  zx = (M.yx * M.zy - M.yy * M.zx) * oneOverDet;
  zy = (M.xy * M.zx - M.xx * M.zy) * oneOverDet;
  zz = (M.xx * M.yy - M.xy * M.yx) * oneOverDet;
  return 0;
}


int Mat3::invert()
{
  Real D, oneOverDet;
  Real oxy, oyz, ozx, oyx, ozy, oxz;

  if (fabs(D = det()) < 1.0e-12) return 1; // not invertible
  oneOverDet = 1 / D;

  oxy = xy; oyx = yx;
  oyz = yz; ozy = zy;
  ozx = zx; oxz = xz;

  xy = (oxz * ozy - zz * oxy) * oneOverDet;
  yz = (oxz * oyx - xx * oyz) * oneOverDet;
  zx = (oyx * ozy - yy * ozx) * oneOverDet;
  yx = (oyz * ozx - oyx * zz) * oneOverDet;
  zy = (oxy * ozx - ozy * xx) * oneOverDet;
  xz = (oxy * oyz - oxz * yy) * oneOverDet;
  xx = (yy * zz - oyz * ozy) * oneOverDet;
  yy = (xx * zz - oxz * ozx) * oneOverDet;
  zz = (xx * yy - oxy * oyx) * oneOverDet;
  return 0;
}


void Mat3::mult(const Mat3 &M, const Mat3 &N)
{
  xx = M.xx * N.xx + M.xy * N.yx + M.xz * N.zx;
  xy = M.xx * N.xy + M.xy * N.yy + M.xz * N.zy;
  xz = M.xx * N.xz + M.xy * N.yz + M.xz * N.zz;
  yx = M.yx * N.xx + M.yy * N.yx + M.yz * N.zx;
  yy = M.yx * N.xy + M.yy * N.yy + M.yz * N.zy;
  yz = M.yx * N.xz + M.yy * N.yz + M.yz * N.zz;
  zx = M.zx * N.xx + M.zy * N.yx + M.zz * N.zx;
  zy = M.zx * N.xy + M.zy * N.yy + M.zz * N.zy;
  zz = M.zx * N.xz + M.zy * N.yz + M.zz * N.zz;
}


void Mat3::premult(const Mat3 &M)
{
  Real oxy, oyz, ozx, oyx, ozy, oxz;

  oxy = xy; oyx = yx; oyz = yz; ozy = zy; ozx = zx; oxz = xz;

  xy = M.xx * oxy + M.xy * yy  + M.xz * ozy;
  xz = M.xx * oxz + M.xy * oyz + M.xz * zz;
  yx = M.yx * xx  + M.yy * oyx + M.yz * ozx;
  yz = M.yx * oxz + M.yy * oyz + M.yz * zz;
  zx = M.zx * xx  + M.zy * oyx + M.zz * ozx;
  zy = M.zx * oxy + M.zy * yy  + M.zz * ozy;

  xx = M.xx * xx  + M.xy * oyx + M.xz * ozx;
  yy = M.yx * oxy + M.yy * yy  + M.yz * ozy;
  zz = M.zx * oxz + M.zy * oyz + M.zz * zz;
}


void Mat3::postmult(const Mat3 &M)
{
  Real oxy, oyz, ozx, oyx, ozy, oxz;

  oxy = xy; oyx = yx; oyz = yz; ozy = zy; ozx = zx; oxz = xz;

  xy = xx *  M.xy + oxy * M.yy + oxz * M.zy;
  xz = xx *  M.xz + oxy * M.yz + oxz * M.zz;
  yx = oyx * M.xx + yy  * M.yx + oyz * M.zx;
  yz = oyx * M.xz + yy  * M.yz + oyz * M.zz;
  zx = ozx * M.xx + ozy * M.yx + zz  * M.zx;
  zy = ozx * M.xy + ozy * M.yy + zz  * M.zy;

  xx = xx  * M.xx + oxy * M.yx + oxz * M.zx;
  yy = oyx * M.xy + yy  * M.yy + oyz * M.zy;
  zz = ozx * M.xz + ozy * M.yz + zz  * M.zz;
}


///////////////////////////////////////////////////////////////////////////////
//
//  class MatX
//
///////////////////////////////////////////////////////////////////////////////


void MatX::mult(const MatX &M, const MatX &N)
{
  // multiply rotation matrices
  R.xx = M.R.xx * N.R.xx + M.R.xy * N.R.yx + M.R.xz * N.R.zx;
  R.xy = M.R.xx * N.R.xy + M.R.xy * N.R.yy + M.R.xz * N.R.zy;
  R.xz = M.R.xx * N.R.xz + M.R.xy * N.R.yz + M.R.xz * N.R.zz;
  R.yx = M.R.yx * N.R.xx + M.R.yy * N.R.yx + M.R.yz * N.R.zx;
  R.yy = M.R.yx * N.R.xy + M.R.yy * N.R.yy + M.R.yz * N.R.zy;
  R.yz = M.R.yx * N.R.xz + M.R.yy * N.R.yz + M.R.yz * N.R.zz;
  R.zx = M.R.zx * N.R.xx + M.R.zy * N.R.yx + M.R.zz * N.R.zx;
  R.zy = M.R.zx * N.R.xy + M.R.zy * N.R.yy + M.R.zz * N.R.zy;
  R.zz = M.R.zx * N.R.xz + M.R.zy * N.R.yz + M.R.zz * N.R.zz;

  // d = M.R * N.d + M.d
  d.x = M.R.xx * N.d.x + M.R.xy * N.d.y + M.R.xz * N.d.z + M.d.x;
  d.y = M.R.yx * N.d.x + M.R.yy * N.d.y + M.R.yz * N.d.z + M.d.y;
  d.z = M.R.zx * N.d.x + M.R.zy * N.d.y + M.R.zz * N.d.z + M.d.z;
}


void MatX::premult(const MatX &M)
{
  Real oxy, oyz, ozx, oyx, ozy, oxz, odx, ody;


  // multiply rotation matrices
  oxy = R.xy; oyx = R.yx; oyz = R.yz; ozy = R.zy; ozx = R.zx; oxz = R.xz;

  R.xy = M.R.xx * oxy  + M.R.xy * R.yy + M.R.xz * ozy;
  R.xz = M.R.xx * oxz  + M.R.xy * oyz  + M.R.xz * R.zz;
  R.yx = M.R.yx * R.xx + M.R.yy * oyx  + M.R.yz * ozx;
  R.yz = M.R.yx * oxz  + M.R.yy * oyz  + M.R.yz * R.zz;
  R.zx = M.R.zx * R.xx + M.R.zy * oyx  + M.R.zz * ozx;
  R.zy = M.R.zx * oxy  + M.R.zy * R.yy + M.R.zz * ozy;

  R.xx = M.R.xx * R.xx + M.R.xy * oyx  + M.R.xz * ozx;
  R.yy = M.R.yx * oxy  + M.R.yy * R.yy + M.R.yz * ozy;
  R.zz = M.R.zx * oxz  + M.R.zy * oyz  + M.R.zz * R.zz;

  // d = M.R * d + M.d
  odx = d.x; ody = d.y;
  d.x = M.R.xx * odx + M.R.xy * ody + M.R.xz * d.z + M.d.x;
  d.y = M.R.yx * odx + M.R.yy * ody + M.R.yz * d.z + M.d.y;
  d.z = M.R.zx * odx + M.R.zy * ody + M.R.zz * d.z + M.d.z;
}


void MatX::postmult(const MatX &M)
{
  Real oxy, oyz, ozx, oyx, ozy, oxz;
  Vect3 v;

  // d = R * M.d + d
  d.x += R.xx * M.d.x + R.xy * M.d.y + R.xz * M.d.z;
  d.y += R.yx * M.d.x + R.yy * M.d.y + R.yz * M.d.z;
  d.z += R.zx * M.d.x + R.zy * M.d.y + R.zz * M.d.z;

  // multiply rotation matrices
  oxy = R.xy; oyx = R.yx; oyz = R.yz; ozy = R.zy; ozx = R.zx; oxz = R.xz;
  R.xy = R.xx * M.R.xy + oxy  * M.R.yy + oxz  * M.R.zy;
  R.xz = R.xx * M.R.xz + oxy  * M.R.yz + oxz  * M.R.zz;
  R.yx = oyx  * M.R.xx + R.yy * M.R.yx + oyz  * M.R.zx;
  R.yz = oyx  * M.R.xz + R.yy * M.R.yz + oyz  * M.R.zz;
  R.zx = ozx  * M.R.xx + ozy  * M.R.yx + R.zz * M.R.zx;
  R.zy = ozx  * M.R.xy + ozy  * M.R.yy + R.zz * M.R.zy;

  R.xx = R.xx * M.R.xx + oxy  * M.R.yx + oxz  * M.R.zx;
  R.yy = oyx  * M.R.xy + R.yy * M.R.yy + oyz  * M.R.zy;
  R.zz = ozx  * M.R.xz + ozy  * M.R.yz + R.zz * M.R.zz;
}


void MatX::invert(const MatX &M)
{
  // invert the rotation part by transposing it
  R.xx = M.R.xx;
  R.xy = M.R.yx;
  R.xz = M.R.zx;
  R.yx = M.R.xy;
  R.yy = M.R.yy;
  R.yz = M.R.zy;
  R.zx = M.R.xz;
  R.zy = M.R.yz;
  R.zz = M.R.zz;

  // new displacement vector given by:  d' = -(R^-1) * d 
  d.x = - (R.xx * M.d.x + R.xy * M.d.y + R.xz * M.d.z);
  d.y = - (R.yx * M.d.x + R.yy * M.d.y + R.yz * M.d.z);
  d.z = - (R.zx * M.d.x + R.zy * M.d.y + R.zz * M.d.z);
}



void MatX::invert()
{
  Real tmp, odx, ody;

  // invert the rotation part by transposing it
  tmp  = R.xy;
  R.xy = R.yx;
  R.yx = tmp;

  tmp  = R.yz;
  R.yz = R.zy;
  R.zy = tmp;

  tmp  = R.zx;
  R.zx = R.xz;
  R.xz = tmp;

  // new displacement vector given by:  d' = -(R^T) * d 
  odx = d.x; ody = d.y;
  d.x = - (R.xx * odx + R.xy * ody + R.xz * d.z);
  d.y = - (R.yx * odx + R.yy * ody + R.yz * d.z);
  d.z = - (R.zx * odx + R.zy * ody + R.zz * d.z);
}



///////////////////////////////////////////////////////////////////////////////
//
//  class Quat
//
///////////////////////////////////////////////////////////////////////////////


void Quat::set(Real angle, const Vect3 &axis, int normalizeAxis)
{
  Vect3 axis0;
  Real theta, sine;

  theta = 0.5 * angle;
  sine = sin(theta);
  s_ = cos(theta);

  if (normalizeAxis) {
    axis0.normalize(axis);
    x_ = axis0.x * sine;
    y_ = axis0.y * sine;
    z_ = axis0.z * sine;
  }
  else {
    x_ = axis.x * sine;
    y_ = axis.y * sine;
    z_ = axis.z * sine;
  }

}

  
void Quat::set(const Mat3 &R)
{
  Real qs2, qx2, qy2, qz2;  // squared magniudes of quaternion components
  Real tmp;
  int i;

  // first compute squared magnitudes of quaternion components - at least one
  // will be greater than 0 since quaternion is unit magnitude

  qs2 = 0.25 * (R.xx + R.yy + R.zz + 1);
  qx2 = qs2 - 0.5 * (R.yy + R.zz);
  qy2 = qs2 - 0.5 * (R.zz + R.xx);
  qz2 = qs2 - 0.5 * (R.xx + R.yy);

  
  // find maximum magnitude component
  i = (qs2 > qx2 ) ?
    ((qs2 > qy2) ? ((qs2 > qz2) ? 0 : 3) : ((qy2 > qz2) ? 2 : 3)) :
    ((qx2 > qy2) ? ((qx2 > qz2) ? 1 : 3) : ((qy2 > qz2) ? 2 : 3));

  // compute signed quaternion components using numerically stable method
  switch(i) {
  case 0:
    s_ = sqrt(qs2);
    tmp = 0.25 / s_;
    x_ = (R.zy - R.yz) * tmp;
    y_ = (R.xz - R.zx) * tmp;
    z_ = (R.yx - R.xy) * tmp;
    break;
  case 1:
    x_ = sqrt(qx2);
    tmp = 0.25 / x_;
    s_ = (R.zy - R.yz) * tmp;
    y_ = (R.xy + R.yx) * tmp;
    z_ = (R.xz + R.zx) * tmp;
    break;
  case 2:
    y_ = sqrt(qy2);
    tmp = 0.25 / y_;
    s_ = (R.xz - R.zx) * tmp;
    z_ = (R.yz + R.zy) * tmp;
    x_ = (R.yx + R.xy) * tmp;
    break;
  case 3:
    z_ = sqrt(qz2);
    tmp = 0.25 / z_;
    s_ = (R.yx - R.xy) * tmp;
    x_ = (R.zx + R.xz) * tmp;
    y_ = (R.zy + R.yz) * tmp;
    break;
  }
  // for consistency, force positive scalar component [ (s; v) = (-s; -v) ]
  if (s_ < 0) {
    s_ = -s_;
    x_ = -x_;
    y_ = -y_;
    z_ = -z_;
  }
  // normalize, just to be safe
  tmp = 1.0 / sqrt(s_*s_ + x_*x_ + y_*y_ + z_*z_);
  s_ *= tmp;
  x_ *= tmp;
  y_ *= tmp;
  z_ *= tmp;
}


ostream& Quat::print(ostream &os) const
{
  int oldFlags = os.setf(ios::showpos);
  os << '(' << s_ << ' ' << x_ << ' ' << y_ << ' ' << z_ << ')';
  os.flags((std::ios_base::fmtflags)oldFlags);
  return os;
}


void Quat::mult(const Quat &p, const Quat &q)
{
  s_ = p.s_ * q.s_ - (p.x_ * q.x_ + p.y_ * q.y_ + p.z_ * q.z_);
  x_ = p.s_ * q.x_ +  q.s_ * p.x_ + p.y_ * q.z_ - p.z_ * q.y_;
  y_ = p.s_ * q.y_ +  q.s_ * p.y_ + p.z_ * q.x_ - p.x_ * q.z_;
  z_ = p.s_ * q.z_ +  q.s_ * p.z_ + p.x_ * q.y_ - p.y_ * q.x_;
}


void Quat::premult(const Quat &q)
{
  Real ox, oy, oz;

  ox = x_; oy = y_; oz = z_;

  x_ = q.s_ * ox +  s_ * q.x_ + q.y_ * oz - q.z_ * oy;
  y_ = q.s_ * oy +  s_ * q.y_ + q.z_ * ox - q.x_ * oz;
  z_ = q.s_ * oz +  s_ * q.z_ + q.x_ * oy - q.y_ * ox;
  s_ = q.s_ * s_ - (q.x_ * ox + q.y_ * oy + q.z_ * oz);
}


void Quat::postmult(const Quat &q)
{
  Real ox, oy, oz;

  ox = x_; oy = y_; oz = z_;

  x_ = s_ * q.x_ +  ox * q.s_ + oy * q.z_ - oz * q.y_;
  y_ = s_ * q.y_ +  oy * q.s_ + oz * q.x_ - ox * q.z_;
  z_ = s_ * q.z_ +  oz * q.s_ + ox * q.y_ - oy * q.x_;
  s_ = s_ * q.s_ - (ox * q.x_ + oy * q.y_ + oz * q.z_);
}


// The Quat transformation routines use 19 multiplies and 12 adds
// (counting the multiplications by 2.0).  See Eqn (20) of "A
// Comparison of Transforms and Quaternions in Robotics," Funda and
// Paul, Proceedings of International Conference on Robotics and
// Automation, 1988, p. 886-991.

void Quat::xform(const Vect3 &v, Vect3 &xv) const
{
  Vect3 *u, uv, uuv;


  u = (Vect3 *) &x_;
  uv.cross(*u, v);
  uuv.cross(*u, uv);
  uv.scale(2.0 * s_);
  uuv.scale(2.0);
  xv.add(v, uv);
  xv.add(uuv);
}


void Quat::xform(Vect3 &v) const
{
  Vect3 *u, uv, uuv;

  u = (Vect3 *) &x_;
  uv.cross(*u, v);
  uuv.cross(*u, uv);
  uv.scale(2.0 * s_);
  uuv.scale(2.0);
  v.add(uv);
  v.add(uuv);
}


void Quat::invXform(const Vect3 &v, Vect3 &xv) const
{
  Vect3 *u, uv, uuv;
  
  u = (Vect3 *) &x_;
  uv.cross(*u, v);
  uuv.cross(*u, uv);
  uv.scale(2.0 * -s_);
  uuv.scale(2.0);
  xv.add(v, uv);
  xv.add(uuv);
}


void Quat::invXform(Vect3 &v) const
{
  Vect3 *u, uv, uuv;
  
  u = (Vect3 *) &x_;
  uv.cross(*u, v);
  uuv.cross(*u, uv);
  uv.scale(2.0 * -s_);
  uuv.scale(2.0);
  v.add(uv);
  v.add(uuv);
}



///////////////////////////////////////////////////////////////////////////////
//
//  class Se3
//
///////////////////////////////////////////////////////////////////////////////


istream& Se3::read(istream &is)
{
  char c;
  int i;
  Real x;
  Vect3 vect;
  Quat quat;
  Se3 op;
  char buffer[STR_LENGTH];
  char tok[STR_LENGTH];
  
  *this = Se3::ID;

  is >> ws;
  if (is.peek() == '{') is.get(c);
  else {
    cerr << "Se3::read : didn't find '{' \a" << endl;
    return is;
  }
    
  // read until closing '}'
  i = 0;
  while (1) {
    is.get(c);
    if (c == '}') {
      buffer[i++] = 0;
      break;
    }
    buffer[i++] = c;
    if (i == STR_LENGTH || is.eof()) {
      cerr << "Se3::read : didn't find '}' or specification too long\a" << endl;
      return is;
    }
  }
  std::istringstream iss(buffer);

  
  while (!((iss >> tok).fail())) {

    if (!strcmp(tok, "trans")) {
      iss >> vect;
      op.set(Quat::ID, vect);
    }
    else if (!strcmp(tok, "rot")) {
      iss >> tok >> vect;
      x = atof(tok) * DEG_TO_RAD;
      quat.set(x, vect);
      op.set(quat, Vect3::ZERO);
    }
    else {
      cerr << "Se3::read : unknown token " << tok << " \a" << endl;
      break;
    }

    postmult(op);
    q.normalize();  // just to be sure.
  }

  return is;

}


