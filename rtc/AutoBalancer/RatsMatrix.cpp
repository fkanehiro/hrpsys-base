#include "RatsMatrix.h"

namespace rats
{
  hrp::Vector3 matrix_log(const hrp::Matrix33& m) {
    hrp::Vector3 mlog;
    double q0, th;
    hrp::Vector3 q;
    double norm;
  
    Eigen::Quaternion<double> eiq(m);
    q0 = eiq.w();
    q = eiq.vec();
    norm = q.norm();
    if (norm > 0) {
      if ((q0 > 1.0e-10) || (q0 < -1.0e-10)) {
        th = 2 * std::atan(norm / q0);
      } else if (q0 > 0) {
        th = M_PI / 2;
      } else {
        th = -M_PI / 2;
      }
      mlog = (th / norm) * q ;
    } else {
      mlog = hrp::Vector3::Zero();
    }
    return mlog;
  }
  void rotation_matrix(hrp::Matrix33& rm, const double theta, const hrp::Vector3& axis) {
    double cs, sn, vers, xv, yv, zv, xyv, yzv, zxv, xs, ys, zs;
    hrp::Vector3 a;

    (cs = std::cos(theta));
    (sn = std::sin(theta));
    (vers = (1 - cs));
    if (axis.norm() > 0) {
      a = axis.normalized();
    } else {
      a = hrp::Vector3(0,0,0);
    }
    (xv = ((a(0) * a(0)) * vers));
    (yv = ((a(1) * a(1)) * vers));
    (zv = ((a(2) * a(2)) * vers));
    (xyv = ((a(0) * a(1)) * vers));
    (yzv = ((a(1) * a(2)) * vers));
    (zxv = ((a(2) * a(0)) * vers));
    (xs = (a(0) * sn));
    (ys = (a(1) * sn));
    (zs = (a(2) * sn));
    (rm(0, 0) = (xv + cs));
    (rm(0, 1) = (xyv - zs));
    (rm(0, 2) = (zxv + ys));
    (rm(1, 0) = (xyv + zs));
    (rm(1, 1) = (yv + cs));
    (rm(1, 2) = (yzv - xs));
    (rm(2, 0) = (zxv - ys));
    (rm(2, 1) = (yzv + xs));
    (rm(2, 2) = (zv + cs));
  }

  void print_vector(std::ostream& strm, const hrp::Vector3& vec, const bool use_newline)
  {
    strm << "#f(";
    for (size_t i = 0; i < 3; i++) strm << vec(i) << " ";
    strm << ")";
    if (use_newline) strm << std::endl;
  }

  void print_matrix(std::ostream& strm, const hrp::Matrix33& mat, const bool use_newline)
  {
    strm << "#2f(";
    for (std::size_t i = 0; i < 3; ++i) {
      strm << "(";
      for (std::size_t j = 0; j < 3; ++j) {
        strm << mat(i,j) << " ";
      }
      strm << ")";
    }
    strm << ")";

    if (use_newline) strm << std::endl;
  }
  // matrix product using quaternion normalization
  void rotm3times (hrp::Matrix33& m12, const hrp::Matrix33& m1, const hrp::Matrix33& m2) {
    Eigen::Quaternion<double> eiq1(m1);
    Eigen::Quaternion<double> eiq2(m2);
    Eigen::Quaternion<double> eiq3;
    eiq3 = eiq1 * eiq2;
    eiq3.normalize();
    m12 = eiq3.toRotationMatrix();
  }

  void difference_rotation(hrp::Vector3& ret_dif_rot, const hrp::Matrix33& self_rot, const hrp::Matrix33& target_rot)
  {
    //ret_dif_rot = self_rot * hrp::omegaFromRot(self_rot.transpose() * target_rot);
    ret_dif_rot = self_rot * hrp::Vector3(rats::matrix_log(hrp::Matrix33(self_rot.transpose() * target_rot)));
  }

  // /* implementation of template functions */
  // void print_vector(std::ostream& strm, const hrp::Vector3& v, const bool use_newline = true) {
  //   print_vector(strm, v.data(), use_newline);
  // }

  // void print_matrix(std::ostream& strm, const hrp::Matrix33& m, const bool use_newline = true) {
  //   print_matrix(strm, m.data(), 3, 3, use_newline);
  // }
  void outer_product_matrix(hrp::Matrix33 &ret, const hrp::Vector3 &v) {
    (ret(0, 0) = 0);
    (ret(0, 1) = (-v(2)));
    (ret(0, 2) = v(1));
    (ret(1, 0) = v(2));
    (ret(1, 1) = 0);
    (ret(1, 2) = (-v(0)));
    (ret(2, 0) = (-v(1)));
    (ret(2, 1) = v(0));
    (ret(2, 2) = 0);
  }

  void matrix_exponent(hrp::Matrix33& mexp, const hrp::Vector3& omega, double p) {
    double w;
    hrp::Vector3 a(omega);
    hrp::Matrix33 ahat;
  
    w = omega.norm();
    if (eps_eq(w, 0)) {
      mexp = hrp::Matrix33::Identity();
      return;
    }
    a.normalize();
    outer_product_matrix(ahat, a);
    //cout << "omega = " << omega << "w = " << w << " a = " << a << "a^ = " << ahat << endl; 
    mexp = hrp::Matrix33::Identity() + std::sin(w * p) * ahat + (1 - std::cos(w * p)) * (ahat * ahat);
  }

  void mid_coords(coordinates& mid_coords, const double p, const coordinates& c1, const coordinates& c2) {
    hrp::Vector3 mid_point, omega;
    hrp::Matrix33 mid_rot, r;
  
    mid_point = (1 - p) * c1.pos + p * c2.pos;
    r = c1.rot.transpose() * c2.rot;
    omega = matrix_log(r);
    matrix_exponent(r, omega, p);
    //mid_rot = c1.rot * r;
    rotm3times(mid_rot, c1.rot, r);
    mid_coords = coordinates(mid_point, mid_rot);
  };

}
