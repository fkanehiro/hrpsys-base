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

  void mid_rot(hrp::Matrix33& mid_rot, const double p, const hrp::Matrix33& rot1, const hrp::Matrix33& rot2, const double eps) {
    hrp::Matrix33 r(rot1.transpose() * rot2);
    hrp::Vector3 omega(matrix_log(r));
    if (eps_eq(omega.norm(),0.0, eps)) { // c1.rot and c2.rot are same
      mid_rot = rot1;
    } else {
      hrp::calcRodrigues(r, omega.normalized(), omega.norm()*p);
      //mid_rot = c1.rot * r;
      rotm3times(mid_rot, rot1, r);
    }
  };

  void mid_coords(coordinates& mid_coords, const double p, const coordinates& c1, const coordinates& c2, const double eps) {
    mid_coords.pos = (1 - p) * c1.pos + p * c2.pos;
    mid_rot(mid_coords.rot, p, c1.rot, c2.rot, eps);
  };

}
