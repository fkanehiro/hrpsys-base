// -*- coding:utf-8-unix; mode: c++; indent-tabs-mode: nil; c-basic-offset: 2; -*-
#ifndef EKFILTER_H
#define EKFILTER_H

#include <hrpUtil/EigenTypes.h>
#include <iostream>
#include "util/Hrpsys.h"

class EKFilter {
public:
  EKFilter()
    : P(Eigen::Matrix<double, 7, 7>::Identity() * 0.1),
      Q(Eigen::Matrix3d::Identity() * 0.001),
      R(Eigen::Matrix3d::Identity() * 0.03),
      g_vec(Eigen::Vector3d(0.0, 0.0, 9.80665))
  {
    x << 1, 0, 0, 0, 0, 0, 0;
  }

  Eigen::Matrix<double, 7, 1> getx() const { return x; }

  void calcOmega(Eigen::Matrix<double, 4, 4>& omega, const Eigen::Vector3d& w) const {
    /* \dot{q} = \frac{1}{2} omega q */
    omega <<
      0, -w[0], -w[1], -w[2],
      w[0],     0,  w[2], -w[1],
      w[1], -w[2],     0,  w[0],
      w[2],  w[1], -w[0],     0;
  }

  void calcPredictedState(Eigen::Matrix<double, 7, 1>& _x_a_priori,
                          const Eigen::Matrix<double, 4, 1>& q,
                          const Eigen::Vector3d& gyro,
                          const Eigen::Vector3d& drift) const {
    /* x_a_priori = f(x, u) */
    Eigen::Matrix<double, 4, 1> q_a_priori;
    Eigen::Vector3d gyro_compensated = gyro - drift;
    Eigen::Matrix<double, 4, 4> omega;
    calcOmega(omega, gyro_compensated);
    q_a_priori = q + dt / 2 * omega * q;
    _x_a_priori.block<4, 1>(0, 0) = q_a_priori.normalized();
    _x_a_priori.block<3, 1>(4, 0) = drift;
  }

  void calcF(Eigen::Matrix<double, 7, 7>& F,
             const Eigen::Matrix<double, 4, 1>& q,
             const Eigen::Vector3d& gyro,
             const Eigen::Vector3d& drift) const {
    F = Eigen::Matrix<double, 7, 7>::Identity();
    Eigen::Vector3d gyro_compensated = gyro - drift;
    Eigen::Matrix<double, 4, 4> omega;
    calcOmega(omega, gyro_compensated);
    F.block<4, 4>(0, 0) += dt / 2 * omega;
    F.block<4, 3>(0, 4) <<
      + dt / 2 * q[1], + dt / 2 * q[2], + dt / 2 * q[3],
      - dt / 2 * q[0], + dt / 2 * q[3], - dt / 2 * q[2],
      - dt / 2 * q[3], - dt / 2 * q[0], + dt / 2 * q[1],
      + dt / 2 * q[2], - dt / 2 * q[1], - dt / 2 * q[0];
  }

  void calcPredictedCovariance(Eigen::Matrix<double, 7, 7>& _P_a_priori,
                               const Eigen::Matrix<double, 7, 7>& F,
                               const Eigen::Matrix<double, 4, 1>& q) const {
    /* P_a_priori = F P F^T + V Q V^T */
    Eigen::Matrix<double, 4, 3> V_upper;
    V_upper <<
      - dt / 2 * q[1], - dt / 2 * q[2], - dt / 2 * q[3],
      + dt / 2 * q[0], - dt / 2 * q[3], + dt / 2 * q[2],
      + dt / 2 * q[3], + dt / 2 * q[0], - dt / 2 * q[1],
      - dt / 2 * q[2], + dt / 2 * q[1], + dt / 2 * q[0];
    Eigen::Matrix<double, 7, 7> VQVt = Eigen::Matrix<double, 7, 7>::Zero();
    VQVt.block<4, 4>(0, 0) = V_upper * Q * V_upper.transpose();
    _P_a_priori = F * P * F.transpose() + VQVt;
  }

  Eigen::Vector3d calcAcc(const Eigen::Matrix<double, 4, 1>& q) const {
    Eigen::Quaternion<double> q_tmp = Eigen::Quaternion<double>(q[0], q[1], q[2], q[3]);
    Eigen::Vector3d acc = q_tmp.conjugate()._transformVector(g_vec);
    return acc;
  }

  void calcH(Eigen::Matrix<double, 3, 7>& H, const Eigen::Matrix<double, 4, 1>& q) const {
    double w = q[0], x = q[1], y = q[2], z = q[3];
    H <<
      -y, +z, -w, +x, 0, 0, 0,
      +x, +w, +z, +y, 0, 0, 0,
      +w, -x, -y, +z, 0, 0, 0;
    H *= 2 * g_vec[2];
  }

  Eigen::Vector3d calcMeasurementResidual(const Eigen::Vector3d& acc_measured,
                                          const Eigen::Matrix<double, 4, 1>& q) const {
    /* y = z - h(x) */
    Eigen::Vector3d y = acc_measured - calcAcc(q);
    return y;
  }


  void prediction(const Eigen::Vector3d& u) {
    Eigen::Matrix<double, 4, 1> q = x.block<4, 1>(0, 0);
    Eigen::Vector3d drift = x.block<3, 1>(4, 0);
    Eigen::Matrix<double, 7, 7> F;
    calcF(F, q, u, drift);
    Eigen::Matrix<double, 7, 1> x_tmp;
    calcPredictedState(x_tmp, q, u, drift);
    x_a_priori = x_tmp;
    Eigen::Matrix<double, 7, 7> P_tmp;
    calcPredictedCovariance(P_tmp, F, q);
    P_a_priori = P_tmp;
  }

  void correction(const Eigen::Vector3d& z) {
    Eigen::Matrix<double, 4, 1> q_a_priori = x_a_priori.block<4, 1>(0, 0);
    Eigen::Matrix<double, 3, 7> H;
    Eigen::Vector3d y = calcMeasurementResidual(z, q_a_priori);
    calcH(H, q_a_priori);
    Eigen::Matrix<double, 3, 3> S = H * P_a_priori * H.transpose() + R;
    Eigen::Matrix<double, 7, 3> K = P_a_priori * H.transpose() * S.inverse();
    Eigen::Matrix<double, 7, 1> x_tmp = x_a_priori + K * y;
    x.block<4, 1>(0, 0) = x_tmp.block<4, 1>(0, 0).normalized(); /* quaternion */
    x.block<3, 1>(4, 0) = x_tmp.block<3, 1>(4, 0); /* bias */
    P = (Eigen::Matrix<double, 7, 7>::Identity() - K * H) * P_a_priori;
  }

  void printAll() const {
    std::cerr << "x" << std::endl << x << std::endl;
    std::cerr << "x_a_priori" << std::endl << x_a_priori << std::endl;
    std::cerr << "P" << std::endl << P << std::endl << std::endl;
    std::cerr << "P_a_priori" << std::endl << P_a_priori << std::endl << std::endl;
  }

  void main_one (hrp::Vector3& rpy, hrp::Vector3& rpyRaw, const hrp::Vector3& acc, const hrp::Vector3& gyro)
  {
    prediction(gyro);
    correction(acc);
    /* ekf_filter.printAll(); */
    Eigen::Quaternion<double> q = Eigen::Quaternion<double>(x[0], x[1], x[2], x[3]);
    rpy = hrp::rpyFromRot(q.toRotationMatrix());
  };

  void setdt (const double _dt) { dt = _dt;};
private:
  Eigen::Matrix<double, 7, 1> x, x_a_priori;
  Eigen::Matrix<double, 7, 7> P, P_a_priori;
  Eigen::Matrix<double, 3, 3> Q, R;
  Eigen::Vector3d g_vec;
  double dt;
};

#endif /* EKFILTER_H */
