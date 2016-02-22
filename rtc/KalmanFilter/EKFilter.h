#ifndef EKFILTER_H
#define EKFILTER_H

#include <hrpUtil/EigenTypes.h>
#include <iostream>
#include "util/Hrpsys.h"

class EKFilter {
public:
  EKFilter() {
    x << 1, 0, 0, 0, 0, 0, 0;
    P = Eigen::Matrix<double, 7, 7>::Identity() * 5;
    Q = Eigen::Matrix<double, 7, 7>::Identity();
    Q.block<4, 4>(0, 0) *= 0.1;
    Q.block<3, 3>(4, 4) *= 0.001;
    R = Eigen::Matrix<double, 3, 3>::Zero();
    R(0, 0) = 4;
    R(1, 1) = 4;
    R(2, 2) = 2;
    g_vec = Eigen::Vector3d(0.0, 0.0, 9.80665);
  }

  Eigen::Matrix<double, 7, 1> getx() { return x; }

  Eigen::Matrix<double, 3, 1> accelerationToRpy(const double& acc_x, const double& acc_y, const double& acc_z) {
    /*
     * acc = \dot{v} + w \times v + R^T g;
     * -> R^T g = acc - \dot{v} + w \times v;
     * -> R^T g = acc;
     * -> acc_x = -sinb g, acc_y = sina cosb g, acc_z = cosa cosb g;
     */
    double roll = atan2(acc_y, acc_z);
    double pitch = atan2(-acc_x, sqrt(acc_y * acc_y + acc_z * acc_z));
    double yaw = 0;             /* cannot be defined only by acceleration */
    Eigen::Vector3d rpy = Eigen::Vector3d(roll, pitch, yaw);
    return rpy;
  }

  Eigen::Matrix<double, 4, 4> calcOmega(const Eigen::Vector3d& w) {
    /* \dot{q} = \frac{1}{2} omega q */
    Eigen::Matrix<double, 4, 4> omega;
    omega <<
      0, -w[0], -w[1], -w[2],
      w[0],     0,  w[2], -w[1],
      w[1], -w[2],     0,  w[0],
      w[2],  w[1], -w[0],     0;
    return omega;
  }

  Eigen::Matrix<double, 7, 1> calcPredictedState(Eigen::Matrix<double, 4, 1> q,
                                                 Eigen::Vector3d gyro,
                                                 Eigen::Vector3d drift,
                                                 const double& dt) {
    /* x_a_priori = f(x, u) */
    Eigen::Matrix<double, 7, 1> ret;
    Eigen::Matrix<double, 4, 1> q_a_priori;
    Eigen::Vector3d gyro_compensated = gyro - drift;
    q_a_priori = q + dt / 2 * calcOmega(gyro_compensated) * q;
    ret.block<4, 1>(0, 0) = q_a_priori;
    ret.block<3, 1>(4, 0) = drift;
    return ret;
  }

  Eigen::Matrix<double, 7, 7> calcF(Eigen::Matrix<double, 4, 1> q,
                                    const Eigen::Vector3d& gyro,
                                    Eigen::Vector3d drift,
                                    const double& dt) {
    Eigen::Matrix<double, 7, 7> F;
    Eigen::Vector3d gyro_compensated = gyro - drift;
    F.block<4, 4>(0, 0) =
      Eigen::Matrix<double, 4, 4>::Identity() + dt / 2 * calcOmega(gyro_compensated);
    F.block<4, 3>(0, 4) <<
      dt / 2 * q[1],   dt / 2 * q[2],   dt / 2 * q[3],
      - dt / 2 * q[0],   dt / 2 * q[3], - dt / 2 * q[2],
      - dt / 2 * q[3], - dt / 2 * q[0],   dt / 2 * q[1],
      dt / 2 * q[2], - dt / 2 * q[1], - dt / 2 * q[0];
    F.block<3, 4>(0, 4) = Eigen::Matrix<double, 3, 4>::Zero();
    F.block<3, 3>(4, 4) = Eigen::Matrix<double, 3, 3>::Identity();
    return F;
  }

  Eigen::Matrix<double, 7, 7> calcPredictedCovariance(Eigen::Matrix<double, 7, 7> F) {
    /* P_a_priori = F P F^T + Q */
    return F * P * F.transpose() + Q;
  }

  Eigen::Vector3d calcAcc(Eigen::Matrix<double, 4, 1> q,
                          const Eigen::Vector3d& vel_ref,
                          const Eigen::Vector3d& acc_ref,
                          const Eigen::Vector3d& angular_rate_ref) {
    /* acc = \dot{v} + w \times v + R^T g; */
    Eigen::Quaternion<double> q_tmp = Eigen::Quaternion<double>(q[0], q[1], q[2], q[3]);
    Eigen::Vector3d acc =
      acc_ref + angular_rate_ref.cross(vel_ref) +
      q_tmp.toRotationMatrix().transpose() * g_vec;
    /* 
     * Eigen::Vector3d hoge;
     * hoge <<
     *   -2 * q_tmp2.w() * q_tmp2.y() + 2 * q_tmp2.x() * q_tmp2.z(),
     *   2 * q_tmp2.w() * q_tmp2.x() + 2 * q_tmp2.y() * q_tmp2.z(),
     *   q_tmp2.w() * q_tmp2.w()  - q_tmp2.x() * q_tmp2.x() - q_tmp2.y() * q_tmp2.y() + q_tmp2.z() * q_tmp2.z();
     * hoge *= g_vec[2];
     * std::cerr << "diff 2" << std::endl << acc - hoge << std::endl;
     */
    return acc;
  }

  Eigen::Matrix<double, 3, 7> calcH(Eigen::Matrix<double, 4, 1> q) {
    Eigen::Matrix<double, 3, 7> H;
    double w = q[0], x = q[1], y = q[2], z = q[3];
    /* 
     * H <<
     *   2 * y,  2 * z,  2 * w,  2 * x, 0, 0, 0,
     *   -2 * x, -2 * w,  2 * z,  2 * y, 0, 0, 0,
     *   2 * w, -2 * x, -2 * y,  2 * z, 0, 0, 0;
     * H *= g_vec[2];
     */
    H <<
      -y,  z, -w, x, 0, 0, 0,
       x,  w,  z, y, 0, 0, 0,
       w, -x, -y, z, 0, 0, 0;
    H *= 2 * g_vec[2];
    return H;
  }

  Eigen::Vector3d calcMeasurementResidual(const Eigen::Vector3d& acc_measured,
                                          const Eigen::Vector3d& vel_ref,
                                          const Eigen::Vector3d& acc_ref,
                                          const Eigen::Vector3d& angular_rate_ref,
                                          Eigen::Matrix<double, 4, 1> q) {
    /* y = z - h(x) */
    Eigen::Vector3d y = acc_measured - calcAcc(q, vel_ref, acc_ref, angular_rate_ref);
    /* 
     * std::cerr << "acc_measured" << std::endl << acc_measured << std::endl;
     * std::cerr << "calc acc" << std::endl << calcAcc(q, vel_ref, acc_ref, angular_rate_ref) << std::endl;
     * std::cerr << "diff" << std::endl << y << std::endl;
     */
    return y;
  }


  void prediction(const Eigen::Vector3d& u) {
    Eigen::Matrix<double, 4, 1> q = x.block<4, 1>(0, 0);
    Eigen::Vector3d drift = x.block<3, 1>(4, 0);
    Eigen::Matrix<double, 7, 7> F = calcF(q, u, drift, dt);
    x_a_priori = calcPredictedState(q, u, drift, dt);
    P_a_priori = calcPredictedCovariance(F);
  }

  void correction(const Eigen::Vector3d& z,
                  const Eigen::Vector3d& vel_ref,
                  const Eigen::Vector3d& acc_ref,
                  const Eigen::Vector3d& angular_rate_ref) {
    Eigen::Matrix<double, 4, 1> q_a_priori = x_a_priori.block<4, 1>(0, 0).normalized();
    Eigen::Matrix<double, 3, 7> H;
    Eigen::Matrix<double, 3, 3> S;
    Eigen::Matrix<double, 7, 3> K;
    Eigen::Vector3d y;
    /* need to normalize q_a_priori ? */
    y = calcMeasurementResidual(z, vel_ref, acc_ref, angular_rate_ref, q_a_priori);
    H = calcH(q_a_priori);
    S = H * P_a_priori * H.transpose() + R;
    K = P_a_priori * H.transpose() * S.inverse();
    Eigen::Matrix<double, 7, 1> x_tmp = x_a_priori + K * y;
    x.block<4, 1>(0, 0) = x_tmp.block<4, 1>(0, 0).normalized(); /* quaternion */
    x.block<3, 1>(4, 0) = x_tmp.block<3, 1>(4, 0); /* bias */
    P = (Eigen::Matrix<double, 7, 7>::Identity() - K * H) * P_a_priori;
  }

  void printAll() {
    std::cerr << "x" << std::endl << x << std::endl;
    std::cerr << "x_a_priori" << std::endl << x_a_priori << std::endl;
    std::cerr << "P" << std::endl << P << std::endl << std::endl;
    std::cerr << "P_a_priori" << std::endl << P_a_priori << std::endl << std::endl;
    /*
     * std::cerr << "Q" << std::endl << Q << std::endl << std::endl;
     * std::cerr << "R" << std::endl << R << std::endl << std::endl;
     */
  }

  void main_one (hrp::Vector3& rpy, hrp::Vector3& rpyRaw, const hrp::Vector3& acc, const hrp::Vector3& gyro)
  {
      prediction(gyro);
      correction(acc, Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
      /* ekf_filter.printAll(); */
      Eigen::Matrix<double, 7, 1> x = getx();
      Eigen::Quaternion<double> q = Eigen::Quaternion<double>(x[0], x[1], x[2], x[3]);
      hrp::Vector3 eulerZYX = q.toRotationMatrix().eulerAngles(2,1,0);
      rpy(2) = eulerZYX(0);
      rpy(1) = eulerZYX(1);
      rpy(0) = eulerZYX(2);
  };

  void setdt (const double _dt) { dt = _dt;};
private:
  Eigen::Matrix<double, 7, 1> x, x_a_priori;
  Eigen::Matrix<double, 7, 7> P, P_a_priori;
  Eigen::Matrix<double, 7, 7> Q;
  Eigen::Matrix<double, 3, 3> R;
  /* static const Eigen::Vector3d g_vec = Eigen::Vector3d(0.0, 0.0, 9.80665); */
  Eigen::Vector3d g_vec;
  double dt;
};

#endif /* EKFILTER_H */
