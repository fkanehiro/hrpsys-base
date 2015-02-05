#ifndef RPYKALMANFILTER_H
#define RPYKALMANFILTER_H

#include <hrpUtil/EigenTypes.h>
namespace hrp{
  typedef Eigen::Vector2d Vector2;
  typedef Eigen::Matrix2d Matrix22;
}
#include <iostream>

class KFilter {
public:
  KFilter() {
      F(0,0) = 1; F(0,1) =-0.005;
      F(1,0) = 0; F(1,1) =    1;

      P(0,0) = 0; P(0,1) =    0;
      P(1,0) = 0; P(1,1) =    0;

      Q(0,0) = 0.001 * 0.005; Q(0,1) = 0.000 * 0.005;
      Q(1,0) = 0.000 * 0.005; Q(1,1) = 0.003 * 0.005;

      R = 0.03;

      B(0) = 0.005; B(1) = 0;

      H(0,0) = 1; H(0,1) = 0;

      x(0) = 0; x(1) = 0;
      z = 0;

      I = hrp::dmatrix::Identity(2,2);
  }
  void setF(double _f0, double _f1, double _f2, double _f3) { F(0,0) = _f0; F(0,1) = _f1; F(1,0) = _f2; F(1,1) = _f3;}
  void setP(double _p0, double _p1, double _p2, double _p3) { P(0,0) = _p0; P(0,1) = _p1; P(1,0) = _p2; P(1,1) = _p3;}
  void setQ(double _q0, double _q1, double _q2, double _q3) { Q(0,0) = _q0; Q(0,1) = _q1; Q(1,0) = _q2; Q(1,1) = _q3;}
  void setR(double _R) { R = _R; }
  void setB(double _b0, double _b1) { B[0] = _b0; B[1] = _b1; }
  hrp::Vector2 &getx() { return x; }
  void update(double u, double _z) {
      // Predicted (a priori) state estimate
      x = F * x + B * u;
      // Predicted (a priori) estimate covariance
      P = F * P * F.transpose() + Q;
      //
      // Innovation or measurement residual
      z = _z;
      double y = z - H * x;
      // Innovation (or residual) covariance
      double S = H * P * H.transpose() + R;
      // Optimal Kalman gain
      K = P * H.transpose() / S;
      // Updated (a posteriori) state estimate
      x = x + K * y;
      // Updated (a posteriori) estimate covariance
      P = (I - K * H) * P;
  }
  void resetStateByObservation() {
      x(0) = z;
      x(1) = 0;
  };
private:
  hrp::Matrix22 P, Q, I, F;
  Eigen::Matrix<double, 1, 2> H;
  hrp::Vector2 B, K;
  hrp::Vector2 x;
  double R, z;
};

class RPYKalmanFilter {
public:
    RPYKalmanFilter() : m_sensorR(hrp::Matrix33::Identity()) {};
    void main_one (hrp::Vector3& rpy, hrp::Vector3& rpyRaw, const hrp::Vector3& acc, const hrp::Vector3& gyro)
    {
      //
      // G = [ cosb, sinb sina, sinb cosa,
      //          0,      cosa,     -sina,
      //      -sinb, cosb sina, cosb cosa]
      // s = [sx, sy, sz]t ( accelerometer )
      // g = [ 0 0 -g]t
      // s = G g = [g sinb, -g cosb sina, -g cosb cosa]t
      // hrp::RateGyroSensor* sensor = m_robot->sensor<hrp::RateGyroSensor>("gyrometer");
      double g = sqrt(acc(0) * acc(0) + acc(1) * acc(1) + acc(2) * acc(2));
      // atan2(y, x) = atan(y/x)
      double a, b;
      b = atan2( - acc(0) / g, sqrt( acc(1)/g * acc(1)/g + acc(2)/g * acc(2)/g ) );
      a = atan2( ( acc(1)/g ), ( acc(2)/g ) );
      rpyRaw = hrp::Vector3(a,b,0);
      // #if 0
      //       // complementary filter
      //       m_rpy.data.r = 0.98 *(m_rpy.data.r+m_rate.data.avx*m_dt) + 0.02*m_rpyRaw.data.r;
      //       m_rpy.data.p = 0.98 *(m_rpy.data.p+m_rate.data.avy*m_dt) + 0.02*m_rpyRaw.data.p;
      //       m_rpy.data.y = 0.98 *(m_rpy.data.y+m_rate.data.avz*m_dt) + 0.02*m_rpyRaw.data.y;
      // #endif
      // kalman filter
      // x[0] = m_rpyRaw.data.r; // angle (from m/sec Acceleration3D, unstable, no drift )
      // x[1] = m_rate.data.avx; // rate ( rad/sec, AngularVelocity, gyro, stable/drift )
      // use kalman filter with imaginary data
      r_filter.update(gyro(0), rpyRaw(0));
      p_filter.update(gyro(1), rpyRaw(1));
      y_filter.update(gyro(2), rpyRaw(2));

      Eigen::AngleAxis<double> aaZ(y_filter.getx()[0], Eigen::Vector3d::UnitZ());
      Eigen::AngleAxis<double> aaY(p_filter.getx()[0], Eigen::Vector3d::UnitY());
      Eigen::AngleAxis<double> aaX(r_filter.getx()[0], Eigen::Vector3d::UnitX());
      Eigen::Quaternion<double> q = aaZ * aaY * aaX;
      hrp::Matrix33 imaginaryRotationMatrix = q.toRotationMatrix();
      hrp::Matrix33 realRotationMatrix = imaginaryRotationMatrix * m_sensorR; // inverse transform to real data
      hrp::Vector3 euler = realRotationMatrix.eulerAngles(2,1,0);
      rpy(0) = euler(2);
      rpy(1) = euler(1);
      rpy(2) = euler(0);
    };
    void setParam (const double _dt, const double _Q_angle, const double _Q_rate, const double _R_angle, const std::string print_str = "")
    {
        Q_angle = _Q_angle;
        Q_rate = _Q_rate;
        R_angle = _R_angle;
        r_filter.setF(1, -_dt, 0, 1);
        r_filter.setP(0, 0, 0, 0);
        r_filter.setQ(Q_angle*_dt, 0, 0, Q_rate*_dt);
        r_filter.setR(R_angle);
        r_filter.setB(_dt, 0);

        p_filter.setF(1, -_dt, 0, 1);
        p_filter.setP(0, 0, 0, 0);
        p_filter.setQ(Q_angle*_dt, 0, 0, Q_rate*_dt);
        p_filter.setR(R_angle);
        p_filter.setB(_dt, 0);

        y_filter.setF(1, -_dt, 0, 1);
        y_filter.setP(0, 0, 0, 0);
        y_filter.setQ(Q_angle*_dt, 0, 0, Q_rate*_dt);
        y_filter.setR(R_angle);
        y_filter.setB(_dt, 0);
        std::cerr << "[" << print_str << "]   Q_angle=" << Q_angle << ", Q_rate=" << Q_rate << ", R_angle=" << R_angle << std::endl;
    };
    void resetKalmanFilterState()
    {
        r_filter.resetStateByObservation();
        p_filter.resetStateByObservation();
        y_filter.resetStateByObservation();
    };
    void setSensorR (const hrp::Matrix33& sr) { m_sensorR = sr;};
    double getQangle () const { return Q_angle;};
    double getQrate () const { return Q_rate;};
    double getRangle () const { return R_angle;};
private:
    KFilter r_filter, p_filter, y_filter;
    double Q_angle, Q_rate, R_angle;
    hrp::Matrix33 m_sensorR;
};

#endif /* RPYKALMANFILTER_H */
