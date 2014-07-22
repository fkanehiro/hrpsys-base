#include "JointPathEx.h"
#include <iostream>
#include <iomanip>
#include <limits.h>
#include <float.h>
#include <hrpUtil/MatrixSolvers.h>

#define deg2rad(x)((x)*M_PI/180)
#define rad2deg(rad) (rad * 180 / M_PI)
#define eps_eq(a, b, c)  (fabs((a)-(b)) <= c)

std::ostream& operator<<(std::ostream& out, hrp::dmatrix &a) {
    const int c = a.rows();
    const int n = a.cols();

    for(int i = 0; i < c; i++){
        out << "      :";
        for(int j = 0; j < n; j++){
            out << " " << std::setw(7) << std::setiosflags(std::ios::fixed) << std::setprecision(4) << (a)(i,j);
        }
        out << std::endl;
    }
}

std::ostream& operator<<(std::ostream& out, hrp::dvector &a) {
    const int n = a.size();

    for(int i = 0; i < n; i++){
        out << std::setw(7) << std::setiosflags(std::ios::fixed) << std::setprecision(4) << a(i) << " ";
    }
    out << std::endl;
}

//#define DEBUG true
#define DEBUG false


using namespace std;
using namespace hrp;
int hrp::calcSRInverse(const dmatrix& _a, dmatrix &_a_sr, double _sr_ratio, dmatrix _w) {
    // J# = W Jt(J W Jt + kI)-1 (Weighted SR-Inverse)
    // SR-inverse :
    // Y. Nakamura and H. Hanafusa : "Inverse Kinematic Solutions With
    // Singularity Robustness for Robot Manipulator Control"
    // J. Dyn. Sys., Meas., Control  1986. vol 108, Issue 3, pp. 163--172.

    const int c = _a.rows(); // 6
    const int n = _a.cols(); // n

    if ( _w.cols() != n || _w.rows() != n ) {
        _w = dmatrix::Identity(n, n);
    }

    dmatrix at = _a.transpose();
    dmatrix a1(c, c);
    a1 = (_a * _w * at +  _sr_ratio * dmatrix::Identity(c,c)).inverse();

    //if (DEBUG) { dmatrix aat = _a * at; std::cerr << " a*at :" << std::endl << aat; }

    _a_sr  = _w * at * a1;
    //if (DEBUG) { dmatrix ii = _a * _a_sr; std::cerr << "    i :" << std::endl << ii; }
}

// overwrite hrplib/hrpUtil/Eigen3d.cpp
Vector3 omegaFromRotEx(const Matrix33& r)
{
    using ::std::numeric_limits;

    double alpha = (r(0,0) + r(1,1) + r(2,2) - 1.0) / 2.0;

    if(fabs(alpha - 1.0) < 1.0e-12) {   //th=0,2PI;
        return Vector3::Zero();
    } else {
        double th = acos(alpha);
        double s = sin(th);

        if (s < numeric_limits<double>::epsilon()) {   //th=PI
            return Vector3( sqrt((r(0,0)+1)*0.5)*th, sqrt((r(1,1)+1)*0.5)*th, sqrt((r(2,2)+1)*0.5)*th );
        }

        double k = -0.5 * th / s;

        return Vector3( (r(1,2) - r(2,1)) * k,
                        (r(2,0) - r(0,2)) * k,
                        (r(0,1) - r(1,0)) * k );
    }
}

JointPathEx::JointPathEx(BodyPtr& robot, Link* base, Link* end)
    : JointPath(base, end), sr_gain(1.0), manipulability_limit(0.1), manipulability_gain(0.001), maxIKPosErrorSqr(1.0e-8), maxIKRotErrorSqr(1.0e-6), maxIKIteration(50) {
  for (int i = 0 ; i < numJoints(); i++ ) {
    joints.push_back(joint(i));
  }

  avoid_weight_gain.resize(numJoints());
}

void JointPathEx::setMaxIKError(double epos, double erot) {
  maxIKPosErrorSqr = epos*epos;
  maxIKRotErrorSqr = erot*erot;
}

void JointPathEx::setMaxIKError(double e)
{
    maxIKErrorSqr = e * e;
}

void JointPathEx::setMaxIKIteration(int iter) {
    maxIKIteration = iter;
}

bool JointPathEx::calcJacobianInverseNullspace(dmatrix &J, dmatrix &Jinv, dmatrix &Jnull) {
    const int n = numJoints();
                
    hrp::dmatrix w = hrp::dmatrix::Identity(n,n);
    //
    // wmat/weight: weighting joint angle weight
    //
    // w_i = 1 + | dH/dt |      if d|dH/dt| >= 0
    //     = 1                  if d|dH/dt| <  0
    // dH/dt = (t_max - t_min)^2 (2t - t_max - t_min)
    //         / 4 (t_max - t)^2 (t - t_min)^2
    //
    // T. F. Chang and R.-V. Dubey: "A weighted least-norm solution based
    // scheme for avoiding joint limits for redundant manipulators", in IEEE
    // Trans. On Robotics and Automation, 11((2):286-292, April 1995.
    //
    for ( int j = 0; j < n ; j++ ) {
        double jang = joints[j]->q;
        double jmax = joints[j]->ulimit;
        double jmin = joints[j]->llimit;
        double e = deg2rad(1);
        if ( eps_eq(jang, jmax,e) && eps_eq(jang, jmin,e) ) {
        } else if ( eps_eq(jang, jmax,e) ) {
            jang = jmax - e;
        } else if ( eps_eq(jang, jmin,e) ) {
            jang = jmin + e;
        }

        double r;
        if ( eps_eq(jang, jmax,e) && eps_eq(jang, jmin,e) ) {
            r = DBL_MAX;
        } else {
            r = fabs( (pow((jmax - jmin),2) * (( 2 * jang) - jmax - jmin)) /
                      (4 * pow((jmax - jang),2) * pow((jang - jmin),2)) );
            if (isnan(r)) r = 0;
        }

        if (( r - avoid_weight_gain[j] ) >= 0 ) {
	  w(j, j) = ( 1.0 / ( 1.0 + r) );
	} else {
	  w(j, j) = 1.0;
	}
        avoid_weight_gain[j] = r;
    }
    if ( DEBUG ) {
        std::cerr << " cost :";
        for(int j = 0; j < n; j++ ) { std::cerr << std::setw(8) << std::setiosflags(std::ios::fixed) << std::setprecision(4) << avoid_weight_gain[j]; }
        std::cerr << std::endl;
        std::cerr << "    w :";
        for(int j = 0; j < n; j++ ) { std::cerr << std::setw(8) << std::setiosflags(std::ios::fixed) << std::setprecision(4) << w(j, j); }
        std::cerr << std::endl;
    }

    calcJacobian(J);

    double manipulability = sqrt((J*J.transpose()).determinant());
    double k = 0;
    if ( manipulability < manipulability_limit ) {
	k = manipulability_gain * pow((1 - ( manipulability / manipulability_limit )), 2);
    }
    if ( DEBUG ) {
	std::cerr << " manipulability = " <<  manipulability << " < " << manipulability_limit << ", k = " << k << " -> " << sr_gain * k << std::endl;
    }

    calcSRInverse(J, Jinv, sr_gain * k, w);

    Jnull = ( hrp::dmatrix::Identity(n, n) - Jinv * J);

    return true;
}

bool JointPathEx::calcInverseKinematics2Loop(const Vector3& dp, const Vector3& omega,
                                             const double LAMBDA, const double avoid_gain, const double reference_gain, const hrp::dvector* reference_q) {
    const int n = numJoints();

    if ( DEBUG ) {
        std::cerr << "angle :";
        for(int j=0; j < n; ++j){
            std::cerr << std::setw(8) << std::setiosflags(std::ios::fixed) << std::setprecision(4) << rad2deg(joints[j]->q);
        }
        std::cerr << endl;
    }
    dvector v(6);
    v << dp, omega;

    hrp::dmatrix J(6, n);
    hrp::dmatrix Jinv(n, 6);
    hrp::dmatrix Jnull(n, n);

    calcJacobianInverseNullspace(J, Jinv, Jnull);

    hrp::dvector dq(n);
    dq = Jinv * v; // dq = pseudoInverse(J) * v

    if ( DEBUG ) {
        std::cerr << "    v :";
        for(int j=0; j < 6; ++j){
            std::cerr << " " << v(j);
        }
        std::cerr << std::endl;
        std::cerr << "    J :" << std::endl << J;
        std::cerr << " Jinv :" << std::endl << Jinv;
    }
    // If avoid_gain is set, add joint limit avoidance by null space vector
    if ( avoid_gain > 0.0 ) {
      // dq = J#t a dx + ( I - J# J ) Jt b dx
      // avoid-nspace-joint-limit: avoiding joint angle limit
      //
      // dH/dq = (((t_max + t_min)/2 - t) / ((t_max - t_min)/2)) ^2
      hrp::dvector u(n);
      for ( int j = 0; j < n ; j++ ) {
        double jang = joint(j)->q;
        double jmax = joint(j)->ulimit;
        double jmin = joint(j)->llimit;
        double r = ((( (jmax + jmin) / 2.0) - jang) / ((jmax - jmin) / 2.0));
        if ( r > 0 ) { r = r*r; } else { r = - r*r; }
        u[j] = avoid_gain * r;
      }
      if ( DEBUG ) {
        std::cerr << " u(jl):";
        for(int j=0; j < n; ++j){
          std::cerr << std::setw(8) << std::setiosflags(std::ios::fixed) << std::setprecision(4) << rad2deg(u(j));
        }
        std::cerr << std::endl;
        std::cerr << " JN*u :";
        hrp::dvector Jnullu = Jnull * u;
        for(int j=0; j < n; ++j){
          std::cerr << std::setw(8) << std::setiosflags(std::ios::fixed) << std::setprecision(4) << rad2deg(Jnullu(j));
        }
        std::cerr << std::endl;
      }
      dq = dq + Jnull * u;
    }
    // If reference_gain and reference_q are set, add following to reference_q by null space vector
    if ( reference_gain > 0.0 && reference_q != NULL ) {
      //
      // qref - qcurr
      hrp::dvector u(n);
      for ( int j = 0; j < numJoints(); j++ ) {
        u[j] = reference_gain * ( (*reference_q)[joint(j)->jointId] - joint(j)->q );
      }
      if ( DEBUG ) {
        std::cerr << "u(ref):";
        for(int j=0; j < n; ++j){
          std::cerr << std::setw(8) << std::setiosflags(std::ios::fixed) << std::setprecision(4) << rad2deg(u(j));
        }
        std::cerr << std::endl;
        std::cerr << "  JN*u:";
        hrp::dvector nullu = Jnull * u;
        for(int j=0; j < n; ++j){
          std::cerr << std::setw(8) << std::setiosflags(std::ios::fixed) << std::setprecision(4) << rad2deg(nullu(j));
        }
        std::cerr << std::endl;
      }
      dq = dq + Jnull * u;
    }
    if ( DEBUG ) {
      std::cerr << "   dq :";
      for(int j=0; j < n; ++j){
        std::cerr << std::setw(8) << std::setiosflags(std::ios::fixed) << std::setprecision(4) << rad2deg(dq(j));
      }
      std::cerr << std::endl;
    }

    // default servoErrorLimit in RobotHardware(DEFAULT_ANGLE_ERROR_LIMIT) = 0.2[rad]
    double max_speed = 0;
    for(int j=0; j < n; ++j){
      max_speed = std::max(max_speed, fabs(dq(j)));
    }
    if ( max_speed > 0.2*0.5 ) { // 0.5 safety margin
      if ( DEBUG ) {
        std::cerr << "spdlmt: ";
        for(int j=0; j < n; ++j) { std::cerr << dq(j) << " "; } std::cerr << std::endl;
      }
      for(int j=0; j < n; ++j) {
        dq(j) = dq(j) * 0.2*0.5 / max_speed;
      }
      if ( DEBUG ) {
        std::cerr << "spdlmt: ";
        for(int j=0; j < n; ++j) { std::cerr << dq(j) << " "; } std::cerr << std::endl;
      }
    }

    // check nan / inf
    bool solve_linear_equation = true;
    for(int j=0; j < n; ++j){
      if ( isnan(dq(j)) || isinf(dq(j)) ) {
        solve_linear_equation = false;
        break;
      }
    }
    if ( ! solve_linear_equation ) {
      std::cerr << "ERROR nan/inf is found" << std::endl;
      return false;
    }

    // joint angles update
    for(int j=0; j < n; ++j){
      joints[j]->q += LAMBDA * dq(j);
    }

    // upper/lower limit check
    for(int j=0; j < n; ++j){
      if ( joints[j]->q > joints[j]->ulimit) {
        std::cerr << "Upper joint limit error " << joints[j]->name << std::endl;
        joints[j]->q = joints[j]->ulimit;
      }
      if ( joints[j]->q < joints[j]->llimit) {
        std::cerr << "Lower joint limit error " << joints[j]->name << std::endl;
        joints[j]->q = joints[j]->llimit;
      }
      joints[j]->q = std::max(joints[j]->q, joints[j]->llimit);
    }

    calcForwardKinematics();

    return true;
}


bool JointPathEx::calcInverseKinematics2(const Vector3& end_p, const Matrix33& end_R,
                                         const double avoid_gain, const double reference_gain, const hrp::dvector* reference_q)
{
    static const int MAX_IK_ITERATION = maxIKIteration;
    static const double LAMBDA = 0.9;

    LinkPath linkPath(baseLink(), endLink());

    if(joints.empty()){
        if(linkPath.empty()){
            return false;
        }
        if(baseLink() == endLink()){
            baseLink()->p = end_p;
            baseLink()->R = end_R;
            return true;
        } else {
            // \todo implement here
            return false;
        }
    }
    
    const int n = numJoints();
    dvector qorg(n);

    Link* target = linkPath.endLink();

    for(int i=0; i < n; ++i){
        qorg[i] = joints[i]->q;
        avoid_weight_gain[i] = 100000000000000000000.0;
    }

    
    double errsqr = DBL_MAX;//maxIKErrorSqr * 100.0;
    double errsqr0 = errsqr;
    bool converged = false;

    int iter = 0;
    for(iter = 0; iter < MAX_IK_ITERATION; iter++){
        
      if ( DEBUG ) {
        std::cerr << " iter : " << iter << " / " << MAX_IK_ITERATION << ", n = " << n << std::endl;
      }
        
      Vector3 dp(end_p - target->p);
      Vector3 omega(target->R * omegaFromRotEx(target->R.transpose() * end_R));
      if ( dp.norm() > 0.1 ) dp = dp*0.1/dp.norm();
      if ( omega.norm() > 0.5 ) omega = omega*0.5/omega.norm();


      if ( DEBUG ) {
        std::cerr << "   dp : " << dp[0] << " " << dp[1] << " " << dp[2] << std::endl;
        std::cerr << "omega : " << omega[0] << " " << omega[1] << " " << omega[2] << std::endl;
        //std::cerr << "    J :" << std::endl << J;
        //std::cerr << "  det : " << det(J) << std::endl;
        std::cerr << "  err : dp = " << dp.dot(dp) << ", omega = " <<  omega.dot(omega) << std::endl;
      }

      if(isBestEffortIKMode){
        errsqr0 = errsqr;
        errsqr = dp.dot(dp) + omega.dot(omega);
        if ( DEBUG ) std::cerr << "  err : fabs(" << std::setw(18) << std::setiosflags(std::ios::fixed) << std::setprecision(14) << errsqr << " - " << errsqr0 << ") = " << fabs(errsqr-errsqr0) << " < " << maxIKErrorSqr << " BestEffortIKMode" << std::endl;
        if(fabs(errsqr - errsqr0) < maxIKErrorSqr){
          converged = true;
          break;
        }
      } else {
        if ( DEBUG ) std::cerr << "  err : " << std::setw(18) << std::setiosflags(std::ios::fixed) << std::setprecision(14) << sqrt(dp.dot(dp)) << " < " << sqrt(maxIKPosErrorSqr) << ", " << std::setw(18) << std::setiosflags(std::ios::fixed) << std::setprecision(14) << sqrt(omega.dot(omega)) << " < " << sqrt(maxIKRotErrorSqr) << std::endl;
        if( (dp.dot(dp) < maxIKPosErrorSqr) && (omega.dot(omega) < maxIKRotErrorSqr) ) {
          converged = true;
          break;
        }
      }

      if ( !calcInverseKinematics2Loop(dp, omega, LAMBDA, avoid_gain, reference_gain, reference_q) )
        return false;
    }

    if(!converged){
      std::cerr << "IK Fail, iter = " << iter << std::endl;
      Vector3 dp(end_p - target->p);
      Vector3 omega(target->R * omegaFromRotEx(target->R.transpose() * end_R));
      const double errsqr = dp.dot(dp) + omega.dot(omega);
      if(isBestEffortIKMode){
        std::cerr << "  err : fabs(" << errsqr << " - " << errsqr0 << ") = " << fabs(errsqr-errsqr0) << " < " << maxIKErrorSqr << " BestEffortIKMode" << std::endl;
      } else {
          std::cerr << "  err : " << dp.dot(dp) << " ( " << dp[0] << " " << dp[1] << " " << dp[2] << ") < " << maxIKPosErrorSqr << std::endl;
          std::cerr << "      : " << omega.dot(omega) << " ( " << omega[0] << " " << omega[1] << " " << omega[2] << ") < " << maxIKRotErrorSqr << std::endl;
      }
      for(int i=0; i < n; ++i){
        joints[i]->q = qorg[i];
      }
      calcForwardKinematics();
    }
    
    return converged;
}
