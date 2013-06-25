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

JointPathEx::JointPathEx(BodyPtr& robot, Link* base, Link* end)
  : JointPath(base, end), sr_gain(1.0), manipulability_limit(0.1), manipulability_gain(0.001) {
  for (int i = 0 ; i < numJoints(); i++ ) {
    joints.push_back(joint(i));
  }

  avoid_weight_gain.resize(numJoints());
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

bool JointPathEx::calcInverseKinematics2Loop(const Vector3& dp, const Vector3& omega, dvector &dq) {
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

    dq = Jinv * v; // dq = pseudoInverse(J) * v

    if ( DEBUG ) {
        std::cerr << "    v :";
        for(int j=0; j < 6; ++j){
            std::cerr << " " << v(j);
        }
        std::cerr << std::endl;
        std::cerr << "    J :" << std::endl << J;
        std::cerr << " Jinv :" << std::endl << Jinv;
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

    return true;
}


bool JointPathEx::calcInverseKinematics2(const Vector3& end_p, const Matrix33& end_R)
{
    static const int MAX_IK_ITERATION = 50;
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
    dvector dq(n);
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
      Vector3 omega(target->R * omegaFromRot(target->R.transpose() * end_R));
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
        const double errsqr = dp.dot(dp) + omega.dot(omega);
        if ( DEBUG ) std::cerr << "  err : " << std::setw(18) << std::setiosflags(std::ios::fixed) << std::setprecision(14) << errsqr << " < " << maxIKErrorSqr << std::endl;
        if(errsqr < maxIKErrorSqr){
          converged = true;
          break;
        }
      }

      calcInverseKinematics2Loop(dp, omega, dq);

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
                
      for(int j=0; j < n; ++j){
          joints[j]->q += LAMBDA * dq(j);
      }

      calcForwardKinematics();

    }
 
    if ( converged ) {
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
    }

    if(!converged){
      std::cerr << "IK Fail, iter = " << iter << std::endl;
      Vector3 dp(end_p - target->p);
      Vector3 omega(target->R * omegaFromRot(target->R.transpose() * end_R));
      const double errsqr = dp.dot(dp) + omega.dot(omega);
      if(isBestEffortIKMode){
        std::cerr << "  err : fabs(" << errsqr << " - " << errsqr0 << ") = " << fabs(errsqr-errsqr0) << " < " << maxIKErrorSqr << " BestEffortIKMode" << std::endl;
      } else {
        std::cerr << "  err : " << errsqr << " ( " << dp[0] << " " << dp[1] << " " << dp[2] << ", " << omega[0] << " " << omega[1] << " " << omega[2] << ") < " << maxIKErrorSqr << std::endl;
      }
      for(int i=0; i < n; ++i){
        joints[i]->q = qorg[i];
      }
      calcForwardKinematics();
    }
    
    return converged;
}

// This should be merged with calcInverseKinematics2 and calcInverseKinematics2Loop
void JointPathEx::solveLimbIK (const hrp::Vector3& _vel_p,
                               const hrp::Vector3& _vel_r,
                               const int transition_count,
                               const double avoid_gain,
                               const double reference_gain,
                               const double MAX_TRANSITION_COUNT,
                               const hrp::dvector& qrefv,
                               bool DEBUGP)
{
            const int n = numJoints();
            double transition_smooth_gain = 1.0;
            if ( transition_count < 0 ) {
                // (/ (log (/ (- 1 0.99) 0.99)) 0.5)
                transition_smooth_gain = 1/(1+exp(-9.19*(((MAX_TRANSITION_COUNT + transition_count) / MAX_TRANSITION_COUNT) - 0.5)));
                // vel_p = vel_p * transition_smooth_gain;
                // vel_r = vel_r * transition_smooth_gain;
            }

#if 0
            setBestEffortIKMode(true);
            setMaxIKError(0.001);
            calcInverseKinematics2(current_p0 + vel_p, target->R);
#else
	    hrp::dmatrix J(6, n);
	    hrp::dmatrix Jinv(n, 6);
	    hrp::dmatrix Jnull(n, n);
	    
	    calcJacobianInverseNullspace(J, Jinv, Jnull);
	    //calcInverseKinematics2Loop(vel_p, vel_r, dq);

	    hrp::dvector v(6);
	    v << hrp::Vector3(transition_smooth_gain *_vel_p), hrp::Vector3(transition_smooth_gain *_vel_r);
	    hrp::dvector dq(n);
	    dq = Jinv * v; // dq = pseudoInverse(J) * v

	    // dq = J#t a dx + ( I - J# J ) Jt b dx
	    // avoid-nspace-joint-limit: avoiding joint angle limit
	    //
	    // dH/dq = (((t_max + t_min)/2 - t) / ((t_max - t_min)/2)) ^2
	    hrp::dvector u(n);
	    for(int j=0; j < n; ++j) { u[j] = 0; }
	    for ( int j = 0; j < n ; j++ ) {
	      double jang = joint(j)->q;
	      double jmax = joint(j)->ulimit;
	      double jmin = joint(j)->llimit;
	      double r = ((( (jmax + jmin) / 2.0) - jang) / ((jmax - jmin) / 2.0));
	      if ( r > 0 ) { r = r*r; } else { r = - r*r; }
	      u[j] += r;
	    }
	    if ( DEBUGP ) {
	      std::cerr << "    u : " << u;
	      std::cerr << "  dqb : " << Jnull * u;
	    }
            if ( transition_count < 0 ) {
              u = u * transition_smooth_gain;
            }
	    dq = dq + Jnull * ( avoid_gain *  u );
	    //
	    // qref - qcurr
	    for(int j=0; j < n; ++j) { u[j] = 0; }
	    for ( int j = 0; j < numJoints(); j++ ) {
              u[j] = ( qrefv[joint(j)->jointId] - joint(j)->q );
	    }
            if ( transition_count < 0 ) {
              u = u * transition_smooth_gain;
            }
	    dq = dq + Jnull * ( reference_gain *  u );

            // break if dq(j) is nan/nil
            bool dq_check = true;
            for(int j=0; j < n; ++j){
                if ( isnan(dq(j)) || isinf(dq(j)) ) {
                    dq_check = false;
                    break;
                }
            }
            if ( ! dq_check ) return;

            // check max speed
            double max_speed = 0;
            for(int j=0; j < n; ++j){
                max_speed = std::max(max_speed, fabs(dq(j)));
            }
	    if ( max_speed > 0.2*0.5 ) { // 0.5 safety margin
                if ( DEBUGP ) {
                    std::cerr << "spdlmt: ";
                    for(int j=0; j < n; ++j) { std::cerr << dq(j) << " "; } std::cerr << std::endl;
                }
                for(int j=0; j < n; ++j) {
                    dq(j) = dq(j) * 0.2*0.5 / max_speed;
                }
                if ( DEBUGP ) {
                    std::cerr << "spdlmt: ";
                    for(int j=0; j < n; ++j) { std::cerr << dq(j) << " "; } std::cerr << std::endl;
                }
            }

            // update robot model
            for(int j=0; j < n; ++j){
                joint(j)->q += dq(j);
            }


            // check limit
            for(int j=0; j < n; ++j){
                if ( joint(j)->q > joint(j)->ulimit) {
                    std::cerr << "Upper joint limit error " << joint(j)->name << std::endl;
                    joint(j)->q = joint(j)->ulimit;
                }
                if ( joint(j)->q < joint(j)->llimit) {
                    std::cerr << "Lower joint limit error " << joint(j)->name << std::endl;
                    joint(j)->q = joint(j)->llimit;
                }
                joint(j)->q = std::max(joint(j)->q, joint(j)->llimit);
            }
#endif

            calcForwardKinematics();
}

