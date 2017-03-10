#ifndef __JOINT_PATH_EX_H__
#define __JOINT_PATH_EX_H__
#include <hrpModel/Body.h>
#include <hrpModel/Link.h>
#include <hrpModel/JointPath.h>
#include <cmath>
#include <coil/stringutil.h>

// hrplib/hrpUtil/MatrixSolvers.h
namespace hrp {
    int calcSRInverse(const dmatrix& _a, dmatrix &_a_sr, double _sr_ratio = 1.0, dmatrix _w = dmatrix::Identity(0,0));
};

// hrplib/hrpModel/JointPath.h
namespace hrp {
    class JointPathEx : public JointPath {
  public:
    JointPathEx(BodyPtr& robot, Link* base, Link* end, double control_cycle, bool _use_inside_joint_weight_retrieval = true, const std::string& _debug_print_prefix = "");
    bool calcJacobianInverseNullspace(dmatrix &J, dmatrix &Jinv, dmatrix &Jnull);
    bool calcInverseKinematics2Loop(const Vector3& dp, const Vector3& omega, const double LAMBDA, const double avoid_gain = 0.0, const double reference_gain = 0.0, const dvector* reference_q = NULL);
    bool calcInverseKinematics2Loop(const Vector3& end_effector_p, const Matrix33& end_effector_R,
                                    const double LAMBDA, const double avoid_gain = 0.0, const double reference_gain = 0.0, const hrp::dvector* reference_q = NULL,
                                    const double vel_gain = 1.0,
                                    const hrp::Vector3& localPos = hrp::Vector3::Zero(), const hrp::Matrix33& localR = hrp::Matrix33::Identity());
    bool calcInverseKinematics2(const Vector3& end_p, const Matrix33& end_R, const double avoid_gain = 0.0, const double reference_gain = 0.0, const dvector* reference_q = NULL);
    void solveFullbodyID(const hrp::BodyPtr robot, hrp::Vector3& f_ans, hrp::Vector3& t_ans);
    double getSRGain() { return sr_gain; }
    bool setSRGain(double g) { sr_gain = g; }
    double getManipulabilityLimit() { return manipulability_limit; }
    bool setManipulabilityLimit(double l) { manipulability_limit = l; }
    bool setManipulabilityGain(double l) { manipulability_gain = l; }
    void setMaxIKError(double epos, double erot);
    void setMaxIKError(double e);
    void setMaxIKIteration(int iter);
    void setOptionalWeightVector(const std::vector<double>& _opt_w)
    {
        for (unsigned int i = 0 ; i < numJoints(); i++ ) {
            optional_weight_vector[i] = _opt_w[i];
        }
    };
    bool setInterlockingJointPairIndices (const std::vector<std::pair<Link*, Link*> >& pairs, const std::string& print_str = "");
    bool setInterlockingJointPairIndices (const std::vector<std::pair<size_t, size_t> >& pairs);
    void getInterlockingJointPairIndices (std::vector<std::pair<size_t, size_t> >& pairs);
    void getOptionalWeightVector(std::vector<double>& _opt_w)
    {
        for (unsigned int i = 0 ; i < numJoints(); i++ ) {
            _opt_w[i] = optional_weight_vector[i];
        }
    };
  protected:
        double maxIKPosErrorSqr, maxIKRotErrorSqr;
        int maxIKIteration;
        std::vector<Link*> joints;
        std::vector<double> avoid_weight_gain, optional_weight_vector;
        // Interlocking joint pairs
        //  pair = [index of joint1, index of joint2], index is considered as index for "joints[index]"
        //  Joint angles of joint1 and joint2 has relathionships.
        //  Currently joint1 = joint2 is assumed.
        std::vector<std::pair<size_t, size_t> > interlocking_joint_pair_indices;
        double sr_gain, manipulability_limit, manipulability_gain, dt;
        std::string debug_print_prefix;
        // Print message Hz management
        std::vector<size_t> joint_limit_debug_print_counts;
        size_t debug_print_freq_count;
        bool use_inside_joint_weight_retrieval;
    };

    typedef boost::shared_ptr<JointPathEx> JointPathExPtr;

    struct VirtualForceSensorParam {
        int id;
        hrp::Link* link;
        hrp::Vector3 localPos;
        hrp::Matrix33 localR;
    };

    void readVirtualForceSensorParamFromProperties (std::map<std::string, hrp::VirtualForceSensorParam>& vfs,
                                                    hrp::BodyPtr m_robot,
                                                    const std::string& prop_string,
                                                    const std::string& instance_name);

    void readInterlockingJointsParamFromProperties (std::vector<std::pair<Link*, Link*> >& pairs,
                                                    hrp::BodyPtr m_robot,
                                                    const std::string& prop_string,
                                                    const std::string& instance_name);
};


#include "../TorqueFilter/IIRFilter.h"
class SimpleFullbodyInverseDynamicsSolver{
  private:
    hrp::BodyPtr m_robot;
    hrp::dvector q, q_old, q_oldold, dq, ddq, ddq_filtered;
    hrp::Vector3 base_p, base_p_old, base_p_oldold, base_v, base_dv, base_dv_filtered;
    hrp::Matrix33 base_R, base_R_old, base_dR, base_w_hat;
    hrp::Vector3 base_w, base_w_old, base_dw, base_dw_filtered;
    std::vector<IIRFilter> ddq_filter, base_dv_filter, base_dw_filter;
    double DT;
    bool is_RobotStateInitialized;

    void calcAccelerationsForInverseDynamics(){
      for(int i=0;i<m_robot->numJoints();i++)q(i) = m_robot->joint(i)->q;
      dq = (q - q_old) / DT;
      ddq = (q - 2 * q_old + q_oldold) / (DT * DT);
      q_oldold = q_old;
      q_old = q;
      const hrp::Vector3 g(0, 0, 9.80665);
      base_p = m_robot->rootLink()->p;
      base_v = (base_p - base_p_old) / DT;
      base_dv = g + (base_p - 2 * base_p_old + base_p_oldold) / (DT * DT);
      base_p_oldold = base_p_old;
      base_p_old = base_p;
      base_R =  m_robot->rootLink()->R;
      base_dR = (base_R - base_R_old) / DT;
      base_w_hat = base_dR * base_R.transpose();
      base_w = hrp::Vector3(base_w_hat(2,1), - base_w_hat(0,2), base_w_hat(1,0));
      base_dw = (base_w - base_w_old) / DT;
      base_R_old = base_R;
      base_w_old = base_w;
    };
    void passAccelerationFilters(){
      for(int i=0;i<m_robot->numJoints();i++)ddq_filtered[i] = ddq_filter[i].passFilter(ddq[i]);
      for(int i=0;i<3;i++){
        base_dv_filtered[i] = base_dv_filter[i].passFilter(base_dv[i]);
        base_dw_filtered[i] = base_dw_filter[i].passFilter(base_dw[i]);
      }
    };
    void setCurrentRobotState(){
      for(int i=0;i<m_robot->numJoints();i++){
        m_robot->joint(i)->dq = dq(i);
        m_robot->joint(i)->ddq = ddq_filtered(i);
      }
      m_robot->rootLink()->vo = base_v - base_w.cross(base_p);
      //static hrp::Vector3 vo_old; m_robot->rootLink()->dvo = g + (m_robot->rootLink()->vo - vo_old)/DT; vo_old = m_robot->rootLink()->vo; // calc in incremental way
      m_robot->rootLink()->dvo = base_dv_filtered - base_dw_filtered.cross(base_p) - base_w.cross(base_v); // calc in differential way
      m_robot->rootLink()->w = base_w;
      m_robot->rootLink()->dw = base_dw_filtered;
    };

  public:
    SimpleFullbodyInverseDynamicsSolver(const hrp::BodyPtr& _robot, const double _dt, const double _acceleration_filter_fc = 25.0)
        : m_robot(_robot),
          DT(_dt),
          is_RobotStateInitialized(false)
    {
      q.resize(m_robot->numJoints());
      q_old.resize(m_robot->numJoints());
      q_oldold.resize(m_robot->numJoints());
      dq.resize(m_robot->numJoints());
      ddq.resize(m_robot->numJoints());
      ddq_filtered.resize(m_robot->numJoints());
      initializeAccelerationFilters(_acceleration_filter_fc, _dt);
    };
    ~SimpleFullbodyInverseDynamicsSolver(){};
    void initializeAccelerationFilters(const double _fc_in, const double _dt){//set up filters as 2nd order Biquad IIR
      ddq_filter.resize(m_robot->numJoints());
      base_dv_filter.resize(3);
      base_dw_filter.resize(3);
      std::vector<double> fb_coeffs(3), ff_coeffs(3);
      const double fc = std::tan(_fc_in * M_PI * _dt) / (2 * M_PI);
      const double denom = 1 + (2 * sqrt(2) * M_PI * fc) + 4 * M_PI * M_PI * fc*fc;
      ff_coeffs[0] = (4 * M_PI * M_PI * fc*fc) / denom;
      ff_coeffs[1] = (8 * M_PI * M_PI * fc*fc) / denom;
      ff_coeffs[2] = (4 * M_PI * M_PI * fc*fc) / denom;
      fb_coeffs[0] = 1.0;
      fb_coeffs[1] = (8 * M_PI * M_PI * fc*fc - 2) / denom;
      fb_coeffs[2] = (1 - (2 * sqrt(2) * M_PI * fc) + 4 * M_PI * M_PI * fc*fc) / denom;
      for(int i=0;i<m_robot->numJoints();i++)ddq_filter[i].setParameter(2,fb_coeffs,ff_coeffs);
      for(int i=0;i<3;i++)base_dv_filter[i].setParameter(2,fb_coeffs,ff_coeffs);
      for(int i=0;i<3;i++)base_dw_filter[i].setParameter(2,fb_coeffs,ff_coeffs);
    };
    void initializeRobotState(){
      for(int i=0;i<m_robot->numJoints();i++)q(i) = m_robot->joint(i)->q;
      q_oldold = q_old = q;
      dq = ddq = ddq_filtered = hrp::dvector::Zero(m_robot->numJoints());
      base_p_oldold = base_p_old = base_p = m_robot->rootLink()->p;
      base_R_old = base_R = m_robot->rootLink()->R;
      base_dR = base_w_hat = hrp::Matrix33::Zero();
      base_w_old = base_w = base_dw = base_dw_filtered = hrp::Vector3::Zero();
      is_RobotStateInitialized = true;
    };
    void solveFullbodyID(hrp::Vector3& f_ans, hrp::Vector3& t_ans){
      if(!is_RobotStateInitialized) initializeRobotState();
      calcAccelerationsForInverseDynamics();
      passAccelerationFilters();
      setCurrentRobotState();
      m_robot->calcForwardKinematics(true,true);// calc every link's acc and vel
      m_robot->calcInverseDynamics(m_robot->rootLink(),f_ans,t_ans);// this returns f,t at the coordinate origin (not at base link pos)
    };
};


#include <iomanip>

#endif //__JOINT_PATH_EX_H__
