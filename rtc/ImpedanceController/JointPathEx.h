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
    double getSRGain() { return sr_gain; }
    bool setSRGain(double g) { sr_gain = g; return true; }
    double getManipulabilityLimit() { return manipulability_limit; }
    bool setManipulabilityLimit(double l) { manipulability_limit = l; return true; }
    bool setManipulabilityGain(double l) { manipulability_gain = l; return true; }
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


namespace hrp {
  class InvDynStateBuffer{
    public:
      int N_DOF;
      bool is_initialized;
      double DT;
      hrp::dvector q, q_old, q_oldold, dq, ddq;
      hrp::Vector3 base_p, base_p_old, base_p_oldold, base_v, base_dv;
      hrp::Matrix33 base_R, base_R_old, base_dR, base_w_hat;
      hrp::Vector3 base_w, base_w_old, base_dw;
      InvDynStateBuffer():is_initialized(false){};
      ~InvDynStateBuffer(){};
      void setInitState(const hrp::BodyPtr _m_robot, const double _dt){
        N_DOF = _m_robot->numJoints();
        DT = _dt;
        q.resize(N_DOF);
        q_old.resize(N_DOF);
        q_oldold.resize(N_DOF);
        dq.resize(N_DOF);
        ddq.resize(N_DOF);
        for(int i=0;i<N_DOF;i++)q(i) = _m_robot->joint(i)->q;
        q_oldold = q_old = q;
        dq = ddq = hrp::dvector::Zero(N_DOF);
        base_p_oldold = base_p_old = base_p = _m_robot->rootLink()->p;
        base_R_old = base_R = _m_robot->rootLink()->R;
        base_dR = base_w_hat = hrp::Matrix33::Zero();
        base_w_old = base_w = base_dw = hrp::Vector3::Zero();
        is_initialized = true;
      };
  };
  // set current Body q,base_p,base_R into InvDynStateBuffer and update vel and acc
  void calcAccelerationsForInverseDynamics(const hrp::BodyPtr _m_robot, InvDynStateBuffer& _idsb);
  // set all vel and acc into Body, and call Body::calcInverseDynamics()
  void calcRootLinkWrenchFromInverseDynamics(hrp::BodyPtr _m_robot, InvDynStateBuffer& _idsb, hrp::Vector3& _f_ans, hrp::Vector3& _t_ans);
  // call calcRootLinkWrenchFromInverseDynamics() and convert f,tau into ZMP
  void calcWorldZMPFromInverseDynamics(hrp::BodyPtr _m_robot, InvDynStateBuffer& _idsb, hrp::Vector3& _zmp_ans);
  // increment InvDynStateBuffer for 1 step
  void updateInvDynStateBuffer(InvDynStateBuffer& _idsb);
}

#include <iomanip>

#endif //__JOINT_PATH_EX_H__
