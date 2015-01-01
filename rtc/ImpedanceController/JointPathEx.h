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
    JointPathEx(BodyPtr& robot, Link* base, Link* end);
    bool calcJacobianInverseNullspace(dmatrix &J, dmatrix &Jinv, dmatrix &Jnull);
    bool calcInverseKinematics2Loop(const Vector3& dp, const Vector3& omega, const double LAMBDA, const double avoid_gain = 0.0, const double reference_gain = 0.0, const dvector* reference_q = NULL);
    bool calcInverseKinematics2(const Vector3& end_p, const Matrix33& end_R, const double avoid_gain = 0.0, const double reference_gain = 0.0, const dvector* reference_q = NULL);
    double getSRGain() { return sr_gain; }
    bool setSRGain(double g) { sr_gain = g; }
    double getManipulabilityLimit() { return manipulability_limit; }
    bool setManipulabilityLimit(double l) { manipulability_limit = l; }
    bool setManipulabilityGain(double l) { manipulability_gain = l; }
    void setMaxIKError(double epos, double erot);
    void setMaxIKError(double e);
    void setMaxIKIteration(int iter);
  protected:
        double maxIKPosErrorSqr, maxIKRotErrorSqr;
        int maxIKIteration;
        std::vector<Link*> joints;
        std::vector<double> avoid_weight_gain;
	double sr_gain, manipulability_limit, manipulability_gain;
    };

    typedef boost::shared_ptr<JointPathEx> JointPathExPtr;

    // JointLimitTable for one joint
    //   self_joint   : a joint to obtain llimit and ulimit from this class.
    //   target_joint : self_joint's limit is difference for target_joint's joint angle.
    class JointLimitTable {
    private:
        int target_jointId; // jointId for target_joint
        int target_llimit_angle, target_ulimit_angle; // llimit and ulimit angle [deg] for target_joint
        hrp::dvector llimit_table, ulimit_table; // Tables for self_joint's llimit and ulimit
        double getInterpolatedLimitAngle (const double target_joint_angle, const bool is_llimit_angle) const;
    public:
        JointLimitTable (const int _target_jointId,
                         const int _target_llimit_angle, const int _target_ulimit_angle,
                         const hrp::dvector& _llimit_table, const hrp::dvector& _ulimit_table)
            : target_jointId(_target_jointId), target_llimit_angle(_target_llimit_angle), target_ulimit_angle(_target_ulimit_angle), llimit_table(_llimit_table), ulimit_table(_ulimit_table) {};
        ~JointLimitTable() {};
        int getTargetJointId () const { return target_jointId; };
        double getLlimit (const double target_joint_angle) const // [rad]
        {
            return getInterpolatedLimitAngle(target_joint_angle, true); // [rad]
        };
        double getUlimit (const double target_joint_angle) const // [rad]
        {
            return getInterpolatedLimitAngle(target_joint_angle, false); // [rad]
        };
    };

    void readJointLimitTableFromProperties (std::map<std::string, hrp::JointLimitTable>& joint_mm_tables,
                                            hrp::BodyPtr m_robot,
                                            const std::string& prop_string,
                                            const std::string& instance_name);
};

#include <iomanip>

#endif //__JOINT_PATH_EX_H__
