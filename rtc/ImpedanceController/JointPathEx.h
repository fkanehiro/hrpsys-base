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
    JointPathEx(BodyPtr& robot, Link* base, Link* end, double control_cycle, bool _use_inside_joint_weight_retrieval = true);
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
    void setOptionalWeightVector(const std::vector<double>& _opt_w)
    {
        for (int i = 0 ; i < numJoints(); i++ ) {
            optional_weight_vector[i] = _opt_w[i];
        }
    };
    void getOptionalWeightVector(std::vector<double>& _opt_w)
    {
        for (int i = 0 ; i < numJoints(); i++ ) {
            _opt_w[i] = optional_weight_vector[i];
        }
    };
  protected:
        double maxIKPosErrorSqr, maxIKRotErrorSqr;
        int maxIKIteration;
        std::vector<Link*> joints;
        std::vector<double> avoid_weight_gain, optional_weight_vector;
        double sr_gain, manipulability_limit, manipulability_gain, dt;
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
};

#include <iomanip>

#endif //__JOINT_PATH_EX_H__
