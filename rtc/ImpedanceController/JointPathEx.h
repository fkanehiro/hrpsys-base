#ifndef __JOINT_PATH_EX_H__
#define __JOINT_PATH_EX_H__
#include <hrpModel/Body.h>
#include <hrpModel/Link.h>
#include <hrpModel/JointPath.h>

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
    bool calcInverseKinematics2Loop(const Vector3& dp, const Vector3& omega, dvector &dq);
    bool calcInverseKinematics2(const Vector3& end_p, const Matrix33& end_R);
    double getSRGain() { return sr_gain; }
    bool setSRGain(double g) { sr_gain = g; }
    double getManipulabilityLimit() { return manipulability_limit; }
    bool setManipulabilityLimit(double l) { manipulability_limit = l; }
    bool setManipulabilityGain(double l) { manipulability_gain = l; }
    void solveLimbIK (const hrp::Vector3& _vel_p,
                      const hrp::Vector3& _vel_r,
                      const int transition_count,
                      const double avoid_gain,
                      const double reference_gain,
                      const double MAX_TRANSITION_COUNT,
                      const hrp::dvector& qrefv,
                      bool DEBUGP = false);
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

};

#include <iomanip>

#endif //__JOINT_PATH_EX_H__
