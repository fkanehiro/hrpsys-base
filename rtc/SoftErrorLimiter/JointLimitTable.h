#ifndef __JOINT_LIMIT_TABLE_H__
#define __JOINT_LIMIT_TABLE_H__
#include <hrpModel/Body.h>
#include <hrpModel/Link.h>
#include <cmath>
#include <coil/stringutil.h>

namespace hrp {
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

#endif //__JOINT_LIMIT_TABLE_H__
