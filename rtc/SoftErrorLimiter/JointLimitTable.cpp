#include "JointLimitTable.h"
#include <iostream>
#include <iomanip>
#include <limits.h>
#include <float.h>

double hrp::JointLimitTable::getInterpolatedLimitAngle(
    const double target_joint_angle, const bool is_llimit_angle) const {
  double target_angle = target_joint_angle * 180.0 / M_PI;  // [rad]=>[deg]
  int int_target_angle = static_cast<int>(std::floor(target_angle));
  int target_range[2] = {int_target_angle, 1 + int_target_angle};
  double self_joint_range[2];
  for (size_t i = 0; i < 2; i++) {
    size_t idx = std::min(std::max(target_llimit_angle, target_range[i]),
                          target_ulimit_angle) -
                 target_llimit_angle;
    self_joint_range[i] =
        (is_llimit_angle ? llimit_table(idx) : ulimit_table(idx));
  }
  double tmp_ratio = target_angle - int_target_angle;
  return (self_joint_range[0] * (1 - tmp_ratio) +
          self_joint_range[1] * tmp_ratio) *
         M_PI / 180.0;  // [deg]=>[rad]
};

void hrp::readJointLimitTableFromProperties(
    std::map<std::string, hrp::JointLimitTable>& joint_limit_tables,
    hrp::BodyPtr m_robot, const std::string& prop_string,
    const std::string& instance_name) {
  if (prop_string != "") {
    coil::vstring limit_tables = coil::split(prop_string, ":");
    size_t limit_table_size =
        6;  // self_joint_name:target_joint_name:target_min_angle:target_max_angle:min:max
    size_t num_limit_table = limit_tables.size() / limit_table_size;
    std::cerr << "[" << instance_name << "] Load joint limit table ["
              << num_limit_table << "]" << std::endl;
    for (size_t i = 0; i < num_limit_table; i++) {
      size_t start_idx = i * limit_table_size;
      int target_llimit_angle, target_ulimit_angle;
      coil::stringTo(target_llimit_angle, limit_tables[start_idx + 2].c_str());
      coil::stringTo(target_ulimit_angle, limit_tables[start_idx + 3].c_str());
      coil::vstring llimit_str_v =
          coil::split(limit_tables[start_idx + 4], ",");
      coil::vstring ulimit_str_v =
          coil::split(limit_tables[start_idx + 5], ",");
      hrp::dvector llimit_table(llimit_str_v.size()),
          ulimit_table(ulimit_str_v.size());
      int target_jointId = -1;
      for (size_t j = 0; j < m_robot->numJoints(); j++) {
        if (m_robot->joint(j)->name == limit_tables[start_idx + 1])
          target_jointId = m_robot->joint(j)->jointId;
      }
      if (llimit_str_v.size() != ulimit_str_v.size() || target_jointId == -1) {
        std::cerr << "[" << instance_name << "] " << limit_tables[start_idx + 0]
                  << ":" << limit_tables[start_idx + 1] << " fail" << std::endl;
      } else {
        std::cerr << "[" << instance_name << "] " << limit_tables[start_idx + 0]
                  << ":" << limit_tables[start_idx + 1] << "(" << target_jointId
                  << ")" << std::endl;
        std::cerr << "[" << instance_name << "]   target_llimit_angle "
                  << limit_tables[start_idx + 2]
                  << "[deg], target_ulimit_angle "
                  << limit_tables[start_idx + 3] << "[deg]" << std::endl;
        std::cerr << "[" << instance_name << "]   llimit_table[deg] "
                  << limit_tables[start_idx + 4] << std::endl;
        std::cerr << "[" << instance_name << "]   ulimit_table[deg] "
                  << limit_tables[start_idx + 5] << std::endl;
        for (int j = 0; j < llimit_table.size(); j++) {
          coil::stringTo(llimit_table[j], llimit_str_v[j].c_str());
          coil::stringTo(ulimit_table[j], ulimit_str_v[j].c_str());
        }
        joint_limit_tables.insert(std::pair<std::string, hrp::JointLimitTable>(
            limit_tables[start_idx],
            hrp::JointLimitTable(target_jointId, target_llimit_angle,
                                 target_ulimit_angle, llimit_table,
                                 ulimit_table)));
      }
    }
  } else {
    std::cerr << "[" << instance_name << "] Do not load joint limit table"
              << std::endl;
  }
};
