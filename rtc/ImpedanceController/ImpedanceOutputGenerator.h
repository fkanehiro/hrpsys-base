#ifndef IMPEDANCETRGETGENERATOR_H
#define IMPEDANCETRGETGENERATOR_H

#include "RatsMatrix.h"

struct ImpedanceOutputGenerator
{
    // target  : Target pos and rot from SequencePlayer and StateHolder
    // current : Current ee pos and rot (IK result)
    // output  : Output from ImpedanceOutput which response is characterized by MDK
    // index 0 : t+dt. Values in current onExecute.
    // index 1 : t. Values in previous onExecute.
    // index 2 : t-dt. Values in previous previous onExecute.
    hrp::Vector3 target_p0, target_p1, target_p2, current_p1, output_p1, output_p2;
    hrp::Matrix33 target_r0, target_r1, target_r2, current_r1, output_r1, output_r2;
    double M_p, D_p, K_p;
    double M_r, D_r, K_r;
    hrp::Matrix33 force_gain, moment_gain;

    ImpedanceOutputGenerator ()
    : target_p0(hrp::Vector3::Zero()), target_p1(hrp::Vector3::Zero()), target_p2(hrp::Vector3::Zero()),
      current_p1(hrp::Vector3::Zero()),  output_p1(hrp::Vector3::Zero()), output_p2(hrp::Vector3::Zero()),
      target_r0(hrp::Matrix33::Identity()), target_r1(hrp::Matrix33::Identity()), target_r2(hrp::Matrix33::Identity()),
      current_r1(hrp::Matrix33::Identity()), output_r1(hrp::Matrix33::Identity()), output_r2(hrp::Matrix33::Identity()),
      M_p(10), D_p(200), K_p(400), M_r(5), D_r(100), K_r(200),
      force_gain(hrp::Matrix33::Identity()), moment_gain(hrp::Matrix33::Identity())
    {};
    const hrp::Matrix33& getOutputRot () { return output_r1; };
    const hrp::Vector3& getOutputPos () { return output_p1; };
    void resetPreviousTargetParam ()
    {
        target_p1 = target_p0;
        target_p2 = target_p1;
        target_r1 = target_r0;
        target_r2 = target_r1;
    };
    void resetPreviousCurrentParam ()
    {
        output_p1 = current_p1;
        output_p2 = output_p1;
        output_r1 = current_r1;
        output_r2 = output_r1;
    };
    void calcTargetVelocityOrg (hrp::Vector3& vel_p, hrp::Vector3& vel_r,
                             const hrp::Matrix33& eeR,
                             const hrp::Vector3& force_diff, const hrp::Vector3& moment_diff,
                             const double _dt, const bool printp = false, const std::string& print_str = "", const std::string& ee_name = "")
    {
        if ( printp ) {
            std::cerr << "[" << print_str << "] impedance calc [" << ee_name << "] (Old version)" << std::endl;
            std::cerr << "[" << print_str << "]   cur1 = " << current_p1.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
            std::cerr << "[" << print_str << "]   out1 = " << output_p1.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
            std::cerr << "[" << print_str << "]   out2 = " << output_p2.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
            std::cerr << "[" << print_str << "]   tgt0 = " << target_p0.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
            std::cerr << "[" << print_str << "]   tgt1 = " << target_p1.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
        }

        hrp::Vector3 dif_output_pos, dif_target_pos, vel_target_pos, err_pos;
        hrp::Vector3 dif_output_rot, dif_target_rot, vel_target_rot, err_rot;

        dif_output_pos = output_p1 - output_p2;
        dif_target_pos = target_p0 - target_p1;
        vel_target_pos = target_p0 - current_p1;
        err_pos = output_p1 - current_p1;

        rats::difference_rotation(dif_output_rot, output_r2, output_r1);
        rats::difference_rotation(dif_target_rot, target_r1, target_r0);
        rats::difference_rotation(vel_target_rot, current_r1, target_r0);
        rats::difference_rotation(err_rot, current_r1, output_r1);

        if ( printp ) {
            std::cerr << "[" << print_str << "]   dif_out_p  = " << dif_output_pos.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
            std::cerr << "[" << print_str << "]   dif_tgt_p = " << dif_target_pos.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
            std::cerr << "[" << print_str << "]   vel_tgt_p = " << vel_target_pos.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
            std::cerr << "[" << print_str << "]   err_pos   = " << err_pos.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
            std::cerr << "[" << print_str << "]   dif_out_r  = " << dif_output_rot.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[rad]" << std::endl;
            std::cerr << "[" << print_str << "]   dif_tgt_r = " << dif_target_rot.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[rad]" << std::endl;
            std::cerr << "[" << print_str << "]   vel_tgt_r = " << vel_target_rot.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[rad]" << std::endl;
            std::cerr << "[" << print_str << "]   err_rot   = " << err_rot.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[rad]" << std::endl;
        }
        vel_p =  ( eeR * (force_gain * (eeR.transpose() * force_diff)) * _dt * _dt
                   + M_p * ( dif_output_pos + err_pos )
                   + D_p * ( dif_target_pos + err_pos ) * _dt
                   + K_p * ( vel_target_pos * _dt * _dt ) ) /
            (M_p + (D_p * _dt) + (K_p * _dt * _dt));
        vel_r =  ( eeR * (moment_gain * (eeR.transpose() * moment_diff)) * _dt * _dt
                   + M_r * ( dif_output_rot + err_rot )
                   + D_r * ( dif_target_rot + err_rot ) * _dt
                   + K_r * ( vel_target_rot * _dt * _dt  ) ) /
            (M_r + (D_r * _dt) + (K_r * _dt * _dt));
        // generate smooth motion just after impedance started
        if ( printp ) {
            std::cerr << "[" << print_str << "]   vel_p  = " << vel_p.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
            std::cerr << "[" << print_str << "]   vel_r  = " << vel_r.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[rad]" << std::endl;
        }

        // Spin time-related parameter
        output_p2 = output_p1;
        output_r2 = output_r1;

        output_p1 = current_p1 + vel_p;
        // if ( std::fabs(vel_r.norm() - 0.0) < ::std::numeric_limits<double>::epsilon() ) {
        if ( vel_r.norm() != 0.0 ) {
            hrp::Matrix33 tmpm;
            Eigen::AngleAxis<double> tmpr(vel_r.norm(), vel_r.normalized());
            rats::rotm3times(tmpm, tmpr.toRotationMatrix(), current_r1);
            output_r1 = tmpm;
        } else {
            output_r1 = current_r1;
        }

        target_p1 = target_p0;
        target_r1 = target_r0;
    };
    void calcTargetVelocity (hrp::Vector3& vel_p, hrp::Vector3& vel_r,
                                const hrp::Matrix33& eeR,
                                const hrp::Vector3& force_diff, const hrp::Vector3& moment_diff,
                                const double _dt, const bool printp = false, const std::string& print_str = "", const std::string& ee_name = "")
    {
        if ( printp ) {
            std::cerr << "[" << print_str << "] impedance calc [" << ee_name << "] (New version)" << std::endl;
            std::cerr << "[" << print_str << "]   cur1 = " << current_p1.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
            std::cerr << "[" << print_str << "]   out1 = " << output_p1.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
            std::cerr << "[" << print_str << "]   out2 = " << output_p2.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
            std::cerr << "[" << print_str << "]   tgt0 = " << target_p0.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
            std::cerr << "[" << print_str << "]   tgt1 = " << target_p1.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
            std::cerr << "[" << print_str << "]   tgt2 = " << target_p2.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
        }
        hrp::Vector3 dif_pos1, dif_rot1, dif_pos2, dif_rot2, vel_target_pos, vel_target_rot;
        dif_pos1 = output_p1 - target_p1;
        dif_pos2 = output_p2 - target_p2;
        vel_target_pos = target_p0 - current_p1;
        rats::difference_rotation(dif_rot1, target_r1, output_r1);
        rats::difference_rotation(dif_rot2, target_r2, output_r2);
        rats::difference_rotation(vel_target_rot, current_r1, target_r0);
        vel_p =  ( eeR * (force_gain * (eeR.transpose() * force_diff)) * _dt * _dt
                   + (2 * M_p + D_p * _dt) * dif_pos1
                   - M_p * dif_pos2) /
            (M_p + (D_p * _dt) + (K_p * _dt * _dt))
            + vel_target_pos;
        vel_r =  ( eeR * (moment_gain * (eeR.transpose() * moment_diff)) * _dt * _dt
                   + (2 * M_r + D_r * _dt) * dif_rot1
                   - M_r * dif_rot2) /
            (M_r + (D_r * _dt) + (K_r * _dt * _dt))
            + vel_target_rot;
        // generate smooth motion just after impedance started
        if ( printp ) {
            std::cerr << "[" << print_str << "]   vel_p  = " << vel_p.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
            std::cerr << "[" << print_str << "]   vel_r  = " << vel_r.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[rad]" << std::endl;
        }

        // Spin time-related parameter
        output_p2 = output_p1;
        output_r2 = output_r1;

        output_p1 = current_p1 + vel_p;
        // if ( std::fabs(vel_r.norm() - 0.0) < ::std::numeric_limits<double>::epsilon() ) {
        if ( vel_r.norm() != 0.0 ) {
            hrp::Matrix33 tmpm;
            Eigen::AngleAxis<double> tmpr(vel_r.norm(), vel_r.normalized());
            rats::rotm3times(tmpm, tmpr.toRotationMatrix(), current_r1);
            output_r1 = tmpm;
        } else {
            output_r1 = current_r1;
        }

        target_p2 = target_p1;
        target_r2 = target_r1;
        target_p1 = target_p0;
        target_r1 = target_r0;
    };
};
#endif // IMPEDANCETRGETGENERATOR_H
