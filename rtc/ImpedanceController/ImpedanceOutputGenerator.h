#ifndef IMPEDANCETRGETGENERATOR_H
#define IMPEDANCETRGETGENERATOR_H

#include "RatsMatrix.h"

struct ImpedanceOutputGenerator
{
    hrp::Vector3 target_p0, target_p1, target_p2, current_p0, current_p1, current_p2;
    hrp::Matrix33 target_r0, target_r1, target_r2, current_r0, current_r1, current_r2;
    double M_p, D_p, K_p;
    double M_r, D_r, K_r;
    hrp::Matrix33 force_gain, moment_gain;

    ImpedanceOutputGenerator ()
    : target_p0(hrp::Vector3::Zero()), target_p1(hrp::Vector3::Zero()), target_p2(hrp::Vector3::Zero()),
      current_p0(hrp::Vector3::Zero()), current_p1(hrp::Vector3::Zero()), current_p2(hrp::Vector3::Zero()),
      target_r0(hrp::Matrix33::Identity()), target_r1(hrp::Matrix33::Identity()), target_r2(hrp::Matrix33::Identity()),
      current_r0(hrp::Matrix33::Identity()), current_r1(hrp::Matrix33::Identity()), current_r2(hrp::Matrix33::Identity()),
      M_p(10), D_p(200), K_p(400), M_r(5), D_r(100), K_r(200),
      force_gain(hrp::Matrix33::Identity()), moment_gain(hrp::Matrix33::Identity())
    {};
    const hrp::Matrix33& getOutputRot () { return current_r1; };
    const hrp::Vector3& getOutputPos () { return current_p1; };
    void resetPreviousTargetParam ()
    {
        target_p1 = target_p0;
        target_p2 = target_p1;
        target_r1 = target_r0;
        target_r2 = target_r1;
    };
    void resetPreviousCurrentParam ()
    {
        current_p1 = current_p0;
        current_p2 = current_p1;
        current_r1 = current_r0;
        current_r2 = current_r1;
    };
    void calcTargetVelocity (hrp::Vector3& vel_p, hrp::Vector3& vel_r,
                             const hrp::Matrix33& eeR,
                             const hrp::Vector3& force_diff, const hrp::Vector3& moment_diff,
                             const double _dt, const bool printp = false, const std::string& print_str = "", const std::string& ee_name = "")
    {
        if ( printp ) {
            std::cerr << "[" << print_str << "] impedance calc [" << ee_name << "]" << std::endl;
            std::cerr << "[" << print_str << "]   cur0 = " << current_p0.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
            std::cerr << "[" << print_str << "]   cur1 = " << current_p1.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
            std::cerr << "[" << print_str << "]   cur2 = " << current_p2.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
            std::cerr << "[" << print_str << "]   tgt0 = " << target_p0.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
            std::cerr << "[" << print_str << "]   tgt1 = " << target_p1.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
        }

        hrp::Vector3 dif_pos = hrp::Vector3(0,0,0);
        hrp::Vector3 vel_pos0 = hrp::Vector3(0,0,0);
        hrp::Vector3 vel_pos1 = hrp::Vector3(0,0,0);
        hrp::Vector3 dif_target_pos = hrp::Vector3(0,0,0);
        hrp::Vector3 dif_rot = hrp::Vector3(0,0,0);
        hrp::Vector3 vel_rot0 = hrp::Vector3(0,0,0);
        hrp::Vector3 vel_rot1 = hrp::Vector3(0,0,0);
        hrp::Vector3 dif_target_rot = hrp::Vector3(0,0,0);

        // rats/plugins/impedancecontrol.cpp
        //double M = 5, D = 100, K = 200;
        // dif_pos  = target_p0 (target_coords0) - current_p0(move_coords)
        // vel_pos0 = current_p0(move_coors) - current_p1(prev_coords0)
        // vel_pos1 = current_p1(prev_coords0) - current_p2(prev_coords1)
        // dif_target  = target_p0(target_coords0) - target_p1(target_coords1)
        //
        // current_p2(prev_coords1) = current_p1(prev_coords0)
        // currnet_p1(prev_coords0) = current_p0(move_coords) + vel_p
        // target_p1(target_coords1) = target_p0(target_coords0)

        dif_pos  = target_p0 - current_p0;
        vel_pos0 = current_p0 - current_p1;
        vel_pos1 = current_p1 - current_p2;
        dif_target_pos = target_p0 - target_p1;

        rats::difference_rotation(dif_rot, current_r0, target_r0);
        rats::difference_rotation(vel_rot0, current_r1, current_r0);
        rats::difference_rotation(vel_rot1, current_r2, current_r1);
        rats::difference_rotation(dif_target_rot, target_r1, target_r0);

        if ( printp ) {
            std::cerr << "[" << print_str << "]   dif_p  = " << dif_pos.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
            std::cerr << "[" << print_str << "]   vel_p0 = " << vel_pos0.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
            std::cerr << "[" << print_str << "]   vel_p1 = " << vel_pos1.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
            std::cerr << "[" << print_str << "]   dif_t  = " << dif_target_pos.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
            std::cerr << "[" << print_str << "]   dif_r  = " << dif_rot.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[rad]" << std::endl;
            std::cerr << "[" << print_str << "]   vel_r0 = " << vel_rot0.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[rad]" << std::endl;
            std::cerr << "[" << print_str << "]   vel_r1 = " << vel_rot1.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[rad]" << std::endl;
            std::cerr << "[" << print_str << "]   dif_t  = " << dif_target_rot.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[rad]" << std::endl;
        }
        vel_p =  ( eeR * (force_gain * (eeR.transpose() * force_diff)) * _dt * _dt
                   + M_p * ( vel_pos1 - vel_pos0 )
                   + D_p * ( dif_target_pos - vel_pos0 ) * _dt
                   + K_p * ( dif_pos * _dt * _dt ) ) /
            (M_p + (D_p * _dt) + (K_p * _dt * _dt));
        vel_r =  ( eeR * (moment_gain * (eeR.transpose() * moment_diff)) * _dt * _dt
                   + M_r * ( vel_rot1 - vel_rot0 )
                   + D_r * ( dif_target_rot - vel_rot0 ) * _dt
                   + K_r * ( dif_rot * _dt * _dt  ) ) /
            (M_r + (D_r * _dt) + (K_r * _dt * _dt));
        // generate smooth motion just after impedance started
        if ( printp ) {
            std::cerr << "[" << print_str << "]   vel_p  = " << vel_p.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
            std::cerr << "[" << print_str << "]   vel_r  = " << vel_r.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[rad]" << std::endl;
        }

        // Spin time-related parameter
        current_p2 = current_p1;
        current_r2 = current_r1;

        current_p1 = current_p0 + vel_p;
        // if ( std::fabs(vel_r.norm() - 0.0) < ::std::numeric_limits<double>::epsilon() ) {
        if ( vel_r.norm() != 0.0 ) {
            hrp::Matrix33 tmpm;
            Eigen::AngleAxis<double> tmpr(vel_r.norm(), vel_r.normalized());
            rats::rotm3times(tmpm, tmpr.toRotationMatrix(), current_r0);
            current_r1 = tmpm;
        } else {
            current_r1 = current_r0;
        }

        target_p1 = target_p0;
        target_r1 = target_r0;
    };
    void calcTargetVelocityNew (hrp::Vector3& vel_p, hrp::Vector3& vel_r,
                                const hrp::Matrix33& eeR,
                                const hrp::Vector3& force_diff, const hrp::Vector3& moment_diff,
                                const double _dt, const bool printp = false, const std::string& print_str = "", const std::string& ee_name = "")
    {
        if ( printp ) {
            std::cerr << "[" << print_str << "] impedance calc [" << ee_name << "]" << std::endl;
            std::cerr << "[" << print_str << "]   cur0 = " << current_p0.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
            std::cerr << "[" << print_str << "]   cur1 = " << current_p1.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
            std::cerr << "[" << print_str << "]   cur2 = " << current_p2.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
            std::cerr << "[" << print_str << "]   tgt0 = " << target_p0.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
            std::cerr << "[" << print_str << "]   tgt1 = " << target_p1.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
            std::cerr << "[" << print_str << "]   tgt2 = " << target_p2.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
        }
        hrp::Vector3 dif_pos0, dif_rot0, dif_pos1, dif_rot1;
        hrp::Vector3 err_p, err_r;
        dif_pos0 = current_p1 - target_p1;
        rats::difference_rotation(dif_rot0, target_r1, current_r1);
        dif_pos1 = current_p2 - target_p2;
        rats::difference_rotation(dif_rot1, target_r2, current_r2);
        err_p = target_p0 - current_p0;
        rats::difference_rotation(err_r, current_r0, target_r0);
        vel_p =  ( eeR * (force_gain * (eeR.transpose() * force_diff)) * _dt * _dt
                   + (2 * M_p + D_p * _dt) * dif_pos0
                   - M_p * dif_pos1) /
            (M_p + (D_p * _dt) + (K_p * _dt * _dt))
            + err_p;
        vel_r =  ( eeR * (moment_gain * (eeR.transpose() * moment_diff)) * _dt * _dt
                   + (2 * M_r + D_r * _dt) * dif_rot0
                   - M_r * dif_rot1) /
            (M_r + (D_r * _dt) + (K_r * _dt * _dt))
            + err_r;
        // generate smooth motion just after impedance started
        if ( printp ) {
            std::cerr << "[" << print_str << "]   vel_p  = " << vel_p.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
            std::cerr << "[" << print_str << "]   vel_r  = " << vel_r.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[rad]" << std::endl;
        }

        // Spin time-related parameter
        current_p2 = current_p1;
        current_r2 = current_r1;

        current_p1 = current_p0 + vel_p;
        // if ( std::fabs(vel_r.norm() - 0.0) < ::std::numeric_limits<double>::epsilon() ) {
        if ( vel_r.norm() != 0.0 ) {
            hrp::Matrix33 tmpm;
            Eigen::AngleAxis<double> tmpr(vel_r.norm(), vel_r.normalized());
            rats::rotm3times(tmpm, tmpr.toRotationMatrix(), current_r0);
            current_r1 = tmpm;
        } else {
            current_r1 = current_r0;
        }

        target_p2 = target_p1;
        target_r2 = target_r1;
        target_p1 = target_p0;
        target_r1 = target_r0;
    };
};
#endif // IMPEDANCETRGETGENERATOR_H
