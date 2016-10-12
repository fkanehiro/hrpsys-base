/* -*- coding:utf-8-unix; mode:c++; -*- */

#include "ImpedanceOutputGenerator.h"
/* samples */
#include <stdio.h>
#include <cstdio>
#include <iostream>
#include <vector>

#ifndef rad2deg
#define rad2deg(rad) (rad * 180 / M_PI)
#endif
#ifndef deg2rad
#define deg2rad(deg) (deg * M_PI / 180)
#endif

class testImpedanceOutputGenerator
{
protected:
    double dt; /* [s] */
    ImpedanceOutputGenerator imp;
    bool use_gnuplot;
    void gen_pattern_and_plot (const std::vector<hrp::Vector3>& force_diff_vec,
                               const std::vector<hrp::Vector3>& moment_diff_vec,
                               const std::vector<hrp::Vector3>& target_p0_vec,
                               const std::vector<hrp::Matrix33>& target_r0_vec,
                               const std::vector<double>& time_vec)
    {
        parse_params();
        std::string fname("/tmp/plot-imp.dat");
        FILE* fp = fopen(fname.c_str(), "w");
        for (size_t i = 0; i < time_vec.size();i++) {
            imp.target_p0 = target_p0_vec[i];
            imp.target_r0 = target_r0_vec[i];
            imp.current_p1 = imp.output_p1;
            imp.current_r1 = imp.output_r1;
            hrp::Vector3 vel_p, vel_r;
            hrp::Matrix33 eeR = hrp::Matrix33::Identity();
            imp.calcTargetVelocity(vel_p, vel_r,
                                   eeR, force_diff_vec[i], moment_diff_vec[i], dt);
            hrp::Vector3 output_rot, target_rot;
            rats::difference_rotation(output_rot, hrp::Matrix33::Identity(), imp.output_r1);
            rats::difference_rotation(target_rot, hrp::Matrix33::Identity(), imp.target_r1);
            fprintf(fp, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
                    time_vec[i],
                    imp.output_p1(0), imp.output_p1(1), imp.output_p1(2),
                    imp.target_p1(0), imp.target_p1(1), imp.target_p1(2),
                    force_diff_vec[i](0)/imp.K_p, force_diff_vec[i](1)/imp.K_p, force_diff_vec[i](2)/imp.K_p,
                    rad2deg(output_rot(0)), rad2deg(output_rot(1)), rad2deg(output_rot(2)),
                    rad2deg(target_rot(0)), rad2deg(target_rot(1)), rad2deg(target_rot(2)),
                    rad2deg(moment_diff_vec[i](0)/imp.K_r), rad2deg(moment_diff_vec[i](1)/imp.K_r), rad2deg(moment_diff_vec[i](2)/imp.K_r)
                    );
        }
        fclose(fp);
        if (use_gnuplot) {
        // plot
        std::string titles[3] = {"X", "Y", "Z"};
        //   plot pos
        FILE* gp_pos = popen("gnuplot", "w");
        fprintf(gp_pos, "set multiplot layout 3, 1 title 'Pos results'\n");
        for (size_t ii = 0; ii < 3; ii++) {
            fprintf(gp_pos, "set xlabel 'Time [s]'\n");
            fprintf(gp_pos, "set ylabel 'pos %s [m]'\n", titles[ii].c_str());
            fprintf(gp_pos, "plot '/tmp/plot-imp.dat' using 1:%d with lines title 'cur pos(%s)' lw 4, '/tmp/plot-imp.dat' using 1:%d with lines title 'tgt pos(%s)' lw 3, '/tmp/plot-imp.dat' using 1:%d with lines title 'force_diff/K(%s)' lw 2\n",
                    ii+2, titles[ii].c_str(), ii+2+3, titles[ii].c_str(), ii+2+3*2, titles[ii].c_str());
        }
        fflush(gp_pos);
        //   plot rot
        FILE* gp_rot = popen("gnuplot", "w");
        fprintf(gp_rot, "set multiplot layout 3, 1 title 'Rot results'\n");
        for (size_t ii = 0; ii < 3; ii++) {
            fprintf(gp_rot, "set xlabel 'Time [s]'\n");
            fprintf(gp_rot, "set ylabel 'rot %s [deg]'\n", titles[ii].c_str());
            fprintf(gp_rot, "plot '/tmp/plot-imp.dat' using 1:%d with lines title 'cur rot(%s)' lw 4, '/tmp/plot-imp.dat' using 1:%d with lines title 'tgt rot(%s)' lw 3, '/tmp/plot-imp.dat' using 1:%d with lines title 'moment_diff/K(%s)' lw 2\n",
                    ii+2+9, titles[ii].c_str(), ii+2+3+9, titles[ii].c_str(), ii+2+3*2+9, titles[ii].c_str());
        }
        fflush(gp_rot);
        double tmp;
        std::cin >> tmp;
        pclose(gp_pos);
        pclose(gp_rot);
        }
    };
public:
    std::vector<std::string> arg_strs;
    testImpedanceOutputGenerator (const double _dt = 0.004) : dt(_dt), imp(), use_gnuplot(true) {};
    void test0 ()
    {
        std::cerr << "test0 : Set ref force" << std::endl;
        double tm = 0.0, total_tm = 4.0;
        std::vector<double> time_vec;
        std::vector<hrp::Vector3> force_diff_vec, moment_diff_vec, target_p0_vec;
        std::vector<hrp::Matrix33> target_r0_vec;
        for (size_t i = 0; i < static_cast<size_t>(total_tm/dt);i++) {
            time_vec.push_back(tm);
            force_diff_vec.push_back((i*dt < total_tm * 0.2 ? hrp::Vector3::Zero() : hrp::Vector3(10,-20,30)));
            moment_diff_vec.push_back((i*dt < total_tm * 0.3 ? hrp::Vector3::Zero() : hrp::Vector3(5,-10,15)));
            target_p0_vec.push_back(hrp::Vector3::Zero());
            target_r0_vec.push_back(hrp::Matrix33::Identity());
            tm += dt;
        }
        gen_pattern_and_plot (force_diff_vec, moment_diff_vec, target_p0_vec, target_r0_vec, time_vec);
    };
    void test1 ()
    {
        std::cerr << "test1 : Move pos and rot" << std::endl;
        double tm = 0.0, total_tm = 1.0;
        std::vector<double> time_vec;
        std::vector<hrp::Vector3> force_diff_vec, moment_diff_vec, target_p0_vec;
        std::vector<hrp::Matrix33> target_r0_vec;
        //imp.M_p = 0.0; imp.M_r = 0.0;
        for (size_t i = 0; i < static_cast<size_t>(total_tm/dt);i++) {
            time_vec.push_back(tm);
            force_diff_vec.push_back(hrp::Vector3::Zero());
            moment_diff_vec.push_back(hrp::Vector3::Zero());
            double ratio = (i*dt < total_tm * 0.3 ? i*dt/(total_tm * 0.3) : 1.0);
            target_p0_vec.push_back(hrp::Vector3((1-ratio)*hrp::Vector3::Zero()+ratio*hrp::Vector3(0.01,-0.02,0.03)));
            hrp::Vector3 tmpv(hrp::Vector3((1-ratio)*hrp::Vector3::Zero() + ratio*hrp::Vector3(0.1,-0.2,0.3)));
            Eigen::AngleAxis<double> tmpr;
            if (tmpv.norm() != 0.0) {
                tmpr = Eigen::AngleAxis<double>(tmpv.norm(), tmpv.normalized());
            } else {
                tmpr = hrp::Matrix33::Identity();
            }
            target_r0_vec.push_back(tmpr.toRotationMatrix());
            tm += dt;
        }
        gen_pattern_and_plot (force_diff_vec, moment_diff_vec, target_p0_vec, target_r0_vec, time_vec);
    };
    void parse_params ()
    {
      for (unsigned int i = 0; i < arg_strs.size(); ++ i) {
          if ( arg_strs[i]== "--use-gnuplot" ) {
              if (++i < arg_strs.size()) use_gnuplot = (arg_strs[i]=="true");
          }
      }
    };
};

void print_usage ()
{
    std::cerr << "Usage : testImpedanceOutputGenerator [option]" << std::endl;
    std::cerr << " [option] should be:" << std::endl;
    std::cerr << "  --test0 : Set ref force" << std::endl;
    std::cerr << "  --test1 : Move pos and rot" << std::endl;
};

int main(int argc, char* argv[])
{
    int ret = 0;
    if (argc >= 2) {
        testImpedanceOutputGenerator tiog;
        for (int i = 1; i < argc; ++ i) {
          tiog.arg_strs.push_back(std::string(argv[i]));
        }
        if (std::string(argv[1]) == "--test0") {
            tiog.test0();
        } else if (std::string(argv[1]) == "--test1") {
            tiog.test1();
        } else {
            print_usage();
            ret = 1;
        }
    } else {
        print_usage();
        ret = 1;
    }
    return ret;
}

