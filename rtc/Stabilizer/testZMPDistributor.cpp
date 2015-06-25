/* -*- coding:utf-8-unix; mode:c++; -*- */

#include "ZMPDistributor.h"
/* samples */
#include <stdio.h>
#include <cstdio>
#include <iostream>

class testZMPDistributor
{
protected:
    double dt; // [s]
    double total_fz; // [N]
    SimpleZMPDistributor* szd;
    std::vector<hrp::Vector3> ee_pos;
    std::vector<hrp::Vector3> cop_pos;
    std::vector<std::vector<Eigen::Vector2d> > fs;
private:
public:
    testZMPDistributor() {};
    virtual ~testZMPDistributor()
    {
        if (szd != NULL) {
            delete szd;
            szd = NULL;
        }
    };

    void gen_and_plot ()
    {
        std::vector<hrp::Matrix33> ee_rot;
        hrp::Matrix33 tmpr;
        tmpr = hrp::rotFromRpy(hrp::Vector3(0,0,0*M_PI/180.0));
        ee_rot.push_back(tmpr);
        ee_rot.push_back(tmpr);
        szd->print_params(std::string(""));
        //
        std::vector<double> x_vec;
        x_vec.push_back(0.0);
        x_vec.push_back(0.05);
        x_vec.push_back(0.1);
        std::vector<double> y_vec;
        double tyy = -0.1;
        while (tyy < 0.11) {
            y_vec.push_back(tyy);
            tyy += 0.01;
        };
        std::vector<hrp::Vector3> refzmp_vec;
        for (size_t xx = 0; xx < x_vec.size(); xx++) {
            for (size_t yy = 0; yy < y_vec.size(); yy++) {
                refzmp_vec.push_back(hrp::Vector3(x_vec[xx], y_vec[yy], 0.0));
            }
        }
        //

        // plot
        FILE* gp = popen("gnuplot", "w");
        FILE* gp_m = popen("gnuplot", "w");
        FILE* gp_f = popen("gnuplot", "w");
        FILE* gp_a = popen("gnuplot", "w");
        std::string fname_fm("/tmp/plot-fm.dat");
        FILE* fp_fm = fopen(fname_fm.c_str(), "w");
        std::vector<std::string> names;
        names.push_back("rleg");
        names.push_back("lleg");
        hrp::Vector3 ref_foot_force[2], ref_foot_moment[2];
        std::vector<std::vector<Eigen::Vector2d> > fs;
        szd->get_vertices(fs);
        for (size_t i = 0; i < refzmp_vec.size(); i++) {
            double alpha = szd->calcAlpha(refzmp_vec[i], ee_pos, ee_rot);
            szd->distributeZMPToForceMoments(ref_foot_force, ref_foot_moment,
                                             //szd->distributeZMPToForceMomentsQP(ref_foot_force, ref_foot_moment,
                                             ee_pos, cop_pos, ee_rot,
                                             refzmp_vec[i], refzmp_vec[i],
                                             total_fz, 0.004);
            for (size_t j = 0; j < fs.size(); j++) {
                std::string fname("/tmp/plot"+names[j]+".dat");
                FILE* fp = fopen(fname.c_str(), "w");
                for (size_t i = 0; i < fs[j].size(); i++) {
                    hrp::Vector3 tmpf(hrp::Vector3(ee_rot[j] * hrp::Vector3(fs[j][i](0), fs[j][i](1), 0) + ee_pos[j]));
                    fprintf(fp, "%f %f %f\n", tmpf(0), tmpf(1), tmpf(2));
                }
                hrp::Vector3 tmpf(hrp::Vector3(ee_rot[j] * hrp::Vector3(fs[j][0](0), fs[j][0](1), 0) + ee_pos[j]));
                fprintf(fp, "%f %f %f\n", tmpf(0), tmpf(1), tmpf(2));
                fclose(fp);
            }
            for (size_t j = 0; j < fs.size(); j++) {
                std::string fname("/tmp/plot"+names[j]+"fm.dat");
                FILE* fp = fopen(fname.c_str(), "w");
                hrp::Vector3 cop;
                cop(0) = -ref_foot_moment[j](1)/ref_foot_force[j](2) + cop_pos[j](0);
                cop(1) =  ref_foot_moment[j](0)/ref_foot_force[j](2) + cop_pos[j](1);
                fprintf(fp, "%f %f 0\n", cop(0), cop(1));
                fprintf(fp, "%f %f %f\n", cop(0), cop(1), ref_foot_force[j](2));
                fclose(fp);
            }
            fprintf(fp_fm, "%f %f %f %f %f %f %f %f %f\n",
                    refzmp_vec[i](0), refzmp_vec[i](1),
                    ref_foot_force[0](2), ref_foot_force[1](2),
                    ref_foot_moment[0](0), ref_foot_moment[1](0),
                    ref_foot_moment[0](1), ref_foot_moment[1](1),
                    alpha);
            {
                std::string fname("/tmp/plotzmp.dat");
                FILE* fp = fopen(fname.c_str(), "w");
                fprintf(fp, "%f %f %f\n", refzmp_vec[i](0), refzmp_vec[i](1), refzmp_vec[i](2));
                fclose(fp);
            }
            fprintf(gp, "splot [-0.5:0.5][-0.5:0.5][-1:1000] '/tmp/plotrleg.dat' using 1:2:3 with lines title 'rleg'\n");
            fprintf(gp, "replot '/tmp/plotlleg.dat' using 1:2:3 with lines title 'lleg'\n");
            fprintf(gp, "replot '/tmp/plotrlegfm.dat' using 1:2:3 with lines title 'rleg fm' lw 5\n");
            fprintf(gp, "replot '/tmp/plotllegfm.dat' using 1:2:3 with lines title 'lleg fm' lw 5\n");
            fprintf(gp, "replot '/tmp/plotzmp.dat' using 1:2:3 with points title 'zmp' lw 10\n");
            fflush(gp);
            usleep(100000);
        }
        fclose(fp_fm);
        fprintf(gp_m, "splot [-0.5:0.5][-0.5:0.5][-50:50] '/tmp/plotrleg.dat' using 1:2:3 with lines title 'rleg'\n");
        fprintf(gp_m, "replot '/tmp/plotlleg.dat' using 1:2:3 with lines title 'lleg'\n");
        fprintf(gp_m, "replot '/tmp/plot-fm.dat' using 1:2:5 with points title 'rleg nx' lw 5\n");
        fprintf(gp_m, "replot '/tmp/plot-fm.dat' using 1:2:6 with points title 'lleg nx' lw 5\n");
        fprintf(gp_m, "replot '/tmp/plot-fm.dat' using 1:2:7 with points title 'rleg ny' lw 5\n");
        fprintf(gp_m, "replot '/tmp/plot-fm.dat' using 1:2:8 with points title 'lleg ny' lw 5\n");
        fflush(gp_m);
        fprintf(gp_f, "splot [-0.5:0.5][-0.5:0.5][-50:800] '/tmp/plotrleg.dat' using 1:2:3 with lines title 'rleg'\n");
        fprintf(gp_f, "replot '/tmp/plotlleg.dat' using 1:2:3 with lines title 'lleg'\n");
        fprintf(gp_f, "replot '/tmp/plot-fm.dat' using 1:2:3 with points title 'rleg fz' lw 5\n");
        fprintf(gp_f, "replot '/tmp/plot-fm.dat' using 1:2:4 with points title 'lleg fz' lw 5\n");
        fflush(gp_f);
        fprintf(gp_a, "splot [-0.5:0.5][-0.5:0.5][-0.1:1.1] '/tmp/plotrleg.dat' using 1:2:3 with lines title 'rleg'\n");
        fprintf(gp_a, "replot '/tmp/plotlleg.dat' using 1:2:3 with lines title 'lleg'\n");
        fprintf(gp_a, "replot '/tmp/plot-fm.dat' using 1:2:9 with points title 'alpha' lw 5\n");
        fflush(gp_a);
        double tmp;
        std::cin >> tmp;
        pclose(gp);
        pclose(gp_m);
        pclose(gp_f);
        pclose(gp_a);
    }
};

class testZMPDistributorHRP2JSK : public testZMPDistributor
{
 public:
    testZMPDistributorHRP2JSK ()
        {
            dt = 0.004;
            total_fz = 56*9.8066;
            szd = new SimpleZMPDistributor();
            szd->set_leg_inside_margin(0.070104);
            szd->set_leg_outside_margin(0.070104);
            szd->set_leg_front_margin(0.137525);
            szd->set_leg_rear_margin(0.106925);
            szd->set_vertices_from_margin_params();
            szd->print_vertices("");
            ee_pos.push_back(hrp::Vector3(0,-0.105,0));
            ee_pos.push_back(hrp::Vector3(0,0.105,0));
            cop_pos.push_back(hrp::Vector3(0,-0.105,0));
            cop_pos.push_back(hrp::Vector3(0,0.105,0));
        };
};

int main(int argc, char* argv[])
{
    testZMPDistributorHRP2JSK tzd;
    tzd.gen_and_plot();
    return 0;
}

