/* -*- coding:utf-8-unix; mode:c++; -*- */

#include "ZMPDistributor.h"
/* samples */
#include <stdio.h>
#include <cstdio>
#include <iostream>
#include "hrpsys/util/Hrpsys.h" // added for QNX compile

class testZMPDistributor
{
protected:
    double dt; // [s]
    double total_fz; // [N]
    SimpleZMPDistributor* szd;
    bool use_gnuplot;
    enum {EEFM, EEFMQP, EEFMQP2, EEFMQPCOP, EEFMQPCOP2} distribution_algorithm;
    size_t sleep_msec;
    std::vector<hrp::Vector3> leg_pos;
    std::vector<hrp::Vector3> ee_pos;
    std::vector<hrp::Vector3> cop_pos;
    std::vector<hrp::Matrix33> ee_rot;
    std::vector<std::vector<Eigen::Vector2d> > fs;
private:
    void gen_and_plot ()
    {
        parse_params();
        std::cerr << "distribution_algorithm = ";
        if (distribution_algorithm==EEFM) std::cerr << "EEFM";
        else if (distribution_algorithm==EEFMQP) std::cerr << "EEFMQP";
        else if (distribution_algorithm==EEFMQP2) std::cerr << "EEFMQP(cop_distribution)";
        else if (distribution_algorithm==EEFMQPCOP) std::cerr << "EEFMQPCOP";
        else if (distribution_algorithm==EEFMQPCOP2) std::cerr << "EEFMQPCOP2";
        else std::cerr << "None?";
        std::cerr << ", sleep_msec = " << sleep_msec << "[msec]" << std::endl;
        szd->print_vertices("");
        szd->print_params(std::string(""));
        //
        std::vector<double> x_vec;
        double txx = -0.25;
        while (txx < 0.25) {
            x_vec.push_back(txx);
            txx += 0.05;
        };
        std::vector<double> y_vec;
        double tyy = -0.24;
        while (tyy < 0.25) {
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
        FILE* gp;
        FILE* gp_m;
        FILE* gp_f;
        FILE* gp_a;
        if (use_gnuplot) {
            gp = popen("gnuplot", "w");
            gp_m = popen("gnuplot", "w");
            gp_f = popen("gnuplot", "w");
            gp_a = popen("gnuplot", "w");
        }
        std::string fname_fm("/tmp/plot-fm.dat");
        FILE* fp_fm = fopen(fname_fm.c_str(), "w");
        std::vector<std::string> names;
        names.push_back("rleg");
        names.push_back("lleg");
        std::vector<double> limb_gains(names.size(), 1.0);
        std::vector<double> toeheel_ratio(names.size(), 1.0);
        std::vector<hrp::Vector3> ref_foot_force(names.size(), hrp::Vector3::Zero()), ref_foot_moment(names.size(), hrp::Vector3::Zero());
        std::vector<std::vector<Eigen::Vector2d> > fs;
        szd->get_vertices(fs);
        //
        std::ostringstream oss_foot_params("");
        oss_foot_params << "'/tmp/plotrleg.dat' using 1:2:3 with lines title 'rleg', "
                        << "'/tmp/plotlleg.dat' using 1:2:3 with lines title 'lleg', "
                        << "'/tmp/plotrleg-cop.dat' using 1:2:3 with points title 'rleg cop', "
                        << "'/tmp/plotlleg-cop.dat' using 1:2:3 with points title 'lleg cop', ";
        std::string foot_params_str = oss_foot_params.str();
        //
        for (size_t i = 0; i < refzmp_vec.size(); i++) {
            double alpha = szd->calcAlpha(refzmp_vec[i], ee_pos, ee_rot, names);
            if (distribution_algorithm == EEFMQP) {
                szd->distributeZMPToForceMoments(ref_foot_force, ref_foot_moment,
                                                 ee_pos, cop_pos, ee_rot, names, limb_gains, toeheel_ratio,
                                                 refzmp_vec[i], refzmp_vec[i],
                                                 total_fz, dt);
            } else if (distribution_algorithm == EEFMQP || distribution_algorithm == EEFMQP2) {
                szd->distributeZMPToForceMomentsQP(ref_foot_force, ref_foot_moment,
                                                   ee_pos, cop_pos, ee_rot, names, limb_gains, toeheel_ratio,
                                                   refzmp_vec[i], refzmp_vec[i],
                                                   total_fz, dt, true, "", (distribution_algorithm == EEFMQP2));
            } else if (distribution_algorithm == EEFMQPCOP) {
                szd->distributeZMPToForceMomentsPseudoInverse(ref_foot_force, ref_foot_moment,
                                                              ee_pos, cop_pos, ee_rot, names, limb_gains, toeheel_ratio,
                                                              refzmp_vec[i], refzmp_vec[i],
                                                              total_fz, dt, true, "");
            } else if (distribution_algorithm == EEFMQPCOP2) {
                // std::vector<double> ee_forcemoment_distribution_weight(names.size(), 1.0);
                // szd->distributeZMPToForceMomentsPseudoInverse2(ref_foot_force, ref_foot_moment,
                //                                                ee_pos, cop_pos, ee_rot, names, limb_gains, toeheel_ratio,
                //                                                refzmp_vec[i], refzmp_vec[i],
                //                                                total_fz, dt, true, "");
            }
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
                std::string fname("/tmp/plot"+names[j]+"-cop.dat");
                FILE* fp = fopen(fname.c_str(), "w");
                fprintf(fp, "%f %f %f\n", ee_pos[j](0), ee_pos[j](1), ee_pos[j](2));
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
            if (use_gnuplot) {
                std::ostringstream oss("");
                oss << "splot [-0.5:0.5][-0.5:0.5][-1:1000] " << foot_params_str
                    << "'/tmp/plotrlegfm.dat' using 1:2:3 with lines title 'rleg fm' lw 5, "
                    << "'/tmp/plotllegfm.dat' using 1:2:3 with lines title 'lleg fm' lw 5, "
                    << "'/tmp/plotzmp.dat' using 1:2:3 with points title 'zmp' lw 10";
                fprintf(gp, "%s\n", oss.str().c_str());
                fflush(gp);
                usleep(1000*sleep_msec);
            }
        }
        fclose(fp_fm);
        if (use_gnuplot) {
            std::ostringstream oss("");
            oss << "splot [-0.5:0.5][-0.5:0.5][-100:100] " << foot_params_str
                << "'/tmp/plot-fm.dat' using 1:2:5 with points title 'rleg nx' lw 5, "
                << "'/tmp/plot-fm.dat' using 1:2:6 with points title 'lleg nx' lw 5, "
                << "'/tmp/plot-fm.dat' using 1:2:7 with points title 'rleg ny' lw 5, "
                << "'/tmp/plot-fm.dat' using 1:2:8 with points title 'lleg ny' lw 5";
            fprintf(gp_m, "%s\n", oss.str().c_str());
            fflush(gp_m);
            oss.str("");
            oss << "splot [-0.5:0.5][-0.5:0.5][-50:" << total_fz*1.1 << "] " << foot_params_str
                << "'/tmp/plot-fm.dat' using 1:2:3 with points title 'rleg fz' lw 5, "
                << "'/tmp/plot-fm.dat' using 1:2:4 with points title 'lleg fz' lw 5";
            fprintf(gp_f, "%s\n", oss.str().c_str());
            fflush(gp_f);
            oss.str("");
            oss << "splot [-0.5:0.5][-0.5:0.5][-0.1:1.1] " << foot_params_str
                << "'/tmp/plot-fm.dat' using 1:2:9 with points title 'alpha' lw 5";
            fprintf(gp_a, "%s\n", oss.str().c_str());
            fflush(gp_a);
            double tmp;
            std::cin >> tmp;
            pclose(gp);
            pclose(gp_m);
            pclose(gp_f);
            pclose(gp_a);
        }
    };
public:
    std::vector<std::string> arg_strs;
    testZMPDistributor(const double _dt) : dt(_dt), distribution_algorithm(EEFMQP), use_gnuplot(true), sleep_msec(100)
    {
        szd = new SimpleZMPDistributor(_dt);
    };
    virtual ~testZMPDistributor()
    {
        if (szd != NULL) {
            delete szd;
            szd = NULL;
        }
    };

    void parse_params ()
    {
      for (int i = 0; i < arg_strs.size(); ++ i) {
          if ( arg_strs[i]== "--distribution-algorithm" ) {
              if (++i < arg_strs.size()) {
                  if (arg_strs[i]=="EEFM") {
                      distribution_algorithm = EEFM;
                  } else if (arg_strs[i]=="EEFMQP") {
                      distribution_algorithm = EEFMQP;
                  } else if (arg_strs[i]=="EEFMQP2") {
                      distribution_algorithm = EEFMQP2;
                  } else if (arg_strs[i]=="EEFMQPCOP") {
                      distribution_algorithm = EEFMQPCOP;
                  } else if (arg_strs[i]=="EEFMQPCOP") {
                      distribution_algorithm = EEFMQPCOP;
                  } else if (arg_strs[i]=="EEFMQPCOP2") {
                      distribution_algorithm = EEFMQPCOP2;
                  } else {
                      distribution_algorithm = EEFM;
                  }
              }
          } else if ( arg_strs[i]== "--sleep-msec" ) {
              if (++i < arg_strs.size()) sleep_msec = atoi(arg_strs[i].c_str());
          } else if ( arg_strs[i]== "--use-gnuplot" ) {
              if (++i < arg_strs.size()) use_gnuplot = (arg_strs[i]=="true");
          }
      }
    };

    void test0 ()
    {
        std::cerr << "test0 : Default foot pos" << std::endl;
        ee_pos = leg_pos;
        cop_pos = leg_pos;
        ee_rot.push_back(hrp::Matrix33::Identity());
        ee_rot.push_back(hrp::Matrix33::Identity());
        gen_and_plot();
    };

    void test1 ()
    {
        std::cerr << "test1 : Fwd foot pos" << std::endl;
        ee_pos = leg_pos;
        ee_pos[0] = ee_pos[0] + hrp::Vector3(0.05,0,0);
        ee_pos[1] = ee_pos[1] + hrp::Vector3(-0.05,0,0);
        cop_pos = ee_pos;
        ee_rot.push_back(hrp::Matrix33::Identity());
        ee_rot.push_back(hrp::Matrix33::Identity());
        gen_and_plot();
    };

    void test2 ()
    {
        std::cerr << "test2 : Rot foot pos" << std::endl;
        ee_pos = leg_pos;
        ee_pos[0] = ee_pos[0] + hrp::Vector3(0.02,0,0);
        ee_pos[1] = ee_pos[1] + hrp::Vector3(-0.02,0,0);
        cop_pos = ee_pos;
        hrp::Matrix33 tmpr;
        tmpr = hrp::rotFromRpy(hrp::Vector3(0,0,-15*M_PI/180.0));
        ee_rot.push_back(tmpr);
        tmpr = hrp::rotFromRpy(hrp::Vector3(0,0,15*M_PI/180.0));
        ee_rot.push_back(tmpr);
        gen_and_plot();
    };
};

class testZMPDistributorHRP2JSK : public testZMPDistributor
{
 public:
    testZMPDistributorHRP2JSK () : testZMPDistributor(0.004)
        {
            total_fz = 56*9.8066;
            szd->set_leg_inside_margin(0.070104);
            szd->set_leg_outside_margin(0.070104);
            szd->set_leg_front_margin(0.137525);
            szd->set_leg_rear_margin(0.106925);
            szd->set_vertices_from_margin_params();
            leg_pos.push_back(hrp::Vector3(0,-0.105,0));
            leg_pos.push_back(hrp::Vector3(0,0.105,0));
        };
};

class testZMPDistributorJAXON_RED : public testZMPDistributor
{
 public:
    testZMPDistributorJAXON_RED () : testZMPDistributor(0.002)
        {
            total_fz = 130.442*9.8066;
            szd->set_leg_inside_margin(0.055992);
            szd->set_leg_outside_margin(0.075992);
            szd->set_leg_front_margin(0.133242);
            szd->set_leg_rear_margin(0.100445);
            szd->set_vertices_from_margin_params();
            leg_pos.push_back(hrp::Vector3(0,-0.100,0));
            leg_pos.push_back(hrp::Vector3(0,0.100,0));
        };
};

void print_usage ()
{
    std::cerr << "Usage : testZMPDistributor [robot-name] [test-type] [option]" << std::endl;
    std::cerr << " [robot-name] should be: --hrp2jsk, --jaxon_red" << std::endl;
    std::cerr << " [test-type] should be:" << std::endl;
    std::cerr << "  --test0 : Default foot pos" << std::endl;
    std::cerr << "  --test1 : Fwd foot pos" << std::endl;
    std::cerr << "  --test2 : Rot foot pos" << std::endl;
};

int main(int argc, char* argv[])
{
    int ret = 0;
    if (argc >= 3) {
        testZMPDistributor* tzd = NULL;
        if (std::string(argv[1]) == "--hrp2jsk") {
            tzd = new testZMPDistributorHRP2JSK();
        } else if (std::string(argv[1]) == "--jaxon_red") {
            tzd = new testZMPDistributorJAXON_RED();
        } else {
            print_usage();
            ret = 1;
        }
        if (tzd != NULL) {
            for (int i = 2; i < argc; ++ i) {
                tzd->arg_strs.push_back(std::string(argv[i]));
            }
            if (std::string(argv[2]) == "--test0") {
                tzd->test0();
            } else if (std::string(argv[2]) == "--test1") {
                tzd->test1();
            } else if (std::string(argv[2]) == "--test2") {
                tzd->test2();
            } else {
                print_usage();
                ret = 1;
            }
            delete tzd;
        }
    } else {
        print_usage();
        ret = 1;
    }
    return ret;
}

