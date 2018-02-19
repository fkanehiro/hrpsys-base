/* -*- coding:utf-8-unix; mode:c++; -*- */

#include "ObjectContactTurnaroundDetectorBase.h"
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

class testObjectContactTurnaroundDetectorBase
{
protected:
    double dt; /* [s] */
    ObjectContactTurnaroundDetectorBase octd;
    bool use_gnuplot;
    void gen_pattern_and_plot (const std::vector<double>& force_vec,
                               const std::vector<double>& time_vec)
    {
        parse_params();
        std::string fname("/tmp/plot-octd.dat");
        FILE* fp = fopen(fname.c_str(), "w");
        double detect_time;
        bool detected = false;
        for (size_t i = 0; i < time_vec.size();i++) {
            bool tmp_detected = octd.checkDetection(force_vec[i], 0.0);
            if (tmp_detected && !detected) {
                detect_time = time_vec[i];
                detected = true;
            }
            fprintf(fp, "%f %f %f %f\n", time_vec[i], force_vec[i], octd.getFilteredWrench(), octd.getFilteredDwrench(), detected);
        }
        fclose(fp);
        if (use_gnuplot) {
            // plot
            std::string titles[2] = {"Wrench", "Dwrench"};
            //   plot pos
            FILE* gp = popen("gnuplot", "w");
            fprintf(gp, "set multiplot layout 2, 1 title 'Results'\n");
            fprintf(gp, "set xlabel 'Time [s]'\n");
            fprintf(gp, "set ylabel 'Wrench'\n");
            fprintf(gp, "set arrow from %f,%f to %f,%f\n", detect_time, force_vec.front(), detect_time, force_vec.back());
            fprintf(gp, "plot '/tmp/plot-octd.dat' using 1:2 with lines title 'Wrench' lw 4, '/tmp/plot-octd.dat' using 1:3 with lines title 'FilteredWrench' lw 4\n");
            fprintf(gp, "set xlabel 'Time [s]'\n");
            fprintf(gp, "set ylabel 'Dwrench'\n");
            fprintf(gp, "plot '/tmp/plot-octd.dat' using 1:4 with lines title 'Dwrench' lw 4\n");
            fflush(gp);
            double tmp;
            std::cin >> tmp;
            pclose(gp);
        }
    };
public:
    std::vector<std::string> arg_strs;
    testObjectContactTurnaroundDetectorBase (const double _dt = 0.004) : dt(_dt), octd(_dt), use_gnuplot(true)
    {
        octd.setWrenchCutoffFreq(5.0);
        octd.setDwrenchCutoffFreq(5.0);
        octd.setFrictionCoeffWrenchCutoffFreq(5.0);
    };
    void test0 ()
    {
        std::cerr << "test0 : Increasing->saturation" << std::endl;
        double tm = 0.0, total_tm = 4.0, df = 10;
        std::vector<double> time_vec, force_vec;
        for (size_t i = 0; i < static_cast<size_t>(total_tm/dt);i++) {
            time_vec.push_back(tm);
            if (i*dt < total_tm*0.1) {
                force_vec.push_back(0);
            } else if (i*dt > total_tm*0.4) {
                force_vec.push_back(df*total_tm*(0.4-0.1));
            } else {
                force_vec.push_back(df*(i*dt-total_tm*0.1));
            }
            tm += dt;
        }
        octd.startDetection(df, total_tm);
        gen_pattern_and_plot (force_vec, time_vec);
    };
    void test1 ()
    {
        std::cerr << "test1 : Increasing->decreasing" << std::endl;
        double tm = 0.0, total_tm = 4.0, df = 10;
        std::vector<double> time_vec, force_vec;
        for (size_t i = 0; i < static_cast<size_t>(total_tm/dt);i++) {
            time_vec.push_back(tm);
            if (i*dt < total_tm*0.1) {
                force_vec.push_back(0);
            } else if (i*dt > total_tm*0.4) {
                force_vec.push_back(-2*df*(i*dt-total_tm*0.4) + df*total_tm*(0.4-0.1));
            } else {
                force_vec.push_back(df*(i*dt-total_tm*0.1));
            }
            tm += dt;
        }
        octd.startDetection(df, total_tm);
        gen_pattern_and_plot (force_vec, time_vec);
    };
    void test2 ()
    {
        std::cerr << "test2 : Deacreasing->saturation" << std::endl;
        double tm = 0.0, total_tm = 4.0, df = -10;
        std::vector<double> time_vec, force_vec;
        for (size_t i = 0; i < static_cast<size_t>(total_tm/dt);i++) {
            time_vec.push_back(tm);
            if (i*dt < total_tm*0.1) {
                force_vec.push_back(0);
            } else if (i*dt > total_tm*0.4) {
                force_vec.push_back(df*total_tm*(0.4-0.1));
            } else {
                force_vec.push_back(df*(i*dt-total_tm*0.1));
            }
            tm += dt;
        }
        octd.startDetection(df, total_tm);
        gen_pattern_and_plot (force_vec, time_vec);
    };
    void test3 ()
    {
        std::cerr << "test3 : Decreasing->increasing" << std::endl;
        double tm = 0.0, total_tm = 4.0, df = -10;
        std::vector<double> time_vec, force_vec;
        for (size_t i = 0; i < static_cast<size_t>(total_tm/dt);i++) {
            time_vec.push_back(tm);
            if (i*dt < total_tm*0.1) {
                force_vec.push_back(0);
            } else if (i*dt > total_tm*0.4) {
                force_vec.push_back(-2*df*(i*dt-total_tm*0.4) + df*total_tm*(0.4-0.1));
            } else {
                force_vec.push_back(df*(i*dt-total_tm*0.1));
            }
            tm += dt;
        }
        octd.startDetection(df, total_tm);
        gen_pattern_and_plot (force_vec, time_vec);
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
    std::cerr << "Usage : testObjectContactTurnaroundDetectorBase [option]" << std::endl;
    std::cerr << " [option] should be:" << std::endl;
    std::cerr << "  --test0 : Increasing->saturation" << std::endl;
    std::cerr << "  --test1 : Increasing->decreasing" << std::endl;
    std::cerr << "  --test2 : Decreasing->saturation" << std::endl;
    std::cerr << "  --test3 : Decreasing->increasing" << std::endl;
    // std::cerr << "  --test1 : Move pos and rot" << std::endl;
};

int main(int argc, char* argv[])
{
    int ret = 0;
    if (argc >= 2) {
        testObjectContactTurnaroundDetectorBase toctd;
        for (int i = 1; i < argc; ++ i) {
            toctd.arg_strs.push_back(std::string(argv[i]));
        }
        if (std::string(argv[1]) == "--test0") {
            toctd.test0();
        } else if (std::string(argv[1]) == "--test1") {
            toctd.test1();
        } else if (std::string(argv[1]) == "--test2") {
            toctd.test2();
        } else if (std::string(argv[1]) == "--test3") {
            toctd.test3();
        // } else if (std::string(argv[1]) == "--test1") {
        //     tiog.test1();
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

