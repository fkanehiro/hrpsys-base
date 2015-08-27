/* -*- coding:utf-8-unix; mode:c++; -*- */

#include "IIRFilter.h"
/* samples */
#include <stdio.h>
#include <cstdio>
#include <iostream>
#include <vector>
#include <boost/shared_ptr.hpp>

class testIIRFilter
{
protected:
    double dt; /* [s] */
    double input_freq; /* [Hz] */
    boost::shared_ptr<FirstOrderLowPassFilter<double> > filter;
    bool use_gnuplot;
    void gen_pattern_and_plot (const std::vector<double>& time_vec, const std::vector<double>& input_vec)
    {
        std::string fname("/tmp/plot-iirfilter.dat");
        FILE* fp = fopen(fname.c_str(), "w");
        for (size_t i = 0; i < time_vec.size();i++) {
            fprintf(fp, "%f %f %f\n", time_vec[i], input_vec[i], filter->passFilter(input_vec[i]));
        }
        fclose(fp);
        if (use_gnuplot) {
            //   plot pos
            FILE* gp_pos = popen("gnuplot", "w");
            fprintf(gp_pos, "set xlabel 'Time [s]'\n");
            fprintf(gp_pos, "set ylabel 'var []'\n");
            fprintf(gp_pos, "plot '/tmp/plot-iirfilter.dat' using 1:2 with lines title 'input' lw 4, '/tmp/plot-iirfilter.dat' using 1:3 with lines title 'filtered' lw 3\n");
            fflush(gp_pos);
            double tmp;
            std::cin >> tmp;
            pclose(gp_pos);
        }
    };
public:
    std::vector<std::string> arg_strs;
    testIIRFilter (const double _dt = 0.004) : dt(_dt), input_freq(1.0), filter(boost::shared_ptr<FirstOrderLowPassFilter<double> >(new FirstOrderLowPassFilter<double>(4.0, dt, 0))), use_gnuplot(true) {};
    void test0 ()
    {
        std::cerr << "test0 : test" << std::endl;
        parse_params();
        double tm = 0.0, total_tm = 4.0;
        std::vector<double> time_vec;
        std::vector<double> input_vec;
        for (size_t i = 0; i < static_cast<size_t>(total_tm/dt);i++) {
            time_vec.push_back(tm);
            input_vec.push_back(std::sin(2*M_PI*i*dt*input_freq));
            tm += dt;
        }
        gen_pattern_and_plot (time_vec,input_vec);
    };
    void parse_params ()
    {
      for (int i = 0; i < arg_strs.size(); ++ i) {
          if ( arg_strs[i]== "--use-gnuplot" ) {
              if (++i < arg_strs.size()) use_gnuplot = (arg_strs[i]=="true");
          } else if ( arg_strs[i]== "--cutoff-freq" ) {
              if (++i < arg_strs.size()) filter->setCutOffFreq(atof(arg_strs[i].c_str()));
          } else if ( arg_strs[i]== "--input-freq" ) {
              if (++i < arg_strs.size()) input_freq = atof(arg_strs[i].c_str());
          }
      }
      std::cerr << "[testIIRFilter] params" << std::endl;
      std::cerr << "[testIIRFilter]   dt = " << dt << "[s], cutoff-freq = " << filter->getCutOffFreq() << "[Hz], input-freq = " << input_freq << "[Hz]" << std::endl;
    };
};

void print_usage ()
{
    std::cerr << "Usage : testIIRFilter [option]" << std::endl;
    std::cerr << " [option] should be:" << std::endl;
    std::cerr << "  --test0 : test" << std::endl;
};

int main(int argc, char* argv[])
{
    int ret = 0;
    if (argc >= 2) {
        testIIRFilter tiir;
        for (int i = 1; i < argc; ++ i) {
            tiir.arg_strs.push_back(std::string(argv[i]));
        }
        if (std::string(argv[1]) == "--test0") {
            tiir.test0();
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

