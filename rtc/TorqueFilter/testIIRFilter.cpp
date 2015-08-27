/* -*- coding:utf-8-unix; mode:c++; -*- */

#include "IIRFilter.h"
/* samples */
#include <stdio.h>
#include <cstdio>
#include <iostream>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <hrpUtil/Eigen3d.h>

template <class T>
class testIIRFilter
{
protected:
    double dt; /* [s] */
    double input_freq; /* [Hz] */
    boost::shared_ptr<FirstOrderLowPassFilter<T> > filter;
    bool use_gnuplot;
    void fprintf_value (FILE* fp, const double _time, const T& _input, const T& _output);
    void fprintf_plot (FILE* gp_pos);
    void gen_pattern_and_plot (const std::vector<double>& time_vec, const std::vector<T>& input_vec)
    {
        std::string fname("/tmp/plot-iirfilter.dat");
        FILE* fp = fopen(fname.c_str(), "w");
        for (size_t i = 0; i < time_vec.size();i++) {
            fprintf_value(fp, time_vec[i], input_vec[i], filter->passFilter(input_vec[i]));
        }
        fclose(fp);
        if (use_gnuplot) {
            //   plot pos
            FILE* gp_pos = popen("gnuplot", "w");
            fprintf(gp_pos, "set xlabel 'Time [s]'\n");
            fprintf(gp_pos, "set ylabel 'var []'\n");
            fprintf_plot (gp_pos);
            fflush(gp_pos);
            double tmp;
            std::cin >> tmp;
            pclose(gp_pos);
        }
    };
    T init_value ();
    T test0_input_value (const size_t i);
    double calc_sin_value (const size_t i) { return std::sin(2*M_PI*i*dt*input_freq); };
public:
    std::vector<std::string> arg_strs;
    testIIRFilter (const double _dt = 0.004) : dt(_dt), input_freq(1.0),
                                               filter(boost::shared_ptr<FirstOrderLowPassFilter<T> >(new FirstOrderLowPassFilter<T>(4.0, dt, init_value()))),
                                               use_gnuplot(true) {};
    void test0 ()
    {
        std::cerr << "test0 : test" << std::endl;
        parse_params();
        double tm = 0.0, total_tm = 4.0;
        std::vector<double> time_vec;
        std::vector<T> input_vec;
        for (size_t i = 0; i < static_cast<size_t>(total_tm/dt);i++) {
            time_vec.push_back(tm);
            input_vec.push_back(test0_input_value(i));
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

// Specialization for double
template<> void testIIRFilter<double>::fprintf_value (FILE* fp, const double _time, const double& _input, const double& _output)
{
    fprintf(fp, "%f %f %f\n", _time, _input, _output);
};
template<> double testIIRFilter<double>::init_value () { return 0;};
template<> double testIIRFilter<double>::test0_input_value (const size_t i) { return calc_sin_value(i);};
template<> void testIIRFilter<double>::fprintf_plot (FILE* gp_pos)
{
    fprintf(gp_pos, "plot '/tmp/plot-iirfilter.dat' using 1:2 with lines title 'input' lw 4, '/tmp/plot-iirfilter.dat' using 1:3 with lines title 'filtered' lw 3\n");
};

// Specialization for hrp::Vector3
template<> void testIIRFilter<hrp::Vector3>::fprintf_value (FILE* fp, const double _time, const hrp::Vector3& _input, const hrp::Vector3& _output)
{
    fprintf(fp, "%f %f %f %f %f %f %f\n", _time, _input[0], _output[0], _input[1], _output[1], _input[2], _output[2]);
};
template<> hrp::Vector3 testIIRFilter<hrp::Vector3>::init_value () { return hrp::Vector3::Zero();};
template<> hrp::Vector3 testIIRFilter<hrp::Vector3>::test0_input_value (const size_t i)
{
    double tmp = calc_sin_value(i);
    return hrp::Vector3(tmp, 2*tmp, -0.5*tmp);
};
template<> void testIIRFilter<hrp::Vector3>::fprintf_plot (FILE* gp_pos)
{
    fprintf(gp_pos, "plot '/tmp/plot-iirfilter.dat' using 1:2 with lines title 'input (0)' lw 4, '/tmp/plot-iirfilter.dat' using 1:3 with lines title 'filtered (0)' lw 3,");
    fprintf(gp_pos, "'/tmp/plot-iirfilter.dat' using 1:4 with lines title 'input (1)' lw 4, '/tmp/plot-iirfilter.dat' using 1:5 with lines title 'filtered (1)' lw 3,");
    fprintf(gp_pos, "'/tmp/plot-iirfilter.dat' using 1:6 with lines title 'input (2)' lw 4, '/tmp/plot-iirfilter.dat' using 1:7 with lines title 'filtered (2)' lw 3\n");
};

void print_usage ()
{
    std::cerr << "Usage : testIIRFilter [mode] [test-name] [option]" << std::endl;
    std::cerr << " [mode] should be: --double, --vector3" << std::endl;
    std::cerr << " [test-name] should be:" << std::endl;
    std::cerr << "  --test0 : test" << std::endl;
    std::cerr << " [option] should be:" << std::endl;
};

int main(int argc, char* argv[])
{
    int ret = 0;
    if (argc >= 3) {
        if (std::string(argv[1]) == "--double") {
            testIIRFilter<double> tiir;
            for (int i = 2; i < argc; ++ i) {
                tiir.arg_strs.push_back(std::string(argv[i]));
            }
            if (std::string(argv[2]) == "--test0") {
                tiir.test0();
            } else {
                print_usage();
                ret = 1;
            }
        } else if (std::string(argv[1]) == "--vector3") {
            testIIRFilter<hrp::Vector3> tiir;
            for (int i = 2; i < argc; ++ i) {
                tiir.arg_strs.push_back(std::string(argv[i]));
            }
            if (std::string(argv[2]) == "--test0") {
                tiir.test0();
            } else {
                print_usage();
                ret = 1;
            }
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

