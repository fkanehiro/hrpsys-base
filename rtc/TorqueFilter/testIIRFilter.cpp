/* -*- coding:utf-8-unix; mode:c++; -*- */

#include "IIRFilter.h"
/* samples */
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <hrpUtil/Eigen3d.h>

template <class T, class FT>
class testIIRFilter
{
protected:
    double dt; /* [s] */
    double input_freq; /* [Hz] */
    boost::shared_ptr<FT > filter;
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
                                               use_gnuplot(true) { initialize(); };
    void initialize() {
        filter = boost::shared_ptr<FT >(new FT (4.0, dt, init_value()));
    }
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
template<> void testIIRFilter<double, FirstOrderLowPassFilter<double> >::fprintf_value (FILE* fp, const double _time, const double& _input, const double& _output)
{
    fprintf(fp, "%f %f %f\n", _time, _input, _output);
};
template<> double testIIRFilter<double, FirstOrderLowPassFilter<double> >::init_value () { return 0;};
template<> double testIIRFilter<double, FirstOrderLowPassFilter<double> >::test0_input_value (const size_t i) { return calc_sin_value(i);};
template<> void testIIRFilter<double, FirstOrderLowPassFilter<double> >::fprintf_plot (FILE* gp_pos)
{
    fprintf(gp_pos, "plot '/tmp/plot-iirfilter.dat' using 1:2 with lines title 'input' lw 4, '/tmp/plot-iirfilter.dat' using 1:3 with lines title 'filtered' lw 3\n");
};

// Specialization for hrp::Vector3
template<> void testIIRFilter<hrp::Vector3, FirstOrderLowPassFilter<hrp::Vector3> >::fprintf_value (FILE* fp, const double _time, const hrp::Vector3& _input, const hrp::Vector3& _output)
{
    fprintf(fp, "%f %f %f %f %f %f %f\n", _time, _input[0], _output[0], _input[1], _output[1], _input[2], _output[2]);
};
template<> hrp::Vector3 testIIRFilter<hrp::Vector3, FirstOrderLowPassFilter<hrp::Vector3> >::init_value () { return hrp::Vector3::Zero();};
template<> hrp::Vector3 testIIRFilter<hrp::Vector3, FirstOrderLowPassFilter<hrp::Vector3> >::test0_input_value (const size_t i)
{
    double tmp = calc_sin_value(i);
    return hrp::Vector3(tmp, 2*tmp, -0.5*tmp);
};
template<> void testIIRFilter<hrp::Vector3, FirstOrderLowPassFilter<hrp::Vector3> >::fprintf_plot (FILE* gp_pos)
{
    fprintf(gp_pos, "plot '/tmp/plot-iirfilter.dat' using 1:2 with lines title 'input (0)' lw 4, '/tmp/plot-iirfilter.dat' using 1:3 with lines title 'filtered (0)' lw 3,");
    fprintf(gp_pos, "'/tmp/plot-iirfilter.dat' using 1:4 with lines title 'input (1)' lw 4, '/tmp/plot-iirfilter.dat' using 1:5 with lines title 'filtered (1)' lw 3,");
    fprintf(gp_pos, "'/tmp/plot-iirfilter.dat' using 1:6 with lines title 'input (2)' lw 4, '/tmp/plot-iirfilter.dat' using 1:7 with lines title 'filtered (2)' lw 3\n");
};
///
template<> void testIIRFilter<double, IIRFilter >::initialize () {
#if 0 // use obsolated method
    int filter_dim = 0;
    std::vector<double> fb_coeffs, ff_coeffs;
    filter_dim = 2;
    fb_coeffs.resize(filter_dim+1);
    fb_coeffs[0] = 1.00000;
    fb_coeffs[1] = 1.88903;
    fb_coeffs[2] =-0.89487;
    ff_coeffs.resize(filter_dim+1);
    ff_coeffs[0] = 0.0014603;
    ff_coeffs[1] = 0.0029206;
    ff_coeffs[2] = 0.0014603;
    filter = boost::shared_ptr<IIRFilter >(new IIRFilter (filter_dim, fb_coeffs, ff_coeffs));
#else
    filter = boost::shared_ptr<IIRFilter >(new IIRFilter ());
    int dim = 2;
    std::vector<double> A(dim+1);
    std::vector<double> B(dim+1);
    // octave
    // [B, A] = butter(2, 0.004 * 2 * 8) ;;; 2 * dt * cutoff_freq
    //b =
    //0.00882608666843131   0.01765217333686262   0.00882608666843131
    //a =
    //1.000000000000000  -1.717211834908084   0.752516181581809
    A[0] = 1.000000000000000;
    A[1] = -1.717211834908084;
    A[2] = 0.752516181581809;
    B[0] = 0.00882608666843131;
    B[1] = 0.01765217333686262;
    B[2] = 0.00882608666843131;
    filter->setParameter(dim, A, B);
#endif
};
template<> void testIIRFilter<double, IIRFilter >::parse_params () {
    for (int i = 0; i < arg_strs.size(); ++ i) {
        if ( arg_strs[i]== "--use-gnuplot" ) {
            if (++i < arg_strs.size()) use_gnuplot = (arg_strs[i]=="true");
#if 0
        } else if ( arg_strs[i]== "--cutoff-freq" ) {
            if (++i < arg_strs.size()) filter->setCutOffFreq(atof(arg_strs[i].c_str()));
#endif
        } else if ( arg_strs[i]== "--input-freq" ) {
            if (++i < arg_strs.size()) input_freq = atof(arg_strs[i].c_str());
        }
    }
    std::cerr << "[testIIRFilter] params" << std::endl;
    std::cerr << "[testIIRFilter]   dt = " << dt << "[s], cutoff-freq = " << 8 << "[Hz], input-freq = " << input_freq << "[Hz]" << std::endl;
};
template<> void testIIRFilter<double, IIRFilter >::fprintf_value (FILE* fp, const double _time, const double& _input, const double& _output)
{
    fprintf(fp, "%f %f %f\n", _time, _input, _output);
};
template<> double testIIRFilter<double, IIRFilter >::init_value () { return 0;};
template<> double testIIRFilter<double, IIRFilter >::test0_input_value (const size_t i) { return calc_sin_value(i);};
template<> void testIIRFilter<double, IIRFilter >::fprintf_plot (FILE* gp_pos)
{
    fprintf(gp_pos, "plot '/tmp/plot-iirfilter.dat' using 1:2 with lines title 'input' lw 4, '/tmp/plot-iirfilter.dat' using 1:3 with lines title 'filtered' lw 3\n");
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
            testIIRFilter<double, FirstOrderLowPassFilter<double> > tiir;
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
            testIIRFilter<hrp::Vector3, FirstOrderLowPassFilter<hrp::Vector3> > tiir;
            for (int i = 2; i < argc; ++ i) {
                tiir.arg_strs.push_back(std::string(argv[i]));
            }
            if (std::string(argv[2]) == "--test0") {
                tiir.test0();
            } else {
                print_usage();
                ret = 1;
            }
        } else if (std::string(argv[1]) == "--iir") {
            testIIRFilter<double, IIRFilter > tiir;
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

