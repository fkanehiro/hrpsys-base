/* -*- coding:utf-8-unix; mode:c++; -*- */

#include "ObjectContactTurnaroundDetectorBase.h"
/* samples */
#include <stdio.h>
#include <cstdio>
#include <iostream>
#include <vector>

class testObjectContactTurnaroundDetectorBase
{
protected:
    double dt; /* [s] */
    ObjectContactTurnaroundDetectorBase octd;
    bool use_gnuplot;
    std::vector<std::vector<hrp::Vector3> > forces_vec, moments_vec, hpos_vec;
    double detect_time, true_turnaround_time;
    std::vector<double> time_vec;
    void gen_pattern_and_plot ()
    {
        parse_params();
        octd.printParams();
        std::string fname("/tmp/plot-octd.dat");
        FILE* fp = fopen(fname.c_str(), "w");
        bool detected = false;
        double max_f = -1e10, min_f = 1e10;
        for (size_t i = 0; i < time_vec.size();i++) {
            bool tmp_detected = octd.checkDetection(forces_vec[i], moments_vec[i], hpos_vec[i]);
            if (tmp_detected && !detected) {
                detect_time = time_vec[i];
                detected = true;
            }
            hrp::dvector log_data = octd.getDataForLogger();
            fprintf(fp, "%f %f %f %f\n", time_vec[i], log_data[1], log_data[2], log_data[3], detected);
            max_f = std::max(max_f, log_data[1]);
            min_f = std::min(min_f, log_data[1]);
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
            fprintf(gp, "set arrow from %f,%f to %f,%f\n", detect_time, min_f, detect_time, max_f);
            fprintf(gp, "plot '/tmp/plot-octd.dat' using 1:2 with lines title 'Wrench' lw 4, '/tmp/plot-octd.dat' using 1:3 with lines title 'FilteredWrench' lw 4\n");
            fprintf(gp, "unset arrow\n");
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
    testObjectContactTurnaroundDetectorBase (const double _dt = 0.004) : dt(_dt), octd(_dt), use_gnuplot(true), detect_time(1e10), true_turnaround_time(1e10)
    {
        // Defaults
        octd.setWrenchCutoffFreq(5.0);
        octd.setDwrenchCutoffFreq(5.0);
        octd.setAxis(hrp::Vector3::UnitZ());
        //octd.setDetectorTotalWrench(ObjectContactTurnaroundDetectorBase::GENERALIZED_WRENCH);
        //octd.setFrictionCoeffWrenchCutoffFreq(5.0);
    };
    void gen_forces_moments (const std::vector<double>& force_vec, const hrp::Vector3& force_dir = hrp::Vector3::UnitZ())
    {
        for (size_t i = 0; i < force_vec.size();i++) {
            std::vector<hrp::Vector3> tmpv(1, hrp::Vector3::Zero());
            moments_vec.push_back(tmpv);
            hpos_vec.push_back(tmpv);
            tmpv[0] = force_vec[i] * force_dir;
            forces_vec.push_back(tmpv);
        }
    };
    hrp::dvector6 get_ccm1_by_index (size_t idx, bool is_positive = true)
    {
        hrp::dvector6 ccm1(hrp::dvector6::Zero());
        ccm1(idx) = (is_positive?1.0:-1.0);
        return ccm1;
    }
    // convert object_resultant_wrench => constraint_generalized_force
    double get_a_coeff_by_index (const double df, const hrp::dvector6& ccm1, const hrp::Vector3& fdir)
    {
        hrp::Vector3 fpos(0.2, 0.0, 0.7); // [m]
        hrp::Vector3 tmp(fdir);
        hrp::dvector6 resultant_wrench_direction_vector;
        for (size_t i = 0; i < 3; i++) resultant_wrench_direction_vector(i) = tmp(i);
        tmp = fpos.cross(fdir);
        for (size_t i = 0; i < 3; i++) resultant_wrench_direction_vector(i+3) = tmp(i);
        return -1*ccm1.dot(resultant_wrench_direction_vector) * df;
    }
    // Resultant force : robot's side resultant force
    double gen_forces_moments_for_saturation (const double total_tm, const double start_tm, const double turnaround_tm,
                                              const double start_resultant_force, const double turnaround_resultant_force,
                                              const hrp::Vector3& force_dir = hrp::Vector3::UnitZ())
    {
        std::vector<double> phi_vec;
        double dphi = (turnaround_resultant_force-start_resultant_force)/(turnaround_tm-start_tm);
        true_turnaround_time = turnaround_tm;
        for (size_t i = 0; i < static_cast<size_t>(total_tm/dt);i++) {
            double current_tm = i*dt;
            time_vec.push_back(current_tm);
            if (current_tm < start_tm) {
                phi_vec.push_back(start_resultant_force);
            } else if (current_tm < turnaround_tm) {
                phi_vec.push_back(start_resultant_force+dphi*(current_tm-start_tm));
            } else {
                phi_vec.push_back(turnaround_resultant_force);
            }
        }
        gen_forces_moments(phi_vec, force_dir);
        return dphi;
    };
    double gen_forces_moments_for_inverting (const double total_tm, const double start_tm, const double turnaround_tm,
                                             const double start_resultant_force, const double turnaround_resultant_force,
                                             const hrp::Vector3& force_dir = hrp::Vector3::UnitZ())
    {
        std::vector<double> phi_vec;
        double dphi = (turnaround_resultant_force-start_resultant_force)/(turnaround_tm-start_tm);
        true_turnaround_time = turnaround_tm;
        for (size_t i = 0; i < static_cast<size_t>(total_tm/dt);i++) {
            double current_tm = i*dt;
            time_vec.push_back(current_tm);
            if (current_tm < start_tm) {
                phi_vec.push_back(start_resultant_force);
            } else if (current_tm < turnaround_tm) {
                phi_vec.push_back(start_resultant_force+dphi*(current_tm-start_tm));
            } else {
                phi_vec.push_back(turnaround_resultant_force+(current_tm-turnaround_tm)*-2*dphi );
            }
        }
        gen_forces_moments(phi_vec, force_dir);
        return dphi;
    };
    void test0 ()
    {
        std::cerr << "test0 : Increasing->saturation (TOTAL_FORCE)" << std::endl;
        double total_tm = 4.0, start_tm = total_tm*0.1, turnaround_tm = total_tm*0.4, start_resultant_force = 0.0, turnaround_resultant_force = 40.0, dphi;
        dphi = gen_forces_moments_for_saturation(total_tm, start_tm, turnaround_tm, start_resultant_force, turnaround_resultant_force);
        octd.startDetection (dphi, total_tm);
        gen_pattern_and_plot ();
    };
    void test1 ()
    {
        std::cerr << "test1 : Increasing->decreasing (TOTAL_FORCE)" << std::endl;
        double total_tm = 4.0, start_tm = total_tm*0.1, turnaround_tm = total_tm*0.4, start_resultant_force = 0.0, turnaround_resultant_force = 40.0, dphi;
        dphi = gen_forces_moments_for_inverting(total_tm, start_tm, turnaround_tm, start_resultant_force, turnaround_resultant_force);
        octd.startDetection (dphi, total_tm);
        gen_pattern_and_plot ();
    };
    void test2 ()
    {
        std::cerr << "test2 : Deacreasing->saturation (TOTAL_FORCE)" << std::endl;
        double total_tm = 4.0, start_tm = total_tm*0.1, turnaround_tm = total_tm*0.4, start_resultant_force = 0.0, turnaround_resultant_force = -40.0, dphi;
        dphi = gen_forces_moments_for_saturation(total_tm, start_tm, turnaround_tm, start_resultant_force, turnaround_resultant_force);
        octd.startDetection (dphi, total_tm);
        gen_pattern_and_plot ();
    };
    void test3 ()
    {
        std::cerr << "test3 : Decreasing->increasing (TOTAL_FORCE)" << std::endl;
        double total_tm = 4.0, start_tm = total_tm*0.1, turnaround_tm = total_tm*0.4, start_resultant_force = 0.0, turnaround_resultant_force = -40.0, dphi;
        dphi = gen_forces_moments_for_inverting(total_tm, start_tm, turnaround_tm, start_resultant_force, turnaround_resultant_force);
        octd.startDetection (dphi, total_tm);
        gen_pattern_and_plot ();
    };
    void test4 ()
    {
        std::cerr << "test4 : Lift up (GENERALIZED_WRENCH)" << std::endl;
        double total_tm = 4.0, start_tm = total_tm*0.1, turnaround_tm = total_tm*0.4, start_resultant_force = 0.0, turnaround_resultant_force = -40.0, dphi;
        dphi = std::fabs(gen_forces_moments_for_saturation(total_tm, start_tm, turnaround_tm, start_resultant_force, turnaround_resultant_force));
        octd.setDetectorTotalWrench(ObjectContactTurnaroundDetectorBase::GENERALIZED_WRENCH);
        std::vector<hrp::dvector6> ccm1(1, get_ccm1_by_index(2));
        double ref_dwrench = get_a_coeff_by_index(dphi, ccm1[0], hrp::Vector3::UnitZ());
        octd.setConstraintConversionMatricesRefDwrench(ccm1,
                                                       std::vector<hrp::dvector6>(1, hrp::dvector6::Zero()),
                                                       std::vector<double>(1, ref_dwrench));
        octd.setMaxTime(total_tm);
        octd.startDetectionForGeneralizedWrench();
        gen_pattern_and_plot ();
    };
    void test5 ()
    {
        std::cerr << "test5 : Push fwd (GENERALIZED_WRENCH)" << std::endl;
        double total_tm = 4.0, start_tm = total_tm*0.1, turnaround_tm = total_tm*0.4, start_resultant_force = 0.0, turnaround_resultant_force = -40.0, dphi;
        dphi = std::fabs(gen_forces_moments_for_saturation(total_tm, start_tm, turnaround_tm, start_resultant_force, turnaround_resultant_force, hrp::Vector3::UnitX()));
        octd.setDetectorTotalWrench(ObjectContactTurnaroundDetectorBase::GENERALIZED_WRENCH);
        std::vector<hrp::dvector6> ccm1(1, get_ccm1_by_index(0));
        double ref_dwrench = get_a_coeff_by_index(dphi, ccm1[0], hrp::Vector3::UnitX());
        octd.setConstraintConversionMatricesRefDwrench(ccm1,
                                                       std::vector<hrp::dvector6>(1, hrp::dvector6::Zero()),
                                                       std::vector<double>(1, ref_dwrench));
        octd.setMaxTime(total_tm);
        octd.startDetectionForGeneralizedWrench();
        gen_pattern_and_plot ();
    };
    void test6 ()
    {
        std::cerr << "test6 : Push bwd (GENERALIZED_WRENCH)" << std::endl;
        double total_tm = 4.0, start_tm = total_tm*0.1, turnaround_tm = total_tm*0.4, start_resultant_force = 0.0, turnaround_resultant_force = 40.0, dphi;
        dphi = std::fabs(gen_forces_moments_for_saturation(total_tm, start_tm, turnaround_tm, start_resultant_force, turnaround_resultant_force, hrp::Vector3::UnitX()));
        octd.setDetectorTotalWrench(ObjectContactTurnaroundDetectorBase::GENERALIZED_WRENCH);
        std::vector<hrp::dvector6> ccm1(1, get_ccm1_by_index(0, false));
        double ref_dwrench = get_a_coeff_by_index(dphi, ccm1[0], -1*hrp::Vector3::UnitX());
        octd.setConstraintConversionMatricesRefDwrench(ccm1,
                                                       std::vector<hrp::dvector6>(1, hrp::dvector6::Zero()),
                                                       std::vector<double>(1, ref_dwrench));
        octd.setMaxTime(total_tm);
        octd.startDetectionForGeneralizedWrench();
        gen_pattern_and_plot ();
    };
    void test7 ()
    {
        std::cerr << "test7 : Tilt upward (GENERALIZED_WRENCH)" << std::endl;
        double total_tm = 4.0, start_tm = total_tm*0.1, turnaround_tm = total_tm*0.4, start_resultant_force = 0.0, turnaround_resultant_force = -40.0, dphi;
        dphi = std::fabs(gen_forces_moments_for_saturation(total_tm, start_tm, turnaround_tm, start_resultant_force, turnaround_resultant_force));
        octd.setDetectorTotalWrench(ObjectContactTurnaroundDetectorBase::GENERALIZED_WRENCH);
        std::vector<hrp::dvector6> ccm1(1, hrp::dvector6::Zero());
        ccm1[0](2) = 0.9; ccm1[0](4) = 1.0;
        double ref_dwrench = get_a_coeff_by_index(dphi, ccm1[0], hrp::Vector3::UnitZ());
        octd.setConstraintConversionMatricesRefDwrench(ccm1,
                                                       std::vector<hrp::dvector6>(1, hrp::dvector6::Zero()),
                                                       std::vector<double>(1, ref_dwrench));
        octd.setMaxTime(total_tm);
        octd.startDetectionForGeneralizedWrench();
        gen_pattern_and_plot ();
    };
    bool check_detection_time_validity (const double time_thre = 1.0) // [s]
    {
        return true_turnaround_time < detect_time && (detect_time-true_turnaround_time) < time_thre;
    };
    bool check_all_results ()
    {
        std::cerr << "Results:" << std::endl;
        std::cerr << "  Detected? : " << (octd.isDetected()?"true":"false") << std::endl;
        std::cerr << "  Detection time : " << (check_detection_time_validity()?"true":"false") << ", detect_time = " << detect_time << "[s], true_turnaround_time = " << true_turnaround_time << "[s]" << std::endl;
        return octd.isDetected() && check_detection_time_validity();
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
    std::cerr << "  --test0 : Increasing->saturation (TOTAL_FORCE)" << std::endl;
    std::cerr << "  --test1 : Increasing->saturation (GENERALIZED_WRENCH)" << std::endl;
    std::cerr << "  --test2 : Increasing->decreasing (GENERALIZED_WRENCH)" << std::endl;
    std::cerr << "  --test3 : Decreasing->saturation (GENERALIZED_WRENCH)" << std::endl;
    std::cerr << "  --test4 : Decreasing->increasing (GENERALIZED_WRENCH)" << std::endl;
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
        } else if (std::string(argv[1]) == "--test4") {
            toctd.test4();
        } else if (std::string(argv[1]) == "--test5") {
            toctd.test5();
        } else if (std::string(argv[1]) == "--test6") {
            toctd.test6();
        } else if (std::string(argv[1]) == "--test7") {
            toctd.test7();
        } else {
            print_usage();
            ret = 1;
        }
        ret = (toctd.check_all_results()?0:2);
    } else {
        print_usage();
        ret = 1;
    }
    return ret;
}

