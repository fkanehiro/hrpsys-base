#ifndef OBJECTTURNAROUNDDETECTOR_H
#define OBJECTTURNAROUNDDETECTOR_H

#include "RatsMatrix.h"
#include "../TorqueFilter/IIRFilter.h"
#include <boost/shared_ptr.hpp>
#include <iostream>

class ObjectTurnaroundDetector
{
 private:
    typedef enum {MODE_IDLE, MODE_STARTED, MODE_DETECTED, MODE_MAX_TIME} process_mode;
    boost::shared_ptr<FirstOrderLowPassFilter<double> > wrench_filter;
    boost::shared_ptr<FirstOrderLowPassFilter<double> > dwrench_filter;
    double prev_wrench, dt;
    double detect_ratio_thre, start_ratio_thre, ref_dwrench, max_time, current_time;
    process_mode pmode;
    std::string print_str;
 public:
    ObjectTurnaroundDetector (const double _dt) : prev_wrench(0.0), dt(_dt), detect_ratio_thre(0.1), start_ratio_thre(0.2), pmode(MODE_IDLE)
    {
        double default_cutoff_freq = 10; // [Hz]
        wrench_filter = boost::shared_ptr<FirstOrderLowPassFilter<double> >(new FirstOrderLowPassFilter<double>(default_cutoff_freq, _dt, 0));
        dwrench_filter = boost::shared_ptr<FirstOrderLowPassFilter<double> >(new FirstOrderLowPassFilter<double>(default_cutoff_freq, _dt, 0));
    };
    ~ObjectTurnaroundDetector () {};
    void startDetection (const double _ref_dwrench, const double _max_time)
    {
        ref_dwrench = _ref_dwrench;
        max_time = _max_time;
        current_time = 0;
        std::cerr << "[" << print_str << "] Start Object Turnaround Detection (ref_dwrench = " << ref_dwrench
                  << ", detect_thre = " << detect_ratio_thre * ref_dwrench << ", start_thre = " << start_ratio_thre * ref_dwrench << "), max_time = " << max_time << "[s]" << std::endl;
        pmode = MODE_IDLE;
    };
    bool checkDetection (const double wrench_value)
    {
        double tmp_wr = wrench_filter->passFilter(wrench_value);
        double tmp_dwr = dwrench_filter->passFilter((tmp_wr-prev_wrench)/dt);
        prev_wrench = tmp_wr;
        switch (pmode) {
        case MODE_IDLE:
            if (tmp_dwr > ref_dwrench*start_ratio_thre) {
                pmode = MODE_STARTED;
                std::cerr << "[" << print_str << "] Object Turnaround Detection Started." << std::endl;
            }
            break;
        case MODE_STARTED:
            if (tmp_dwr < ref_dwrench*detect_ratio_thre) {
                pmode = MODE_DETECTED;
                std::cerr << "[" << print_str << "] Object Turnaround Detected (time = " << current_time << "[s])" << std::endl;
            }
            break;
        case MODE_DETECTED:
            break;
        case MODE_MAX_TIME:
            break;
        default:
            break;
        }
        if (max_time <= current_time) {
            pmode = MODE_MAX_TIME;
            std::cerr << "[" << print_str << "] Object Turnaround Detection max time reached." << std::endl;
        }
        current_time += dt;
        return (pmode == MODE_DETECTED);
    };
    void printParams () const
    {
        std::cerr << "[" << print_str << "]   ObjectTurnaroundDetector params" << std::endl;
        std::cerr << "[" << print_str << "]    wrench_cutoff_freq = " << wrench_filter->getCutOffFreq() << "[Hz], dwrench_cutoff_freq = " << dwrench_filter->getCutOffFreq() << "[Hz]" << std::endl;
        std::cerr << "[" << print_str << "]    detect_ratio_thre = " << detect_ratio_thre << ", start_ratio_thre = " << start_ratio_thre << std::endl;
    };
    void setPrintStr (const std::string& str) { print_str = str; };
    void setWrenchCutoffFreq (const double a) { wrench_filter->setCutOffFreq(a); };
    void setDwrenchCutoffFreq (const double a) { dwrench_filter->setCutOffFreq(a); };
    void setDetectRatioThre (const double a) { detect_ratio_thre = a; };
    double getWrenchCutoffFreq () { return wrench_filter->getCutOffFreq(); };
    double getDwrenchCutoffFreq () { return dwrench_filter->getCutOffFreq(); };
    double getFilteredWrench () { return wrench_filter->getCurrentValue(); };
    double getFilteredDwrench () { return dwrench_filter->getCurrentValue(); };
};
#endif // OBJECTTURNAROUNDDETECTOR_H
