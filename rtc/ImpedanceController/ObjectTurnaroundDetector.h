#ifndef OBJECTTURNAROUNDDETECTOR_H
#define OBJECTTURNAROUNDDETECTOR_H

#include "RatsMatrix.h"
#include "../TorqueFilter/IIRFilter.h"
#include <boost/shared_ptr.hpp>
#include <iostream>

class ObjectTurnaroundDetector
{
 public:
    typedef enum {MODE_IDLE, MODE_STARTED, MODE_DETECTED, MODE_MAX_TIME} process_mode;
 private:
    boost::shared_ptr<FirstOrderLowPassFilter<double> > wrench_filter;
    boost::shared_ptr<FirstOrderLowPassFilter<double> > dwrench_filter;
    hrp::Vector3 axis;
    double prev_wrench, dt;
    double detect_ratio_thre, start_ratio_thre, ref_dwrench, max_time, current_time;
    size_t count;
    process_mode pmode;
    std::string print_str;
 public:
    ObjectTurnaroundDetector (const double _dt) : axis(-1*hrp::Vector3::UnitZ()), prev_wrench(0.0), dt(_dt), detect_ratio_thre(0.01), start_ratio_thre(0.5), pmode(MODE_IDLE)
    {
        double default_cutoff_freq = 1; // [Hz]
        wrench_filter = boost::shared_ptr<FirstOrderLowPassFilter<double> >(new FirstOrderLowPassFilter<double>(default_cutoff_freq, _dt, 0));
        dwrench_filter = boost::shared_ptr<FirstOrderLowPassFilter<double> >(new FirstOrderLowPassFilter<double>(default_cutoff_freq, _dt, 0));
    };
    ~ObjectTurnaroundDetector () {};
    void startDetection (const double _ref_diff_wrench, const double _max_time)
    {
        ref_dwrench = _ref_diff_wrench/_max_time;
        max_time = _max_time;
        current_time = 0;
        count = 0;
        std::cerr << "[" << print_str << "] Start Object Turnaround Detection (ref_dwrench = " << ref_dwrench
                  << ", detect_thre = " << detect_ratio_thre * ref_dwrench << ", start_thre = " << start_ratio_thre * ref_dwrench << "), max_time = " << max_time << "[s]" << std::endl;
        pmode = MODE_IDLE;
    };
    bool checkDetection (const std::vector<hrp::Vector3>& fmv)
    {
        hrp::Vector3 tmpv = hrp::Vector3::Zero();
        for (size_t i = 0; i < fmv.size(); i++) {
            tmpv += fmv[i];
        }
        checkDetection(axis.dot(tmpv));
    };
    bool checkDetection (const double wrench_value)
    {
        double tmp_wr = wrench_filter->passFilter(wrench_value);
        double tmp_dwr = dwrench_filter->passFilter((tmp_wr-prev_wrench)/dt);
        prev_wrench = tmp_wr;
        switch (pmode) {
        case MODE_IDLE:
            if (tmp_dwr > ref_dwrench*start_ratio_thre) {
                count++;
                if (count > 5) {
                    pmode = MODE_STARTED;
                    count = 0;
                    std::cerr << "[" << print_str << "] Object Turnaround Detection Started." << std::endl;
                }
            }
            break;
        case MODE_STARTED:
            if (tmp_dwr < ref_dwrench*detect_ratio_thre) {
                count++;
                if (count > 5) {
                    pmode = MODE_DETECTED;
                    std::cerr << "[" << print_str << "] Object Turnaround Detected (time = " << current_time << "[s])" << std::endl;
                }
            }
            //std::cerr << "[" << print_str << "] " << tmp_wr << " " << tmp_dwr << " " << count << std::endl;
            break;
        case MODE_DETECTED:
            break;
        case MODE_MAX_TIME:
            break;
        default:
            break;
        }
        if (max_time <= current_time && (pmode != MODE_DETECTED)) {
            if (pmode != MODE_MAX_TIME) std::cerr << "[" << print_str << "] Object Turnaround Detection max time reached." << std::endl;
            pmode = MODE_MAX_TIME;
        }
        current_time += dt;
        return isDetected();
    };
    bool isDetected () const { return (pmode == MODE_DETECTED); };
    process_mode getMode () const { return pmode; };
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
    void setStartRatioThre (const double a) { start_ratio_thre = a; };
    void setAxis (const hrp::Vector3& a) { axis = a; };
    double getWrenchCutoffFreq () const { return wrench_filter->getCutOffFreq(); };
    double getDwrenchCutoffFreq () const { return dwrench_filter->getCutOffFreq(); };
    double getDetectRatioThre () const { return detect_ratio_thre; };
    double getStartRatioThre () const { return start_ratio_thre; };
    hrp::Vector3 getAxis () const { return axis; };
    double getFilteredWrench () const { return wrench_filter->getCurrentValue(); };
    double getFilteredDwrench () const { return dwrench_filter->getCurrentValue(); };
};
#endif // OBJECTTURNAROUNDDETECTOR_H
