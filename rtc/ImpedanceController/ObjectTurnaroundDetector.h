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
    typedef enum {TOTAL_FORCE, TOTAL_MOMENT} detector_total_wrench;
 private:
    boost::shared_ptr<FirstOrderLowPassFilter<double> > wrench_filter;
    boost::shared_ptr<FirstOrderLowPassFilter<double> > dwrench_filter;
    hrp::Vector3 axis, moment_center;
    double prev_wrench, dt;
    double detect_ratio_thre, start_ratio_thre, ref_dwrench, max_time, current_time;
    size_t count;
    // detect_count_thre*dt and start_ratio_thre*dt are threshould for time.
    //   detect_count_thre*dt : Threshould for time [s] after the first object turnaround detection (Wait detect_time_thre [s] after first object turnaround detection).
    //   start_count_thre*dt  : Threshould for time [s] after the first starting detection (Wait start_time_thre [s] after first start detection).
    size_t detect_count_thre, start_count_thre;
    process_mode pmode;
    detector_total_wrench dtw;
    std::string print_str;
    bool is_dwr_changed;
 public:
    ObjectTurnaroundDetector (const double _dt) : axis(-1*hrp::Vector3::UnitZ()), moment_center(hrp::Vector3::Zero()), prev_wrench(0.0), dt(_dt), detect_ratio_thre(0.01), start_ratio_thre(0.5),
      count(0), detect_count_thre(5), start_count_thre(5), pmode(MODE_IDLE), dtw(TOTAL_FORCE), is_dwr_changed(false)
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
    double calcTotalForce (const std::vector<hrp::Vector3>& fmv)
    {
        hrp::Vector3 tmpv = hrp::Vector3::Zero();
        for (size_t i = 0; i < fmv.size(); i++) {
            tmpv += fmv[i];
        }
        return axis.dot(tmpv);
    };
    double calcTotalMoment (const std::vector<hrp::Vector3>& fmv, const std::vector<hrp::Vector3>& hposv)
    {
        hrp::Vector3 tmpv = hrp::Vector3::Zero();
        for (size_t i = 0; i < fmv.size(); i++) {
            tmpv += (hposv[i]-moment_center).cross(fmv[i]);
        }
        return axis.dot(tmpv);
    };
    bool checkDetection (const std::vector<hrp::Vector3>& fmv, const std::vector<hrp::Vector3>& hposv)
    {
        switch(dtw) {
        case TOTAL_FORCE:
            checkDetection(calcTotalForce(fmv));
            break;
        case TOTAL_MOMENT:
            checkDetection(calcTotalMoment(fmv, hposv));
            break;
        default:
            break;
        };
    };
    bool checkDetection (const double wrench_value)
    {
        if (is_dwr_changed) {
          wrench_filter->reset(wrench_value);
          dwrench_filter->reset(0);
          is_dwr_changed = false;
        }
        double tmp_wr = wrench_filter->passFilter(wrench_value);
        double tmp_dwr = dwrench_filter->passFilter((tmp_wr-prev_wrench)/dt);
        prev_wrench = tmp_wr;
        switch (pmode) {
        case MODE_IDLE:
            if (tmp_dwr > ref_dwrench*start_ratio_thre) {
                count++;
                if (count > start_count_thre) {
                    pmode = MODE_STARTED;
                    count = 0;
                    std::cerr << "[" << print_str << "] Object Turnaround Detection Started. (" << start_count_thre*dt << "[s] after the first start detection)" << std::endl;
                }
            } else {
                /* count--; */
            }
            break;
        case MODE_STARTED:
            if (tmp_dwr < ref_dwrench*detect_ratio_thre) {
                count++;
                if (count > detect_count_thre) {
                    pmode = MODE_DETECTED;
                    std::cerr << "[" << print_str << "] Object Turnaround Detected (time = " << current_time << "[s], " << detect_count_thre*dt << "[s] after the first detection)" << std::endl;
                }
            } else {
                /* count--; */
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
        std::cerr << "[" << print_str << "]   ObjectTurnaroundDetector params (" << (dtw==TOTAL_FORCE?"TOTAL_FORCE":"TOTAL_MOMENT") << ")" << std::endl;
        std::cerr << "[" << print_str << "]    wrench_cutoff_freq = " << wrench_filter->getCutOffFreq() << "[Hz], dwrench_cutoff_freq = " << dwrench_filter->getCutOffFreq() << "[Hz]" << std::endl;
        std::cerr << "[" << print_str << "]    detect_ratio_thre = " << detect_ratio_thre << ", start_ratio_thre = " << start_ratio_thre
                  << ", start_time_thre = " << start_count_thre*dt << "[s], detect_time_thre = " << detect_count_thre*dt << "[s]" << std::endl;
        std::cerr << "[" << print_str << "]    axis = [" << axis(0) << ", " << axis(1) << ", " << axis(2)
                  << "], moment_center = " << moment_center(0) << ", " << moment_center(1) << ", " << moment_center(2) << "][m]" << std::endl;
    };
    void setPrintStr (const std::string& str) { print_str = str; };
    void setWrenchCutoffFreq (const double a) { wrench_filter->setCutOffFreq(a); };
    void setDwrenchCutoffFreq (const double a) { dwrench_filter->setCutOffFreq(a); };
    void setDetectRatioThre (const double a) { detect_ratio_thre = a; };
    void setStartRatioThre (const double a) { start_ratio_thre = a; };
    void setDetectTimeThre (const double a) { detect_count_thre = static_cast<size_t>(a/dt); };
    void setStartTimeThre (const double a) { start_count_thre = static_cast<size_t>(a/dt); };
    void setAxis (const hrp::Vector3& a) { axis = a; };
    void setMomentCenter (const hrp::Vector3& a) { moment_center = a; };
    void setDetectorTotalWrench (const detector_total_wrench _dtw)
    {
        if (_dtw != dtw) {
          is_dwr_changed = true;
        }
        dtw = _dtw;
    };
    double getWrenchCutoffFreq () const { return wrench_filter->getCutOffFreq(); };
    double getDwrenchCutoffFreq () const { return dwrench_filter->getCutOffFreq(); };
    double getDetectRatioThre () const { return detect_ratio_thre; };
    double getStartRatioThre () const { return start_ratio_thre; };
    double getDetectTimeThre () const { return detect_count_thre*dt; };
    double getStartTimeThre () const { return start_count_thre*dt; };
    hrp::Vector3 getAxis () const { return axis; };
    hrp::Vector3 getMomentCenter () const { return moment_center; };
    detector_total_wrench getDetectorTotalWrench () const { return dtw; };
    double getFilteredWrench () const { return wrench_filter->getCurrentValue(); };
    double getFilteredDwrench () const { return dwrench_filter->getCurrentValue(); };
};
#endif // OBJECTTURNAROUNDDETECTOR_H
