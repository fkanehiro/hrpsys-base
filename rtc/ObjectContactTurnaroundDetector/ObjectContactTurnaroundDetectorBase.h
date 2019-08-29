#ifndef OBJECTCONTACTTURNAROUNDDETECTORBASE_H
#define OBJECTCONTACTTURNAROUNDDETECTORBASE_H

#include "../TorqueFilter/IIRFilter.h"
#include <boost/shared_ptr.hpp>
#include <iostream>
#include <cmath>
#include "hrpsys/util/Hrpsys.h"
#include <hrpUtil/Eigen3d.h>

class ObjectContactTurnaroundDetectorBase
{
 public:
    typedef enum {MODE_IDLE, MODE_STARTED, MODE_DETECTED, MODE_MAX_TIME, MODE_OTHER_DETECTED} process_mode;
    typedef enum {TOTAL_FORCE, TOTAL_MOMENT, TOTAL_MOMENT2, GENERALIZED_WRENCH} detector_total_wrench;
 private:
    boost::shared_ptr<FirstOrderLowPassFilter<double> > wrench_filter;
    boost::shared_ptr<FirstOrderLowPassFilter<double> > dwrench_filter;
    boost::shared_ptr<FirstOrderLowPassFilter<double> > friction_coeff_wrench_filter;
    boost::shared_ptr<FirstOrderLowPassFilter<hrp::dvector6> > resultant_wrench_filter;
    boost::shared_ptr<FirstOrderLowPassFilter<hrp::dvector6> > resultant_dwrench_filter;
    hrp::Vector3 axis, moment_center;
    std::vector<hrp::dvector6> constraint_conversion_matrix1, constraint_conversion_matrix2;
    hrp::dvector6 filtered_resultant_wrench_with_hold;
    double dt;
    double detect_ratio_thre, start_ratio_thre, forgetting_ratio_thre, max_time, current_time;
    // detect_count_thre*dt, start_ratio_thre*dt, and other_detect_count_thre*dt are threshould for time.
    //   detect_count_thre*dt : Threshould for time [s] after the first object contact turnaround detection (Wait detect_time_thre [s] after first object contact turnaround detection).
    //   start_count_thre*dt  : Threshould for time [s] after the first starting detection (Wait start_time_thre [s] after first start detection).
    //   other_detect_count_thre*dt : Threshould for time [s] to move to MODE_OTHER_DETECTED after the first MODE_DETECTED, that is, do not check contact change other than detected element.
    size_t detect_count_thre, start_count_thre, other_detect_count_thre;
    detector_total_wrench dtw;
    std::string print_str;
    bool is_filter_reset, is_hold_values, is_other_constraint_detected;
    // Parameters which size can be changed, especially for GENERALIZED_WRENCH mode
    std::vector<double> ref_dwrench, raw_wrench, filtered_wrench_with_hold, filtered_friction_coeff_wrench_with_hold, phi1, phi2, dphi1;
    std::vector<process_mode> pmode;
    std::vector<size_t> count;
 public:
    ObjectContactTurnaroundDetectorBase (const double _dt) : axis(-1*hrp::Vector3::UnitZ()), moment_center(hrp::Vector3::Zero()),
                                                             constraint_conversion_matrix1(std::vector<hrp::dvector6>(1, hrp::dvector6::Zero())), constraint_conversion_matrix2(std::vector<hrp::dvector6>(1, hrp::dvector6::Zero())), filtered_resultant_wrench_with_hold(hrp::dvector6::Zero()),
                                                             dt(_dt), detect_ratio_thre(0.01), start_ratio_thre(0.5), forgetting_ratio_thre(1e3), // Too large threshold for forgetting ratio. Forgetting ratio is disabled by default.
                                                             max_time(0.0), current_time(0.0),
                                                             detect_count_thre(5), start_count_thre(5), other_detect_count_thre(round(detect_count_thre*1.5)), dtw(TOTAL_FORCE),
                                                             is_filter_reset(false), is_hold_values(false), is_other_constraint_detected(false),
                                                             ref_dwrench(std::vector<double>(1, 0.0)), raw_wrench(std::vector<double>(1, 0.0)), filtered_wrench_with_hold(std::vector<double>(1, 0.0)), filtered_friction_coeff_wrench_with_hold(std::vector<double>(1, 0.0)),
                                                             phi1(std::vector<double>(1, 0.0)), phi2(std::vector<double>(1, 0.0)), dphi1(std::vector<double>(1, 0.0)),
                                                             pmode(std::vector<process_mode>(1, MODE_MAX_TIME)), count(std::vector<size_t>(1))

    {
        double default_cutoff_freq = 1; // [Hz]
        wrench_filter = boost::shared_ptr<FirstOrderLowPassFilter<double> >(new FirstOrderLowPassFilter<double>(default_cutoff_freq, _dt, 0));
        dwrench_filter = boost::shared_ptr<FirstOrderLowPassFilter<double> >(new FirstOrderLowPassFilter<double>(default_cutoff_freq, _dt, 0));
        friction_coeff_wrench_filter = boost::shared_ptr<FirstOrderLowPassFilter<double> >(new FirstOrderLowPassFilter<double>(default_cutoff_freq, _dt, 0));
        resultant_wrench_filter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::dvector6> >(new FirstOrderLowPassFilter<hrp::dvector6>(default_cutoff_freq, _dt, hrp::dvector6::Zero()));
        resultant_dwrench_filter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::dvector6> >(new FirstOrderLowPassFilter<hrp::dvector6>(default_cutoff_freq, _dt, hrp::dvector6::Zero()));
    };
    ~ObjectContactTurnaroundDetectorBase () {};
    void startDetection (const double _ref_diff_wrench, const double _max_time)
    {
        if (ref_dwrench.size() != 1) ref_dwrench.resize(1);
        resizeVariablesForGeneralizedWrench(1);
        // NOTE : _ref_diff_wrench is difference. d_xx is velocity.
        ref_dwrench[0] = _ref_diff_wrench/_max_time;
        max_time = _max_time;
        startDetectionForGeneralizedWrench();
    };
    void startDetectionForGeneralizedWrench ()
    {
        for (size_t i = 0; i < count.size(); i++) {
            count[i] = 0;
            pmode[i] = MODE_IDLE;
        }
        current_time = 0;
        is_filter_reset = true;
        is_other_constraint_detected = false;
        std::cerr << "[" << print_str << "] Start Object Turnaround Detection (";
        std::cerr << "ref_dwrench = [";
        for (size_t i = 0; i < ref_dwrench.size(); i++) std::cerr << ref_dwrench[i] << ", ";
        std::cerr << "], detect_thre = [";
        for (size_t i = 0; i < ref_dwrench.size(); i++) std::cerr << ref_dwrench[i] * detect_ratio_thre << ", ";
        std::cerr << "], start_thre = [";
        for (size_t i = 0; i < ref_dwrench.size(); i++) std::cerr << ref_dwrench[i] * start_ratio_thre << ", ";
        std::cerr << "]), max_time = " << max_time << "[s]" << std::endl;
    };
    void resizeVariablesForGeneralizedWrench (size_t generalized_wrench_dim)
    {
        phi1.resize(generalized_wrench_dim);
        phi2.resize(generalized_wrench_dim);
        dphi1.resize(generalized_wrench_dim);
        filtered_wrench_with_hold.resize(generalized_wrench_dim);
        filtered_friction_coeff_wrench_with_hold.resize(generalized_wrench_dim);
        raw_wrench.resize(generalized_wrench_dim);
        count.resize(generalized_wrench_dim);
        pmode.resize(generalized_wrench_dim);
        for (size_t i = 0; i < generalized_wrench_dim; i++) {
            count[i] = 0;
            pmode[i] = MODE_MAX_TIME;
        }
    };
    void calcTotalForceMoment (hrp::Vector3& total_force, hrp::Vector3& total_moment1, hrp::Vector3& total_moment2,
                               const std::vector<hrp::Vector3>& forces, const std::vector<hrp::Vector3>& moments, const std::vector<hrp::Vector3>& hposv)
    {
        // Total wrench around the origin
        total_force = total_moment1 = total_moment2 = hrp::Vector3::Zero();
        for (size_t i = 0; i < forces.size(); i++) {
            total_force += forces[i];
            total_moment1 += hposv[i].cross(forces[i]);
            total_moment2 += moments[i];
        }
    };
    bool checkDetection (const std::vector<hrp::Vector3>& forces,
                         const std::vector<hrp::Vector3>& moments,
                         const std::vector<hrp::Vector3>& hposv)
    {
        // Calculate total force and moments
        //   forces, moments, hposv : Force, moment, position for all EE : f_i, n_i, p_i
        //   total_force : F = \sum_i f_i
        //   total_moment1 : M1 = \sum_i p_i \times f_i = \sum_i p_i \times f_i - p_c \times F
        //   total_moment2 : M2 = \sum_i n_i
        hrp::Vector3 total_force, total_moment1, total_moment2;
        calcTotalForceMoment(total_force, total_moment1, total_moment2, forces, moments, hposv);
        // Calculate generalized force/moment values and check detection
        bool ret = false;
        switch(dtw) {
        case TOTAL_FORCE:
            {
                ret = checkDetection(axis.dot(total_force), total_force(2));
                break;
            }
        case TOTAL_MOMENT:
            {
                // \sum_i (p_i - p_c) \times f_i = M1 - p_c \times F
                ret = checkDetection(axis.dot(total_moment1 - moment_center.cross(total_force)), 0.0);
            }
            break;
        case TOTAL_MOMENT2:
            {
                // \sum_i (p_i - p_c) \times f_i + n_i = M1 + M2 - p_c \times F
                ret = checkDetection(axis.dot(total_moment1 + total_moment2 - moment_center.cross(total_force)), 0.0);
            }
            break;
        case GENERALIZED_WRENCH:
            {
                hrp::dvector6 resultant_OR_wrench;
                for (size_t i = 0; i < 3; i++) {
                    resultant_OR_wrench(i) = total_force(i);
                    resultant_OR_wrench(i+3) = total_moment1(i) + total_moment2(i);
                }
                ret = checkDetection(resultant_OR_wrench);
            };
            break;
        default:
            break;
        };
        return ret;
    };
    bool checkDetection (const double raw_wrench_value, const double raw_friction_coeff_wrench_value)
    {
        if (is_filter_reset) {
          std::cerr << "[" << print_str << "] Object Turnaround Detection Reset Values. (raw_wrench_value = " << raw_wrench_value << ", raw_friction_coeff_wrench_value = " << raw_friction_coeff_wrench_value << ")" << std::endl;
          wrench_filter->reset(raw_wrench_value);
          dwrench_filter->reset(0);
          friction_coeff_wrench_filter->reset(raw_friction_coeff_wrench_value);
          filtered_wrench_with_hold[0] = wrench_filter->getCurrentValue();
          filtered_friction_coeff_wrench_with_hold[0] = friction_coeff_wrench_filter->getCurrentValue();
          is_filter_reset = false;
        }
        raw_wrench[0] = raw_wrench_value;
        double prev_filtered_wrench = wrench_filter->getCurrentValue();
        double filtered_wrench = wrench_filter->passFilter(raw_wrench_value);
        double filtered_dwrench = dwrench_filter->passFilter((filtered_wrench-prev_filtered_wrench)/dt);
        friction_coeff_wrench_filter->passFilter(raw_friction_coeff_wrench_value);
        // Hold values : is_hold_values is true and previously "detected", hold values. Otherwise, update values.
        if ( !(is_hold_values && isDetected()) ) {
            filtered_wrench_with_hold[0] = wrench_filter->getCurrentValue();
            filtered_friction_coeff_wrench_with_hold[0] = friction_coeff_wrench_filter->getCurrentValue();
        }
        // For logging
        phi1[0] = wrench_filter->getCurrentValue();
        phi2[0] = friction_coeff_wrench_filter->getCurrentValue();
        dphi1[0] = dwrench_filter->getCurrentValue();
        return updateProcessModeFromDwrench(std::vector<double>(1, filtered_dwrench));
    };
    bool checkDetection (const hrp::dvector6& raw_resultant_wrench_value)
    {
        if (is_filter_reset) {
          std::cerr << "[" << print_str << "] Object Turnaround Detection Reset Values. (raw_resultant_wrench_value = " << raw_resultant_wrench_value.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << ")" << std::endl;
          resultant_wrench_filter->reset(raw_resultant_wrench_value);
          resultant_dwrench_filter->reset(hrp::dvector6::Zero());
        }
        hrp::dvector6 prev_filtered_resultant_wrench = resultant_wrench_filter->getCurrentValue();
        hrp::dvector6 filtered_resultant_wrench = resultant_wrench_filter->passFilter(raw_resultant_wrench_value);
        hrp::dvector6 filtered_resultant_dwrench = resultant_dwrench_filter->passFilter((filtered_resultant_wrench - prev_filtered_resultant_wrench)/dt);
        calcPhiValueFromConstraintConversionMatrix(phi1, constraint_conversion_matrix1, filtered_resultant_wrench);
        calcPhiValueFromConstraintConversionMatrix(phi2, constraint_conversion_matrix2, filtered_resultant_wrench);
        calcPhiValueFromConstraintConversionMatrix(dphi1, constraint_conversion_matrix1, filtered_resultant_dwrench);
        if (is_filter_reset) {
            filtered_wrench_with_hold = phi1;
            filtered_friction_coeff_wrench_with_hold = phi2;
            filtered_resultant_wrench_with_hold = filtered_resultant_wrench;
            is_filter_reset = false;
        }
        // Hold values : is_hold_values is true and previously "detected", hold values. Otherwise, update values.
        if ( !(is_hold_values && isDetected()) ) {
            filtered_wrench_with_hold = phi1;
            filtered_friction_coeff_wrench_with_hold = phi2;
            filtered_resultant_wrench_with_hold = filtered_resultant_wrench;
        }
        // For logger, just used as buffer
        calcPhiValueFromConstraintConversionMatrix(raw_wrench, constraint_conversion_matrix1, raw_resultant_wrench_value);
        // Update process mode and return
        return updateProcessModeFromDwrench(dphi1);
    };
    void calcPhiValueFromConstraintConversionMatrix (std::vector<double>& phi, const std::vector<hrp::dvector6>& ccm, const hrp::dvector6& res_wrench)
    {
        for (size_t i = 0; i < ccm.size(); i++) phi[i] = ccm[i].dot(res_wrench);
    };
    bool updateProcessModeFromDwrench (const std::vector<double>& tmp_dwrench)
    {
        // Checking of wrench profile turn around
        //   Sign of ref_dwrench and tmp_dwrench shuold be same
        //   Supprot both ref_dwrench > 0 case and ref_dwrench < 0 case
        for (size_t i = 0; i < ref_dwrench.size(); i++) {
            if (!is_other_constraint_detected) {
                switch (pmode[i]) {
                case MODE_IDLE:
                    if ( (ref_dwrench[i] > 0.0) ? (tmp_dwrench[i] > ref_dwrench[i]*start_ratio_thre) : (tmp_dwrench[i] < ref_dwrench[i]*start_ratio_thre) ) {
                        count[i]++;
                        if (count[i] > start_count_thre) {
                            pmode[i] = MODE_STARTED;
                            count[i] = 0;
                            std::cerr << "[" << print_str << "] Object Turnaround Detection Started [idx=" << i << "]. (time = " << current_time << "[s], " << start_count_thre*dt << "[s] after the first start detection)" << std::endl;
                        }
                    } else {
                        /* count--; */
                    }
                    break;
                case MODE_STARTED:
                    if ( (ref_dwrench[i] > 0.0) ? (tmp_dwrench[i] < ref_dwrench[i]*detect_ratio_thre) : (tmp_dwrench[i] > ref_dwrench[i]*detect_ratio_thre) ) {
                        count[i]++;
                        if (count[i] > detect_count_thre) {
                            pmode[i] = MODE_DETECTED;
                            count[i] = 0;
                            std::cerr << "[" << print_str << "] Object Turnaround Detected [idx=" << i << "]. (time = " << current_time << "[s], " << detect_count_thre*dt << "[s] after the first detection)" << std::endl;
                        }
                    } else {
                        if ( ((ref_dwrench[i] > 0.0) ? (tmp_dwrench[i] > ref_dwrench[i]*forgetting_ratio_thre) : (tmp_dwrench[i] < ref_dwrench[i]*forgetting_ratio_thre)) &&
                             (count[i] > 0) ) {
                            count[i]--;
                        }
                    }
                    break;
                case MODE_DETECTED:
                    {
                        count[i]++;
                        if (count[i] > other_detect_count_thre) {
                            is_other_constraint_detected = true;
                            std::cerr << "[" << print_str << "] Object Turnaround Other Detected Time Limit [idx=" << i << "]. (time = " << current_time << "[s], " << other_detect_count_thre*dt << "[s] after the first detection)" << std::endl;
                        }
                    }
                    break;
                case MODE_MAX_TIME:
                    break;
                default:
                    break;
                }
                if (max_time <= current_time && (pmode[i] != MODE_DETECTED)) {
                    if (pmode[i] != MODE_MAX_TIME) std::cerr << "[" << print_str << "] Object Turnaround Detection max time reached. [idx=" << i << "]" << std::endl;
                    pmode[i] = MODE_MAX_TIME;
                }
            } else {
                if (pmode[i] != MODE_DETECTED) pmode[i] = MODE_OTHER_DETECTED;
            }
        }
        current_time += dt;
        return isDetected();
    };
    bool isDetected (const size_t idx) const { return (pmode[idx] == MODE_DETECTED); };
    bool isDetected () const
    {
        for (size_t i = 0; i < pmode.size(); i++) {
            if (isDetected(i)) return true;
        }
        return false;
    };
    void printParams () const
    {
        std::string tmpstr;
        switch (dtw) {
        case TOTAL_FORCE:
            tmpstr = "TOTAL_FORCE";break;
        case TOTAL_MOMENT:
            tmpstr = "TOTAL_MOMENT";break;
        case TOTAL_MOMENT2:
            tmpstr = "TOTAL_MOMENT2";break;
        case GENERALIZED_WRENCH:
            tmpstr = "GENERALIZED_WRENCH";break;
        default:
            tmpstr = "";break;
        }
        std::cerr << "[" << print_str << "]   ObjectContactTurnaroundDetectorBase params (" << tmpstr << ")" << std::endl;
        std::cerr << "[" << print_str << "]    wrench_cutoff_freq = " << wrench_filter->getCutOffFreq() << "[Hz], dwrench_cutoff_freq = " << dwrench_filter->getCutOffFreq() << "[Hz], friction_coeff_wrench_freq = " << friction_coeff_wrench_filter->getCutOffFreq()
                  << "[Hz], resultant_wrench_cutoff_freq = " << resultant_wrench_filter->getCutOffFreq() << "[Hz], resultant_dwrench_cutoff_freq = " << resultant_dwrench_filter->getCutOffFreq() << "[Hz]" << std::endl;
        std::cerr << "[" << print_str << "]    detect_ratio_thre = " << detect_ratio_thre << ", start_ratio_thre = " << start_ratio_thre << ", forgetting_ratio_thre = " << forgetting_ratio_thre
                  << ", start_time_thre = " << start_count_thre*dt << "[s], detect_time_thre = " << detect_count_thre*dt << "[s], other_detect_time_thre = " << other_detect_count_thre*dt << std::endl;
        std::cerr << "[" << print_str << "]    axis = [" << axis(0) << ", " << axis(1) << ", " << axis(2)
                  << "], moment_center = " << moment_center(0) << ", " << moment_center(1) << ", " << moment_center(2) << "][m]" << std::endl;
        std::cerr << "[" << print_str << "]    constraint_conversion_matrix1 = [";
        for (size_t i = 0; i < constraint_conversion_matrix1.size(); i++) {
            std::cerr << constraint_conversion_matrix1[i].format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]"));
        }
        std::cerr << "], constraint_conversion_matrix2 = [";
        for (size_t i = 0; i < constraint_conversion_matrix2.size(); i++) {
            std::cerr << constraint_conversion_matrix2[i].format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]"));
        }
        std::cerr << "]" << std::endl;
        std::cerr << "[" << print_str << "]    is_hold_values = " << (is_hold_values?"true":"false") << std::endl;
        std::cerr << "[" << print_str << "]    ref_dwrench = [";
        for (size_t i = 0; i < ref_dwrench.size(); i++) std::cerr << ref_dwrench[i] << ", ";
        std::cerr << "], max_time = " << max_time << "[s]" << std::endl;
    };
    // Setter
    void setPrintStr (const std::string& str) { print_str = str; };
    void setWrenchCutoffFreq (const double a)
    {
        // All filters for wrench related values have the same cutoff freq.
        wrench_filter->setCutOffFreq(a);
        friction_coeff_wrench_filter->setCutOffFreq(a);
        resultant_wrench_filter->setCutOffFreq(a);
    };
    void setDwrenchCutoffFreq (const double a)
    {
        // All filters for dwrench related values have the same cutoff freq.
        dwrench_filter->setCutOffFreq(a);
        resultant_dwrench_filter->setCutOffFreq(a);
    };
    // void setWrenchCutoffFreq (const double a) { wrench_filter->setCutOffFreq(a); };
    // void setDwrenchCutoffFreq (const double a) { dwrench_filter->setCutOffFreq(a); };
    // void setFrictionCoeffWrenchCutoffFreq (const double a) { friction_coeff_wrench_filter->setCutOffFreq(a); };
    void setDetectRatioThre (const double a) { detect_ratio_thre = a; };
    void setStartRatioThre (const double a) { start_ratio_thre = a; };
    void setDetectTimeThre (const double a) { detect_count_thre = round(a/dt); };
    void setStartTimeThre (const double a) { start_count_thre = round(a/dt); };
    void setOtherDetectTimeThre (const double a) { other_detect_count_thre = round(a/dt); };
    void setForgettingRatioThre (const double a) { forgetting_ratio_thre = a; };
    void setAxis (const hrp::Vector3& a) { axis = a; };
    void setMomentCenter (const hrp::Vector3& a) { moment_center = a; };
    void setConstraintConversionMatricesRefDwrench (const std::vector<hrp::dvector6>& ccm1, const std::vector<hrp::dvector6>& ccm2, const std::vector<double>& refdw)
    {
        constraint_conversion_matrix1 = ccm1;
        constraint_conversion_matrix2 = ccm2;
        ref_dwrench = refdw;
        resizeVariablesForGeneralizedWrench(constraint_conversion_matrix1.size());
    };
    void setDetectorTotalWrench (const detector_total_wrench _dtw)
    {
        if (_dtw != dtw) {
          is_filter_reset = true;
        }
        dtw = _dtw;
    };
    void setIsHoldValues (const bool a) { is_hold_values = a; };
    void setMaxTime (const double a) { max_time = a; };
    // Getter
    double getWrenchCutoffFreq () const { return wrench_filter->getCutOffFreq(); };
    double getDwrenchCutoffFreq () const { return dwrench_filter->getCutOffFreq(); };
    //double getFrictionCoeffWrenchCutoffFreq () const { return friction_coeff_wrench_filter->getCutOffFreq(); };
    double getDetectRatioThre () const { return detect_ratio_thre; };
    double getStartRatioThre () const { return start_ratio_thre; };
    double getDetectTimeThre () const { return detect_count_thre*dt; };
    double getStartTimeThre () const { return start_count_thre*dt; };
    double getOtherDetectTimeThre () const { return other_detect_count_thre*dt; };
    double getForgettingRatioThre () const { return forgetting_ratio_thre; };
    hrp::Vector3 getAxis () const { return axis; };
    hrp::Vector3 getMomentCenter () const { return moment_center; };
    void getConstraintConversionMatricesRefDwrench (std::vector<hrp::dvector6>& ccm1, std::vector<hrp::dvector6>& ccm2, std::vector<double>& refdw) const
    {
        ccm1 = constraint_conversion_matrix1;
        ccm2 = constraint_conversion_matrix2;
        refdw = ref_dwrench;
    };
    detector_total_wrench getDetectorTotalWrench () const { return dtw; };
    process_mode getMode (const size_t idx) const { return pmode[idx]; };
    size_t getDetectGeneralizedWrenchDim () const { return pmode.size(); };
    hrp::dvector getDataForLogger () const
    {
        size_t data_size = 5;
        hrp::dvector ret(pmode.size()*data_size);
        for (size_t i = 0; i < pmode.size(); i++) {
            size_t tmpoff = i*data_size;
            ret(tmpoff) = static_cast<double>(getMode(i));
            ret(1+tmpoff) = raw_wrench[i];
            ret(2+tmpoff) = phi1[i];
            ret(3+tmpoff) = dphi1[i];
            ret(4+tmpoff) = phi2[i];
        }
        return ret;
    };
    bool getIsHoldValues () const { return is_hold_values; };
    double getMaxTime () const { return max_time; };
    // For values with hold
    std::vector<double> getFilteredWrenchWithHold () const { return filtered_wrench_with_hold; };
    std::vector<double> getFilteredFrictionCoeffWrenchWithHold () const { return filtered_friction_coeff_wrench_with_hold; };
    hrp::dvector6 getFilteredResultantWrenchWithHold () const { return filtered_resultant_wrench_with_hold; };
};
#endif // OBJECTCONTACTTURNAROUNDDETECTORBASE_H
