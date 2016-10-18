// -*- C++ -*-

/*!
 * @file  IIRFIlter.h
 * @brief null component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef IIR_FILTER_H
#define IIR_FILTER_H

#include <vector>
#include <deque>
#include <string>
#include <cmath>
#include <iostream>

/**
   Infinite Impulse Filter
   y[n] = sum(0, dim, ff_coeffs[i] * x[n - i]) + sum(1, dim, fb_coeffs[i] * y[n - i])
 */
class IIRFilter
{
public:
    /**
       \brief Constructor
    */
    IIRFilter(const std::string& error_prefix = "");
    /**
       \brief Constructor, this constructure will be obsolated
       \param dim dimension of the filter
       \param fb_coeffs coeeficients of feedback
       \param ff_coeffs coefficients of feedforward
    */
    IIRFilter(unsigned int dim, std::vector<double>& fb_coeffs, std::vector<double>& ff_coeffs, const std::string& error_prefix = "");
    /**
       \brief Destructor
    */
    ~IIRFilter() {};

    /**
       \brief Set parameters
       Y[n] = B[0] * X[n] + B[1] * X[n-1] + ... + B[dim] * X[n-dim] - A[1] * Y[n-1] ... - A[dim] * Y[n-dim]
       A[0] would be 1.0

       How to generete parameter by octave
       butterworth filter (dimension = 2, cutoff_freq = 8Hz)
       [B, A] = butter(2, 2 * 0.004 * 8) ;;; dimension=2, 2 * dt * cutoff_freq
    */
    bool setParameter(int dim, std::vector<double>& A, std::vector<double>& B);

    /**
     */
    void getParameter(int &dim, std::vector<double>&A, std::vector<double>& B);

    /**
     */
    void reset(double initial_input = 0.0);

    /**
       \brief Execute filtering, this method will be obsolated
    */
    double executeFilter(double input) {
        // std::cerr << "executeFilter will be obsolated." << std::endl;
        return passFilter(input);
    };
    /**
       \brief passFilter
    */
    double passFilter(double input);
    // double getCurrentValue () const { return m_prev_output; };
private:
    // Configuration variable declaration
    // <rtc-template block="config_declare">
    int m_dimension;
    std::vector<double> m_fb_coefficients; // fb parameters (dim must be m_dimension + 1, m_fb_coefficients[0] would be 1.0)
    std::vector<double> m_ff_coefficients; // ff parameters (dim must be m_dimension + 1)
    std::deque<double> m_previous_values;
    // double m_prev_output;
    bool m_initialized;
    std::string m_error_prefix;
};

/**
   First order low pass filter
 */
template <class T> class FirstOrderLowPassFilter
{
private:
    T prev_value;
    double cutoff_freq, dt, const_param;
public:
    FirstOrderLowPassFilter (const double _cutoff_freq, const double _dt, const T init_value) : prev_value(init_value), dt(_dt)
    {
        setCutOffFreq(_cutoff_freq);
    };
    ~FirstOrderLowPassFilter()
    {
    };
    T passFilter (T value)
    {
        prev_value = 1.0/(1+const_param) * prev_value + const_param/(1+const_param) * value;
        return prev_value;
    };
    void reset (T value) { prev_value = value; };
    void setCutOffFreq (const double f)
    {
        cutoff_freq = f;
        const_param = 2 * M_PI * cutoff_freq * dt;
    };
    double getCutOffFreq () const { return cutoff_freq; };
    T getCurrentValue () const { return prev_value; };
};

#endif // IIRFilter_H
