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

/**
   Infinite Impulse Filter
   y[n] = sum(0, dim, ff_coeffs[i] * x[n - i]) + sum(1, dim, fb_coeffs[i] * y[n - i])
 */
class IIRFilter
{
 public:
  
  /**
     \brief Constructor
     \param dim dimention of the filter
     \param fb_coeffs coeeficients of feedback
     \param ff_coeffs coefficients of feedforward
  */
  IIRFilter(int dim, std::vector<double>& fb_coeffs, std::vector<double>& ff_coeffs, const std::string& error_prefix = "");
  /**
     \brief Destructor
  */
  ~IIRFilter();

  /**
     \brief Execute filtering
  */
  double executeFilter(double input);

 private:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  int m_dimention;
  std::vector<double> m_fb_coefficients; // fb parameters (dim must be m_dimention + 1, m_fb_coefficients[0] would be 1.0)
  std::vector<double> m_ff_coefficients; // ff parameters (dim must be m_dimention + 1)
  std::deque<double> m_previous_values;

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
    FirstOrderLowPassFilter (const double _cutoff_freq, const double _dt, const T init_value) : dt(_dt), prev_value(init_value)
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
    double getCurrentValue () const { return prev_value; };
};

#endif // IIRFilter_H
