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
  IIRFilter(int dim, std::vector<double>& fb_coeffs, std::vector<double>& ff_coeffs);
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

#endif // IIRFilter_H
