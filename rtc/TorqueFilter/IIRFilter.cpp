#include <iostream>
#include "IIRFilter.h"

IIRFilter::IIRFilter(unsigned int dim, std::vector<double>& fb_coeffs, std::vector<double>& ff_coeffs, const std::string& error_prefix)
{
  // init dimention
  m_dimention = dim;
  
  // init coefficients
  if(fb_coeffs.size() != dim + 1|| ff_coeffs.size() != dim + 1){
    std::cout << "[" <<  error_prefix << "]" << "IIRFilter coefficients size error" << std::endl;
    return;
  }
  for(std::vector<double>::iterator it = fb_coeffs.begin(); it != fb_coeffs.end(); it++){
    m_fb_coefficients.push_back(*it);
  }
  for(std::vector<double>::iterator it = ff_coeffs.begin(); it != ff_coeffs.end(); it++){
    m_ff_coefficients.push_back(*it);
  }
  
  // init previous values
  m_previous_values.assign(dim, 0.0);
 
  return;
}

IIRFilter::~IIRFilter()
{
};

double IIRFilter::executeFilter(double input)
{
  double feedback, filtered;
  // calcurate retval
  feedback = m_fb_coefficients[0] * input;
  for (int i = 0; i < m_dimention; i++){
    feedback += m_fb_coefficients[i + 1] * m_previous_values[i];    
  }
  filtered = m_ff_coefficients[0] * feedback;
  for(int i = 0; i < m_dimention; i++){
    filtered += m_ff_coefficients[i + 1] * m_previous_values[i];
  }
  // update previous values
  m_previous_values.push_front(feedback);
  m_previous_values.pop_back();
  
  return filtered;
}
