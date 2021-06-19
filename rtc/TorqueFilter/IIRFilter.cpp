#include "IIRFilter.h"
#include <numeric>

IIRFilter::IIRFilter(unsigned int dim, std::vector<double>& fb_coeffs, std::vector<double>& ff_coeffs, const std::string& error_prefix)
{
    std::cerr << "This IIRFilter constructure is obsolated method." << std::endl;
    m_dimension = dim;
    m_error_prefix = error_prefix;

    // init coefficients
    if(fb_coeffs.size() != dim + 1|| ff_coeffs.size() != dim + 1){
        std::cout << "[" <<  m_error_prefix << "]" << "IIRFilter coefficients size error" << std::endl;
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
    m_initialized = true;
    return;
}

IIRFilter::IIRFilter(const std::string& error_prefix) :
    m_initialized(false) {
    m_error_prefix = error_prefix;
}

bool IIRFilter::setParameter(int dim, std::vector<double>& A, std::vector<double>& B) {
    m_dimension = dim;

    // init coefficients
    if((A.size() != dim && A.size() != dim + 1) || B.size() != dim + 1) {
        std::cout << "[" <<  m_error_prefix << "]" << "IIRFilter coefficients size error" << std::endl;
        return false;
    }

    // clear previous coefficients
    m_fb_coefficients.clear();
    m_ff_coefficients.clear();

    if (A.size() == dim) {
        m_fb_coefficients.push_back(1.0);
    }
    for(std::vector<double>::iterator it = A.begin(); it != A.end(); it++){
        if (it == A.begin()) {
            if( *it != 1.0 ) {
                std::cout << "[" <<  m_error_prefix << "]" << "IIRFilter : parameter A[0] is not 1.0 !!!" << std::endl;
            }
            m_fb_coefficients.push_back(*it);
        } else {
            m_fb_coefficients.push_back(- *it);
        }
    }
    for(std::vector<double>::iterator it = B.begin(); it != B.end(); it++){
        m_ff_coefficients.push_back(*it);
    }

    // init previous values
    m_previous_values.assign(dim, 0.0);
    m_initialized = true;
    return true;
}

bool IIRFilter::setParameterAsBiquad(const double f_cutoff, const double Q, const double hz){
    std::vector<double> fb_coeffs(3), ff_coeffs(3);
    const double omega = 2 * 3.14159265 * f_cutoff / hz;
    const double alpha = std::sin(omega) / (2 * Q);
    const double denom = 1 + alpha;
    fb_coeffs[0] = 1;
    fb_coeffs[1] = -2 * std::cos(omega) / denom;
    fb_coeffs[2] = (1 - alpha) / denom;
    ff_coeffs[0] = (1 - std::cos(omega)) / 2 / denom;
    ff_coeffs[1] = (1 - std::cos(omega)) / denom;
    ff_coeffs[2] = (1 - std::cos(omega)) / 2 / denom;
    return this->setParameter(2, fb_coeffs, ff_coeffs);
};


void IIRFilter::getParameter(int &dim, std::vector<double>& A, std::vector<double>& B)
{
    dim = m_dimension;
    B.resize(m_ff_coefficients.size());
    std::copy(m_ff_coefficients.begin(), m_ff_coefficients.end(), B.begin());
    A.resize(0);
    for(std::vector<double>::iterator it = m_fb_coefficients.begin();
        it != m_fb_coefficients.end(); it++) {
        if (it == m_fb_coefficients.begin()) {
            A.push_back(*it);
        } else {
            A.push_back(- *it);
        }
    }
}

void IIRFilter::reset(double initial_input)
{
    // y[n] = b[0]*w[n] + b[1]*w[n-1] + ... + b[m]*w[n-m] in DirectForm-II.
    // When n->inf, y[n]->initial_input and w[n], w[n-1], ..., w[n-m] -> w,
    // m_previous_values should preserve w
    double sum_ff_coeffs = std::accumulate(m_ff_coefficients.begin(), m_ff_coefficients.end(), 0.0);
    double reset_val = initial_input / sum_ff_coeffs;
    m_previous_values.assign(m_dimension, reset_val);
}

double IIRFilter::passFilter(double input)
{
    // IIRFilter implementation based on DirectForm-II.
    // Cf. https://en.wikipedia.org/wiki/Digital_filter
    if (! m_initialized) {
        return 0.0;
    }
    double feedback, filtered;
    // calcurate retval
    feedback = m_fb_coefficients[0] * input;
    for (int i = 0; i < m_dimension; i++) {
        feedback += m_fb_coefficients[i + 1] * m_previous_values[i];
    }
    filtered = m_ff_coefficients[0] * feedback;
    for(int i = 0; i < m_dimension; i++) {
        filtered += m_ff_coefficients[i + 1] * m_previous_values[i];
    }
    // update previous values
    m_previous_values.push_front(feedback);
    m_previous_values.pop_back();

    return filtered;
}
