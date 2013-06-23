// -*- C++ -*-
/*!
 * @file  HrpsysRTC.h
 * @brief header file for rtc components
 * @date  $Date$
 *
 * $Id$
 */

#ifdef __QNX__
#include <cmath>
#include <sys/syspage.h>
using std::FILE;
using std::exp;
using std::fabs;
using std::sprintf;
using std::strcmp;
using std::system;
using std::localtime;
using std::round;
using std::fopen;
using std::fprintf;
using std::fclose;
using std::fflush;
#endif