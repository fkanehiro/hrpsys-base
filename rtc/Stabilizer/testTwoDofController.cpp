#include <iostream>
#include <vector>
#include "TwoDofController.h"
#include <boost/assign/std/vector.hpp>
#include <boost/assert.hpp>

#define ABS(x) (((x) < 0) ? (-(x)) : (x))

int main () {
  double ke = 400, tc = 0.04, dt = 0.005;
  TwoDofController tdc(ke, tc, dt);
  double q = 0, dq = 0, tau = 0, tau_d = 10.0;

  double time = 0, start = 0;
  while ( true ) {
    tau = -ke * q;
    dq = tdc.update(tau, tau_d);
    q += dq;
    start = time;
    if ( ABS(tau - tau_d) < 0.01 ){
      return 0;
    }
    std::cout << time << " " << q << " " << tau << " " << tau_d << std::endl;
    time += dt;
  }
  return 0;
}
