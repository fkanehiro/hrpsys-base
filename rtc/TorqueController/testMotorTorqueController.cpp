#include <iostream>
#include <string>
#include <stdlib.h>
#include "MotorTorqueController.h"

#define ABS(x) (((x) < 0) ? (-(x)) : (x))

int main (int argc, char* argv[]) {
  double ke = 100, tc = 1.0, dt = 0.005;
  MotorTorqueController controller("hoge", ke, tc, dt);
  double q = 0, dq = 0, q_ref = 0, tau = 0, tau_d = 10.0, limit = 90.0;
  double qgain = 0.000001;

  if (argc > 1) {
    qgain = atof(argv[1]);
  }
  if (argc > 2) {
    tau_d = atof(argv[2]);
  }
  if (argc > 3) {
    limit = atof(argv[3]);
  }

  double time = 0;
  bool activate_flag = false; 
  bool stop_flag = false;
  bool state1_flag = false;
  while ( true ) {
    double tmp_q_ref = q_ref;
    controller.setReferenceTorque(tau_d);

    if (time > 5.0 && activate_flag == false) {
      controller.activate();
      std::cout << "#activated" << std::endl;
      activate_flag = true;
    }

    // execute controller
    dq = controller.execute(tau, limit);
    tmp_q_ref -= dq;

    // joint simulation
    tau = -ke * (q - tmp_q_ref);
    q += qgain * (tmp_q_ref - q);

    if (ABS(tau - tau_d) < 0.1 && state1_flag == false && time > 100) {
      tau_d = 5.0;
      state1_flag = true;
      std::cout << "#taud changed" << std::endl;
    } else if (ABS(tau - tau_d) < 0.1 && stop_flag == false && time > 200) {
      controller.deactivate();
      stop_flag = true;
      std::cout << "#stopped" << std::endl;
    }
    std::cout << time << " " << q << " " << dq << " " << tau << " " << tau_d << std::endl;
    time += dt;

    if((stop_flag == true && ABS(q) < 0.0001) || time > 500){
      return 0;
    }
    
  }
  return 0;
}
