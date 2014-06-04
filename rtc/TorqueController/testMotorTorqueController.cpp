#include <iostream>
#include <string>
#include <stdlib.h>
#include "MotorTorqueController.h"

#define ABS(x) (((x) < 0) ? (-(x)) : (x))

int main (int argc, char* argv[]) {
  double ke = 2.0, kd = 20.0, tc = 0.05, dt = 0.005;
  const int test_num = 2;
  MotorTorqueController *controller[test_num];
  controller[0] = new MotorTorqueController("hoge", ke, tc, dt);
  controller[1] = new MotorTorqueController("hoge", ke, kd, tc, dt);
  double q[test_num], dq[test_num], q_ref[test_num], tau[test_num], dqref[test_num], ddqref[test_num];
  double tau_d = 10.0, limit = 90.0, pgain = 1.0, dgain = 0.01;
  bool activate_flag[test_num], stop_flag[test_num];

  // initialize
  for (int i = 0; i < test_num; i++){
    q[i] = dq[i] = q_ref[i] = tau[i] = dqref[i] = ddqref[i] = 0;
    activate_flag[i] = stop_flag[i] = false;
  }

  double time = 0;
  while ( true ) {
    for (int i = 0; i < test_num; i++) {
      controller[i]->setReferenceTorque(tau_d);

      if (time > 5.0 && activate_flag[i] == false) {
        controller[i]->activate();
        std::cerr << "#activated[" << i << "]" << std::endl;
        activate_flag[i] = true;
      }

      // execute controller
      tau[i] = -ke * q[i] + kd * (ddqref[i] / dt);
      dqref[i] = controller[i]->execute(tau[i], limit); // execute returns dqRef = sum(dq)
      ddqref[i] = dqref[i] - q[i];

      // joint simulation
      q[i] = dqref[i];

      if (activate_flag[i] == true) {
        if (time > 20 && stop_flag[i] != true) {
          controller[i]->deactivate();
          stop_flag[i] = true;
        } else if (time > 10) {
          tau_d = 5.0;
        }
      }
      if (time > 30) {
        return 0;
      }
    }
    std::cout << time << " " << q[0] << " " << dqref[0] << " " << tau[0] << " " << q[1] << " " << dqref[1] << " " << tau[1] << " " << tau_d << std::endl;
    time += dt;
  }
  return 0;
}
