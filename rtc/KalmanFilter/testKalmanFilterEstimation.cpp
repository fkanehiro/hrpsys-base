#include <fstream>
#include "RPYKalmanFilter.h"
#include "EKFilter.h"

int main(int argc, char *argv[])
{
  std::ifstream ratef("test.rate"), accf("test.acc"), posef("test.pose");
  RPYKalmanFilter rpy_kf;
  //rpy_kf.setParam(0.002, 0.001, 0.003, 1000);
  rpy_kf.setParam(0.002, 0.01, 0.003, 0.01);

  hrp::Vector3 rate, acc, rpy, rpyRaw, rpyAct;
  double time;
  while(!ratef.eof()){
    posef >> time >> time >> time >> time >> rpyAct[0] >> rpyAct[1] >> rpyAct[2];
    ratef >> time >> rate[0] >> rate[1] >> rate[2];
    accf >> time >> acc[0] >> acc[1] >> acc[2];
    rpy_kf.main_one(rpy, rpyRaw, acc, rate);
    std::cout << rpy[0] << " " << rpy[1] << " " << rpy[2] << " " << rpyAct[0] << " " << rpyAct[1] << " " << rpyAct[2] << std::endl;
  }
  return 0;
}
