#include <fstream>
#include "RPYKalmanFilter.h"
#include "EKFilter.h"

int main(int argc, char *argv[])
{
  double Q_angle = 0.001;
  double Q_rate = 0.003;
  double R_angle = 10;
  double dt = 0.002;
  std::string pose_file("test.pose"), acc_file("test.acc"), rate_file("test.rate");
  for (int i = 0; i < argc; ++ i) {
      std::string arg(argv[i]);
      if ( arg == "--Q-angle" ) {
          if (++i < argc) Q_angle = atof(argv[i]);
      } else if ( arg == "--Q-rate" ) {
          if (++i < argc) Q_rate = atof(argv[i]);
      } else if ( arg == "--R-angle" ) {
          if (++i < argc) R_angle = atof(argv[i]);
      } else if ( arg == "--dt" ) {
          if (++i < argc) dt = atof(argv[i]);
      } else if ( arg == "--rate-file" ) {
          if (++i < argc) rate_file = argv[i];
      } else if ( arg == "--acc-file" ) {
          if (++i < argc) acc_file = argv[i];
      } else if ( arg == "--pose-file" ) {
          if (++i < argc) pose_file = argv[i];
      }
  }
  std::ifstream ratef(rate_file.c_str()), accf(acc_file.c_str()), posef(pose_file.c_str());
  std::cerr << "File : " << rate_file << " " << acc_file << " " << pose_file << std::endl;
  if (!ratef.is_open() || !accf.is_open() || !posef.is_open()) {
      std::cerr << "No such " << rate_file << " " << acc_file << " " << pose_file << std::endl;
      return -1;
  }
  RPYKalmanFilter rpy_kf;
  rpy_kf.setParam(dt, Q_angle, Q_rate, R_angle);

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
