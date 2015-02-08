// -*- C++ -*-
/*!
 * @file  testKalmanFilterEstimation.cpp
 * @brief Test program for KalmanFilter.cpp requires input log data files
 * @date  $Date$
 *
 * $Id$
 */


#include <fstream>
#include "RPYKalmanFilter.h"
#include "EKFilter.h"

// Usage
// testKalmanFilterEstimation --acc-file /tmp/kftest/test.acc --rate-file /tmp/kftest/test.rate --pose-file /tmp/kftest/test.pose --Q-angle 1e-3 --Q-rate 1e-10 --dt 0.005 --R-angle 1 --use-gnuplot true

int main(int argc, char *argv[])
{
  // Default file names and params
  double Q_angle = 0.001;
  double Q_rate = 0.003;
  double R_angle = 10;
  double dt = 0.002;
  std::string pose_file("test.pose"), acc_file("test.acc"), rate_file("test.rate");
  bool use_gnuplot = false;

  // Parse argument
  for (int i = 0; i < argc; ++ i) {
      std::string arg(argv[i]);
      if ( arg == "--Q-angle" ) { // KF parameters
          if (++i < argc) Q_angle = atof(argv[i]);
      } else if ( arg == "--Q-rate" ) {
          if (++i < argc) Q_rate = atof(argv[i]);
      } else if ( arg == "--R-angle" ) {
          if (++i < argc) R_angle = atof(argv[i]);
      } else if ( arg == "--dt" ) { // sampling time[s]
          if (++i < argc) dt = atof(argv[i]);
      } else if ( arg == "--rate-file" ) { // File path for rate
          if (++i < argc) rate_file = argv[i];
      } else if ( arg == "--acc-file" ) { // File path for acc
          if (++i < argc) acc_file = argv[i];
      } else if ( arg == "--pose-file" ) { // File path for actual pose
          if (++i < argc) pose_file = argv[i];
      } else if ( arg == "--use-gnuplot" ) { // Use gnuplot (true or false)
          if (++i < argc) use_gnuplot = (std::string(argv[i])=="true"?true:false);
      }
  }

  // Setup input and output files
  std::ifstream ratef(rate_file.c_str()), accf(acc_file.c_str()), posef(pose_file.c_str());
  std::cerr << "File : " << rate_file << " " << acc_file << " " << pose_file << std::endl;
  if (!ratef.is_open() || !accf.is_open() || !posef.is_open()) {
      std::cerr << "No such " << rate_file << " " << acc_file << " " << pose_file << std::endl;
      return -1;
  }
  std::string ofname("/tmp/testKalmanFilterEstimation.dat");
  std::ofstream ofs(ofname.c_str());

  // Test kalman filter
  RPYKalmanFilter rpy_kf;
  rpy_kf.setParam(dt, Q_angle, Q_rate, R_angle);
  hrp::Vector3 rate, acc, rpy, rpyRaw, rpyAct;
  double time, time2=0.0;
  while(!ratef.eof()){
    posef >> time >> time >> time >> time >> rpyAct[0] >> rpyAct[1] >> rpyAct[2]; // Neglect translation in .pose file
    ratef >> time >> rate[0] >> rate[1] >> rate[2];
    accf >> time >> acc[0] >> acc[1] >> acc[2];
    rpy_kf.main_one(rpy, rpyRaw, acc, rate);
    // rad->deg
    rpy*=180/3.14159;
    rpyAct*=180/3.14159;
    if (use_gnuplot) {
        ofs << time2 << " " << rpy[0] << " " << rpy[1] << " " << rpy[2] << " " << rpyAct[0] << " " << rpyAct[1] << " " << rpyAct[2] << std::endl;
    } else {
        std::cout << rpy[0] << " " << rpy[1] << " " << rpy[2] << " " << rpyAct[0] << " " << rpyAct[1] << " " << rpyAct[2] << std::endl;
    }
    time2+=dt;
  }
  // gnuplot
  if (use_gnuplot) {
      FILE* gp[3];
      std::string titles[3] = {"Roll", "Pitch", "Yaw"};
      for (size_t ii = 0; ii < 3; ii++) {
          gp[ii] = popen("gnuplot", "w");
          fprintf(gp[ii], "set title \"%s\"\n", titles[ii].c_str());
          fprintf(gp[ii], "set xlabel \"Time [s]\"\n");
          fprintf(gp[ii], "set ylabel \"Attitude [rad]\"\n");
          fprintf(gp[ii], "plot \"%s\" using 1:%d with lines title \"Estimated\"\n", ofname.c_str(), (2 + ii));
          fprintf(gp[ii], "replot \"%s\" using 1:%d with lines title \"Actual\"\n", ofname.c_str(), (2 + ii + 3));
          fflush(gp[ii]);
      }
      std::cout << "Type any keys + enter to exit." << std::endl;
      double tmp;
      std::cin >> tmp;
      for (size_t j = 0; j < 3; j++) pclose(gp[j]);
  }
  return 0;
}
