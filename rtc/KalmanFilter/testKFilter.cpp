// -*- C++ -*-
/*!
 * @file  testKalmanFilter.cpp
 * @brief Test program for KalmanFilter.cpp requires input log data files
 * @date  $Date$
 *
 * $Id$
 */


#include <stdio.h>
#include <fstream>
#include "RPYKalmanFilter.h"

// Usage
// testKalmanFilter --acc-file /tmp/kftest/test.acc --rate-file /tmp/kftest/test.rate --pose-file /tmp/kftest/test.pose --Q-angle 1e-3 --Q-rate 1e-10 --dt 0.005 --R-angle 1 --use-gnuplot true

int main(int argc, char *argv[])
{
  // Default file names and params
  double Q_pos = 0.001;
  double Q_vel = 0.003;
  double R_pos = 0.005;
  double dt = 0.004;
  std::string input_file("test.dat");
  bool use_gnuplot = false;

  // Parse argument
  for (int i = 0; i < argc; ++ i) {
      std::string arg(argv[i]);
      if ( arg == "--Q-pos" ) { // KF parameters
          if (++i < argc) Q_pos = atof(argv[i]);
      } else if ( arg == "--Q-vel" ) {
          if (++i < argc) Q_vel = atof(argv[i]);
      } else if ( arg == "--R-pos" ) {
          if (++i < argc) R_pos = atof(argv[i]);
      } else if ( arg == "--dt" ) { // sampling time[s]
          if (++i < argc) dt = atof(argv[i]);
      } else if ( arg == "--input-file" ) { // File path for rate
          if (++i < argc) input_file = argv[i];
      } else if ( arg == "--use-gnuplot" ) { // Use gnuplot (true or false)
          if (++i < argc) use_gnuplot = (std::string(argv[i])=="true"?true:false);
      }
  }

  // Setup input and output files
  std::ifstream inputf(input_file.c_str());
  std::cerr << "File : " << input_file << std::endl;
  if (!inputf.is_open()) {
      std::cerr << "No such " << input_file << std::endl;
      return -1;
  }
  std::string ofname("/tmp/testKalmanFilter.dat");
  std::ofstream ofs(ofname.c_str());

  // Test kalman filter
  KFilter kf;
  //kf.setF(1, -dt, 0, 1);
  //kf.setB(dt, 0);
  kf.setQ(Q_pos*dt, 0, 0, Q_vel*dt);
  kf.setB(0, 0);
  kf.setF(1, dt, 0, 1);
  kf.setP(0, 0, 0, 0);
  kf.setR(R_pos);
  //hrp::Vector3 rate, acc, rpy, rpyRaw, baseRpyCurrent, rpyAct;
  double time, time2=0.0;
  double data;
  while(!inputf.eof()){
      inputf >> time >> data;
      kf.update(0, data);
      if (use_gnuplot) {
          ofs << time2 << " " << data << " " << kf.getx()[0] << " " << kf.getx()[1] << std::endl;
      } else {
          std::cout << data << std::endl;
      }
      time2+=dt;
  }
  // gnuplot
  if (use_gnuplot) {
      FILE* gp[2];
      std::string titles[2] = {"Pos", "Vel"};
      for (size_t ii = 0; ii < 2; ii++) {
          gp[ii] = popen("gnuplot", "w");
          fprintf(gp[ii], "set title \"%s\"\n", titles[ii].c_str());
          fprintf(gp[ii], "set xlabel \"Time [s]\"\n");
          fprintf(gp[ii], "set ylabel \"Pos\"\n");
          if (ii==0) {
              fprintf(gp[ii], "plot \"%s\" using 1:%d with lines title \"Filtered\"\n", ofname.c_str(), 2);
              fprintf(gp[ii], "replot \"%s\" using 1:%d with lines title \"Raw\"\n", ofname.c_str(), 3);
          } else {
              fprintf(gp[ii], "plot \"%s\" using 1:%d with lines title \"Vel\"\n", ofname.c_str(), 4);
          }
          fflush(gp[ii]);
      }
      std::cout << "Type any keys + enter to exit." << std::endl;
      double tmp;
      std::cin >> tmp;
      for (size_t j = 0; j < 2; j++) pclose(gp[j]);
  }
  return 0;
}
