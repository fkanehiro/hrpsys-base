/* -*- coding:utf-8-unix; mode:c++; -*- */
#include "PreviewController.h"

using namespace hrp;
using namespace rats;

struct gait_parameter /* for test */
{
  double tm;
  hrp::Vector3 ref_zmp;
  gait_parameter (const double _tm, const hrp::Vector3& _ref_zmp)
    : tm(_tm), ref_zmp(_ref_zmp) {};
};

#include<cstdio>

int main(int argc, char* argv[])
{
  /* this is c++ version example of test-preview-filter1-modified in euslib/jsk/preview.l*/
  bool use_gnuplot = true;
  if (argc >= 2) {
      if ( std::string(argv[1])== "--use-gnuplot" ) {
          use_gnuplot = (std::string(argv[2])=="true");
      }
  }

  double dt = 0.01, max_tm = 8.0;
  std::queue<hrp::Vector3> ref_zmp_list;
  std::deque<double> tm_list;
  for (size_t i = 0; i < static_cast<size_t>(round(max_tm / dt)); i++) {
    double tmp_tm = i * dt;
    tm_list.push_back(tmp_tm);
    hrp::Vector3 v;
    if (tmp_tm < 2) {
      v << 0, 0, 0;
    } else if (tmp_tm < 4) {
      v << -0.02, 0.02, 0;
    } else if (tmp_tm < 6) {
      v << 0.02, -0.02, 0;
    } else {
      v << 0, -0.02, 0;
    }
    ref_zmp_list.push(v);
  }

  //preview_dynamics_filter<preview_control> df(dt, 0.8, ref_zmp_list.front());
  preview_dynamics_filter<extended_preview_control> df(dt, 0.8, ref_zmp_list.front());
  std::string fname("/tmp/plot.dat");
  FILE* fp = fopen(fname.c_str(), "w");  
  double cart_zmp[3], refzmp[3];
  bool r = true;
  size_t index = 0;
  while (r) {
    hrp::Vector3 p, x;
    std::vector<hrp::Vector3> qdata;
    r = df.update(p, x, qdata, ref_zmp_list.front(), qdata, !ref_zmp_list.empty());
    if (r) {
      index++;
      df.get_cart_zmp(cart_zmp);
      df.get_current_refzmp(refzmp);
      fprintf(fp, "%f %f %f %f %f %f %f\n",
              tm_list[index],
              cart_zmp[0], /* zmpx ;; this zmp is "zmp as a table-cart model" */
              x[0], /* cogy */
              refzmp[0], /* refzmpx */
              cart_zmp[1], /* zmpy ;; this zmp is "zmp as a table-cart model" */
              x[1], /* cogy */
              refzmp[1] /* refzmpy */
              );
    } else if ( !ref_zmp_list.empty() ) r = true;
    if (!ref_zmp_list.empty()) ref_zmp_list.pop();
  }
  fclose(fp);
  if (use_gnuplot) {
  FILE* gp[3];
  std::string titles[2] = {"X", "Y"};
  for (size_t ii = 0; ii < 2; ii++) {
    gp[ii] = popen("gnuplot", "w");
    fprintf(gp[ii], "set title \"%s\"\n", titles[ii].c_str());
    fprintf(gp[ii], "plot \"%s\" using 1:%zu with lines title \"cart-table zmp\"\n", fname.c_str(), ( ii * 3 + 2));
    fprintf(gp[ii], "replot \"%s\" using 1:%zu with lines title \"cog\"\n", fname.c_str(), ( ii * 3 + 3));
    fprintf(gp[ii], "replot \"%s\" using 1:%zu with lines title \"refzmp\"\n", fname.c_str(), ( ii * 3 + 4));
    fflush(gp[ii]);
  }
  double tmp;
  std::cin >> tmp;
  for (size_t j = 0; j < 2; j++) pclose(gp[j]);
  }
  return 0;
}
