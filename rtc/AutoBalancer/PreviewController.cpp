/* -*- coding:utf-8-unix; mode:c++; -*- */
#include "PreviewController.h"

using namespace hrp;
using namespace rats;

template <std::size_t dim>
void preview_control_base<dim>::update_x_k(const hrp::Vector3& pr)
{
  zmp_z = pr(2);
  Eigen::Matrix<double, 2, 1> tmpv;
  tmpv(0,0) = pr(0);
  tmpv(1,0) = pr(1);
  p.push_back(tmpv);
  if ( p.size() > 1 + delay ) p.pop_front();
  if ( is_doing() ) calc_x_k();
}

template <std::size_t dim>
void preview_control_base<dim>::update_zc(double zc)
{
  riccati.c(0, 2) = - zc / g; 
  riccati.solve();
}

void preview_control::calc_f()
{
  f.resize(delay+1);
  Eigen::Matrix<double, 1, 1> fa;
  hrp::Matrix33 gsi(hrp::Matrix33::Identity());
  for (size_t i = 0; i < delay; i++) {
    fa = riccati.R_btPb_inv * riccati.b.transpose() * (gsi * riccati.Q * riccati.c.transpose());
    gsi = riccati.A_minus_bKt * gsi;
    f(i+1) = fa(0,0);
  }
}

void preview_control::calc_u()
{
  Eigen::Matrix<double, 1, 2> gfp(Eigen::Matrix<double, 1, 2>::Zero());
  for (size_t i = 0; i < 1 + delay; i++)
    gfp += f(i) * p[i];
  u_k = -riccati.K * x_k + gfp;
};

void preview_control::calc_x_k()
{
  calc_u();
  x_k = riccati.A * x_k + riccati.b * u_k;  
}

void extended_preview_control::calc_f()
{
  f.resize(delay + 1);
  Eigen::Matrix<double, 1, 1> fa;
  Eigen::Matrix<double, 4, 4> gsi(Eigen::Matrix<double, 4, 4>::Identity());
  Eigen::Matrix<double, 4, 1> qt(riccati.Q * riccati.c.transpose());
  for (size_t i = 0; i < delay; i++) {
    if ( i == delay - 1 ) qt = riccati.P * qt;
    fa = riccati.R_btPb_inv * riccati.b.transpose() * (gsi * qt);
    gsi = riccati.A_minus_bKt * gsi;
    f(i+1) = fa(0,0);
  }
}

void extended_preview_control::calc_u()
{
  Eigen::Matrix<double, 1, 2> gfp(Eigen::Matrix<double, 1, 2>::Zero());
  for (size_t i = 0; i < 1 + delay; i++)
    gfp += f(i) * p[i];
  u_k = -riccati.K * x_k_e + gfp;
};

void extended_preview_control::calc_x_k()
{
  calc_u();
  x_k_e = riccati.A * x_k_e + riccati.b * u_k;
  for (size_t i = 0; i < 3; i++)
    for (size_t j = 0; j < 2; j++)
      x_k(i,j) += x_k_e(i+1,j);
}

#ifdef HAVE_MAIN

struct gait_parameter /* for test */
{
  double tm;
  hrp::Vector3 ref_zmp;
  gait_parameter (const double _tm, const hrp::Vector3& _ref_zmp)
    : tm(_tm), ref_zmp(_ref_zmp) {};
};

#include<cstdio>

int main()
{
  /* this is c++ version example of test-preview-filter1-modified in euslib/jsk/preview.l*/

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
    r = df.update(p, x, ref_zmp_list.front(), !ref_zmp_list.empty());
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
  FILE* gp[3];
  std::string titles[2] = {"X", "Y"};
  for (size_t ii = 0; ii < 2; ii++) {
    gp[ii] = popen("gnuplot", "w");
    fprintf(gp[ii], "set title \"%s\"\n", titles[ii].c_str());
    fprintf(gp[ii], "plot \"%s\" using 1:%d with lines title \"cart-table zmp\"\n", fname.c_str(), ( ii * 3 + 2));
    fprintf(gp[ii], "replot \"%s\" using 1:%d with lines title \"cog\"\n", fname.c_str(), ( ii * 3 + 3));
    fprintf(gp[ii], "replot \"%s\" using 1:%d with lines title \"refzmp\"\n", fname.c_str(), ( ii * 3 + 4));
    fflush(gp[ii]);
  }
  double tmp;
  std::cin >> tmp;
  for (size_t j = 0; j < 2; j++) pclose(gp[j]);
  return 0;
}
#endif
