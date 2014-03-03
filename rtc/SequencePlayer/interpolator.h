#ifndef __INTERPOLATOR_H__
#define __INTERPOLATOR_H__

#include <deque>
#include <string>

using namespace std;

class interpolator
{
public:
  typedef enum {LINEAR, HOFFARBIB,QUINTICSPLINE,CUBICSPLINE} interpolation_mode;
  interpolator(int dim_, double dt_, interpolation_mode imode_=HOFFARBIB, double default_avg_vel_=0.5); // default_avg_vel = [rad/s]
  ~interpolator();
  void push(const double *x, const double *v, const double *a, bool immediate=true);
  double *front();
  void get(double *x, bool popp=true);
  void get(double *x, double *v, bool popp=true);
  void get(double *x, double *v, double *a, bool popp=true);
  void set(const double *x, const double *v=NULL);
  void go(const double *gx, const double *gv, double time, bool immediate=true);
  void go(const double *gx, double time, bool immediate=true);
  void pop();
  void pop_back();
  void clear();
  void sync();
  void load(string fname, double time_to_start=1.0, double scale=1.0,
	    bool immediate=true, size_t offset1 = 0, size_t offset2 = 0);
  void load(const char *fname, double time_to_start=1.0, double scale=1.0,
	    bool immediate=true, size_t offset1 = 0, size_t offset2 = 0);
  bool isEmpty();
  double remain_time();
  double calc_interpolation_time(const double *g);
  bool setInterpolationMode (interpolation_mode i_mode_);
  void setGoal(const double *gx, const double *gv, double time,
               bool online=true);
  void setGoal(const double *gx, double time);
  void interpolate(double& remain_t);
  double deltaT() const { return dt; }
private:
  interpolation_mode imode;
  deque<double *> q, dq, ddq;
  int length;
  int dim;
  double dt;
  double *x, *v, *a;
  double *gx, *gv, *ga;
  double target_t, remain_t;
  double *a0, *a1, *a2, *a3, *a4, *a5;
  double default_avg_vel;

  void hoffarbib(double &remain_t,
		 double a0, double a1, double a2,
		 double a3, double a4, double a5,
		 double &xx, double &vv, double &aa);
  void linear_interpolation(double &remain_t,
			    double gx,
			    double &xx, double &vv, double &aa);
};

#endif
