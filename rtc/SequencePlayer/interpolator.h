#ifndef __INTERPOLATOR_H__
#define __INTERPOLATOR_H__

#include <deque>
#include <string>

using namespace std;

class interpolator
{
public:
  typedef enum {LINEAR, HOFFARBIB,QUINTICSPLINE,CUBICSPLINE} interpolation_mode;
  interpolator(int dim_, double dt_, interpolation_mode imode_=HOFFARBIB);
  ~interpolator();
  void push(const double *a, bool immediate=true);
  double *front();
  void get(double *a, bool popp=true);
  void set(const double *a);
  void go(const double *gx, const double *gv, double time, bool immediate=true);
  void go(const double *gx, double time, bool immediate=true);
  void pop();
  void pop_back();
  void clear();
  void sync();
  void load(string fname, double time_to_start=1.0, double scale=1.0,
	    bool immediate=true);
  void load(const char *fname, double time_to_start=1.0, double scale=1.0,
	    bool immediate=true);
  bool isEmpty();
  double remain_time();
#define DEFAULT_AVG_VEL	(0.5) // [rad/s]
  double calc_interpolation_time(const double *g, 
				 double avg_vel=DEFAULT_AVG_VEL);
  bool setInterpolationMode (interpolation_mode i_mode_);
  void setGoal(const double *gx, const double *gv, double time,
               bool online=true);
  void setGoal(const double *gx, double time);
  void interpolate(double& remain_t);
private:
  interpolation_mode imode;
  deque<double *> q;
  int length;
  int dim;
  double dt;
  double *x, *v, *a;
  double *gx, *gv, *ga;
  double target_t, remain_t;
  double *a0, *a1, *a2, *a3, *a4, *a5;

  void hoffarbib(double &remain_t,
		 double a0, double a1, double a2,
		 double a3, double a4, double a5,
		 double &xx, double &vv, double &aa);
  void linear_interpolation(double &remain_t,
			    double gx,
			    double &xx, double &vv, double &aa);
};

#endif
