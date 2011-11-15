#ifndef __INTERPOLATOR_H__
#define __INTERPOLATOR_H__

#include <deque>
#include <string>

using namespace std;

class interpolator
{
public:
  typedef enum {LINEAR, HOFFARBIB} interpolation_mode;
  interpolator(int dim_, double dt_, interpolation_mode imode_=HOFFARBIB);
  ~interpolator();
  void push(const double *a, bool immediate=true);
  double *front();
  void get(double *a, bool popp=true);
  void set(const double *a);
  void go(const double *g, double time, bool immediate=true);
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
  double delay;
private:
  interpolation_mode imode;
  deque<double *> q;
  int length;
  int dim;
  double dt;
  double *x, *v, *a, *g;

  void hoffarbib(double &remain_t, double goal,
		 double &xx, double &vv, double &aa);
  void linear_interpolation(double &remain_t, double goal,
			    double &xx, double &vv, double &aa);
};

#endif
