#ifndef __INTERPOLATOR_H__
#define __INTERPOLATOR_H__

#include <deque>
#include <string>

using namespace std;

class interpolator
{
  // interpolator class is to interpolate from current value to goal value considering position, velocities, and accelerations.
  //   Two status : empty or not
  //                Interpolator interpolates based on remaining time (remain_t) and pushes value to queue (q, dq, ddq).
  //                Users can get interpolated results from queue (q, dq, ddq).
  //                If remain_t <= 0 and queue is empty, interpolator is "empty", otherwise "not empty".
  //                This is related with isEmpty() function.
  //   Setting goal value : setGoal(), go(), and load()
  //   Getting current value : get()
  //   Resetting current value : set()
  //   Interpolate : interpolate()
public:
  typedef enum {LINEAR, HOFFARBIB,QUINTICSPLINE,CUBICSPLINE} interpolation_mode;
  interpolator(int dim_, double dt_, interpolation_mode imode_=HOFFARBIB, double default_avg_vel_=0.5); // default_avg_vel = [rad/s]
  ~interpolator();
  void push(const double *x_, const double *v_, const double *a_, bool immediate=true);
  double *front();
  // Getter function.
  //   1. Interpolate value if remain_t > 0 (time to goal is remaining).
  //   2. Get value.
  //   3. Pop value queue (q, dq, ddq) if popp = true.
  void get(double *x_, bool popp=true);
  void get(double *x_, double *v_, bool popp=true);
  void get(double *x_, double *v_, double *a_, bool popp=true);
  // Reset current value.
  void set(const double *x, const double *v=NULL);
  // Set goal and complete all interpolation.
  //   After calling of go(), value queue (q, dq, ddq) is full and remain_t = 0.
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
  // Set goal
  //   If online=true, user can get and interpolate value through get() function.
  void setGoal(const double *gx, const double *gv, double time,
               bool online=true);
  void setGoal(const double *gx, double time, bool online=true);
  // Interpolate value and push value to queue (q, dq, ddq).
  //   If remain_t <= 0, do nothing.
  void interpolate(double& remain_t_);
  double deltaT() const { return dt; }
  double dimension() const { return dim; }
  void setName (const std::string& _name) { name = _name; };
private:
  // Current interpolation mode
  interpolation_mode imode;
  // Queue of positions, velocities, and accelerations ([q_t, q_t+1, ...., q_t+n]).
  deque<double *> q, dq, ddq;
  // Length of queue.
  int length;
  // Dimension of interpolated vector (dim of x, v, a, ... etc)
  int dim;
  // Control time [s]
  double dt;
  // Current positions, velocities, and accelerations.
  double *x, *v, *a;
  // Current goal positions, velocities, and accelerations.
  double *gx, *gv, *ga;
  // target_t : time to goal [s] at setGoal
  // remain_t : time to goal [s] from current time. remain_t is [0, target_t].
  double target_t, remain_t;
  // Coefficients for interpolation polynomials.
  double *a0, *a1, *a2, *a3, *a4, *a5;
  // Default average velocity for calc_interpolation_time
  double default_avg_vel;
  // Interpolator name
  std::string name;

  void hoffarbib(double &remain_t_,
		 double a0, double a1, double a2,
		 double a3, double a4, double a5,
		 double &xx, double &vv, double &aa);
  void linear_interpolation(double &remain_t_,
			    double gx,
			    double &xx, double &vv, double &aa);
};

#endif
