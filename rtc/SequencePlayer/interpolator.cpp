#include <fstream>
#include <iostream>
#include <cmath>
#include <cstring>
using namespace std;
#include "interpolator.h"

interpolator::interpolator(int dim_, double dt_, interpolation_mode imode_)
{
  imode = imode_;
  dim = dim_;
  dt = dt_;
  length = 0;
  gx = new double[dim];
  gv = new double[dim];
  ga = new double[dim];
  a0 = new double[dim];
  a1 = new double[dim];
  a2 = new double[dim];
  a3 = new double[dim];
  a4 = new double[dim];
  a5 = new double[dim];
  x = new double[dim];
  v = new double[dim];
  a = new double[dim];
  for (int i=0; i<dim; i++){
    gx[i] = gv[i] = ga[i] = x[i] = v[i] = a[i] = 0.0;
  }
  remain_t = 0;
  target_t = 0;
}

interpolator::~interpolator()
{
  clear();
  delete [] gx;
  delete [] gv;
  delete [] ga;
  delete [] a0;
  delete [] a1;
  delete [] a2;
  delete [] a3;
  delete [] a4;
  delete [] a5;
  delete [] x;
  delete [] v;
  delete [] a;
}

void interpolator::clear()
{
  while (!isEmpty()){
    pop();
  }
}

// 1dof interpolator
void interpolator::hoffarbib(double &remain_t,
			     double a0, double a1, double a2, 
			     double a3, double a4, double a5, 
			     double &xx, double &vv, double &aa)
{
#define EPS 1e-6
  if (remain_t > dt+EPS){
    remain_t -= dt;
  }else{
    remain_t = 0;
  }
  double t = target_t - remain_t;
  xx=a0+a1*t+a2*t*t+a3*t*t*t+a4*t*t*t*t+a5*t*t*t*t*t;
  vv=a1+2*a2*t+3*a3*t*t+4*a4*t*t*t+5*a5*t*t*t*t;
  aa=2*a2+6*a3*t+12*a4*t*t+20*a5*t*t*t;
}

void interpolator::linear_interpolation(double &remain_t,
					double gx,
					double &xx, double &vv, double &aa)
{
  if (remain_t > dt+EPS){
    aa = 0;
    vv = (gx-xx)/remain_t;
    xx += vv*dt;
    remain_t -= dt;
  }else{
    aa = vv = 0;
    xx = gx;
    remain_t = 0;
  }
}

void interpolator::sync()
{
  //cout << "sync:" << length << "," << q.size() << endl;
  length = q.size();
}

double interpolator::calc_interpolation_time(const double *newg,
					     double avg_vel)
{
  double remain_t;
  double max_diff = 0, diff;
  for (int i=0; i<dim; i++){
    diff = fabs(newg[i]-gx[i]);
    if (diff > max_diff) max_diff = diff;
  }
  remain_t = max_diff/avg_vel;
#define MIN_INTERPOLATION_TIME	(1.0)
  if (remain_t < MIN_INTERPOLATION_TIME) remain_t = MIN_INTERPOLATION_TIME;
  return remain_t;
}

bool interpolator::setInterpolationMode (interpolation_mode i_mode_)
{
    if (i_mode_ != LINEAR && i_mode_ != HOFFARBIB) return false;
    imode = i_mode_;
    return true;
};

void interpolator::setGoal(const double *newg, double time)
{
    setGoal(newg, NULL, time);
}

void interpolator::setGoal(const double *newg, const double *newv, double time,
                           bool online)
{
    memcpy(gx, newg, sizeof(double)*dim);
    if ( newv != NULL ) memcpy(gv, newv, sizeof(double)*dim);
    else { for(int i = 0; i < dim; i++) { gv[i] = 0; } }
    target_t = time;

    double A,B,C;
    for (int i=0; i<dim; i++){
        A=(gx[i]-(x[i]+v[i]*target_t+(a[i]/2.0)*target_t*target_t))/(target_t*target_t*target_t);
        B=(gv[i]-(v[i]+a[i]*target_t))/(target_t*target_t);
        C=(ga[i]-a[i])/target_t;

        a0[i]=x[i];
        a1[i]=v[i];
        a2[i]=a[i]/2.0;
        a3[i]=10*A-4*B+0.5*C;
        a4[i]=(-15*A+7*B-C)/target_t;
        a5[i]=(6*A-3*B+0.5*C)/(target_t*target_t);
    }
    if (online) remain_t = time; // interpolation will start
}

void interpolator::interpolate(double& remain_t)
{
    if (remain_t <= 0) return;

    double tm;
    for (int i=0; i<dim; i++){
        tm = remain_t;
        switch(imode){
        case LINEAR:
            linear_interpolation(tm,
				 gx[i],
				 x[i], v[i], a[i]);
            break;
        case HOFFARBIB:
            hoffarbib(tm,
		      a0[i], a1[i], a2[i], a3[i], a4[i], a5[i],
		      x[i], v[i], a[i]);
            break;
        }
    }
    push(x);
    remain_t = tm;
}

void interpolator::go(const double *newg, double time, bool immediate)
{
  go(newg, NULL, time, immediate);
}

void interpolator::go(const double *newg, const double *newv, double time, bool immediate)
{
  if (time == 0) time = calc_interpolation_time(newg);
  setGoal(newg, newv, time, false);
  
  do{
      interpolate(time);
  }while(time>0);
  if (immediate) sync();
}

void interpolator::load(const char *fname, double time_to_start, double scale,
			bool immediate)
{
  ifstream strm(fname);
  if (!strm.is_open()) {
    cerr << "file not found(" << fname << ")" << endl;
    return;
  }
  double *vs, ptime=-1,time;
  vs = new double[dim];
  strm >> time;
  while(strm.eof()==0){
    for (int i=0; i<dim; i++){
      strm >> vs[i];
    }
    if (ptime <0){
      go(vs, time_to_start, false);
    }else{
      go(vs, scale*(time-ptime), false);
    }
    ptime = time;
    strm >> time;
  }
  strm.close();
  delete [] vs;
  if (immediate) sync();
}

void interpolator::load(string fname, double time_to_start, double scale,
			bool immediate)
{
  load(fname.c_str(), time_to_start, scale, immediate);
}

void interpolator::push(const double *a, bool immediate)
{
  double *p = new double[dim];
  memcpy(p, a, sizeof(double)*dim);
  q.push_back(p);
  if (immediate) sync();
}

void interpolator::pop()
{
  if (length > 0){
    length--;
    double *&vs = q.front();
    delete [] vs;
    q.pop_front();
  }
}

void interpolator::pop_back()
{
  if (length > 0){
    length--;
    double *&vs = q.back();
    delete [] vs;
    q.pop_back();
    if (length > 0){
      memcpy(x, q.back(), sizeof(double)*dim);
    }else{
      memcpy(x, gx, sizeof(double)*dim);
    }
  }

  if (remain_t > 0)
    remain_t = 0;
}

void interpolator::set(const double *angle)
{
  for (int i=0; i<dim; i++){
    gx[i] = x[i] = angle[i];
    gv[i] = ga[i] = v[i] = a[i] = 0;
  }
}

double *interpolator::front()
{
  if (length!=0){
    return q.front();
  }else{
    return gx;
  }
}

void interpolator::get(double *a, bool popp)
{
  interpolate(remain_t);

  if (length!=0){
    double *&vs = q.front();
    if (vs == NULL) {
      cerr << "interpolator::get vs = NULL, q.size() = " << q.size() 
	   << ", length = " << length << endl;
    }
    memcpy(a, vs, sizeof(double)*dim);
    if (popp) pop();
  }else{
    memcpy(a, gx, sizeof(double)*dim);
  }
}

bool interpolator::isEmpty()
{
    return length==0 && remain_t <= 0;
}

double interpolator::remain_time()
{
  return dt*length;
}
