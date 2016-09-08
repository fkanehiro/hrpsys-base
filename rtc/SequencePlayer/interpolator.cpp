#include <fstream>
#include <iostream>
#include <cmath>
#include <cstring>
using namespace std;
#include "interpolator.h"
#include <coil/Guard.h>

interpolator::interpolator(int dim_, double dt_, interpolation_mode imode_, double default_avg_vel_)
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
  default_avg_vel = default_avg_vel_;
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
void interpolator::hoffarbib(double &remain_t_,
			     double a0, double a1, double a2, 
			     double a3, double a4, double a5, 
			     double &xx, double &vv, double &aa)
{
#define EPS 1e-6
  if (remain_t_ > dt+EPS){
    remain_t_ -= dt;
  }else{
    remain_t_ = 0;
  }
  double t = target_t - remain_t_;
  xx=a0+a1*t+a2*t*t+a3*t*t*t+a4*t*t*t*t+a5*t*t*t*t*t;
  vv=a1+2*a2*t+3*a3*t*t+4*a4*t*t*t+5*a5*t*t*t*t;
  aa=2*a2+6*a3*t+12*a4*t*t+20*a5*t*t*t;
}

void interpolator::linear_interpolation(double &remain_t_,
					double gx,
					double &xx, double &vv, double &aa)
{
  if (remain_t_ > dt+EPS){
    aa = 0;
    vv = (gx-xx)/remain_t_;
    xx += vv*dt;
    remain_t_ -= dt;
  }else{
    aa = vv = 0;
    xx = gx;
    remain_t_ = 0;
  }
}

void interpolator::sync()
{
  //cout << "sync:" << length << "," << q.size() << endl;
  length = q.size();
}

double interpolator::calc_interpolation_time(const double *newg)
{
  double remain_t_;
  double max_diff = 0, diff;
  for (int i=0; i<dim; i++){
    diff = fabs(newg[i]-gx[i]);
    if (diff > max_diff) max_diff = diff;
  }
  remain_t_ = max_diff/default_avg_vel;
#define MIN_INTERPOLATION_TIME	(1.0)
  if (remain_t_ < MIN_INTERPOLATION_TIME) {
      std::cerr << "[interpolator][" << name << "] MIN_INTERPOLATION_TIME violated!! Limit remain_t (" << remain_t << ") by MIN_INTERPOLATION_TIME (" << MIN_INTERPOLATION_TIME << ")."
                << "(max_diff = " << max_diff << ", default_avg_vel = " << default_avg_vel << ")" << std::endl;;
      remain_t_ = MIN_INTERPOLATION_TIME;
  }
  return remain_t_;
}

bool interpolator::setInterpolationMode (interpolation_mode i_mode_)
{
    if (i_mode_ != LINEAR && i_mode_ != HOFFARBIB &&
        i_mode_ != QUINTICSPLINE && i_mode_ != CUBICSPLINE) return false;
    imode = i_mode_;
    return true;
};

void interpolator::setGoal(const double *newg, double time,
                           bool online)
{
    setGoal(newg, NULL, time, online);
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
        switch(imode){
        case HOFFARBIB:
        A=(gx[i]-(x[i]+v[i]*target_t+(a[i]/2.0)*target_t*target_t))/(target_t*target_t*target_t);
        B=(gv[i]-(v[i]+a[i]*target_t))/(target_t*target_t);
        C=(ga[i]-a[i])/target_t;

        a0[i]=x[i];
        a1[i]=v[i];
        a2[i]=a[i]/2.0;
        a3[i]=10*A-4*B+0.5*C;
        a4[i]=(-15*A+7*B-C)/target_t;
        a5[i]=(6*A-3*B+0.5*C)/(target_t*target_t);
        break;
        case QUINTICSPLINE:
        a0[i]=x[i];
        a1[i]=v[i];
        a2[i]=0.5*a[i];
        a3[i]=(-20*x[i] + 20*gx[i] - 3*a[i]*target_t*target_t + ga[i]*target_t*target_t -
               12*v[i]*target_t - 8*gv[i]*target_t) / (2*target_t*target_t*target_t);
        a4[i]=(30*x[i] - 30*gx[i] + 3*a[i]*target_t*target_t - 2*ga[i]*target_t*target_t +
               16*v[i]*target_t + 14*gv[i]*target_t) / (2*target_t*target_t*target_t*target_t);
        a5[i]=(-12*x[i] + 12*gx[i] - a[i]*target_t*target_t + ga[i]*target_t*target_t -
               6*v[i]*target_t - 6*gv[i]*target_t) / (2*target_t*target_t*target_t*target_t*target_t);
        break;
        case CUBICSPLINE:
        a0[i]=x[i];
        a1[i]=v[i];
        a2[i]=(-3*x[i] + 3*gx[i] - 2*v[i]*target_t - gv[i]*target_t) / (target_t*target_t);
        a3[i]=( 2*x[i] - 2*gx[i] +   v[i]*target_t + gv[i]*target_t) / (target_t*target_t*target_t);
        a4[i]=a[5]=0;
        break;
        }
    }
    if (online) remain_t = time; // interpolation will start
}

void interpolator::interpolate(double& remain_t_)
{
    if (remain_t_ <= 0) return;

    double tm;
    for (int i=0; i<dim; i++){
        tm = remain_t_;
        switch(imode){
        case LINEAR:
            linear_interpolation(tm,
				 gx[i],
				 x[i], v[i], a[i]);
            break;
        case HOFFARBIB:
        case QUINTICSPLINE:
        case CUBICSPLINE:
            hoffarbib(tm,
		      a0[i], a1[i], a2[i], a3[i], a4[i], a5[i],
		      x[i], v[i], a[i]);
            break;
        }
    }
    push(x, v, a);
    remain_t_ = tm;
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
			bool immediate, size_t offset1, size_t offset2)
{
  ifstream strm(fname);
  if (!strm.is_open()) {
    cerr << "[interpolator " << name << "] file not found(" << fname << ")" << endl;
    return;
  }
  double *vs, ptime=-1,time, tmp;
  vs = new double[dim];
  strm >> time;
  while(strm.eof()==0){
    for (size_t i=0; i<offset1; i++){
      strm >> tmp;
    }
    for (int i=0; i<dim; i++){
      strm >> vs[i];
    }
    for (size_t i=0; i<offset2; i++){
      strm >> tmp;
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
			bool immediate, size_t offset1, size_t offset2)
{
  load(fname.c_str(), time_to_start, scale, immediate, offset1, offset2);
}

void interpolator::push(const double *x_, const double *v_, const double *a_, bool immediate)
{
  double *p = new double[dim];
  double *dp = new double[dim];
  double *ddp = new double[dim];
  memcpy(p, x_, sizeof(double)*dim);
  memcpy(dp, v_, sizeof(double)*dim);
  memcpy(ddp, a_, sizeof(double)*dim);
  q.push_back(p);
  dq.push_back(dp);
  ddq.push_back(ddp);
  if (immediate) sync();
}

void interpolator::pop()
{
  coil::Guard<coil::Mutex> lock(pop_mutex_);
  if (length > 0){
    length--;
    double *&vs = q.front();
    delete [] vs;
    q.pop_front();
    double *&dvs = dq.front();
    delete [] dvs;
    dq.pop_front();
    double *&ddvs = ddq.front();
    delete [] ddvs;
    ddq.pop_front();
  }
}

void interpolator::pop_back()
{
  coil::Guard<coil::Mutex> lock(pop_mutex_);
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
    double *&dvs = dq.back();
    delete [] dvs;
    dq.pop_back();
    if (length > 0){
      memcpy(v, dq.back(), sizeof(double)*dim);
    }else{
      memcpy(v, gv, sizeof(double)*dim);
    }
    double *&ddvs = ddq.back();
    delete [] ddvs;
    ddq.pop_back();
    if (length > 0){
      memcpy(a, ddq.back(), sizeof(double)*dim);
    }else{
      memcpy(a, ga, sizeof(double)*dim);
    }
  } else if (remain_t > 0) {
    remain_t = 0;
  }
}

void interpolator::set(const double *x_, const double *v_)
{
  for (int i=0; i<dim; i++){
    gx[i] = x[i] = x_[i];
    if (v_){
        gv[i] = v[i] = v_[i];
    }else{
        gv[i] = v[i] = 0;
    }
    ga[i] = a[i] = 0;
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

void interpolator::get(double *x_, bool popp)
{
  get(x_, NULL, NULL, popp);
}

void interpolator::get(double *x_, double *v_, bool popp)
{
  get(x_, v_, NULL, popp);
}

void interpolator::get(double *x_, double *v_, double *a_, bool popp)
{
  interpolate(remain_t);

  if (length!=0){
    double *&vs = q.front();
    if (vs == NULL) {
      cerr << "[interpolator " << name << "] interpolator::get vs = NULL, q.size() = " << q.size() 
	   << ", length = " << length << endl;
    }
    double *&dvs = dq.front();
    if (dvs == NULL) {
      cerr << "[interpolator " << name << "] interpolator::get dvs = NULL, dq.size() = " << dq.size() 
	   << ", length = " << length << endl;
    }
    double *&ddvs = ddq.front();
    if (ddvs == NULL) {
      cerr << "[interpolator " << name << "] interpolator::get ddvs = NULL, ddq.size() = " << ddq.size() 
	   << ", length = " << length << endl;
    }
    memcpy(x_, vs, sizeof(double)*dim);
    if ( v_ != NULL ) memcpy(v_, dvs, sizeof(double)*dim);
    if ( a_ != NULL ) memcpy(a_, ddvs, sizeof(double)*dim);
    if (popp) pop();
  }else{
    memcpy(x_, gx, sizeof(double)*dim);
    if ( v_ != NULL) memcpy(v_, gv, sizeof(double)*dim);
    if ( a_ != NULL) memcpy(a_, ga, sizeof(double)*dim);
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
