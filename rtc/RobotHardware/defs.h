#ifndef __DEFS_H__
#define __DEFS_H__

#include <math.h>

enum {RLEG, LLEG};
enum {FX, FY, FZ, MX, MY, MZ};
enum {X, Y, Z};
enum {WX, WY, WZ};
enum {TOE, HEEL};
enum {INSIDE, OUTSIDE};
enum {RL, FB};

#define sgn(x)	(((x)>0)?1:-1)
#define deg2rad(x)	((x)*M_PI/180)
#define rad2deg(x)	((x)*180/M_PI)
#define sqr(x)		((x)*(x))
#define LPF(dT, omega, x, y)	((y) = (((dT)*(omega)/(1+(dT)*(omega)))*(x)+1/(1+(dT)*(omega))*(y)))

template<class T>
void LIMITER(T &org, const T &min_, const T &max_)
{
    for (unsigned int i=0; i<org.size(); i++){
        if (org(i) > max_(i)){
            org(i) = max_(i);
        }else if (org(i) < min_(i)){
            org(i) = min_(i);
        }
    }
}

template<class T>
void DEADZONE(T &org, const T &min_, const T &max_)
{
    for (unsigned int i=0; i<org.size(); i++){
        if (org(i) > max_(i)){
            org(i) -= max_(i);
        }else if (org(i) < min_(i)){
            org(i) -= min_(i);
        }else{
            org(i) = 0;
        }
    }
}

inline void LIMIT(double &org, const double max_)
{
    if (org > max_){
        org = max_;
    }else if (org < -max_){
        org = -max_;
    }
}

inline void LIMITER(double &org, const double min_, const double max_)
{
    if (org > max_){
        org = max_;
    }else if (org < min_){
        org = min_;
    }
}

inline void LIMITER(int &org, const int min_, const int max_)
{
    if (org > max_){
        org = max_;
    }else if (org < min_){
        org = min_;
    }
}

inline void DEADZONE(double &org, const double min_, const double max_)
{
    if (org > max_){
        org -= max_;
    }else if (org < min_){
        org -= min_;
    }else{
        org = 0;
    }
}

inline void HYSTERESIS(double &newv, const double old, 
		       const double min_, const double max_)
{
    if (old >= newv-min_){
        newv -= min_;
    }else if (old <= newv - max_){
        newv -= max_;
    }else{
        newv = old;
    }
}

#endif
