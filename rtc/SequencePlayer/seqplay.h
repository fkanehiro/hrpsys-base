#ifndef __SEQPLAY_H__
#define __SEQPLAY_H__

#include <fstream>
#include <hrpUtil/uBlasCommonTypes.h>
#include "interpolator.h"
#include "timeUtil.h"

using namespace hrp;

class seqplay
{
public:
    seqplay(unsigned int i_dof, double i_dt);
    ~seqplay();
    //
    bool isEmpty() const;
    //
    void setJointAngles(const double *i_qRef, double i_tm=0.0);
    void setZmp(const double *i_zmp, double i_tm=0.0);
    void setBasePos(const double *i_pos, double i_tm=0.0);
    void setBaseRpy(const double *i_rpy, double i_tm=0.0);
    void setBaseAcc(const double *i_acc, double i_tm=0.0);
    //
    void setJointAngle(unsigned int i_rank, double jv, double tm);
    void loadPattern(const char *i_basename, double i_tm);
    void clear(double i_timeLimit=0);
    void get(double *o_q, double *o_zmp, double *o_accel,
	     double *o_basePos, double *o_baseRpy);
    void go(const double *i_q, const double *i_zmp, const double *i_acc,
            const double *i_p, const double *i_rpy, double i_time, 
            bool immediate=true);
    void push(const double *i_q, const double *i_zmp, const double *i_acc,
              const double *i_p, const double *i_rpy, bool immediate=true);
    void sync();
private:
    void pop_back();
    enum {Q, ZMP, ACC, P, RPY, NINTERPOLATOR};
    interpolator *interpolators[NINTERPOLATOR];
    int debug_level, m_dof;
};

#endif
