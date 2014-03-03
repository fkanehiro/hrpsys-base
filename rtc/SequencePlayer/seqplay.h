#ifndef __SEQPLAY_H__
#define __SEQPLAY_H__

#include <fstream>
#include <vector>
#include <map>
#include <hrpUtil/EigenTypes.h>
#include "interpolator.h"
#include "timeUtil.h"

using namespace hrp;

class seqplay
{
public:
    seqplay(unsigned int i_dof, double i_dt, unsigned int i_fnum = 0);
    ~seqplay();
    //
    bool isEmpty() const;
    bool isEmpty(const char *gname);
    //
    void setJointAngles(const double *i_qRef, double i_tm=0.0);
    void getJointAngles(double *i_qRef);
    void setZmp(const double *i_zmp, double i_tm=0.0);
    void setBasePos(const double *i_pos, double i_tm=0.0);
    void setBaseRpy(const double *i_rpy, double i_tm=0.0);
    void setBaseAcc(const double *i_acc, double i_tm=0.0);
    void setWrenches(const double *i_wrenches, double i_tm=0.0);
    void playPattern(std::vector<const double*> pos, std::vector<const double*> zmp, std::vector<const double*> rpy, std::vector<double> tm, const double *qInit, unsigned int len);
    //
    bool addJointGroup(const char *gname, const std::vector<int>& indices);
    bool getJointGroup(const char *gname, std::vector<int>& indices);
    bool removeJointGroup(const char *gname, double time=2.5);
    bool setJointAnglesOfGroup(const char *gname, const double *i_qRef, double i_tm=0.0);
    void clearOfGroup(const char *gname, double i_timeLimit);
    bool playPatternOfGroup(const char *gname, std::vector<const double*> pos, std::vector<double> tm, const double *qInit, unsigned int len);

    bool resetJointGroup(const char *gname, const double *full);
    //
    void setJointAngle(unsigned int i_rank, double jv, double tm);
    void loadPattern(const char *i_basename, double i_tm);
    void clear(double i_timeLimit=0);
    void get(double *o_q, double *o_zmp, double *o_accel,
	     double *o_basePos, double *o_baseRpy, double *o_tq, double *o_wrenches);
    void go(const double *i_q, const double *i_zmp, const double *i_acc,
            const double *i_p, const double *i_rpy, const double *i_tq, const double *i_wrenches, double i_time, 
            bool immediate=true);
    void go(const double *i_q, const double *i_zmp, const double *i_acc,
            const double *i_p, const double *i_rpy, const double *i_tq, const double *i_wrenches,
	    const double *ii_q, const double *ii_zmp, const double *ii_acc,
            const double *ii_p, const double *ii_rpy, const double *ii_tq, const double *ii_wrenches,
            double i_time, bool immediate=true);
    void sync();
    bool setInterpolationMode(interpolator::interpolation_mode i_mode_);
private:
    class groupInterpolator{
    public:
        groupInterpolator(const std::vector<int>& i_indices, double i_dt)
            : indices(i_indices), state(created){
            inter = new interpolator(i_indices.size(), i_dt);
        }
        ~groupInterpolator(){
            delete inter;
        }
        void get(double *full, double *dfull = NULL, bool popp=true){
            if (state == created) return;
            if (state == removing){
                double x[indices.size()];
                double v[indices.size()];
                for (size_t i=0; i<indices.size(); i++){
                    x[i] = full[indices[i]];
                    v[i] = dfull ? dfull[indices[i]] : 0;
                }
                inter->setGoal(x, v, time2remove);
                time2remove -= inter->deltaT();
                if (time2remove <= 0) state = removed;
            }
            double x[indices.size()], v[indices.size()];
            inter->get(x, v, popp);
            for (size_t i=0; i<indices.size(); i++){
                full[indices[i]] = x[i];
                if (dfull) dfull[indices[i]] = v[i];
            }
        }
        void set(const double *full, const double *dfull=NULL){
            double x[indices.size()], v[indices.size()];
            for (size_t i=0; i<indices.size(); i++){
                x[i] = full[indices[i]];
                v[i] = dfull ? dfull[indices[i]] : 0;
                //std::cout << v[i] << " ";
            }
            //std::cout << std::endl;
            inter->set(x,v);
        }
        void extract(double *dst, const double *src){
            for (size_t i=0; i<indices.size(); i++){
                dst[i] = src[indices[i]];
            }
        }
        bool isEmpty() { return inter->isEmpty() && state != removing; } 
        void go(const double *g, double tm){
            inter->go(g, tm);
            state = working;
        }
        void go(const double *g, const double *v, double tm){
            inter->go(g, v, tm);
            state = working;
        }
        void setGoal(const double *g, double tm){
            inter->setGoal(g, tm);
            inter->sync();
            state = working;
        }
        void setGoal(const double *g, const double *v, double tm){
            inter->setGoal(g, v, tm);
            inter->sync();
            state = working;
        }
        void remove(double time){
            state = removing;
            time2remove = time;
        }
        void clear(double i_timeLimit=0) {
            tick_t t1 = get_tick();
            while (!isEmpty()){
		if (i_timeLimit > 0 
			&& tick2sec(get_tick()-t1)>=i_timeLimit) break;
		inter->pop_back();
            }
        }

        interpolator *inter;
        std::vector<int> indices;
        typedef enum { created, working, removing, removed } gi_state;
        gi_state state;
        double time2remove;
    };
    void pop_back();
    enum {Q, ZMP, ACC, P, RPY, TQ, WRENCHES, NINTERPOLATOR};
    interpolator *interpolators[NINTERPOLATOR];
    std::map<std::string, groupInterpolator *> groupInterpolators; 
    int debug_level, m_dof;
};

#endif
