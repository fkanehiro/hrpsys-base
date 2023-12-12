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
    seqplay(unsigned int i_dof, double i_dt, unsigned int i_fnum = 0, unsigned int optional_data_dim = 1);
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
    bool setJointAnglesOfGroup(const char *gname, const double* i_qRef, const size_t i_qsize, double i_tm=0.0);
    bool setJointVelocitiesOfGroup(const char *gname, const double* i_dqRef, const size_t i_dqsize, double i_tm=0.0);
    bool setJointTorquesOfGroup(const char *gname, const double* i_tqRef, const size_t i_tqsize, double i_tm=0.0);
    bool setJointCommonOfGroup(unsigned int type, unsigned int g_type, const char *gname, const double* i_qRef, const size_t i_qsize, double i_tm=0.0);
    void clearOfGroup(const char *gname, double i_timeLimit);
    bool playPatternOfGroup(const char *gname, std::vector<const double*> pos, std::vector<double> tm, const double *qInit, unsigned int len);

    bool resetJointGroup(const char *gname, const double *full);
    //
    bool setJointAnglesSequence(std::vector<const double*> pos, std::vector<double> tm);
    bool setJointAnglesSequenceOfGroup(const char *gname, std::vector<const double*> pos, std::vector<double> tm, const size_t pos_size);
    bool setJointVelocitiesSequenceOfGroup(const char *gname, std::vector<const double*> vel, std::vector<double> tm, const size_t vel_size);
    bool setJointTorquesSequenceOfGroup(const char *gname, std::vector<const double*> torque, std::vector<double> tm, const size_t torque_size);
    bool setJointCommonSequenceOfGroup(unsigned int type, unsigned int g_type, const char *gname, std::vector<const double*> pos, std::vector<double> tm, const size_t pos_size);
    bool setJointAnglesSequenceFull(std::vector<const double*> pos, std::vector<const double*> vel, std::vector<const double*> torques, std::vector<const double*> bpos, std::vector<const double*> brpy, std::vector<const double*> bacc, std::vector<const double*> zmps, std::vector<const double*> wrenches, std::vector<const double*> optionals, std::vector<double> tm);
    bool clearJointAngles();
    bool clearJointAnglesOfGroup(const char *gname);
    //
    void setJointAngle(unsigned int i_rank, double jv, double tm);
    void loadPattern(const char *i_basename, double i_tm);
    bool setJointVelocitiesSequence(std::vector<const double*> vel, std::vector<double> tm);
    bool setJointTorquesSequence(std::vector<const double*> torque, std::vector<double> tm);
    void clear(double i_timeLimit=0);
    void get(double *o_q, double *o_zmp, double *o_accel,
	     double *o_basePos, double *o_baseRpy, double *o_tq, double *o_wrenches, double *o_optional_data, double *o_dq);
    void go(const double *i_q, const double *i_zmp, const double *i_acc,
            const double *i_p, const double *i_rpy, const double *i_tq, const double *i_wrenches, const double *i_optional_data, const double *i_dq, double i_time, 
            bool immediate=true);
    void go(const double *i_q, const double *i_zmp, const double *i_acc,
            const double *i_p, const double *i_rpy, const double *i_tq, const double *i_wrenches, const double *i_optional_data, const double *i_dq,
	    const double *ii_q, const double *ii_zmp, const double *ii_acc,
            const double *ii_p, const double *ii_rpy, const double *ii_tq, const double *ii_wrenches, const double *ii_optional_data, const double *ii_dq,
            double i_time, bool immediate=true);
    void sync();
    bool setInterpolationMode(interpolator::interpolation_mode i_mode_);
private:
    class groupInterpolator{
    public:
        groupInterpolator(const std::vector<int>& i_indices, double i_dt)
            : indices(i_indices), state(created){
            inters[G_Q] = new interpolator(i_indices.size(), i_dt);
            inters[G_TQ] = new interpolator(i_indices.size(), i_dt);
            inters[G_DQ] = new interpolator(i_indices.size(), i_dt);
            xs.resize(G_NINTERPOLATOR, std::vector<double>(i_indices.size(),0));
            vs.resize(G_NINTERPOLATOR, std::vector<double>(i_indices.size(),0));
        }
        ~groupInterpolator(){
            delete inters[G_Q];
            delete inters[G_TQ];
            delete inters[G_DQ];
        }
        void interpolate(const double* full_q, const double* full_vq,
                         const double* full_dq, const double* full_vdq,
                         const double* full_tq, const double* full_vtq){
            if (state == created || state == removed) {
                set(G_Q, full_q, full_vq);
                set(G_DQ, full_dq, full_vdq);
                set(G_TQ, full_tq, full_vtq);
                return;
            }
            double x[indices.size()];
            double v[indices.size()];
            if (state == removing){
                for (size_t i=0; i<indices.size(); i++){
                    x[i] = full_q[indices[i]];
                    v[i] = full_vq ? full_vq[indices[i]] : 0;
                }
                inters[G_Q]->setGoal(x, v, time2remove);

                for (size_t i=0; i<indices.size(); i++){
                    x[i] = full_dq[indices[i]];
                    v[i] = full_vdq ? full_vdq[indices[i]] : 0;
                }
                inters[G_DQ]->setGoal(x, v, time2remove);

                for (size_t i=0; i<indices.size(); i++){
                    x[i] = full_tq[indices[i]];
                    v[i] = full_vtq ? full_vtq[indices[i]] : 0;
                }
                inters[G_TQ]->setGoal(x, v, time2remove);

                time2remove -= inters[G_Q]->deltaT();
                if (time2remove <= 0) state = removed;
            }
            inters[G_Q]->get(xs[G_Q].data(), vs[G_Q].data(), true);
            inters[G_DQ]->get(xs[G_DQ].data(), vs[G_DQ].data(), true);
            inters[G_TQ]->get(xs[G_TQ].data(), vs[G_TQ].data(), true);
        }
        void get(unsigned int type, double *full, double *dfull = NULL){
            if (state == created || state == removed) return;
            for (size_t i=0; i<indices.size(); i++){
                full[indices[i]] = xs[type][i];
                if (dfull) dfull[indices[i]] = vs[type][i];
            }
        }
        void set(unsigned int type, const double *full, const double *dfull=NULL){
            double x[indices.size()], v[indices.size()];
            for (size_t i=0; i<indices.size(); i++){
                x[i] = full[indices[i]];
                v[i] = dfull ? dfull[indices[i]] : 0;
            }
            inters[type]->set(x,v);
            inters[type]->clear();
        }
        void extract(double *dst, const double *src){
            for (size_t i=0; i<indices.size(); i++){
                dst[i] = src[indices[i]];
            }
        }
        bool isEmpty() {
            if(state == created) return true;
            if(state == removing || state == removed) return false;
            for(unsigned int i=0; i<G_NINTERPOLATOR; i++){
                if(!inters[i]->isEmpty()) return false;
            }
            return true;
        }
        void go(unsigned int type, const double *g, double tm){
            if(state == removing || state == removed) return;
            inters[type]->go(g, tm);
            state = working;
        }
        void go(unsigned int type, const double *g, const double *v, double tm){
            if(state == removing || state == removed) return;
            inters[type]->go(g, v, tm);
            state = working;
        }
        void setGoal(unsigned int type, const double *g, double tm){
            if(state == removing || state == removed) return;
            inters[type]->setGoal(g, tm);
            inters[type]->sync();
            state = working;
        }
        void setGoal(unsigned int type, const double *g, const double *v, double tm){
            if(state == removing || state == removed) return;
            inters[type]->setGoal(g, v, tm);
            inters[type]->sync();
            state = working;
        }
        void remove(double time){
            if(state == created) {
                state = removed;
                return;
            }
            if(state == working) {
                state = removing;
                time2remove = time;
                return;
            }
        }
        void clear() {
            if(state == removing) return;
            for(int i=0; i<G_NINTERPOLATOR; i++){
                inters[i]->clear();
            }
        }
        void clear(unsigned int type) {
            if(state == removing) return;
            inters[type]->clear();
        }

        bool setInterpolationMode (interpolator::interpolation_mode i_mode_) {
            bool ret = true;
            for(int i=0; i<G_NINTERPOLATOR; i++){
                ret &= inters[i]->setInterpolationMode(i_mode_);
            }
            return ret;
        }

        enum {G_Q, G_TQ, G_DQ, G_NINTERPOLATOR};
        interpolator *inters[G_NINTERPOLATOR];
        std::vector<int> indices;
        std::vector<std::vector<double> > xs, vs;
        typedef enum { created, working, removing, removed } gi_state;
        gi_state state;
        double time2remove;
    };
    void pop_back();
    enum {Q, ZMP, ACC, P, RPY, TQ, WRENCHES, OPTIONAL_DATA, DQ, NINTERPOLATOR};
    interpolator *interpolators[NINTERPOLATOR];
    std::map<std::string, groupInterpolator *> groupInterpolators; 
    int debug_level, m_dof;
};

#endif
