// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-

#include <iostream>
#include <unistd.h>
#include "seqplay.h"

#define deg2rad(x)	((x)*M_PI/180)

seqplay::seqplay(unsigned int i_dof, double i_dt) : m_dof(i_dof)
{
    interpolators[Q] = new interpolator(i_dof, i_dt);
    interpolators[ZMP] = new interpolator(3, i_dt);
    interpolators[ACC] = new interpolator(3, i_dt);
    interpolators[P] = new interpolator(3, i_dt);
    interpolators[RPY] = new interpolator(3, i_dt);
    //

#ifdef WAIST_HEIGHT
    double initial_zmp[3] = {0,0,-WAIST_HEIGHT};
    double initial_waist[3] = {0,0,WAIST_HEIGHT};
    interpolators[P]->set(initial_waist);
#elif defined(INITIAL_ZMP_REF_X)
    double initial_zmp[3] = {INITIAL_ZMP_REF_X, 0, INITIAL_ZMP_REF_Z};
#else
    double initial_zmp[] = {0,0,0};
#endif
    interpolators[ZMP]->set(initial_zmp);
}

seqplay::~seqplay()
{
	for (unsigned int i=0; i<NINTERPOLATOR; i++){
		delete interpolators[i];
	}
}

#if 0 // TODO
void seqplay::goHalfSitting(double tm)
{
    if (tm == 0){
        tm = (double)angle_interpolator->calc_interpolation_time(get_half_sitting_posture());
    }
    angle_interpolator->setGoal(get_half_sitting_posture(), tm);
#ifdef INITIAL_ZMP_REF_X
    double zmp[]={INITIAL_ZMP_REF_X, 0, INITIAL_ZMP_REF_Z};
#else
    double zmp[] = {0,0,0};
#endif
    zmp_interpolator->setGoal(zmp, tm);
#ifdef INITIAL_ZMP_REF_Z
    double waist_pos[]={ref_state.basePosAtt.px, 
                        ref_state.basePosAtt.py,
                        -INITIAL_ZMP_REF_Z};
#else
    double waist_pos[] = {0,0,0};
#endif
    waist_pos_interpolator->setGoal(waist_pos, tm);
    double p[3], rpy[3];
    OpenHRP::convTransformToPosRpy(ref_state.basePosAtt, p, rpy);
    double initial_rpy[3]={0,0,rpy[2]};
    waist_rpy_interpolator->setGoal(initial_rpy, tm);
}

void seqplay::goInitial(double tm)
{
    if (tm == 0){
        tm = (double)angle_interpolator->calc_interpolation_time(get_initial_posture());
    }
    angle_interpolator->setGoal(get_initial_posture(), tm);
#ifdef WAIST_HEIGHT
    double zmp[] = {0,0,-WAIST_HEIGHT};
    double pos[] = {ref_state.basePosAtt.px,
                    ref_state.basePosAtt.py,
                    WAIST_HEIGHT};
    waist_pos_interpolator->setGoal(pos, tm);
#elif defined(INITIAL_ZMP_REF_Z)
    double zmp[] = {0,0, INITIAL_ZMP_REF_Z};
#else
    double zmp[] = {0,0,0};
#endif
    zmp_interpolator->setGoal(zmp, tm);
}
#endif

bool seqplay::isEmpty() const
{
	for (unsigned int i=0; i<NINTERPOLATOR; i++){
		if (!interpolators[i]->isEmpty()) return false;
	}
	std::map<std::string, groupInterpolator *>::const_iterator it;
	for (it=groupInterpolators.begin(); it!=groupInterpolators.end(); it++){
		groupInterpolator *gi = it->second;
		if (gi && !gi->isEmpty()) return false;
	}

	return true;
}

bool seqplay::isEmpty(const char *gname)
{
	char *s = (char *)gname; while(*s) {*s=toupper(*s);s++;}
	groupInterpolator *i = groupInterpolators[gname];
	if (!i) return true;
	return i->isEmpty();
}

#if 0
void seqplay::setReferenceState(const ::CharacterState& ref, double tm)
{
    if (tm == 0){
        tm = (double)angle_interpolator->calc_interpolation_time(ref.angle);
    }
    if (ref.angle.length()>0) angle_interpolator->setGoal(ref.angle, tm, false);
    if (ref.velocity.length()>0)
        velocity_interpolator->setGoal(ref.velocity, tm, false);
    if (ref.zmp.length()>0) zmp_interpolator->setGoal(ref.zmp, tm, false);
    if (ref.accel.length()>0 && ref.accel[0].length() == 3)
        acc_interpolator->setGoal(ref.accel[0], tm, false);
    double p[3], rpy[3];
    OpenHRP::convTransformToPosRpy(ref.basePosAtt, p, rpy);
    waist_pos_interpolator->setGoal(p, tm, false);
    waist_rpy_interpolator->setGoal(rpy, tm, false);

    sync();
}

void seqplay::getReferenceState(::CharacterState_out ref)
{
    ref = new CharacterState;
    *ref = ref_state;
}
#endif


void seqplay::setJointAngles(const double *jvs, double tm)
{
	if (tm == 0){
		interpolators[Q]->set(jvs);
	}else{
		interpolators[Q]->setGoal(jvs, tm);
	}
}

void seqplay::getJointAngles(double *jvs)
{
    interpolators[Q]->get(jvs, false);
}


void seqplay::setZmp(const double *i_zmp, double i_tm)
{
	if (i_tm == 0){
		interpolators[ZMP]->set(i_zmp);
	}else{
		interpolators[ZMP]->setGoal(i_zmp, i_tm);
	}
}

void seqplay::setBasePos(const double *i_pos, double i_tm)
{
	if (i_tm == 0){
		interpolators[P]->set(i_pos);
	}else{
		interpolators[P]->setGoal(i_pos, i_tm);
	}
}

void seqplay::setBaseRpy(const double *i_rpy, double i_tm)
{
	if (i_tm == 0){
		interpolators[RPY]->set(i_rpy);
	}else{
		interpolators[RPY]->setGoal(i_rpy, i_tm);
	}
}

void seqplay::setBaseAcc(const double *i_acc, double i_tm)
{
	if (i_tm == 0){
		interpolators[ACC]->set(i_acc);
	}else{
		interpolators[ACC]->setGoal(i_acc, i_tm);
	}
}

void seqplay::setJointAngle(unsigned int i_rank, double jv, double tm)
{
    double pos[m_dof];
	getJointAngles(pos);
    pos[i_rank] = jv;
    interpolators[Q]->setGoal(pos, tm);
}

void seqplay::playPattern(std::vector<const double*> pos, std::vector<const double*> zmp, std::vector<const double*> rpy, std::vector<double> tm, const double *qInit, unsigned int len)
{
    const double *q=NULL, *z=NULL, *a=NULL, *p=NULL, *e=NULL; double t=0;
    double *v = new double[len];
    for (unsigned int i=0; i<pos.size(); i++){
        q = pos[i];
	if (i < pos.size() - 1 ) {
	  double t0, t1;
	  if (tm.size() == pos.size()) {
	    t0 = tm[i]; t1 = tm[i+1];
	  } else {
	    t0 = t1 = tm[0];
	  }
          const double *q_next = pos[i+1];
          const double *q_prev 
              = i==0 ? qInit : pos[i-1];
	  for (unsigned int j = 0; j < len; j++) {
	    double d0, d1, v0, v1;
	    d0 = (q[j] - q_prev[j]);
	    d1 = (q_next[j] - q[j]);
	    v0 = d0/t0;
	    v1 = d1/t1;
	    if ( v0 * v1 >= 0 ) {
	      v[j] = 0.5 * (v0 + v1);
	    } else {
	      v[j] = 0;
	    }
	  }
	} else {
	  for (unsigned int j = 0; j < len; j++) { v[j] = 0.0; }
	}
        if (i < zmp.size()) z = zmp[i];
        if (i < rpy.size()) e = rpy[i];
        if (i < tm.size()) t = tm[i];
        go(q, z, a, p, e,
		  v, NULL, NULL, NULL, NULL,
		  t, false);
    }
    sync();
    delete [] v;
}

void seqplay::clear(double i_timeLimit)
{
	tick_t t1 = get_tick();
	while (!isEmpty()){
		if (i_timeLimit > 0 
			&& tick2sec(get_tick()-t1)>=i_timeLimit) break;
		pop_back();
	}
}

void seqplay::loadPattern(const char *basename, double tm)
{
    double scale = 1.0;
    bool found = false;
    if (debug_level > 0) cout << "pos   = ";
    string pos = basename; pos.append(".pos");
    if (access(pos.c_str(),0)==0){
        found = true;
        interpolators[Q]->load(pos, tm, scale, false);
        if (debug_level > 0) cout << pos;
    }
    if (debug_level > 0) cout << endl << "zmp   = ";
    string zmp = basename; zmp.append(".zmp");
    if (access(zmp.c_str(),0)==0){
        found = true;
        interpolators[ZMP]->load(zmp, tm, scale, false);
        if (debug_level > 0) cout << zmp;
    }
    if (debug_level > 0) cout << endl << "gsens = ";
    string acc = basename; acc.append(".gsens");
    if (access(acc.c_str(),0)==0){
        found = true;
        interpolators[ACC]->load(acc, tm, scale, false);
        if (debug_level > 0) cout << acc;
    }
    if (debug_level > 0) cout << endl << "hip   = ";
    string hip = basename; hip.append(".hip");
    if (access(hip.c_str(),0)==0){
        found = true;
        interpolators[P]->load(hip, tm, scale, false, 0, 3);
        interpolators[RPY]->load(hip, tm, scale, false, 3, 0);
        if (debug_level > 0) cout << hip;
    }else{
        hip = basename; hip.append(".waist");
        if (access(hip.c_str(),0)==0){
            found = true;
            interpolators[P]->load(hip, tm, scale, false, 0, 3);
            interpolators[RPY]->load(hip, tm, scale, false, 3, 0);
            if (debug_level > 0) cout << hip;
        }
    }
    if (debug_level > 0) cout << endl;
    if (!found) cerr << "pattern not found(" << basename << ")" << endl;
    //
    sync();
}

void seqplay::sync()
{
	for (unsigned int i=0; i<NINTERPOLATOR; i++){
		interpolators[i]->sync();
	}
}

void seqplay::pop_back()
{
	for (unsigned int i=0; i<NINTERPOLATOR; i++){
		interpolators[i]->pop_back();
	}
}

void seqplay::get(double *o_q, double *o_zmp, double *o_accel,
				  double *o_basePos, double *o_baseRpy)
{
	double v[m_dof];
	interpolators[Q]->get(o_q, v);
	std::map<std::string, groupInterpolator *>::iterator it;
	for (it=groupInterpolators.begin(); it!=groupInterpolators.end();){
		groupInterpolator *gi = it->second;
		if (gi){
			gi->get(o_q, v);
			if (gi->state == groupInterpolator::removed){
				groupInterpolators.erase(it++);
				delete gi;
				continue;
			}
		}
		++it;
	}
	interpolators[ZMP]->get(o_zmp);
	interpolators[ACC]->get(o_accel);
	interpolators[P]->get(o_basePos);
	interpolators[RPY]->get(o_baseRpy);
}

void seqplay::go(const double *i_q, const double *i_zmp, const double *i_acc,
				 const double *i_p, const double *i_rpy, double i_time, 
				 bool immediate)
{
	go(i_q, i_zmp, i_acc, i_p, i_rpy,
	   NULL, NULL, NULL, NULL, NULL,
	   i_time, immediate);
}

void seqplay::go(const double *i_q, const double *i_zmp, const double *i_acc,
				 const double *i_p, const double *i_rpy,
				 const double *ii_q, const double *ii_zmp, const double *ii_acc,
				 const double *ii_p, const double *ii_rpy,
				 double i_time,	 bool immediate)
{
	if (i_q) interpolators[Q]->go(i_q, ii_q, i_time, false);
	if (i_zmp) interpolators[ZMP]->go(i_zmp, ii_zmp, i_time, false);
	if (i_acc) interpolators[ACC]->go(i_acc, ii_acc, i_time, false);
	if (i_p) interpolators[P]->go(i_p, ii_p, i_time, false);
	if (i_rpy) interpolators[RPY]->go(i_rpy, ii_rpy, i_time, false);
	if (immediate) sync();
}

bool seqplay::setInterpolationMode (interpolator::interpolation_mode i_mode_)
{
    if (i_mode_ != interpolator::LINEAR && i_mode_ != interpolator::HOFFARBIB &&
		i_mode_ != interpolator::QUINTICSPLINE && i_mode_ != interpolator::CUBICSPLINE) return false;

	bool ret=true; 
	for (unsigned int i=0; i<NINTERPOLATOR; i++){
		ret &= interpolators[i]->setInterpolationMode(i_mode_);
	}
	std::map<std::string, groupInterpolator *>::const_iterator it;
	for (it=groupInterpolators.begin(); it!=groupInterpolators.end(); it++){
		groupInterpolator *gi = it->second;
		ret &= gi->inter->setInterpolationMode(i_mode_);
	}
	return ret;
}

bool seqplay::addJointGroup(const char *gname, const std::vector<int>& indices)
{
	char *s = (char *)gname; while(*s) {*s=toupper(*s);s++;}
	groupInterpolator *i = groupInterpolators[gname];
	if (i) {
		std::cerr << "[addJointGroup] group name " << gname << " is already installed" << std::endl;
		return false;
	}
	i = new groupInterpolator(indices, interpolators[Q]->deltaT());
	groupInterpolators[gname] = i;
	return true;
}

bool seqplay::getJointGroup(const char *gname, std::vector<int>& indices)
{
	char *s = (char *)gname; while(*s) {*s=toupper(*s);s++;}
	groupInterpolator *i = groupInterpolators[gname];
	if (i) {
		for(unsigned j = 0; j < i->indices.size(); j++) {
			indices.push_back(i->indices[j]);
		}
		return true;
	}else{
		std::cerr << "[getJointGroup] group name " << gname << " is not installed" << std::endl;
		return false;
	}
}

bool seqplay::removeJointGroup(const char *gname, double time)
{
	char *s = (char *)gname; while(*s) {*s=toupper(*s);s++;}
	groupInterpolator *i = groupInterpolators[gname];
	if (i){
		i->remove(time);
		return true;
	}else{
		std::cerr << "[removeJointGroup] group name " << gname << " is not installed" << std::endl;
		return false;
	}
}

bool seqplay::resetJointGroup(const char *gname, const double *full)
{
	char *s = (char *)gname; while(*s) {*s=toupper(*s);s++;}
	groupInterpolator *i = groupInterpolators[gname];
	if (i){
		i->set(full);
		return true;
	}else{
		std::cerr << "[resetJointGroup] group name " << gname << " is not installed" << std::endl;
		return false;
	}
}

bool seqplay::setJointAnglesOfGroup(const char *gname, const double *i_qRef, double i_tm)
{
	char *s = (char *)gname; while(*s) {*s=toupper(*s);s++;}
	groupInterpolator *i = groupInterpolators[gname];
	if (i){
		if (i->state == groupInterpolator::created){
			double q[m_dof], dq[m_dof];
			interpolators[Q]->get(q, dq, false);
			std::map<std::string, groupInterpolator *>::iterator it;
			for (it=groupInterpolators.begin(); it!=groupInterpolators.end(); it++){
				groupInterpolator *gi = it->second;
				if (gi)	gi->get(q, dq, false);
			}
			double x[i->indices.size()], v[i->indices.size()];
			i->extract(x, q);
			i->extract(v, dq);
			i->inter->setGoal(x,v,interpolators[Q]->deltaT());
		}
		i->setGoal(i_qRef, i_tm);
		return true;
	}else{
		std::cerr << "[setJointAnglesOfGroup] group name " << gname << " is not installed" << std::endl;
		return false;
	}
}

bool seqplay::playPatternOfGroup(const char *gname, std::vector<const double *> pos, std::vector<double> tm, const double *qInit, unsigned int len)
{
	char *s = (char *)gname; while(*s) {*s=toupper(*s);s++;}
	groupInterpolator *i = groupInterpolators[gname];
	if (i){
		if (len != i->indices.size() ) {
			std::cerr << "[playPatternOfGroup] group name " << gname << " : size of manipulater is not equal to input. " << len << " /= " << i->indices.size() << std::endl;
			return false;
		}
		if (i->state == groupInterpolator::created){
			double q[m_dof], dq[m_dof];
			interpolators[Q]->get(q, dq, false);
			std::map<std::string, groupInterpolator *>::iterator it;
			for (it=groupInterpolators.begin(); it!=groupInterpolators.end(); it++){
				groupInterpolator *gi = it->second;
				if (gi)	gi->get(q, dq, false);
			}
			double x[i->indices.size()], v[i->indices.size()];
			i->extract(x, q);
			i->extract(v, dq);
			i->inter->go(x,v,interpolators[Q]->deltaT());
		}
		const double *q=NULL; double t=0;
		double *v = new double[len];
		double *qi = new double[len];
		for (unsigned int j=0; j<len; j++){
			qi[j] = qInit[i->indices[j]];
		}
		for (unsigned int l=0; l<pos.size(); l++){
			q = pos[l];
			if (l < pos.size() - 1 ) {
				double t0, t1;
				if (tm.size() == pos.size()) {
					t0 = tm[l]; t1 = tm[l+1];
				} else {
					t0 = t1 = tm[0];
				}
				const double *q_next = pos[l+1];
				const double *q_prev = l==0 ? qi : pos[l-1];
				for (unsigned int j = 0; j < len; j++) {
					double d0, d1, v0, v1;
					d0 = (q[j] - q_prev[j]);
					d1 = (q_next[j] - q[j]);
					v0 = d0/t0;
					v1 = d1/t1;
					if ( v0 * v1 >= 0 ) {
						v[j] = 0.5 * (v0 + v1);
					} else {
						v[j] = 0;
					}
				}
			} else {
				for (unsigned int j = 0; j < len; j++) { v[j] = 0.0; }
			}
			if (l < tm.size()) t = tm[l];
			i->go(q, v, t);
		}
		sync();
		delete [] v;
		delete [] qi;

		return true;
	}else{
		std::cerr << "[playPatternOfGroup] group name " << gname << " is not installed" << std::endl;
		return false;
	}
}
