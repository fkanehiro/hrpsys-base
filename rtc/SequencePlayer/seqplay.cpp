// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-

#include <iostream>
#include <unistd.h>
#include "seqplay.h"

#define deg2rad(x)	((x)*M_PI/180)

seqplay::seqplay(unsigned int i_dof, double i_dt, unsigned int i_fnum, unsigned int optional_data_dim) : m_dof(i_dof)
{
    interpolators[Q] = new interpolator(i_dof, i_dt);
    interpolators[ZMP] = new interpolator(3, i_dt);
    interpolators[ACC] = new interpolator(3, i_dt);
    interpolators[P] = new interpolator(3, i_dt);
    interpolators[RPY] = new interpolator(3, i_dt);
    interpolators[TQ] = new interpolator(i_dof, i_dt);
    interpolators[WRENCHES] = new interpolator(6 * i_fnum, i_dt, interpolator::HOFFARBIB, 100); // wrenches = 6 * [number of force sensors]
	interpolators[OPTIONAL_DATA] = new interpolator(optional_data_dim, i_dt);
    // Set interpolator name
    interpolators[Q]->setName("Q");
    interpolators[ZMP]->setName("ZMP");
    interpolators[ACC]->setName("ACC");
    interpolators[P]->setName("P");
    interpolators[RPY]->setName("RPY");
    interpolators[TQ]->setName("TQ");
    interpolators[WRENCHES]->setName("WRENCHES");
    interpolators[OPTIONAL_DATA]->setName("OPTIONAL_DATA");
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
    double initial_wrenches[6 * i_fnum];
    for (size_t i = 0; i < 6 * i_fnum; i++) initial_wrenches[i] = 0;
    interpolators[WRENCHES]->set(initial_wrenches);
	double initial_optional_data[optional_data_dim];
	for (size_t i = 0; i < optional_data_dim; i++) initial_optional_data[i] = 0;
	interpolators[OPTIONAL_DATA]->set(initial_optional_data);
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

void seqplay::setWrenches(const double *i_wrenches, double i_tm)
{
	if (i_tm == 0){
		interpolators[WRENCHES]->set(i_wrenches);
	}else{
		interpolators[WRENCHES]->setGoal(i_wrenches, i_tm);
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
    const double *q=NULL, *z=NULL, *a=NULL, *p=NULL, *e=NULL, *tq=NULL, *wr=NULL, *od=NULL; double t=0;
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
        go(q, z, a, p, e, tq, wr, od,
		   v, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
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
        interpolators[RPY]->load(hip, tm, scale, false);
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
    if (debug_level > 0) cout << endl << "torque = ";
    string torque = basename; torque.append(".torque");
    if (access(torque.c_str(),0)==0){
        found = true;
        interpolators[TQ]->load(torque, tm, scale, false);
        if (debug_level > 0) cout << torque;
    }
    if (debug_level > 0) cout << endl << "wrenches   = ";
    string wrenches = basename; wrenches.append(".wrenches");
    if (access(wrenches.c_str(),0)==0){
        found = true;
        interpolators[WRENCHES]->load(wrenches, tm, scale, false);
        if (debug_level > 0) cout << wrenches;
    }
    if (debug_level > 0) cout << endl << "optional_data   = ";
    string optional_data = basename; optional_data.append(".optionaldata");
    if (access(optional_data.c_str(),0)==0){
        found = true;
        interpolators[OPTIONAL_DATA]->load(optional_data, tm, scale, false);
        if (debug_level > 0) cout << optional_data;
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
				  double *o_basePos, double *o_baseRpy, double *o_tq, double *o_wrenches, double *o_optional_data)
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
	interpolators[TQ]->get(o_tq);
	interpolators[WRENCHES]->get(o_wrenches);
	interpolators[OPTIONAL_DATA]->get(o_optional_data);
}

void seqplay::go(const double *i_q, const double *i_zmp, const double *i_acc,
				 const double *i_p, const double *i_rpy, const double *i_tq, const double *i_wrenches, const double *i_optional_data, double i_time, 
				 bool immediate)
{
	go(i_q, i_zmp, i_acc, i_p, i_rpy, i_tq, i_wrenches, i_optional_data,
	   NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
	   i_time, immediate);
}

void seqplay::go(const double *i_q, const double *i_zmp, const double *i_acc,
				 const double *i_p, const double *i_rpy, const double *i_tq, const double *i_wrenches, const double *i_optional_data,
				 const double *ii_q, const double *ii_zmp, const double *ii_acc,
				 const double *ii_p, const double *ii_rpy, const double *ii_tq, const double *ii_wrenches, const double *ii_optional_data,
				 double i_time,	 bool immediate)
{
	if (i_q) interpolators[Q]->go(i_q, ii_q, i_time, false);
	if (i_zmp) interpolators[ZMP]->go(i_zmp, ii_zmp, i_time, false);
	if (i_acc) interpolators[ACC]->go(i_acc, ii_acc, i_time, false);
	if (i_p) interpolators[P]->go(i_p, ii_p, i_time, false);
	if (i_rpy) interpolators[RPY]->go(i_rpy, ii_rpy, i_time, false);
	if (i_tq) interpolators[TQ]->go(i_tq, ii_tq, i_time, false);
	if (i_wrenches) interpolators[WRENCHES]->go(i_wrenches, ii_wrenches, i_time, false);
	if (i_optional_data) interpolators[OPTIONAL_DATA]->go(i_optional_data, ii_optional_data, i_time, false);
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
		std::map<std::string, groupInterpolator *>::iterator it;
        for (it=groupInterpolators.begin(); it!=groupInterpolators.end(); it++){
			if ( it->first != std::string(gname) ) { // other 
				groupInterpolator *gi = it->second;
				if (gi && (gi->state == groupInterpolator::created || gi->state == groupInterpolator::working) && gi->inter->isEmpty()) {
					gi->set(full);
				}
			}
		}
        // update for non working interpolators
		
		return true;
	}else{
		std::cerr << "[resetJointGroup] group name " << gname << " is not installed" << std::endl;
		return false;
	}
}

bool seqplay::setJointAnglesOfGroup(const char *gname, const double* i_qRef, const size_t i_qsize, double i_tm)
{
	char *s = (char *)gname; while(*s) {*s=toupper(*s);s++;}
	groupInterpolator *i = groupInterpolators[gname];
	if (i){
		if (i_qsize != i->indices.size() ) {
			std::cerr << "[setJointAnglesOfGroup] group name " << gname << " : size of manipulater is not equal to input. " << i_qsize << " /= " << i->indices.size() << std::endl;
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
		double x[i->indices.size()], v[i->indices.size()];
		i->inter->get(x, v, false);
		i->setGoal(i_qRef, i_tm);
		return true;
	}else{
		std::cerr << "[setJointAnglesOfGroup] group name " << gname << " is not installed" << std::endl;
		return false;
	}
}

void seqplay::clearOfGroup(const char *gname, double i_timeLimit)
{
	char *s = (char *)gname; while(*s) {*s=toupper(*s);s++;}
	groupInterpolator *i = groupInterpolators[gname];
	if (i){
		i->clear(i_timeLimit);
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

bool seqplay::setJointAnglesSequence(std::vector<const double*> pos, std::vector<double> tm)
{
	// setJointAngles to override curren tgoal
	double x[m_dof], v[m_dof], a[m_dof];
	interpolators[Q]->get(x, v, a, false);
	interpolators[Q]->set(x, v);
	interpolators[Q]->clear();
	interpolators[Q]->push(x, v, a, true);

    const double *q=NULL;
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
			const double *q_prev = i==0?x:pos[i-1];
			for (int j = 0; j < m_dof; j++) {
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
			for (int j = 0; j < m_dof; j++) { v[j] = 0.0; }
		}

		interpolators[Q]->setGoal(pos[i], v, tm[i], false);
		do{
			interpolators[Q]->interpolate(tm[i]);
		}while(tm[i]>0);
		sync();
	}
	return true;
}

bool seqplay::clearJointAngles()
{
	// setJointAngles to override curren tgoal
	double x[m_dof], v[m_dof], a[m_dof];
	interpolators[Q]->get(x, v, a, false);
	interpolators[Q]->set(x, v);
	interpolators[Q]->clear();
	double tm = interpolators[Q]->deltaT();
	interpolators[Q]->setGoal(x, v, tm, false);
	do{
		interpolators[Q]->interpolate(tm);
	}while(tm>0);
	sync();
	return true;
}

bool seqplay::setJointAnglesSequenceFull(std::vector<const double*> i_pos, std::vector<const double*> i_vel, std::vector<const double*> i_torques, std::vector<const double*> i_bpos, std::vector<const double*> i_brpy, std::vector<const double*> i_bacc,  std::vector<const double*> i_zmps, std::vector<const double*> i_wrenches, std::vector<const double*> i_optionals, std::vector<double> i_tm)
{
	// setJointAngles to override curren tgoal
	double x[m_dof], v[m_dof], a[m_dof];
	interpolators[Q]->get(x, v, a, false);
	interpolators[Q]->set(x, v);
	interpolators[Q]->clear();
	interpolators[Q]->push(x, v, a, true);
	double torque[m_dof], dummy_dof[m_dof];
	for (int j = 0; j < m_dof; j++) { dummy_dof[j] = 0.0; }
	interpolators[TQ]->get(torque, false);
	interpolators[TQ]->set(torque);
	interpolators[TQ]->clear();
	interpolators[TQ]->push(torque, dummy_dof, dummy_dof, true);
	double bpos[3], brpy[3], bacc[3], dummy_3[3]={0,0,0};
	interpolators[P]->get(bpos, false);
	interpolators[P]->set(bpos);
	interpolators[P]->clear();
	interpolators[P]->push(bpos, dummy_3, dummy_3, true);
	interpolators[RPY]->get(brpy, false);
	interpolators[RPY]->set(brpy);
	interpolators[RPY]->clear();
	interpolators[RPY]->push(brpy, dummy_3, dummy_3, true);
	interpolators[ACC]->get(bacc, false);
	interpolators[ACC]->set(bacc);
	interpolators[ACC]->clear();
	interpolators[ACC]->push(bacc, dummy_3, dummy_3, true);
	int fnum = interpolators[WRENCHES]->dimension()/6, optional_data_dim = interpolators[OPTIONAL_DATA]->dimension();
	double zmp[3], wrench[6*fnum], dummy_fnum[6*fnum], optional[optional_data_dim], dummy_optional[optional_data_dim];
	for (int j = 0; j < 6*fnum; j++) { dummy_dof[j] = 0.0; }
	for (int j = 0; j < optional_data_dim; j++) { dummy_optional[j] = 0.0; }
	interpolators[ZMP]->get(zmp, false);
	interpolators[ZMP]->set(zmp);
	interpolators[ZMP]->clear();
	interpolators[ZMP]->push(zmp, dummy_3, dummy_3, true);
	interpolators[WRENCHES]->get(wrench, false);
	interpolators[WRENCHES]->set(wrench);
	interpolators[WRENCHES]->clear();
	interpolators[WRENCHES]->push(wrench, dummy_fnum, dummy_fnum, true);
	interpolators[OPTIONAL_DATA]->get(optional, false);
	interpolators[OPTIONAL_DATA]->set(optional);
	interpolators[OPTIONAL_DATA]->clear();
	interpolators[OPTIONAL_DATA]->push(optional, dummy_optional, dummy_optional, true);

    const double *q=NULL;
    for (unsigned int i=0; i<i_pos.size(); i++){
		if (i_vel.size() > 0 ) {
			for (int j = 0; j < m_dof; j++) {
				v[j] = i_vel[i][j];
			}
		}else{
			q = i_pos[i];
			if (i < i_pos.size() - 1 ) {
				double t0, t1;
				if (i_tm.size() == i_pos.size()) {
					t0 = i_tm[i]; t1 = i_tm[i+1];
				} else {
					t0 = t1 = i_tm[0];
				}
				const double *q_next = i_pos[i+1];
				const double *q_prev = i==0?x:i_pos[i-1];
				for (int j = 0; j < m_dof; j++) {
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
				for (int j = 0; j < m_dof; j++) { v[j] = 0.0; }
			}
		}

		interpolators[Q]->setGoal(i_pos[i], v, i_tm[i], false);
		interpolators[TQ]->setGoal(i_torques[i], i_tm[i], false);
		interpolators[P]->setGoal(i_bpos[i], i_tm[i], false);
		interpolators[RPY]->setGoal(i_brpy[i], i_tm[i], false);
		interpolators[ACC]->setGoal(i_bacc[i], i_tm[i], false);
		interpolators[ZMP]->setGoal(i_zmps[i], i_tm[i], false);
		interpolators[WRENCHES]->setGoal(i_wrenches[i], i_tm[i], false);
		interpolators[OPTIONAL_DATA]->setGoal(i_optionals[i], i_tm[i], false);
		do{
			double tm = i_tm[i], tm_tmp;
			interpolators[Q]->interpolate(i_tm[i]);
			tm_tmp = tm; interpolators[TQ]->interpolate(tm_tmp);
			tm_tmp = tm; interpolators[P]->interpolate(tm_tmp);
			tm_tmp = tm; interpolators[RPY]->interpolate(tm_tmp);
			tm_tmp = tm; interpolators[ACC]->interpolate(tm_tmp);
			tm_tmp = tm; interpolators[ZMP]->interpolate(tm_tmp);
			tm_tmp = tm; interpolators[WRENCHES]->interpolate(tm_tmp);
			tm_tmp = tm; interpolators[OPTIONAL_DATA]->interpolate(tm_tmp);
		}while(i_tm[i]>0);
		sync();
	}
	return true;
}

bool seqplay::setJointAnglesSequenceOfGroup(const char *gname, std::vector<const double*> pos, std::vector<double> tm, const size_t pos_size)
{
	char *s = (char *)gname; while(*s) {*s=toupper(*s);s++;}
	groupInterpolator *i = groupInterpolators[gname];

	if (! i){
		std::cerr << "[setJointAnglesSequenceOfGroup] group name " << gname << " is not installed" << std::endl;
		return false;
	}
	if (pos_size != i->indices.size() ) {
		std::cerr << "[setJointAnglesSequenceOfGroup] group name " << gname << " : size of manipulater is not equal to input. " << pos_size << " /= " << i->indices.size() << std::endl;
		return false;
	}
	int len = i->indices.size();
	double x[len], v[len];
	double q[m_dof], dq[m_dof];
	i->inter->get(q, dq, false);
	i->inter->set(q, dq);
	i->extract(x, q);
	i->extract(v, dq);
	i->inter->clear();
    const double *q_curr=NULL;
    for (unsigned int j=0; j<pos.size(); j++){
        q_curr = pos[j];
		if ( j < pos.size() - 1 ) {
			double t0, t1;
			if (tm.size() == pos.size()) {
				t0 = tm[j]; t1 = tm[j+1];
			} else {
				t0 = t1 = tm[0];
			}
			const double *q_next = pos[j+1];
			const double *q_prev = j==0?x:pos[j-1];
			for (int k = 0; k < len; k++) {
				double d0, d1, v0, v1;
				d0 = (q_curr[k] - q_prev[k]);
				d1 = (q_next[k] - q_curr[k]);
				v0 = d0/t0;
				v1 = d1/t1;
				if ( v0 * v1 >= 0 ) {
					v[k] = 0.5 * (v0 + v1);
				} else {
					v[k] = 0;
				}
			}
		} else {
			for (int k = 0; k < len; k++) { v[k] = 0.0; }
		}
		if (i->state == groupInterpolator::created){
			interpolators[Q]->get(q, dq, false);
			std::map<std::string, groupInterpolator *>::iterator it;
			for (it=groupInterpolators.begin(); it!=groupInterpolators.end(); it++){
				groupInterpolator *gi = it->second;
				if (gi)	gi->get(q, dq, false);
			}
			i->extract(x, q);
			i->extract(v, dq);
			i->inter->go(x,v,interpolators[Q]->deltaT());
		}
		i->inter->setGoal(pos[j], v, tm[j], false);
		do{
			i->inter->interpolate(tm[j]);
		}while(tm[j]>0);
		i->inter->sync();
		i->state = groupInterpolator::working;
	}
	return true;
}

bool seqplay::clearJointAnglesOfGroup(const char *gname)
{
	char *s = (char *)gname; while(*s) {*s=toupper(*s);s++;}
	groupInterpolator *i = groupInterpolators[gname];

	if (! i){
		std::cerr << "[clearJointAnglesOfGroup] group name " << gname << " is not installed" << std::endl;
		return false;
	}

	if (i->state == groupInterpolator::created){
		std::cerr << "[clearJointAnglesOfGroup] group name " << gname << " is not created" << std::endl;
		return false;
	}

	if (i->state == groupInterpolator::removing || i->state == groupInterpolator::removed){
		std::cerr << "[clearJointAnglesOfGroup] group name " << gname << " is removing" << std::endl;
		return false;
	}

	int len = i->indices.size();
	double x[len], v[len], a[len];
	i->inter->get(x, v, a, false);
	i->inter->set(x, v);
	while(i->inter->remain_time() > 0){
		i->inter->pop();
	}
	double tm = interpolators[Q]->deltaT();
	i->inter->setGoal(x, v, tm, true);// true: update remian_t
	do{
		i->inter->interpolate(tm);
	}while(tm>0);
	i->inter->sync();

	return true;
}
