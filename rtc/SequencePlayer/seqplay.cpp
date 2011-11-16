// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-

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
	return true;
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
    dvector pos(m_dof);
    interpolators[Q]->get(&pos[0], false);
    pos[i_rank] = jv;
    interpolators[Q]->setGoal(&pos[0], tm);
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
            interpolators[RPY]->load(hip, tm, scale, false);
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
	interpolators[Q]->get(o_q);
	interpolators[ZMP]->get(o_zmp);
	interpolators[ACC]->get(o_accel);
	interpolators[P]->get(o_basePos);
	interpolators[RPY]->get(o_baseRpy);
}

void seqplay::go(const double *i_q, const double *i_zmp, const double *i_acc,
				 const double *i_p, const double *i_rpy, double i_time, 
				 bool immediate)
{
	if (i_q) interpolators[Q]->go(i_q, i_time, false);
	if (i_zmp) interpolators[ZMP]->go(i_zmp, i_time, false);
	if (i_acc) interpolators[ACC]->go(i_acc, i_time, false);
	if (i_p) interpolators[P]->go(i_p, i_time, false);
	if (i_rpy) interpolators[RPY]->go(i_rpy, i_time, false);
	if (immediate) sync();
}

void seqplay::push(const double *i_q, const double *i_zmp, 
				   const double *i_acc,
				   const double *i_p, const double *i_rpy, 
				   bool immediate)
{
	if (i_q) interpolators[Q]->push(i_q, false);
	if (i_zmp) interpolators[ZMP]->push(i_zmp, false);
	if (i_acc) interpolators[ACC]->push(i_acc, false);
	if (i_p) interpolators[P]->push(i_p, false);
	if (i_rpy) interpolators[RPY]->push(i_rpy, false);
	if (immediate) sync();
}

bool seqplay::setInterpolationMode (interpolator::interpolation_mode i_mode_)
{
    if (i_mode_ != interpolator::LINEAR && i_mode_ != interpolator::HOFFARBIB) return false;

	bool ret=true; 
	for (unsigned int i=0; i<NINTERPOLATOR; i++){
		ret &= interpolators[i]->setInterpolationMode(i_mode_);
	}
	return ret;
}
