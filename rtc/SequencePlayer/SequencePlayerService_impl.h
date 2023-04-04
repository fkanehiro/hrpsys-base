// -*-C++-*-
#ifndef SEQPLAYSERVICESVC_IMPL_H
#define SEQPLAYSERVICESVC_IMPL_H

#include "hrpsys/idl/SequencePlayerService.hh"

using namespace OpenHRP;

class SequencePlayer;

class SequencePlayerService_impl 
  : public virtual POA_OpenHRP::SequencePlayerService,
    public virtual PortableServer::RefCountServantBase
{
public:
  SequencePlayerService_impl();
  virtual ~SequencePlayerService_impl();
  //
  void waitInterpolation();
  CORBA::Boolean waitInterpolationOfGroup(const char *gname);
  CORBA::Boolean setJointAnglesSequence(const dSequenceSequence& jvs, const dSequence &tms);
  CORBA::Boolean setJointAnglesSequenceWithMask(const dSequenceSequence& jvs, const bSequence& mask, const dSequence &tms);
  CORBA::Boolean setJointAnglesSequenceFull(const dSequenceSequence& jvss, const dSequenceSequence& vels, const dSequenceSequence& torques, const dSequenceSequence& poss, const dSequenceSequence& rpys, const dSequenceSequence& accs, const dSequenceSequence& zmps, const dSequenceSequence& wrenches, const dSequenceSequence& optionals, const dSequence &tms);
  CORBA::Boolean clearJointAngles();
  CORBA::Boolean setJointAngles(const dSequence& jvs, CORBA::Double tm);
  CORBA::Boolean setJointAnglesWithMask(const dSequence& jvs, const bSequence& mask, CORBA::Double tm);
  CORBA::Boolean setJointAngle(const char *jname, CORBA::Double jv, CORBA::Double tm);
  CORBA::Boolean setJointVelocities(const dSequence& jvs, CORBA::Double tm);
  CORBA::Boolean setJointVelocitiesSequence(const dSequenceSequence& jvs, const dSequence &tms);
  CORBA::Boolean setJointTorques(const dSequence& jvs, CORBA::Double tm);
  CORBA::Boolean setJointTorquesSequence(const dSequenceSequence& jvs, const dSequence &tms);
  CORBA::Boolean setBasePos(const dSequence& pos, CORBA::Double tm);
  CORBA::Boolean setBaseRpy(const dSequence& rpy, CORBA::Double tm);
  CORBA::Boolean setZmp(const dSequence& zmp, CORBA::Double tm);
  CORBA::Boolean setWrenches(const dSequence& wrenches, CORBA::Double tm);
  CORBA::Boolean setTargetPose(const char* gname, const dSequence& xyz, const dSequence& rpy, CORBA::Double tm);
  CORBA::Boolean isEmpty();
  void loadPattern(const char* basename, CORBA::Double tm);
  void playPattern(const dSequenceSequence& pos, const dSequenceSequence& rpy, const dSequenceSequence& zmp, const dSequence& tm);
  void clear();
  void clearNoWait();
  CORBA::Boolean setInterpolationMode(OpenHRP::SequencePlayerService::interpolationMode i_mode_);
  CORBA::Boolean setInitialState();
  CORBA::Boolean addJointGroup(const char* gname, const OpenHRP::SequencePlayerService::StrSequence& jnames);
  CORBA::Boolean removeJointGroup(const char* gname);
  CORBA::Boolean setJointAnglesOfGroup(const char *gname, const dSequence& jvs, CORBA::Double tm);
  CORBA::Boolean setJointAnglesSequenceOfGroup(const char *gname, const dSequenceSequence& jvs, const dSequence &tms);
  CORBA::Boolean clearJointAnglesOfGroup(const char *gname);
  CORBA::Boolean clearOfGroup(const char *gname, CORBA::Double  i_timelimit);
  CORBA::Boolean playPatternOfGroup(const char *gname, const dSequenceSequence& pos, const dSequence& tm);
  void setMaxIKError(CORBA::Double pos, CORBA::Double rot);
  void setMaxIKIteration(CORBA::Short iter);
  //
  void player(SequencePlayer *i_player);
  SequencePlayer *m_player;
};				 

#endif
