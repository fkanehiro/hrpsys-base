// -*-C++-*-
#ifndef SEQPLAYSERVICESVC_IMPL_H
#define SEQPLAYSERVICESVC_IMPL_H

#include "SequencePlayerService.hh"

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
  CORBA::Boolean setJointAngles(const dSequence& jvs, CORBA::Double tm);
  CORBA::Boolean setJointAnglesWithMask(const dSequence& jvs, const bSequence& mask, CORBA::Double tm);
  CORBA::Boolean setJointAngle(const char *jname, CORBA::Double jv, CORBA::Double tm);
  CORBA::Boolean setBasePos(const dSequence& pos, CORBA::Double tm);
  CORBA::Boolean setBaseRpy(const dSequence& rpy, CORBA::Double tm);
  CORBA::Boolean setZmp(const dSequence& zmp, CORBA::Double tm);
  CORBA::Boolean isEmpty();
  void loadPattern(const char* basename, CORBA::Double tm);
  void playPattern(const dSequenceSequence& pos, const dSequenceSequence& rpy, const dSequenceSequence& zmp, const dSequence& tm);
  void clear();
  void clearNoWait();
  CORBA::Boolean setInterpolationMode(OpenHRP::SequencePlayerService::interpolationMode i_mode_);
  CORBA::Boolean setInitialState();
  //
  void player(SequencePlayer *i_player);
private:
  SequencePlayer *m_player;
};				 

#endif
