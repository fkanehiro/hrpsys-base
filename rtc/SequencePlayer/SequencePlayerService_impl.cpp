#include "SequencePlayerService_impl.h"
#include "SequencePlayer.h"
#include <hrpModel/Body.h>
#include <hrpModel/Link.h>

SequencePlayerService_impl::SequencePlayerService_impl() : m_player(NULL)
{
}

SequencePlayerService_impl::~SequencePlayerService_impl()
{
}

void SequencePlayerService_impl::waitInterpolation()
{
  m_player->waitInterpolation();
}

CORBA::Boolean SequencePlayerService_impl::waitInterpolationOfGroup(const char *gname)
{
    return m_player->waitInterpolationOfGroup(gname);
}

CORBA::Boolean SequencePlayerService_impl::setJointAngles(const dSequence& jvs, CORBA::Double tm)
{
  if (jvs.length() != (unsigned int)(m_player->robot()->numJoints())) {
      std::cerr << __PRETTY_FUNCTION__ << " num of joint is differ, input:" << jvs.length() << ", robot:" << (unsigned int)(m_player->robot()->numJoints()) << std::endl;
      return false;
  }
  return m_player->setJointAngles(jvs.get_buffer(), tm);
}

CORBA::Boolean SequencePlayerService_impl::setJointAnglesWithMask(const dSequence& jvs, const bSequence& mask, CORBA::Double tm)
{
    if (jvs.length() != (unsigned int)(m_player->robot()->numJoints())
        || mask.length() != (unsigned int)(m_player->robot()->numJoints())) {
        std::cerr << __PRETTY_FUNCTION__ << " num of joint is differ, input:" << jvs.length() << ", mask:" << mask.length() << ", robot" << (unsigned int)(m_player->robot()->numJoints()) << std::endl;
        return false;
    }
    return m_player->setJointAngles(jvs.get_buffer(), mask.get_buffer(), tm);
}

CORBA::Boolean SequencePlayerService_impl::setJointAnglesSequence(const dSequenceSequence& jvss, const dSequence& tms)
{
  const OpenHRP::bSequence mask;
  return setJointAnglesSequenceWithMask(jvss, mask, tms);
}

CORBA::Boolean SequencePlayerService_impl::clearJointAngles()
{
  return m_player->clearJointAngles();
}

CORBA::Boolean SequencePlayerService_impl::setJointAnglesSequenceWithMask(const dSequenceSequence& jvss, const bSequence& mask, const dSequence& tms)
{
  if (jvss.length() <= 0) {
      std::cerr << __PRETTY_FUNCTION__ << " num of joint angles sequence is invalid:" << jvss.length() << " > 0" << std::endl;
      return false;
  }
  if (jvss.length() != tms.length()) {
      std::cerr << __PRETTY_FUNCTION__ << " length of joint angles sequence and time sequence differ, joint angle:" << jvss.length() << ", time:" << tms.length() << std::endl;
      return false;
  }
  const dSequence& jvs = jvss[0];
  if (jvs.length() != (unsigned int)(m_player->robot()->numJoints())) {
      std::cerr << __PRETTY_FUNCTION__ << " num of joint is differ, input:" << jvs.length() << ", robot:" << (unsigned int)(m_player->robot()->numJoints()) << std::endl;
      return false;
  }

  if (mask.length() > 0 && mask.length() != (unsigned int)(m_player->robot()->numJoints())) {
      std::cerr << __PRETTY_FUNCTION__ << " num of joint is differ, input:" << jvs.length() << ", mask:" << mask.length() << ", robot" << (unsigned int)(m_player->robot()->numJoints()) << std::endl;
      return false;
  }
  
  return m_player->setJointAnglesSequence(jvss, mask, tms);
}

CORBA::Boolean SequencePlayerService_impl::setJointAnglesSequenceFull(const dSequenceSequence& jvss, const dSequenceSequence& vels, const dSequenceSequence& torques, const dSequenceSequence& poss, const dSequenceSequence& rpys, const dSequenceSequence& accs, const dSequenceSequence& zmps, const dSequenceSequence& wrenches, const dSequenceSequence& optionals, const dSequence &tms)
{
  if (jvss.length() <= 0) {
      std::cerr << __PRETTY_FUNCTION__ << " num of joint angles sequence is invalid:" << jvss.length() << " > 0" << std::endl;
      return false;
  }
  if (jvss.length() != tms.length()) {
      std::cerr << __PRETTY_FUNCTION__ << " length of joint angles sequence and time sequence differ, joint angle:" << jvss.length() << ", time:" << tms.length() << std::endl;
      return false;
  }
  const dSequence& jvs = jvss[0];
  if (jvs.length() != (unsigned int)(m_player->robot()->numJoints())) {
      std::cerr << __PRETTY_FUNCTION__ << " num of joint is differ, input:" << jvs.length() << ", robot:" << (unsigned int)(m_player->robot()->numJoints()) << std::endl;
      return false;
  }
  if (vels.length() > 0 ) {
      const dSequence& vel = vels[0];
      if (vels.length() != tms.length()) {
          std::cerr << __PRETTY_FUNCTION__ << " length of joint velocitys sequence and time sequence differ, joint velocity:" << vels.length() << ", time:" << tms.length() << std::endl;
          return false;
      }
      if ( vel.length() != (unsigned int)(m_player->robot()->numJoints())) {
          std::cerr << __PRETTY_FUNCTION__ << " num of joint is differ, input:" << jvs.length() << ", vel:" << vel.length() << ", robot" << (unsigned int)(m_player->robot()->numJoints()) << std::endl;
          return false;
      }
  }
  if (torques.length() > 0 ) {
      const dSequence& torque = torques[0];
      if (torques.length() != tms.length()) {
          std::cerr << __PRETTY_FUNCTION__ << " length of joint torque sequence and time sequence differ, joint torque:" << torques.length() << ", time:" << tms.length() << std::endl;
          return false;
      }
      if ( torque.length() != (unsigned int)(m_player->robot()->numJoints())) {
          std::cerr << __PRETTY_FUNCTION__ << " num of joint is differ, input:" << jvs.length() << ", torque:" << torque.length() << ", robot" << (unsigned int)(m_player->robot()->numJoints()) << std::endl;
          return false;
      }
  }

  if (poss.length() > 0) {
      const dSequence& pos = poss[0];
      if (poss.length() != tms.length()) {
          std::cerr << __PRETTY_FUNCTION__ << " length of base pos sequence and time sequence differ, pos:" << poss.length() << ", time:" << tms.length() << std::endl;
          return false;
      }
      if ( pos.length() != 3) {
          std::cerr << __PRETTY_FUNCTION__ << " num of base pos is differ, pos:" << pos.length() << std::endl;
          return false;
      }
  }

  if (rpys.length() > 0) {
      const dSequence& rpy = rpys[0];
      if (rpys.length() != tms.length()) {
          std::cerr << __PRETTY_FUNCTION__ << " length of base rpy sequence and time sequence differ, rpy:" << rpys.length() << ", time:" << tms.length() << std::endl;
          return false;
      }
      if ( rpy.length() != 3) {
          std::cerr << __PRETTY_FUNCTION__ << " num of base rpy is differ, rpy:" << rpy.length() << std::endl;
          return false;
      }
  }

  if (accs.length() > 0 ) {
      const dSequence& acc = accs[0];
      if (accs.length() != tms.length()) {
          std::cerr << __PRETTY_FUNCTION__ << " length of joint accocitys sequence and time sequence differ, joint accocity:" << accs.length() << ", time:" << tms.length() << std::endl;
          return false;
      }
      if ( acc.length() != 3) {
          std::cerr << __PRETTY_FUNCTION__ << " num of joint is differ, input:" << jvs.length() << ", acc:" << acc.length() << ", robot" << (unsigned int)(m_player->robot()->numJoints()) << std::endl;
          return false;
      }
  }

  if (zmps.length() > 0) {
      const dSequence& zmp = zmps[0];
      if (zmps.length() != tms.length()) {
          std::cerr << __PRETTY_FUNCTION__ << " length of zmp sequence and time sequence differ, zmp:" << zmps.length() << ", time:" << tms.length() << std::endl;
          return false;
      }
      if ( zmp.length() != 3) {
          std::cerr << __PRETTY_FUNCTION__ << " num of zmp is differ, zmp:" << zmp.length() <<  std::endl;
          return false;
      }
  }

  if (wrenches.length() > 0) {
      if (wrenches.length() != tms.length()) {
          std::cerr << __PRETTY_FUNCTION__ << " length of wrench sequence and time sequence differ, wrench:" << wrenches.length() << ", time:" << tms.length() << std::endl;
          return false;
      }
      // need to check size of wrench
      //const dSequence& wrench = wrenches[0];
  }

  if (optionals.length() > 0) {
      if (optionals.length() != tms.length()) {
          std::cerr << __PRETTY_FUNCTION__ << " length of optional sequence and time sequence differ, optional:" << optionals.length() << ", time:" << tms.length() << std::endl;
          return false;
      }
      // need to check size of optional
      //const dSequence& optional = optionasl[0];
  }

  return m_player->setJointAnglesSequenceFull(jvss, vels, torques, poss, rpys, accs, zmps, wrenches, optionals, tms);
}

CORBA::Boolean SequencePlayerService_impl::setJointAngle(const char *jname, CORBA::Double jv, CORBA::Double tm)
{
    BodyPtr r = m_player->robot();
    Link *l = r->link(jname);
    if (!l){
        std::cerr << "can't find(" << jname << ")" << std::endl;
        return false;
    }
    int id = l->jointId;
    return m_player->setJointAngle(id, jv, tm);
}

CORBA::Boolean SequencePlayerService_impl::setBasePos(const dSequence& pos, CORBA::Double tm)
{
    if (pos.length() != 3) return false;

    return m_player->setBasePos(pos.get_buffer(), tm);
}

CORBA::Boolean SequencePlayerService_impl::setBaseRpy(const dSequence& rpy, CORBA::Double tm)
{
    if (rpy.length() != 3) return false;

    return m_player->setBaseRpy(rpy.get_buffer(), tm);
}

CORBA::Boolean SequencePlayerService_impl::setZmp(const dSequence& zmp, CORBA::Double tm)
{
    if (zmp.length() != 3) return false;

    return m_player->setZmp(zmp.get_buffer(), tm);
}

CORBA::Boolean SequencePlayerService_impl::setWrenches(const dSequence& wrenches, CORBA::Double tm)
{
  //if (wrenches.length() != ) return false;

    return m_player->setWrenches(wrenches.get_buffer(), tm);
}

CORBA::Boolean SequencePlayerService_impl::setTargetPose(const char* gname, const dSequence& xyz, const dSequence& rpy, CORBA::Double tm){
    char* frame_name = (char *)strrchr(gname, ':');
    if ( frame_name ) {
        ((char *)gname)[frame_name - gname] = '\0'; // cut frame_name, gname[strpos(':')] = 0x00
        frame_name++; // skip ":"
    }
    return m_player->setTargetPose(gname, xyz.get_buffer(), rpy.get_buffer(), tm, frame_name);
}

CORBA::Boolean SequencePlayerService_impl::isEmpty()
{
  return m_player->player()->isEmpty();
}

void SequencePlayerService_impl::loadPattern(const char* basename, CORBA::Double tm)
{
  if (!m_player->player()){
    std::cerr << "player is not set"<< std::endl;
    return;
  }
  m_player->loadPattern(basename, tm);
}

void SequencePlayerService_impl::clear()
{
  m_player->player()->clear();
}

CORBA::Boolean  SequencePlayerService_impl::clearOfGroup(const char *gname, CORBA::Double i_limitation)
{
    m_player->player()->clearOfGroup(gname, i_limitation);
}

void SequencePlayerService_impl::clearNoWait()
{
  m_player->setClearFlag();
}

CORBA::Boolean SequencePlayerService_impl::setInterpolationMode(OpenHRP::SequencePlayerService::interpolationMode i_mode_)
{
  return m_player->setInterpolationMode(i_mode_);
}

CORBA::Boolean SequencePlayerService_impl::setInitialState()
{
  m_player->setInitialState();
  return m_player->setInitialState(m_player->dt);
}

void SequencePlayerService_impl::player(SequencePlayer *i_player)
{
  m_player = i_player;
} 

void SequencePlayerService_impl::playPattern(const dSequenceSequence& pos, const dSequenceSequence& rpy, const dSequenceSequence& zmp, const dSequence& tm)
{
    m_player->playPattern(pos, rpy, zmp, tm);
}

CORBA::Boolean SequencePlayerService_impl::addJointGroup(const char* gname, const OpenHRP::SequencePlayerService::StrSequence& jnames)
{
    return m_player->addJointGroup(gname, jnames);
}

CORBA::Boolean SequencePlayerService_impl::removeJointGroup(const char* gname)
{
    return m_player->removeJointGroup(gname);
}

CORBA::Boolean SequencePlayerService_impl::setJointAnglesOfGroup(const char *gname, const dSequence& jvs, CORBA::Double tm)
{
    return m_player->setJointAnglesOfGroup(gname, jvs, tm);
}

CORBA::Boolean SequencePlayerService_impl::setJointAnglesSequenceOfGroup(const char *gname, const dSequenceSequence& jvss, const dSequence& tms)
{
    if (jvss.length() != tms.length()) {
        std::cerr << __PRETTY_FUNCTION__ << " length of joint angles sequence and time sequence differ, joint angle:" << jvss.length() << ", time:" << tms.length() << std::endl;
        return false;
    }
    return m_player->setJointAnglesSequenceOfGroup(gname, jvss, tms);
}

CORBA::Boolean SequencePlayerService_impl::clearJointAnglesOfGroup(const char *gname)
{
    return m_player->clearJointAnglesOfGroup(gname);
}

CORBA::Boolean SequencePlayerService_impl::playPatternOfGroup(const char *gname, const dSequenceSequence& pos, const dSequence& tm)
{
    return m_player->playPatternOfGroup(gname, pos, tm);
}

void SequencePlayerService_impl::setMaxIKError(CORBA::Double pos, CORBA::Double rot)
{
    return m_player->setMaxIKError(pos, rot);
}

void SequencePlayerService_impl::setMaxIKIteration(CORBA::Short iter)
{
    return m_player->setMaxIKIteration(iter);
}
