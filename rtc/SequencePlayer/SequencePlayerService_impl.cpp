#include <iostream>

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

CORBA::Boolean SequencePlayerService_impl::setTargetPoseMatrix(const char* gname, const dSequence& xyz, const dSequence& rot, CORBA::Double tm) {
    char* frame_name = (char *)strrchr(gname, ':');
    if ( frame_name ) {
        ((char *)gname)[frame_name - gname] = '\0'; // cut frame_name, gname[strpos(':')] = 0x00
        frame_name++; // skip ":"
    }
    return m_player->setTargetPoseMatrix(gname, xyz.get_buffer(), rot.get_buffer(), tm, frame_name);
}

CORBA::Boolean SequencePlayerService_impl::isEmpty()
{
  return m_player->player()->isEmpty();
}

void SequencePlayerService_impl::loadPattern(const char* basename, CORBA::Double tm)
{
  if (!m_player->player()){
    std::cerr << "player is not set" << std::endl;
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
    return m_player->setJointAnglesOfGroup(gname, jvs.get_buffer(), tm);
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
