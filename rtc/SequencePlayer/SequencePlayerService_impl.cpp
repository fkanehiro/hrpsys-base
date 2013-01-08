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

CORBA::Boolean SequencePlayerService_impl::setJointAngles(const dSequence& jvs, CORBA::Double tm)
{
  if (jvs.length() != (unsigned int)(m_player->robot()->numJoints())) return false;  
  return m_player->setJointAngles(jvs.get_buffer(), tm);
}

CORBA::Boolean SequencePlayerService_impl::setJointAnglesWithMask(const dSequence& jvs, const bSequence& mask, CORBA::Double tm)
{
    if (jvs.length() != (unsigned int)(m_player->robot()->numJoints())
        || mask.length() != (unsigned int)(m_player->robot()->numJoints())) return false;  
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
