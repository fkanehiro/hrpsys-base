#include <hrpModel/Link.h>
#include "scc.h"

using namespace hrp;

SelfCollisionChecker::SelfCollisionChecker(hrp::BodyPtr body) : m_robot(body)
{
    for (int i=0; i<m_robot->numLinks(); i++){
        Link *link1 = m_robot->link(i);
        for (int j=i+1; j<m_robot->numLinks(); j++){
            Link *link2 = m_robot->link(j);
            if (link1->parent != link2 && link2->parent != link1){
                m_checkPairs.push_back(ColdetModelPair(link1->coldetModel,
                                                       link2->coldetModel));
            }
        }
    }
}


std::vector<std::pair<std::string, std::string> > SelfCollisionChecker::check(const double *q)
{
    std::vector<std::pair<std::string, std::string> > pairs;

    for (int i=0; i<m_robot->numJoints(); i++){
        m_robot->joint(i)->q = q[i];
    }
    m_robot->calcForwardKinematics();
    for (int i=0; i<m_robot->numLinks(); i++){
        Link *l = m_robot->link(i);
        l->coldetModel->setPosition(l->R, l->p);
    }
    for (unsigned int i=0; i<m_checkPairs.size(); i++){
        if (m_checkPairs[i].checkCollision()){
            pairs.push_back(std::make_pair(m_checkPairs[i].model(0)->name(),
                                           m_checkPairs[i].model(1)->name()));
        }
    }
    return pairs;
}


