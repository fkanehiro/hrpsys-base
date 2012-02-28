// -*- C++ -*-
/*!
 * @file SetupCollisionPair.cpp
 * @brief Standalone component
 * @date $Date$
 *
 * $Id$
 */

#include <hrpModel/Link.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpModel/JointPath.h>
#include <hrpUtil/Tvmet3d.h>
#include <hrpUtil/Tvmet4d.h>
#include <hrpCollision/ColdetModel.h>
#include "CollisionDetector.h"

#define deg2rad(x)	((x)*M_PI/180)
#define rad2deg(x)      ((x)*180/M_PI)

std::vector<hrp::ColdetLinkPairPtr> m_pair;
std::vector<std::string> blacklist;

bool checkCollisionForAllJointRange(hrp::BodyPtr m_robot, int i, hrp::JointPathPtr jointPath, hrp::ColdetLinkPairPtr collisionPair)
{
    if ( i >= jointPath->numJoints() ) return false;
    hrp::Link *l = jointPath->joint(i);

    if ( l->jointType == hrp::Link::FIXED_JOINT ) {
	checkCollisionForAllJointRange(m_robot, i+1, jointPath, collisionPair);
    } else {
	double step = (l->ulimit - l->llimit)/2;
	for(float angle = l->llimit; angle <= l->ulimit; angle += step)
	{
	    l->q = angle;
	    m_robot->calcForwardKinematics(false,false);
	    m_robot->updateLinkColdetModelPositions();

	    //std::cerr << "    " << collisionPair->link(0)->name << ":" << collisionPair->link(1)->name << " ";
	    //for(int j = 0; j < jointPath->numJoints();j++) std::cerr << rad2deg(jointPath->joint(j)->q) << " ";
	    // std::cerr << "id = " << i << "/" << jointPath->numJoints() << ", " << l->name << " " << rad2deg(angle) << " (" << rad2deg(l->llimit) << "," << rad2deg(l->ulimit) << ") " << collisionPair->detectIntersection() << " " << collisionPair->link(0)->name << ":" << collisionPair->link(1)->name << std::endl;
	    if ( collisionPair->detectIntersection() ) {
		//std::cerr << "collision!!!!!!!!!!!" << std::endl;
		return true;
	    }
	    //std::cerr << std::endl;
	    if ( checkCollisionForAllJointRange(m_robot, i+1, jointPath, collisionPair) )
	    {
		return true;
	    }
	}
    }
    return false;
}

bool checkBlackListJoint(hrp::Link *l) {
    for(std::vector<std::string>::iterator it = blacklist.begin(); it != blacklist.end(); it++ ) {
	    if (l->name == (*it)) return true;
    }
    return false;
}

void setupCollisionLinkPair(hrp::BodyPtr m_robot)
{
    std::vector<hrp::ColdetLinkPairPtr> tmp_pair;

    std::cerr << "Setup Initial collision pair without " << std::endl;
    for(std::vector<std::string>::iterator it = blacklist.begin(); it != blacklist.end(); it++ ) {
	std::cerr << *it << " ";
    }
    std::cerr << std::endl;
    // find link to be ignored as the link is included in the parent link
    // need AABBCollision?
    //
    // set all collisoin pair
    for (int i=0; i<m_robot->numLinks(); i++) {
	hrp::Link *l1 = m_robot->link(i);
	for (int j=i+1; j<m_robot->numLinks(); j++) {
	    hrp::Link *l2 = m_robot->link(j);
	    if ( l1->coldetModel && l2->coldetModel
		 &&  (!(checkBlackListJoint(l1) ||  checkBlackListJoint(l2)))
		) {
		tmp_pair.push_back(new hrp::ColdetLinkPair(l1, l2));
	    }
	}
    }
    
    std::cerr << "Initial collision pair size " << tmp_pair.size() << std::endl;
    std::vector<hrp::ColdetLinkPairPtr>::iterator it;
    // Remove collision pair if the pair always collides
    std::cerr << "step 0: Remove collision pair if adjacent pair" << std::endl;
    it = tmp_pair.begin();
    while ( it != tmp_pair.end() ) {
	hrp::JointPathPtr jointPath = m_robot->getJointPath((*it)->link(0),(*it)->link(1));
	if ( jointPath->numJoints() == 1 ) {
	    hrp::Link *l = jointPath->joint(0);
	    bool always_collide = true; // true if always collide
	    bool never_collide = true;  // true if never collide ( crrect )
	    for(float angle = l->llimit; angle <= l->ulimit; angle += deg2rad(5)) {
		l->q = angle;
		m_robot->calcForwardKinematics(false,false);
		m_robot->updateLinkColdetModelPositions();
		if ( (*it)->detectIntersection() ) {
		    never_collide = false;
		} else {
		    always_collide = false;
		}
		//std::cerr << l->name << " " << deg2rad(l->q) << " intersect:" << (*it)->detectIntersection() <<  " check:" << (*it)->checkCollision() << std::endl;
		(*it)->clearCollisions();
	    }
	    std::cerr << "  pair (" << jointPath->numJoints() << ") " << (*it)->link(0)->name << "/" << (*it)->link(1)->name << (always_collide?" always collide":"") << (never_collide?" never collide":"") << std::endl;
	    it = tmp_pair.erase(it);
	    continue;
	}
	it++;
    }
    //
    std::cerr << "step 1: Remove collision pair if adjacent pair" << std::endl;
    it = tmp_pair.begin();
    while ( it != tmp_pair.end() ) {
	hrp::JointPathPtr jointPath = m_robot->getJointPath((*it)->link(0),(*it)->link(1));
	if ( jointPath->numJoints() == 2 || jointPath->numJoints() == 3 ) {
	    hrp::Link *l = jointPath->joint(0);
	    bool always_collide = true; // true if always collide
	    bool never_collide = true;  // true if never collide ( crrect )
	    for(float angle = l->llimit; angle <= l->ulimit; angle += deg2rad(5)) {
		l->q = angle;
		m_robot->calcForwardKinematics(false,false);
		m_robot->updateLinkColdetModelPositions();
		if ( (*it)->detectIntersection() ) {
		    never_collide = false;
		} else {
		    always_collide = false;
		}
		//std::cerr << l->name << " " << deg2rad(l->q) << " intersect:" << (*it)->detectIntersection() <<  " check:" << (*it)->checkCollision() << std::endl;
		(*it)->clearCollisions();
	    }
	    std::cerr << "  pair (" << jointPath->numJoints() << ") " << (*it)->link(0)->name << "/" << (*it)->link(1)->name << " always:" << always_collide << ", never:" << never_collide << " " << (( always_collide || never_collide )?"remove from list":"") << std::endl;
	    if ( always_collide || never_collide ) {
		it = tmp_pair.erase(it);
		continue;
	    } else {
		m_pair.push_back(*it);
	    }
	}
	it++;
    }
    std::cerr << "  Remove always/never collide pair for a length of 1,2,3 ... " << tmp_pair.size() << std::endl;


    // copy collisin pair from tmp_pair to m_pair if the pair has collide posture
    std::cerr << "step 3: Remove collision pair if they never collide" << std::endl;
    int i = 0;
    it = tmp_pair.begin();
    while ( it != tmp_pair.end() ) {
	hrp::JointPathPtr jointPath = m_robot->getJointPath((*it)->link(0),(*it)->link(1));
	bool collide = checkCollisionForAllJointRange(m_robot, 0, jointPath, *it);
	std::cerr << "  " << i << "/" << tmp_pair.size() << "  pair (" << jointPath->numJoints() << ") " <<  (*it)->link(0)->name << "/" << (*it)->link(1)->name << " collision:" << collide << std::endl;
	if ( collide ) {
	    m_pair.push_back(*it);
	}
	it++; i++;
    }
    std::cerr << "Reduced collision pair size " << m_pair.size() << std::endl;
}

int main (int argc, char** argv)
{
    std::string url;

    for (int i = 1; i < argc; ++ i) {
	std::string arg(argv[i]);
	coil::normalize(arg);
	if ( arg == "--model" ) {
	    if (++i < argc) url = argv[i];
	} else if ( arg == "-o" ) {
	    ++i;
	} else if ( arg[0] == '-' ||  arg[0] == '_'  ) {
	    std::cerr << argv[0] << " : Unknwon arguments " << arg << std::endl;
	} else {
	    blacklist.push_back(argv[i]);
	}
    }
    if (argc < 2) {
	std::cerr << "usage: " << argv[0] << " --model <model file> <blacklist : R_HAND_J0 L_HAND_J0 EYEBROW_P EYELID_P EYE_Y EYE_P MOUSE_P UPPERLIP_P LOWERLIP_P CHEEK_P>" << std::endl;
	exit(1);
    }

    hrp::BodyPtr m_robot = new hrp::Body();
    OpenHRP::BodyInfo_var binfo = hrp::loadBodyInfo(url.c_str(), argc, argv);
    if (CORBA::is_nil(binfo)){
        std::cerr << "failed to load model[" << url << "]" << std::endl;
        return 1;
    }
    
    if (!loadBodyFromBodyInfo(m_robot, binfo)) {
        std::cerr << "failed to load model[" << url << "]" << std::endl;
        return 1;
    }

    setupCollisionModel(m_robot, url.c_str(), binfo);
    setupCollisionLinkPair(m_robot);

    std::string fname = "/tmp/"+m_robot->name()+"collision_pair.conf";
    std::ofstream ofs(fname.c_str());
    ofs << "collision_pair:";
    for (std::vector<hrp::ColdetLinkPairPtr>::iterator it = m_pair.begin(); it != m_pair.end(); it++) {
	ofs << " " << (*it)->link(0)->name << ":" << (*it)->link(1)->name;
	std::cerr << (*it)->link(0)->name << ":" << (*it)->link(1)->name << std::endl;
    }
    ofs << std::endl;
    ofs.close();
    std::cerr << "Write collision pair conf file to " << fname << std::endl;

    return 0;
}
