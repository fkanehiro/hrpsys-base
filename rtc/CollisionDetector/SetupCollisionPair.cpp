// -*- C++ -*-
/*!
 * @file SetupCollisionPair.cpp
 * @brief Standalone component
 * @date $Date$
 *
 * $Id$
 */

#include <iomanip>
#include <hrpModel/Link.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpModel/JointPath.h>
#include <hrpUtil/Eigen3d.h>
#include <hrpUtil/Eigen4d.h>
#include <hrpCollision/ColdetModel.h>
#include "CollisionDetector.h"
extern "C" {
#include <qhull/qhull_a.h>
}

#define deg2rad(x)	((x)*M_PI/180)
#define rad2deg(x)      ((x)*180/M_PI)

std::vector<hrp::ColdetLinkPairPtr> m_pair;
std::vector<std::string> blacklist;
hrp::BodyPtr m_robot;

RTC::ReturnCode_t setupCollisionModel(hrp::BodyPtr m_robot, const char *url, OpenHRP::BodyInfo_var binfo) {
    // do qhull
    OpenHRP::ShapeInfoSequence_var sis = binfo->shapes();
    OpenHRP::LinkInfoSequence_var lis = binfo->links();
    for(unsigned int i = 0; i < m_robot->numLinks(); i++ ) {
	const OpenHRP::LinkInfo& i_li = lis[i];
	const OpenHRP::TransformedShapeIndexSequence& tsis = i_li.shapeIndices;
	// setup
	int numTriangles = 0;
	for (unsigned int l=0; l<tsis.length(); l++) {
	    OpenHRP::ShapeInfo& si = sis[tsis[l].shapeIndex];
	    const OpenHRP::LongSequence& triangles = si.triangles;
	    numTriangles += triangles.length();
	}
	double points[numTriangles*3];
	int points_i = 0;
	hrp::Matrix44 Rs44; // inv
	hrp::Matrix33 Rs33 = m_robot->link(i)->Rs;

	Rs44 << Rs33(0,0),Rs33(0,1), Rs33(0,2), 0,
	    Rs33(1,0),Rs33(1,1), Rs33(1,2), 0,
	    Rs33(2,0),Rs33(2,1), Rs33(2,2), 0,
	    0.0,      0.0,       0.0,    1.0;
	for (unsigned int l=0; l<tsis.length(); l++) {
	    const OpenHRP::DblArray12& M = tsis[l].transformMatrix;
	    hrp::Matrix44 T0;
	    T0 << M[0], M[1], M[2],  M[3],
		M[4], M[5], M[6],  M[7],
		M[8], M[9], M[10], M[11],
		0.0,  0.0,  0.0,   1.0;
	    hrp::Matrix44 T(Rs44 * T0);
	    const OpenHRP::ShapeInfo& si = sis[tsis[l].shapeIndex];
	    const OpenHRP::LongSequence& triangles = si.triangles;
	    const float *vertices = si.vertices.get_buffer();

	    for(unsigned int j=0; j < triangles.length() / 3; ++j){
		for(int k=0; k < 3; ++k){
		    long orgVertexIndex = si.triangles[j * 3 + k];
		    int p = orgVertexIndex * 3;

		    hrp::Vector4 v(T * hrp::Vector4(vertices[p+0], vertices[p+1], vertices[p+2], 1.0));
		    points[points_i++] =  v[0];
		    points[points_i++] =  v[1];
		    points[points_i++] =  v[2];
		}
	    }
	}

	hrp::ColdetModelPtr coldetModel(new hrp::ColdetModel());
	coldetModel->setName(std::string(i_li.name));
	// qhull
	int vertexIndex = 0;
	int triangleIndex = 0;
	int num = 0;
	char flags[250];
	boolT ismalloc = False;
	sprintf(flags,"qhull Qt Tc");
	if (! qh_new_qhull (3,numTriangles,points,ismalloc,flags,NULL,stderr) ) {

	    qh_triangulate();
	    qh_vertexneighbors();

	    vertexT *vertex,**vertexp;
	    coldetModel->setNumVertices(qh num_vertices);
	    coldetModel->setNumTriangles(qh num_facets);
	    int index[qh num_vertices];
	    FORALLvertices {
		int p = qh_pointid(vertex->point);
		index[p] = vertexIndex;
		coldetModel->setVertex(vertexIndex++, points[p*3+0], points[p*3+1], points[p*3+2]);
	    }
	    facetT *facet;
	    num = qh num_facets;;
	    {
		FORALLfacets {
		    int j = 0, p[3];
		    setT *vertices = qh_facet3vertex (facet);
		    FOREACHvertexreverse12_ (vertices) {
			if (j<3) {
			    p[j] = index[qh_pointid(vertex->point)];
			} else {
			    fprintf(stderr, "extra vertex %d\n",j);
			}
			j++;
		    }
		    coldetModel->setTriangle(triangleIndex++, p[0], p[1], p[2]);
		}
	    }
	} // qh_new_qhull

        coldetModel->build();
	m_robot->link(i)->coldetModel =  coldetModel;

	qh_freeqhull(!qh_ALL);
	int curlong, totlong;
	qh_memfreeshort (&curlong, &totlong);
	if (curlong || totlong) {
	    fprintf(stderr, "convhulln: did not free %d bytes of long memory (%d pieces)\n", totlong, curlong);
	}
	//
	std::cerr << std::setw(16) << i_li.name << " reduce triangles from " << std::setw(5) << numTriangles << " to " << std::setw(4) << num << std::endl;
    }

    return RTC::RTC_OK;
}

bool checkCollisionForAllJointRange(int i, hrp::JointPathPtr jointPath, std::vector<hrp::ColdetLinkPairPtr> &collisionPairs)
{
    if ( i >= (int)jointPath->numJoints() ) return false;
    if ( collisionPairs.size() == 0 ) return true;
    hrp::Link *l = jointPath->joint(i);

    if ( l->jointType == hrp::Link::FIXED_JOINT ) {
	checkCollisionForAllJointRange(i+1, jointPath, collisionPairs);
    } else {
	double step = (l->ulimit - l->llimit)/2;
	for(float angle = l->llimit; angle <= l->ulimit; angle += step)
	{
	    l->q = angle;
	    m_robot->calcForwardKinematics(false,false);
	    m_robot->updateLinkColdetModelPositions();

	    //std::cerr << "    " << collisionPair->link(0)->name << ":" << collisionPair->link(1)->name << " ";
	    //for(int j = 0; j < jointPath->numJoints(); j++) std::cerr << rad2deg(jointPath->joint(j)->q) << " ";
	    // std::cerr << "id = " << i << "/" << jointPath->numJoints() << ", " << l->name << " " << rad2deg(angle) << " (" << rad2deg(l->llimit) << "," << rad2deg(l->ulimit) << ") " << collisionPair->detectIntersection() << " " << collisionPair->link(0)->name << ":" << collisionPair->link(1)->name << std::endl;
	    for(std::vector<hrp::ColdetLinkPairPtr>::iterator it = collisionPairs.begin(); it != collisionPairs.end(); ) {
		if ( (*it)->detectIntersection() ) {
		    std::cerr << "Find collision for " << (*it)->link(0)->name << " " << (*it)->link(1)->name << std::endl;
		    it = collisionPairs.erase(it);
		    continue;
		}
		it++;
	    }
	    checkCollisionForAllJointRange(i+1, jointPath, collisionPairs);
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

bool checkBlackListJointPair(hrp::Link *l1, hrp::Link *l2) {
    for(std::vector<std::string>::iterator it = blacklist.begin(); it != blacklist.end(); it++ ) {
	    if ((l1->name + ":" + l2->name) == (*it) ||
	        (l2->name + ":" + l1->name) == (*it)) return true;
    }
    return false;
}

bool compare_joint_path_length(const hrp::ColdetLinkPairPtr& p1, const hrp::ColdetLinkPairPtr& p2) {
    return (m_robot->getJointPath(p1->link(0),p1->link(1))->numJoints()) > (m_robot->getJointPath(p2->link(0),p2->link(1))->numJoints());
}

void setupCollisionLinkPair()
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
    for (unsigned int i=0; i<m_robot->numLinks(); i++) {
	hrp::Link *l1 = m_robot->link(i);
	for (unsigned int j=i+1; j<m_robot->numLinks(); j++) {
	    hrp::Link *l2 = m_robot->link(j);
	    if ( l1->coldetModel && l2->coldetModel
		 &&  (!(checkBlackListJoint(l1) ||  checkBlackListJoint(l2) || checkBlackListJointPair(l1, l2)))
		) {
		tmp_pair.push_back(new hrp::ColdetLinkPair(l1, l2));
	    }
	}
    }

    std::cerr << "Initial collision pair size " << tmp_pair.size() << std::endl;
    std::vector<hrp::ColdetLinkPairPtr>::iterator it;
    // Remove collision pair if the pair always collides
    std::cerr << "step 0: Remove collision pair if they are adjacent pair" << std::endl;
    it = tmp_pair.begin();
    while ( it != tmp_pair.end() ) {
	hrp::JointPathPtr jointPath = m_robot->getJointPath((*it)->link(0),(*it)->link(1));
	if ( jointPath->numJoints() == 1 ) {
	    hrp::Link *l = jointPath->joint(0);
	    bool always_collide = true; // true if always collide
	    bool never_collide = true;  // true if never collide ( crrect )
            l->llimit = -M_PI;l->ulimit = M_PI;
            std::cerr << l->name << " ulimit - " << l->llimit << " " << l->ulimit << std::endl;
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
    std::cerr << "step 1: Remove complex (2 or 3 distance) collision pair if they never/always collide" << std::endl;
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

    // setup data structure for check collision pair
    // pair_tree[<pair:L1,L2,L3,L4>, [ <pair:L1,L2,L3,L4>, <pair:L1,L2,L3>, <pair:L1,L2> ]]
    // pair_tree[<pair:L2,L3,L4,L5>, [ <pair:L2,L3,L4,L5>, <pair:L2,L3,L4>, <pair:L2,L3> ]]
    std::map<hrp::ColdetLinkPairPtr, std::vector<hrp::ColdetLinkPairPtr> > pair_tree;
    std::sort(tmp_pair.begin(), tmp_pair.end(), compare_joint_path_length);
    int i = 0;
    it = tmp_pair.begin();
    while ( it != tmp_pair.end() ) {
	hrp::JointPathPtr jointPath1 = m_robot->getJointPath((*it)->link(0),(*it)->link(1));
	//std::cerr << "  " << i << "/" << tmp_pair.size() << "  pair (" << jointPath1->numJoints() << ") " << (*it)->link(0)->name << " " << (*it)->link(1)->name <<std::endl;
	// check if JointPath1 is included in some of the pair_tree (jointPath2)
	bool is_new_key = true;
	for (std::map<hrp::ColdetLinkPairPtr, std::vector<hrp::ColdetLinkPairPtr> >::iterator ii=pair_tree.begin(); ii != pair_tree.end(); ++ii) {
	    hrp::JointPathPtr jointPath2 = m_robot->getJointPath(((*ii).first)->link(0),((*ii).first)->link(1));
	    // check if JointPath1 is included in jointPath2
	    bool find_key = true;
	    for (unsigned int j = 0; j < jointPath1->numJoints() ; j++ ) {
		if ( jointPath1->joint(j)->name != jointPath2->joint(j)->name ) {
		    find_key = false;
		    break;
		}
	    }
	    if ( find_key ) {
		(*ii).second.push_back(*it);
		is_new_key = false;
	    }
	}
	if (is_new_key) {
	    pair_tree[*it] = std::vector<hrp::ColdetLinkPairPtr>(1,*it)
;
	}
	it++; i++;
    }
    // remove non-collisin pair from tmp_pair
    std::cerr << "step 2: Remove collision pair if they never collide" << std::endl;
    i = 0;
    for (std::map<hrp::ColdetLinkPairPtr, std::vector<hrp::ColdetLinkPairPtr> >::iterator ii=pair_tree.begin(); ii != pair_tree.end(); ++ii) {
	hrp::ColdetLinkPairPtr key_pair = (*ii).first;
	std::vector<hrp::ColdetLinkPairPtr> sub_pairs = (*ii).second;
	hrp::JointPathPtr jointPath = m_robot->getJointPath(key_pair->link(0),key_pair->link(1));
	std::sort(sub_pairs.begin(), sub_pairs.end(), compare_joint_path_length);
	//
	std::cerr << "  " << i << "/" << tmp_pair.size() << "  pair (" << jointPath->numJoints() << ") " << (*ii).first->link(0)->name << " " << (*ii).first->link(1)->name <<std::endl;
	i += (*ii).second.size();
	for(std::vector<hrp::ColdetLinkPairPtr>::iterator it = sub_pairs.begin(); it != sub_pairs.end(); it++ ) {
	    hrp::JointPathPtr jointPath = m_robot->getJointPath((*it)->link(0),(*it)->link(1));
	    for ( unsigned int j = 0; j < jointPath->numJoints(); j++ ) {
		std::cerr << jointPath->joint(j)->name << " ";
	    }
	    std::cerr << std::endl;
	}
	// rmeove non-collision pair from sub_paris
	checkCollisionForAllJointRange(0, jointPath, sub_pairs);
	for(std::vector<hrp::ColdetLinkPairPtr>::iterator p = sub_pairs.begin(); p != sub_pairs.end(); p++ ) {
	    for(std::vector<hrp::ColdetLinkPairPtr>::iterator it = tmp_pair.begin(); it != tmp_pair.end(); ) {
		if ( (*p)->link(0) == (*it)->link(0) &&  (*p)->link(1) == (*it)->link(1) ) {
		    tmp_pair.erase(it);
		    continue;
		}
		it++;
	    }
	    //tmp_pair.erase(it); // does not work???
	}
    }
    it = tmp_pair.begin();
    while ( it != tmp_pair.end() ) {
	m_pair.push_back(*it);
	it++;
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
	std::cerr << "usage: " << argv[0] << " --model <model file> <blacklist : R_HAND_J0 L_HAND_J0 EYEBROW_P EYELID_P EYE_Y EYE_P MOUTH_P UPPERLIP_P LOWERLIP_P CHEEK_P>" << std::endl;
	exit(1);
    }

    m_robot = hrp::BodyPtr(new hrp::Body());
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
    setupCollisionLinkPair();

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
