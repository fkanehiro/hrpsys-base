// -*- C++ -*-
/*!
 * @file  CollisionDetector.cpp
 * @brief collisoin detector component
 * $Date$
 *
 * $Id$
 */

#include <rtm/CorbaNaming.h>
#include <hrpModel/Link.h>
#include <hrpModel/JointPath.h>
#include <hrpUtil/Tvmet3d.h>
#include <hrpUtil/Tvmet4d.h>
#include <hrpCollision/ColdetModel.h>

#include "IrrModel.h"
#include "CollisionDetector.h"

extern "C" {
#include <qhull/qhull_a.h>
}
#define deg2rad(x)	((x)*M_PI/180)
#define rad2deg(x)      ((x)*180/M_PI)

// Module specification
// <rtc-template block="module_spec">
static const char* component_spec[] =
{
    "implementation_id", "CollisionDetector",
    "type_name",         "CollisionDetector",
    "description",       "collisoin detector component",
    "version",           "1.0",
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.project", "",
    ""
};
// </rtc-template>

CollisionDetector::CollisionDetector(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      // <rtc-template block="initializer">
      m_qIn("q", m_q),
      // </rtc-template>
      m_robot(NULL),
      dummy(0)
{
    m_scene = new GLscene();
}

CollisionDetector::~CollisionDetector()
{
}



RTC::ReturnCode_t CollisionDetector::onInitialize()
{
    std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
    // <rtc-template block="bind_config">
    // Bind variables and configuration variable
  
    // </rtc-template>

    // Registration: InPort/OutPort/Service
    // <rtc-template block="registration">
    // Set InPort buffers
    addInPort("q", m_qIn);

    // Set OutPort buffer
  
    // Set service provider to Ports
  
    // Set service consumers to Ports
  
    // Set CORBA Service Ports
  
    // </rtc-template>

    //RTC::Properties& prop = getProperties();

    return RTC::RTC_OK;
}



/*
  RTC::ReturnCode_t CollisionDetector::onFinalize()
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t CollisionDetector::onStartup(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t CollisionDetector::onShutdown(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

RTC::ReturnCode_t setupCollisionModel(hrp::BodyPtr m_robot, const char *url, OpenHRP::BodyInfo_var binfo) {
    // do qhull
    OpenHRP::ShapeInfoSequence_var sis = binfo->shapes();
    OpenHRP::LinkInfoSequence_var lis = binfo->links();
    for(int i = 0; i < m_robot->numLinks(); i++ ) {
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

	Rs44 = Rs33(0,0),Rs33(0,1), Rs33(0,2), 0,
	    Rs33(1,0),Rs33(1,1), Rs33(1,2), 0,
	    Rs33(2,0),Rs33(2,1), Rs33(2,2), 0,
	    0.0,      0.0,       0.0,    1.0;
	for (unsigned int l=0; l<tsis.length(); l++) {
	    const OpenHRP::DblArray12& M = tsis[l].transformMatrix;
	    hrp::Matrix44 T0;
	    T0 = M[0], M[1], M[2],  M[3],
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
	coldetModel->setName(i_li.name);
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
	    coldetModel->initNeighbor(numTriangles);
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
		    coldetModel->setTriangle(triangleIndex, p[0], p[1], p[2]);
		    coldetModel->setNeighborTriangle(triangleIndex++, p[0], p[1], p[2]);
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

RTC::ReturnCode_t CollisionDetector::onActivated(RTC::UniqueId ec_id)
{
    std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;

    RTC::Manager& rtcManager = RTC::Manager::instance();
    std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
    int comPos = nameServer.find(",");
    if (comPos < 0){
        comPos = nameServer.length();
    }
    nameServer = nameServer.substr(0, comPos);
    RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());

    m_scene->init();

    RTC::Properties& prop = getProperties();

    m_robot = new hrp::Body();
    OpenHRP::BodyInfo_var binfo;
    binfo = hrp::loadBodyInfo(prop["model"].c_str(),
			      CosNaming::NamingContext::_duplicate(naming.getRootContext()));
    if (CORBA::is_nil(binfo)){
	std::cerr << "failed to load model[" << prop["model"] << "]"
		  << std::endl;
	return RTC::RTC_ERROR;
    }
    if (!loadBodyFromBodyInfo(m_robot, binfo)) {
	std::cerr << "failed to load model[" << prop["model"] << "]" << std::endl;
	return RTC::RTC_ERROR;
    }
    setupCollisionModel(m_robot, prop["model"].c_str(), binfo);

    if ( prop["collision_pair"] != "" ) {
	std::cerr << "prop[collision_pair] ->" << prop["collision_pair"] << std::endl;
	std::istringstream iss(prop["collision_pair"]);
	std::string tmp;
	while (getline(iss, tmp, ' ')) {
	    size_t pos = tmp.find_first_of(':');
	    std::string name1 = tmp.substr(0, pos), name2 = tmp.substr(pos+1);
	    std::cerr << "check collisions between " << m_robot->link(name1)->name << " and " <<  m_robot->link(name2)->name << std::endl;
	    m_pair.push_back(new hrp::ColdetLinkPair(m_robot->link(name1), m_robot->link(name2)));
	}
    }

    if ( m_pair.size() == 0 ) {
	std::cerr << "failed to setup collisions" << std::endl;
	return RTC::RTC_ERROR;
    }

    m_body = m_scene->addBody(m_robot);
    m_scene->draw();

    return RTC::RTC_OK;
}

RTC::ReturnCode_t CollisionDetector::onDeactivated(RTC::UniqueId ec_id)
{
    std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;

    delete m_body;
    delete m_scene;

    return RTC::RTC_OK;
}

RTC::ReturnCode_t CollisionDetector::onExecute(RTC::UniqueId ec_id)
{

    if (m_qIn.isNew()) {
	m_qIn.read();

	assert(m_q.data.length() == m_robot->numJoints());
	double posture[m_q.data.length()];
	for ( int i = 0; i < m_robot->numJoints(); i++ ){
	    m_robot->joint(i)->q = posture[i] = m_q.data[i];
	}
	m_robot->calcForwardKinematics();
	m_robot->updateLinkColdetModelPositions();
	m_body->setPosture(posture);

	coil::TimeValue tm1 = coil::gettimeofday();
	for (unsigned int i = 0; i < m_pair.size(); i++){
	    hrp::ColdetLinkPairPtr p = m_pair[i];
	    double point0[3], point1[3];
	    double d = p->computeDistance(point0, point1);
	    if ( d <= 0.05 ) {
		hrp::JointPathPtr jointPath = m_robot->getJointPath(p->link(0),p->link(1));
		std::cerr << i << "/" << m_pair.size() << " pair: " << p->link(0)->name << "/" << p->link(1)->name << "(" << jointPath->numJoints() << "), distance = " << d << std::endl;
	    }
	}
	coil::TimeValue tm2 = coil::gettimeofday();
	std::cerr << "check collisions for for " << m_pair.size() << " pairs in " << (tm2.sec()-tm1.sec())*1000+(tm2.usec()-tm1.usec())/1000 << "[msec]" << std::endl;

    }

    m_scene->draw();

    return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t CollisionDetector::onAborting(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t CollisionDetector::onError(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t CollisionDetector::onReset(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t CollisionDetector::onStateUpdate(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t CollisionDetector::onRateChanged(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/



extern "C"
{

    void CollisionDetectorInit(RTC::Manager* manager)
    {
        RTC::Properties profile(component_spec);
        manager->registerFactory(profile,
                                 RTC::Create<CollisionDetector>,
                                 RTC::Delete<CollisionDetector>);
    }

};


