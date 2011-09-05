// -*- C++ -*-
/*!
 * @file  OccupancyGridMap3D.cpp
 * @brief 3d occupancy grid map
 * $Date$
 *
 * $Id$
 */

#include "OccupancyGridMap3D.h"
#include "hrpUtil/Tvmet3d.h"
#include <octomap/octomap.h>
#include <octomap/OcTreeLabeled.h>

typedef coil::Guard<coil::Mutex> Guard;

using namespace octomap;

// Module specification
// <rtc-template block="module_spec">
static const char* occupancygridmap3d_spec[] =
  {
    "implementation_id", "OccupancyGridMap3D",
    "type_name",         "OccupancyGridMap3D",
    "description",       "null component",
    "version",           "1.0",
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.accumulate", "1",
    "conf.default.occupiedThd", "0.5",
    "conf.default.resolution", "0.1",
    "conf.default.initialMap", "",
    ""
  };
// </rtc-template>

OccupancyGridMap3D::OccupancyGridMap3D(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_cloudIn("cloud", m_cloud),
    m_poseIn("pose", m_pose),
    m_updateIn("update", m_update),
    m_OGMap3DServicePort("OGMap3DService"),
    // </rtc-template>
    m_service0(this),
    m_map(NULL),
    m_accumulate(true),
    dummy(0)
{
}

OccupancyGridMap3D::~OccupancyGridMap3D()
{
}



RTC::ReturnCode_t OccupancyGridMap3D::onInitialize()
{
  std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("accumulate", m_accumulate, "1");
  bindParameter("occupiedThd", m_occupiedThd, "0.5");
  bindParameter("resolution", m_resolution, "0.1");
  bindParameter("initialMap", m_initialMap, "");
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("cloud", m_cloudIn);
  addInPort("pose", m_poseIn);
  addInPort("update", m_updateIn);

  // Set OutPort buffer
  
  // Set service provider to Ports
  m_OGMap3DServicePort.registerProvider("service1", "OGMap3DService", m_service0);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_OGMap3DServicePort);
  
  // </rtc-template>

  RTC::Properties& prop = getProperties();

  char cwd[PATH_MAX];
  char *p = getcwd(cwd, PATH_MAX);
  m_cwd = p;
  m_cwd += "/";

  m_update.data = 1;
 
  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t OccupancyGridMap3D::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t OccupancyGridMap3D::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t OccupancyGridMap3D::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t OccupancyGridMap3D::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;

  if (m_initialMap != ""){
    // Working directories of threads which calls onInitialize() and onActivate() are different on MacOS
    // Assume path of initial map is given by a relative path to working directory of the thread which calls onInitialize()
    m_map = new OcTree(m_cwd+m_initialMap);
  }else{
    m_map = new OcTree(m_resolution);
  }

  return RTC::RTC_OK;
}

RTC::ReturnCode_t OccupancyGridMap3D::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
  delete m_map;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t OccupancyGridMap3D::onExecute(RTC::UniqueId ec_id)
{
    //std::cout << "OccupancyGrid3D::onExecute(" << ec_id << ")" << std::endl;
    if (m_cloudIn.isNew())   m_cloudIn.read();
    if (m_poseIn.isNew())    m_poseIn.read();
    if (m_updateIn.isNew())  m_updateIn.read();

    if (!m_update.data) {
        return RTC::RTC_OK;
    }

    hrp::Matrix33 R;
    hrp::Vector3 p;
    p[0] = m_pose.data.position.x; 
    p[1] = m_pose.data.position.y; 
    p[2] = m_pose.data.position.z; 
    R = hrp::rotFromRpy(m_pose.data.orientation.r,
                        m_pose.data.orientation.p,
                        m_pose.data.orientation.y);
    //std::cout << "p:" << p << std::endl;
    //std::cout << "R:" << R << std::endl;

    hrp::Vector3 absP, relP; 
    double res = m_map->getResolution();
    point3d sensorP(p[0], p[1], p[2]);
    {
        Guard guard(m_mutex);
        for (unsigned int i=0; i<m_cloud.points.length(); i++){
            relP[0] = m_cloud.points[i].point.x;
            relP[1] = m_cloud.points[i].point.y;
            relP[2] = m_cloud.points[i].point.z;
            absP = p + R*relP;
            //if (i%320==160)std::cout << i/320 << ", abs:(" << absP[0] << ", " << absP[1] << ", " << absP[2] << "), rel:(" << relP[0] << ", " << relP[1] << ", " << relP[2] << ")" << std::endl;
            point3d p(((int)(absP[0]/res))*res,
                      ((int)(absP[1]/res))*res,
                      ((int)(absP[2]/res))*res);
            if (m_accumulate){
                m_map->updateNode(p, true);
            }else{
                m_map->insertRay(sensorP, p);
            }
        }
    }

    return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t OccupancyGridMap3D::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t OccupancyGridMap3D::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t OccupancyGridMap3D::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t OccupancyGridMap3D::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t OccupancyGridMap3D::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

OpenHRP::OGMap3D* OccupancyGridMap3D::getOGMap3D(const OpenHRP::AABB& region)
{
    OpenHRP::OGMap3D *map = new OpenHRP::OGMap3D;
    double size = m_map->getResolution();
    map->resolution = size;
    map->pos.x = ((int)(region.pos.x/size))*size+size/2;
    map->pos.y = ((int)(region.pos.y/size))*size+size/2;
    map->pos.z = ((int)(region.pos.z/size))*size+size/2;
    map->nx = region.size.l/size;
    map->ny = region.size.w/size;
    map->nz = region.size.h/size;
    int rank=0;
    point3d p;
    map->cells.length(map->nx*map->ny*map->nz);
    {
        int no=0, ne=0, nu=0;
        Guard guard(m_mutex);
        for (int i=0; i<map->nx; i++){
            p.x() = map->pos.x + i*size;
            for (int j=0; j<map->ny; j++){
                p.y() = map->pos.y + j*size;
                for (int k=0; k<map->nz; k++){
                    p.z() = map->pos.z + k*size;
                    OcTreeNode *result = m_map->search(p);
                    if (result){
                        if (result->getOccupancy() > m_occupiedThd){
                            map->cells[rank++] = OpenHRP::gridOccupied;
                            no++;
                        }else{
                            map->cells[rank++] = OpenHRP::gridEmpty;
                            ne++;
                        }
                    }else{
                        map->cells[rank++] = OpenHRP::gridUnknown;
                        nu++;
                    }
                }
            }
        }
        std::cout << "occupied/empty/unknown = " << no << "/" << ne << "/" 
                  << nu << std::endl;
    }
    return map;
}

extern "C"
{

  void OccupancyGridMap3DInit(RTC::Manager* manager)
  {
    RTC::Properties profile(occupancygridmap3d_spec);
    manager->registerFactory(profile,
                             RTC::Create<OccupancyGridMap3D>,
                             RTC::Delete<OccupancyGridMap3D>);
  }

};


