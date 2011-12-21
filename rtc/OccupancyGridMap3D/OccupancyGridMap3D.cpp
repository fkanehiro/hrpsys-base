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
    "conf.default.scan", "0",
    "conf.default.occupiedThd", "0.5",
    "conf.default.resolution", "0.1",
    "conf.default.initialMap", "",
    "conf.default.debugLevel", "0",
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
  bindParameter("scan", m_scan, "0");
  bindParameter("occupiedThd", m_occupiedThd, "0.5");
  bindParameter("resolution", m_resolution, "0.1");
  bindParameter("initialMap", m_initialMap, "");
  bindParameter("debugLevel", m_debugLevel, "0");
  
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

  //RTC::Properties& prop = getProperties();

  char cwd[PATH_MAX];
  char *p = getcwd(cwd, PATH_MAX);
  m_cwd = p;
  m_cwd += "/";

  m_update.data = 1;

  m_pose.data.position.x = 0; 
  m_pose.data.position.y = 0; 
  m_pose.data.position.z = 1.5; 
  m_pose.data.orientation.r = 0;
  m_pose.data.orientation.p = M_PI/2;
  m_pose.data.orientation.y = 0;
 
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
    coil::TimeValue t1(coil::gettimeofday());

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
#ifdef USE_ONLY_GRIDS
    double res = m_map->getResolution();
#endif
    point3d sensorP(p[0], p[1], p[2]);
    if (m_cloudIn.isNew()){
        m_cloudIn.read();
        Guard guard(m_mutex);
        if (strcmp(m_cloud.type, "xyz")){
            std::cout << "point type(" << m_cloud.type 
                      << ") is not supported" << std::endl;
            return RTC::RTC_ERROR;
        }
        float *ptr = (float *)m_cloud.data.get_buffer();
        for (unsigned int i=0; i<m_cloud.height*m_cloud.width; i++, ptr+=4){
            if (isnan(ptr[0])) continue;
            relP[0] = ptr[0];
            relP[1] = ptr[1];
            relP[2] = ptr[2];
            //std::cout << "relP:" << relP << std::endl;

            absP = p + R*relP;
            //if (i%320==160)std::cout << i/320 << ", abs:(" << absP[0] << ", " << absP[1] << ", " << absP[2] << "), rel:(" << relP[0] << ", " << relP[1] << ", " << relP[2] << ")" << std::endl;
#ifdef USE_ONLY_GRIDS
            point3d p(((int)(absP[0]/res))*res,
                      ((int)(absP[1]/res))*res,
                      ((int)(absP[2]/res))*res);
#else
            point3d p(absP[0], absP[1], absP[2]);
#endif
            //printf("%4d:%6.3f %6.3f %6.3f\n", i, p.x(), p.y(), p.z());
            if (m_scan){
                m_map->insertRay(sensorP, p);
            }else{
                m_map->updateNode(p, true);
            }
        }
    }

    coil::TimeValue t2(coil::gettimeofday());
    if (m_debugLevel > 0){
        coil::TimeValue dt = t2-t1;
        std::cout << "OccupancyGridMap3D::onExecute() : " 
                  << dt.sec()*1e3+dt.usec()/1e3 << "[ms]" << std::endl;
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
#ifdef USE_ONLY_GRIDS
    map->pos.x = ((int)(region.pos.x/size))*size;
    map->pos.y = ((int)(region.pos.y/size))*size;
    map->pos.z = ((int)(region.pos.z/size))*size;
#else
    map->pos.x = region.pos.x;
    map->pos.y = region.pos.y;
    map->pos.z = region.pos.z;
#endif
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
                        double prob = result->getOccupancy();
                        if (prob >= m_occupiedThd){
                            map->cells[rank++] = prob*0xfe;
                            no++;
                        }else{
                            map->cells[rank++] = OpenHRP::gridEmpty;
                            ne++;
                        }
                        //printf("%6.3f %6.3f %6.3f:%d\n", p.x(), p.y(), p.z(),map->cells[rank-1]);
                    }else{
                        map->cells[rank++] = OpenHRP::gridUnknown;
                        nu++;
                    }
                }
            }
        }
#if 0
        std::cout << "occupied/empty/unknown = " << no << "/" << ne << "/" 
                  << nu << std::endl;
#endif
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


