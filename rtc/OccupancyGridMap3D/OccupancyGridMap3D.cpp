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

#define KDEBUG 0

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
    m_updateOut("updateSignal", m_updateSignal),
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
  addOutPort("updateSignal", m_updateOut);
  
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
  m_pose.data.orientation.r = -M_PI/2;
  m_pose.data.orientation.p = 0;
  m_pose.data.orientation.y = 0;

  m_updateSignal.data = 0;
 
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
    m_updateOut.write();
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

    if (m_updateIn.isNew())  m_updateIn.read();

    if (!m_update.data) {
        // suspend updating map
        while (m_poseIn.isNew()) m_poseIn.read();
        while (m_cloudIn.isNew()) m_cloudIn.read();
        return RTC::RTC_OK;
    }

    if (m_cloudIn.isNew()){
        do{
            m_cloudIn.read();
            if (m_poseIn.isNew()) m_poseIn.read();
            Guard guard(m_mutex);
            float *ptr = (float *)m_cloud.data.get_buffer();
            if (strcmp(m_cloud.type, "xyz")==0 
                || strcmp(m_cloud.type, "xyzrgb")==0){
                Pointcloud cloud;
                for (unsigned int i=0; i<m_cloud.data.length()/16; i++, ptr+=4){
                    if (isnan(ptr[0])) continue;
                    cloud.push_back(point3d(ptr[0],ptr[1],ptr[2]));
                }
                point3d sensor(0,0,0);
                pose6d frame(m_pose.data.position.x,
                             m_pose.data.position.y,
                             m_pose.data.position.z, 
                             m_pose.data.orientation.r,
                             m_pose.data.orientation.p,
                             m_pose.data.orientation.y);
                m_map->insertScan(cloud, sensor, frame);
            }else if (strcmp(m_cloud.type, "xyzv")==0){
                hrp::Matrix33 R;
                hrp::Vector3 p;
                p[0] = m_pose.data.position.x; 
                p[1] = m_pose.data.position.y; 
                p[2] = m_pose.data.position.z; 
                R = hrp::rotFromRpy(m_pose.data.orientation.r,
                                    m_pose.data.orientation.p,
                                    m_pose.data.orientation.y);
                int ocnum = 0;
                int emnum = 0;
                for (unsigned int i=0; i<m_cloud.data.length()/16; i++, ptr+=4){
                    if (isnan(ptr[0])) continue;
                    hrp::Vector3 peye(ptr[0],ptr[1],ptr[2]);
                    hrp::Vector3 pworld(R*peye+p);
                    point3d pog(pworld[0],pworld[1],pworld[2]);
                    m_map->updateNode(pog, ptr[3]>0.0?true:false, false);
                    ptr[3]>0.0?ocnum++:emnum++;
                }
                if(KDEBUG) std::cout << m_profile.instance_name << ": " << ocnum << " " << emnum << " " << p << std::endl;
            }else{
                std::cout << "point type(" << m_cloud.type 
                          << ") is not supported" << std::endl;
                return RTC::RTC_ERROR;
            }
        }while(m_cloudIn.isNew());
        m_updateOut.write();
    }
    while(m_poseIn.isNew()) m_poseIn.read();

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

    double min[3];
    m_map->getMetricMin(min[0],min[1],min[2]);
    double max[3];
    m_map->getMetricMax(max[0],max[1],max[2]);
    for (int i=0; i<3; i++){
        min[i] -= size; 
        max[i] += size; 
    }
    double s[3];
    s[0] = region.pos.x;
    s[1] = region.pos.y;
    s[2] = region.pos.z;
    double e[3];
    e[0] = region.pos.x + region.size.l;
    e[1] = region.pos.y + region.size.w;
    e[2] = region.pos.z + region.size.h;
    double l[3];
    
    for (int i=0; i<3; i++){
        if (e[i] < min[i] || s[i] > max[i]){ // no overlap
            s[i] = e[i] = 0;
        }else{
            if (s[i] < min[i]) s[i] = min[i];
            if (e[i] > max[i]) e[i] = max[i];
        } 
        l[i] = e[i] - s[i];
    }

#ifdef USE_ONLY_GRIDS
    map->pos.x = ((int)(s[0]/size))*size;
    map->pos.y = ((int)(s[1]/size))*size;
    map->pos.z = ((int)(s[2]/size))*size;
#else
    map->pos.x = s[0];
    map->pos.y = s[1];
    map->pos.z = s[2];
#endif
    map->nx = l[0]/size;
    map->ny = l[1]/size;
    map->nz = l[2]/size;
    int rank=0;
    point3d p;
    map->cells.length(map->nx*map->ny*map->nz);
    {
        int no=0, ne=0, nu=0;
        //Guard guard(m_mutex);
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

void OccupancyGridMap3D::save(const char *filename)
{
    Guard guard(m_mutex);
    m_map->writeBinary(filename);
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


