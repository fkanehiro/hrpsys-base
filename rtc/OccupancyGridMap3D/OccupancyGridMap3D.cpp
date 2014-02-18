// -*- C++ -*-
/*!
 * @file  OccupancyGridMap3D.cpp
 * @brief 3d occupancy grid map
 * $Date$
 *
 * $Id$
 */

#include "OccupancyGridMap3D.h"
#include "hrpUtil/Eigen3d.h"
#include <octomap/octomap.h>

#define KDEBUG 0
//#define KDEBUG 1 // 121022

#ifdef __APPLE__
inline bool isnan(double x)
{
  return (x != x);
}
#endif

typedef coil::Guard<coil::Mutex> Guard;

using namespace octomap;

// Module specification
// <rtc-template block="module_spec">
static const char* occupancygridmap3d_spec[] =
  {
    "implementation_id", "OccupancyGridMap3D",
    "type_name",         "OccupancyGridMap3D",
    "description",       "null component",
    "version",           HRPSYS_PACKAGE_VERSION,
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
    "conf.default.knownMap", "",
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
    m_knownMap(NULL),
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
  bindParameter("knownMap", m_knownMapPath, "");
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

  if (m_knownMapPath != ""){
      m_knownMap = new OcTree(m_cwd+m_knownMapPath);
      m_updateOut.write();
  }

  if (m_initialMap != ""){
    // Working directories of threads which calls onInitialize() and onActivate() are different on MacOS
    // Assume path of initial map is given by a relative path to working directory of the thread which calls onInitialize()
    m_map = new OcTree(m_cwd+m_initialMap);
    m_updateOut.write();
  }else{
    m_map = new OcTree(m_resolution);
  }

  if(KDEBUG){
    std::cout << m_profile.instance_name << ": initial tree depth = " << m_map->getTreeDepth() << std::endl;
    std::cout << m_profile.instance_name << ": initial tree size = " << m_map->size() << std::endl;
    point3d p;
    //    p.x() = 0.1; //0.05;
    //    p.y() = 2.2; //2.15;
    //    p.z() = 1.4; //1.45;
    p.x() = -0.5; //-0.55;
    p.y() = 3.0; //2.95;
    p.z() = 0.5; //0.55;
    OcTreeNode *result = m_map->search(p);
    if (result){
      double prob = result->getOccupancy();
      std::cout << m_profile.instance_name << ": initial " << p << " " << prob << std::endl;
    std::cout << m_profile.instance_name << ": second tree depth = " << m_map->getTreeDepth() << std::endl;
    std::cout << m_profile.instance_name << ": second tree size = " << m_map->size() << std::endl;
    }
  }

  return RTC::RTC_OK;
}

RTC::ReturnCode_t OccupancyGridMap3D::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
  delete m_map;
  if (m_knownMap) delete m_knownMap;
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
        while (m_cloudIn.isNew()) m_cloudIn.read();
        while (m_poseIn.isNew())  m_poseIn.read();
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
#if KDEBUG
	    point3d pp1,pp2,pp3,pp4,pp5;
	    //	    pp1.x() = 0.1; //0.05;
	    //	    pp1.y() = 2.2; //2.15;
	    //	    pp1.z() = 1.4; //1.45;
	    pp1.x() = -0.35;
	    pp1.y() = 2.9;
	    pp1.z() = 0.5;
	    pp2.x() = -0.45; //-0.45;
	    pp2.y() = 2.9; //2.95;
	    pp2.z() = 0.5; //0.55;
	    pp3.x() = -0.55; //-0.55;
	    pp3.y() = 2.9; //2.95;
	    pp3.z() = 0.5; //0.55;
	    pp4.x() = -0.65; //-0.45;
	    pp4.y() = 2.9; //3.05;
	    pp4.z() = 0.5; //0.55;
	    pp5.x() = -0.75; //-0.55;
	    pp5.y() = 2.9; //3.05;
	    pp5.z() = 0.5; //0.55;
	    OcTreeNode *result1 = m_map->search(pp1);
	    OcTreeKey result1_key;
	    m_map->genKey(pp1, result1_key);
	    OcTreeNode *result2 = m_map->search(pp2);
	    OcTreeKey result2_key;
	    m_map->genKey(pp2, result2_key);
	    OcTreeNode *result3 = m_map->search(pp3);
	    OcTreeKey result3_key;
	    m_map->genKey(pp3, result3_key);
	    OcTreeNode *result4 = m_map->search(pp4);
	    OcTreeKey result4_key;
	    m_map->genKey(pp4, result4_key);
	    OcTreeNode *result5 = m_map->search(pp5);
	    OcTreeKey result5_key;
	    m_map->genKey(pp5, result5_key);
	    double prob1,prob2,prob3,prob4,prob5;
	    if (result1){
	      prob1 = result1->getOccupancy();
	      std::cout << m_profile.instance_name << ": " << pp1 << " original prob = " << prob1 << std::endl;
	      std::cout << m_profile.instance_name << ": " << pp1 << " key = " << result1_key[0] << " " << result1_key[1] << " " << result1_key[2] << std::endl;
	    }
	    else
	      std::cout << m_profile.instance_name << ": " << pp1 << " can not be searched." << std::endl;
	    if (result2){
	      prob2 = result2->getOccupancy();
	      std::cout << m_profile.instance_name << ": " << pp2 << " original prob = " << prob2 << std::endl;
	      std::cout << m_profile.instance_name << ": " << pp2 << " key = " << result2_key[0] << " " << result2_key[1] << " " << result2_key[2] << std::endl;
	    }
	    else
	      std::cout << m_profile.instance_name << ": " << pp2 << " can not be searched." << std::endl;
	    if (result3){
	      prob3 = result3->getOccupancy();
	      std::cout << m_profile.instance_name << ": " << pp3 << " original prob = " << prob3 << std::endl;
	      std::cout << m_profile.instance_name << ": " << pp3 << " key = " << result3_key[0] << " " << result3_key[1] << " " << result3_key[2] << std::endl;
	    }
	    else
	      std::cout << m_profile.instance_name << ": " << pp3 << " can not be searched." << std::endl;
	    if (result4){
	      prob4 = result4->getOccupancy();
	      std::cout << m_profile.instance_name << ": " << pp4 << " original prob = " << prob4 << std::endl;
	      std::cout << m_profile.instance_name << ": " << pp4 << " key = " << result4_key[0] << " " << result4_key[1] << " " << result4_key[2] << std::endl;
	    }
	    else
	      std::cout << m_profile.instance_name << ": " << pp4 << " can not be searched." << std::endl;
	    if (result5){
	      prob5 = result5->getOccupancy();
	      std::cout << m_profile.instance_name << ": " << pp5 << " original prob = " << prob5 << std::endl;
	      std::cout << m_profile.instance_name << ": " << pp5 << " key = " << result5_key[0] << " " << result5_key[1] << " " << result5_key[2] << std::endl;
	    }
	    else
	      std::cout << m_profile.instance_name << ": " << pp5 << " can not be searched." << std::endl;

	    OcTreeNode *result;
	    OcTreeKey result_key;
	    double prob;
	    point3d pp;
	    pp.z() = 0.5;
	    for(int i=0; i<5; i++){
	      pp.x() = -0.3 - 0.1*i;
	      for(int j=0; j<5; j++){
		pp.y() = 2.8 + 0.1*j;
		result = m_map->search(pp);
		m_map->genKey(pp, result_key);
		if(result){
		  prob = result->getOccupancy();
		  std::cout << m_profile.instance_name << ": " << pp << " original prob = " << prob << std::endl;
		  std::cout << m_profile.instance_name << ": " << pp << " key = " << result_key[0] << " " << result_key[1] << " " << result_key[2] << std::endl;
		}
		else
		  std::cout << m_profile.instance_name << ": " << pp << " can not be searched." << std::endl;
	      }
	    }
#endif
            for (unsigned int i=0; i<m_cloud.data.length()/16; i++, ptr+=4){
                if (isnan(ptr[0])) continue;
                hrp::Vector3 peye(ptr[0],ptr[1],ptr[2]);
                hrp::Vector3 pworld(R*peye+p);
                point3d pog(pworld[0],pworld[1],pworld[2]);
#if KDEBUG
		OcTreeNode *target_node; // 121023
		OcTreeKey target_key; // 121023
		if((pworld[0]>(pp1.x()-0.1)&&pworld[0]<pp1.x()&&pworld[1]>pp1.y()&&pworld[1]<(pp1.y()+0.1)&&pworld[2]>pp1.z()&&pworld[2]<(pp1.z()+0.1)) ||
		   (pworld[0]>(pp2.x()-0.1)&&pworld[0]<pp2.x()&&pworld[1]>pp2.y()&&pworld[1]<(pp2.y()+0.1)&&pworld[2]>pp2.z()&&pworld[2]<(pp2.z()+0.1)) ||
		   (pworld[0]>(pp3.x()-0.1)&&pworld[0]<pp3.x()&&pworld[1]>pp3.y()&&pworld[1]<(pp3.y()+0.1)&&pworld[2]>pp3.z()&&pworld[2]<(pp3.z()+0.1))){
		  target_node = m_map->search(pog); // 121023
		  m_map->genKey(pog, target_key); // 121023
		  if (target_node){
		    double beforeprob = target_node->getOccupancy();
		    std::cout << m_profile.instance_name << ": " << pog << " before prob = " << beforeprob << std::endl;
		    std::cout << m_profile.instance_name << ": " << pog << " key = " << target_key[0] << " " << target_key[1] << " " << target_key[2] << std::endl;
		  }
		  else
		    std::cout << m_profile.instance_name << ": " << pog << " can not be searched." << std::endl;
		}
#endif
		//                m_map->updateNode(pog, ptr[3]>0.0?true:false, false);
		OcTreeNode *updated_node = m_map->updateNode(pog, ptr[3]>0.0?true:false, false); // 121023
#if KDEBUG
#if 0
		std::cout << m_profile.instance_name << ": tree depth = " << m_map->getTreeDepth() << std::endl;
		std::cout << m_profile.instance_name << ": tree size = " << m_map->size() << std::endl;
#endif
		if((pworld[0]>(pp1.x()-0.1)&&pworld[0]<pp1.x()&&pworld[1]>pp1.y()&&pworld[1]<(pp1.y()+0.1)&&pworld[2]>pp1.z()&&pworld[2]<(pp1.z()+0.1)) ||
		   (pworld[0]>(pp2.x()-0.1)&&pworld[0]<pp2.x()&&pworld[1]>pp2.y()&&pworld[1]<(pp2.y()+0.1)&&pworld[2]>pp2.z()&&pworld[2]<(pp2.z()+0.1)) ||
		   (pworld[0]>(pp3.x()-0.1)&&pworld[0]<pp3.x()&&pworld[1]>pp3.y()&&pworld[1]<(pp3.y()+0.1)&&pworld[2]>pp3.z()&&pworld[2]<(pp3.z()+0.1))){
		  target_node = m_map->search(pog); // 121023
		  m_map->genKey(pog, target_key); // 121023
		  if (target_node){
		    double afterprob = target_node->getOccupancy();
		    std::cout << m_profile.instance_name << ": " << pog << " after prob = " << afterprob << std::endl;
		  }
		  if (updated_node){
		    double updatedprob = updated_node->getOccupancy();
		    std::cout << m_profile.instance_name << ": " << pog << " updated prob = " << updatedprob << std::endl;
		  }
		  //		}
		result2 = m_map->search(pp2);
		m_map->genKey(pp2, result2_key);
		if (result2){
		  double newprob = result2->getOccupancy();
		  //		  if(newprob != prob){ // wrong
		  //		  if(!(abs(newprob - prob) < 0.0001)){ // wrong
		  //		  if(!(fabs(newprob - prob2) < 0.0001)){ // correct 121023
		    std::cout << m_profile.instance_name << ": " << pp2 << " updated prob = " << newprob << " by " << pog << " prob = " << prob2 << std::endl;
		    std::cout << m_profile.instance_name << ": " << pp2 << " key = " << result2_key[0] << " " << result2_key[1] << " " << result2_key[2] << std::endl;
		    std::cout << m_profile.instance_name << ": " << pp2 << " hasChildren = " << result2->hasChildren() << std::endl;
		    prob2 = newprob;
		    //		  }
		}
		else
		  std::cout << m_profile.instance_name << ": " << pp2 << " can not be searched." << std::endl;
		}
#endif
                ptr[3]>0.0?ocnum++:emnum++;
            }
            if(KDEBUG) std::cout << m_profile.instance_name << ": " << ocnum << " " << emnum << " " << p << std::endl;
        }else{
            std::cout << "point type(" << m_cloud.type 
                      << ") is not supported" << std::endl;
            return RTC::RTC_ERROR;
        }
        m_updateOut.write();
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
    Guard guard(m_mutex);
    coil::TimeValue t1(coil::gettimeofday());

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

    if (m_knownMap){
        double kmin[3];
        m_knownMap->getMetricMin(kmin[0],kmin[1],kmin[2]);
        double kmax[3];
        m_knownMap->getMetricMax(kmax[0],kmax[1],kmax[2]);
        for (int i=0; i<3; i++){
            kmin[i] -= size; 
            kmax[i] += size; 
            if (kmin[i] < min[i]) min[i] = kmin[i]; 
            if (kmax[i] > max[i]) max[i] = kmax[i]; 
        }
        
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
#if 0
    map->pos.x = s[0];
    map->pos.y = s[1];
    map->pos.z = s[2];
    if(KDEBUG){
      std::cout << m_profile.instance_name << ": pos = " << map->pos.x << " " << map->pos.y << " " << map->pos.z << " " << std::endl;
      map->pos.x += size/2.0;
      map->pos.y += size/2.0;
      map->pos.z += size/2.0;
    }
#endif      
    map->pos.x = ((int)(s[0]/size)+0.5)*size; // 121024
    map->pos.y = ((int)(s[1]/size)+0.5)*size; // 121024
    map->pos.z = ((int)(s[2]/size)+0.5)*size; // 121024
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
		    //                    if (result){
                    if (result && !(result->hasChildren())){ // 121023
                        double prob = result->getOccupancy();
                        if (prob >= m_occupiedThd){
                            map->cells[rank] = prob*0xfe;
                            no++;
			    //			    if(KDEBUG) std::cout << m_profile.instance_name << ": output " << p << " " << prob << std::endl;
                        }else{
                            map->cells[rank] = OpenHRP::gridEmpty;
                            ne++;
                        }
                        //printf("%6.3f %6.3f %6.3f:%d\n", p.x(), p.y(), p.z(),map->cells[rank-1]);
                    }else{
                        map->cells[rank] = OpenHRP::gridUnknown;
                        nu++;
                    }
                    if (m_knownMap){
                        OcTreeNode *result = m_knownMap->search(p);
                        if (result){
                            double prob = result->getOccupancy();
                            if (prob >= m_occupiedThd){
                                map->cells[rank] = prob*0xfe;
                            }
                        }
                    }
                    rank++;
                }
            }
        }
#if 0
        std::cout << "occupied/empty/unknown = " << no << "/" << ne << "/" 
                  << nu << std::endl;
#endif
    }
    coil::TimeValue t2(coil::gettimeofday());
    if (m_debugLevel > 0){
        coil::TimeValue dt = t2-t1;
        std::cout << "OccupancyGridMap3D::getOGMap3D() : " 
                  << dt.sec()*1e3+dt.usec()/1e3 << "[ms]" << std::endl;
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


