// -*- C++ -*-
/*!
 * @file  PCDLoader.cpp
 * @brief PCD file loader
 * $Date$
 *
 * $Id$
 */

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "PCDLoader.h"
#include "pointcloud.hh"

// Module specification
// <rtc-template block="module_spec">
static const char* spec[] =
  {
    "implementation_id", "PCDLoader",
    "type_name",         "PCDLoader",
    "description",       "PCD file loader",
    "version",           HRPSYS_PACKAGE_VERSION,
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables

    ""
  };
// </rtc-template>

PCDLoader::PCDLoader(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_cloudOut("cloud", m_cloud),
    // </rtc-template>
    dummy(0)
{
}

PCDLoader::~PCDLoader()
{
}



RTC::ReturnCode_t PCDLoader::onInitialize()
{
  //std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers

  // Set OutPort buffer
  addOutPort("cloudOut", m_cloudOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  RTC::Properties& prop = getProperties();

  m_cloud.height = 1;
  m_cloud.type = "xyz";
  m_cloud.fields.length(3);
  m_cloud.fields[0].name = "x";
  m_cloud.fields[0].offset = 0;
  m_cloud.fields[0].data_type = PointCloudTypes::FLOAT32;
  m_cloud.fields[0].count = 4;
  m_cloud.fields[1].name = "y";
  m_cloud.fields[1].offset = 4;
  m_cloud.fields[1].data_type = PointCloudTypes::FLOAT32;
  m_cloud.fields[1].count = 4;
  m_cloud.fields[2].name = "z";
  m_cloud.fields[2].offset = 8;
  m_cloud.fields[2].data_type = PointCloudTypes::FLOAT32;
  m_cloud.fields[2].count = 4;
  m_cloud.is_bigendian = false;
  m_cloud.point_step = 16;
  m_cloud.is_dense = true;

  return RTC::RTC_OK;
}



/*
RTC::ReturnCode_t PCDLoader::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PCDLoader::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PCDLoader::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t PCDLoader::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t PCDLoader::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t PCDLoader::onExecute(RTC::UniqueId ec_id)
{
  //std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;
  std::cerr << "PCD filename: " << std::flush;
  std::string filename;
  std::cin >> filename;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PCDReader reader;
  reader.read (filename, *cloud);
  int npoint = cloud->points.size();
  m_cloud.width = npoint;
  m_cloud.height = 1;
  m_cloud.data.length(npoint*m_cloud.point_step);
  m_cloud.row_step = m_cloud.data.length();
  float *ptr = (float *)m_cloud.data.get_buffer();
  std::cout << "npoint = " << npoint << std::endl;
  for (int i=0; i<npoint; i++){
    ptr[0] = cloud->points[i].x;
    ptr[1] = cloud->points[i].y;
    ptr[2] = cloud->points[i].z;
    ptr += 4;
  }

  m_cloudOut.write();

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t PCDLoader::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PCDLoader::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PCDLoader::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PCDLoader::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PCDLoader::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void PCDLoaderInit(RTC::Manager* manager)
  {
    RTC::Properties profile(spec);
    manager->registerFactory(profile,
                             RTC::Create<PCDLoader>,
                             RTC::Delete<PCDLoader>);
  }

};


