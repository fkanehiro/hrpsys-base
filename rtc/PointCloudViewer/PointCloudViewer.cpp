// -*- C++ -*-
/*!
 * @file  PointCloudViewer.cpp
 * @brief Point Cloud Viewer
 * $Date$
 *
 * $Id$
 */

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "PointCloudViewer.h"
#include "hrpsys/idl/pointcloud.hh"
#include <string>

// Module specification
// <rtc-template block="module_spec">
static const char* spec[] =
  {
    "implementation_id", "PointCloudViewer",
    "type_name",         "PointCloudViewer",
    "description",       "Point Cloud Viewer",
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

PointCloudViewer::PointCloudViewer(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_cloudIn("cloud", m_cloud),
    // </rtc-template>
    m_viewer("Point Cloud Viewer"),
    dummy(0)
{
}

PointCloudViewer::~PointCloudViewer()
{
}



RTC::ReturnCode_t PointCloudViewer::onInitialize()
{
  //std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("cloudIn", m_cloudIn);

  // Set OutPort buffer
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  RTC::Properties& prop = getProperties();

  return RTC::RTC_OK;
}



/*
RTC::ReturnCode_t PointCloudViewer::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PointCloudViewer::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PointCloudViewer::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t PointCloudViewer::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t PointCloudViewer::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t PointCloudViewer::onExecute(RTC::UniqueId ec_id)
{
  //std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;

  if (m_cloudIn.isNew()){
    m_cloudIn.read();

    bool is_color_points = false;
    for (int i = 0; i < m_cloud.fields.length(); i++) {
        std::string tmp_name(m_cloud.fields[i].name);
        if (tmp_name.find("r") != std::string::npos || tmp_name.find("g") != std::string::npos || tmp_name.find("b") != std::string::npos) {
            is_color_points = true; // color pointcloud should have rgb field 
        }
    }

    // currently only support PointXYZ and PointXYZRGB
    if (is_color_points) { 
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud->is_dense = m_cloud.is_dense;
        float *src = reinterpret_cast<float*>(m_cloud.data.get_buffer());
        for (unsigned int i = 0; i< (m_cloud.width * m_cloud.height); i++){
            pcl::PointXYZRGB tmp_point;
            tmp_point.x = src[0];
            tmp_point.y = src[1];
            tmp_point.z = src[2];
            uint32_t rgb = *reinterpret_cast<uint32_t*>(&(src[3])); // http://docs.pointclouds.org/1.7.2/a01059.html#a1678cfbe6e832aa61ec0de773cab15ae
            tmp_point.r = (uint8_t)((rgb >> 16) & 0x0000ff);
            tmp_point.g = (uint8_t)((rgb >> 8) & 0x0000ff);
            tmp_point.b = (uint8_t)((rgb) & 0x0000ff);
            if (m_cloud.is_dense || (pcl::isFinite(tmp_point) && !std::isnan(rgb))) { // check validity of point
                cloud->push_back(tmp_point);
            }
            src += 4;
        }
        if (!m_viewer.wasStopped()){
            m_viewer.showCloud(cloud);
        }
    } else {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        cloud->points.resize(m_cloud.width*m_cloud.height);
        float *src = (float *)m_cloud.data.get_buffer();
        for (unsigned int i=0; i<cloud->points.size(); i++){
            cloud->points[i].x = src[0];
            cloud->points[i].y = src[1];
            cloud->points[i].z = src[2];
            src += 4;
        }
        if (!m_viewer.wasStopped()){
            m_viewer.showCloud(cloud);
        }        
    }
  }

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t PointCloudViewer::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PointCloudViewer::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PointCloudViewer::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PointCloudViewer::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PointCloudViewer::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void PointCloudViewerInit(RTC::Manager* manager)
  {
    RTC::Properties profile(spec);
    manager->registerFactory(profile,
                             RTC::Create<PointCloudViewer>,
                             RTC::Delete<PointCloudViewer>);
  }

};


