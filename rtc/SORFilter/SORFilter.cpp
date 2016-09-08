// -*- C++ -*-
/*!
 * @file  SORFilter.cpp
 * @brief Statistical Outlier Removal Filter
 * $Date$
 *
 * $Id$
 */

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "SORFilter.h"
#include "hrpsys/idl/pointcloud.hh"

// Module specification
// <rtc-template block="module_spec">
static const char* spec[] =
  {
    "implementation_id", "SORFilter",
    "type_name",         "SORFilter",
    "description",       "Statistical Outlier Removal Filter",
    "version",           HRPSYS_PACKAGE_VERSION,
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.meanK", "50",
    "conf.default.stddevMulThresh", "1.0",

    ""
  };
// </rtc-template>

SORFilter::SORFilter(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_originalIn("original", m_original),
    m_filteredOut("filtered", m_filtered),
    // </rtc-template>
    dummy(0)
{
}

SORFilter::~SORFilter()
{
}



RTC::ReturnCode_t SORFilter::onInitialize()
{
  //std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("meanK", m_meanK, "50");
  bindParameter("stddevMulThresh", m_stddevMulThresh, "1.0");
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("originalIn", m_originalIn);

  // Set OutPort buffer
  addOutPort("filteredOut", m_filteredOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  RTC::Properties& prop = getProperties();

  m_filtered.height = 1;
  m_filtered.type = "xyz";
  m_filtered.fields.length(3);
  m_filtered.fields[0].name = "x";
  m_filtered.fields[0].offset = 0;
  m_filtered.fields[0].data_type = PointCloudTypes::FLOAT32;
  m_filtered.fields[0].count = 4;
  m_filtered.fields[1].name = "y";
  m_filtered.fields[1].offset = 4;
  m_filtered.fields[1].data_type = PointCloudTypes::FLOAT32;
  m_filtered.fields[1].count = 4;
  m_filtered.fields[2].name = "z";
  m_filtered.fields[2].offset = 8;
  m_filtered.fields[2].data_type = PointCloudTypes::FLOAT32;
  m_filtered.fields[2].count = 4;
  m_filtered.is_bigendian = false;
  m_filtered.point_step = 16;
  m_filtered.is_dense = true;

  return RTC::RTC_OK;
}



/*
RTC::ReturnCode_t SORFilter::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SORFilter::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SORFilter::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t SORFilter::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t SORFilter::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t SORFilter::onExecute(RTC::UniqueId ec_id)
{
  //std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;

  if (m_originalIn.isNew()){
    m_originalIn.read();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    cloud->points.resize(m_original.width*m_original.height);
    float *src = (float *)m_original.data.get_buffer();
    for (unsigned int i=0; i<cloud->points.size(); i++){
      cloud->points[i].x = src[0];
      cloud->points[i].y = src[1];
      cloud->points[i].z = src[2];
      src += 4;
    }
    
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (m_meanK);
    sor.setStddevMulThresh (m_stddevMulThresh);
    sor.filter (*cloud_filtered);

    m_filtered.width = cloud_filtered->points.size();
    m_filtered.row_step = m_filtered.point_step*m_filtered.width;
    m_filtered.data.length(m_filtered.height*m_filtered.row_step);
    float *dst = (float *)m_filtered.data.get_buffer();
    for (unsigned int i=0; i<cloud_filtered->points.size(); i++){
      dst[0] = cloud_filtered->points[i].x;
      dst[1] = cloud_filtered->points[i].y;
      dst[2] = cloud_filtered->points[i].z;
      dst += 4;
    }
    m_filteredOut.write();
  }

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t SORFilter::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SORFilter::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SORFilter::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SORFilter::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SORFilter::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void SORFilterInit(RTC::Manager* manager)
  {
    RTC::Properties profile(spec);
    manager->registerFactory(profile,
                             RTC::Create<SORFilter>,
                             RTC::Delete<SORFilter>);
  }

};


