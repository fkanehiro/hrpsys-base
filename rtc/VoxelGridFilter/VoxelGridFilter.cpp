// -*- C++ -*-
/*!
 * @file  VoxelGridFilter.cpp
 * @brief Moving Least Squares Filter
 * $Date$
 *
 * $Id$
 */

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "VoxelGridFilter.h"
#include "hrpsys/idl/pointcloud.hh"

// Module specification
// <rtc-template block="module_spec">
static const char* spec[] =
  {
    "implementation_id", "VoxelGridFilter",
    "type_name",         "VoxelGridFilter",
    "description",       "Voxel Grid Filter",
    "version",           HRPSYS_PACKAGE_VERSION,
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.size", "0.01",

    ""
  };
// </rtc-template>

VoxelGridFilter::VoxelGridFilter(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_originalIn("original", m_original),
    m_filteredOut("filtered", m_filtered),
    // </rtc-template>
    dummy(0)
{
}

VoxelGridFilter::~VoxelGridFilter()
{
}



RTC::ReturnCode_t VoxelGridFilter::onInitialize()
{
  //std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("size", m_size, "0.01");
  
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
RTC::ReturnCode_t VoxelGridFilter::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VoxelGridFilter::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VoxelGridFilter::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t VoxelGridFilter::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t VoxelGridFilter::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t VoxelGridFilter::onExecute(RTC::UniqueId ec_id)
{
  //std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;

  if (m_originalIn.isNew()){
    m_originalIn.read();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    // RTM -> PCL
    cloud->points.resize(m_original.width*m_original.height);
    float *src = (float *)m_original.data.get_buffer();
    for (unsigned int i=0; i<cloud->points.size(); i++){
      cloud->points[i].x = src[0];
      cloud->points[i].y = src[1];
      cloud->points[i].z = src[2];
      src += 4;
    }
    
    // PCL Processing 
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize(m_size, m_size, m_size);
    sor.filter(*cloud_filtered);

    // PCL -> RTM
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
RTC::ReturnCode_t VoxelGridFilter::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VoxelGridFilter::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VoxelGridFilter::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VoxelGridFilter::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VoxelGridFilter::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void VoxelGridFilterInit(RTC::Manager* manager)
  {
    RTC::Properties profile(spec);
    manager->registerFactory(profile,
                             RTC::Create<VoxelGridFilter>,
                             RTC::Delete<VoxelGridFilter>);
  }

};


