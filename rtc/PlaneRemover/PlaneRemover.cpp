// -*- C++ -*-
/*!
 * @file  PlaneRemover.cpp
 * @brief plane remover
 * $Date$
 *
 * $Id$
 */

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include "PlaneRemover.h"
#include "hrpsys/idl/pointcloud.hh"

// Module specification
// <rtc-template block="module_spec">
static const char* spec[] =
  {
    "implementation_id", "PlaneRemover",
    "type_name",         "PlaneRemover",
    "description",       "Plane Remover",
    "version",           HRPSYS_PACKAGE_VERSION,
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.distanceThd", "0.02",
    "conf.default.pointNumThd", "500",

    ""
  };
// </rtc-template>

PlaneRemover::PlaneRemover(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_originalIn("original", m_original),
    m_filteredOut("filtered", m_filtered),
    // </rtc-template>
    dummy(0)
{
}

PlaneRemover::~PlaneRemover()
{
}



RTC::ReturnCode_t PlaneRemover::onInitialize()
{
  //std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("distanceThd", m_distThd, "0.02");
  bindParameter("pointNumThd", m_pointNumThd, "500");
  
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
RTC::ReturnCode_t PlaneRemover::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PlaneRemover::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PlaneRemover::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t PlaneRemover::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t PlaneRemover::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t PlaneRemover::onExecute(RTC::UniqueId ec_id)
{
  //std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;

  if (m_originalIn.isNew()){
    m_originalIn.read();

    // CORBA -> PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr original (new pcl::PointCloud<pcl::PointXYZ>);
    original->points.resize(m_original.width*m_original.height);
    float *src = (float *)m_original.data.get_buffer();
    for (unsigned int i=0; i<original->points.size(); i++){
      original->points[i].x = src[0];
      original->points[i].y = src[1];
      original->points[i].z = src[2];
      src += 4;
    }

    // PROCESSING

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (m_distThd);
  
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = original;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
  
    pcl::ExtractIndices<pcl::PointXYZ> extract;
  
    while(1){
      seg.setInputCloud (cloud);
      seg.segment (*inliers, *coefficients);
    
      if (inliers->indices.size () < m_pointNumThd) break;
    
      extract.setInputCloud( cloud );
      extract.setIndices( inliers );
      extract.setNegative( true );
      extract.filter( *cloud_f );
      cloud = cloud_f;
    }

    //std::cout << "PLaneRemover: original = " << original->points.size() << ", filtered = " << cloud->points.size() << ", thd=" << m_distThd << std::endl;

    // PCL -> CORBA
    m_filtered.width = cloud->points.size();
    m_filtered.row_step = m_filtered.point_step*m_filtered.width;
    m_filtered.data.length(m_filtered.height*m_filtered.row_step);
    float *dst = (float *)m_filtered.data.get_buffer();
    for (unsigned int i=0; i<cloud->points.size(); i++){
      dst[0] = cloud->points[i].x;
      dst[1] = cloud->points[i].y;
      dst[2] = cloud->points[i].z;
      dst += 4;
    }
    m_filteredOut.write();
  }

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t PlaneRemover::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PlaneRemover::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PlaneRemover::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PlaneRemover::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PlaneRemover::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void PlaneRemoverInit(RTC::Manager* manager)
  {
    RTC::Properties profile(spec);
    manager->registerFactory(profile,
                             RTC::Create<PlaneRemover>,
                             RTC::Delete<PlaneRemover>);
  }

};


