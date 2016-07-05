// -*- C++ -*-
/*!
 * @file  OpenNIGrabber.cpp
 * @brief OpenNI grabber
 * $Date$
 *
 * $Id$
 */

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include "OpenNIGrabber.h"
#include "hrpsys/idl/pointcloud.hh"

// Module specification
// <rtc-template block="module_spec">
static const char* spec[] =
  {
    "implementation_id", "OpenNIGrabber",
    "type_name",         "OpenNIGrabber",
    "description",       "OpenNI grabber",
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

OpenNIGrabber::OpenNIGrabber(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_cloudOut("cloud", m_cloud),
    m_imageOut("image", m_image),
    // </rtc-template>
    dummy(0)
{
}

OpenNIGrabber::~OpenNIGrabber()
{
}



RTC::ReturnCode_t OpenNIGrabber::onInitialize()
{
  std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers

  // Set OutPort buffer
  addOutPort("cloudOut", m_cloudOut);
  addOutPort("imageOut", m_imageOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  RTC::Properties& prop = getProperties();

  m_cloud.type = "xyzrgb";
  m_cloud.fields.length(6);
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
  m_cloud.fields[3].name = "r";
  m_cloud.fields[3].offset = 12;
  m_cloud.fields[3].data_type = PointCloudTypes::UINT8;
  m_cloud.fields[3].count = 1;
  m_cloud.fields[4].name = "g";
  m_cloud.fields[4].offset = 13;
  m_cloud.fields[4].data_type = PointCloudTypes::UINT8;
  m_cloud.fields[4].count = 1;
  m_cloud.fields[5].name = "b";
  m_cloud.fields[5].offset = 14;
  m_cloud.fields[5].data_type = PointCloudTypes::UINT8;
  m_cloud.fields[5].count = 1;
  m_cloud.is_bigendian = false;
  m_cloud.point_step = 16;
  m_cloud.is_dense = false;

  return RTC::RTC_OK;
}


void OpenNIGrabber::grabberCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
{
    // PCL -> RTM
    m_cloud.width = cloud->width;
    m_cloud.height = cloud->height;
    m_cloud.row_step = m_cloud.point_step*m_cloud.width;
    m_cloud.data.length(m_cloud.height*m_cloud.row_step);

    m_image.data.image.width = cloud->width;
    m_image.data.image.height = cloud->height;
    m_image.data.image.format = Img::CF_RGB;
    m_image.data.image.raw_data.length(cloud->width*cloud->height*3);

    float *dst_cloud = (float *)m_cloud.data.get_buffer();
    unsigned char *dst_image = (unsigned char*)m_image.data.image.raw_data.get_buffer();
    for (int i=0; i<cloud->points.size(); i++){
        dst_cloud[0] = cloud->points[i].x;
        dst_cloud[1] = cloud->points[i].y;
        dst_cloud[2] = cloud->points[i].z;
        dst_cloud[3] = cloud->points[i].rgb;
        dst_cloud += 4;

        dst_image[0] = cloud->points[i].r;
        dst_image[1] = cloud->points[i].g;
        dst_image[2] = cloud->points[i].b;
        dst_image += 3;
    }
    m_cloudOut.write();
    m_imageOut.write();
}

/*
RTC::ReturnCode_t OpenNIGrabber::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t OpenNIGrabber::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t OpenNIGrabber::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t OpenNIGrabber::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;

  m_interface = new pcl::io::OpenNI2Grabber();
  boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f = boost::bind(&OpenNIGrabber::grabberCallback, this, _1);

  m_interface->registerCallback(f);

  m_interface->start();

  return RTC::RTC_OK;
}

RTC::ReturnCode_t OpenNIGrabber::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;

  m_interface->stop();

  delete m_interface;

  return RTC::RTC_OK;
}

RTC::ReturnCode_t OpenNIGrabber::onExecute(RTC::UniqueId ec_id)
{
  //std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t OpenNIGrabber::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t OpenNIGrabber::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t OpenNIGrabber::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t OpenNIGrabber::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t OpenNIGrabber::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void OpenNIGrabberInit(RTC::Manager* manager)
  {
    RTC::Properties profile(spec);
    manager->registerFactory(profile,
                             RTC::Create<OpenNIGrabber>,
                             RTC::Delete<OpenNIGrabber>);
  }

};


