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
    "conf.default.mode", "depth",
    "conf.default.debugLevel", "0",

    ""
  };
// </rtc-template>

OpenNIGrabber::OpenNIGrabber(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_cloudOut("cloud", m_cloud),
    m_imageOut("image", m_image),
    // </rtc-template>
    m_interface(NULL),
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
  bindParameter("mode", m_mode, "depth");
  bindParameter("debugLevel", m_debugLevel, "0");
  
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

  return RTC::RTC_OK;
}


void OpenNIGrabber::grabberCallbackDepthAndColor(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
{
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
    for (unsigned int i=0; i<cloud->points.size(); i++){
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

void OpenNIGrabber::grabberCallbackDepth(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
    m_cloud.width = cloud->width;
    m_cloud.height = cloud->height;
    m_cloud.row_step = m_cloud.point_step*m_cloud.width;
    m_cloud.data.length(m_cloud.height*m_cloud.row_step);

    float *dst_cloud = (float *)m_cloud.data.get_buffer();
    for (unsigned int i=0; i<cloud->points.size(); i++){
        dst_cloud[0] = cloud->points[i].x;
        dst_cloud[1] = cloud->points[i].y;
        dst_cloud[2] = cloud->points[i].z;
        dst_cloud += 4;
    }
    m_cloudOut.write();
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

  try {
      m_interface = new pcl::io::OpenNI2Grabber();

      if (m_mode == "depth"){
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
          m_cloud.is_dense = false;
          boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f = boost::bind(&OpenNIGrabber::grabberCallbackDepth, this, _1);
          m_interface->registerCallback(f);
      }else if(m_mode == "depth_and_color"){
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
          boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f = boost::bind(&OpenNIGrabber::grabberCallbackDepthAndColor, this, _1);
          m_interface->registerCallback(f);
      }else{
          std::cerr << "[" << m_profile.instance_name << "] Error: unknown mode ("
                    << m_mode << ")" << std::endl;
          return RTC::RTC_ERROR;
      }

      m_interface->start();
  }catch(pcl::IOException& ex){
      std::cerr << "[" << m_profile.instance_name << "] Error: " << ex.what() << std::endl;
      return RTC::RTC_ERROR;
  }catch(...){
      std::cerr << "[" << m_profile.instance_name
                << "] Error: An exception occurred while starting grabber" << std::endl;
      return RTC::RTC_ERROR;
  }

  return RTC::RTC_OK;
}

RTC::ReturnCode_t OpenNIGrabber::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;

  if (m_interface){
      m_interface->stop();

      delete m_interface;
  }

  return RTC::RTC_OK;
}

RTC::ReturnCode_t OpenNIGrabber::onExecute(RTC::UniqueId ec_id)
{
  //std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;
    if (m_debugLevel>0){
        if (m_interface->isRunning()){
            std::cerr << "[" << m_profile.instance_name
                      << "] grabber is running. frame rate=" << m_interface->getFramesPerSecond() << std::endl;
        }else{
            std::cerr << "[" << m_profile.instance_name
                      << "] grabber is not running" << std::endl;
        }
    }
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


