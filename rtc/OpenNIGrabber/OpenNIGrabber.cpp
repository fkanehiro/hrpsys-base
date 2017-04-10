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
    "conf.default.outputColorImage", "0",
    "conf.default.outputDepthImage", "0",
    "conf.default.outputPointCloud", "0",
    "conf.default.outputPointCloudRGBA", "0",
    "conf.default.debugLevel", "0",

    ""
  };
// </rtc-template>

OpenNIGrabber::OpenNIGrabber(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_cloudOut("cloud", m_cloud),
    m_imageOut("image", m_image),
    m_depthOut("depth", m_depth),
    // </rtc-template>
    m_interface(NULL),
    m_requestToWriteImage(false),
    m_requestToWritePointCloud(false),
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
  bindParameter("outputColorImage", m_outputColorImage, "0");
  bindParameter("outputDepthImage", m_outputDepthImage, "0");
  bindParameter("outputPointCloud", m_outputPointCloud, "0");
  bindParameter("outputPointCloudRGBA", m_outputPointCloudRGBA, "0");
  bindParameter("debugLevel", m_debugLevel, "0");
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers

  // Set OutPort buffer
  addOutPort("cloudOut", m_cloudOut);
  addOutPort("imageOut", m_imageOut);
  addOutPort("depthOut", m_depthOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  RTC::Properties& prop = getProperties();

  return RTC::RTC_OK;
}


void OpenNIGrabber::grabberCallbackColorImage(const boost::shared_ptr<pcl::io::Image>& image)
{
    if (!m_requestToWriteImage) return;

    outputColorImage(image);

    m_requestToWriteImage = false;
}

void OpenNIGrabber::outputColorImage(const boost::shared_ptr<pcl::io::Image>& image)
{
    setTimestamp(m_image);

    Img::ImageData &id = m_image.data.image;
    id.width = image->getWidth();
    id.height = image->getHeight();
    id.format = Img::CF_RGB;
    id.raw_data.length(id.width*id.height*3);

    unsigned char *dst = (unsigned char*)id.raw_data.get_buffer();
    unsigned char *src = (unsigned char*)image->getData();
    memcpy(dst, src, id.raw_data.length());

    m_imageOut.write();
}

void OpenNIGrabber::grabberCallbackDepthImage(const boost::shared_ptr<pcl::io::DepthImage>& image)
{
    if (!m_requestToWriteImage) return;

    outputDepthImage(image);

    m_requestToWriteImage = false;
}

void OpenNIGrabber::outputDepthImage(const boost::shared_ptr<pcl::io::DepthImage>& image)
{
    setTimestamp(m_depth);

    Img::ImageData &id = m_depth.data.image;
    id.width = image->getWidth();
    id.height = image->getHeight();
    id.format = Img::CF_DEPTH;
    id.raw_data.length(id.width*id.height*2);

    unsigned char *dst = (unsigned char*)id.raw_data.get_buffer();
    unsigned char *src = (unsigned char*)image->getData();
    memcpy(dst, src, id.raw_data.length());

    m_depthOut.write();
}

void OpenNIGrabber::grabberCallbackColorAndDepthImage(const boost::shared_ptr<pcl::io::Image>& image, const boost::shared_ptr<pcl::io::DepthImage>& depth, float reciprocalFocalLength)
{
    if (!m_requestToWriteImage) return;

    outputColorImage(image);
    outputDepthImage(depth);

    m_requestToWriteImage = false;
}

void OpenNIGrabber::grabberCallbackPointCloudRGBA(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
{
    if (!m_requestToWritePointCloud) return;

    setTimestamp(m_cloud);

    m_cloud.width = cloud->width;
    m_cloud.height = cloud->height;
    m_cloud.row_step = m_cloud.point_step*m_cloud.width;
    m_cloud.data.length(m_cloud.height*m_cloud.row_step);

    float *dst_cloud = (float *)m_cloud.data.get_buffer();
    for (unsigned int i=0; i<cloud->points.size(); i++){
        dst_cloud[0] =  cloud->points[i].x;
        dst_cloud[1] = -cloud->points[i].y;
        dst_cloud[2] = -cloud->points[i].z;
        dst_cloud[3] =  cloud->points[i].rgb;
        dst_cloud += 4;
    }
    m_cloudOut.write();
    m_requestToWritePointCloud = false;
}

void OpenNIGrabber::grabberCallbackPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
    if (!m_requestToWritePointCloud) return;

    setTimestamp(m_cloud);
    m_cloud.width = cloud->width;
    m_cloud.height = cloud->height;
    m_cloud.row_step = m_cloud.point_step*m_cloud.width;
    m_cloud.data.length(m_cloud.height*m_cloud.row_step);

    float *dst_cloud = (float *)m_cloud.data.get_buffer();
    for (unsigned int i=0; i<cloud->points.size(); i++){
        dst_cloud[0] =  cloud->points[i].x;
        dst_cloud[1] = -cloud->points[i].y;
        dst_cloud[2] = -cloud->points[i].z;
        dst_cloud += 4;
    }
    m_cloudOut.write();
    m_requestToWritePointCloud = false;
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

      if (m_outputColorImage && !m_outputDepthImage){
          boost::function<void (const boost::shared_ptr<pcl::io::Image>&)> f = boost::bind(&OpenNIGrabber::grabberCallbackColorImage, this, _1);
          m_interface->registerCallback(f);
      }else if (!m_outputColorImage && m_outputDepthImage){
          boost::function<void (const boost::shared_ptr<pcl::io::DepthImage>&)> f = boost::bind(&OpenNIGrabber::grabberCallbackDepthImage, this, _1);
          m_interface->registerCallback(f);
      }else if (m_outputColorImage && m_outputDepthImage){
          boost::function<void (const boost::shared_ptr<pcl::io::Image>&, const boost::shared_ptr<pcl::io::DepthImage>&, float)> f = boost::bind(&OpenNIGrabber::grabberCallbackColorAndDepthImage, this, _1, _2, _3);
          m_interface->registerCallback(f);
      }
      if (m_outputPointCloud){
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
          boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f = boost::bind(&OpenNIGrabber::grabberCallbackPointCloud, this, _1);
          m_interface->registerCallback(f);
      }else if(m_outputPointCloudRGBA){
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
          m_cloud.is_dense = true;
          boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind(&OpenNIGrabber::grabberCallbackPointCloudRGBA, this, _1);
          m_interface->registerCallback(f);
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
    m_requestToWriteImage      = true;
    m_requestToWritePointCloud = true;

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


