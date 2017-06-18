// -*- C++ -*-
/*!
 * @file  Range2PointCloud.cpp
 * @brief range2pointcloud component
 * $Date$
 *
 * $Id$
 */

#include <math.h>
#include <hrpUtil/Eigen3d.h>
#include "Range2PointCloud.h"

// Module specification
// <rtc-template block="module_spec">
static const char* range2pointcloud_spec[] = {
    "implementation_id", "Range2PointCloud", "type_name", "Range2PointCloud",
    "description", "range2pointcloud component", "version",
    HRPSYS_PACKAGE_VERSION, "vendor", "AIST", "category", "example",
    "activity_type", "DataFlowComponent", "max_instance", "10", "language",
    "C++", "lang_type", "compile",
    // Configuration variables

    ""};
// </rtc-template>

Range2PointCloud::Range2PointCloud(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      // <rtc-template block="initializer">
      m_rangeIn("range", m_range),
      m_cloudOut("cloud", m_cloud),
      // </rtc-template>
      dummy(0) {}

Range2PointCloud::~Range2PointCloud() {}

RTC::ReturnCode_t Range2PointCloud::onInitialize() {
  std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable

  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("range", m_rangeIn);

  // Set OutPort buffer
  addOutPort("cloud", m_cloudOut);

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
RTC::ReturnCode_t Range2PointCloud::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Range2PointCloud::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Range2PointCloud::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t Range2PointCloud::onActivated(RTC::UniqueId ec_id) {
  std::cout << m_profile.instance_name << ": onActivated(" << ec_id << ")"
            << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t Range2PointCloud::onDeactivated(RTC::UniqueId ec_id) {
  std::cout << m_profile.instance_name << ": onDeactivated(" << ec_id << ")"
            << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t Range2PointCloud::onExecute(RTC::UniqueId ec_id) {
  // std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" <<
  // std::endl;
  if (!m_rangeIn.isNew()) return RTC::RTC_OK;

  m_cloud.width = 0;
  int npoint = 0;
  int nlines = 0;
  while (m_rangeIn.isNew()) {
    nlines++;
    m_rangeIn.read();
    m_cloud.width += m_range.ranges.length();
    m_cloud.row_step = m_cloud.point_step * m_cloud.width;
    m_cloud.data.length(m_cloud.row_step);  // shrinked later
    // range -> point cloud
    float* ptr = (float*)m_cloud.data.get_buffer() + npoint * 4;
    Pose3D& pose = m_range.geometry.geometry.pose;
    hrp::Vector3 relP, absP,
        sensorP(pose.position.x, pose.position.y, pose.position.z);
    hrp::Matrix33 sensorR = hrp::rotFromRpy(
        pose.orientation.r, pose.orientation.p, pose.orientation.y);
    for (unsigned int i = 0; i < m_range.ranges.length(); i++) {
      double th = m_range.config.minAngle + i * m_range.config.angularRes;
      double d = m_range.ranges[i];
      if (d == 0) continue;
      relP << -d * sin(th), 0, -d * cos(th);
      absP = sensorP + sensorR * relP;
      ptr[0] = absP[0];
      ptr[1] = absP[1];
      ptr[2] = absP[2];
      // std::cout << "(" << i << "," << ptr[2] << "," << d << ")" << std::endl;
      ptr += 4;
      npoint++;
    }
  }
#if 0
  std::cout << "Range2PointCloud: processed " << nlines << " lines, " 
	    << npoint << " points" << std::endl;
#endif
  m_cloud.width = npoint;
  m_cloud.data.length(npoint * m_cloud.point_step);
  m_cloudOut.write();

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t Range2PointCloud::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Range2PointCloud::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Range2PointCloud::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Range2PointCloud::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Range2PointCloud::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

extern "C" {
void Range2PointCloudInit(RTC::Manager* manager) {
  RTC::Properties profile(range2pointcloud_spec);
  manager->registerFactory(profile, RTC::Create<Range2PointCloud>,
                           RTC::Delete<Range2PointCloud>);
}
};
