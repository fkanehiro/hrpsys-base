// -*- C++ -*-
/*!
 * @file  PCDLoader.cpp
 * @brief PCD file loader
 * $Date$
 *
 * $Id$
 */

#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "PCDLoader.h"
#include "hrpsys/idl/pointcloud.hh"

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
    "conf.default.path",  "",
    "conf.default.fields","XYZ",

    ""
  };
// </rtc-template>

PCDLoader::PCDLoader(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_offsetIn("offset", m_offset),
    m_isOutputOut("isOutput", m_isOutput),
    m_cloudOut("cloud", m_cloud),
    // </rtc-template>
    m_PCDLoaderServicePort("PCDLoaderService"),
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
    bindParameter("path", m_path, "");
    bindParameter("fields", m_fields, "XYZ");
    
    // </rtc-template>
    
    // Registration: InPort/OutPort/Service
    // <rtc-template block="registration">
    // Set InPort buffers
    addInPort("offsetIn", m_offsetIn);
    
    // Set OutPort buffer
    addOutPort("isOutputOut", m_isOutputOut);
    addOutPort("cloudOut", m_cloudOut);
    
    // Set service provider to Ports
    m_PCDLoaderServicePort.registerProvider("service0", "PCDLoaderService", m_service0);
    addPort(m_PCDLoaderServicePort);
    m_service0.setComp(this);
    m_isOutput.data = true;
    
    // Set service consumers to Ports
    
    // Set CORBA Service Ports
    
    // </rtc-template>
    
    RTC::Properties& prop = getProperties();
    
    return RTC::RTC_OK;
}

void PCDLoader::setCloudXYZ(PointCloudTypes::PointCloud& cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw)
{
    int npoint = cloud_raw->points.size();
    
    cloud.type = "xyz";
    cloud.fields.length(3);
    cloud.fields[0].name = "x";
    cloud.fields[0].offset = 0;
    cloud.fields[0].data_type = PointCloudTypes::FLOAT32;
    cloud.fields[0].count = 4;
    cloud.fields[1].name = "y";
    cloud.fields[1].offset = 4;
    cloud.fields[1].data_type = PointCloudTypes::FLOAT32;
    cloud.fields[1].count = 4;
    cloud.fields[2].name = "z";
    cloud.fields[2].offset = 8;
    cloud.fields[2].data_type = PointCloudTypes::FLOAT32;
    cloud.fields[2].count = 4;
    cloud.is_bigendian = false;
    cloud.point_step = 16;
    cloud.width = cloud_raw->width;
    cloud.height = cloud_raw->height;
    cloud.data.length(npoint*cloud.point_step);
    cloud.row_step = cloud.width*cloud.point_step;
    cloud.is_dense = cloud_raw->is_dense;
    float *ptr = (float *)cloud.data.get_buffer();
    std::cout << "npoint = " << npoint << std::endl;
    for (int i=0; i<npoint; i++){
        ptr[0] = cloud_raw->points[i].x;
        ptr[1] = cloud_raw->points[i].y;
        ptr[2] = cloud_raw->points[i].z;
        ptr += 4;
    }
}

void PCDLoader::setCloudXYZRGB(PointCloudTypes::PointCloud& cloud, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_raw)
{
    int npoint = cloud_raw->points.size();
    
    cloud.type = "xyzrgb";
    cloud.fields.length(6);
    cloud.fields[0].name = "x";
    cloud.fields[0].offset = 0;
    cloud.fields[0].data_type = PointCloudTypes::FLOAT32;
    cloud.fields[0].count = 4;
    cloud.fields[1].name = "y";
    cloud.fields[1].offset = 4;
    cloud.fields[1].data_type = PointCloudTypes::FLOAT32;
    cloud.fields[1].count = 4;
    cloud.fields[2].name = "z";
    cloud.fields[2].offset = 8;
    cloud.fields[2].data_type = PointCloudTypes::FLOAT32;
    cloud.fields[2].count = 4;
    cloud.fields[3].name = "r";
    cloud.fields[3].offset = 12;
    cloud.fields[3].data_type = PointCloudTypes::UINT8;
    cloud.fields[3].count = 1;
    cloud.fields[4].name = "g";
    cloud.fields[4].offset = 13;
    cloud.fields[4].data_type = PointCloudTypes::UINT8;
    cloud.fields[4].count = 1;
    cloud.fields[5].name = "b";
    cloud.fields[5].offset = 14;
    cloud.fields[5].data_type = PointCloudTypes::UINT8;
    cloud.fields[5].count = 1;
    cloud.is_bigendian = false;
    cloud.point_step = 16;
    cloud.width = cloud_raw->width;
    cloud.height = cloud_raw->height;
    cloud.data.length(npoint*cloud.point_step);
    cloud.row_step = cloud.width*cloud.point_step;
    cloud.is_dense = cloud_raw->is_dense;
    float *ptr = (float *)cloud.data.get_buffer();
    std::cout << "npoint = " << npoint << std::endl;
    for (int i=0; i<npoint; i++){
        ptr[0] = cloud_raw->points[i].x;
        ptr[1] = cloud_raw->points[i].y;
        ptr[2] = cloud_raw->points[i].z;
        unsigned char *rgb = (unsigned char *)(ptr+3);
        rgb[0] = cloud_raw->points[i].r;
        rgb[1] = cloud_raw->points[i].g;
        rgb[2] = cloud_raw->points[i].b;
        ptr += 4;
    }
}

void PCDLoader::updateOffsetToCloudXYZ(void)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr clouds(new pcl::PointCloud<pcl::PointXYZ>);
    for (unsigned int i=0; i<m_offset.length(); i++){
        const OpenHRP::PCDOffset& offset = m_offset[i];
        const std::string label(offset.label);
        if( m_clouds_xyz.find(label) != m_clouds_xyz.end() ){
            const hrp::Vector3 center(offset.center.x, offset.center.y, offset.center.z);
            const hrp::Vector3 offsetP(offset.data.position.x, offset.data.position.y, offset.data.position.z);
            const hrp::Matrix33 offsetR(hrp::rotFromRpy(offset.data.orientation.r,
                                                        offset.data.orientation.p,
                                                        offset.data.orientation.y));
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw = m_clouds_xyz[label];
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_new(new pcl::PointCloud<pcl::PointXYZ>);
            int npoint = cloud_raw->points.size();
            cloud_new->points.resize(npoint);
            cloud_new->width = cloud_raw->width;
            cloud_new->height = cloud_raw->height;
            cloud_new->is_dense = cloud_raw->is_dense;
            hrp::Vector3 point, point_new;
            for(int j = 0 ; j < npoint ; j++){
                point << cloud_raw->points[j].x, cloud_raw->points[j].y, cloud_raw->points[j].z;
                point_new = offsetR * (point - center) + center + offsetP;
                cloud_new->points[j].x = point_new[0];
                cloud_new->points[j].y = point_new[1];
                cloud_new->points[j].z = point_new[2];
            }
            *clouds += *cloud_new;
        }
    }
    setCloudXYZ(m_cloud, clouds);
}

void PCDLoader::updateOffsetToCloudXYZRGB(void)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clouds(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (unsigned int i=0; i<m_offset.length(); i++){
        const OpenHRP::PCDOffset& offset = m_offset[i];
        const std::string label(offset.label);
        if( m_clouds_xyzrgb.find(label) != m_clouds_xyzrgb.end() ){
            const hrp::Vector3 center(offset.center.x, offset.center.y, offset.center.z);
            const hrp::Vector3 offsetP(offset.data.position.x, offset.data.position.y, offset.data.position.z);
            const hrp::Matrix33 offsetR(hrp::rotFromRpy(offset.data.orientation.r,
                                                        offset.data.orientation.p,
                                                        offset.data.orientation.y));
            
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_raw = m_clouds_xyzrgb[label];
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_new(new pcl::PointCloud<pcl::PointXYZRGB>);
            int npoint = cloud_raw->points.size();
            cloud_new->points.resize(npoint);
            cloud_new->width = cloud_raw->width;
            cloud_new->height = cloud_raw->height;
            cloud_new->is_dense = cloud_raw->is_dense;
            hrp::Vector3 point, point_new;
            for(int j = 0 ; j < npoint ; j++){
                point << cloud_raw->points[j].x, cloud_raw->points[j].y, cloud_raw->points[j].z;
                point_new = offsetR * (point - center) + center + offsetP;
                cloud_new->points[j].x = point_new[0];
                cloud_new->points[j].y = point_new[1];
                cloud_new->points[j].z = point_new[2];
            }
            *clouds += *cloud_new;
        }
    }
    setCloudXYZRGB(m_cloud, clouds);
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
    if ( !m_path.empty()){
        pcl::PCDReader reader;
        
        if (m_fields=="XYZ"){
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            if (reader.read (m_path, *cloud)){
                std::cerr << m_profile.instance_name << ": failed to load("
                          << m_path << ")" << std::endl;
                m_path = "";
                return RTC::RTC_OK;
            }
            setCloudXYZ(m_cloud, cloud);
        }else if(m_fields=="XYZRGB"){
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            if (reader.read (m_path, *cloud)){
                std::cerr << m_profile.instance_name << ": failed to load("
                          << m_path << ")" << std::endl;
                m_path = "";
                return RTC::RTC_OK;
            }
            setCloudXYZRGB(m_cloud, cloud);
        }else{
            std::cerr << "fields[" << m_fields << "] is not supported" << std::endl;
        }
        m_cloudOut.write();
        m_isOutputOut.write();
        m_path = "";
    }
    
    if( m_offsetIn.isNew() ){
        m_offsetIn.read();
        if( !m_clouds_xyz.empty() ){
            updateOffsetToCloudXYZ();
            m_cloudOut.write();
            m_isOutputOut.write();
        }
        else if( !m_clouds_xyzrgb.empty() ){
            updateOffsetToCloudXYZRGB();
            m_cloudOut.write();
            m_isOutputOut.write();
        }
    }
    
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


bool PCDLoader::load(const std::string& filename, const std::string& label)
{
    pcl::PCDReader reader;
    if( m_fields == "XYZ" ){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (reader.read (filename, *cloud)){
            std::cerr << m_profile.instance_name << ": failed to load("
                      << filename << ")" << std::endl;
            return RTC::RTC_OK;
        }
        else
            std::cout << "Loading " << filename << " with XYZ fields." << std::endl;
        m_clouds_xyz[label] = cloud;
    }
    else if( m_fields == "XYZRGB" ){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        if (reader.read (filename, *cloud)){
            std::cerr << m_profile.instance_name << ": failed to load("
                      << filename << ")" << std::endl;
            return RTC::RTC_OK;
        }
        else
            std::cout << "Loading " << filename << " with XYZRGB fields." << std::endl;
        m_clouds_xyzrgb[label] = cloud;
    }else{
        std::cerr << "fields[" << m_fields << "] is not supported" << std::endl;
    }
}

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


