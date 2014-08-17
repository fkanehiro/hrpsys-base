// -*- C++ -*-
/*!
 * @file  AverageFilter.cpp
 * @brief Statistical Outlier Removal Filter
 * $Date$
 *
 * $Id$
 */

#include <math.h>
#include <limits>
#include "AverageFilter.h"
#include "pointcloud.hh"

// Module specification
// <rtc-template block="module_spec">
static const char* spec[] =
  {
    "implementation_id", "AverageFilter",
    "type_name",         "AverageFilter",
    "description",       "Average Filter",
    "version",           HRPSYS_PACKAGE_VERSION,
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.resolution", "0.01",
    "conf.default.windowSize", "4",

    ""
  };
// </rtc-template>

AverageFilter::AverageFilter(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_originalIn("original", m_original),
    m_filteredOut("filtered", m_filtered),
    // </rtc-template>
    dummy(0)
{
}

AverageFilter::~AverageFilter()
{
}



RTC::ReturnCode_t AverageFilter::onInitialize()
{
  //std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("resolution", m_resolution, "0.01");
  bindParameter("windowSize", m_windowSize, "4");
  
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
RTC::ReturnCode_t AverageFilter::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AverageFilter::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AverageFilter::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t AverageFilter::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t AverageFilter::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t AverageFilter::onExecute(RTC::UniqueId ec_id)
{
  //std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;

  if (m_originalIn.isNew()){
    m_originalIn.read();

    if (!m_original.data.length()) return RTC::RTC_OK;

    // compute bbox
    float xmin, xmax, ymin, ymax;
    float *src = (float *)m_original.data.get_buffer();
    int npoint = m_original.data.length()/m_original.point_step;
    for (unsigned int i=0; i<npoint; i++){
      if (i==0){
	xmin = xmax = src[0];
	ymin = ymax = src[1];
      }else{
	if (xmin > src[0]) xmin = src[0];
	if (xmax < src[0]) xmax = src[0];
	if (ymin > src[1]) ymin = src[1];
	if (ymax < src[1]) ymax = src[1];
      }
      src += 4;
    }
    //std::cout << "xmin=" << xmin << ", xmax=" << xmax << ", ymin=" << ymin << ", ymax=" << ymax << std::endl;
    
    float xstart = (floor(xmin/m_resolution)-m_windowSize)*m_resolution;
    float ystart = (floor(ymin/m_resolution)-m_windowSize)*m_resolution;
    int nx = (xmax - xstart)/m_resolution+m_windowSize*2;
    int ny = (ymax - ystart)/m_resolution+m_windowSize*2;
    std::vector<float> cell(nx*ny, std::numeric_limits<float>::quiet_NaN());
    //std::cout << "xstart=" << xstart << ", ystart=" << ystart << std::endl;
    //std::cout << "nx=" << nx << ", ny=" << ny << std::endl;
    
    src = (float *)m_original.data.get_buffer();
    for (unsigned int i=0; i<npoint; i++){
#if 1
      int ix = round((src[0] - xstart)/m_resolution);
      int iy = round((src[1] - ystart)/m_resolution);
      cell[ix+nx*iy] = src[2];
#else
      int ix = floor((src[0] - xstart)/m_resolution);
      int iy = floor((src[1] - ystart)/m_resolution);
      cell[ix   + nx*iy    ] = src[2];
      cell[ix+1 + nx*iy    ] = src[2];
      cell[ix+1 + nx*(iy+1)] = src[2];
      cell[ix   + nx*(iy+1)] = src[2];
#endif
      
      src += 4;
    }

    
    m_filtered.data.length(nx*ny*m_filtered.point_step); // shrinked later
    float *dst = (float *)m_filtered.data.get_buffer();
    npoint=0;
    int whalf = m_windowSize/2;
    for (int x=whalf; x<nx-whalf; x++){
      for (int y=whalf; y<ny-whalf; y++){
	int cnt=0;
	float zsum=0;
	//if (isnan(cell[x + nx*y])) continue;
	for (int dx=-whalf; dx<=whalf; dx++){
	  for (int dy=-whalf; dy<whalf; dy++){
	    int index = x + dx + nx*(y + dy);
	    if (!isnan(cell[index])){
	      zsum += cell[index];
	      cnt++;
	    }
	  }
	}
	if (cnt){
	  dst[0] = xstart + m_resolution*x;
	  dst[1] = ystart + m_resolution*y;
	  dst[2] = zsum/cnt;
	  dst += 4;
	  npoint++;
	}
      }
    }
    m_filtered.width = npoint;
    m_filtered.row_step = m_filtered.point_step*m_filtered.width;
    m_filtered.data.length(npoint*m_filtered.point_step);

    m_filteredOut.write();
  }

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t AverageFilter::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AverageFilter::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AverageFilter::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AverageFilter::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AverageFilter::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void AverageFilterInit(RTC::Manager* manager)
  {
    RTC::Properties profile(spec);
    manager->registerFactory(profile,
                             RTC::Create<AverageFilter>,
                             RTC::Delete<AverageFilter>);
  }

};


