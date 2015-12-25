// -*- C++ -*-
/*!
 * @file  Logger.cpp
 * @brief logger component
 * $Date$
 *
 * $Id$
 */

#include "DataLogger.h"
#include "util/Hrpsys.h"
#include "pointcloud.hh"


typedef coil::Guard<coil::Mutex> Guard;

// Module specification
// <rtc-template block="module_spec">
static const char* nullcomponent_spec[] =
  {
    "implementation_id", "DataLogger",
    "type_name",         "DataLogger",
    "description",       "data logger component",
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


void printData(std::ostream& os, const RTC::Acceleration3D& data)
{
    os << data.ax << " " << data.ay << " " << data.az << " ";
}

void printData(std::ostream& os, const RTC::Velocity2D& data)
{
    os << data.vx << " " << data.vy << " " << data.va << " ";
}

void printData(std::ostream& os, const RTC::Pose3D& data)
{
    os << data.position.x << " " << data.position.y << " " 
       << data.position.z << " " << data.orientation.r << " "
       << data.orientation.p << " " << data.orientation.y << " ";
}

void printData(std::ostream& os, const RTC::AngularVelocity3D& data)
{
    os << data.avx << " " << data.avy << " " << data.avz << " ";
}

void printData(std::ostream& os, const RTC::Point3D& data)
{
    os << data.x << " " << data.y << " " << data.z << " ";
}

void printData(std::ostream& os, const RTC::Orientation3D& data)
{
    os << data.r << " " << data.p << " " << data.y << " ";
}

void printData(std::ostream& os, const PointCloudTypes::PointCloud& data)
{
  uint npoint = data.data.length()/data.point_step;
  os << data.width << " " << data.height << " " << data.type << " " << npoint;
  float *ptr = (float *)data.data.get_buffer();
  std::string type(data.type);
  if (type != "xyz" && type != "xyzrgb"){
    std::cerr << "point cloud type(" << type << ") is not supported" 
	      << std::endl;
    return;
  } 
  for (uint i=0; i<npoint ;i++){
    os << " " << *ptr++ << " " << *ptr++ << " " << *ptr++;
    if (type == "xyzrgb"){
      unsigned char *rgb = (unsigned char *)ptr;
      os << " " << (int)rgb[0] << " " << (int)rgb[1] << " " << (int)rgb[2];
      ptr++;
    }
  }
} 

template<class T>
std::ostream& operator<<(std::ostream& os, const _CORBA_Unbounded_Sequence<T > & data)
{
  for (unsigned int j=0; j<data.length(); j++){
    os << data[j] << " ";
  }
  return os;
}

template <class T>
void printData(std::ostream& os, const T& data)
{
    for (unsigned int j=0; j<data.length(); j++){
        os << data[j] << " ";
    }
}

template <class T>
class LoggerPort : public LoggerPortBase
{
public:
    LoggerPort(const char *name) : m_port(name, m_data) {}
    const char *name(){
        return m_port.name();
    }
    virtual void dumpLog(std::ostream& os){
        os.setf(std::ios::fixed, std::ios::floatfield);
        for (unsigned int i=0; i<m_log.size(); i++){
            // time
            os << std::setprecision(6) << (m_log[i].tm.sec + m_log[i].tm.nsec/1e9) << " ";
            // data
            printData(os, m_log[i].data);
            os << std::endl;
        }
    }
    InPort<T>& port(){
            return m_port;
    }
    void log(){
        if (m_port.isNew()){
            m_port.read();
            m_log.push_back(m_data);
            while (m_log.size() > m_maxLength){
                m_log.pop_front();
            }
        }
    }
    void clear(){
        m_log.clear();
    }
protected:
    InPort<T> m_port;
    T m_data;
    std::deque<T> m_log;
};

class LoggerPortForPointCloud : public LoggerPort<PointCloudTypes::PointCloud>
{
public:
    LoggerPortForPointCloud(const char *name) : LoggerPort<PointCloudTypes::PointCloud>(name) {}
    void dumpLog(std::ostream& os){
        os.setf(std::ios::fixed, std::ios::floatfield);
        for (unsigned int i=0; i<m_log.size(); i++){
            // time
            os << std::setprecision(6) << (m_log[i].tm.sec + m_log[i].tm.nsec/1e9) << " ";
            // data
            printData(os, m_log[i]);
            os << std::endl;
        }
    }
};


DataLogger::DataLogger(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_emergencySignalIn("emergencySignal", m_emergencySignal),
    m_DataLoggerServicePort("DataLoggerService"),
    // </rtc-template>
    m_suspendFlag(false),
	dummy(0)
{
  m_service0.setLogger(this);
}

DataLogger::~DataLogger()
{
}



RTC::ReturnCode_t DataLogger::onInitialize()
{
  std::cerr << "[" << m_profile.instance_name << "] onInitialize()" << std::endl;
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("emergencySignal", m_emergencySignalIn);

  // Set OutPort buffer
  
  // Set service provider to Ports
  m_DataLoggerServicePort.registerProvider("service0", "DataLoggerService", m_service0);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_DataLoggerServicePort);
  
  // </rtc-template>
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  
  // </rtc-template>

  return RTC::RTC_OK;
}



/*
RTC::ReturnCode_t DataLogger::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t DataLogger::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t DataLogger::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t DataLogger::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

RTC::ReturnCode_t DataLogger::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

RTC::ReturnCode_t DataLogger::onExecute(RTC::UniqueId ec_id)
{
  if (ec_id == 0){
    if (m_emergencySignalIn.isNew()){
        m_emergencySignalIn.read();
        time_t sec = time(NULL);
        struct tm *tm_ = localtime(&sec);
        char date[20];
        strftime(date,20, "%Y-%m-%d", tm_);
        char basename[30];
        sprintf(basename, "emglog-%s-%02d%02d",
                date, tm_->tm_hour, tm_->tm_min);
        std::cout << "received emergency signal. saving log files("
                  << basename << ")" << std::endl;
        save(basename);
        while (m_emergencySignalIn.isNew()){
            m_emergencySignalIn.read();
        }
    }
  }else{
    Guard guard(m_suspendFlagMutex);
    
    if (m_suspendFlag) return RTC::RTC_OK;
    
    for (unsigned int i=0; i<m_ports.size(); i++){
      m_ports[i]->log();
    }
  }
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t DataLogger::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t DataLogger::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t DataLogger::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t DataLogger::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t DataLogger::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

bool DataLogger::add(const char *i_type, const char *i_name)
{
  suspendLogging();
  for (unsigned int i=0; i<m_ports.size(); i++){
      if (strcmp(m_ports[i]->name(),i_name) == 0){
          std::cerr << "Logger port named \"" << i_name << "\" already exists"
                    << std::endl;
          resumeLogging();
          return false;
      }
  }  

  LoggerPortBase *new_port=NULL;
  if (strcmp(i_type, "TimedDoubleSeq")==0){
      LoggerPort<TimedDoubleSeq> *lp = new LoggerPort<TimedDoubleSeq>(i_name);
      new_port = lp;
      if (!addInPort(i_name, lp->port())) {
          resumeLogging();
          return false;
      }
  }else if (strcmp(i_type, "TimedLongSeq")==0){
      LoggerPort<TimedLongSeq> *lp = new LoggerPort<TimedLongSeq>(i_name);
      new_port = lp;
      if (!addInPort(i_name, lp->port())) {
          resumeLogging();
          return false;
      }
  }else if (strcmp(i_type, "TimedBooleanSeq")==0){
      LoggerPort<TimedBooleanSeq> *lp = new LoggerPort<TimedBooleanSeq>(i_name);
      new_port = lp;
      if (!addInPort(i_name, lp->port())) {
          resumeLogging();
          return false;
      }
  }else if (strcmp(i_type, "TimedLongSeqSeq")==0){
      LoggerPort<OpenHRP::TimedLongSeqSeq> *lp = new LoggerPort<OpenHRP::TimedLongSeqSeq>(i_name);
      new_port = lp;
      if (!addInPort(i_name, lp->port())) {
          resumeLogging();
          return false;
      }
  }else if (strcmp(i_type, "TimedPoint3D")==0){
      LoggerPort<TimedPoint3D> *lp = new LoggerPort<TimedPoint3D>(i_name);
      new_port = lp;
      if (!addInPort(i_name, lp->port())) {
          resumeLogging();
          return false;
      }
  }else if (strcmp(i_type, "TimedOrientation3D")==0){
      LoggerPort<TimedOrientation3D> *lp = new LoggerPort<TimedOrientation3D>(i_name);
      new_port = lp;
      if (!addInPort(i_name, lp->port())) {
          resumeLogging();
          return false;
      }
  }else if (strcmp(i_type, "TimedAcceleration3D")==0){
      LoggerPort<TimedAcceleration3D> *lp = new LoggerPort<TimedAcceleration3D>(i_name);
      new_port = lp;
      if (!addInPort(i_name, lp->port())) {
          resumeLogging();
          return false;
      }
  }else if (strcmp(i_type, "TimedAngularVelocity3D")==0){
      LoggerPort<TimedAngularVelocity3D> *lp = new LoggerPort<TimedAngularVelocity3D>(i_name);
      new_port = lp;
      if (!addInPort(i_name, lp->port())) {
          resumeLogging();
          return false;
      }
  }else if (strcmp(i_type, "TimedVelocity2D")==0){
      LoggerPort<TimedVelocity2D> *lp = new LoggerPort<TimedVelocity2D>(i_name);
      new_port = lp;
      if (!addInPort(i_name, lp->port())) {
          resumeLogging();
          return false;
      }
  }else if (strcmp(i_type, "TimedPose3D")==0){
      LoggerPort<TimedPose3D> *lp = new LoggerPort<TimedPose3D>(i_name);
      new_port = lp;
      if (!addInPort(i_name, lp->port())) {
          resumeLogging();
          return false;
      }
  }else if (strcmp(i_type, "PointCloud")==0){
    LoggerPort<PointCloudTypes::PointCloud> *lp = new LoggerPortForPointCloud(i_name);
      new_port = lp;
      if (!addInPort(i_name, lp->port())) {
          resumeLogging();
          return false;
      }
  }else{
      std::cout << "DataLogger: unsupported data type(" << i_type << ")"
                << std::endl;
      resumeLogging();
      return false;
  }
  m_ports.push_back(new_port);
  resumeLogging();
  return true;
}

bool DataLogger::save(const char *i_basename)
{
  suspendLogging();
  bool ret = true;
  for (unsigned int i=0; i<m_ports.size(); i++){
    std::string fname = i_basename;
    fname.append(".");
    fname.append(m_ports[i]->name());
    std::ofstream ofs(fname.c_str());
    if (ofs.is_open()){
      m_ports[i]->dumpLog(ofs);
    }else{
      std::cerr << "[" << m_profile.instance_name << "] failed to open(" << fname << ")" << std::endl;
      ret = false;
    }
  }
  if (ret) std::cerr << "[" << m_profile.instance_name << "] Save log to " << i_basename << ".*" << std::endl;
  resumeLogging();
  return ret;
}

bool DataLogger::clear()
{
  suspendLogging();
  for (unsigned int i=0; i<m_ports.size(); i++){
    m_ports[i]->clear();
  }
  std::cerr << "[" << m_profile.instance_name << "] Log cleared" << std::endl;
  resumeLogging();
  return true;
}

void DataLogger::suspendLogging()
{
  Guard guard(m_suspendFlagMutex);
  m_suspendFlag = true;
}

void DataLogger::resumeLogging()
{
  Guard guard(m_suspendFlagMutex);
  m_suspendFlag = false;
}

void DataLogger::maxLength(unsigned int len)
{
  suspendLogging();
  for (unsigned int i=0; i<m_ports.size(); i++){
    m_ports[i]->maxLength(len);
  }
  std::cerr << "[" << m_profile.instance_name << "] Log max length is set to " << len << std::endl;
  resumeLogging();
}

extern "C"
{

  void DataLoggerInit(RTC::Manager* manager)
  {
    RTC::Properties profile(nullcomponent_spec);
    manager->registerFactory(profile,
                             RTC::Create<DataLogger>,
                             RTC::Delete<DataLogger>);
  }

};


