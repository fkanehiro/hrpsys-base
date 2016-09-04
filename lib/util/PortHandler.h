#ifndef __PORT_HANDLER_H__
#define __PORT_HANDLER_H__

#include <rtm/idl/InterfaceDataTypes.hh>
#include "hrpsys/idl/HRPDataTypes.hh"
#include "hrpsys/idl/pointcloud.hh"
#include "BodyRTC.h"

namespace hrp{
    class ForceSensor;
    class RateGyroSensor;
    class AccelSensor;
    class RangeSensor;
    class VisionSensor;
};

class InPortHandlerBase
{
public:
    virtual void update()=0;
};

class OutPortHandlerBase
{
public:
    virtual void update(double time)=0;
};

template<class T>
class InPortHandler : public InPortHandlerBase
{
public:
    InPortHandler(RTC::DataFlowComponentBase *i_rtc, const char *i_portName) : 
        m_port(i_portName, m_data){
        i_rtc->addInPort(i_portName, m_port);
    }
protected:
    T m_data;
    RTC::InPort<T> m_port;
};

template<class T>
class OutPortHandler : public OutPortHandlerBase
{
public:
    OutPortHandler(RTC::DataFlowComponentBase *i_rtc,
                   const char *i_portName) : 
        m_port(i_portName, m_data){
        i_rtc->addOutPort(i_portName, m_port);
    }
    void write(double time){
        m_data.tm.sec = time;
        m_data.tm.nsec = (time - m_data.tm.sec)*1e9;
        m_port.write();
    }
protected:
    T m_data;
    RTC::OutPort<T> m_port;
};

class JointInPortHandler : public InPortHandler<RTC::TimedDoubleSeq>
{
public:
    JointInPortHandler(RTC::DataFlowComponentBase *i_rtc,
                       const char *i_portName,
                       const std::vector<hrp::Link *> &i_joints,
                       std::vector<OpenHRP::RobotHardwareService::SwitchStatus> *i_servo);
protected:
    std::vector<hrp::Link *> m_joints;
    std::vector<OpenHRP::RobotHardwareService::SwitchStatus> &m_servo;
};

class JointOutPortHandler : public OutPortHandler<RTC::TimedDoubleSeq>
{
public:
    JointOutPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                        const char *i_portName,
                        const std::vector<hrp::Link *> &i_joints);
protected:
    std::vector<hrp::Link *> m_joints;
};

class JointValueInPortHandler : public JointInPortHandler
{
public:
    JointValueInPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                            const char *i_portName,
                            const std::vector<hrp::Link *> &i_joints,
                            std::vector<OpenHRP::RobotHardwareService::SwitchStatus> *i_servo);
    void update();
};

class JointValueOutPortHandler : public JointOutPortHandler
{
public:
    JointValueOutPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                             const char *i_portName,
                             const std::vector<hrp::Link *> &i_joints);
    void update(double time);
};

class JointVelocityInPortHandler : public JointInPortHandler
{
public:
    JointVelocityInPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                               const char *i_portName,
                               const std::vector<hrp::Link *> &i_joints,
                               std::vector<OpenHRP::RobotHardwareService::SwitchStatus> *i_servo);
    void update();
};

class JointVelocityOutPortHandler : public JointOutPortHandler
{
public:
    JointVelocityOutPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                                const char *i_portName,
                                const std::vector<hrp::Link *> &i_joints);
    void update(double time);
};

class JointAccelerationInPortHandler : public JointInPortHandler
{
public:
    JointAccelerationInPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                                   const char *i_portName,
                                   const std::vector<hrp::Link *> &i_joints,
                                   std::vector<OpenHRP::RobotHardwareService::SwitchStatus> *i_servo);
    void update();
};

class JointAccelerationOutPortHandler : public JointOutPortHandler
{
public:
    JointAccelerationOutPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                                    const char *i_portName,
                                    const std::vector<hrp::Link *> &i_joints);
    void update(double time);
};

class JointTorqueInPortHandler : public JointInPortHandler
{
public:
    JointTorqueInPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                             const char *i_portName,
                             const std::vector<hrp::Link *> &i_joints,
                             std::vector<OpenHRP::RobotHardwareService::SwitchStatus> *i_servo);
    void update();
};

class JointTorqueOutPortHandler : public JointOutPortHandler
{
public:
    JointTorqueOutPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                              const char *i_portName,
                              const std::vector<hrp::Link *> &i_joints);
    void update(double time);
private:
    hrp::Link *m_link;
};

class AbsTransformInPortHandler : public InPortHandler<RTC::TimedPose3D>
{
public:
    AbsTransformInPortHandler(RTC::DataFlowComponentBase *i_rtc,
                              const char *i_portName,
                              hrp::Link *i_link);
    void update();
private:
    hrp::Link *m_link;
};

class AbsVelocityInPortHandler : public InPortHandler<RTC::TimedDoubleSeq>
{
public:
    AbsVelocityInPortHandler(RTC::DataFlowComponentBase *i_rtc,
                             const char *i_portName,
                             hrp::Link *i_link);
    void update();
private:
    hrp::Link *m_link;
};

class AbsAccelerationInPortHandler : public InPortHandler<RTC::TimedDoubleSeq>
{
public:
    AbsAccelerationInPortHandler(RTC::DataFlowComponentBase *i_rtc,
                                 const char *i_portName,
                                 hrp::Link *i_link);
    void update();
private:
    hrp::Link *m_link;
};

class FrameRateInPortHandler : public InPortHandler<RTC::TimedDouble>
{
public:
    FrameRateInPortHandler(RTC::DataFlowComponentBase *i_rtc,
                           const char *i_portName,
                           hrp::VisionSensor *i_sensor);
    void update();
private:
    hrp::VisionSensor *m_sensor;
};

class LightSwitchInPortHandler : public InPortHandler<RTC::TimedBoolean>
{
public:
    LightSwitchInPortHandler(RTC::DataFlowComponentBase *i_rtc,
                             const char *i_portName,
                             hrp::Light *i_light);
    void update();
private:
    hrp::Light *m_light;
};

class AbsTransformOutPortHandler : public OutPortHandler<RTC::TimedPose3D>
{
public:
    AbsTransformOutPortHandler(RTC::DataFlowComponentBase *i_rtc,
                               const char *i_portName,
                               hrp::Link *i_link);
    AbsTransformOutPortHandler(RTC::DataFlowComponentBase *i_rtc,
                               const char *i_portName,
                               hrp::Sensor *i_sensor);
    void update(double time);
private:
    hrp::Link *m_link;
    hrp::Sensor *m_sensor;
};

class AbsVelocityOutPortHandler : public OutPortHandler<RTC::TimedDoubleSeq>
{
public:
    AbsVelocityOutPortHandler(RTC::DataFlowComponentBase *i_rtc,
                              const char *i_portName,
                              hrp::Link *i_link);
    void update(double time);
private:
    hrp::Link *m_link;
};

class AbsAccelerationOutPortHandler : 
    public OutPortHandler<RTC::TimedDoubleSeq>
{
public:
    AbsAccelerationOutPortHandler(RTC::DataFlowComponentBase *i_rtc,
                                  const char *i_portName,
                                  hrp::Link *i_link);
    void update(double time);
private:
    hrp::Link *m_link;
};

template<class T, class S>
class SensorPortHandler : public OutPortHandler<S>
{
public:
    SensorPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                      const char *i_portName, T *i_sensor) : 
        OutPortHandler<S>(i_rtc, i_portName),
        m_sensor(i_sensor){
    }
protected:
    T *m_sensor;
};

class ForceSensorPortHandler : 
    public SensorPortHandler<hrp::ForceSensor, RTC::TimedDoubleSeq>
{
public:
    ForceSensorPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                           const char *i_portName,
                           hrp::ForceSensor *i_sensor);
    void update(double time);
};

class RateGyroSensorPortHandler : 
    public SensorPortHandler<hrp::RateGyroSensor, RTC::TimedAngularVelocity3D>
{
public:
    RateGyroSensorPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                              const char *i_portName,
                              hrp::RateGyroSensor *i_sensor);
    void update(double time);
private:        
};

class AccelSensorPortHandler : 
    public SensorPortHandler<hrp::AccelSensor, RTC::TimedAcceleration3D>
{
public:
    AccelSensorPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                           const char *i_portName,
                           hrp::AccelSensor *i_sensor);
    void update(double time);
private:        
};

class RangeSensorPortHandler : 
    public SensorPortHandler<hrp::RangeSensor, RTC::RangeData>
{
public:
    RangeSensorPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                           const char *i_portName,
                           hrp::RangeSensor *i_sensor);
    void update(double time);
private:        
};

class VisionSensorPortHandler : 
    public SensorPortHandler<hrp::VisionSensor, Img::TimedCameraImage>
{
public:
    VisionSensorPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                            const char *i_portName,
                           hrp::VisionSensor *i_sensor);
    void update(double time);
private:        
};

class PointCloudPortHandler :
    public SensorPortHandler<hrp::VisionSensor, PointCloudTypes::PointCloud>
{
public:
    PointCloudPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                          const char *i_portName,
                          hrp::VisionSensor *i_sensor);
    void update(double time);
private:        
    std::string m_pcFormat;
};

class EmergencySignalPortHandler : public OutPortHandler<RTC::TimedLong>
{
public:
    EmergencySignalPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                               const char *i_portName,
                               BodyRTC *i_body);
    void update(double time);
protected:
    BodyRTC *m_body;
};

class ServoStatePortHandler : public OutPortHandler<OpenHRP::TimedLongSeqSeq>
{
public:
    ServoStatePortHandler(RTC::DataFlowComponentBase *i_rtc,
                          const char *i_portName,
                          BodyRTC *i_body);
    void update(double time);
protected:
    BodyRTC *m_body;
    OpenHRP::RobotHardwareService::RobotState* rs;
};

#endif
