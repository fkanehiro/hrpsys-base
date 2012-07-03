#ifndef __PROJECT_H__
#define __PROJECT_H__

#include <string>
#include <vector>
#include <map>
#include <hrpUtil/Eigen3d.h>

class CollisionPairItem {
public:
    std::string objectName1;
    std::string objectName2;
    std::string jointName1;
    std::string jointName2;
    double slidingFriction;
    double staticFriction;
    double cullingThresh;
    std::string sprintDamperModel;

    CollisionPairItem() {
        jointName1 = "";
        jointName2 = "";
        slidingFriction = 0.5;
        staticFriction = 0.5;
        cullingThresh = 0.01;
        sprintDamperModel = "false";
    }
};

class JointItem {
public:
    JointItem() : 
        angle(0), isHighGain(false), NumOfAABB(0), 
        translation(hrp::Vector3::Zero()), 
        rotation(hrp::Matrix33::Identity()){}
    double angle;
    bool isHighGain;
    int NumOfAABB;
    hrp::Vector3 translation;
    hrp::Matrix33 rotation;
};

class ModelItem {
public:
    std::string url;
    std::map<std::string,JointItem> joint;
    std::string rtcName;
    std::vector<std::string > inports;
    std::vector<std::string > outports;
};

class RTSItem {
public:
    class rtc{
    public:
        rtc() : period(0.0){}
        std::string name;
        std::string path;
        double period;
        std::vector<std::pair<std::string, std::string> > configuration;
    };
    std::map<std::string, rtc> components;
    std::vector<std::pair<std::string, std::string> > connections;
};

class RobotHardwareClientView
{
public:
    RobotHardwareClientView() : 
        hostname("localhost"), RobotHardwareName("RobotHardware0"),
        StateHolderName("StateHolder0"), port(2809), interval(100){}
    std::string hostname, RobotHardwareName, StateHolderName;
    int port, interval;
};

class ThreeDView
{
public:
    ThreeDView();
    bool showScale, showCoM, showCollision;  
    double T[16];
};

class Project{
public:
    Project();
    bool parse(const std::string &filename);
    double timeStep() { return m_timeStep; }
    double totalTime() { return m_totalTime; }
    void totalTime(double time) { m_totalTime = time; }
    double logTimeStep() { return m_logTimeStep; }
    double gravity() { return m_gravity; }
    bool isEuler() { return m_isEuler; }
    bool kinematicsOnly() { return m_kinematicsOnly; }
    bool realTime() { return m_realTime; }
    void realTime(bool flag) { m_realTime = flag; }
    std::map<std::string, ModelItem>& models(){ return m_models; }
    std::vector<CollisionPairItem>& collisionPairs() { return m_collisionPairs; }
    RTSItem &RTS() { return m_rts; }
    RobotHardwareClientView &RobotHardwareClient() { return m_rhview; }
    ThreeDView &view() { return m_3dview; }
private:
    double m_timeStep, m_logTimeStep;
    double m_totalTime;
    double m_gravity;
    bool m_isEuler;
    bool m_kinematicsOnly;
    bool m_realTime;
    std::map<std::string, ModelItem> m_models;
    std::vector<CollisionPairItem> m_collisionPairs;
    RTSItem m_rts;
    RobotHardwareClientView m_rhview;
    ThreeDView m_3dview;
};
#endif
