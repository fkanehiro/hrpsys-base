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
    double angle;
    bool isHighGain;
    int NumOfAABB;
    hrp::Vector3 translation;
    hrp::Matrix33 rotation;
};

class ModelItem {
public:
    std::string url;
    std::string rtcName;
    std::map<std::string,JointItem> joint;
};

class RTSItem {
public:
    class rtc{
    public:
        rtc() : period(0.0){}
        std::string name;
        std::string path;
        double period;
    };
    std::map<std::string, rtc> components;
    std::vector<std::pair<std::string, std::string> > connections;
};

class Project{
public:
    Project();
    bool parse(const std::string &filename);
    double timeStep() { return m_timeStep; }
    double totalTime() { return m_totalTime; }
    double gravity() { return m_gravity; }
    bool isEuler() { return m_isEuler; }
    bool kinematicsOnly() { return m_kinematicsOnly; }
    std::map<std::string, ModelItem>& models(){ return m_models; }
    std::vector<CollisionPairItem>& collisionPairs() { return m_collisionPairs; }
    RTSItem &RTS() { return m_rts; }
private:
    double m_timeStep;
    double m_totalTime;
    double m_gravity;
    bool m_isEuler;
    bool m_kinematicsOnly;
    std::map<std::string, ModelItem> m_models;
    std::vector<CollisionPairItem> m_collisionPairs;
    RTSItem m_rts;
};
#endif
