#ifndef __PROJECT_H__
#define __PROJECT_H__

#include <string>
#include <vector>
#include <map>
#include <hrpUtil/Tvmet3d.h>

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
    std::map<std::string,JointItem> joint;
};

class Project{
public:
    Project();
    bool parse(const std::string &filename);
    double timeStep() { return m_timeStep; }
    bool isEuler() { return m_isEuler; }
    std::map<std::string, ModelItem>& models(){ return m_models; }
    std::vector<CollisionPairItem>& collisionPairs() { return m_collisionPairs; }
private:
    double m_timeStep;
    bool m_isEuler;
    std::map<std::string, ModelItem> m_models;
    std::vector<CollisionPairItem> m_collisionPairs;
};
#endif
