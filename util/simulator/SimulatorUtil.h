#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <rtm/CorbaNaming.h>
#include <hrpModel/World.h>
#include <hrpModel/ConstraintForceSolver.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpUtil/TimeMeasure.h>
#include "Project.h"
#include "BodyRTC.h"
#include "ProjectUtil.h"
#include "GLmodel.h"


hrp::BodyPtr createBody(const std::string& name, const std::string& url,
                        std::vector<BodyRTCPtr> *bodies,
                        RTC::CorbaNaming *naming)
{
    std::cout << "createBody(" << name << "," << url << ")" << std::endl;
    RTC::Manager& manager = RTC::Manager::instance();
    std::string args = "BodyRTC?instance_name="+name;
    BodyRTCPtr body = (BodyRTC *)manager.createComponent(args.c_str());
    if (!loadBodyFromModelLoader(body, url.c_str(), 
                                 CosNaming::NamingContext::_duplicate(naming->getRootContext()),
                                 true)){
        std::cerr << "failed to load model[" << url << "]" << std::endl;
        manager.deleteComponent(body.get());
        return hrp::BodyPtr();
    }else{
        body->createDataPorts();
        bodies->push_back(body);
        return body;
    }
}

class Simulator
{
public:
    void init(Project &prj, GLscene *i_scene, RTC::CorbaNaming &naming){
        BodyFactory factory = boost::bind(createBody, _1, _2, &bodies, &naming);
        initWorld(prj, factory, world);
        initRTS(prj, receivers);
        std::cout << "number of receivers:" << receivers.size() << std::endl;
        initWorldState(state, world);
        totalTime = prj.totalTime();
        scene = i_scene;
    }

    bool oneStep(){
        tm_control.begin();
        for (unsigned int i=0; i<bodies.size(); i++){
            bodies[i]->writeDataPorts();
        }
        
        for (unsigned int i=0; i<bodies.size(); i++){
            bodies[i]->readDataPorts();
        }
        
        for (unsigned int i=0; i<receivers.size(); i++){
            receivers[i].tick(world.timeStep());
        }
        tm_control.end();
        
        tm_dynamics.begin();
        world.constraintForceSolver.clearExternalForces();
        world.calcNextState(state.collisions);

        getWorldState(state, world);
        scene->addState(state);
        tm_dynamics.end();

        if (world.currentTime() > totalTime){
            std::cout << "controller:" << tm_control.totalTime() 
                      << "[s], " << tm_control.averageTime()*1000 
                      << "[ms/frame]" << std::endl;
            std::cout << "dynamics  :" << tm_dynamics.totalTime() 
                      << "[s], " << tm_dynamics.averageTime()*1000 
                      << "[ms/frame]" << std::endl;
        }

        return world.currentTime() <= totalTime;
    }

    TimeMeasure tm_collision, tm_dynamics, tm_control;
    hrp::World<hrp::ConstraintForceSolver> world;
    std::vector<BodyRTCPtr> bodies; 
    std::vector<ClockReceiver> receivers;
    OpenHRP::WorldState state;
    double totalTime;
    GLscene *scene;
};
