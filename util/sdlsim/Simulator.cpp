#include <hrpModel/OnlineViewerUtil.h>
#include "Simulator.h"
#include "GLmodel.h"
#include "BodyRTC.h"

void Simulator::init(Project &prj, BodyFactory &factory, GLscene *i_scene){
    initWorld(prj, factory, world);
    initRTS(prj, receivers);
    std::cout << "number of receivers:" << receivers.size() << std::endl;
    initWorldState(state, world);
    totalTime = prj.totalTime();
    scene = i_scene;
    if (scene) scene->reserveLog(prj.totalTime()/prj.timeStep()+2);
    for (int i=0; i<world.numBodies(); i++){
        bodies.push_back((BodyRTC *)world.body(i).get());
    }
}

bool Simulator::oneStep(){
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
    
    if (scene){
        getWorldState(state, world);
        scene->addState(state);
    }
    tm_dynamics.end();
    
    if (world.currentTime() > totalTime){
        std::cout << "controller:" << tm_control.totalTime() 
                  << "[s], " << tm_control.averageTime()*1000 
                  << "[ms/frame]" << std::endl;
        std::cout << "dynamics  :" << tm_dynamics.totalTime() 
                  << "[s], " << tm_dynamics.averageTime()*1000 
                  << "[ms/frame]" << std::endl;
        return false;
    }else{
        return true;
    }
}

void Simulator::stopSimulation()
{
    totalTime = world.currentTime();
}


