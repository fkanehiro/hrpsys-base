#include <iostream>
#include <rtm/Manager.h>
#include <rtm/RTObject.h>
#include <rtm/CorbaNaming.h>
#include <hrpModel/Link.h>
#include "ProjectUtil.h"

void initWorld(Project& prj, BodyFactory &factory, 
               hrp::World<hrp::ConstraintForceSolver>& world,
               std::vector<hrp::ColdetLinkPairPtr> &pairs)
{
    world.clearBodies();
    world.constraintForceSolver.clearCollisionCheckLinkPairs();
    world.setCurrentTime(0.0);
    
    world.setTimeStep(prj.timeStep());
    if(prj.isEuler()){
        world.setEulerMethod();
    } else {
        world.setRungeKuttaMethod();
    }

    // add bodies
    for (std::map<std::string, ModelItem>::iterator it=prj.models().begin();
         it != prj.models().end(); it++){
        const std::string name
            = it->second.rtcName == "" ? it->first : it->second.rtcName; 
        hrp::BodyPtr body = factory(name, it->second);
        if (body){
            body->setName(name);
            world.addBody(body);
        }
    }

    for (unsigned int i=0; i<prj.collisionPairs().size(); i++){
        const CollisionPairItem &cpi = prj.collisionPairs()[i];
        int bodyIndex1 = world.bodyIndex(cpi.objectName1);
        if (bodyIndex1 < 0){
            // different name is used for RTC
            if (prj.models().find(cpi.objectName1) != prj.models().end()){
                bodyIndex1 
                    = world.bodyIndex(prj.models()[cpi.objectName1].rtcName);
            }
        }
        int bodyIndex2 = world.bodyIndex(cpi.objectName2);
        if (bodyIndex2 < 0){
            // different name is used for RTC
            if (prj.models().find(cpi.objectName2) != prj.models().end()){
                bodyIndex2 
                    = world.bodyIndex(prj.models()[cpi.objectName2].rtcName);
            }
        }

        if(bodyIndex1 >= 0 && bodyIndex2 >= 0){
            hrp::BodyPtr bodyPtr1 = world.body(bodyIndex1);
            hrp::BodyPtr bodyPtr2 = world.body(bodyIndex2);

            std::vector<hrp::Link*> links1;
            if(cpi.jointName1.empty()){
                const hrp::LinkTraverse& traverse = bodyPtr1->linkTraverse();
                links1.resize(traverse.numLinks());
                std::copy(traverse.begin(), traverse.end(), links1.begin());
            } else {
                links1.push_back(bodyPtr1->link(cpi.jointName1));
            }

            std::vector<hrp::Link*> links2;
            if(cpi.jointName2.empty()){
                const hrp::LinkTraverse& traverse = bodyPtr2->linkTraverse();
                links2.resize(traverse.numLinks());
                std::copy(traverse.begin(), traverse.end(), links2.begin());
            } else {
                links2.push_back(bodyPtr2->link(cpi.jointName2));
            }

            for(size_t j=0; j < links1.size(); ++j){
                for(size_t k=0; k < links2.size(); ++k){
                    hrp::Link* link1 = links1[j];
                    hrp::Link* link2 = links2[k];

                    if(link1 && link2 && link1 != link2 
                       && link1->parent != link2 && link1 != link2->parent){
                        world.constraintForceSolver.addCollisionCheckLinkPair
                            (bodyIndex1, link1, bodyIndex2, link2, 
                             cpi.staticFriction, cpi.slidingFriction, 0.01, 0.0, 0.0);
                        pairs.push_back(new hrp::ColdetLinkPair(link1, link2));
                    }
                }
            }
        }
    }

    world.enableSensors(true);

    int nBodies = world.numBodies();
    for(int i=0; i < nBodies; ++i){
        hrp::BodyPtr bodyPtr = world.body(i);
        bodyPtr->initializeConfiguration();
    }
        
    for (std::map<std::string, ModelItem>::iterator it=prj.models().begin();
         it != prj.models().end(); it++){
        const std::string name
            = it->second.rtcName == "" ? it->first : it->second.rtcName; 
        hrp::BodyPtr body = world.body(name);
        for (std::map<std::string, JointItem>::iterator it2=it->second.joint.begin();
             it2 != it->second.joint.end(); it2++){
            hrp::Link *link = body->link(it2->first);
            if (!link) continue;
            if (link->isRoot()){
                link->p = it2->second.translation;
                link->setAttitude(it2->second.rotation);
            }else{
                link->q = it2->second.angle;
            }
        }
        body->calcForwardKinematics();
    }
    world.initialize();
}

RTC::RTObject_var findRTC(const std::string &rtcName)
{
    RTC::Manager& manager = RTC::Manager::instance();
    std::string nameServer = manager.getConfig()["corba.nameservers"];
    int comPos = nameServer.find(",");
    if (comPos < 0){
        comPos = nameServer.length();
    }
    nameServer = nameServer.substr(0, comPos);
    RTC::CorbaNaming naming(manager.getORB(), nameServer.c_str());
    CosNaming::Name name;
    name.length(1);
    name[0].id = CORBA::string_dup(rtcName.c_str());
    name[0].kind = CORBA::string_dup("rtc");
    try{
        CORBA::Object_ptr obj = naming.resolve(name);
        return RTC::RTObject::_narrow(obj);
    }catch(...){
        return NULL;
    }
}

void initRTS(Project &prj, std::vector<ClockReceiver>& receivers)
{
    RTC::Manager& manager = RTC::Manager::instance();

    RTSItem& rts = prj.RTS();
    // load factories
    for (std::map<std::string, RTSItem::rtc>::iterator it 
             = rts.components.begin(); it != rts.components.end(); it++){
        std::string path = it->second.path;
        if (path == "") continue;
        path += ".so";
        std::cout << "loading " << path << std::endl; 
        std::string initfunc = it->second.name + "Init";
        manager.load(path.c_str(), initfunc.c_str());
    }
    // create components
    for (std::map<std::string, RTSItem::rtc>::iterator it 
             = rts.components.begin(); it != rts.components.end(); it++){
        RTC::RTObject_impl *rtc = manager.getComponent(it->first.c_str());
        if (!rtc){
            if (it->second.name == ""){
                std::cerr << "factory name for " << it->first << " is not defined" << std::endl;
                continue;
            }                
            std::cout << "creating " << it->first << std::endl;
            std::string args = it->second.name + "?instance_name=" + it->first; 
            rtc = manager.createComponent(args.c_str());
        }
        RTC::ExecutionContextList_var eclist = rtc->get_owned_contexts();
        for(CORBA::ULong i=0; i < eclist->length(); ++i){
            if(!CORBA::is_nil(eclist[i])){
                OpenRTM::ExtTrigExecutionContextService_var execContext = OpenRTM::ExtTrigExecutionContextService::_narrow(eclist[i]);
                if(!CORBA::is_nil(execContext)){
                    std::cout << it->first << ":" << it->second.period << std::endl;
                    receivers.push_back(ClockReceiver(execContext, it->second.period));
                    execContext->activate_component(rtc->getObjRef());
                }
            }
        }
    }
    // make connections
    for (std::vector<std::pair<std::string, std::string> >::iterator it
             = rts.connections.begin(); it != rts.connections.end(); it++){
        std::cout << "making a connection between "
                  << it->first << " and " << it->second << std::endl;
        int pos1 = it->first.find('.');
        std::string comp1 = it->first.substr(0, pos1);
        std::string port1 = it->first;
        int pos2 = it->second.find('.');
        std::string comp2 = it->second.substr(0, pos2);
        std::string port2 = it->second;

        RTC::RTObject_var rtc1 = findRTC(comp1);
        if (!rtc1){
            std::cerr << "can't find a component named " << comp1 << std::endl;
            return;
        }
        RTC::RTObject_var rtc2 = findRTC(comp2);
        if (!rtc2){
            std::cerr << "can't find a component named " << comp2 << std::endl;
            return;
        }
        RTC::PortServiceList_var ports1 = rtc1->get_ports();
        RTC::PortServiceList_var ports2 = rtc2->get_ports();

        RTC::PortService_ptr portObj1=NULL, portObj2=NULL; 
        for(CORBA::ULong i = 0; i < ports1->length(); ++i ){
            RTC::PortProfile_var profile = ports1[i]->get_port_profile();
            std::string portName(profile->name);
            if (portName == port1){
                portObj1 = ports1[i];
                break;
            }
        }
        if (!portObj1) {
            std::cerr << "can't find a port named " << port1 << std::endl;
            return; 
        }
        for(CORBA::ULong i = 0; i < ports2->length(); ++i ){
            RTC::PortProfile_var profile = ports2[i]->get_port_profile();
            std::string portName(profile->name);
            if (portName == port2){
                portObj2 = ports2[i];
                break;
            }
        }
        if (!portObj2) {
            std::cerr << "can't find a port named " << port2 << std::endl;
            return;
        }
        connectPorts(portObj1, portObj2);
    }
}
