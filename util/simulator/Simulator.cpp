#include "Simulator.h"
#include "hrpsys/util/BodyRTC.h"

Simulator::Simulator(LogManager<SceneState> *i_log) 
  : log(i_log), adjustTime(false)
{
}

void Simulator::init(Project &prj, BodyFactory &factory){
    initWorld(prj, factory, *this, pairs);
    initRTS(prj, receivers);
    std::cout << "number of receivers:" << receivers.size() << std::endl;
    m_totalTime = prj.totalTime();
    m_logTimeStep = prj.logTimeStep();
    m_kinematicsOnly = prj.kinematicsOnly();
    realTime(prj.realTime());

    collisions.length(pairs.size());
    for(size_t colIndex=0; colIndex < pairs.size(); ++colIndex){
        hrp::ColdetLinkPairPtr linkPair = pairs[colIndex];
        hrp::Link *link0 = linkPair->link(0);
        hrp::Link *link1 = linkPair->link(1);
        OpenHRP::LinkPair& pair = collisions[colIndex].pair;
        pair.charName1 = CORBA::string_dup(link0->body->name().c_str());
        pair.charName2 = CORBA::string_dup(link1->body->name().c_str());
        pair.linkName1 = CORBA::string_dup(link0->name.c_str());
        pair.linkName2 = CORBA::string_dup(link1->name.c_str());
    }

    m_nextLogTime = 0;
    appendLog();
}

void Simulator::appendLog()
{
    if (log && currentTime() >= m_nextLogTime){
        state.set(*this, collisions);
        log->add(state);
        m_nextLogTime += m_logTimeStep;
    }
}

void Simulator::checkCollision()
{
    checkCollision(collisions);
}

void Simulator::checkCollision(OpenHRP::CollisionSequence &collisions)
{
    for (unsigned int i=0; i<numBodies(); i++){
        body(i)->updateLinkColdetModelPositions();
    }
    for(size_t colIndex=0; colIndex < pairs.size(); ++colIndex){
        hrp::ColdetLinkPairPtr linkPair = pairs[colIndex];
        OpenHRP::Collision& collision = collisions[colIndex];
        OpenHRP::CollisionPointSequence* pCollisionPoints = &collision.points;
        std::vector<hrp::collision_data>& cdata = linkPair->detectCollisions();
            
        if(cdata.empty()){
            pCollisionPoints->length(0);
        } else {
            int npoints = 0;
            for(unsigned int i = 0; i < cdata.size(); i++) {
                for(int j = 0; j < cdata[i].num_of_i_points; j++){
                    if(cdata[i].i_point_new[j]) npoints++;
                }
            }
            pCollisionPoints->length(npoints);
            int idx = 0;
            for (unsigned int i = 0; i < cdata.size(); i++) {
                hrp::collision_data& cd = cdata[i];
                for(int j=0; j < cd.num_of_i_points; j++){
                    if (cd.i_point_new[j]){
                        OpenHRP::CollisionPoint& point = (*pCollisionPoints)[idx];
                        for(int k=0; k < 3; k++){
                            point.position[k] = cd.i_points[j][k];
                        }
                        for(int k=0; k < 3; k++){
                            point.normal[k] = cd.n_vector[k];
                        }
                        point.idepth = cd.depth;
                        idx++;
                    }
                }
            }
        }
    }
}

bool Simulator::oneStep(){
    ThreadedObject::oneStep();

    if (!currentTime()) gettimeofday(&beginTime, NULL);
    if (adjustTime){
        struct timeval tv;
        gettimeofday(&tv, NULL);
        startTimes.push_back(tv);
        if (startTimes.size() > 1.0/timeStep()){
            startTimes.pop_front();
        }
        if (startTimes.size() >= 2){
            const struct timeval& first = startTimes.front();
            const struct timeval& last  = startTimes.back();
            int realT = (last.tv_sec - first.tv_sec)*1e6
                + (last.tv_usec - first.tv_usec);
            int simT = timeStep()*(startTimes.size()-1)*1e6;
            int usec = simT - realT;
            if (usec > 1000){
                usleep(usec);
            }
        }
    }

    tm_control.begin();
    for (unsigned int i=0; i<numBodies(); i++){
        BodyRTC *bodyrtc = dynamic_cast<BodyRTC *>(body(i).get());
        bodyrtc->writeDataPorts(currentTime());
    }
    
    for (unsigned int i=0; i<numBodies(); i++){
        BodyRTC *bodyrtc = dynamic_cast<BodyRTC *>(body(i).get());
        bodyrtc->readDataPorts();
    }
    for (unsigned int i=0; i<numBodies(); i++){
        BodyRTC *bodyrtc = dynamic_cast<BodyRTC *>(body(i).get());
        bodyrtc->preOneStep();
    }
    for (unsigned int i=0; i<receivers.size(); i++){
        receivers[i].tick(timeStep());
    }
    tm_control.end();

#if 1
    tm_collision.begin();
    checkCollision(collisions);
    tm_collision.end();
#endif

    tm_dynamics.begin();
    constraintForceSolver.clearExternalForces();
    if (m_kinematicsOnly){
        for (unsigned int i=0; i<numBodies(); i++){
            body(i)->calcForwardKinematics();
        }
        currentTime_ += timeStep();
    }else{
        calcNextState(collisions);
    }
    
    for (unsigned int i=0; i<numBodies(); i++){
        BodyRTC *bodyrtc = dynamic_cast<BodyRTC *>(body(i).get());
        bodyrtc->postOneStep();
    }
    appendLog();
    tm_dynamics.end();
    
    if (m_totalTime && currentTime() > m_totalTime){
        struct timeval endTime;
        gettimeofday(&endTime, NULL);
        double realT = (endTime.tv_sec - beginTime.tv_sec)
            + (endTime.tv_usec - beginTime.tv_usec)/1e6;
        printf("total     :%8.3f[s], %8.3f[sim/real]\n",
               realT, m_totalTime/realT);
        printf("controller:%8.3f[s], %8.3f[ms/frame]\n",
               tm_control.totalTime(), tm_control.averageTime()*1000);
        printf("collision :%8.3f[s], %8.3f[ms/frame]\n",
               tm_collision.totalTime(), tm_collision.averageTime()*1000);
        printf("dynamics  :%8.3f[s], %8.3f[ms/frame]\n",
               tm_dynamics.totalTime(), tm_dynamics.averageTime()*1000);
        for (unsigned int i=0; i<numBodies(); i++){
            hrp::BodyPtr body = this->body(i);
            int ntri=0;
            for (unsigned int j=0; j<body->numLinks(); j++){
                hrp::Link *l = body->link(j);
                if (l && l->coldetModel){
                    ntri += l->coldetModel->getNumTriangles();
                }
            }
            printf("num of triangles : %s : %d\n", body->name().c_str(), ntri);
        }
        fflush(stdout);
        return false;
    }else{
        return true;
    }
}

void Simulator::clear()
{
    RTC::Manager* manager = &RTC::Manager::instance();
    for (unsigned int i=0; i<numBodies(); i++){
        BodyRTC *bodyrtc = dynamic_cast<BodyRTC *>(body(i).get());
        bodyrtc->exit();
    }
    manager->cleanupComponents();
    clearBodies();
    constraintForceSolver.clearCollisionCheckLinkPairs();
    setCurrentTime(0.0);
    pairs.clear();
    receivers.clear();
}

void Simulator::addCollisionCheckPair(BodyRTC *bodyPtr1, BodyRTC *bodyPtr2)
{
    int bodyIndex1 = bodyIndex(bodyPtr1->name());
    int bodyIndex2 = bodyIndex(bodyPtr2->name());

    std::vector<hrp::Link*> links1;
    const hrp::LinkTraverse& traverse1 = bodyPtr1->linkTraverse();
    links1.resize(traverse1.numLinks());
    std::copy(traverse1.begin(), traverse1.end(), links1.begin());
    
    std::vector<hrp::Link*> links2;
    const hrp::LinkTraverse& traverse2 = bodyPtr2->linkTraverse();
    links2.resize(traverse2.numLinks());
    std::copy(traverse2.begin(), traverse2.end(), links2.begin());
    
    for(size_t j=0; j < links1.size(); ++j){
        for(size_t k=0; k < links2.size(); ++k){
            hrp::Link* link1 = links1[j];
            hrp::Link* link2 = links2[k];
            
            if(link1 && link2 && link1 != link2){
                constraintForceSolver.addCollisionCheckLinkPair
                    (bodyIndex1, link1, bodyIndex2, link2, 
                     0.5, 0.5, 0.01, 0.0, 0.0);
                pairs.push_back(new hrp::ColdetLinkPair(link1, link2));
            }
        }
    }

    collisions.length(pairs.size());
    for(size_t colIndex=0; colIndex < pairs.size(); ++colIndex){
        hrp::ColdetLinkPairPtr linkPair = pairs[colIndex];
        hrp::Link *link0 = linkPair->link(0);
        hrp::Link *link1 = linkPair->link(1);
        OpenHRP::LinkPair& pair = collisions[colIndex].pair;
        pair.charName1 = CORBA::string_dup(link0->body->name().c_str());
        pair.charName2 = CORBA::string_dup(link1->body->name().c_str());
        pair.linkName1 = CORBA::string_dup(link0->name.c_str());
        pair.linkName2 = CORBA::string_dup(link1->name.c_str());
    }
}

void Simulator::kinematicsOnly(bool flag)
{
    m_kinematicsOnly = flag;
}
