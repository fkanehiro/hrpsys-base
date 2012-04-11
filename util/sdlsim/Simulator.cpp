#include "Simulator.h"
#include "GLmodel.h"
#include "BodyRTC.h"

void Simulator::init(Project &prj, BodyFactory &factory, GLscene *i_scene){
    initWorld(prj, factory, world, pairs);
    initRTS(prj, receivers);
    std::cout << "number of receivers:" << receivers.size() << std::endl;
    totalTime = prj.totalTime();
    scene = i_scene;

    OpenHRP::CollisionSequence& collisions = state.collisions;

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

    for (int i=0; i<world.numBodies(); i++){
        bodies.push_back((BodyRTC *)world.body(i).get());
    }
}

void Simulator::checkCollision(OpenHRP::CollisionSequence &collisions)
{
    for(size_t colIndex=0; colIndex < pairs.size(); ++colIndex){
        hrp::ColdetLinkPairPtr linkPair = pairs[colIndex];
        OpenHRP::Collision& collision = collisions[colIndex];
        OpenHRP::CollisionPointSequence* pCollisionPoints = &collision.points;
        linkPair->updatePositions();
        std::vector<hrp::collision_data>& cdata = linkPair->detectCollisions();
            
        if(cdata.empty()){
            pCollisionPoints->length(0);
        } else {
            int npoints = 0;
            for(int i = 0; i < cdata.size(); i++) {
                for(int j = 0; j < cdata[i].num_of_i_points; j++){
                    if(cdata[i].i_point_new[j]) npoints++;
                }
            }
            pCollisionPoints->length(npoints);
            int idx = 0;
            for (int i = 0; i < cdata.size(); i++) {
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

    tm_collision.begin();
    checkCollision(state.collisions);
    tm_collision.end();

    tm_dynamics.begin();
    world.constraintForceSolver.clearExternalForces();
    world.calcNextState(state.collisions);
    
    if (scene){
        state.set(world);
        scene->addState(state);
    }
    tm_dynamics.end();
    
    if (world.currentTime() > totalTime){
        printf("controller:%8.3f[s], %8.3f[ms/frame]\n",
               tm_control.totalTime(), tm_control.averageTime()*1000);
        printf("collision :%8.3f[s], %8.3f[ms/frame]\n",
               tm_collision.totalTime(), tm_collision.averageTime()*1000);
        printf("dynamics  :%8.3f[s], %8.3f[ms/frame]\n",
               tm_dynamics.totalTime(), tm_dynamics.averageTime()*1000);
        return false;
    }else{
        return true;
    }
}

void Simulator::stopSimulation()
{
    totalTime = world.currentTime();
}


