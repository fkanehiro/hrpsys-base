#include "SceneState.h"

using namespace hrp;
using namespace OpenHRP;

void SceneState::set(hrp::WorldBase& i_world, OpenHRP::CollisionSequence& i_collisions)
{
    time = i_world.currentTime();
    bodyStates.resize(i_world.numBodies());
    for (unsigned int i=0; i<i_world.numBodies(); i++){
        BodyPtr body = i_world.body(i);
        bodyStates[i].set(body);
    }
    size_t n=0;
    for (size_t i=0; i<i_collisions.length(); i++){
        n += i_collisions[i].points.length();
    }
    collisions.resize(n);
    size_t index=0;
    for (size_t i=0; i<i_collisions.length(); i++){
        Collision &col = i_collisions[i];
        for (size_t j=0; j<col.points.length(); j++){
            CollisionInfo& ci=collisions[index++];
            CollisionPoint& cp=col.points[j];
            for (int k=0; k<3; k++){
                ci.position[k] = cp.position[k];
                ci.normal[k] = cp.normal[k];
            }
            ci.idepth = cp.idepth;
        }
    }
}
