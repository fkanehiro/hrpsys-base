#ifndef __SCENE_STATE_H__
#define __SCENE_STATE_H__

#include <hrpCorba/OpenHRPCommon.hh>
#include <hrpModel/World.h>
#include "BodyState.h"

class CollisionInfo
{
public:
    double position[3];
    double normal[3];
    double idepth;
};

class SceneState
{
public:
    void set(hrp::WorldBase& i_world, OpenHRP::CollisionSequence& i_collisions); 
    double time;
    std::vector<BodyState> bodyStates;
    std::vector<CollisionInfo> collisions;
};

#endif
