#ifndef __SCENE_STATE_H__
#define __SCENE_STATE_H__

#include <hrpCorba/OpenHRPCommon.hh>
#include <hrpModel/World.h>
#include "BodyState.h"

class SceneState
{
public:
    void set(hrp::WorldBase& i_world); 
    double time;
    std::vector<BodyState> bodyStates;
    OpenHRP::CollisionSequence collisions;
};

#endif
