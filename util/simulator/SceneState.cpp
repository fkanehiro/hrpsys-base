#include "SceneState.h"

using namespace hrp;
using namespace OpenHRP;

void SceneState::set(hrp::WorldBase& i_world)
{
    time = i_world.currentTime();
    bodyStates.resize(i_world.numBodies());
    for (int i=0; i<i_world.numBodies(); i++){
        BodyPtr body = i_world.body(i);
        bodyStates[i].set(body);
    }
}
