#include <hrpCorba/OpenHRPCommon.hh>
#include <hrpModel/World.h>
#include <hrpModel/ConstraintForceSolver.h>
#include <hrpUtil/TimeMeasure.h>
#include <SDL/SDL.h>
#include "util/Project.h"
#include "ProjectUtil.h"
#include "SceneState.h"

class GLscene;
class BodyRTC;

class Simulator
{
public:
    Simulator();
    void init(Project &prj, BodyFactory &factory, GLscene *i_scene);
    bool oneStep();
    void start();
    void stop();
    void checkCollision(OpenHRP::CollisionSequence &collisions);

    hrp::World<hrp::ConstraintForceSolver> world;
    GLscene *scene;
private:
    std::vector<BodyRTC *> bodies; 
    std::vector<ClockReceiver> receivers;
    std::vector<hrp::ColdetLinkPairPtr> pairs;
    SceneState state;
    double totalTime;
    TimeMeasure tm_dynamics, tm_control, tm_collision;
    SDL_Thread *m_thread;
};
