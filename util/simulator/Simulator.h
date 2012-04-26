#include <hrpCorba/OpenHRPCommon.hh>
#include <hrpModel/World.h>
#include <hrpModel/ConstraintForceSolver.h>
#include <hrpUtil/TimeMeasure.h>
#include "util/Project.h"
#include "util/ThreadedObject.h"
#include "util/LogManager.h"
#include "util/ProjectUtil.h"
#include "SceneState.h"

class BodyRTC;
class SDL_Thread;

class Simulator : public ThreadedObject
{
public:
    Simulator();
    void init(Project &prj, BodyFactory &factory, 
              LogManager<SceneState> *i_log);
    bool oneStep();
    void checkCollision(OpenHRP::CollisionSequence &collisions);
    void realTime(bool flag) { adjustTime = flag; }
    void totalTime(double time) { m_totalTime = time; }
    double currentTime() { return world.currentTime(); }
private:
    hrp::World<hrp::ConstraintForceSolver> world;
    LogManager<SceneState> *log;
    std::vector<BodyRTC *> bodies; 
    std::vector<ClockReceiver> receivers;
    std::vector<hrp::ColdetLinkPairPtr> pairs;
    SceneState state;
    double m_totalTime;
    TimeMeasure tm_dynamics, tm_control, tm_collision;
    bool adjustTime;
    std::deque<struct timeval> startTimes;
    struct timeval beginTime;
};
