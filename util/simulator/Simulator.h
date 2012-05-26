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

class Simulator : virtual public hrp::World<hrp::ConstraintForceSolver>,
    public ThreadedObject
{
public:
    Simulator(LogManager<SceneState> *i_log);
    void init(Project &prj, BodyFactory &factory);
    bool oneStep();
    void checkCollision(OpenHRP::CollisionSequence &collisions);
    void realTime(bool flag) { adjustTime = flag; }
    void setTotalTime(double time) { m_totalTime = time; }
    double totalTime() { return m_totalTime; }
    void clear();
    void appendLog();
    void addCollisionCheckPair(BodyRTC *b1, BodyRTC *b2);
private:
    LogManager<SceneState> *log;
    std::vector<ClockReceiver> receivers;
    std::vector<hrp::ColdetLinkPairPtr> pairs;
    SceneState state;
    double m_totalTime;
    TimeMeasure tm_dynamics, tm_control, tm_collision;
    bool adjustTime;
    std::deque<struct timeval> startTimes;
    struct timeval beginTime;
};
