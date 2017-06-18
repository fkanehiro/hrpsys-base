#include <boost/function.hpp>
#include <hrpModel/World.h>
#include <hrpModel/Body.h>
#include <hrpModel/ConstraintForceSolver.h>
#include <hrpModel/ColdetLinkPair.h>
#include "hrpsys/util/Project.h"
#include "hrpsys/util/OpenRTMUtil.h"

typedef boost::function2<hrp::BodyPtr, const std::string&, const ModelItem&>
    BodyFactory;

void initWorld(Project& prj, BodyFactory& factory,
               hrp::World<hrp::ConstraintForceSolver>& world,
               std::vector<hrp::ColdetLinkPairPtr>& pairs);

void initRTS(Project& prj, std::vector<ClockReceiver>& receivers);
