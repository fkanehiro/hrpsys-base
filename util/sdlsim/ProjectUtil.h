#include <boost/function.hpp>
#include <hrpModel/World.h>
#include <hrpModel/Body.h>
#include <hrpModel/ConstraintForceSolver.h>
#include "Project.h"
#include "OpenRTMUtil.h"

typedef boost::function2<hrp::BodyPtr, const std::string&, const std::string&> BodyFactory;

void initWorld(Project& prj, BodyFactory &factory, 
               hrp::World<hrp::ConstraintForceSolver>& world);

void initRTS(Project &prj, std::vector<ClockReceiver>& receivers);


