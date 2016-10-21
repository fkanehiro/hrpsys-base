#include <hrpModel/ModelLoaderUtil.h>
#include <hrpModel/Link.h>
#include <hrpUtil/OnlineViewerUtil.h>
#include <hrpModel/OnlineViewerUtil.h>
#include <iostream>
#include <fstream>
#include "scc.h"

using namespace OpenHRP;
using namespace hrp;

int main(int argc, char *argv[])
{
    if (argc < 3){
        std::cerr << "Usage: " << argv[0] << "[VRML model] [log file] [-olv] [linkName1:linkNam2 ...]" 
                  << std::endl;
        return 1;
    }

    hrp::LinkNamePairList pairs;
    bool useOLV=false;
    for (int i=3; i<argc; i++){
        std::string str = argv[i];
        std::string::size_type pos;
        if ((pos = str.find(":")) != std::string::npos){
            std::string link1 = str.substr(0, pos);
            std::string link2 = str.substr(pos+1);
            std::cerr << "[" << link1 << "],[" << link2 << "]" << std::endl;
            pairs.push_back(std::make_pair(link1, link2));
        }else if(str=="-olv"){
            useOLV=true;
        }
    }

    hrp::BodyPtr robot = hrp::BodyPtr(new hrp::Body());
    if (!loadBodyFromModelLoader(robot, argv[1], 
                                 argc, argv, true)){
        std::cerr << "Error: failed to load model[" << argv[1] << "]" 
                  << std::endl;
        return 2;
    }

    hrp::SelfCollisionChecker scc(robot, pairs);
    std::cerr << scc.numOfCheckPairs() << " pairs are defined" << std::endl;

    OpenHRP::OnlineViewer_var olv;
    if (useOLV){
        olv = getOnlineViewer(argc, argv);
        olv->load(robot->name().c_str(), argv[1]);
        olv->clearLog();
    }
    OpenHRP::WorldState wstate;
    wstate.characterPositions.length(1);
    setupCharacterPosition(wstate.characterPositions[0], robot);
    
    std::ifstream ifs(argv[2]);
    double tm, q[robot->numJoints()];

    ifs >> tm;
    while (!ifs.eof()){
        for (unsigned int i=0; i<robot->numJoints(); i++){
            ifs >> q[i];
        }
        pairs = scc.check(q);
        for (unsigned int i=0; i<pairs.size(); i++){
            std::cout << tm << " " << pairs[i].first << ":" << pairs[i].second
                      << std::endl;
        }
        if (useOLV){
            wstate.time = tm;
            updateCharacterPosition(wstate.characterPositions[0], robot);
            olv->update(wstate);
        }
        ifs >> tm;
    }

    return 0;
}
