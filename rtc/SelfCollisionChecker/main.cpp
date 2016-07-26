#include <hrpModel/ModelLoaderUtil.h>
#include <iostream>
#include <fstream>
#include "scc.h"

int main(int argc, char *argv[])
{
    if (argc < 3){
        std::cerr << "Usage: " << argv[0] << "[VRML model] [log file]" 
                  << std::endl;
        return 1;
    }

    hrp::LinkNamePairList pairs;
    for (int i=3; i<argc; i++){
        std::string str = argv[i];
        int pos;
        if ((pos = str.find(":")) != std::string::npos){
            std::string link1 = str.substr(0, pos);
            std::string link2 = str.substr(pos+1);
            std::cerr << "[" << link1 << "],[" << link2 << "]" << std::endl;
            pairs.push_back(std::make_pair(link1, link2));
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
    
    std::ifstream ifs(argv[2]);
    double tm, q[robot->numJoints()];

    ifs >> tm;
    while (!ifs.eof()){
        for (int i=0; i<robot->numJoints(); i++){
            ifs >> q[i];
        }
        pairs = scc.check(q);
        for (unsigned int i=0; i<pairs.size(); i++){
            std::cout << tm << " " << pairs[i].first << ":" << pairs[i].second
                      << std::endl;
        }
        ifs >> tm;
    }

    return 0;
}
