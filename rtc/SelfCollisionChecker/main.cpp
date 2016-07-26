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

    hrp::BodyPtr robot = hrp::BodyPtr(new hrp::Body());
    if (!loadBodyFromModelLoader(robot, argv[1], 
                                 argc, argv, true)){
        std::cerr << "Error: failed to load model[" << argv[1] << "]" 
                  << std::endl;
        return 2;
    }

    hrp::SelfCollisionChecker scc(robot);

    std::cerr << scc.numOfCheckPairs() << " pairs are defined" << std::endl;
    
    std::ifstream ifs(argv[2]);
    double tm, q[robot->numJoints()];
    std::vector<std::pair<std::string, std::string> > pairs;

    ifs >> tm;
    while (!ifs.eof()){
        for (int i=0; i<robot->numJoints(); i++){
            ifs >> q[i];
        }
        pairs = scc.check(q);
        for (unsigned int i=0; i<pairs.size(); i++){
            std::cout << tm << " " << pairs[i].first << ":" << pairs[2].second
                      << std::endl;
        }
        ifs >> tm;
    }

    return 0;
}
