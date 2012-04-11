#include <hrpModel/Link.h>
#include "BodyState.h"

using namespace hrp;

void BodyState::set(BodyPtr i_body)
{
    Link *root = i_body->rootLink();
    p = root->p;
    R = root->R;
    q.resize(i_body->numJoints());
    for (int i=0; i<i_body->numJoints(); i++){
        Link *joint =  i_body->joint(i);
        if (joint){
            q[i] = joint->q;
        }
    }
}
