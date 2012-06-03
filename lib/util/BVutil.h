#ifndef __BVUTIL_H__
#define __BVUTIL_H__

#include <hrpModel/Body.h>

void convertToAABB(hrp::BodyPtr i_body);
void convertToAABB(hrp::Link *i_link);
void convertToConvexHull(hrp::BodyPtr i_body);
void convertToConvexHull(hrp::Link *i_link);

#endif
