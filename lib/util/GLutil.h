#ifndef __GLUTIL_H__
#define __GLUTIL_H__

#include <vector>
#include <hrpCorba/ModelLoader.hh>
#include <GL/gl.h>

std::vector<GLuint> compileShape(OpenHRP::ShapeSetInfo_ptr i_binfo,
                                 const OpenHRP::TransformedShapeIndexSequence& tsis);
void mulTrans(const double i_m1[16], const double i_m2[16], double o_m[16]);

#endif
