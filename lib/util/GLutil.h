#ifndef __GLUTIL_H__
#define __GLUTIL_H__

#include <vector>
#include <hrpCorba/ModelLoader.hh>
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

class GLshape;
class GLlink;
class GLbody;

void mulTrans(const double i_m1[16], const double i_m2[16], double o_m[16]);
void loadCube(GLshape *shape, double x, double y, double z);
void loadShapeFromBodyInfo(GLbody *body, OpenHRP::BodyInfo_var i_binfo,
                           GLshape *(*shapeFactory)()=NULL);
void loadShapeFromSceneInfo(GLlink *link, OpenHRP::SceneInfo_var i_sinfo,
                            GLshape *(*shapeFactory)()=NULL);
void loadShapeFromLinkInfo(GLlink *link, 
                           const OpenHRP::LinkInfo &i_li, 
                           OpenHRP::ShapeSetInfo_ptr i_ssinfo,
                           GLshape *(*shapeFactory)()=NULL);

#endif
