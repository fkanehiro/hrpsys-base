#ifndef __GLLINK_H__
#define __GLLINK_H__

#include <string>
#include <vector>
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#include <hrpModel/Link.h>
#include "GLcoordinates.h"

class GLcamera;
class GLshape;

class GLlink : public hrp::Link, public GLcoordinates
{
public:
    GLlink();
    ~GLlink();
    void draw();
    void setQ(double i_q);

    GLcamera *findCamera(const char *i_name);

    void computeAbsTransform();
    void computeAbsTransform(double o_trans[16]);
    void setAbsTransform(double o_trans[16]);
    void addShape(GLshape *shape);
    void addCamera(GLcamera *camera);
    void showAxes(bool flag);
    const std::vector<GLcamera *>& cameras();
    static void useAbsTransformToDraw();

protected:
    static bool m_useAbsTransformToDraw;
    std::vector<GLcamera *> m_cameras;
    double m_T_j[16], m_absTrans[16];
    std::vector<GLshape *> m_shapes;
    bool m_showAxes;
};

hrp::Link *GLlinkFactory();

#endif
