#ifndef __GLBODY_H__
#define __GLBODY_H__

#include <vector>
#include <hrpModel/Body.h>

class GLcamera;
class GLlink;

class GLbody : virtual public hrp::Body
{
public:
    GLbody();
    ~GLbody();
    void setPosture(const double *i_angles);
    void setPosition(double x, double y, double z);
    void setOrientation(double r, double p, double y);
    void setPosture(const double *i_angles, double *i_pos, double *i_rpy);
    void setPosture(const hrp::dvector& i_q, const hrp::Vector3& i_p,
                    const hrp::Matrix33& i_R);
    void draw();
    GLcamera *findCamera(const char *i_name);
    static void useAbsTransformToDraw();

private:
    static bool m_useAbsTransformToDraw;
};

#endif
