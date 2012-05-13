#ifndef __GLCOORDINATES_H__
#define __GLCOORDINATES_H__

#include <hrpUtil/Eigen3d.h>

class GLcoordinates
{
public:
    GLcoordinates();
    void setTransform(const double i_trans[12]);
    double *getTransform() { return m_trans; } 
    void setPosition(const hrp::Vector3 &p);
    void setRotation(const hrp::Matrix33 &R);
protected:
    double m_trans[16];
};
#endif
