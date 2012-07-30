#ifndef __GLCOORDINATES_H__
#define __GLCOORDINATES_H__

#include <hrpUtil/Eigen3d.h>

class GLcoordinates
{
public:
    GLcoordinates();
    void setTransform(const double i_trans[12]);
    double *getTransform() { return m_trans; } 
    hrp::Vector3 getPosition();
    void setPosition(double x, double y, double z);
    void getPosition(double& x, double& y, double& z);
    template<class T>
    void setPosition(const T &p){
        m_trans[12] = p[0]; m_trans[13] = p[1]; m_trans[14] = p[2];
    }
    void setRotation(double r, double p, double y);
    void setRotation(double ax, double ay, double az, double th);
    hrp::Matrix33 getRotation();
    void setRotation(const hrp::Matrix33 &R);
    void setRotation(const double *R);
    void getRotation(hrp::Matrix33 &R);
protected:
    double m_trans[16];
};
#endif
