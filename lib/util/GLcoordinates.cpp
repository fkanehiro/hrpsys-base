#include <iostream>
#include "GLcoordinates.h"

GLcoordinates::GLcoordinates()
{
    for (int i=0; i<16; i++) m_trans[i] = 0.0;
    m_trans[0] = m_trans[5] = m_trans[10] = m_trans[15] = 1.0;
}

void GLcoordinates::setTransform(const double i_trans[12]){
    for (int i=0; i<3; i++){
        for (int j=0; j<4; j++){
            m_trans[j*4+i] = i_trans[i*4+j];// transposition for GL
        }
    }
}

hrp::Vector3 GLcoordinates::getPosition()
{
    return hrp::Vector3(m_trans[12], m_trans[13], m_trans[14]);
}

void GLcoordinates::setPosition(double x, double y, double z)
{
    m_trans[12] = x; m_trans[13] = y; m_trans[14] = z;
}

void GLcoordinates::getPosition(double& x, double& y, double& z)
{
    x = m_trans[12]; y = m_trans[13]; z = m_trans[14];
}

void GLcoordinates::setRotation(double r, double p, double y)
{
    hrp::Matrix33 R = hrp::rotFromRpy(r,p,y);
    setRotation(R);
}

void GLcoordinates::setRotation(double ax, double ay, double az, double th)
{
    hrp::Vector3 axis(ax, ay, az);
    hrp::Matrix33 R;
    hrp::calcRodrigues(R, axis, th);
    setRotation(R);
}

hrp::Matrix33 GLcoordinates::getRotation()
{
    hrp::Matrix33 R;
    for (int i=0; i<3; i++){
        for (int j=0; j<3; j++){
            R(j,i) = m_trans[i*4+j];
        }
    }
    return R;
}

void GLcoordinates::setRotation(const hrp::Matrix33 &R)
{
    for (int i=0; i<3; i++){
        for (int j=0; j<3; j++){
            m_trans[i*4+j] = R(j,i);
        }
    }
}

void GLcoordinates::setRotation(const double *R)
{
    for (int i=0; i<3; i++){
        for (int j=0; j<3; j++){
            m_trans[j*4+i] = R[i*3+j];
        }
    }
}

void GLcoordinates::getRotation(hrp::Matrix33 &R)
{
    for (int i=0; i<3; i++){
        for (int j=0; j<3; j++){
            R(j,i) = m_trans[i*4+j];
        }
    }
}

