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
    m_trans[3] = m_trans[7] = m_trans[11] = 0.0; m_trans[15] = 1.0;
}

void GLcoordinates::setPosition(const hrp::Vector3 &p)
{
    m_trans[12] = p[0]; m_trans[13] = p[1]; m_trans[14] = p[2];
}

void GLcoordinates::setRotation(const hrp::Matrix33 &R)
{
    for (int i=0; i<3; i++){
        for (int j=0; j<3; j++){
            m_trans[i*4+j] = R(j,i);
        }
    }
}

