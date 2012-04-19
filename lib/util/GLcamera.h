#ifndef __GLCAMERA_H__
#define __GLCAMERA_H__

#include <string>
#include <hrpCorba/ModelLoader.hh>

class GLlink;

class GLcamera
{
public:
    GLcamera(const OpenHRP::SensorInfo &i_si, OpenHRP::BodyInfo_var i_binfo,
             GLlink *i_link);
    GLcamera(int i_width, int i_height, double i_near, double i_far, double i_fovy);
    const std::string& name() const;
    void setView();
    void computeAbsTransform(double o_trans[16]);
    void setTransform(double i_trans[16]);
    void getAbsTransform(double o_trans[16]);
    double near() { return m_near; }
    double far() { return m_far; }
    double fovy() { return m_fovy; }
    int width() { return m_width; }
    int height() { return m_height; }
    void getDepthOfLine(int i_row, float *o_depth);
private:
    std::string m_name;
    double m_trans[16], m_absTrans[16];
    GLlink *m_link;
    double m_near, m_far, m_fovy;
    int m_width, m_height;
};

#endif
