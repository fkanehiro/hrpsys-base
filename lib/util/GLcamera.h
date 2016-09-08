#ifndef __GLCAMERA_H__
#define __GLCAMERA_H__

#include <string>
#include <vector>
#include "GLcoordinates.h"

class GLsceneBase;
class GLlink;
class GLshape;
namespace hrp{
    class VisionSensor;
};

class GLcamera : public GLcoordinates
{
public:
    GLcamera(int i_width, int i_height, 
             double i_near, double i_far, double i_fovy, 
             GLlink *i_link=NULL, int i_id=-1);
    ~GLcamera();
    const std::string& name() const;
    void setView(int w, int h);
    void setView();
    void computeAbsTransform(double o_trans[16]);
    double *getAbsTransform();
    double near() { return m_near; }
    double far() { return m_far; }
    double fovy() { return m_fovy; }
    unsigned int width() { return m_width; }
    unsigned int height() { return m_height; }
    void setViewPoint(double x, double y, double z);
    void setViewTarget(double x, double y, double z);
    size_t draw(int i_mode);
    GLlink *link();
    void highlight(bool flag);
    void render(GLsceneBase *i_scene);
    hrp::VisionSensor *sensor();
    void addShape(GLshape *i_shape);
    void name(const std::string &i_name);
private:
    void initFramebuffer( void );
    void initRenderbuffer( void );
    void initTexture( void );
    std::string m_name;
    double m_absTrans[16];
    GLlink *m_link;
    double m_near, m_far, m_fovy;
    unsigned int m_width, m_height;
    double m_viewPoint[3], m_viewTarget[3];
    std::vector<GLshape *> m_shapes;
    GLuint m_frameBuffer, m_renderBuffer, m_texture;
    hrp::VisionSensor *m_sensor;
    unsigned char *m_colorBuffer;
};

#endif
