#include <iostream>
#include <fstream>
#include <cstdio>
#include <GL/glew.h>
#ifdef __APPLE__
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#endif
#include <hrpUtil/Eigen3d.h>
#include <hrpModel/Sensor.h>
#include "GLutil.h"
#include "GLsceneBase.h"
#include "GLlink.h"
#include "GLcamera.h"
#include "GLshape.h"

using namespace OpenHRP;
using namespace hrp;

GLcamera::GLcamera(int i_width, int i_height, 
                   double i_near, double i_far, double i_fovy,
                   GLlink *i_link, int i_id) : 
    m_link(i_link),
    m_near(i_near), m_far(i_far), 
    m_fovy(i_fovy), m_width(i_width), m_height(i_height), 
    m_frameBuffer(0), m_renderBuffer(0), m_texture(0),
    m_sensor(NULL), m_colorBuffer(NULL)
{
    if (m_link) m_sensor = m_link->body->sensor<VisionSensor>(i_id);
}

GLcamera::~GLcamera()
{
    for (size_t i=0; i<m_shapes.size(); i++){
        delete m_shapes[i];
    }
    if (m_colorBuffer) delete [] m_colorBuffer;
}

size_t GLcamera::draw(int i_mode)
{
    size_t ntri = 0;
    glPushMatrix();
    glMultMatrixd(m_trans);
    for (size_t i=0; i<m_shapes.size(); i++){
        ntri += m_shapes[i]->draw(i_mode);
    }
    glPopMatrix();
    return ntri;
}    



const std::string& GLcamera::name() const {
    return m_name;
}

void GLcamera::name(const std::string &i_name){
    m_name = i_name;
}

void GLcamera::computeAbsTransform(double o_trans[16]){
    if (m_link){
        double trans[16];
        m_link->computeAbsTransform(trans);
        mulTrans(m_trans, trans, o_trans);
    }else{
        memcpy(o_trans, m_trans, sizeof(double)*16);
    }
}

void GLcamera::setView()
{
    setView(width(), height());
}
void GLcamera::setView(int w, int h)
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fovy()*180/M_PI, 
                   (double)w / (double)h, 
                   near(), far());
    if (m_link){
        computeAbsTransform(m_absTrans);
        gluLookAt(m_absTrans[12], m_absTrans[13], m_absTrans[14], 
                  m_absTrans[12]-m_absTrans[8], 
                  m_absTrans[13]-m_absTrans[9], 
                  m_absTrans[14]-m_absTrans[10],
                  m_absTrans[4], m_absTrans[5], m_absTrans[6]);
    }else{
        gluLookAt(m_viewPoint[0], m_viewPoint[1], m_viewPoint[2], 
                  m_viewTarget[0], m_viewTarget[1], m_viewTarget[2], 
                  0,0,1);
    }
}

void GLcamera::setViewPoint(double x, double y, double z)
{
    m_viewPoint[0] = x; m_viewPoint[1] = y; m_viewPoint[2] = z;
}

void GLcamera::setViewTarget(double x, double y, double z)
{
    m_viewTarget[0] = x; m_viewTarget[1] = y; m_viewTarget[2] = z;
}

double *GLcamera::getAbsTransform(){
    return m_absTrans;
}

GLlink *GLcamera::link()
{
    return m_link;
}

void GLcamera::highlight(bool flag)
{
    for (size_t i=0; i<m_shapes.size(); i++){
        m_shapes[i]->highlight(flag);
    }
}

void GLcamera::render(GLsceneBase *i_scene)
{
    if (!m_frameBuffer){
        initTexture();
        initRenderbuffer();
        initFramebuffer();
    }
    /* switch to framebuffer object */
    glBindFramebufferEXT( GL_FRAMEBUFFER_EXT, m_frameBuffer );

    glViewport( 0, 0, m_width, m_height );

    setView();

    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    int dm = GLlink::drawMode();
    GLlink::drawMode(GLlink::DM_SOLID);
    i_scene->drawObjects(false);
    GLlink::drawMode(dm);

    glFlush();

    glBindTexture( GL_TEXTURE_2D, m_texture );
    if (m_sensor->imageType != VisionSensor::NONE 
        && m_sensor->imageType != VisionSensor::DEPTH){
        if (!m_colorBuffer) {
            m_colorBuffer = new unsigned char[m_width*m_height*3];
        }
        glReadPixels(0,0, m_width, m_height, GL_RGB, GL_UNSIGNED_BYTE, m_colorBuffer);

        if (m_sensor->imageType == VisionSensor::COLOR
            || m_sensor->imageType == VisionSensor::COLOR_DEPTH){
            if (m_sensor->image.size() != m_width*m_height*3){
                std::cerr << "invalid image length" << std::endl;
            }else{
                unsigned char *src=m_colorBuffer;
                unsigned char *dst=&m_sensor->image[m_width*(m_height-1)*3];
                for (unsigned int i=0; i<m_height; i++){
                    memcpy(dst, src, m_width*3);
                    src += m_width*3;
                    dst -= m_width*3;
                }
                m_sensor->isUpdated = true;
            }
        }else if (m_sensor->imageType == VisionSensor::MONO
                  || m_sensor->imageType == VisionSensor::MONO_DEPTH){
            if (m_sensor->image.size() != m_width*m_height){
                std::cerr << "invalid image length" << std::endl;
            }else{
                unsigned char *src=m_colorBuffer;
                unsigned char *dst=&m_sensor->image[m_width*(m_height-1)];
                for (unsigned int i=0; i<m_height; i++){
                    for (unsigned int j=0; j<m_width; j++){
                        *dst = 0.299*src[0] + 0.587*src[1] + 0.114*src[2];
                        dst++;
                        src+=3;
                    }
                    dst -= m_width*2;
                }
                m_sensor->isUpdated = true;
            }
        }
    }
    if (m_sensor->imageType == VisionSensor::DEPTH
        || m_sensor->imageType == VisionSensor::COLOR_DEPTH
        || m_sensor->imageType == VisionSensor::MONO_DEPTH){
        float depth[m_width*m_height];
        glReadPixels(0,0,m_width, m_height, GL_DEPTH_COMPONENT, GL_FLOAT,
                     depth);
        // depth -> point cloud
        int w = m_sensor->width;
        int h = m_sensor->height;
        m_sensor->depth.resize(w*h*16);// will be shrinked later
        double far = m_sensor->far;
        double near = m_sensor->near;
        double fovx = 2*atan(w*tan(m_sensor->fovy/2)/h);
        double zs = w/(2*tan(fovx/2));
        unsigned int npoints=0;
        float *ptr = (float *)&m_sensor->depth[0];
        unsigned char *rgb = &m_sensor->image[0];
        bool colored = m_sensor->imageType == VisionSensor::COLOR_DEPTH;
        int step = 1;
        for (int i=0; i<h; i+=step){
            for (int j=0; j<w; j+=step){
                float d = depth[i*w+j];
                if (d == 1.0) {
                    continue;
                }
                ptr[2] = far*near/(d*(far-near)-far);
                ptr[0] = -(j-w/2)*ptr[2]/zs;
                ptr[1] = -(i-h/2)*ptr[2]/zs;
                if (colored){
                    unsigned char *c = (unsigned char *)(ptr + 3);
                    int offset = ((h-1-i)*w+j)*3;
                    c[0] = rgb[offset];
                    c[1] = rgb[offset+1];
                    c[2] = rgb[offset+2];
                }
                ptr += 4;
                npoints++;
            }
        }
        m_sensor->depth.resize(npoints*16);
        
        m_sensor->isUpdated = true;
    }
    /* switch to default buffer */
    glBindFramebufferEXT( GL_FRAMEBUFFER_EXT, 0 );
}

void GLcamera::initTexture( void )
{
    glPixelStorei( GL_UNPACK_ALIGNMENT, 1 );
    glGenTextures( 1, &m_texture );
    glBindTexture( GL_TEXTURE_2D, m_texture );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );
    glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, m_width, m_height,
                  0, GL_RGBA, GL_UNSIGNED_BYTE, 0 );
}

void GLcamera::initFramebuffer( void )
{
    glGenFramebuffersEXT( 1, &m_frameBuffer );
    glBindFramebufferEXT( GL_FRAMEBUFFER_EXT, m_frameBuffer );

    glFramebufferTexture2DEXT( GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT,
                               GL_TEXTURE_2D, m_texture, 0 );
    glFramebufferRenderbufferEXT( GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT,
                                  GL_RENDERBUFFER_EXT, m_renderBuffer );

    glBindFramebufferEXT( GL_FRAMEBUFFER_EXT, 0 );
}

void GLcamera::initRenderbuffer( void )
{
    glGenRenderbuffersEXT( 1, &m_renderBuffer );
    glBindRenderbufferEXT( GL_RENDERBUFFER_EXT, m_renderBuffer );
    glRenderbufferStorageEXT( GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT,
                              m_width, m_height );
}

VisionSensor *GLcamera::sensor()
{
    return m_sensor;
}

void GLcamera::addShape(GLshape *i_shape)
{
    m_shapes.push_back(i_shape);
}
