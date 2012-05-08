#ifndef __GLLINK_H__
#define __GLLINK_H__

#include <string>
#include <vector>
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#include <hrpCorba/ModelLoader.hh>
#include <hrpModel/Link.h>

class GLcamera;

class GLlink : public hrp::Link
{
public:
    typedef enum {FREE_JOINT, 
                  FIXED_JOINT, 
                  ROTATIONAL_JOINT, 
                  SLIDE_JOINT} JointType;

    GLlink();
    ~GLlink();
    void setDrawInfo(const OpenHRP::LinkInfo &i_li, OpenHRP::BodyInfo_var i_binfo);
    void draw();
    void setQ(double i_q);
    void setTransform(double i_trans[16]);
    double *getTransform() { return m_trans; } 

    GLcamera *findCamera(const char *i_name);

    void computeAbsTransform(double o_trans[16]);
    void setAbsTransform(double o_trans[16]);
    static void useAbsTransformToDraw();

private:
    static bool m_useAbsTransformToDraw;
    std::vector<GLcamera *> m_cameras;
    double m_trans[16], m_T_j[16], m_absTrans[16];
    int m_list;
    std::vector<GLuint> m_textures;
};

hrp::Link *GLlinkFactory();

#endif
