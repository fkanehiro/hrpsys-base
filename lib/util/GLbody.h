#ifndef __GLBODY_H__
#define __GLBODY_H__

#include <vector>
#include <hrpCorba/ModelLoader.hh>
#include <hrpUtil/Eigen3d.h>

class GLcamera;
class GLlink;

class GLbody
{
public:
    GLbody(OpenHRP::BodyInfo_var i_binfo);
    ~GLbody();
    void setPosture(const double *i_angles);
    void setPosition(double x, double y, double z);
    void setOrientation(double r, double p, double y);
    void setPosture(const double *i_angles, double *i_pos, double *i_rpy);
    void setPosture(const hrp::dvector& i_q, const hrp::Vector3& i_p,
                    const hrp::Matrix33& i_R);
    void draw();
    GLcamera *findCamera(const char *i_name);
    GLlink *link(unsigned int i);
    int numLinks() { return m_links.size(); }
    GLlink *rootLink() { return m_root; }
    GLlink *joint(unsigned int i) { return m_joints[i]; }
    int numJoints() { return m_joints.size(); }
    static void useAbsTransformToDraw();

private:
    static bool m_useAbsTransformToDraw;
    GLlink *m_root;
    std::vector<GLlink *> m_links;
    std::vector<GLlink *> m_joints;
};

#endif
