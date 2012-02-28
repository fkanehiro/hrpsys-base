#ifndef __IRR_MODEL_H__
#define __IRR_MODEL_H__

#include <irrlicht/irrlicht.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpModel/Link.h>

class GLbody;
class GLcamera;
class GLlink;

class GLscene
{
public:
    GLscene();
    ~GLscene();
    static GLscene *getInstance();
    bool init(int w=640, int h=480);
    void draw();
    GLbody *addBody(OpenHRP::BodyInfo_var i_binfo);
    GLbody *addBody(hrp::BodyPtr i_body);
    void setCamera(GLcamera *i_camera);
    GLcamera *getCamera();
    irr::scene::ISceneManager *getSceneManager();
    irr::video::IVideoDriver *getVideoDriver();
private:
    static GLscene *m_scene;
    irr::IrrlichtDevice *m_device;
    GLcamera *m_camera, *m_defaultCamera;
    irr::scene::ICameraSceneNode *m_cnode;
    irr::IEventReceiver *m_receiver;
};

class GLcamera
{
public:
    GLcamera(const OpenHRP::SensorInfo& i_info,
             irr::scene::ISceneNode *i_node);
    GLcamera(irr::scene::ISceneNode *i_node);
    void setCameraParameters(irr::scene::ICameraSceneNode *i_camera);
    void updateCameraTransform(irr::scene::ICameraSceneNode *i_camera);
    const char *name();
    int width();
    int height();
    float near();
    float far();
    float fovy();
    void getAbsTransform(double *o_T);
private:
    irr::scene::ISceneNode *m_node;
    float m_near, m_far, m_fovy;
    int m_width, m_height;
};

class GLbody : public irr::scene::ISceneNode
{
public:
    GLbody(irr::scene::ISceneNode *i_parent, 
           irr::scene::ISceneManager *i_mgr, 
           irr::s32 i_id,
           OpenHRP::BodyInfo_var i_binfo);
    GLbody(irr::scene::ISceneNode *i_parent, 
           irr::scene::ISceneManager *i_mgr, 
           irr::s32 i_id,
           hrp::BodyPtr i_body);
    virtual void render() {}
    virtual const irr::core::aabbox3d<irr::f32>& getBoundingBox() const {
        return m_box; 
    }
    void setPosture(const double *i_angles);
    void setPosition(double x, double y, double z);
    void setOrientation(double r, double p, double y);
    void setPosture(double *i_angles, double *i_pos, double *i_rpy);
    GLcamera *findCamera(const char *i_name);
    GLlink *rootLink() { return m_root; }
    unsigned int numJoints() { return m_joints.size(); }
private:
    irr::core::aabbox3d<irr::f32> m_box;
    std::vector<GLlink *> m_links;
    std::vector<GLlink *> m_joints;
    GLlink *m_root;
};

class GLlink : public irr::scene::ISceneNode
{
public:
    typedef enum {FREE_JOINT, 
                  FIXED_JOINT, 
                  ROTATIONAL_JOINT, 
                  SLIDE_JOINT} JointType;

    GLlink(irr::scene::ISceneNode *i_parent, irr::scene::ISceneManager *i_mgr, irr::s32 i_id,
           const OpenHRP::LinkInfo &i_li, OpenHRP::BodyInfo_var i_binfo);
    GLlink(irr::scene::ISceneNode *i_parent, irr::scene::ISceneManager *i_mgr, irr::s32 i_id,
	   const hrp::Link *i_link, hrp::BodyPtr i_body) ;
    virtual void render() {}
    virtual const irr::core::aabbox3d<irr::f32>& getBoundingBox() const { return m_box; }
    void setQ(double i_q);
    int jointId() const { return m_jointId; }
    GLcamera *findCamera(const char *i_name);
    JointType jointType() { return m_jointType; }
private:
    irr::core::aabbox3d<irr::f32> m_box;
    hrp::Vector3 m_axis;
    int m_jointId;
    JointType m_jointType;
    std::vector<GLcamera *> m_cameraInfos;
};

#endif
