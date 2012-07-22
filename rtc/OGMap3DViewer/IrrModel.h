#ifndef __IRR_MODEL_H__
#define __IRR_MODEL_H__

#include <irrlicht/irrlicht.h>
#include <hrpModel/ModelLoaderUtil.h>

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
    virtual void render() {}
    virtual const irr::core::aabbox3d<irr::f32>& getBoundingBox() const {
        return m_box; 
    }
    void setPosture(double *i_angles, double *i_pos, double *i_rpy);
    GLcamera *findCamera(const char *i_name);
private:
    irr::core::aabbox3d<irr::f32> m_box;
    std::vector<GLlink *> m_links;
    GLlink *m_root;
};

#endif
