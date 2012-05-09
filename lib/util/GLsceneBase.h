#ifndef __GL_SCENE_BASE_H__
#define __GL_SCENE_BASE_H__

#include <string>
#include <vector>
#include <map>
#include <sys/time.h>
//Open CV header
#include <cv.h>
#include <highgui.h>
#include <SDL/SDL_thread.h>
#include <hrpCorba/ModelLoader.hh>
#include <hrpModel/ConstraintForceSolver.h>
#include <hrpModel/World.h>

class GLbody;
class GLcamera;
class LogManagerBase;

#define DEFAULT_W 640
#define DEFAULT_H 480
#define SLIDER_AREA_HEIGHT 30
#define SLIDER_SIDE_MARGIN 10

class GLsceneBase : virtual public hrp::World<hrp::ConstraintForceSolver>
{
public:
    GLsceneBase(LogManagerBase *i_log);
    virtual ~GLsceneBase();
    void addBody(const std::string &i_name, OpenHRP::BodyInfo_var i_binfo);
    void save(const char *i_fname);
    void capture(char *o_image);
    void init();
    void clear();
    void setCamera(GLcamera *i_camera);
    GLcamera *getCamera();
    GLcamera *getDefaultCamera();
    void setMessages(const std::vector<std::string>& i_msgs) { m_msgs = i_msgs;}
    void showSlider(bool flag) { m_showSlider = flag; }
    void setScreenSize(int w, int h);
    void toggleRobotState() { m_showingStatus = !m_showingStatus; }
    void draw();
    virtual void showStatus() {}
    virtual void drawAdditionalLines() {}
    virtual void updateScene()=0;
    void showFloorGrid(bool flag);
    void showInfo(bool flag);
protected:
    void drawFloorGrid();
    void drawInfo(double fps);

    std::vector<std::string> m_msgs; 
    bool m_showingStatus, m_showSlider;
    int m_width, m_height;
    GLcamera *m_camera, *m_default_camera;
    struct timeval m_lastDraw;
    CvVideoWriter *m_videoWriter;
    IplImage *m_cvImage;
    LogManagerBase *m_log;
    SDL_sem *m_sem;
    std::string m_newBodyName;
    OpenHRP::BodyInfo_var m_newBodyInfo;
    bool m_isNewBody;
    bool m_showFloorGrid, m_showInfo;
    bool m_clearRequested;
};

#endif
