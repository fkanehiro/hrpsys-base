#ifndef __GL_SCENE_BASE_H__
#define __GL_SCENE_BASE_H__

#include <string>
#include <vector>
#include <map>
#include <sys/time.h>
//Open CV header
#include <opencv2/core/core_c.h>
#include <opencv2/highgui/highgui_c.h>
#ifndef CV_VERSION_EPOCH
  #if CV_VERSION_MAJOR > 3
    #include <opencv2/videoio/videoio_c.h>
  #endif
#endif
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
    void save(const char *i_fname);
    void capture(char *o_image);
    void init();
    void initLights();
    void defaultLights(bool flag);
    bool defaultLights();
    void clear();
    void requestClear();
    void requestCapture(const char *i_fname);
    void setCamera(GLcamera *i_camera);
    void nextCamera();
    void nextObject();
    GLcamera *getCamera();
    GLcamera *getDefaultCamera();
    void setMessages(const std::vector<std::string>& i_msgs) { m_msgs = i_msgs;}
    void showSlider(bool flag) { m_showSlider = flag; }
    void setScreenSize(int w, int h);
    void toggleRobotState() { m_showingStatus = !m_showingStatus; }
    void draw();
    size_t drawObjects(bool showSensors=true);
    void setView();
    virtual void showStatus() {}
    virtual void drawAdditionalLines() {}
    virtual void updateScene()=0;
    void showFloorGrid(bool flag);
    bool showFloorGrid();
    void showInfo(bool flag);
    void addBody(hrp::BodyPtr i_body);
    void maxEdgeLen(double i_len);
    hrp::BodyPtr targetObject();
    void setBackGroundColor(float rgb[3]);
    hrp::Vector3 center();
    void capture() { m_isCapturing = true; }
protected:
    enum {REQ_NONE, REQ_CLEAR, REQ_CAPTURE};

    void drawFloorGrid();
    void drawInfo(double fps, size_t ntri);

    std::vector<std::string> m_msgs; 
    bool m_showingStatus, m_showSlider;
    int m_width, m_height;
    GLcamera *m_camera, *m_default_camera;
    struct timeval m_lastDraw;
    CvVideoWriter *m_videoWriter;
    IplImage *m_cvImage;
    LogManagerBase *m_log;
    SDL_sem *m_sem;
    bool m_showFloorGrid, m_showInfo, m_defaultLights;
    int m_request;
    std::string m_fname;
    double m_maxEdgeLen;
    int m_targetObject;
    float m_bgColor[3];
    bool m_isCapturing;
};

#endif
