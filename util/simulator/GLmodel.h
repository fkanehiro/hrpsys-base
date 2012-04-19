#ifndef __GLMODEL_H__
#define __GLMODEL_H__

#include <hrpCorba/ModelLoader.hh>
#include <hrpUtil/Eigen3d.h>
#include <vector>
#include <deque>
#include <map>
//Open CV header
#include <cv.h>
#include <highgui.h>
#include "SceneState.h"

#define SLIDER_AREA_HEIGHT 30
#define SLIDER_SIDE_MARGIN 10

class GLcamera;
class GLbody;

class GLscene
{
public:
    void addBody(const std::string& i_name, GLbody *i_body);
    GLbody *findBody(const std::string& i_name);
    void draw();
    void save(const char *i_fname);
    void capture(char *o_image);
    void init();
    void setCamera(GLcamera *i_camera);
    GLcamera *getCamera();
    void addState(const SceneState &state);
    void clearLog();
    void play();
    void record();
    void prev(int delta=1);
    void next(int delta=1);
    void head();
    void tail();
    void move(double ratio);
    bool isNewStateAdded();
    bool isPlaying();
    bool isRecording();
    void faster();
    void slower();
    void setScreenSize(int w, int h);
    void setMessages(const std::vector<std::string>& i_msgs) { m_msgs = i_msgs;}
    void toggleRobotState() { m_showingRobotState = !m_showingRobotState; }
    void showSlider(bool flag) { m_showSlider = flag; }

    static GLscene *getInstance();
private:
    GLscene();
    ~GLscene();
    void showRobotState();
    void setIndex(int i);

    static GLscene *m_scene;
    std::map<std::string, GLbody *> m_nameBodyMap; 
    std::vector<GLbody *> m_bodies; 
    GLcamera *m_camera, *m_default_camera;
    std::deque<SceneState> m_log;
    bool m_isPlaying, m_isNewStateAdded, m_isRecording;
    int m_index;
    double m_initT;
    struct timeval m_startT, m_lastDraw;
    double m_playRatio;
    int m_width, m_height;
    CvVideoWriter *m_videoWriter;
    IplImage *m_cvImage;
    std::vector<std::string> m_msgs; 
    bool m_showingRobotState, m_showSlider, m_atLast;
};

void printMatrix(double mat[16]);

#endif
