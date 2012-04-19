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
#include <SDL/SDL_thread.h>

#define SLIDER_AREA_HEIGHT 30
#define SLIDER_SIDE_MARGIN 10

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

class GLlink
{
public:
    GLlink(const OpenHRP::LinkInfo &i_li, OpenHRP::BodyInfo_var i_binfo);

    void draw();
    void setParent(GLlink *i_parent);
    void addChild(GLlink *i_child);
    void setQ(double i_q);
    void setTransform(double i_trans[16]);
    int jointId();
    const std::string& name() { return m_name; }

    GLcamera *findCamera(const char *i_name);

    void computeAbsTransform(double o_trans[16]);
    void setAbsTransform(double o_trans[16]);

private:
    GLlink *m_parent;
    std::string m_name;
    std::vector<GLlink *> m_children;
    std::vector<GLcamera *> m_cameras;
    hrp::Vector3 m_axis;
    double m_trans[16], m_T_j[16], m_absTrans[16];
    int m_list, m_jointId;
};

class GLbody
{
public:
    GLbody(OpenHRP::BodyInfo_var i_binfo);
    ~GLbody();
    void setPosture(double *i_angles, double *i_pos, double *i_rpy);
    void draw();
    GLcamera *findCamera(const char *i_name);
    GLlink *link(unsigned int i);
    int numLinks() { return m_links.size(); }
    GLlink *rootLink() { return m_root; }
    GLlink *joint(unsigned int i) { return m_joints[i]; }
    int numJoints() { return m_joints.size(); }
private:
    GLlink *m_root;
    std::vector<GLlink *> m_links;
    std::vector<GLlink *> m_joints;
};

class GLscene
{
public:
    void addBody(const std::string& i_name, GLbody *i_body);
    void addBody(const std::string& i_name, OpenHRP::BodyInfo_var i_binfo);
    GLbody *findBody(const std::string& i_name);
    void draw();
    void save(const char *i_fname);
    void capture(char *o_image);
    void init();
    void setCamera(GLcamera *i_camera);
    GLcamera *getCamera();
    void addState(const OpenHRP::WorldState &state);
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
    std::map<std::string, GLbody *> m_bodies; 
    GLcamera *m_camera, *m_default_camera;
    std::deque<OpenHRP::WorldState> m_log;
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
    SDL_sem *m_sem;
    std::string m_newBodyName;
    OpenHRP::BodyInfo_var m_newBodyInfo;
    bool m_isNewBody;
};

void mulTrans(const double i_m1[16], const double i_m2[16], double o_m[16]);
void printMatrix(double mat[16]);

#endif
