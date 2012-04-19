#include <cstdio>
#include <iostream>
#include <fstream>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include <sys/time.h>
#include "util/GLcamera.h"
#include "util/GLlink.h"
#include "util/GLbody.h"
#include "GLmodel.h"

using namespace OpenHRP;
using namespace hrp;

#define DEFAULT_W 640
#define DEFAULT_H 480
#define DEFAULT_FPS 10
#define MAX_LOG_LENGTH 5000

void setRobotState(GLbody * body,
                   const OpenHRP::RobotHardwareService::RobotState& state)
{ 
    if (state.angle.length()) body->setPosture(state.angle.get_buffer());
}

void GLscene::addBody(const std::string &i_name, GLbody *i_body){
    //std::cout <<"addBody(" << i_name << "," << i_body << ")" << std::endl;
    m_nameBodyMap[i_name] = i_body;
    m_bodies.push_back(i_body);
}

GLbody *GLscene::findBody(const std::string &i_name)
{
    return m_nameBodyMap[i_name];
}

void drawString(const char *str)
{
    for (unsigned int i=0; i<strlen(str); i++){
        glutBitmapCharacter(GLUT_BITMAP_9_BY_15, str[i]);
    }
}
void drawString2(const char *str)
{
    for (unsigned int i=0; i<strlen(str); i++){
        glutBitmapCharacter(GLUT_BITMAP_8_BY_13, str[i]);
    }
}

void white(){
    glColor3d(1.0,1.0,1.0);
}

void red(){
    glColor3d(1.0,0.0,0.0);
}

void yellow(){
    glColor3d(1.0,1.0,0.0);
}

void green(){
    glColor3d(0.0,1.0,0.0);
}

void blue(){
    glColor3d(0.0,0.0,1.0);
}

void black(){
    glColor3d(0.0,0.0,0.0);
}

bool isCalibrated(int s) { 
    return s & OpenHRP::RobotHardwareService::CALIB_STATE_MASK; 
}
bool isPowerOn(int s) { 
    return s & OpenHRP::RobotHardwareService::POWER_STATE_MASK; 
}
bool isServoOn(int s) { 
    return s & OpenHRP::RobotHardwareService::SERVO_STATE_MASK; 
}
int servoAlarm(int s){
    return (s & OpenHRP::RobotHardwareService::SERVO_ALARM_MASK)>>OpenHRP::RobotHardwareService::SERVO_ALARM_SHIFT; 
}
int temperature(int s){
    return (s & OpenHRP::RobotHardwareService::DRIVER_TEMP_MASK)>>OpenHRP::RobotHardwareService::DRIVER_TEMP_SHIFT; 
}

void GLscene::draw(){
    struct timeval tv;
    gettimeofday(&tv, NULL);
    double fps = 1.0/((tv.tv_sec - m_lastDraw.tv_sec)+(tv.tv_usec - m_lastDraw.tv_usec)/1e6);
    m_lastDraw = tv;

    if (m_isPlaying){
        // compute time to draw
        struct timeval tv;
        gettimeofday(&tv, NULL);
        double drawT = m_initT + ((tv.tv_sec - m_startT.tv_sec) + (tv.tv_usec - m_startT.tv_usec)*1e-6)*m_playRatio;
        //
        while(drawT > m_log[m_index].time){
            setIndex(m_index+1);
            if (m_atLast) {
                m_isPlaying = false;
                break;
            }
        }
    } else if (m_isNewStateAdded && m_atLast){
        // draw newest state
        setIndex(m_log.size() - 1);
        m_isNewStateAdded = false;
    }
    
    if (m_index >= 0) setRobotState(m_bodies[0], m_log[m_index].state);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // robots
    std::vector<GLbody *>::iterator it;
    for (it=m_bodies.begin(); it!=m_bodies.end(); it++){
        (*it)->draw();
    }

    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);

    // floor grids
    glColor3f(1,1,1);
    double s[3], e[3];
    s[2] = e[2] = 0;
    s[0] = 10; e[0] = -10;
    for (int i=-10;i<=10; i++){
        s[1] = e[1] = i;
        glVertex3dv(s);
        glVertex3dv(e);
    }
    s[1] = 10; e[1] = -10;
    for (int i=-10;i<=10; i++){
        s[0] = e[0] = i;
        glVertex3dv(s);
        glVertex3dv(e);
    }
    glEnd();

    // draw texts
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, m_width, 0, m_height);
    glPushMatrix();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glColor3d(1.0,1.0,1.0);
    glRasterPos2f(10, m_height-15);
    char buf[256];
    if (m_index >= 0){
        double dt = m_log[m_index].time - m_log[0].time;
        sprintf(buf, "Time:%6.3f[s]" , dt);
    }else{
        sprintf(buf, "Time:------[s]");
    }
    drawString(buf);
    glRasterPos2f(10, m_height-30);
    sprintf(buf, "Playback x%6.3f", m_playRatio);
    drawString(buf);
    glRasterPos2f(10, m_height-45);
    sprintf(buf, "FPS %2.0f", fps);
    drawString(buf);
    for (unsigned int i=0; i<m_msgs.size(); i++){
        glRasterPos2f(10, (m_msgs.size()-i)*15);
        drawString(m_msgs[i].c_str());
    }
    showRobotState();
    if (m_showSlider){
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_BLEND);
        glColor4f(0.0,0.0,0.0, 0.5);
        glRectf(SLIDER_SIDE_MARGIN,10,m_width-SLIDER_SIDE_MARGIN,20);
        if (m_log.size()>1){
            int x = ((double)m_index)/(m_log.size()-1)*(m_width-20)+10;
            glRectf(x-5,5,x+5,25);
        }
        glDisable(GL_BLEND);
    }
    glPopMatrix();
    glEnable(GL_LIGHTING);

    if(m_isRecording){
        while(m_initT > m_log[m_index].time){
            setIndex(m_index+1);
            if (m_atLast) {
                m_isRecording = false;
                break;
            }
        } 
        char *dst = m_cvImage->imageData;
        capture(dst);
        cvWriteFrame(m_videoWriter, m_cvImage);
        if (!m_isRecording){
            cvReleaseVideoWriter(&m_videoWriter);
            cvReleaseImage(&m_cvImage);
        }
        //printf("t:%6.3f, %4d\n", m_initT, m_index);
        m_initT += 1.0/DEFAULT_FPS*m_playRatio;
    }
}

void GLscene::showRobotState()
{
    if (m_index < 0) return;

    OpenHRP::RobotHardwareService::RobotState &rstate = m_log[m_index].state;
    if (m_showingRobotState){
        GLbody *body = m_bodies[0];
#define HEIGHT_STEP 12
        int width = m_width - 410;
        int height = m_height-HEIGHT_STEP;
        char buf[256];
        for (int i=0; i<body->numJoints(); i++){
            GLlink *l = body->joint(i);
            if (l){
                int ss = rstate.servoState[i];
                int x = width;
                // joint ID
                sprintf(buf, "%2d",i);
                if (!isCalibrated(ss)){
                    yellow();
                }else if(isServoOn(ss)){
                    red();
                }
                glRasterPos2f(x, height);
                drawString2(buf);
                white();
                x += 8*3;
                // power status
                if (isPowerOn(ss)) blue();
                glRasterPos2f(x, height);
                drawString2("o");
                if (isPowerOn(ss)) white();
                x += 8*2;
                // joint name, current angle, command angle and torque
                sprintf(buf, "%13s %8.3f %8.3f %6.1f", 
                        l->name().c_str(), 
                        rstate.angle[i]*180/M_PI,
                        rstate.command[i]*180/M_PI,
                        rstate.torque[i]*180/M_PI);
                glRasterPos2f(x, height);
                drawString2(buf);
                x += 8*(14+9+9+7);
                // servo alarms
                sprintf(buf, "%03x", servoAlarm(ss));
                glRasterPos2f(x, height);
                drawString2(buf);
                x += 8*4;
                // driver temperature
                int temp = temperature(ss);
                if (!temp){
                    sprintf(buf, "--", temp);
                }else{
                    sprintf(buf, "%2d", temp);
                }
                if (temp >= 60) red();
                glRasterPos2f(x, height);
                drawString2(buf);
                if (temp >= 60) white();
                x += 8*3;
                
                height -= HEIGHT_STEP;
            }
        }
        if (rstate.accel.length()){
            glRasterPos2f(width, height);
            height -= HEIGHT_STEP;
            drawString2("acc:");
            for (unsigned int i=0; i<rstate.accel.length(); i++){
                sprintf(buf, "  %8.4f %8.4f %8.4f",
                        rstate.accel[i][0], rstate.accel[i][1], rstate.accel[i][2]);
                glRasterPos2f(width, height);
                height -= HEIGHT_STEP;
                drawString2(buf);
            }
        }
        if (rstate.rateGyro.length()){
            glRasterPos2f(width, height);
            height -= HEIGHT_STEP;
            drawString2("rate:");
            for (unsigned int i=0; i<rstate.rateGyro.length(); i++){
                sprintf(buf, "  %8.4f %8.4f %8.4f",
                        rstate.rateGyro[i][0], rstate.rateGyro[i][1], rstate.rateGyro[i][2]);
                glRasterPos2f(width, height);
                height -= HEIGHT_STEP;
                drawString2(buf);
            }
        }
        if (rstate.force.length()){
            glRasterPos2f(width, height);
            height -= HEIGHT_STEP;
            drawString2("force/torque:");
            for (unsigned int i=0; i<rstate.force.length(); i++){
                sprintf(buf, "  %6.1f %6.1f %6.1f %6.2f %6.2f %6.2f",
                        rstate.force[i][0], 
                        rstate.force[i][1], 
                        rstate.force[i][2],
                        rstate.force[i][3], 
                        rstate.force[i][4], 
                        rstate.force[i][5]);
                glRasterPos2f(width, height);
                height -= HEIGHT_STEP;
                drawString2(buf);
            }
        }
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_BLEND);
        glColor4f(0.0,0.0,0.0, 0.5);
        if (m_showSlider){
            glRectf(width,SLIDER_AREA_HEIGHT,m_width,m_height);
        }else{
            glRectf(width,0,m_width,m_height);
        }
        glDisable(GL_BLEND);
    }else{
        // !m_showingRobotState
        bool servo=false, power=false;
        for (unsigned int i=0; i<rstate.servoState.length(); i++){
            if (isServoOn(rstate.servoState[i])) servo = true;
            if (isPowerOn(rstate.servoState[i])) power = true;
        }
        struct timeval tv;
        gettimeofday(&tv, NULL);
        double dt = tv.tv_sec + tv.tv_usec/1e6 - m_log[m_log.size()-1].time;
        if (dt < 1.0) green(); else black();
        glRectf(m_width-115,m_height-45,m_width-85,m_height-15);
        if (power) blue(); else black();
        glRectf(m_width- 80,m_height-45,m_width-50,m_height-15);
        if (servo) red(); else black();
        glRectf(m_width- 45,m_height-45,m_width-15,m_height-15);
    }
}

GLscene *GLscene::getInstance()
{
    return m_scene;
}

void printMatrix(double mat[16])
{
    for (int i=0; i<4; i++){
        for (int j=0; j<4; j++){
            printf("%6.3f ", mat[j*4+i]);
        }
        printf("\n");
    }
}

GLscene::GLscene() 
    : m_camera(m_default_camera), 
      m_isPlaying(false), 
      m_isNewStateAdded(false), 
      m_isRecording(false), 
      m_index(-1), m_playRatio(1.0), m_width(DEFAULT_W), m_height(DEFAULT_H),
      m_showingRobotState(false), m_showSlider(false), m_atLast(true)
{
    m_default_camera = new GLcamera(DEFAULT_W, DEFAULT_H, 1.0, 100.0, 40*M_PI/180);
    double T[] = {0,1,0,0,
                  0,0,1,0,
                  1,0,0,0,
                  4,0,0.8,1};
    m_default_camera->setTransform(T);
}

GLscene::~GLscene()
{
    delete m_default_camera;
}

void GLscene::init()
{
    setCamera(m_default_camera);

    GLfloat light0pos[] = { 0.0, 4.0, 6.0, 1.0 };
    GLfloat light1pos[] = { 6.0, 4.0, 0.0, 1.0 };
    GLfloat white[] = { 0.6, 0.6, 0.6, 1.0 };

    //glClearColor(0.4, 0.4, 0.55, 1.0);
    glClearColor(0.2, 0.2, 0.5, 1.0);
    glEnable(GL_DEPTH_TEST);

    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, white);
    glLightfv(GL_LIGHT0, GL_SPECULAR, white);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, white);
    glLightfv(GL_LIGHT1, GL_SPECULAR, white);
    glLightfv(GL_LIGHT0, GL_POSITION, light0pos);
    glLightfv(GL_LIGHT1, GL_POSITION, light1pos);
}

void GLscene::save(const char *i_fname)
{
#if 0
    int w, h;
    glfwGetWindowSize(&w,&h);
    unsigned char *buffer = new unsigned char[w*h*3];

    capture(buffer);
    std::ofstream ofs("test.ppm", std::ios::out | std::ios::trunc | std::ios::binary );
    char buf[10];
    sprintf(buf, "%d %d", w, h);
    ofs << "P6" << std::endl << buf << std::endl << "255" << std::endl;
    for (int i=h-1; i>=0; i--){
        ofs.write((char *)(buffer+i*w*3), w*3);
    }
    delete [] buffer;
#endif
}

void GLscene::capture(char *o_buffer)
{
    glReadBuffer(GL_BACK);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    char buf[m_width*m_height*3];
    glReadPixels(0,0, m_width,m_height,GL_BGR,GL_UNSIGNED_BYTE, buf);
    char *dst = o_buffer, *src;
    for (int i=0; i<m_height; i++){
        src = buf + (m_height -1 - i)*m_width*3;
        memcpy(dst, src, m_width*3);
        dst += m_width*3;
    }
}

void GLscene::setCamera(GLcamera *i_camera)
{
    if (!i_camera) return;

    m_camera = i_camera;
    //glfwSetWindowSize(m_camera->width(), m_camera->height());
    glViewport(0, 0, m_camera->width(), m_camera->height());
}

GLcamera *GLscene::getCamera()
{
    return m_camera;
}

void GLscene::addState(const TimedRobotState& state)
{
    m_log.push_back(state);
    if (m_log.size() > MAX_LOG_LENGTH) m_log.pop_front();
    m_isNewStateAdded = true;
}

void GLscene::clearLog()
{
    m_log.clear();
    m_index = -1;
}

void GLscene::play()
{
    if (m_log.size() == 0) return;

    if (!m_isPlaying){
        m_isPlaying = true;
        if (m_atLast) setIndex(0);
        m_initT = m_log[m_index].time;
        gettimeofday(&m_startT, NULL);
    }else{
        m_isPlaying = false;
    }
}

void GLscene::record()
{
    if (m_log.size() == 0) return;

    setIndex(0);
    m_videoWriter = cvCreateVideoWriter(
        "olv.avi",
        CV_FOURCC('D','I','V','X'),
        DEFAULT_FPS,
        cvSize(m_width, m_height));
    m_cvImage = cvCreateImage(
        cvSize(m_width, m_height),
        IPL_DEPTH_8U, 3);
    m_initT = m_log[0].time;
    m_isRecording = true;
}

void GLscene::prev(int delta)
{
    setIndex(m_index - delta);
}

void GLscene::next(int delta)
{
    setIndex(m_index+delta);
}

void GLscene::head()
{
    setIndex(0);
}

void GLscene::tail()
{
    if (!m_log.empty()) setIndex(m_log.size()-1);
}

void GLscene::move(double ratio)
{
    if (m_log.size()){ 
        setIndex(ratio*(m_log.size()-1));
    }
}

void GLscene::setIndex(int i)
{
    if (m_log.empty()) return;

    m_index = i;
    if (m_index < 0) m_index = 0;
    if (m_index >= m_log.size()) m_index = m_log.size()-1;
    m_atLast = m_index == m_log.size()-1; 
}

bool GLscene::isPlaying()
{
    return m_isPlaying;
}

bool GLscene::isRecording()
{
    return m_isRecording;
}

bool GLscene::isNewStateAdded()
{
    return m_isNewStateAdded;
}

void GLscene::faster()
{
    m_playRatio *= 2;
    if (m_isPlaying){
        m_initT = m_log[m_index].time;
        gettimeofday(&m_startT, NULL);
    }
}

void GLscene::slower()
{
    m_playRatio /= 2;
    if (m_isPlaying){
        m_initT = m_log[m_index].time;
        gettimeofday(&m_startT, NULL);
    }
}

void GLscene::setScreenSize(int w, int h)
{
    m_width = w;
    m_height = h;
}

GLscene* GLscene::m_scene = new GLscene();

