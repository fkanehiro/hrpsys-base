#include <iostream>
#include <cstdio>
#include <math.h>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include <hrpModel/ModelLoaderUtil.h>
#include "GLcamera.h"
#include "GLbody.h"
#include "GLlink.h"
#include "GLsceneBase.h"
#include "LogManagerBase.h"

static void drawString(const char *str)
{
    for (unsigned int i=0; i<strlen(str); i++){
        glutBitmapCharacter(GLUT_BITMAP_9_BY_15, str[i]);
    }
}

GLsceneBase::GLsceneBase(LogManagerBase *i_log) : 
    m_width(DEFAULT_W), m_height(DEFAULT_H),
    m_showingStatus(false), m_showSlider(false),
    m_log(i_log), m_videoWriter(NULL), m_cvImage(NULL), m_isNewBody(false),
    m_showFloorGrid(true), m_showInfo(true), m_clearRequested(false)
{
    m_default_camera = new GLcamera(DEFAULT_W, DEFAULT_H, 1.0, 100.0, 40*M_PI/180);
    m_default_camera->setViewPoint(4,0,0.8);
    m_default_camera->setViewTarget(0,0,0.8);
    m_camera = m_default_camera;
    m_sem = SDL_CreateSemaphore(0);
}

GLsceneBase::~GLsceneBase()
{
    SDL_DestroySemaphore(m_sem);
    delete m_default_camera;
}


void GLsceneBase::setScreenSize(int w, int h){
    m_width = w;
    m_height = h;
}

void GLsceneBase::addBody(const std::string &i_name, OpenHRP::BodyInfo_var i_binfo){
    m_newBodyName = i_name;
    m_newBodyInfo = i_binfo;
    m_isNewBody = true;
    SDL_SemWait(m_sem);
}

void GLsceneBase::setCamera(GLcamera *i_camera)
{
    if (!i_camera) return;

    m_camera = i_camera;
    //glfwSetWindowSize(m_camera->width(), m_camera->height());
    glViewport(0, 0, m_camera->width(), m_camera->height());
}

GLcamera *GLsceneBase::getCamera()
{
    return m_camera;
}

GLcamera *GLsceneBase::getDefaultCamera()
{
    return m_default_camera;
}

void GLsceneBase::save(const char *i_fname)
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

void GLsceneBase::capture(char *o_buffer)
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

void GLsceneBase::init()
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

void GLsceneBase::showFloorGrid(bool flag)
{
    m_showFloorGrid = flag;
}

void GLsceneBase::drawFloorGrid()
{
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
}

void GLsceneBase::showInfo(bool flag)
{
    m_showInfo = flag;
}

void GLsceneBase::drawInfo(double fps)
{
    glColor3d(1.0,1.0,1.0);
    glRasterPos2f(10, m_height-15);
    char buf[256];
    double tm = m_log->currentTime();
    if (tm >= 0){
        sprintf(buf, "Time:%6.3f[s]" , tm);
    }else{
        sprintf(buf, "Time:------[s]");
    }
    drawString(buf);
    glRasterPos2f(10, m_height-30);
    sprintf(buf, "Playback x%6.3f", m_log->playRatio());
    drawString(buf);
    glRasterPos2f(10, m_height-45);
    sprintf(buf, "FPS %2.0f", fps);
    drawString(buf);
    for (unsigned int i=0; i<m_msgs.size(); i++){
        glRasterPos2f(10, (m_msgs.size()-i)*15);
        drawString(m_msgs[i].c_str());
    }
    showStatus();
}

void GLsceneBase::draw()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    double fps = 1.0/((tv.tv_sec - m_lastDraw.tv_sec)+(tv.tv_usec - m_lastDraw.tv_usec)/1e6);
    m_lastDraw = tv;

    if (m_clearRequested) {
        clearBodies();
        m_clearRequested = false;
        SDL_SemPost(m_sem);
    }

    if (m_isNewBody){
        GLbody *glbody = new GLbody();
        glbody->setName(m_newBodyName);
        hrp::BodyPtr body(glbody);
        hrp::loadBodyFromBodyInfo(body, m_newBodyInfo, false, GLlinkFactory);
        glbody->setDrawInfo(m_newBodyInfo);
        WorldBase::addBody(body);
        m_isNewBody = false;
        SDL_SemPost(m_sem);
    }

    int index = m_log->updateIndex();
    
    updateScene();

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // robots
    for (int i=0; i<numBodies(); i++){
        ((GLbody *)body(i).get())->draw();
    }

    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);

    if (m_showFloorGrid) drawFloorGrid();
    drawAdditionalLines();
    glEnd();

    // draw texts
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, m_width, 0, m_height);
    glPushMatrix();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    if (m_showInfo) drawInfo(fps);
    if (m_showSlider){
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_BLEND);
        glColor4f(0.0,0.0,0.0, 0.5);
        glRectf(SLIDER_SIDE_MARGIN,10,m_width-SLIDER_SIDE_MARGIN,20);
        unsigned int len = m_log->length();
        if (len>1){
            int x = ((double)index)/(len-1)*(m_width-20)+10;
            glRectf(x-5,5,x+5,25);
        }
        glDisable(GL_BLEND);
    }
    glPopMatrix();
    glEnable(GL_LIGHTING);

    if (m_log->isRecording() && !m_videoWriter){
        m_videoWriter = cvCreateVideoWriter(
            "olv.avi",
            CV_FOURCC('D','I','V','X'),
            DEFAULT_FPS,
            cvSize(m_width, m_height));
        m_cvImage = cvCreateImage(
            cvSize(m_width, m_height),
            IPL_DEPTH_8U, 3);
    }
    if(!m_log->isRecording() && m_videoWriter){
        char *dst = m_cvImage->imageData;
        capture(dst);
        cvWriteFrame(m_videoWriter, m_cvImage);
        if (!m_log->isRecording()){
            cvReleaseVideoWriter(&m_videoWriter);
            cvReleaseImage(&m_cvImage);
            m_videoWriter = NULL;
            m_cvImage = NULL;
        }
    }
}

void GLsceneBase::clear()
{
    m_clearRequested = true;
    SDL_SemWait(m_sem);
}

