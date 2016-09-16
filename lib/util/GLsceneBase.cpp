#include <iostream>
#include <fstream>
#include <cstdio>
#include <math.h>
#include <GL/glew.h>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpModel/Sensor.h>
#include <hrpModel/Light.h>
#include "GLcamera.h"
#include "GLbody.h"
#include "GLlink.h"
#include "GLutil.h"
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
    m_log(i_log), m_videoWriter(NULL), m_cvImage(NULL), 
    m_showFloorGrid(true), m_showInfo(true), m_defaultLights(true),
    m_request(REQ_NONE), 
    m_maxEdgeLen(0),
    m_targetObject(-1),
    m_isCapturing(false)
{
    m_default_camera = new GLcamera(DEFAULT_W, DEFAULT_H, 0.1, 100.0, 30*M_PI/180);
    m_default_camera->setViewPoint(4,0,0.8);
    m_default_camera->setViewTarget(0,0,0.8);
    m_camera = m_default_camera;
    m_sem = SDL_CreateSemaphore(0);
    m_bgColor[0] = m_bgColor[1] = m_bgColor[2] = 0.0;

    hrp::Light::nextId = 2;
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

void GLsceneBase::setCamera(GLcamera *i_camera)
{
    if (!i_camera) return;

    m_camera = i_camera;
}

void GLsceneBase::nextCamera()
{
    bool found = m_camera == m_default_camera ? true : false;
    for (int i=0; i<numBodies(); i++){
        hrp::BodyPtr b = body(i);
        for (int j=0; j<b->numLinks(); j++){
            GLlink *l = dynamic_cast<GLlink *>(b->link(j));
            const std::vector<GLcamera *>& cameras = l->cameras();
            for (size_t k=0; k<cameras.size(); k++){
                if (cameras[k] == m_camera){
                    found = true;
                }else if(found){
                    m_camera = cameras[k];
                    return;
                }
            }
        } 
    }
    m_camera = m_default_camera;
}

void GLsceneBase::nextObject()
{
    m_targetObject++;
    if (m_targetObject == numBodies()) m_targetObject = -1;
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
    char pixels[m_width*m_height*3];

    glReadBuffer(GL_BACK);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glReadPixels(0,0, m_width,m_height,GL_RGB,GL_UNSIGNED_BYTE, pixels);

    std::ofstream ofs(i_fname, std::ios::out | std::ios::trunc | std::ios::binary );
    char buf[10];
    sprintf(buf, "%d %d", m_width, m_height);
    ofs << "P6" << std::endl << buf << std::endl << "255" << std::endl;
    for (int i=m_height-1; i>=0; i--){
        ofs.write((char *)(pixels+i*m_width*3), m_width*3);
    }
}

void GLsceneBase::capture(char *o_buffer)
{
    glReadBuffer(GL_BACK);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    char *buf  = new char[m_width*m_height*3];
    glReadPixels(0,0, m_width,m_height,GL_BGR,GL_UNSIGNED_BYTE, buf);
    char *dst = o_buffer, *src;
    for (int i=0; i<m_height; i++){
        src = buf + (m_height -1 - i)*m_width*3;
        memcpy(dst, src, m_width*3);
        dst += m_width*3;
    }
    delete [] buf;
}

void GLsceneBase::initLights()
{
    GLfloat light0pos[] = { 40.0, 60.0, 60.0, 1.0 };
    GLfloat light1pos[] = { -40.0, -60.0, -60.0, 1.0 };
    GLfloat light0col[] = { 0.9, 0.9, 1.0, 1.0 };
    GLfloat light1col[] = { 0.5, 0.4, 0.4, 1.0 };

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light0col);
    //glLightfv(GL_LIGHT0, GL_SPECULAR, light0col);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, light1col);
    //glLightfv(GL_LIGHT1, GL_SPECULAR, light1col);
    glLightfv(GL_LIGHT0, GL_POSITION, light0pos);
    glLightfv(GL_LIGHT1, GL_POSITION, light1pos);
}

void GLsceneBase::defaultLights(bool flag)
{
    if (m_defaultLights == flag) return;

    m_defaultLights = flag;
    if (flag){
        glEnable(GL_LIGHT0);
        glEnable(GL_LIGHT1);
    }else{
        glDisable(GL_LIGHT0);
        glDisable(GL_LIGHT1);
    }
}

bool GLsceneBase::defaultLights()
{
    return m_defaultLights;
}

void GLsceneBase::init()
{
    setCamera(m_default_camera);

    glewInit();
    initLights();

    glClearColor(m_bgColor[0], m_bgColor[1], m_bgColor[2], 0.0);
    glEnable(GL_DEPTH_TEST);

    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);
}

void GLsceneBase::showFloorGrid(bool flag)
{
    m_showFloorGrid = flag;
}

bool GLsceneBase::showFloorGrid()
{
    return m_showFloorGrid;
}

void GLsceneBase::drawFloorGrid()
{
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
}

void GLsceneBase::showInfo(bool flag)
{
    m_showInfo = flag;
}

void GLsceneBase::drawInfo(double fps, size_t ntri)
{
    glColor3d(1.0,1.0,1.0);
    int h = m_height-15;
    glRasterPos2f(10, h);
    char buf[256];
    double tm = m_log->currentTime();
    if (tm >= 0){
        sprintf(buf, "Time:%6.3f[s]" , tm);
    }else{
        sprintf(buf, "Time:------[s]");
    }
    drawString(buf);
    h -= 15;
    glRasterPos2f(10, h);
    sprintf(buf, "Playback x%6.3f", m_log->playRatio());
    drawString(buf);
    h -= 15;
    glRasterPos2f(10, h);
    sprintf(buf, "FPS %2.0f(%6zutris)", fps, ntri);
    drawString(buf);
    if (m_camera != m_default_camera){
        sprintf(buf, "Camera: %s.%s", 
                m_camera->link()->body->name().c_str(), 
                m_camera->name().c_str());
        h -= 15;
        glRasterPos2f(10, h);
        drawString(buf);
    }
    if (m_targetObject >=0 && m_targetObject < numBodies()){
        sprintf(buf, "Target: %s", targetObject()->name().c_str());
        h -= 15;
        glRasterPos2f(10, h);
        drawString(buf);
    }
    for (unsigned int i=0; i<m_msgs.size(); i++){
        glRasterPos2f(10, (m_msgs.size()-i)*15);
        drawString(m_msgs[i].c_str());
    }
    showStatus();
}

size_t GLsceneBase::drawObjects(bool showSensors)
{
    size_t ntri = 0;
    boost::function2<void, hrp::Body *, hrp::Sensor *> callback;
    for (int i=0; i<numBodies(); i++){
        GLbody *glbody = dynamic_cast<GLbody *>(body(i).get());
        if (!glbody) std::cout << "dynamic_cast failed" << std::endl;
        if (!showSensors) {
            callback = glbody->getSensorDrawCallback();
            glbody->setSensorDrawCallback(NULL);
        }
        ntri += glbody->draw();
        if (!showSensors) glbody->setSensorDrawCallback(callback);

    }
    return ntri;
}

void GLsceneBase::draw()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    double fps = 1.0/((tv.tv_sec - m_lastDraw.tv_sec)+(tv.tv_usec - m_lastDraw.tv_usec)/1e6);
    m_lastDraw = tv;

    if (m_request == REQ_CLEAR) {
        clear();
        m_request = REQ_NONE;
        SDL_SemPost(m_sem);
    }

    int index = m_log->updateIndex();
    
    updateScene();

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    size_t ntri = drawObjects();

    glDisable(GL_LIGHTING);

    if (m_showFloorGrid) drawFloorGrid();
    drawAdditionalLines();

    // draw texts
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, m_width, 0, m_height);
    glPushMatrix();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    if (m_showInfo) drawInfo(fps, ntri);
    if (m_showSlider){
        glColor4f(0.0,0.0,0.0, 0.5);
        glRectf(SLIDER_SIDE_MARGIN,10,m_width-SLIDER_SIDE_MARGIN,20);
        unsigned int len = m_log->length();
        if (len>1){
            int x = ((double)index)/(len-1)*(m_width-20)+10;
            glRectf(x-5,5,x+5,25);
        }
    }
    glPopMatrix();
    glEnable(GL_LIGHTING);

    if (m_log->isRecording() && !m_isCapturing && !m_videoWriter){
        m_videoWriter = cvCreateVideoWriter(
            "olv.avi",
            CV_FOURCC('D','I','V','X'),
            m_log->fps(),
            cvSize(m_width, m_height));
        m_cvImage = cvCreateImage(
            cvSize(m_width, m_height),
            IPL_DEPTH_8U, 3);
    }
    if(m_videoWriter){
        char *dst = m_cvImage->imageData;
        capture(dst);
        cvWriteFrame(m_videoWriter, m_cvImage);
    }
    if(m_isCapturing){
        char fname[64];
        sprintf(fname, "capture%05.2f.png", m_log->time());
        save(fname);
    }
    if (!m_log->isRecording()){
        if (m_videoWriter){
            cvReleaseVideoWriter(&m_videoWriter);
            cvReleaseImage(&m_cvImage);
            m_videoWriter = NULL;
            m_cvImage = NULL;
        }
        if (m_isCapturing) m_isCapturing = false;
    }
    if (m_request == REQ_CAPTURE){
        save(m_fname.c_str());
        m_request = REQ_NONE;
        SDL_SemPost(m_sem);
    }

    // offscreen redering
    for (int i=0; i<numBodies(); i++){
        hrp::BodyPtr b = body(i);
        for (int j=0; j<b->numLinks(); j++){
            GLlink *l = dynamic_cast<GLlink *>(b->link(j));
            const std::vector<GLcamera *>& cameras = l->cameras();
            for (size_t k=0; k<cameras.size(); k++){
                hrp::VisionSensor *s = cameras[k]->sensor();
                if (!s->isEnabled) continue;
                if (s->nextUpdateTime < m_log->currentTime()){
                    cameras[k]->render(this);
                    s->nextUpdateTime += 1.0/s->frameRate;
                } 
            }
        } 
    }
}

void GLsceneBase::requestClear()
{
    m_request = REQ_CLEAR;
    SDL_SemWait(m_sem);
}

void GLsceneBase::requestCapture(const char *i_fname)
{
    m_fname = i_fname;
    m_request = REQ_CAPTURE;
    SDL_SemWait(m_sem);
}

void GLsceneBase::clear()
{
    clearBodies();
    m_camera = m_default_camera;
}

void GLsceneBase::setView()
{
    glViewport(0,0,m_width, m_height);
    m_camera->setView(m_width, m_height);
}

void GLsceneBase::addBody(hrp::BodyPtr i_body)
{
    if (m_maxEdgeLen){
        GLbody *glbody = dynamic_cast<GLbody *>(i_body.get());
        if (glbody) glbody->divideLargeTriangles(m_maxEdgeLen);
    }
    WorldBase::addBody(i_body);
}

void GLsceneBase::maxEdgeLen(double i_len)
{
    m_maxEdgeLen = i_len;
}

hrp::BodyPtr GLsceneBase::targetObject()
{
    if (m_targetObject >= 0 && m_targetObject < numBodies()){
        return body(m_targetObject);
    }else{
        return hrp::BodyPtr();
    }
}

void GLsceneBase::setBackGroundColor(float rgb[3])
{
    for (int i=0; i<3; i++) m_bgColor[i] = rgb[i];
}

hrp::Vector3 GLsceneBase::center()
{
    hrp::Vector3 mi,ma,min,max;
    for (int i=0; i<numBodies(); i++){
        GLbody *glbody = dynamic_cast<GLbody *>(body(i).get());
        glbody->computeAABB(mi,ma);
        if (i==0){
            min = mi; max = ma;
        }else{
            for (int j=0; j<3; j++){
                if (min[j] > mi[j]) min[j] = mi[j];
                if (max[j] < ma[j]) max[j] = ma[j];
            }
        }
    }
    return (min+max)/2;
}
