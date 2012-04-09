#include <cstdio>
#include <iostream>
#include <fstream>
#include <sys/time.h>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include "GLmodel.h"

using namespace OpenHRP;
using namespace hrp;

#define DEFAULT_W 640
#define DEFAULT_H 480
#define DEFAULT_FPS 10

void compileShape(OpenHRP::BodyInfo_var i_binfo,
                  const TransformedShapeIndexSequence& tsis)
{
    ShapeInfoSequence_var sis = i_binfo->shapes();
    AppearanceInfoSequence_var ais = i_binfo->appearances();
    MaterialInfoSequence_var mis = i_binfo->materials();
    for (unsigned int l=0; l<tsis.length(); l++){
        const TransformedShapeIndex &tsi = tsis[l];
        double tform[16];
        for (int i=0; i<3; i++){
            for (int j=0; j<4; j++){
                tform[j*4+i] = tsi.transformMatrix[i*4+j];
            }
        }
        tform[3] = tform[7] = tform[11] = 0.0; tform[15] = 1.0;
        
        glPushMatrix();
        glMultMatrixd(tform);
        //printMatrix(tform);
        
        glBegin(GL_TRIANGLES);
        short index = tsi.shapeIndex;
        ShapeInfo& si = sis[index];
        const float *vertices = si.vertices.get_buffer();
        const LongSequence& triangles = si.triangles;
        const AppearanceInfo& ai = ais[si.appearanceIndex];
        const float *normals = ai.normals.get_buffer();
        //std::cout << "length of normals = " << ai.normals.length() << std::endl;
        const LongSequence& normalIndices = ai.normalIndices;
        //std::cout << "length of normalIndices = " << normalIndices.length() << std::endl;
        const int numTriangles = triangles.length() / 3;
        //std::cout << "numTriangles = " << numTriangles << std::endl;
        if (ai.colors.length()){
            glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, 
                         ai.colors.get_buffer());
        }else if (ai.materialIndex >= 0){ 
            const MaterialInfo& mi = mis[ai.materialIndex];
            glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, 
                         mi.diffuseColor);
        }else{
            std::cout << "no material" << std::endl;
        }
        for(int j=0; j < numTriangles; ++j){
            if (!ai.normalPerVertex){
                int p;
                if (normalIndices.length() == 0){
                    p = j*3;
                }else{
                    p = normalIndices[j]*3;
                }
                glNormal3fv(normals+p);
            }
            for(int k=0; k < 3; ++k){
                if (ai.normalPerVertex){
                    int p = normalIndices[j*3+k]*3;
                    glNormal3fv(normals+p);
                }
                long orgVertexIndex = si.triangles[j * 3 + k];
                int p = orgVertexIndex * 3;
                glVertex3fv(vertices+p);
            }
        }
        glEnd();
        glPopMatrix();
    }
}

GLcamera::GLcamera(const SensorInfo &i_si, OpenHRP::BodyInfo_var i_binfo,
                   GLlink *i_link) : m_name(i_si.name), m_link(i_link) {
    
    Matrix33 R;
    Vector3 axis;
    axis[0] = i_si.rotation[0];
    axis[1] = i_si.rotation[1];
    axis[2] = i_si.rotation[2];
    
    hrp::calcRodrigues(R, axis, i_si.rotation[3]);
    
    m_trans[ 0]=R(0,0);m_trans[ 1]=R(1,0);m_trans[ 2]=R(2,0);m_trans[3]=0; 
    m_trans[ 4]=R(0,1);m_trans[ 5]=R(1,1);m_trans[ 6]=R(2,1);m_trans[7]=0; 
    m_trans[ 8]=R(0,2);m_trans[ 9]=R(1,2);m_trans[10]=R(2,2);m_trans[11]=0; 
    m_trans[12]=i_si.translation[0];m_trans[13]=i_si.translation[1];
    m_trans[14]=i_si.translation[2];m_trans[15]=1; 

    // display list
    glPushMatrix();
    glMultMatrixd(m_trans);
    compileShape(i_binfo, i_si.shapeIndices);
    glPopMatrix();
    

    m_near = i_si.specValues[0];
    m_far  = i_si.specValues[1];
    m_fovy = i_si.specValues[2];
    m_width  = i_si.specValues[4];
    m_height = i_si.specValues[5];
}

GLcamera::GLcamera(int i_width, int i_height, double i_near, double i_far, double i_fovy) : m_near(i_near), m_far(i_far), m_fovy(i_fovy), m_width(i_width), m_height(i_height)
{
}

const std::string& GLcamera::name() const {
    return m_name;
}

void GLcamera::computeAbsTransform(double o_trans[16]){
    if (m_link){
        double trans[16];
        m_link->computeAbsTransform(trans);
        mulTrans(m_trans, trans, o_trans);
    }else{
        memcpy(o_trans, m_trans, sizeof(double)*16);
    }
}

void GLcamera::setView()
{
    computeAbsTransform(m_absTrans);
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fovy()*180/M_PI, 
                   (double)width() / (double)height(), 
                   near(), far());
    gluLookAt(m_absTrans[12], m_absTrans[13], m_absTrans[14], 
              m_absTrans[12]-m_absTrans[8], 
              m_absTrans[13]-m_absTrans[9], 
              m_absTrans[14]-m_absTrans[10],
              m_absTrans[4], m_absTrans[5], m_absTrans[6]);
}

void GLcamera::setTransform(double i_trans[16]){
    memcpy(m_trans, i_trans, sizeof(double)*16);
}

void GLcamera::getAbsTransform(double o_trans[16]){
    memcpy(o_trans, m_absTrans, sizeof(double)*16);
}

void GLcamera::getDepthOfLine(int i_row, float *o_depth)
{
    glReadPixels(0, i_row, width(), 1, GL_DEPTH_COMPONENT, GL_FLOAT, o_depth);
}


GLlink::GLlink(const LinkInfo &i_li, BodyInfo_var i_binfo) : m_parent(NULL), m_jointId(i_li.jointId){
    Vector3 axis;
    Matrix33 R;
    
    for (int i=0; i<3; i++){
        m_axis[i] = i_li.jointAxis[i];
        axis[i] = i_li.rotation[i];
    }
    setQ(0);
    
    hrp::calcRodrigues(R, axis, i_li.rotation[3]);
    
    m_trans[ 0]=R(0,0);m_trans[ 1]=R(1,0);m_trans[ 2]=R(2,0);m_trans[3]=0; 
    m_trans[ 4]=R(0,1);m_trans[ 5]=R(1,1);m_trans[ 6]=R(2,1);m_trans[7]=0; 
    m_trans[ 8]=R(0,2);m_trans[ 9]=R(1,2);m_trans[10]=R(2,2);m_trans[11]=0; 
    m_trans[12]=i_li.translation[0];m_trans[13]=i_li.translation[1];
    m_trans[14]=i_li.translation[2];m_trans[15]=1; 
    
    
    m_list = glGenLists(1);
    //std::cout << i_li.name << std::endl;
    //printMatrix(m_trans);
    
    glNewList(m_list, GL_COMPILE);
    
    compileShape(i_binfo, i_li.shapeIndices);

    const SensorInfoSequence& sensors = i_li.sensors;
    for (unsigned int i=0; i<sensors.length(); i++){
        const SensorInfo& si = sensors[i];
        std::string type(si.type);
        if (type == "Vision"){
            //std::cout << si.name << std::endl;
            m_cameras.push_back(new GLcamera(si,i_binfo, this));
        }
    }
        
    glEndList();
    
}
        
void GLlink::draw(){
    glPushMatrix();
    glMultMatrixd(m_absTrans);
    glCallList(m_list);
    glPopMatrix();
}

void GLlink::setParent(GLlink *i_parent){
    m_parent = i_parent;
}

void GLlink::addChild(GLlink *i_child){
    i_child->setParent(this);
    m_children.push_back(i_child);
}

void GLlink::setQ(double i_q){
    Matrix33 R;
    hrp::calcRodrigues(R, m_axis, i_q);
    m_T_j[ 0]=R(0,0);m_T_j[ 1]=R(1,0);m_T_j[ 2]=R(2,0);m_T_j[3]=0; 
    m_T_j[ 4]=R(0,1);m_T_j[ 5]=R(1,1);m_T_j[ 6]=R(2,1);m_T_j[7]=0; 
    m_T_j[ 8]=R(0,2);m_T_j[ 9]=R(1,2);m_T_j[10]=R(2,2);m_T_j[11]=0;
    m_T_j[12]=0;     m_T_j[13]=0;     m_T_j[14]=0;     m_T_j[15]=1;    
    //printf("m_T_j:\n");
    //printMatrix(m_T_j);
}

void GLlink::setTransform(double i_trans[16]){
    memcpy(m_trans, i_trans, sizeof(double)*16);
}

void GLlink::setAbsTransform(double i_trans[16]){
    memcpy(m_absTrans, i_trans, sizeof(double)*16);
}

int GLlink::jointId(){
    return m_jointId;
}

GLcamera *GLlink::findCamera(const char *i_name){
    std::string name(i_name);
    for (unsigned int i=0; i<m_cameras.size(); i++){
        if (m_cameras[i]->name() == name) return m_cameras[i];
    }
    return NULL;
}

void GLlink::computeAbsTransform(double o_trans[16]){
    if (m_parent){
        double trans1[16], trans2[16];
        mulTrans(m_T_j, m_trans, trans1);
        m_parent->computeAbsTransform(trans2);
        mulTrans(trans1, trans2, o_trans);
    }else{
        memcpy(o_trans, m_trans, sizeof(double)*16);
    }
}

GLbody::GLbody(BodyInfo_var i_binfo){
    LinkInfoSequence_var lis = i_binfo->links();
    
    for (unsigned int i=0; i<lis->length(); i++){
        m_links.push_back(new GLlink(lis[i], i_binfo));
    }
    // setup tree
    for (unsigned int i=0; i<m_links.size(); i++){
        const LinkInfo &li = lis[i];
        if (li.parentIndex < 0) m_root = m_links[i];
        for (unsigned int j=0; j<li.childIndices.length(); j++){
            m_links[i]->addChild(m_links[li.childIndices[j]]);
        }
    }
    for (unsigned int i=0; i<m_links.size(); i++){
        double T[16];
        m_links[i]->computeAbsTransform(T);
        m_links[i]->setAbsTransform(T);
    }
}

GLbody::~GLbody(){
    for (unsigned int i=0; i<m_links.size(); i++){
        delete m_links[i];
    }
}

void GLbody::setPosture(double *i_angles, double *i_pos, double *i_rpy){
    double tform[16];
    Matrix33 R = rotFromRpy(i_rpy[0], i_rpy[1], i_rpy[2]);
    tform[ 0]=R(0,0);tform[ 1]=R(1,0);tform[ 2]=R(2,0);tform[ 3]=0;
    tform[ 4]=R(0,1);tform[ 5]=R(1,1);tform[ 6]=R(2,1);tform[ 7]=0;
    tform[ 8]=R(0,2);tform[ 9]=R(1,2);tform[10]=R(2,2);tform[11]=0;
    tform[12]=i_pos[0];tform[13]=i_pos[1];tform[14]=i_pos[2];tform[15]=1;
    m_root->setTransform(tform);
    for (unsigned int i=0; i<m_links.size(); i++){
        int id = m_links[i]->jointId();
        if (id >= 0){
            m_links[i]->setQ(i_angles[id]);
        }
    }
}

void GLbody::draw(){
    for (unsigned int i=0; i<m_links.size(); i++){
        m_links[i]->draw();
    }
}

GLcamera *GLbody::findCamera(const char *i_name){
    for (unsigned int i=0; i<m_links.size(); i++){
        GLcamera *camera = m_links[i]->findCamera(i_name);
        if (camera) return camera;
    }
    return NULL;
}

GLlink *GLbody::link(unsigned int i)
{
    if (i >= m_links.size()) return NULL;
    return m_links[i];
}

void setWorldState(std::map<std::string, GLbody *>& models,
                   const WorldState& state)
{ 
    for (unsigned int i=0; i<state.characterPositions.length(); i++){
        const CharacterPosition& cpos = state.characterPositions[i];
        std::string cname(cpos.characterName);
        GLbody *body = models[cname];
        if (!body) {
            //std::cerr << "can't find a body named " << cname << std::endl;
            continue;
        }
        for (unsigned int j=0; j<cpos.linkPositions.length(); j++){
            const LinkPosition &lp = cpos.linkPositions[j];
            double T[] = {lp.R[0], lp.R[3], lp.R[6],0,
                          lp.R[1], lp.R[4], lp.R[7],0,
                          lp.R[2], lp.R[5], lp.R[8],0,
                          lp.p[0], lp.p[1], lp.p[2],1};
#if 0
            for (int i=0; i<4; i++){
                for (int j=0; j<4; j++){
                    printf("%6.3f ", T[i*4+j]);
                }
                printf("\n");
            }
            printf("\n");
#endif
            body->link(j)->setAbsTransform(T);
        }
    }
}



void GLscene::addBody(const std::string &i_name, GLbody *i_body){
    //std::cout <<"addBody(" << i_name << "," << i_body << ")" << std::endl;
    m_bodies[i_name] = i_body;
}

GLbody *GLscene::findBody(const std::string &i_name)
{
    return m_bodies[i_name];
}

void drawString(const char *str)
{
    for (unsigned int i=0; i<strlen(str); i++){
        glutBitmapCharacter(GLUT_BITMAP_9_BY_15, str[i]);
    }
}

void GLscene::draw(){
    if (m_isPlaying){
        // compute time to draw
        struct timeval tv;
        gettimeofday(&tv, NULL);
        double drawT = m_initT + ((tv.tv_sec - m_startT.tv_sec) + (tv.tv_usec - m_startT.tv_usec)*1e-6)*m_playRatio;
        //
        while(drawT > m_log[m_index].time){
            if (m_index+1 >= (int)m_log.size()){
                m_isPlaying = false;
                break;
            }else{
                m_index++;
            }
        }
    } else if (m_isNewStateAdded){
        // draw newest state
        m_index = m_log.size() - 1;
        m_isNewStateAdded = false;
    }
    
    if (m_index >= 0) setWorldState(m_bodies, m_log[m_index]);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // robots
    std::map<std::string, GLbody *>::iterator it;
    for (it=m_bodies.begin(); it!=m_bodies.end(); it++){
        if (it->second){
            it->second->draw();
        }
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

    // lines
    if (m_index >=0){
        glColor3f(1,0,0);
        const CollisionSequence &cs = m_log[m_index].collisions;
        for (unsigned int i=0; i<cs.length(); i++){
            const CollisionPointSequence& cps = cs[i].points; 
            for (unsigned int j=0; j<cps.length(); j++){
                glVertex3dv(cps[j].position);
                for (int k=0; k<3; k++){
                    e[k] = cps[j].position[k] + cps[j].normal[k]*(cps[j].idepth*10+0.1);
                }
                glVertex3dv(e);
            }
        }
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
        sprintf(buf, "Time:%6.3f[s]" , m_log[m_index].time);
    }else{
        sprintf(buf, "Time:------[s]");
    }
    drawString(buf);
    glRasterPos2f(10, m_height-30);
    sprintf(buf, "Playback x%6.3f", m_playRatio);
    drawString(buf);
    glPopMatrix();

    glEnable(GL_LIGHTING);

    if(m_isRecording){
        while(m_initT > m_log[m_index].time){
            if (m_index+1 >= (int)m_log.size()){
                m_isRecording = false;
                break;
            }else{
                m_index++;
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
        m_initT += 1.0/DEFAULT_FPS;
    }
}

void mulTrans(const double i_m1[16], const double i_m2[16], double o_m[16])
{
    for (int i=0; i<4; i++){
        for (int j=0; j<4;j++){
            double v = 0;
            for (int k=0; k<4; k++){
                v += i_m1[i*4+k]*i_m2[j+k*4];
            }
            o_m[i*4+j] = v;
        }
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
      m_index(-1), m_playRatio(1.0), m_width(DEFAULT_W), m_height(DEFAULT_H)
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

void GLscene::addState(const WorldState& state)
{
    m_log.push_back(state);
    m_isNewStateAdded = true;
}

void GLscene::clearLog()
{
    m_log.clear();
    m_index = -1;
}

void GLscene::reserveLog(unsigned int len)
{
    m_log.reserve(len);
}

void GLscene::play()
{
    if (m_log.size() == 0) return;

    if (!m_isPlaying){
        m_isPlaying = true;
        if (m_index+1 == (int)m_log.size()){
            m_index = 0;
        }
        m_initT = m_log[m_index].time;
        gettimeofday(&m_startT, NULL);
    }else{
        m_isPlaying = false;
    }
}

void GLscene::record()
{
    if (m_log.size() == 0) return;

    m_index = 0;
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
    if (!m_log.size()) return;

    m_index -= delta;
    if (m_index < 0) m_index = 0;
}

void GLscene::next(int delta)
{
    m_index += delta;
    if (m_index >= (int)m_log.size()) m_index = m_log.size()-1;
}

void GLscene::head()
{
    m_index = 0;
}

void GLscene::tail()
{
    m_index = m_log.size()-1;
    if (m_index < 0) m_index = 0;
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

