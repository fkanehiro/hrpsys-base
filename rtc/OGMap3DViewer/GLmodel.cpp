#include <cstdio>
#include <fstream>
#include <GL/glfw.h>
#include <sys/time.h>
#include "GLmodel.h"

using namespace OpenHRP;
using namespace hrp;

#define DEFAULT_W 640
#define DEFAULT_H 480

GLcamera::GLcamera(const SensorInfo &i_si, GLlink *i_link) : m_name(i_si.name), m_link(i_link) {
    
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
    
    ShapeInfoSequence_var sis = i_binfo->shapes();
    AppearanceInfoSequence_var ais = i_binfo->appearances();
    MaterialInfoSequence_var mis = i_binfo->materials();
    const TransformedShapeIndexSequence& tsis = i_li.shapeIndices;
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
        if (ai.materialIndex >= 0){ 
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
    
    glEndList();
    
    const SensorInfoSequence& sensors = i_li.sensors;
    for (unsigned int i=0; i<sensors.length(); i++){
        const SensorInfo& si = sensors[i];
        std::string type(si.type);
        if (type == "Vision"){
            //std::cout << si.name << std::endl;
            m_cameras.push_back(new GLcamera(si, this));
        }
    }
        
}
        
void GLlink::draw(){
    glPushMatrix();
    glMultMatrixd(m_trans);
    glMultMatrixd(m_T_j);
    glCallList(m_list);
    for (unsigned int i=0; i<m_children.size(); i++){
        m_children[i]->draw();
    }
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
    m_root->draw();
}

GLcamera *GLbody::findCamera(const char *i_name){
    for (unsigned int i=0; i<m_links.size(); i++){
        GLcamera *camera = m_links[i]->findCamera(i_name);
        if (camera) return camera;
    }
    return NULL;
}

void GLscene::addBody(GLbody *i_body){
    m_bodies.push_back(i_body);
}

void GLscene::draw(bool swap){
    m_camera->setView();
#if 0
    struct timeval tv1, tv2;
    gettimeofday(&tv1, NULL);
#endif
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);

    for (unsigned int i=0; i<m_bodies.size(); i++){
        m_bodies[i]->draw();
    }

    if (swap) glfwSwapBuffers();
#if 0
    gettimeofday(&tv2, NULL);
    //std::cout << "t = " << ((tv2.tv_sec - tv1.tv_sec)*1e3 + (tv2.tv_usec - tv1.tv_usec)*1e-3) << "[ms]" << std::endl;
    //std::cout << (int)(1.0/((tv2.tv_sec - tv1.tv_sec) + (tv2.tv_usec - tv1.tv_usec)*1e-6)) << "[FPS]" << std::endl;
#endif
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

GLscene::GLscene() : m_camera(m_default_camera)
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
    glfwInit();

    glfwOpenWindow(DEFAULT_W,DEFAULT_H,0,0,0,0,24,0, GLFW_WINDOW);

    setCamera(m_default_camera);

    GLfloat light0pos[] = { 0.0, 4.0, 6.0, 1.0 };
    GLfloat light1pos[] = { 6.0, 4.0, 0.0, 1.0 };
    GLfloat white[] = { 0.6, 0.6, 0.6, 1.0 };

    glClearColor(0, 0, 0, 1.0);
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
    int w, h;
    glfwGetWindowSize(&w,&h);
    unsigned char *buffer = new unsigned char[w*h*3];

    capture(buffer);
    std::ofstream ofs(i_fname, std::ios::out | std::ios::trunc | std::ios::binary );
    char buf[10];
    sprintf(buf, "%d %d", w, h);
    ofs << "P6" << std::endl << buf << std::endl << "255" << std::endl;
    for (int i=0; i<h; i++){
        ofs.write((char *)(buffer+i*w*3), w*3);
    }
    delete [] buffer;
}

void GLscene::capture(unsigned char *o_buffer)
{
    int w, h;
    glfwGetWindowSize(&w,&h);
    glReadBuffer(GL_BACK);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    for (int i=0; i<h; i++){
        glReadPixels(0,(h-1-i),w,1,GL_RGB,GL_UNSIGNED_BYTE,
                     o_buffer + i*3*w);
    }
}

void GLscene::setCamera(GLcamera *i_camera)
{
    if (!i_camera) return;

    m_camera = i_camera;
    glfwSetWindowSize(m_camera->width(), m_camera->height());
    glViewport(0, 0, m_camera->width(), m_camera->height());
}

GLcamera *GLscene::getCamera()
{
    return m_camera;
}

GLscene* GLscene::m_scene = new GLscene();


unsigned int GLscene::numBodies() const
{
    return m_bodies.size();
}

GLbody *GLscene::body(unsigned int i_rank)
{
    if (i_rank >= numBodies()) return NULL;
    return m_bodies[i_rank];
}
