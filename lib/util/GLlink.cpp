#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#include "GLutil.h"
#include "GLcamera.h"
#include "GLlink.h"

using namespace OpenHRP;
using namespace hrp;

bool GLlink::m_useAbsTransformToDraw = false;

void GLlink::useAbsTransformToDraw()
{
    m_useAbsTransformToDraw = true;
}

GLlink::GLlink(const LinkInfo &i_li, BodyInfo_var i_binfo) : m_parent(NULL), m_jointId(i_li.jointId){
    Vector3 axis;
    Matrix33 R;
    
    m_name = i_li.name;
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
    
    CORBA::String_var jointType = i_li.jointType;
    const std::string jt( jointType );
    if(jt == "fixed" ){
        m_jointType = FIXED_JOINT;
    } else if(jt == "free" ){
        m_jointType = FREE_JOINT;
    } else if(jt == "rotate" ){
        m_jointType = ROTATIONAL_JOINT;
    } else if(jt == "slide" ){
        m_jointType = SLIDE_JOINT;
    }
    
    m_list = glGenLists(1);
    //std::cout << i_li.name << std::endl;
    //printMatrix(m_trans);
    
    glNewList(m_list, GL_COMPILE);
    
    m_textures = compileShape(i_binfo, i_li.shapeIndices);

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

GLlink::~GLlink()
{
    for (unsigned int i=0; i<m_cameras.size(); i++){
        delete m_cameras[i];
    }
    for (unsigned int i=0; i<m_textures.size(); i++){
        glDeleteTextures(1, &m_textures[i]);
    }
    glDeleteLists(m_list, 1);
}
        
void GLlink::draw(){
    glPushMatrix();
    if (m_useAbsTransformToDraw){
        glMultMatrixd(m_absTrans);
    }else{
        glMultMatrixd(m_trans);
        glMultMatrixd(m_T_j);
    }
    glCallList(m_list);
    if (!m_useAbsTransformToDraw){
        for (unsigned int i=0; i<m_children.size(); i++){
            m_children[i]->draw();
        }
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

