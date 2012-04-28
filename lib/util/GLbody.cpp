#include "GLlink.h"
#include "GLbody.h"

using namespace OpenHRP;
using namespace hrp;

bool GLbody::m_useAbsTransformToDraw = false;

void GLbody::useAbsTransformToDraw()
{
    m_useAbsTransformToDraw = true;
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

    for (unsigned int i=0; i<m_links.size(); i++){
        if (m_links[i]->jointId() >=0){
            if (m_links[i]->jointId() >= m_joints.size()) {
                m_joints.resize(m_links[i]->jointId()+1);
            }
            m_joints[m_links[i]->jointId()] = m_links[i];
        }
    }
}

GLbody::~GLbody(){
    for (unsigned int i=0; i<m_links.size(); i++){
        delete m_links[i];
    }
}

void GLbody::setPosture(const double *i_angles){
    for (unsigned int i=0; i<m_links.size(); i++){
        int id = m_links[i]->jointId();
        if (id >= 0){
            m_links[i]->setQ(i_angles[id]);
        }
    }
}

void GLbody::setPosition(double x, double y, double z)
{
    double *tform = m_root->getTransform();
    tform[12] = x; tform[13] = y; tform[14] = z; 
}

void GLbody::setOrientation(double r, double p, double y)
{
    double *tform = m_root->getTransform();
    Matrix33 R = rotFromRpy(r, p, y);
    tform[ 0]=R(0,0);tform[ 1]=R(1,0);tform[ 2]=R(2,0);
    tform[ 4]=R(0,1);tform[ 5]=R(1,1);tform[ 6]=R(2,1);
    tform[ 8]=R(0,2);tform[ 9]=R(1,2);tform[10]=R(2,2);
}

void GLbody::setPosture(const double *i_angles, double *i_pos, double *i_rpy){
    setPosition(i_pos[0], i_pos[1], i_pos[2]); 
    setOrientation(i_rpy[0], i_rpy[1], i_rpy[2]); 
    setPosture(i_angles);
}

void GLbody::setPosture(const dvector& i_q, const Vector3& i_p,
                        const Matrix33& i_R)
{
    double tform[16];
    tform[ 0]=i_R(0,0);tform[ 1]=i_R(1,0);tform[ 2]=i_R(2,0);tform[ 3]=0;
    tform[ 4]=i_R(0,1);tform[ 5]=i_R(1,1);tform[ 6]=i_R(2,1);tform[ 7]=0;
    tform[ 8]=i_R(0,2);tform[ 9]=i_R(1,2);tform[10]=i_R(2,2);tform[11]=0;
    tform[12]=i_p[0];tform[13]=i_p[1];tform[14]=i_p[2];tform[15]=1;
    m_root->setTransform(tform);
    for (unsigned int i=0; i<m_links.size(); i++){
        int id = m_links[i]->jointId();
        if (id >= 0){
            m_links[i]->setQ(i_q[id]);
        }
    }
}

void GLbody::draw(){
    if (m_useAbsTransformToDraw){
        for (unsigned int i=0; i<m_links.size(); i++){
            m_links[i]->draw();
        }
    }else{
        m_root->draw();
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

