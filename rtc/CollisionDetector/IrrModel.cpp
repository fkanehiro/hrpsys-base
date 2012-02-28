#include <hrpUtil/Tvmet3d.h>
#include <hrpUtil/Tvmet4d.h>
#include "IrrModel.h"

using namespace irr;

using namespace core;
using namespace scene;
using namespace video;
using namespace io;

using namespace OpenHRP;
using namespace hrp;

class MyEventReceiver : public IEventReceiver
{
public:
    MyEventReceiver(ICameraSceneNode *i_camera, double i_radius=3.0) : 
        m_camera(i_camera),
        m_radius(i_radius),
        m_pan(-M_PI/6),
        m_tilt(0){
        updateEye();
    }

    virtual bool OnEvent(const SEvent& event)
        {
            if(event.EventType == EET_KEY_INPUT_EVENT)
                {
                    if(event.KeyInput.PressedDown)
                        {
                            switch(event.KeyInput.Key)
                                {
                                case KEY_LEFT:
                                    return true;
                                case KEY_RIGHT:
                                    return true;
                                case KEY_UP:
                                    return true;
                                case KEY_DOWN:
                                    return true;
                                default:
                                    return false;
                                }
                        }
                    return true;
                }
            if(event.EventType == EET_MOUSE_INPUT_EVENT)
                {
                    switch(event.MouseInput.Event)
                        {
                        case EMIE_LMOUSE_PRESSED_DOWN:
                            m_mouse.X = event.MouseInput.X;
                            m_mouse.Y = event.MouseInput.Y;
                            return true;
                        case EMIE_MOUSE_WHEEL:
                            //上回転
                            if(event.MouseInput.Wheel == 1)
                                {
                                    if (m_radius > 0.001){
                                        m_radius *= 0.9;
                                        updateEye();
                                    }
                                }
                            //下回転
                            else if(event.MouseInput.Wheel == -1)
                                {
                                    m_radius *= 1.1;
                                    updateEye();
                                }
                            return true;
                        case EMIE_MOUSE_MOVED:
                            if (event.MouseInput.isLeftPressed()){
                                s32 dx = event.MouseInput.X - m_mouse.X;
                                s32 dy = event.MouseInput.Y - m_mouse.Y;
                                m_pan  += dx*0.01;
                                m_tilt += dy*0.01;
                                if (m_tilt < -M_PI/2) m_tilt = -M_PI/2;
                                if (m_tilt >  M_PI/2) m_tilt =  M_PI/2;
                                updateEye();
                                m_mouse.X = event.MouseInput.X;
                                m_mouse.Y = event.MouseInput.Y;
                            }
                            return true;

                        default:
                            return true;
                        }
                }
            return false;
        }
private:
    void updateEye(){
        vector3df target = m_camera->getTarget();
        m_eye.X = target.X + m_radius*cos(m_tilt)*cos(m_pan);
        m_eye.Y = target.Y + m_radius*cos(m_tilt)*sin(m_pan);
        m_eye.Z = target.Z + m_radius*sin(m_tilt);
        m_camera->setPosition(m_eye);
    }
    ICameraSceneNode *m_camera;
    position2d<s32> m_mouse;
    vector3df m_eye;
    float m_radius, m_pan, m_tilt;
};

GLlink::GLlink(ISceneNode *i_parent, ISceneManager *i_mgr, s32 i_id,
               const LinkInfo &i_li, BodyInfo_var i_binfo) :
    ISceneNode(i_parent, i_mgr, i_id),
    m_jointId(i_li.jointId){
    setAutomaticCulling(scene::EAC_OFF);
    
    
    setPosition(vector3df( i_li.translation[0], 
                           -i_li.translation[1], 
                           i_li.translation[2]));
    Vector3 axis(i_li.rotation[0],
                 i_li.rotation[1],
                 i_li.rotation[2]);
    Matrix33 R;
    hrp::calcRodrigues(R, axis, i_li.rotation[3]);
    Vector3 rpy(rpyFromRot(R));
    //std::cout << "rpy:" << rpy << std::endl;
    setRotation(vector3df(-180/M_PI*rpy[0],
                          180/M_PI*rpy[1],
                          -180/M_PI*rpy[2]));
    
    m_axis = i_li.jointAxis[0], i_li.jointAxis[1], i_li.jointAxis[2];
    
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
    
    ShapeInfoSequence_var sis = i_binfo->shapes();
    AppearanceInfoSequence_var ais = i_binfo->appearances();
    MaterialInfoSequence_var mis = i_binfo->materials();
    TextureInfoSequence_var txs = i_binfo->textures();
    const TransformedShapeIndexSequence& tsis = i_li.shapeIndices;
    
    
    core::vector3df vertex;
    core::vector3df normal;
    
    for (unsigned int l=0; l<tsis.length(); l++){
        SMesh* mesh = new SMesh();
        SMeshBuffer* meshBuffer = new SMeshBuffer();
        mesh->addMeshBuffer(meshBuffer);
        meshBuffer->drop();
        
        const TransformedShapeIndex &tsi = tsis[l];
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

        video::SColor color(0xffffffff);
        if (ai.materialIndex >= 0){ 
            const MaterialInfo& mi = mis[ai.materialIndex];
            color.set(0xff, 
                      0xff*mi.diffuseColor[0], 
                      0xff*mi.diffuseColor[1], 
                      0xff*mi.diffuseColor[2]);
        }else{
            std::cout << "no material" << std::endl;
        }
        
        
        SMeshBuffer* mb = reinterpret_cast<SMeshBuffer*>(mesh->getMeshBuffer(mesh->getMeshBufferCount()-1));
        u32 vCount = mb->getVertexCount();
        
        const float *textureCoordinate = NULL;
        if (ai.textureIndex >= 0){
            textureCoordinate = ai.textureCoordinate.get_buffer();
            //std::cout << "length of textureCoordinate:" << ai.textureCoordinate.length() << std::endl;
            //std::cout << "length of vertices:" << si.vertices.length() << std::endl;
            
        }
        
        for(int j=0; j < numTriangles; ++j){
            if (!ai.normalPerVertex){
                int p;
                if (normalIndices.length() == 0){
                    p = j*3;
                }else{
                    p = normalIndices[j]*3;
                }
		if ( normals != NULL ) {
		  normal.X =  normals[p];
		  normal.Y = -normals[p+1]; //left-handed->right-handed
		  normal.Z =  normals[p+2];
		} else {
		  normal.X = 0;
		  normal.Y = 0;
		  normal.Z = 1;
		}
            }
            for(int k=0; k < 3; ++k){
                if (ai.normalPerVertex){
                    int p = normalIndices[j*3+k]*3;
		    normal.X =  normals[p];
		    normal.Y = -normals[p+1]; //left-handed -> right-handed
		    normal.Z =  normals[p+2];
                }
                long orgVertexIndex = si.triangles[j * 3 + k];
                int p = orgVertexIndex * 3;
                vertex.X =  vertices[p];
                vertex.Y = -vertices[p+1]; // left-handed -> right-handed
                vertex.Z =  vertices[p+2];
                //std::cout << vertices[p] <<"," << vertices[p+1] << "," << vertices[p+2] << std::endl;
                vector2df texc;
                if (textureCoordinate){
                    
                    texc.X = textureCoordinate[ai.textureCoordIndices[j*3+k]*2];
                    texc.Y = textureCoordinate[ai.textureCoordIndices[j*3+k]*2+1];
                }
                // redundant vertices
                mb->Vertices.push_back(video::S3DVertex(vertex,normal,color, texc));
            }
            mb->Indices.push_back(vCount);
            mb->Indices.push_back(vCount+2);
            mb->Indices.push_back(vCount+1);
            vCount += 3;
        }
        mesh->getMeshBuffer(0)->recalculateBoundingBox();
        
        // Create the Animated mesh if there's anything in the mesh
        SAnimatedMesh* pAM = 0;
        if ( 0 != mesh->getMeshBufferCount() )
            {
                mesh->recalculateBoundingBox();
                pAM = new SAnimatedMesh();
                pAM->Type = EAMT_OBJ;
                pAM->addMesh(mesh);
                pAM->recalculateBoundingBox();
            }
        
        mesh->drop();
        
        const DblArray12& tfm = tsi.transformMatrix;
        CMatrix4<f32> cmat;
        for (int i=0; i<3; i++){
            for (int j=0; j<4; j++){
                cmat[j*4+i] = tfm[i*4+j];
            }
        }
        cmat[3] = cmat[7] = cmat[11] = 0.0; cmat[15] = 1.0;
        vector3df pos = cmat.getTranslation();
        pos.Y *= -1;
        vector3df rpy = cmat.getRotationDegrees();
        rpy.X *= -1;
        rpy.Z *= -1;
        
        IMeshSceneNode *node 
            = i_mgr->addMeshSceneNode(mesh, this, -1,
                                      pos, 
                                      rpy,
                                      cmat.getScale());
        
        if (ai.textureIndex >= 0){
            const TextureInfo& ti = txs[ai.textureIndex];
            //std::cout << "url:" << ti.url << std::endl;
            video::IVideoDriver* driver = i_mgr->getVideoDriver();
            const char *path = ti.url;
            SMaterial& mat = node->getMaterial(0);
            ITexture *texture = driver->getTexture(path);
            mat.setTexture( 0, texture);
        }
        
    }
    
    const SensorInfoSequence& sensors = i_li.sensors;
    for (unsigned int i=0; i<sensors.length(); i++){
        const SensorInfo& si = sensors[i];
        std::string type(si.type);
        if (type == "Vision"){
            //std::cout << si.name << std::endl;
            ISceneNode *camera = i_mgr->addEmptySceneNode(this);
            camera->setName(si.name);
            camera->setPosition(vector3df( si.translation[0],
                                           -si.translation[1],
                                           si.translation[2]));
            Vector3 axis(si.rotation[0],
                         si.rotation[1],
                         si.rotation[2]);
            Matrix33 R;
            hrp::calcRodrigues(R, axis, si.rotation[3]);
            Vector3 rpy(rpyFromRot(R));
            camera->setRotation(vector3df(-180/M_PI*rpy[0],
                                          180/M_PI*rpy[1],
                                          -180/M_PI*rpy[2]));
            m_cameraInfos.push_back(new GLcamera(si, camera));
        }
    }
}

GLlink::GLlink(ISceneNode *i_parent, ISceneManager *i_mgr, s32 i_id,
               const hrp::Link *i_link, BodyPtr i_body) :
    ISceneNode(i_parent, i_mgr, i_id),
    m_jointId(i_link->jointId){
    setAutomaticCulling(scene::EAC_OFF);
    
    setPosition(vector3df( i_link->b[0],
                           i_link->b[1],
                           i_link->b[2]));
    setRotation(vector3df(-180/M_PI*rpyFromRot(i_link->Rs)[0],
                           180/M_PI*rpyFromRot(i_link->Rs)[1],
                          -180/M_PI*rpyFromRot(i_link->Rs)[2]));
    
    m_axis =-i_link->a[0], i_link->a[1],-i_link->a[2];
    
    switch ( i_link->jointType ) {
    case hrp::Link::FIXED_JOINT:
        m_jointType = FIXED_JOINT; break;
    case hrp::Link::FREE_JOINT:
        m_jointType = FREE_JOINT; break;
    case hrp::Link::ROTATIONAL_JOINT:
	m_jointType = ROTATIONAL_JOINT; break;
    case hrp::Link::SLIDE_JOINT:
        m_jointType = SLIDE_JOINT; break;
    }

    SMesh* mesh = new SMesh();
    SMeshBuffer* meshBuffer = new SMeshBuffer();
    mesh->addMeshBuffer(meshBuffer);
    meshBuffer->drop();

    SMeshBuffer* mb = reinterpret_cast<SMeshBuffer*>(mesh->getMeshBuffer(mesh->getMeshBufferCount()-1));
    u32 vCount = mb->getVertexCount();

    for (int i = 0; i < i_link->coldetModel->getNumTriangles(); i++ ) {
	int index[3];
	core::vector3df vertices3df[3];
	i_link->coldetModel->getTriangle(i, index[0], index[1], index[2]);
	for(int j = 0; j < 3; j++ ) {
	    float x, y, z;
	    i_link->coldetModel->getVertex(index[j], x, y, z);
	    vertices3df[j].X = x;
	    vertices3df[j].Y = y;
	    vertices3df[j].Z = z;
	}
	core::vector3df vertex1 = vertices3df[1] - vertices3df[0];
	core::vector3df vertex2 = vertices3df[2] - vertices3df[1];
	core::vector3df normal3df  = vertex1.normalize().crossProduct(vertex2.normalize()).normalize();
	//
	video::SColor color3df(0xff228822);
	vector2df texc2df;
	for (int j = 0; j < 3; j++ ) {
	    mb->Vertices.push_back(video::S3DVertex(vertices3df[j],normal3df,color3df, texc2df));
	}
	mb->Indices.push_back(vCount);
	mb->Indices.push_back(vCount+1);
	mb->Indices.push_back(vCount+2);
	vCount += 3;
    }

    mesh->getMeshBuffer(0)->recalculateBoundingBox();
    
    // Create the Animated mesh if there's anything in the mesh
    SAnimatedMesh* pAM = 0;
    if ( 0 != mesh->getMeshBufferCount() )
    {
	mesh->recalculateBoundingBox();
	pAM = new SAnimatedMesh();
	pAM->Type = EAMT_OBJ;
	pAM->addMesh(mesh);
	pAM->recalculateBoundingBox();
    }
    
    mesh->drop();

    matrix4 mat = getAbsoluteTransformation();
    vector3df pos = mat.getTranslation();
    pos.Y *= -1;
    vector3df rpy = mat.getRotationDegrees();
    rpy.X *= -1;
    rpy.Z *= -1;

    i_mgr->addMeshSceneNode(mesh, this, -1,
			    pos,
			    rpy,
			    mat.getScale());
}


void GLlink::setQ(double i_q)
{
    Matrix33 R;
    hrp::calcRodrigues(R, m_axis, i_q);
    Vector3 rpy(rpyFromRot(R));
    rpy *= 180/M_PI;
    vector3df euler(-rpy[0], rpy[1], -rpy[2]);
    setRotation(euler);
}

GLcamera *GLlink::findCamera(const char *i_name)
{
    for (unsigned int i=0; i<m_cameraInfos.size(); i++){
        if (strcmp(i_name, m_cameraInfos[i]->name())==0) return m_cameraInfos[i];
    }
    return NULL;
}

GLbody::GLbody(ISceneNode *i_parent, ISceneManager *i_mgr, s32 i_id,
                                 BodyInfo_var i_binfo) :
    ISceneNode(i_parent, i_mgr, i_id){
    setAutomaticCulling(scene::EAC_OFF);
    
    LinkInfoSequence_var lis = i_binfo->links();
    //std::cout << "creating links" << std::endl;
    for (unsigned int i=0; i<lis->length(); i++){
        GLlink *l = new GLlink(i_mgr->getRootSceneNode(), i_mgr, 
                               -1, lis[i], i_binfo);
        m_links.push_back(l);
        if (l->jointId() >= 0){
            if (l->jointId()+1 > (int)m_joints.size()) {
                m_joints.resize(l->jointId()+1);
            }
            m_joints[l->jointId()] = l;
        }
    }
    //std::cout << "creating a tree" << std::endl;
    // setup tree
    for (unsigned int i=0; i<m_links.size(); i++){
        const LinkInfo &li = lis[i];
        if (li.parentIndex < 0) {
            m_root = m_links[i];
            addChild(m_links[i]);
        }
        for (unsigned int j=0; j<li.childIndices.length(); j++){
            m_links[i]->addChild(m_links[li.childIndices[j]]);
        }
    }
    //std::cout << "done" << std::endl;
}

GLbody::GLbody(ISceneNode *i_parent, ISceneManager *i_mgr, s32 i_id,
                                 BodyPtr i_body) :
    ISceneNode(i_parent, i_mgr, i_id){
    setAutomaticCulling(scene::EAC_OFF);

    //std::cerr << i_body->numLinks() << std::endl;

    for (unsigned int i=0; i < i_body->numLinks(); i++) {
	//std::cout << "creating links index:" << i_body->link(i)->index << ", " << i_body->link(i)->jointId << std::endl;
        GLlink *l = new GLlink(i_mgr->getRootSceneNode(), i_mgr, 
                               -1, i_body->link(i), i_body);
        m_links.push_back(l);
        if (l->jointId() >= 0){
            if (l->jointId()+1 > (int)m_joints.size()) {
                m_joints.resize(l->jointId()+1);
            }
            m_joints[l->jointId()] = l;
        }
    }
    // std::cout << "creating a tree" << std::endl;
    // setup tree
    for (unsigned int i=0; i<m_links.size(); i++){
        const hrp::Link *l = i_body->link(i);
        if ( l->parent == NULL ) {
            m_root = m_links[i];
            addChild(m_links[i]);
        }
	if ( l->child != NULL )
            m_links[i]->addChild(m_links[l->child->index]);
	if ( l->sibling != NULL )
            m_links[l->parent->index]->addChild(m_links[l->sibling->index]);
    }
    //std::cout << "done" << std::endl;
}

void GLbody::setPosition(double x, double y, double z)
{
    m_root->setPosition(vector3df(x, -y, z));
}

void GLbody::setOrientation(double r, double p, double y)
{
    m_root->setRotation(vector3df(-180/M_PI*r, 180/M_PI*p, -180/M_PI*y));
}

void GLbody::setPosture(double *i_angles, double *i_pos, double *i_rpy)
{
    setPosition(i_pos[0], i_pos[1], i_pos[2]);
    setOrientation(i_rpy[0], i_rpy[1], i_rpy[2]);
    setPosture(i_angles);
}

void GLbody::setPosture(const double *i_angles){
    for (unsigned int i=0; i<m_links.size(); i++){
        int id = m_links[i]->jointId();
        if (id >= 0){
            m_links[i]->setQ(i_angles[id]);
        }
    }
}

GLcamera *GLbody::findCamera(const char *i_name)
{
    for (unsigned int i=0; i<m_links.size(); i++){
        GLcamera *ci = m_links[i]->findCamera(i_name);
        if (ci) return ci;
    }
    return NULL;
}

GLcamera::GLcamera(const OpenHRP::SensorInfo& i_info, ISceneNode *i_node) :
    m_node(i_node)
{
    m_near = i_info.specValues[0];
    m_far  = i_info.specValues[1];
    m_fovy  = i_info.specValues[2];
    m_width  = i_info.specValues[4];
    m_height = i_info.specValues[5];
}


void GLcamera::setCameraParameters(ICameraSceneNode *i_camera)
{
#if 0 // This doesn't work
    i_camera->setAspectRatio(((float)m_width)/m_height);
    i_camera->setNearValue(m_near);
    i_camera->setFarValue(m_far);
    i_camera->setFOV(m_fov);
#else
    matrix4 m;
    double t = m_near*tan(m_fovy/2), r = (t*m_width)/m_height;
    m[0] = m_near/r;
    m[1] = 0;
    m[2] = 0;
    m[3] = 0;

    m[4] = 0;
    m[5] = m_near/t;
    m[6] = 0;
    m[7] = 0;

    m[8] = 0;
    m[9] = 0;
    m[10] = (m_far+m_near)/(m_far-m_near);
    m[11] = 1;

    m[12] = 0;
    m[13] = 0;
    m[14] = -2*m_far*m_near/(m_far-m_near);
    m[15] = 0;
    i_camera->setProjectionMatrix(m);
#endif
}

void updateAbsoluteTransformation(ISceneNode *i_node)
{
    ISceneNode *parent = i_node->getParent();
    if (parent){
        updateAbsoluteTransformation(parent);
    }
    i_node->updateAbsolutePosition();
}

void GLcamera::updateCameraTransform(ICameraSceneNode *i_camera)
{
    updateAbsoluteTransformation(m_node);
    matrix4 mat = m_node->getAbsoluteTransformation();
    vector3df pos = mat.getTranslation();
    i_camera->setPosition(pos);
    vector3df view(pos.X-mat[8],pos.Y-mat[9],pos.Z-mat[10]); // -Z
    vector3df up(-mat[4], -mat[5], -mat[6]); // -Y axis
    i_camera->setTarget(view);
    i_camera->setUpVector(up);
}

const char *GLcamera::name()
{
    return m_node->getName();
}

int GLcamera::width()
{
    return m_width;
}

int GLcamera::height()
{
    return m_height;
}

void GLcamera::getAbsTransform(double *o_T)
{
    matrix4 mat = m_node->getAbsoluteTransformation();
    vector3df pos = mat.getTranslation();
    vector3df rpy = mat.getRotationDegrees();
    Matrix33 R(rotFromRpy(-rpy.X*M_PI/180, rpy.Y*M_PI/180, -rpy.Z*M_PI/180));
    o_T[ 0] = R(0,0);o_T[ 4] = R(0,1);o_T[ 8] = R(0,2);o_T[12] =  pos.X;
    o_T[ 1] = R(1,0);o_T[ 5] = R(1,1);o_T[ 9] = R(1,2);o_T[13] = -pos.Y;
    o_T[ 2] = R(2,0);o_T[ 6] = R(2,1);o_T[10] = R(2,2);o_T[14] =  pos.Z;
    o_T[ 3] = 0;     o_T[ 7] = 0;     o_T[11] = 0;    ;o_T[15] =  1.0;
}

float GLcamera::near()
{
    return m_near;
}

float GLcamera::far()
{
    return m_far;
}

float GLcamera::fovy()
{
    return m_fovy;
}

GLscene::GLscene() : m_device(NULL), m_camera(NULL), m_cnode(NULL)
{
}

void GLscene::draw()
{
    m_device->run();
    if (m_camera != m_defaultCamera) m_camera->updateCameraTransform(m_cnode);
    m_device->getVideoDriver()->beginScene(true, true, SColor(255,100,101,140));
    m_device->getSceneManager()->drawAll();
    m_device->getVideoDriver()->endScene();
    
    int fps = m_device->getVideoDriver()->getFPS();
    int prims = m_device->getVideoDriver()->getPrimitiveCountDrawn();
    wchar_t tmp[1024];
    swprintf(tmp, 1024, L"Irrlicht (fps:%d) Triangles:%d", fps, prims);
    m_device->setWindowCaption(tmp);
}

GLscene::~GLscene()
{
    if (m_defaultCamera) delete m_defaultCamera;
}

GLscene *GLscene::m_scene = NULL;

GLscene *GLscene::getInstance()
{
    if (!m_scene) m_scene = new GLscene;
    return m_scene;
}

bool GLscene::init(int w, int h)
{
    m_device =
        createDevice( video::EDT_OPENGL, dimension2d<u32>(w, h), 32,
                      false, false, false, 0);
    
    if (!m_device) return false;
    
    m_device->setWindowCaption(L"Irrlicht");

    ISceneManager* smgr = m_device->getSceneManager();
    smgr->addLightSceneNode(0, vector3df(18,-12,6), SColorf(1.0, 1.0, 1.0), 30.0f);
    smgr->addLightSceneNode(0, vector3df(-18,12,6), SColorf(1.0, 1.0, 1.0), 30.0f);
    m_cnode = smgr->addCameraSceneNode();
#if 1
    m_cnode->setTarget(vector3df(0,0,0.7));
    m_cnode->setUpVector(vector3df(0,0,1));
#endif
    m_receiver = new MyEventReceiver(m_cnode, 3);
    m_device->setEventReceiver(m_receiver);
    m_defaultCamera = new GLcamera(m_cnode);
    setCamera(m_defaultCamera);

    return true;
}

GLbody *GLscene::addBody(OpenHRP::BodyInfo_var i_binfo)
{
    ISceneManager* smgr = m_device->getSceneManager();
    return new GLbody(smgr->getRootSceneNode(), smgr, -1, i_binfo);
}

GLbody *GLscene::addBody(hrp::BodyPtr i_body)
{
    ISceneManager* smgr = m_device->getSceneManager();
    return new GLbody(smgr->getRootSceneNode(), smgr, -1, i_body);
}

void GLscene::setCamera(GLcamera *i_camera)
{
    m_camera = i_camera;
    m_camera->setCameraParameters(m_cnode);
}


GLcamera *GLscene::getCamera()
{
    return m_camera;
}

GLcamera::GLcamera(ISceneNode *i_node) : m_node(i_node), m_near(0.1), m_far(100.0), m_fovy(M_PI/4), m_width(640), m_height(480)
{
}

ISceneManager *GLscene::getSceneManager()
{
    return m_device->getSceneManager();
}

IVideoDriver *GLscene::getVideoDriver()
{
    return m_device->getVideoDriver();
}
