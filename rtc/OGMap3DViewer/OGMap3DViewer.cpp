// -*- C++ -*-
/*!
 * @file  OGMap3DViewer.cpp
 * @brief null component
 * $Date$
 *
 * $Id$
 */

#include <rtm/CorbaNaming.h>
#include <GL/gl.h>
#include "IrrModel.h"
#include <math.h>
#include <hrpModel/ModelLoaderUtil.h>
#include "OGMap3DViewer.h"

using namespace irr;
using namespace core;
using namespace scene;
using namespace video;

class CMapSceneNode : public irr::scene::ISceneNode
{
public:
    CMapSceneNode(ISceneNode *i_parent, ISceneManager *i_mgr, s32 i_id,
                  double i_origin[3], double i_size[3]) :
        ISceneNode(i_parent, i_mgr, i_id),
        m_map(NULL){
        // bounding box
        m_vertices[0] = vector3df(i_origin[0], -i_origin[1], i_origin[2]);
        m_vertices[1] = vector3df(i_origin[0]+i_size[0], -i_origin[1], i_origin[2]);
        m_vertices[2] = vector3df(i_origin[0]+i_size[0], -(i_origin[1]+i_size[1]), i_origin[2]);
        m_vertices[3] = vector3df(i_origin[0], -(i_origin[1]+i_size[1]), i_origin[2]);

        m_vertices[4] = m_vertices[0]; m_vertices[4].Z += i_size[2];
        m_vertices[5] = m_vertices[1]; m_vertices[5].Z += i_size[2];
        m_vertices[6] = m_vertices[2]; m_vertices[6].Z += i_size[2];
        m_vertices[7] = m_vertices[3]; m_vertices[7].Z += i_size[2];

        m_box.reset(m_vertices[0]);
        for (int i=1; i<8; i++) m_box.addInternalPoint(m_vertices[i]);

        // vertex indices of cube
        m_cubeIndices[ 0] = 0; m_cubeIndices[ 1] = 1; m_cubeIndices[ 2] = 2;
        m_cubeIndices[ 3] = 2; m_cubeIndices[ 4] = 3; m_cubeIndices[ 5] = 0;

        m_cubeIndices[ 6] = 4; m_cubeIndices[ 7] = 5; m_cubeIndices[ 8] = 6;
        m_cubeIndices[ 9] = 6; m_cubeIndices[10] = 7; m_cubeIndices[11] = 4;

        m_cubeIndices[12] = 8; m_cubeIndices[13] = 9; m_cubeIndices[14] = 10;
        m_cubeIndices[15] = 10; m_cubeIndices[16] = 11; m_cubeIndices[17] = 8;

        m_cubeIndices[18] = 12; m_cubeIndices[19] = 13; m_cubeIndices[20] = 14;
        m_cubeIndices[21] = 14; m_cubeIndices[22] = 15; m_cubeIndices[23] = 12;

        m_cubeIndices[24] = 16; m_cubeIndices[25] = 17; m_cubeIndices[26] = 18;
        m_cubeIndices[27] = 18; m_cubeIndices[28] = 19; m_cubeIndices[29] = 16;

        m_cubeIndices[30] = 20; m_cubeIndices[31] = 21; m_cubeIndices[32] = 22;
        m_cubeIndices[33] = 22; m_cubeIndices[34] = 23; m_cubeIndices[35] = 20;
    }

    void setMap(OpenHRP::OGMap3D *i_map) { m_map = i_map; }

    void setupCubeVertices(double i_res){
        SColor white(0xff, 0xff, 0xff, 0xff);
        // +z
        m_cubeVerts[0] = S3DVertex(-i_res/2, -i_res/2, i_res/2,
                                   0,0,1, white,0,0);
        m_cubeVerts[1] = S3DVertex( i_res/2, -i_res/2, i_res/2,
                                   0,0,1, white,0,0);
        m_cubeVerts[2] = S3DVertex( i_res/2,  i_res/2, i_res/2,
                                   0,0,1, white,0,0);
        m_cubeVerts[3] = S3DVertex(-i_res/2,  i_res/2, i_res/2,
                                   0,0,1, white,0,0);
        // -z
        m_cubeVerts[4] = S3DVertex(-i_res/2,  i_res/2, -i_res/2,
                                   0,0,-1, white,0,0);
        m_cubeVerts[5] = S3DVertex( i_res/2,  i_res/2, -i_res/2,
                                   0,0,-1, white,0,0);
        m_cubeVerts[6] = S3DVertex( i_res/2, -i_res/2, -i_res/2,
                                   0,0,-1, white,0,0);
        m_cubeVerts[7] = S3DVertex(-i_res/2, -i_res/2, -i_res/2,
                                   0,0,-1, white,0,0);
        // +x
        m_cubeVerts[8] = S3DVertex(i_res/2, -i_res/2,  i_res/2,
                                   1,0,0, white,0,0);
        m_cubeVerts[9] = S3DVertex(i_res/2, -i_res/2, -i_res/2,
                                   1,0,0, white,0,0);
        m_cubeVerts[10] = S3DVertex(i_res/2,  i_res/2, -i_res/2,
                                   1,0,0, white,0,0);
        m_cubeVerts[11] = S3DVertex(i_res/2,  i_res/2,  i_res/2,
                                   1,0,0, white,0,0);
        // -x
        m_cubeVerts[12] = S3DVertex(-i_res/2,  i_res/2,  i_res/2,
                                   -1,0,0, white,0,0);
        m_cubeVerts[13] = S3DVertex(-i_res/2,  i_res/2, -i_res/2,
                                   -1,0,0, white,0,0);
        m_cubeVerts[14] = S3DVertex(-i_res/2, -i_res/2, -i_res/2,
                                   -1,0,0, white,0,0);
        m_cubeVerts[15] = S3DVertex(-i_res/2, -i_res/2,  i_res/2,
                                    -1,0,0, white,0,0);
        // +y
        m_cubeVerts[16] = S3DVertex(i_res/2, i_res/2,  i_res/2,
                                   0,1,0, white,0,0);
        m_cubeVerts[17] = S3DVertex(i_res/2, i_res/2, -i_res/2,
                                   0,1,0, white,0,0);
        m_cubeVerts[18] = S3DVertex(-i_res/2,i_res/2, -i_res/2,
                                   0,1,0, white,0,0);
        m_cubeVerts[19] = S3DVertex(-i_res/2,i_res/2,  i_res/2,
                                   0,1,0, white,0,0);
        // -y
        m_cubeVerts[20] = S3DVertex(-i_res/2,-i_res/2,  i_res/2,
                                   0,-1,0, white,0,0);
        m_cubeVerts[21] = S3DVertex(-i_res/2,-i_res/2, -i_res/2,
                                   0,-1,0, white,0,0);
        m_cubeVerts[22] = S3DVertex(i_res/2, -i_res/2, -i_res/2,
                                   0,-1,0, white,0,0);
        m_cubeVerts[23] = S3DVertex(i_res/2, -i_res/2,  i_res/2,
                                   0,-1,0, white,0,0);

    }
    virtual void OnRegisterSceneNode(){
        if (IsVisible)
            SceneManager->registerNodeForRendering(this);
        
        ISceneNode::OnRegisterSceneNode();
    }
    virtual void render() {
        IVideoDriver *driver = SceneManager->getVideoDriver();
        matrix4 m;
        driver->setTransform(ETS_WORLD, m);

        // bottom 
        driver->draw3DLine(m_vertices[0], m_vertices[1]);
        driver->draw3DLine(m_vertices[1], m_vertices[2]);
        driver->draw3DLine(m_vertices[2], m_vertices[3]);
        driver->draw3DLine(m_vertices[3], m_vertices[0]);
        // top 
        driver->draw3DLine(m_vertices[4], m_vertices[5]);
        driver->draw3DLine(m_vertices[5], m_vertices[6]);
        driver->draw3DLine(m_vertices[6], m_vertices[7]);
        driver->draw3DLine(m_vertices[7], m_vertices[4]);
        // vertical lines
        driver->draw3DLine(m_vertices[0], m_vertices[4]);
        driver->draw3DLine(m_vertices[1], m_vertices[5]);
        driver->draw3DLine(m_vertices[2], m_vertices[6]);
        driver->draw3DLine(m_vertices[3], m_vertices[7]);

        if (!m_map) return;
        setupCubeVertices(m_map->resolution);
        double res = m_map->resolution;
        int rank=0;
	int nxny = m_map->nx*m_map->ny;

        for (int i=0; i<m_map->nx; i++){
            m[12] = m_map->pos.x + i*res;
            for (int j=0; j<m_map->ny; j++){
                m[13] = -(m_map->pos.y + j*res);
                for (int k=0; k<m_map->nz; k++){
                    m[14] = m_map->pos.z + k*res;
                    unsigned char p = m_map->cells[rank++]; 
                    if (p != OpenHRP::gridUnknown 
                        && p != OpenHRP::gridEmpty){
                        driver->setTransform(ETS_WORLD, m);
                        driver->drawIndexedTriangleList(m_cubeVerts, 24,
                                                        m_cubeIndices, 12);
                    }
                }

            }

        }
    }
    virtual const aabbox3d<f32>& getBoundingBox() const { return m_box; }
private:
    irr::core::aabbox3d<f32> m_box;
    vector3df m_vertices[8];
    S3DVertex m_tileVerts[4], m_cubeVerts[24];
    u16 m_tileIndices[4], m_cubeIndices[36];
    float m_scale[3], m_origin[3];
    OpenHRP::OGMap3D* m_map;
};

// Module specification
// <rtc-template block="module_spec">
static const char* nullcomponent_spec[] =
  {
    "implementation_id", "OGMap3DViewer",
    "type_name",         "OGMap3DViewer",
    "description",       "null component",
    "version",           "1.0",
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.generateImageSequence", "0",
    "conf.default.generateMovie", "0",
    "conf.default.xSize", "4",
    "conf.default.ySize", "4",
    "conf.default.zSize", "4",
    "conf.default.xOrigin", "0",
    "conf.default.yOrigin", "-2",
    "conf.default.zOrigin", "0",

    ""
  };
// </rtc-template>

OGMap3DViewer::OGMap3DViewer(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_qIn("q", m_q),
    m_pIn("p", m_p),
    m_rpyIn("rpy", m_rpy),
    m_OGMap3DServicePort("OGMap3DService"),
    // </rtc-template>
    dummy(0),
    m_isopen(false),
    m_generateImageSequence(false),
    m_body(NULL),
    m_imageCount(0),
    m_ogmap(NULL),
    m_generateMovie(false),
    m_isGeneratingMovie(false)
{
}

OGMap3DViewer::~OGMap3DViewer()
{
}



RTC::ReturnCode_t OGMap3DViewer::onInitialize()
{
  std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("generateImageSequence", m_generateImageSequence, "0");
  bindParameter("generateMovie",      m_generateMovie, "0");
  bindParameter("xSize",      m_xSize, "4");
  bindParameter("ySize",      m_ySize, "4");
  bindParameter("zSize",      m_zSize, "4");
  bindParameter("xOrigin",      m_xOrigin, "0");
  bindParameter("yOrigin",      m_yOrigin, "-2");
  bindParameter("zOrigin",      m_zOrigin, "0");
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("q", m_qIn);
  addInPort("p", m_pIn);
  addInPort("rpy", m_rpyIn);

  // Set OutPort buffer
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  m_OGMap3DServicePort.registerConsumer("service1","OGMap3DService",m_OGMap3DService);
  
  // Set CORBA Service Ports
  addPort(m_OGMap3DServicePort);
  
  // </rtc-template>

  RTC::Properties& prop = getProperties();

  return RTC::RTC_OK;
}



/*
RTC::ReturnCode_t OGMap3DViewer::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t OGMap3DViewer::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t OGMap3DViewer::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t OGMap3DViewer::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t OGMap3DViewer::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

void capture(int w, int h, unsigned char *o_buffer)
{
    glReadBuffer(GL_BACK);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    for (int i=0; i<h; i++){
        glReadPixels(0,(h-1-i),w,1,GL_RGB,GL_UNSIGNED_BYTE,
                     o_buffer + i*3*w);
    }
}

void save(int w, int h, const char *i_fname)
{
    unsigned char *buffer = new unsigned char[w*h*3];

    capture(w, h, buffer);
    std::ofstream ofs("test.ppm", std::ios::out | std::ios::trunc | std::ios::binary );
    char buf[10];
    sprintf(buf, "%d %d", w, h);
    ofs << "P6" << std::endl << buf << std::endl << "255" << std::endl;
    for (int i=h-1; i>=0; i--){
        ofs.write((char *)(buffer+i*w*3), w*3);
    }
    delete [] buffer;
}


RTC::ReturnCode_t OGMap3DViewer::onExecute(RTC::UniqueId ec_id)
{
    if (m_qIn.isNew()) m_qIn.read();
    if (m_pIn.isNew()) m_pIn.read();
    if (m_rpyIn.isNew()) m_rpyIn.read();

    if (!m_isopen){
        GLscene *scene = GLscene::getInstance();

        scene->init();

        RTC::Properties& prop = getProperties();
        RTC::Manager& rtcManager = RTC::Manager::instance();
        std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
        int comPos = nameServer.find(",");
        if (comPos < 0){
            comPos = nameServer.length();
        }
        nameServer = nameServer.substr(0, comPos);
        RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
        if (prop["model"] != ""){
            std::cerr << "model = " << prop["model"] << std::endl;
            OpenHRP::BodyInfo_var binfo = hrp::loadBodyInfo(
                prop["model"].c_str(), 
                CosNaming::NamingContext::_duplicate(naming.getRootContext()));
            m_body = scene->addBody(binfo);
        }

        double origin[] = {m_xOrigin, m_yOrigin, m_zOrigin};
        double size[] = {m_xSize, m_ySize, m_zSize};
        ISceneManager *smgr = scene->getSceneManager();
        m_mapNode = new CMapSceneNode(smgr->getRootSceneNode(), smgr, -1, origin, size);
        
        m_isopen = true;
    }

    OpenHRP::AABB region;
    region.pos.x = m_xOrigin;
    region.pos.y = m_yOrigin;
    region.pos.z = m_zOrigin;
    region.size.l = m_xSize;
    region.size.w = m_ySize;
    region.size.h = m_zSize;

    if (m_ogmap){
        delete m_ogmap;
        m_ogmap = NULL;
    }
    if (!CORBA::is_nil(m_OGMap3DService.getObject())){
        try{
            m_ogmap = m_OGMap3DService->getOGMap3D(region);
        }catch(CORBA::SystemException& ex){
            // provider is not activated
        }
    }
    m_mapNode->setMap(m_ogmap);
    
    GLscene *scene = GLscene::getInstance();
    GLcamera *camera=scene->getCamera();

    if (m_body && m_q.data.length() > 0){
        double pos[] = {m_p.data.x, m_p.data.y, m_p.data.z}; 
        double rpy[] = {m_rpy.data.r, m_rpy.data.p, m_rpy.data.y}; 
        m_body->setPosture(m_q.data.get_buffer(), pos, rpy);
    }

    scene->draw();

    if (m_generateImageSequence){
        char buf[30];
        sprintf(buf, "OGMap3DViewer%03d.ppm", m_imageCount++);
        save(camera->width(), camera->height(), buf);
    }

    if (m_generateMovie){
        if (!m_isGeneratingMovie){
          std::string fname(m_profile.instance_name);
          fname += ".avi";
            m_videoWriter = cvCreateVideoWriter(
                fname.c_str(),
                CV_FOURCC('D','I','V','X'),
                10,
                cvSize(camera->width(), camera->height()));
            m_cvImage = cvCreateImage(
                cvSize(camera->width(), camera->height()),
                IPL_DEPTH_8U, 3);
            m_isGeneratingMovie = true;
        }
        // RGB -> BGR
        unsigned char rgb[camera->width()*camera->height()*3];
        capture(camera->width(), camera->height(), rgb);
        char *bgr = m_cvImage->imageData;
        for (int i=0; i<camera->width()*camera->height(); i++){
            bgr[i*3  ] = rgb[i*3+2]; 
            bgr[i*3+1] = rgb[i*3+1]; 
            bgr[i*3+2] = rgb[i*3  ]; 
        }
        cvWriteFrame(m_videoWriter, m_cvImage);
    }else{
        if (m_isGeneratingMovie){
            cvReleaseVideoWriter(&m_videoWriter);
            cvReleaseImage(&m_cvImage);
            m_isGeneratingMovie = false;
        }
    }

    

    return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t OGMap3DViewer::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t OGMap3DViewer::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t OGMap3DViewer::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t OGMap3DViewer::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t OGMap3DViewer::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void OGMap3DViewerInit(RTC::Manager* manager)
  {
    RTC::Properties profile(nullcomponent_spec);
    manager->registerFactory(profile,
                             RTC::Create<OGMap3DViewer>,
                             RTC::Delete<OGMap3DViewer>);
  }

};


