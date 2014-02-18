// -*- C++ -*-
/*!
 * @file  VirtualCamera.cpp
 * @brief virtual camera component
 * $Date$
 *
 * $Id$
 */

#ifndef __APPLE__
#include <GL/glu.h>
#else
#include <OpenGL/gl.h>
#endif
#include <rtm/CorbaNaming.h>
#include <hrpModel/ModelLoaderUtil.h>
#include "util/Project.h"
#include "util/VectorConvert.h"
#include "util/GLcamera.h"
#include "util/GLbody.h"
#include "util/GLlink.h"
#include "util/GLutil.h"
#include "VirtualCamera.h"
#include "RTCGLbody.h"
#include "GLscene.h"

// Module specification
// <rtc-template block="module_spec">
static const char* virtualcamera_spec[] =
{
    "implementation_id", "VirtualCamera",
    "type_name",         "VirtualCamera",
    "description",       "virtual camera component",
    "version",           HRPSYS_PACKAGE_VERSION,
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.rangerMaxAngle", "0.25",
    "conf.default.rangerMinAngle", "-0.25",
    "conf.default.rangerAngularRes", "0.01",
    "conf.default.rangerMaxRange", "5.0",
    "conf.default.rangerMinRange", "0.5",
    "conf.default.generateRange", "1",
    "conf.default.generatePointCloud", "0",
    "conf.default.generatePointCloudStep", "1",
    "conf.default.pcFormat", "xyz",
    "conf.default.generateMovie", "0",
    "conf.default.debugLevel", "0",
    "conf.default.project", "",
    "conf.default.camera", "",

    ""
};
// </rtc-template>

VirtualCamera::VirtualCamera(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      // <rtc-template block="initializer">
      m_sceneStateIn("state", m_sceneState),
      m_basePosIn("basePos", m_basePos),
      m_baseRpyIn("baseRpy", m_baseRpy),
      m_qIn("q", m_q),
      m_imageOut("image", m_image),
      m_rangeOut("range", m_range),
      m_cloudOut("cloud", m_cloud),
      m_poseSensorOut("poseSensor", m_poseSensor),
      // </rtc-template>
      m_scene(&m_log),
      m_window(&m_scene, &m_log),
      m_camera(NULL),
      m_generateRange(true),
      m_generatePointCloud(false),
      m_generateMovie(false),
      m_isGeneratingMovie(false),
      m_debugLevel(0),
      dummy(0)
{
    m_scene.showFloorGrid(false);
    m_scene.showInfo(false);
}

VirtualCamera::~VirtualCamera()
{
}



RTC::ReturnCode_t VirtualCamera::onInitialize()
{
    std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
    // <rtc-template block="bind_config">
    // Bind variables and configuration variable
    RTC::Properties& ref = getProperties();
    bindParameter("rangerMaxAngle",    m_range.config.maxAngle, "0.25");
    bindParameter("rangerMinAngle",    m_range.config.minAngle, "-0.25");
    bindParameter("rangerAngularRes",  m_range.config.angularRes, "0.01");
    bindParameter("rangerMaxRange",    m_range.config.maxRange, "5.0");
    bindParameter("rangerMinRange",    m_range.config.minRange, "0.5");
    bindParameter("generateRange",      m_generateRange, "1");
    bindParameter("generatePointCloud", m_generatePointCloud, "0");
    bindParameter("generatePointCloudStep",  m_generatePointCloudStep, "1");
    bindParameter("pcFormat", 	      m_pcFormat, ref["conf.default.pcFormat"].c_str());
    bindParameter("generateMovie",      m_generateMovie, "0");
    bindParameter("debugLevel",         m_debugLevel, "0");
    bindParameter("project", 	      m_projectName, ref["conf.default.project"].c_str());
    bindParameter("camera", 	      m_cameraName, ref["conf.default.camera"].c_str());
  
    // </rtc-template>

    // Registration: InPort/OutPort/Service
    // <rtc-template block="registration">
    // Set InPort buffers
    addInPort("state", m_sceneStateIn);
    addInPort("basePos", m_basePosIn);
    addInPort("baseRpy", m_baseRpyIn);
    addInPort("q", m_qIn);

    // Set OutPort buffer
    addOutPort("image", m_imageOut);
    addOutPort("range", m_rangeOut);
    addOutPort("cloud", m_cloudOut);
    addOutPort("poseSensor", m_poseSensorOut);
  
    // Set service provider to Ports
  
    // Set service consumers to Ports
  
    // Set CORBA Service Ports
  
    // </rtc-template>

    return RTC::RTC_OK;
}



/*
  RTC::ReturnCode_t VirtualCamera::onFinalize()
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t VirtualCamera::onStartup(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t VirtualCamera::onShutdown(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

RTC::ReturnCode_t VirtualCamera::onActivated(RTC::UniqueId ec_id)
{
    std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;

    //RTC::Properties& prop = getProperties();

    RTC::Manager& rtcManager = RTC::Manager::instance();
    RTC::CorbaNaming naming(rtcManager.getORB(), "localhost:2809");
    CORBA::Object_ptr obj = naming.resolve("ModelLoader");
    if (!CORBA::is_nil(obj)){
        std::cout << "found ModelLoader on localhost:2809" << std::endl;
    }else{
        std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
        int comPos = nameServer.find(",");
        if (comPos < 0){
            comPos = nameServer.length();
        }
        nameServer = nameServer.substr(0, comPos);
        naming.init(nameServer.c_str());
    }

    if (m_projectName == ""){
        std::cerr << "Project file is not specified." << std::endl;
        return RTC::RTC_ERROR;
    }

    Project prj;
    if (!prj.parse(m_projectName)) return RTC::RTC_ERROR;

    unsigned int pos = m_cameraName.find(':',0);
    std::string bodyName, cameraName;
    if (pos == std::string::npos){
        std::cerr << "can't find a separator in the camera name" << std::endl;
        return RTC::RTC_ERROR;
    }else{
        bodyName = m_cameraName.substr(0, pos);
        cameraName = m_cameraName.substr(pos+1);
        std::cout << "body:"  << bodyName << ", camera:" << cameraName << std::endl;
    }

    std::vector<std::pair<std::string, OpenHRP::BodyInfo_var> > binfos;
    int w=0, h=0;
    OpenHRP::ModelLoader_var ml = hrp::getModelLoader(CosNaming::NamingContext::_duplicate(naming.getRootContext()));
    for (std::map<std::string, ModelItem>::iterator it=prj.models().begin();
         it != prj.models().end(); it++){

        OpenHRP::BodyInfo_var binfo;
        OpenHRP::ModelLoader::ModelLoadOption opt;
        opt.readImage = true;
        opt.AABBdata.length(0);
        opt.AABBtype = OpenHRP::ModelLoader::AABB_NUM;
        binfo = ml->getBodyInfoEx(it->second.url.c_str(), opt);
        if (CORBA::is_nil(binfo)){
            std::cerr << "failed to load model[" << it->second.url << "]" 
                      << std::endl;
            return RTC::RTC_ERROR;
        }

        // look for camera
        if (it->first == bodyName){
            OpenHRP::LinkInfoSequence_var lis = binfo->links();
            for (unsigned int i=0; i<lis->length(); i++){
                const OpenHRP::SensorInfoSequence& sensors = lis[i].sensors;
                for (unsigned int j=0; j<sensors.length(); j++){
                    const OpenHRP::SensorInfo& si = sensors[j];
                    std::string type(si.type);
                    std::string name(si.name);
                    if (type == "Vision" && name == cameraName){
                        w = si.specValues[4];
                        h = si.specValues[5];
                        break;
                    }
                }
            }
            if (!w || !h){
                std::cout << "invalid image size:" << w << "x" << h << std::endl;
                return RTC::RTC_ERROR;
            }
        }
        binfos.push_back(std::make_pair(it->first, binfo));
    }

    m_window.init(w,h,false);
    RTCGLbody *robot=NULL;
    for (unsigned int i=0; i<binfos.size(); i++){
        GLbody *glbody = new GLbody();
        hrp::BodyPtr body(glbody);
        hrp::loadBodyFromBodyInfo(body, binfos[i].second, false, GLlinkFactory);
        loadShapeFromBodyInfo(glbody, binfos[i].second);
        body->setName(binfos[i].first);
        m_scene.WorldBase::addBody(body);
        RTCGLbody *rtcglbody = new RTCGLbody(glbody, this);
        m_bodies[binfos[i].first] = rtcglbody;
        if (binfos[i].first == bodyName){
            robot = rtcglbody;
        }
    }
    if (!robot) {
        std::cerr << "can't find a robot named " << bodyName << std::endl;
    }
    m_camera = robot->body()->findCamera(cameraName.c_str());
    if (!m_camera){
        std::cerr << "VirtualCamera: can't find camera(" 
                  << cameraName << ")" << std::endl;
        return RTC::RTC_ERROR;
    }
    m_scene.setCamera(m_camera);

    m_image.data.image.width = m_camera->width();
    m_image.data.image.height = m_camera->height();
    m_image.data.image.format = Img::CF_RGB;
    m_image.data.image.raw_data.length(m_image.data.image.width*m_image.data.image.height*3);

    return RTC::RTC_OK;
}

RTC::ReturnCode_t VirtualCamera::onDeactivated(RTC::UniqueId ec_id)
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

RTC::ReturnCode_t VirtualCamera::onExecute(RTC::UniqueId ec_id)
{
    if (m_debugLevel > 0) std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ") " << std::endl;
    coil::TimeValue t1(coil::gettimeofday());

    if (m_sceneStateIn.isNew()){
        do{
            m_sceneStateIn.read();
        }while(m_sceneStateIn.isNew());
        for (unsigned int i=0; i<m_sceneState.states.length(); i++){
            const OpenHRP::RobotState& state = m_sceneState.states[i];
            std::string name(state.name);
            RTCGLbody *rtcglb=m_bodies[name];
            if (rtcglb){
                GLbody *body = rtcglb->body();
                body->setPosition(state.basePose.position.x,
                                  state.basePose.position.y,
                                  state.basePose.position.z);
                body->setRotation(state.basePose.orientation.r,
                                  state.basePose.orientation.p,
                                  state.basePose.orientation.y);
                body->setPosture(state.q.get_buffer());
            }
        }
    }
    GLbody *body = dynamic_cast<GLbody *>(m_camera->link()->body);
    if (m_basePosIn.isNew()){
        do{
            m_basePosIn.read();
        }while(m_basePosIn.isNew());
        body->setPosition(m_basePos.data.x, m_basePos.data.y, m_basePos.data.z);
    }
    if (m_baseRpyIn.isNew()){
        do{
            m_baseRpyIn.read();
        }while(m_baseRpyIn.isNew());
        body->setRotation(m_baseRpy.data.r, m_baseRpy.data.p, m_baseRpy.data.y);
    }
    if (m_qIn.isNew()){
        do{
            m_qIn.read();
        }while(m_qIn.isNew());
        body->setPosture(m_q.data.get_buffer());
    }

    for (std::map<std::string, RTCGLbody *>::iterator it=m_bodies.begin();
         it != m_bodies.end(); it++){
        it->second->input();
    }

    coil::TimeValue t6(coil::gettimeofday());
    m_window.draw();
    m_window.swapBuffers();
    capture(m_camera->width(), m_camera->height(), m_image.data.image.raw_data.get_buffer());
    coil::TimeValue t7(coil::gettimeofday());

    double *T = m_camera->getAbsTransform();
    hrp::Vector3 p;
    p[0] = T[12];
    p[1] = T[13];
    p[2] = T[14];
    hrp::Matrix33 R;
    R(0,0) = T[0]; R(0,1) = T[4]; R(0,2) = T[8]; 
    R(1,0) = T[1]; R(1,1) = T[5]; R(1,2) = T[9]; 
    R(2,0) = T[2]; R(2,1) = T[6]; R(2,2) = T[10]; 
    hrp::Vector3 rpy = hrp::rpyFromRot(R);
    m_poseSensor.data.position.x = p[0];
    m_poseSensor.data.position.y = p[1];
    m_poseSensor.data.position.z = p[2];
    m_poseSensor.data.orientation.r = rpy[0];
    m_poseSensor.data.orientation.p = rpy[1];
    m_poseSensor.data.orientation.y = rpy[2];

    coil::TimeValue t2(coil::gettimeofday());
    if (m_generateRange) setupRangeData();
    coil::TimeValue t3(coil::gettimeofday());
    if (m_generatePointCloud) setupPointCloud();
    coil::TimeValue t4(coil::gettimeofday());
    if (m_generateMovie){
        if (!m_isGeneratingMovie){
            std::string fname(m_profile.instance_name);
            fname += ".avi";
            m_videoWriter = cvCreateVideoWriter(
                fname.c_str(),
                CV_FOURCC('D','I','V','X'),
                10,
                cvSize(m_image.data.image.width, m_image.data.image.height));
            m_cvImage = cvCreateImage(
                cvSize(m_image.data.image.width, m_image.data.image.height),
                IPL_DEPTH_8U, 3);
            m_isGeneratingMovie = true;
        }
        // RGB -> BGR
        char *dst = m_cvImage->imageData;
        for (int i=0; i<m_image.data.image.width*m_image.data.image.height; i++){
            dst[i*3  ] = m_image.data.image.raw_data[i*3+2]; 
            dst[i*3+1] = m_image.data.image.raw_data[i*3+1]; 
            dst[i*3+2] = m_image.data.image.raw_data[i*3  ]; 
        }
        cvWriteFrame(m_videoWriter, m_cvImage);
    }else{
        if (m_isGeneratingMovie){
            cvReleaseVideoWriter(&m_videoWriter);
            cvReleaseImage(&m_cvImage);
            m_isGeneratingMovie = false;
        }
    }

    m_imageOut.write();
    if (m_generateRange) m_rangeOut.write();
    if (m_generatePointCloud) m_cloudOut.write();
    m_poseSensorOut.write();

    coil::TimeValue t5(coil::gettimeofday());
    if (m_debugLevel > 0){
        coil::TimeValue dt;
        dt = t5-t1;
        std::cout << "VirtualCamera::onExecute() : total:" 
                  << dt.sec()*1e3+dt.usec()/1e3;
        dt = t7-t6;
        std::cout << ", render:"
                  << dt.sec()*1e3+dt.usec()/1e3;

        if (m_generateRange){
            dt = t3 - t2;
            std::cout << ", range2d:" << dt.sec()*1e3+dt.usec()/1e3;
        }
        if (m_generatePointCloud){
            dt = t4 - t3;
            std::cout << ", range3d:" << dt.sec()*1e3+dt.usec()/1e3;
        }
        std::cout << "[ms]" << std::endl;
    }

    return RTC::RTC_OK;
}

void VirtualCamera::setupRangeData()
{
    int w = m_camera->width();
    int h = m_camera->height();
    float depth[w];
    glReadPixels(0, h/2, w, 1, GL_DEPTH_COMPONENT, GL_FLOAT, depth);
    double far = m_camera->far();
    double near = m_camera->near();
    double fovx = 2*atan(w*tan(m_camera->fovy()/2)/h);
    RangerConfig &rc = m_range.config;
    double max = ((int)(fovx/2/rc.angularRes))*rc.angularRes;
    if (rc.maxAngle >  max) rc.maxAngle =  max;
    if (rc.minAngle < -max) rc.minAngle = -max;
    unsigned int nrange = round((rc.maxAngle - rc.minAngle)/rc.angularRes)+1;
    //std::cout << "nrange = " << nrange << std::endl;
    m_range.ranges.length(nrange);
#define THETA(x) (-atan(((x)-w/2)*2*tan(fovx/2)/w))
#define RANGE(d,th) (-far*near/((d)*(far-near)-far)/cos(th))
    double dth, alpha, th, th_old = THETA(w-1);
    double r, r_old = RANGE(depth[w-1], th_old);
    int idx = w-2;
    double angle;
    for (unsigned int i=0; i<nrange; i++){
        angle = rc.minAngle + rc.angularRes*i;
        while(idx >= 0){
            th = THETA(idx);
            r = RANGE(depth[idx], th);
            idx--;
            if (th > angle){
                //std::cout << idx << ":" << th << std::endl;
                dth = th - th_old;
                alpha = atan2(r*sin(dth), fabs(r_old - r*cos(dth)));
                //std::cout << "alpha:" << alpha << std::endl;
                if (r != 0 && r_old != 0 && alpha > 0.01){
                    m_range.ranges[i] = ((th - angle)*r_old + (angle - th_old)*r)/dth;
                }else if (th - angle < dth/2){
                    m_range.ranges[i] = r;
                }else{
                    m_range.ranges[i] = r_old;
                }
                if (m_range.ranges[i] > rc.maxRange 
                    || m_range.ranges[i] < rc.minRange){
                    m_range.ranges[i] = rc.maxRange;
                }
                th_old = th;
                r_old = r;
                break;
            }
            th_old = th;
            r_old = r;
        }
#if 0
        std::cout << angle << " " << depth[idx] << " " << m_range.ranges[i] << " " 
                  << m_range.ranges[i]*cos(angle) << " " << m_range.ranges[i]*sin(angle) << std::endl;
#endif
    }
}

void VirtualCamera::setupPointCloud()
{
    int w = m_camera->width();
    int h = m_camera->height();
    float depth[w*h];
    m_cloud.width = w;
    m_cloud.height = h;
    m_cloud.type = m_pcFormat.c_str();
    bool colored = false;
    if (m_pcFormat == "xyz"){
        m_cloud.fields.length(3);
    }else if (m_pcFormat == "xyzrgb"){
        m_cloud.fields.length(6);
        colored = true;
    }else{
        std::cerr << "unknown point cloud format:[" << m_pcFormat << "]" << std::endl;
    }
    m_cloud.fields[0].name = "x";
    m_cloud.fields[0].offset = 0;
    m_cloud.fields[0].data_type = PointCloudTypes::FLOAT32;
    m_cloud.fields[0].count = 4;
    m_cloud.fields[1].name = "y";
    m_cloud.fields[1].offset = 4;
    m_cloud.fields[1].data_type = PointCloudTypes::FLOAT32;
    m_cloud.fields[1].count = 4;
    m_cloud.fields[2].name = "z";
    m_cloud.fields[2].offset = 8;
    m_cloud.fields[2].data_type = PointCloudTypes::FLOAT32;
    m_cloud.fields[2].count = 4;
    if (m_pcFormat == "xyzrgb"){
        m_cloud.fields[3].name = "r";
        m_cloud.fields[3].offset = 12;
        m_cloud.fields[3].data_type = PointCloudTypes::UINT8;
        m_cloud.fields[3].count = 1;
        m_cloud.fields[4].name = "g";
        m_cloud.fields[4].offset = 13;
        m_cloud.fields[4].data_type = PointCloudTypes::UINT8;
        m_cloud.fields[4].count = 1;
        m_cloud.fields[5].name = "b";
        m_cloud.fields[5].offset = 14;
        m_cloud.fields[5].data_type = PointCloudTypes::UINT8;
        m_cloud.fields[5].count = 1;
    }
    m_cloud.is_bigendian = false;
    m_cloud.point_step = 16;
    m_cloud.data.length(w*h*m_cloud.point_step);// will be shrinked later
    m_cloud.row_step = m_cloud.point_step*w;
    m_cloud.is_dense = true;
    double far = m_camera->far();
    double near = m_camera->near();
    double fovx = 2*atan(w*tan(m_camera->fovy()/2)/h);
    double zs = w/(2*tan(fovx/2));
    unsigned int npoints=0;
    float *ptr = (float *)m_cloud.data.get_buffer();
    unsigned char *rgb = m_image.data.image.raw_data.get_buffer();
    glReadPixels(0,0, w, h, GL_DEPTH_COMPONENT, GL_FLOAT, depth);
    for (int i=0; i<h; i+=m_generatePointCloudStep){
        for (int j=0; j<w; j+=m_generatePointCloudStep){
            float d = depth[i*w+j];
            if (d == 1.0) {
                continue;
            }
            ptr[2] = far*near/(d*(far-near)-far);
            ptr[0] = -(j-w/2)*ptr[2]/zs;
            ptr[1] = -(i-h/2)*ptr[2]/zs;
            if (colored){
                unsigned char *c = (unsigned char *)(ptr + 3);
                int offset = ((h-1-i)*w+j)*3;
                c[0] = rgb[offset];
                c[1] = rgb[offset+1];
                c[2] = rgb[offset+2];
            }
            ptr += 4;
            npoints++;
        }
    }
    m_cloud.data.length(npoints*m_cloud.point_step);
}
/*
  RTC::ReturnCode_t VirtualCamera::onAborting(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t VirtualCamera::onError(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t VirtualCamera::onReset(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t VirtualCamera::onStateUpdate(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t VirtualCamera::onRateChanged(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/



extern "C"
{

    void VirtualCameraInit(RTC::Manager* manager)
    {
        RTC::Properties profile(virtualcamera_spec);
        manager->registerFactory(profile,
                                 RTC::Create<VirtualCamera>,
                                 RTC::Delete<VirtualCamera>);
    }

};


