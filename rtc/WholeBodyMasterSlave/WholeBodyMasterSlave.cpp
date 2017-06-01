#include <rtm/CorbaNaming.h>
#include <hrpModel/Link.h>
#include <hrpModel/Sensor.h>
#include <hrpModel/ModelLoaderUtil.h>
#include "WholeBodyMasterSlave.h"
#include <hrpModel/JointPath.h>
#include <hrpUtil/MatrixSolvers.h>
#include "hrpsys/util/Hrpsys.h"

#define DEBUGP (loop%200==0)
#define DEBUGP_ONCE (loop==0)

static const char* WholeBodyMasterSlave_spec[] =
    {
        "implementation_id", "WholeBodyMasterSlave",
        "type_name",         "WholeBodyMasterSlave",
        "description",       "wholebodymasterslave component",
        "version",           HRPSYS_PACKAGE_VERSION,
        "vendor",            "AIST",
        "category",          "example",
        "activity_type",     "DataFlowComponent",
        "max_instance",      "10",
        "language",          "C++",
        "lang_type",         "compile",
        "conf.default.debugLevel", "0",
        ""
    };

WholeBodyMasterSlave::WholeBodyMasterSlave(RTC::Manager* manager) : RTC::DataFlowComponentBase(manager),
      m_qRefIn("qRef", m_qRef),// from sh
      m_zmpIn("zmpIn", m_zmp),
      m_basePosIn("basePosIn", m_basePos),
      m_baseRpyIn("baseRpyIn", m_baseRpy),
      m_optionalDataIn("optionalData", m_optionalData),
      m_qOut("q", m_qRef),// to abc
      m_zmpOut("zmpOut", m_zmp),
      m_basePosOut("basePosOut", m_basePos),
      m_baseRpyOut("baseRpyOut", m_baseRpy),
      m_optionalDataOut("optionalDataOut", m_optionalData),
      m_htzmpIn("htzmpIn", m_htzmp),// from ros bridge
      m_htrfwIn("htrfwIn", m_htrfw),
      m_htlfwIn("htlfwIn", m_htlfw),
      m_htcomIn("htcomIn", m_htcom),
      m_htrfIn("htrfIn", m_htrf),
      m_htlfIn("htlfIn", m_htlf),
      m_htrhIn("htrhIn", m_htrh),
      m_htlhIn("htlhIn", m_htlh),
//      m_actzmpIn("actzmpIn", m_actzmp),
      m_htheadIn("htheadIn", m_hthead),
#ifdef USE_DEBUG_PORT
       m_htcom_dbgOut("htcom_dbgOut", m_htcom_dbg),// to ros bridge
       m_htrf_dbgOut("htrf_dbgOut", m_htrf_dbg),
       m_htlf_dbgOut("htlf_dbgOut", m_htlf_dbg),
       m_htrh_dbgOut("htrh_dbgOut", m_htrh_dbg),
       m_htlh_dbgOut("htlh_dbgOut", m_htlh_dbg),
       m_hthead_dbgOut("hthead_dbgOut", m_hthead_dbg),
       m_htzmp_dbgOut("htzmp_dbgOut", m_htzmp_dbg),
       m_htrfw_dbgOut("htrfw_dbgOut", m_htrfw_dbg),
       m_htlfw_dbgOut("htlfw_dbgOut", m_htlfw_dbg),
       m_rpcom_dbgOut("rpcom_dbgOut", m_rpcom_dbg),
       m_rprf_dbgOut("rprf_dbgOut", m_rprf_dbg),
       m_rplf_dbgOut("rplf_dbgOut", m_rplf_dbg),
       m_rprh_dbgOut("rprh_dbgOut", m_rprh_dbg),
       m_rplh_dbgOut("rplh_dbgOut", m_rplh_dbg),
       m_rphead_dbgOut("rphead_dbgOut", m_rphead_dbg),
       m_rpzmp_dbgOut("rpzmp_dbgOut", m_rpzmp_dbg),
       m_rpdcp_dbgOut("rpdcp_dbgOut", m_rpdcp_dbg),
       m_rpacp_dbgOut("rpacp_dbgOut", m_rpacp_dbg),
       m_invdyn_dbgOut("invdyn_dbgOut", m_invdyn_dbg),
#endif
      m_WholeBodyMasterSlaveServicePort("WholeBodyMasterSlaveService"),
      m_robot(hrp::BodyPtr()),
      m_debugLevel(0)
{
    m_service0.wholebodymasterslave(this);
}

WholeBodyMasterSlave::~WholeBodyMasterSlave(){}

RTC::ReturnCode_t WholeBodyMasterSlave::onInitialize(){
    std::cerr << "[" << m_profile.instance_name << "] onInitialize()" << std::endl;
    bindParameter("debugLevel", m_debugLevel, "0");
    addInPort("qRef", m_qRefIn);// from sh
    addInPort("zmpIn", m_zmpIn);
    addInPort("basePosIn", m_basePosIn);
    addInPort("baseRpyIn", m_baseRpyIn);
    addInPort("optionalData", m_optionalDataIn);
    addOutPort("q", m_qOut);// to abc
    addOutPort("zmpOut", m_zmpOut);
    addOutPort("basePosOut", m_basePosOut);
    addOutPort("baseRpyOut", m_baseRpyOut);
    addOutPort("optionalDataOut", m_optionalDataOut);
    addInPort("htzmpIn", m_htzmpIn);// from ros bridge
    addInPort("htrfwIn", m_htrfwIn);
    addInPort("htlfwIn", m_htlfwIn);
    addInPort("htcomIn", m_htcomIn);
    addInPort("htrfIn", m_htrfIn);
    addInPort("htlfIn", m_htlfIn);
    addInPort("htrhIn", m_htrhIn);
    addInPort("htlhIn", m_htlhIn);
//    addInPort("actzmpIn", m_actzmpIn);
    addInPort("htheadIn", m_htheadIn);
#ifdef USE_DEBUG_PORT
     addOutPort("htcom_dbgOut", m_htcom_dbgOut);// to ros bridge
     addOutPort("htrf_dbgOut", m_htrf_dbgOut);
     addOutPort("htlf_dbgOut", m_htlf_dbgOut);
     addOutPort("htrh_dbgOut", m_htrh_dbgOut);
     addOutPort("htlh_dbgOut", m_htlh_dbgOut);
     addOutPort("hthead_dbgOut", m_hthead_dbgOut);
     addOutPort("htzmp_dbgOut", m_htzmp_dbgOut);
     addOutPort("htrfw_dbgOut", m_htrfw_dbgOut);
     addOutPort("htlfw_dbgOut", m_htlfw_dbgOut);
     addOutPort("rpcom_dbgOut", m_rpcom_dbgOut);
     addOutPort("rprf_dbgOut", m_rprf_dbgOut);
     addOutPort("rplf_dbgOut", m_rplf_dbgOut);
     addOutPort("rprh_dbgOut", m_rprh_dbgOut);
     addOutPort("rplh_dbgOut", m_rplh_dbgOut);
     addOutPort("rphead_dbgOut", m_rphead_dbgOut);
     addOutPort("rpzmp_dbgOut", m_rpzmp_dbgOut);
     addOutPort("rpdcp_dbgOut", m_rpdcp_dbgOut);
     addOutPort("rpacp_dbgOut", m_rpacp_dbgOut);
     addOutPort("invdyn_dbgOut", m_invdyn_dbgOut);
#endif
    m_WholeBodyMasterSlaveServicePort.registerProvider("service0", "WholeBodyMasterSlaveService", m_service0);
    addPort(m_WholeBodyMasterSlaveServicePort);
  
    RTC::Properties& prop =  getProperties();
    coil::stringTo(m_dt, prop["dt"].c_str());

    m_robot = hrp::BodyPtr(new hrp::Body());
    m_robot_rmc = hrp::BodyPtr(new hrp::Body());
    m_robot_ml = hrp::BodyPtr(new hrp::Body());
    RTC::Manager& rtcManager = RTC::Manager::instance();
    std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
    int comPos = nameServer.find(",");
    if (comPos < 0){ comPos = nameServer.length(); }
    nameServer = nameServer.substr(0, comPos);
    RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
    if (!loadBodyFromModelLoader(m_robot, prop["model"].c_str(), CosNaming::NamingContext::_duplicate(naming.getRootContext()) )){
      std::cerr << "[" << m_profile.instance_name << "] failed to load model[" << prop["model"] << "]" << std::endl;
      return RTC::RTC_ERROR;
    }
    if (!loadBodyFromModelLoader(m_robot_rmc, prop["model"].c_str(), CosNaming::NamingContext::_duplicate(naming.getRootContext()) )){
      std::cerr << "[" << m_profile.instance_name << "] failed to load model 2 [" << prop["model"] << "]" << std::endl;
      return RTC::RTC_ERROR;
    }
    if (!loadBodyFromModelLoader(m_robot_ml, prop["model"].c_str(), CosNaming::NamingContext::_duplicate(naming.getRootContext()) )){
      std::cerr << "[" << m_profile.instance_name << "] failed to load model 2 [" << prop["model"] << "]" << std::endl;
      return RTC::RTC_ERROR;
    }
    // allocate memory for outPorts
    m_qRef.data.length(m_robot->numJoints());
    m_htrfw.data.length(6);
    m_htlfw.data.length(6);
    coil::stringTo(optionalDataLength, prop["seq_optional_data_dim"].c_str());
    loop = 0;
    transition_interpolator = new interpolator(1, m_dt, interpolator::HOFFARBIB, 1);
    transition_interpolator->setName(std::string(m_profile.instance_name)+" transition_interpolator");
    transition_interpolator_ratio = 0;
    double tmp_ratio = 0.0;
    transition_interpolator->clear();
    transition_interpolator->set(&tmp_ratio);
    tmp_ratio = 1.0;
    transition_interpolator->setGoal(&tmp_ratio, 3.0, true);
    // Generate FIK
    fik = fikPtr(new SimpleFullbodyInverseKinematicsSolver(m_robot, std::string(m_profile.instance_name), m_dt));
    fik_rmc = fikPtr(new SimpleFullbodyInverseKinematicsSolver(m_robot_rmc, std::string(m_profile.instance_name), m_dt));
    fik_ml = fikPtr(new SimpleFullbodyInverseKinematicsSolver(m_robot_ml, std::string(m_profile.instance_name), m_dt));
    fik_list.push_back(fik);
    fik_list.push_back(fik_rmc);
    fik_list.push_back(fik_ml);
    body_list.push_back(m_robot);
    body_list.push_back(m_robot_rmc);
    body_list.push_back(m_robot_ml);
    for(int i=0;i<fik_list.size();i++){ setupfik(fik_list[i], body_list[i], prop); }

    if (fik->ikp.find("rleg") != fik->ikp.end() && fik->ikp.find("lleg") != fik->ikp.end()) {
      is_legged_robot = true;
    } else {
      is_legged_robot = false;
    }
    hsp = boost::shared_ptr<WBMSCore>(new WBMSCore(m_dt));

    invdyn_zmp_filters.setParameter(25, 1/m_dt, BiquadIIRFilterVec::Q_BUTTERWORTH);
    invdyn_zmp_filters2.setParameter(25, 1/m_dt, BiquadIIRFilterVec::Q_BUTTERWORTH);

    mode = previous_mode = MODE_IDLE;

    return RTC::RTC_OK;
}


void WholeBodyMasterSlave::setupfik(fikPtr& fik_in, hrp::BodyPtr& robot_in, RTC::Properties& prop_in){
  coil::vstring end_effectors_str = coil::split(prop_in["end_effectors"], ",");
  size_t prop_num = 10;
  if (end_effectors_str.size() > 0) {
    size_t num = end_effectors_str.size()/prop_num;
    for (size_t i = 0; i < num; i++) {
      std::string ee_name, ee_target, ee_base;
      coil::stringTo(ee_name, end_effectors_str[i*prop_num].c_str());
      coil::stringTo(ee_target, end_effectors_str[i*prop_num+1].c_str());
      coil::stringTo(ee_base, end_effectors_str[i*prop_num+2].c_str());
      ABCIKparam tp;
      for (size_t j = 0; j < XYZ; j++) {
          coil::stringTo(tp.localPos(j), end_effectors_str[i*prop_num+3+j].c_str());
      }
      double tmpv[4];
      for (int j = 0; j < 4; j++ ) {
        coil::stringTo(tmpv[j], end_effectors_str[i*prop_num+6+j].c_str());
      }
      tp.localR = Eigen::AngleAxis<double>(tmpv[3], hrp::Vector3(tmpv[0], tmpv[1], tmpv[2])).toRotationMatrix(); // rotation in VRML is represented by axis + angle
      // FIK param
      SimpleFullbodyInverseKinematicsSolver::IKparam tmp_fikp;
      tmp_fikp.manip = hrp::JointPathExPtr(new hrp::JointPathEx(robot_in, robot_in->link(ee_base), robot_in->link(ee_target), m_dt, false, std::string(m_profile.instance_name)));
      tmp_fikp.target_link = robot_in->link(ee_target);
      tmp_fikp.localPos = tp.localPos;
      tmp_fikp.localR = tp.localR;
      fik_in->ikp.insert(std::pair<std::string, SimpleFullbodyInverseKinematicsSolver::IKparam>(ee_name, tmp_fikp));
      // Fix for toe joint
      //   Toe joint is defined as end-link joint in the case that end-effector link != force-sensor link
      //   Without toe joints, "end-effector link == force-sensor link" is assumed.
      //   With toe joints, "end-effector link != force-sensor link" is assumed.
      if (robot_in->link(ee_target)->sensors.size() == 0) { // If end-effector link has no force sensor
          std::vector<double> optw(fik_in->ikp[ee_name].manip->numJoints(), 1.0);
          optw.back() = 0.0; // Set weight = 0 for toe joint by default
          fik_in->ikp[ee_name].manip->setOptionalWeightVector(optw);
          tp.has_toe_joint = true;
      } else {
          tp.has_toe_joint = false;
      }
      tp.target_link = robot_in->link(ee_target);
      std::cerr << "[" << m_profile.instance_name << "] End Effector [" << ee_name << "]" << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]   target = " << fik_in->ikp[ee_name].target_link->name << ", base = " << ee_base << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]   offset_pos = " << tp.localPos.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[m]" << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]   has_toe_joint = " << (tp.has_toe_joint?"true":"false") << std::endl;
      contact_states_index_map.insert(std::pair<std::string, size_t>(ee_name, i));////TODO:要移動?
    }
  }
}

#define TIMECALC 1
RTC::ReturnCode_t WholeBodyMasterSlave::onExecute(RTC::UniqueId ec_id){
  if(DEBUGP_ONCE)std::cerr << "[" << m_profile.instance_name<< "] onExecute(" << ec_id << ")" << std::endl;
  struct timeval t_calc_start, t_calc_end;
  if(TIMECALC)gettimeofday(&t_calc_start, NULL);
    if (m_qRefIn.isNew()) { m_qRefIn.read(); }
    if (m_basePosIn.isNew()) { m_basePosIn.read(); }
    if (m_baseRpyIn.isNew()) { m_baseRpyIn.read(); }
    if (m_zmpIn.isNew()) { m_zmpIn.read(); }
    if (m_optionalDataIn.isNew()) { m_optionalDataIn.read(); }
    //for HumanSynchronizer
    if(mode!=MODE_PAUSE){
      if (m_htrfwIn.isNew()){ m_htrfwIn.read(); WBMSCore::DoubleSeqToVector6(m_htrfw.data,hsp->hp_wld_raw.tgt[rf].w); }
      if (m_htlfwIn.isNew()){ m_htlfwIn.read(); WBMSCore::DoubleSeqToVector6(m_htlfw.data,hsp->hp_wld_raw.tgt[lf].w); }
      if (m_htcomIn.isNew()){ m_htcomIn.read(); WBMSCore::Pose3DToWBMSPose3D(m_htcom.data,hsp->hp_wld_raw.tgt[com].abs); }
      if (m_htrfIn.isNew()) { m_htrfIn.read();  WBMSCore::Pose3DToWBMSPose3D(m_htrf.data,hsp->hp_wld_raw.tgt[rf].abs); }
      if (m_htlfIn.isNew()) { m_htlfIn.read();  WBMSCore::Pose3DToWBMSPose3D(m_htlf.data,hsp->hp_wld_raw.tgt[lf].abs); }
      if (m_htrhIn.isNew()) { m_htrhIn.read();  WBMSCore::Pose3DToWBMSPose3D(m_htrh.data,hsp->hp_wld_raw.tgt[rh].abs);}
      if (m_htlhIn.isNew()) { m_htlhIn.read();  WBMSCore::Pose3DToWBMSPose3D(m_htlh.data,hsp->hp_wld_raw.tgt[lh].abs);}
      if (m_htheadIn.isNew()){ m_htheadIn.read(); WBMSCore::Pose3DToWBMSPose3D(m_hthead.data,hsp->hp_wld_raw.tgt[head].abs);}
      if (m_htzmpIn.isNew()){ m_htzmpIn.read();  WBMSCore::Point3DToVector3(m_htzmp.data,hsp->hp_wld_raw.tgt[zmp].abs.p); }
    }
//    if (m_actzmpIn.isNew()){m_actzmpIn.read(); }

    if ( is_legged_robot ) {

      processTransition();


      if(TIMECALC){gettimeofday(&t_calc_end, NULL); if(DEBUGP)cout<<"processTransition:"<<(double)(t_calc_end.tv_sec - t_calc_start.tv_sec) + (t_calc_end.tv_usec - t_calc_start.tv_usec)/1000000.0<<endl; t_calc_start = t_calc_end;}

      for(int i=0; i<fik_list.size(); i++){
        fik_list[i]->current_tm = m_qRef.tm;
        fik_list[i]->ikp["rleg"].is_ik_enable = true;
        fik_list[i]->ikp["lleg"].is_ik_enable = true;
        fik_list[i]->ikp["rarm"].is_ik_enable = hsp->WBMSparam.use_rh;
        fik_list[i]->ikp["larm"].is_ik_enable = hsp->WBMSparam.use_lh;
      }

      if (isRunning()) {
        if(hsp->is_initial_loop){
          preProcessForWholeBodyMasterSlave(fik, m_robot);
        }

        m_robot_ml->rootLink()->p = m_robot_rmc->rootLink()->p;
        m_robot_ml->rootLink()->R = m_robot_rmc->rootLink()->R;
//        for (int i=0; i<m_robot_ml->numJoints(); i++){ m_robot_ml->joint(i)->q = m_robot_rmc->joint(i)->q;}
//        m_robot_ml->rootLink()->p = m_robot->rootLink()->p;
//        m_robot_ml->rootLink()->R = m_robot->rootLink()->R;
//        for (int i=0; i<m_robot_ml->numJoints(); i++){ m_robot_ml->joint(i)->q = m_robot->joint(i)->q;}
//        m_robot_ml->calcForwardKinematics();
        calcManipulabilityJointLimitForWBMS(fik_ml, m_robot_ml);

        if(hsp->WBMSparam.is_doctor){
          processWholeBodyMasterSlave(fik, m_robot, hsp->rp_ref_out);//安全制限つきマスタ・スレーブ
        }else{
          processWholeBodyMasterSlave_Raw(fik, m_robot, raw_pose);//生マスタ・スレーブ
        }


        if(TIMECALC){gettimeofday(&t_calc_end, NULL); if(DEBUGP)cout<<"processWholeBodyMasterSlave:"<<(double)(t_calc_end.tv_sec - t_calc_start.tv_sec) + (t_calc_end.tv_usec - t_calc_start.tv_usec)/1000000.0<<endl; t_calc_start = t_calc_end;}
        //逆動力学初期化
        if(hsp->is_initial_loop){
          idsb.setInitState(m_robot, m_dt);
          idsb2.setInitState(m_robot, m_dt);
        }

        //逆動力学
        calcAccelerationsForInverseDynamics(m_robot, idsb);
        hrp::Vector3 ref_zmp_invdyn;
        calcWorldZMPFromInverseDynamics(m_robot, idsb, ref_zmp_invdyn);
        ref_zmp_invdyn = invdyn_zmp_filters.passFilter(ref_zmp_invdyn);
        updateInvDynStateBuffer(idsb);



        if(TIMECALC){gettimeofday(&t_calc_end, NULL); if(DEBUGP)cout<<"calcWorldZMPFromInverseDynamics:"<<(double)(t_calc_end.tv_sec - t_calc_start.tv_sec) + (t_calc_end.tv_usec - t_calc_start.tv_usec)/1000000.0<<endl; t_calc_start = t_calc_end;}

        //角運動量補償
        if(hsp->WBMSparam.is_doctor){
          processMomentumCompensation(fik_rmc, m_robot_rmc, m_robot, hsp->rp_ref_out);
        }else{
          processMomentumCompensation(fik_rmc, m_robot_rmc, m_robot, raw_pose);
        }
//      calcAccelerationsForInverseDynamics(m_robot, idsb2);
//      calcWorldZMPFromInverseDynamics(m_robot, idsb2, ref_zmp_invdyn2);
//      updateInvDynStateBuffer(idsb2);



        if(TIMECALC){gettimeofday(&t_calc_end, NULL); if(DEBUGP)cout<<"processMomentumCompensation:"<<(double)(t_calc_end.tv_sec - t_calc_start.tv_sec) + (t_calc_end.tv_usec - t_calc_start.tv_usec)/1000000.0<<endl; t_calc_start = t_calc_end;}
        //OutPortデータセット
        //        ref_zmp = ref_zmp_invdyn;
        hrp::Vector3 ref_zmp = hsp->rp_ref_out.tgt[zmp].abs.p;
//        hrp::BodyPtr m_robot_for_out = m_robot;
        hrp::BodyPtr m_robot_for_out = m_robot_rmc;
        // qRef
        for (int i = 0; i < m_qRef.data.length(); i++ ){ m_qRef.data[i] = transition_interpolator_ratio * m_robot_for_out->joint(i)->q  + (1 - transition_interpolator_ratio) * m_qRef.data[i]; }
        // basePos
        m_basePos.data.x = transition_interpolator_ratio * m_robot_for_out->rootLink()->p(0) + (1 - transition_interpolator_ratio) * m_basePos.data.x;
        m_basePos.data.y = transition_interpolator_ratio * m_robot_for_out->rootLink()->p(1) + (1 - transition_interpolator_ratio) * m_basePos.data.y;
        m_basePos.data.z = transition_interpolator_ratio * m_robot_for_out->rootLink()->p(2) + (1 - transition_interpolator_ratio) * m_basePos.data.z;
        m_basePos.tm = m_qRef.tm;
        // baseRpy
        hrp::Vector3 baseRpy = hrp::rpyFromRot(m_robot_for_out->rootLink()->R);
        m_baseRpy.data.r = transition_interpolator_ratio * baseRpy(0) + (1 - transition_interpolator_ratio) * m_baseRpy.data.r;
        m_baseRpy.data.p = transition_interpolator_ratio * baseRpy(1) + (1 - transition_interpolator_ratio) * m_baseRpy.data.p;
        m_baseRpy.data.y = transition_interpolator_ratio * baseRpy(2) + (1 - transition_interpolator_ratio) * m_baseRpy.data.y;
        m_baseRpy.tm = m_qRef.tm;
        // zmp
        hrp::Vector3 rel_ref_zmp = m_robot_for_out->rootLink()->R.transpose() * (ref_zmp - m_robot_for_out->rootLink()->p);
        m_zmp.data.x = transition_interpolator_ratio * rel_ref_zmp(0) + (1 - transition_interpolator_ratio) * m_zmp.data.x;
        m_zmp.data.y = transition_interpolator_ratio * rel_ref_zmp(1) + (1 - transition_interpolator_ratio) * m_zmp.data.y;
        m_zmp.data.z = transition_interpolator_ratio * rel_ref_zmp(2) + (1 - transition_interpolator_ratio) * m_zmp.data.z;
        m_zmp.tm = m_qRef.tm;
        // m_optionalData
        if(m_optionalData.data.length() < optionalDataLength){
          m_optionalData.data.length(optionalDataLength);//TODO:これいいのか？
          for(int i=0;i<optionalDataLength;i++)m_optionalData.data[i] = 0;
        }
        m_optionalData.data[contact_states_index_map["rleg"]] = m_optionalData.data[contact_states_index_map["rleg"]+4] = hsp->rp_ref_out.tgt[rf].is_contact;
        m_optionalData.data[contact_states_index_map["lleg"]] = m_optionalData.data[contact_states_index_map["lleg"]+4] = hsp->rp_ref_out.tgt[lf].is_contact;
      }
      hsp->baselinkpose.p = m_robot->rootLink()->p;
      hsp->baselinkpose.rpy = hrp::rpyFromRot(m_robot->rootLink()->R);
    }
    previous_mode = mode;
    // write
    m_qOut.write();
    m_basePosOut.write();
    m_baseRpyOut.write();
    m_zmpOut.write();
    m_optionalDataOut.write();

#ifdef USE_DEBUG_PORT
    // dbg plot
     m_htcom_dbg.tm = m_qRef.tm;
     WBMSCore::WBMSPose3DToPose3D(hsp->hp_plot.tgt[com].abs, m_htcom_dbg.data);
     m_htcom_dbgOut.write();
     m_htrf_dbg.tm = m_qRef.tm;
     WBMSCore::WBMSPose3DToPose3D(hsp->hp_plot.tgt[rf].abs, m_htrf_dbg.data);
     m_htrf_dbgOut.write();
     m_htlf_dbg.tm = m_qRef.tm;
     WBMSCore::WBMSPose3DToPose3D(hsp->hp_plot.tgt[lf].abs, m_htlf_dbg.data);
     m_htlf_dbgOut.write();
     m_htrh_dbg.tm = m_qRef.tm;
     WBMSCore::WBMSPose3DToPose3D(hsp->hp_plot.tgt[rh].abs, m_htrh_dbg.data);
     m_htrh_dbgOut.write();
     m_htlh_dbg.tm = m_qRef.tm;
     WBMSCore::WBMSPose3DToPose3D(hsp->hp_plot.tgt[lh].abs, m_htlh_dbg.data);
     m_htlh_dbgOut.write();
     m_hthead_dbg.tm = m_qRef.tm;
     WBMSCore::WBMSPose3DToPose3D(hsp->hp_plot.tgt[head].abs, m_hthead_dbg.data);
     m_hthead_dbgOut.write();
     m_htzmp_dbg.tm = m_qRef.tm;
     WBMSCore::Vector3ToPoint3D(hsp->rp_ref_out.tgt[zmp].abs.p,m_rpzmp_dbg.data);
     m_htzmp_dbgOut.write();
     m_htrfw_dbg.tm = m_qRef.tm;
     m_htrfw_dbg.data.length(6);
     WBMSCore::Vector6ToDoubleSeq(hsp->hp_plot.tgt[rf].w, m_htrfw_dbg.data);
     m_htrfw_dbgOut.write();
     m_htlfw_dbg.tm = m_qRef.tm;
     m_htlfw_dbg.data.length(6);
     WBMSCore::Vector6ToDoubleSeq(hsp->hp_plot.tgt[lf].w, m_htlfw_dbg.data);
     m_htlfw_dbgOut.write();
     m_rpcom_dbg.tm = m_qRef.tm;
     WBMSCore::WBMSPose3DToPose3D(hsp->rp_ref_out.tgt[com].abs, m_rpcom_dbg.data);
     m_rpcom_dbgOut.write();
     m_rprf_dbg.tm = m_qRef.tm;
     WBMSCore::WBMSPose3DToPose3D(hsp->rp_ref_out.tgt[rf].abs, m_rprf_dbg.data);
     m_rprf_dbgOut.write();
     m_rplf_dbg.tm = m_qRef.tm;
     WBMSCore::WBMSPose3DToPose3D(hsp->rp_ref_out.tgt[lf].abs, m_rplf_dbg.data);
     m_rplf_dbgOut.write();
     m_rprh_dbg.tm = m_qRef.tm;
     WBMSCore::WBMSPose3DToPose3D(hsp->rp_ref_out.tgt[rh].abs, m_rprh_dbg.data);
     m_rprh_dbgOut.write();
     m_rplh_dbg.tm = m_qRef.tm;
     WBMSCore::WBMSPose3DToPose3D(hsp->rp_ref_out.tgt[lh].abs, m_rplh_dbg.data);
     m_rplh_dbgOut.write();
     m_rphead_dbg.tm = m_qRef.tm;
     WBMSCore::WBMSPose3DToPose3D(hsp->rp_ref_out.tgt[head].abs, m_rphead_dbg.data);
     m_rphead_dbgOut.write();
     m_rpzmp_dbg.tm = m_qRef.tm;
     WBMSCore::Vector3ToPoint3D(hsp->rp_ref_out.tgt[zmp].abs.p, m_rpzmp_dbg.data);
     m_rpzmp_dbgOut.write();
     m_rpdcp_dbg.tm = m_qRef.tm;
     WBMSCore::Vector3ToPoint3D(hsp->cp_dec,m_rpdcp_dbg.data);
     m_rpdcp_dbgOut.write();
     m_rpacp_dbg.tm = m_qRef.tm;
     WBMSCore::Vector3ToPoint3D(hsp->cp_acc,m_rpacp_dbg.data);
     m_rpacp_dbgOut.write();
     m_invdyn_dbg.tm = m_qRef.tm;
     m_invdyn_dbg.data.length(6);
     WBMSCore::Vector6ToDoubleSeq(hsp->invdyn_ft,m_invdyn_dbg.data);
     m_invdyn_dbgOut.write();
#endif
     if(TIMECALC){gettimeofday(&t_calc_end, NULL); if(DEBUGP)cout<<"t_last:"<<(double)(t_calc_end.tv_sec - t_calc_start.tv_sec) + (t_calc_end.tv_usec - t_calc_start.tv_usec)/1.0e6<<endl; t_calc_start = t_calc_end;}
    loop ++;
    return RTC::RTC_OK;
}


void WholeBodyMasterSlave::processTransition(){
  switch(mode){

    case MODE_SYNC_TO_WBMS:
      if(previous_mode == MODE_IDLE){ double tmp = 1.0; transition_interpolator->setGoal(&tmp, 3.0, true); }
      if (!transition_interpolator->isEmpty() ){
        transition_interpolator->get(&transition_interpolator_ratio, true);
      }else{
        mode = MODE_WBMS;
      }
      break;

    case MODE_SYNC_TO_IDLE:
      if(previous_mode == MODE_WBMS || previous_mode == MODE_PAUSE){ double tmp = 0.0; transition_interpolator->setGoal(&tmp, 3.0, true); }
      if (!transition_interpolator->isEmpty()) {
        transition_interpolator->get(&transition_interpolator_ratio, true);
      }else{
        mode = MODE_IDLE;
      }
      break;
  }
}


void WholeBodyMasterSlave::preProcessForWholeBodyMasterSlave(fikPtr& fik_in, hrp::BodyPtr& robot_in){
  hrp::Vector3 basePos_heightChecked = hrp::Vector3(m_basePos.data.x, m_basePos.data.y, m_basePos.data.z);//ベースリンク高さ調整により足裏高さ0に
  robot_in->rootLink()->p = basePos_heightChecked;
  for ( int i = 0; i < robot_in->numJoints(); i++ ){ robot_in->joint(i)->q = m_qRef.data[i]; }
  robot_in->calcForwardKinematics();
  hrp::Vector3 rsole_pos = fik_in->ikp["rleg"].target_link->p+ fik_in->ikp["rleg"].target_link->R * fik_in->ikp["rleg"].localPos;
  hrp::Vector3 lsole_pos = fik_in->ikp["lleg"].target_link->p+ fik_in->ikp["lleg"].target_link->R * fik_in->ikp["lleg"].localPos;
  hrp::Vector3 init_foot_mid_coord = (rsole_pos + lsole_pos) / 2;
  if( fabs((double)init_foot_mid_coord(Z)) > 1e-5 ){
    basePos_heightChecked(Z) -= init_foot_mid_coord(Z);
    init_foot_mid_coord(Z) = 0;
    std::cerr<<"["<<m_profile.instance_name<<"] Input basePos height is invalid. Auto modify "<<m_basePos.data.z<<" -> "<<basePos_heightChecked(Z)<<endl;
  }
  for(int i=0;i<fik_list.size();i++){//初期姿勢でBodyをFK
    body_list[i]->rootLink()->p = basePos_heightChecked;
    body_list[i]->rootLink()->R = hrp::rotFromRpy(m_baseRpy.data.r, m_baseRpy.data.p, m_baseRpy.data.y);
    for ( int j = 0; j < body_list[i]->numJoints(); j++ ){ body_list[i]->joint(j)->q = m_qRef.data[j]; }
    body_list[i]->calcForwardKinematics();
    fik_list[i]->setReferenceJointAngles();
  }
  hsp->setCurrentInputAsOffset(hsp->hp_wld_raw);//現在の入力を人間の初期姿勢としてセット
  const std::string robot_l_names[4] = {"rleg","lleg","rarm","larm"};
  const int human_l_names[4] = {rf,lf,rh,lh};
  for(int i=0;i<4;i++){//HumanSynchronizerの初期姿勢オフセットをセット
    if(fik_in->ikp.count(robot_l_names[i])){
      hrp::Vector3 init_ee_p = fik_in->ikp[robot_l_names[i]].target_link->p + fik_in->ikp[robot_l_names[i]].target_link->R * fik_in->ikp[robot_l_names[i]].localPos;
      hrp::Matrix33 init_ee_R = fik_in->ikp[robot_l_names[i]].target_link->R * fik_in->ikp[robot_l_names[i]].localR;
      hsp->rp_ref_out.tgt[human_l_names[i]].offs.p = init_ee_p;
      hsp->rp_ref_out.tgt[human_l_names[i]].offs.rpy = hrp::rpyFromRot(init_ee_R);
      hsp->rp_ref_out.tgt[human_l_names[i]].abs = hsp->rp_ref_out.tgt[human_l_names[i]].offs;
      fik->ikp[robot_l_names[i]].target_p0 = fik_rmc->ikp[robot_l_names[i]].target_p0 = init_ee_p;
      fik->ikp[robot_l_names[i]].target_r0 = fik_rmc->ikp[robot_l_names[i]].target_r0 = init_ee_R;
    }
  }
  hsp->rp_ref_out.tgt[com].offs.p = robot_in->calcCM();
  hsp->rp_ref_out.tgt[zmp].offs.p(X) = hsp->rp_ref_out.tgt[com].offs.p(X);
  hsp->rp_ref_out.tgt[zmp].offs.p(Y) = hsp->rp_ref_out.tgt[com].offs.p(Y);
  hsp->rp_ref_out.tgt[zmp].offs.p(Z) = init_foot_mid_coord(Z);
  hsp->rp_ref_out.tgt[rf].cnt = hsp->rp_ref_out.tgt[rf].offs;
  hsp->rp_ref_out.tgt[lf].cnt = hsp->rp_ref_out.tgt[lf].offs;
  hsp->baselinkpose.p = robot_in->rootLink()->p;
  hsp->baselinkpose.rpy = hrp::rpyFromRot(robot_in->rootLink()->R);
}


void WholeBodyMasterSlave::processWholeBodyMasterSlave(fikPtr& fik_in, hrp::BodyPtr& robot_in, const HumanPose& pose_ref){
  hsp->update();//////HumanSynchronizerの主要処理
  if(DEBUGP)cout<<"update():"<<hsp->getUpdateTime()<<endl;
  if(DEBUGP)pose_ref.print();

  WBMSPose3D rf_checked = pose_ref.tgt[rf].abs;
  WBMSPose3D lf_checked = pose_ref.tgt[lf].abs;
  WBMSPose3D rh_checked = pose_ref.tgt[rh].abs;
  WBMSPose3D lh_checked = pose_ref.tgt[lh].abs;

  static hrp::Vector3 target_v_old[4];
  if(hsp->WBMSparam.use_manipulability_limit){
    calcManipulabilityJointLimit(fik_in, robot_in, target_v_old, rf_checked, lf_checked, rh_checked, lh_checked);
  }
  solveFullbodyIKStrictCOM(fik_in, robot_in, pose_ref.tgt[com].abs, rf_checked, lf_checked, rh_checked, lh_checked, pose_ref.tgt[head].abs,"processWholeBodyMasterSlave");
}


void WholeBodyMasterSlave::processWholeBodyMasterSlave_Raw(fikPtr& fik_in, hrp::BodyPtr& robot_in, HumanPose& pose_ref){
  static BiquadIIRFilterVec pos_filters[num_pose_tgt], rot_filters[num_pose_tgt];
  static unsigned int callnum;
  for(int i=0;i<num_pose_tgt;i++){
    if(callnum == 0){
      pos_filters[i].setParameter(100.0, 1.0/m_dt, BiquadIIRFilterVec::Q_NOOVERSHOOT);
      rot_filters[i].setParameter(100.0, 1.0/m_dt, BiquadIIRFilterVec::Q_NOOVERSHOOT);
      pos_filters[i].reset(hsp->hp_wld_raw.tgt[i].abs.p);
      rot_filters[i].reset(hsp->hp_wld_raw.tgt[i].abs.rpy);
    }
    pose_ref.tgt[i].abs.p = pos_filters[i].passFilter(hsp->hp_wld_raw.tgt[i].abs.p);
    pose_ref.tgt[i].abs.rpy = rot_filters[i].passFilter(hsp->hp_wld_raw.tgt[i].abs.rpy);
  }
  callnum++;
  if(DEBUGP){ fprintf(stderr,"\x1b[31mmaster-mode:\x1b[39m"); pose_ref.print(); }
  solveFullbodyIKStrictCOM(fik_in, robot_in, pose_ref.tgt[com].abs, pose_ref.tgt[rf].abs, pose_ref.tgt[lf].abs, pose_ref.tgt[rh].abs, pose_ref.tgt[lh].abs, pose_ref.tgt[head].abs,"processWholeBodyMasterSlave_Raw");
}


void WholeBodyMasterSlave::calcVelAccSafeTrajectory(const hrp::Vector3& pos_cur, const hrp::Vector3& vel_cur, const hrp::Vector3& pos_tgt, const hrp::Vector3& max_acc, const double& max_vel, hrp::Vector3& pos_ans){
  hrp::Vector3 direc( SGN(pos_tgt(X)-pos_cur(X)), SGN(pos_tgt(Y)-pos_cur(Y)), SGN(pos_tgt(Z)-pos_cur(Z)));
  hrp::Vector3 vel_ans;
  for(int i=0;i<XYZ;i++){vel_ans(i) = vel_cur(i) + direc(i) * max_acc(i) * m_dt;}
  for(int i=0;i<XYZ;i++){
    double stop_safe_vel = sqrt( 2 * max_acc(i) * fabs(pos_tgt(i) - pos_cur(i)) );
    if( direc(i) > 0 ){
      if(vel_ans(i) > stop_safe_vel){ vel_ans(i) = stop_safe_vel; }
    }else{
      if(vel_ans(i) < - stop_safe_vel){ vel_ans(i) = - stop_safe_vel; }
    }
  }
  pos_ans = pos_cur + vel_ans * m_dt;
}

//これ超計算重い
void WholeBodyMasterSlave::calcManipulabilityJointLimit(fikPtr& fik_in, hrp::BodyPtr& robot_in, hrp::Vector3 target_v_old[], WBMSPose3D& rf_ref, WBMSPose3D& lf_ref, WBMSPose3D& rh_ref, WBMSPose3D& lh_ref){
  const std::string names[4] = {"rleg","lleg","rarm","larm"};
  WBMSPose3D* refs[4] = {&rf_ref, &lf_ref, &rh_ref, &lh_ref};
  double min_manip_val = 1e9;
  const double min_manip_th = 0.1;
  hrp::Vector3 target_p0_cur[4], target_p0_new[4];
  hrp::dmatrix J,Jinv,Jnull;

  for(int l=0; l<4; l++){
    fik_in->ikp[names[l]].manip->calcJacobian(J);
    fik_in->ikp[names[l]].manip->calcJacobianInverseNullspace(J, Jinv, Jnull);
//    hrp::dmatrix manipulability_mat = J*J.transpose();
//    double manipulability = sqrt((J*J.transpose()).determinant());
    Eigen::JacobiSVD< Eigen::MatrixXd > svd(J.block(0,0,3,J.cols()), Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::MatrixXd::Index row,col;
    min_manip_val = svd.singularValues().minCoeff(&row,&col);
    hrp::Vector3 min_manip_direc = svd.matrixU().col(row);// unit vector
    if(min_manip_direc.dot(fik_in->ikp[names[l]].target_p0 - robot_in->rootLink()->p) < 0){ min_manip_direc *= -1; }//向きみて反転(常に外側正)
//    target_p0_cur[i] = fik_in->ikp[names[i]].target_p0;
    target_p0_cur[l] = fik_in->ikp[names[l]].target_link->p + fik_in->ikp[names[l]].target_link->R * fik_in->ikp[names[l]].localPos;
    target_p0_new[l] = refs[l]->p;

    hrp::Vector3 ref_ee_vel = (target_p0_new[l] - target_p0_cur[l]) / m_dt;

    double penalty = 1 - (min_manip_val - min_manip_th) * 10;
    LIMIT_MINMAX(penalty, 0.0, 1.0);//0~1に頭打ち　//thより大で0, thに近づくにつれ0->1, th以下でも1

    double lim_penalty = 1.0;
    double lim_penalty_tmp = 1.0;
    double ulim_mergin[6], llim_mergin[6];
    static double ulim_mergin_old[6], llim_mergin_old[6];
    const double jlim_start_th = 30*D2R;

    hrp::dvector q_vel(6);
    q_vel = Jinv.block(0,0,Jinv.cols(),3) * ref_ee_vel;
    for(int j=0;j<6;j++){
      llim_mergin[j] = fabs(robot_in->joint(j)->llimit - robot_in->joint(j)->q);
      ulim_mergin[j] = fabs(robot_in->joint(j)->ulimit - robot_in->joint(j)->q);
      if( llim_mergin[j] < jlim_start_th && q_vel[j] < 0){
        lim_penalty_tmp =  llim_mergin[j]/jlim_start_th;
        if(lim_penalty_tmp < lim_penalty){lim_penalty = lim_penalty_tmp;}
      }
      if( ulim_mergin[j] < jlim_start_th && q_vel[j] > 0){
        lim_penalty_tmp =  ulim_mergin[j]/jlim_start_th;
        if(lim_penalty_tmp < lim_penalty){lim_penalty = lim_penalty_tmp;}
      }
    }
//    if(DEBUGP){ if(lim_penalty<1.0)cout<<"lim_penalty: "<<lim_penalty<<endl; }
//    double max_dec_vel = 1.0;// m/s
//    hrp::Vector3 ref_ee_vel_v = (ref_ee_vel.dot(min_manip_direc)) * min_manip_direc;
//    hrp::Vector3 ref_ee_vel_h = ref_ee_vel - ref_ee_vel_v;
//        ref_vec_mod -= (ref_vec.dot(min_manip_direc)) * min_manip_direc;
//        ref_vec_mod += (ref_vec.dot(min_manip_direc)) * min_manip_direc * penalty;
//    max_dec_vel *= penalty;
//    if(ref_ee_vel_v.dot(min_manip_direc) > max_dec_vel){
//      ref_ee_vel_v = min_manip_direc * max_dec_vel;
//    }
//    if( (min_manip_val - min_manip_th) <0 ){
//      ref_ee_vel_v = min_manip_direc * (min_manip_val - min_manip_th) * 10;
//    }
//    hrp::Vector3 ref_ee_vel_mod = ref_ee_vel_h + ref_ee_vel_v;//速度空間で一次フィルターしたほうがいい?
//    ref_ee_vel_mod *= lim_penalty;
//    penalty *= lim_penalty;
    hrp::Vector3 max_acc_ratio = hrp::Vector3(1,1,1);

//    LIMIT_MINMAX(penalty, 0.1, 1.0);
    for(int i=0;i<3;i++){
      max_acc_ratio(i) -= fabs(min_manip_direc(i)) * penalty;
    }
    hrp::Vector3 max_acc = max_acc_ratio * 2.0;
    for(int i=0;i<XYZ;i++){ LIMIT_MIN(max_acc(i), 0.1); }

    if(DEBUGP)std::cout << names[l]<<": direc: " << min_manip_direc.transpose() << " min_manip_val: " << min_manip_val << " penalty: "<<penalty<<" max_acc: "<<max_acc.transpose()<<std::endl;
    calcVelAccSafeTrajectory(target_p0_cur[l], target_v_old[l], (target_p0_cur[l] + ref_ee_vel * m_dt), max_acc, 1.0, target_p0_new[l]);

    refs[l]->p = target_p0_new[l];
//    refs[l]->rpy = refs[l]->rpy;
    target_v_old[l] = (target_p0_new[l] - target_p0_cur[l] )/m_dt;
  }
}


void WholeBodyMasterSlave::calcManipulabilityJointLimitForWBMS(fikPtr& fik_in, hrp::BodyPtr& robot_in){
//  const std::string names[4] = {"rleg","lleg","rarm","larm"};
//  double min_manip_val = 1e9;
//  const double min_manip_th = 0.1;
//
//  static hrp::Vector3 target_p0_old[4];
//  static bool firstcall = true;
////    for(int i=0;i<4;i++){
//  for(int i=0;i<1;i++){
//    hrp::dmatrix J,Jinv,Jnull;
//    fik_in->ikp[names[i]].manip->calcJacobian(J);
//    fik_in->ikp[names[i]].manip->calcJacobianInverseNullspace(J, Jinv, Jnull);
//
//    hrp::dmatrix manipulability_mat = J*J.transpose();
//    double manipulability = sqrt((J*J.transpose()).determinant());
//    //    Eigen::JacobiSVD< Eigen::Matrix<float, 3, 3> > svd(mm3, Eigen::ComputeFullU | Eigen::ComputeFullV);
//    Eigen::JacobiSVD< Eigen::MatrixXd > svd(J.block(0,0,3,J.cols()), Eigen::ComputeFullU | Eigen::ComputeFullV);
//
//    Eigen::MatrixXd::Index row,col;
//    min_manip_val = svd.singularValues().minCoeff(&row,&col);
//
//    hrp::Vector3 min_manip_direc = svd.matrixU().col(row);// unit vector
//    //      std::cout << "singular values" << std::endl << svd.singularValues() << std::endl;
//    //      std::cout << "matrix U" << std::endl << svd.matrixU() << std::endl;
//    //      std::cout << "matrix V" << std::endl << svd.matrixV() << std::endl;
//    if(min_manip_direc.dot(fik_in->ikp[names[i]].target_p0 - robot_in->rootLink()->p) < 0){ min_manip_direc *= -1; }//向きみて反転(常に外側正)
//
    hsp->fik = fik_in;
    hsp->m_robot = robot_in;
//    hsp->cur_min_manip_direc = min_manip_direc;
//    hsp->cur_manip_val = min_manip_val;
//  }
}


void WholeBodyMasterSlave::solveFullbodyIKStrictCOM(fikPtr& fik_in, hrp::BodyPtr& robot_in, const WBMSPose3D& com_ref, const WBMSPose3D& rf_ref, const WBMSPose3D& lf_ref, const WBMSPose3D& rh_ref, const WBMSPose3D& lh_ref, const WBMSPose3D& head_ref, const std::string& debug_prefix){
  int com_ik_loop=0;
  const int COM_IK_MAX_LOOP = 10;
  const double COM_IK_MAX_ERROR = 1e-5;//1e-4だと乱れる
  const std::string names[4] = {"rleg","lleg","rarm","larm"};
  const WBMSPose3D* refs[4] = {&rf_ref, &lf_ref, &rh_ref, &lh_ref};
//  robot_in->rootLink()->p = 0.01*com_ref.p + 0.99*robot_in->rootLink()->p;//ベースリンク位置が迷走するのを防ぐ？
  robot_in->rootLink()->R = hrp::rotFromRpy(com_ref.rpy);//move base link at first
  for(int i=0;i<4;i++){
    if(fik_in->ikp.count(names[i])){
      fik_in->ikp[names[i]].target_p0 = refs[i]->p;
      fik_in->ikp[names[i]].target_r0 = hrp::rotFromRpy(refs[i]->rpy);
    }
  }
//  fik_in->storeCurrentParameters();
//  fik_in->setReferenceJointAngles();//これ入れると腕ブワーなる
  if(robot_in->link("HEAD_JOINT0") != NULL)robot_in->joint(15)->q = head_ref.rpy(y);
  if(robot_in->link("HEAD_JOINT1") != NULL)robot_in->joint(16)->q = head_ref.rpy(p);
  hrp::Vector3 tmp_com_err = hrp::Vector3::Zero();
  while( 1 ){  //COM 収束ループ
    com_ik_loop++;
    robot_in->rootLink()->p += tmp_com_err;
    robot_in->calcForwardKinematics();
    for ( std::map<std::string, SimpleFullbodyInverseKinematicsSolver::IKparam>::iterator it = fik_in->ikp.begin(); it != fik_in->ikp.end(); it++ ) {
        if (it->second.is_ik_enable) fik_in->solveLimbIK (it->second, it->first, fik_in->ratio_for_vel, false);
    }
    tmp_com_err = com_ref.p - robot_in->calcCM();
    if(tmp_com_err.norm() < COM_IK_MAX_ERROR){ break; }
    if(com_ik_loop >= COM_IK_MAX_LOOP){std::cerr << "COM constraint IK MAX loop [="<<COM_IK_MAX_LOOP<<"] exceeded!! @ "<<debug_prefix<< std::endl; break; };
  }

  if(com_ik_loop != 1 && DEBUGP)cout<<"com_ik_loop:"<<com_ik_loop<<" @ "<<debug_prefix<<endl;
}


void WholeBodyMasterSlave::processMomentumCompensation(fikPtr& fik_in, hrp::BodyPtr& robot_in, hrp::BodyPtr& robot_normal_in, const HumanPose& pose_ref){
  hrp::Vector3 P,L;//世界座標系
  robot_normal_in->rootLink()->v = idsb.base_v;
  robot_normal_in->rootLink()->w = idsb.base_w;
  robot_normal_in->calcTotalMomentum(P,L);
  L = L - robot_normal_in->calcCM().cross(P);//重心周りの角運動量に変換
  robot_normal_in->calcCM();
  robot_normal_in->rootLink()->calcSubMassCM();
  hrp::Matrix33 I_com;
  static hrp::Vector3 torso_rot;
  hrp::Vector3 torso_rot_vel;
  if(robot_normal_in->link("CHEST_JOINT0")){
    robot_normal_in->link("CHEST_JOINT0")->calcSubMassInertia(I_com);
    torso_rot_vel = - I_com.inverse() * L * hsp->WBMSparam.upper_body_rmc_ratio;
    torso_rot += torso_rot_vel * m_dt;
  }
  WBMSPose3D com_mod = pose_ref.tgt[com].abs;
  com_mod.rpy += torso_rot;

  WBMSPose3D rf_checked = pose_ref.tgt[rf].abs;
  WBMSPose3D lf_checked = pose_ref.tgt[lf].abs;
  WBMSPose3D rh_checked = pose_ref.tgt[rh].abs;
  WBMSPose3D lh_checked = pose_ref.tgt[lh].abs;

  static hrp::Vector3 target_v_old[4];
  if(hsp->WBMSparam.use_manipulability_limit){
    struct timeval t_calc_start, t_calc_end;
    if(TIMECALC)gettimeofday(&t_calc_start, NULL);
    calcManipulabilityJointLimit(fik_in, robot_in, target_v_old, rf_checked, lf_checked, rh_checked, lh_checked);
    if(TIMECALC){gettimeofday(&t_calc_end, NULL); if(DEBUGP)cout<<"calcManipulabilityJointLimit:"<<(double)(t_calc_end.tv_sec - t_calc_start.tv_sec) + (t_calc_end.tv_usec - t_calc_start.tv_usec)/1.0e6<<endl; t_calc_start = t_calc_end;}
  }
  solveFullbodyIKStrictCOM(fik_in, robot_in, com_mod, rf_checked, lf_checked, rh_checked, lh_checked, pose_ref.tgt[head].abs,"calcDynamicsFilterCompensation");
}


bool WholeBodyMasterSlave::startCountDownForWholeBodyMasterSlave(const double sec){
  if(sec >= 0.0 && sec <= 30.0){
    std::cerr << "[" << m_profile.instance_name << "] start Synchronization after "<<sec<<" [s]" << std::endl;
    double remained_usec = sec * 10e6;
    while (remained_usec > 0){
      usleep(2000);
      remained_usec -= 2000;
    }
    startWholeBodyMasterSlave();
    return true;
  }else{
    std::cerr << "[" << m_profile.instance_name << "] Count Down Time must be 0 < T < 30 [s]"<< std::endl;
    return false;
  }
}


bool WholeBodyMasterSlave::startWholeBodyMasterSlave(){
  std::cerr << "[" << m_profile.instance_name << "] startWholeBodyMasterSlave" << std::endl;
  mode = MODE_SYNC_TO_WBMS;
  while(!transition_interpolator->isEmpty()){ usleep(1000); }
  return true;
}


bool WholeBodyMasterSlave::pauseWholeBodyMasterSlave(){
  std::cerr << "[" << m_profile.instance_name << "] pauseWholeBodyMasterSlave" << std::endl;
  mode = MODE_PAUSE;
  return true;
}


bool WholeBodyMasterSlave::resumeWholeBodyMasterSlave(){
  std::cerr << "[" << m_profile.instance_name << "] pauseWholeBodyMasterSlave" << std::endl;
  mode = MODE_WBMS;
  return true;
}


bool WholeBodyMasterSlave::stopWholeBodyMasterSlave(){
  std::cerr << "[" << m_profile.instance_name << "] stopWholeBodyMasterSlave" << std::endl;
  mode = MODE_SYNC_TO_IDLE;
  while(!transition_interpolator->isEmpty()){ usleep(1000); }
  return true;
}


bool WholeBodyMasterSlave::setWholeBodyMasterSlaveParam(const OpenHRP::WholeBodyMasterSlaveService::WholeBodyMasterSlaveParam& i_param){
  std::cerr << "[" << m_profile.instance_name << "] setWholeBodyMasterSlaveParam" << std::endl;
  hsp->WBMSparam.auto_swing_foot_landing_threshold = i_param.auto_swing_foot_landing_threshold;
  hsp->WBMSparam.foot_vertical_vel_limit_coeff = i_param.foot_vertical_vel_limit_coeff;
  hsp->WBMSparam.human_com_height = i_param.human_com_height;
  hsp->WBMSparam.is_doctor = i_param.is_doctor;
  hsp->WBMSparam.set_com_height_fix = i_param.set_com_height_fix;
  hsp->WBMSparam.set_com_height_fix_val = i_param.set_com_height_fix_val;
  hsp->WBMSparam.swing_foot_height_offset = i_param.swing_foot_height_offset;
  hsp->WBMSparam.swing_foot_max_height = i_param.swing_foot_max_height;
  hsp->WBMSparam.upper_body_rmc_ratio = i_param.upper_body_rmc_ratio;
  hsp->WBMSparam.use_rh = hsp->WBMSparam.use_lh = i_param.use_hands;
  hsp->WBMSparam.use_head = i_param.use_head;
  hsp->WBMSparam.use_manipulability_limit = i_param.use_manipulability_limit;
  return true;
}


bool WholeBodyMasterSlave::getWholeBodyMasterSlaveParam(OpenHRP::WholeBodyMasterSlaveService::WholeBodyMasterSlaveParam& i_param){
  std::cerr << "[" << m_profile.instance_name << "] getWholeBodyMasterSlaveParam" << std::endl;
  i_param.auto_swing_foot_landing_threshold = hsp->WBMSparam.auto_swing_foot_landing_threshold;
  i_param.foot_vertical_vel_limit_coeff = hsp->WBMSparam.foot_vertical_vel_limit_coeff;
  i_param.human_com_height = hsp->WBMSparam.human_com_height;
  i_param.is_doctor = hsp->WBMSparam.is_doctor;
  i_param.set_com_height_fix = hsp->WBMSparam.set_com_height_fix;
  i_param.set_com_height_fix_val = hsp->WBMSparam.set_com_height_fix_val;
  i_param.swing_foot_height_offset = hsp->WBMSparam.swing_foot_height_offset;
  i_param.swing_foot_max_height = hsp->WBMSparam.swing_foot_max_height;
  i_param.upper_body_rmc_ratio = hsp->WBMSparam.upper_body_rmc_ratio;
  i_param.use_hands = (hsp->WBMSparam.use_rh || hsp->WBMSparam.use_lh);
  i_param.use_head = hsp->WBMSparam.use_head;
  i_param.use_manipulability_limit = hsp->WBMSparam.use_manipulability_limit;
  return true;
}


RTC::ReturnCode_t WholeBodyMasterSlave::onFinalize(){ return RTC::RTC_OK; }
RTC::ReturnCode_t WholeBodyMasterSlave::onActivated(RTC::UniqueId ec_id){ std::cerr << "[" << m_profile.instance_name<< "] onActivated(" << ec_id << ")" << std::endl; return RTC::RTC_OK; }
RTC::ReturnCode_t WholeBodyMasterSlave::onDeactivated(RTC::UniqueId ec_id){ std::cerr << "[" << m_profile.instance_name<< "] onDeactivated(" << ec_id << ")" << std::endl; return RTC::RTC_OK; }

extern "C"{
    void WholeBodyMasterSlaveInit(RTC::Manager* manager) {
        RTC::Properties profile(WholeBodyMasterSlave_spec);
        manager->registerFactory(profile, RTC::Create<WholeBodyMasterSlave>, RTC::Delete<WholeBodyMasterSlave>);
    }
};
