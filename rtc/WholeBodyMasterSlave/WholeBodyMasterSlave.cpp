#include <rtm/CorbaNaming.h>
#include <hrpModel/Link.h>
#include <hrpModel/Sensor.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpModel/JointPath.h>
#include <hrpUtil/MatrixSolvers.h>
#include "hrpsys/util/Hrpsys.h"
#include "WholeBodyMasterSlave.h"

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

//static std::ostream& operator<<(std::ostream& os, const struct RTC::Time &tm)
//{
//    int pre = os.precision();
//    os.setf(std::ios::fixed);
//    os << std::setprecision(6)
//       << (tm.sec + tm.nsec/1e9)
//       << std::setprecision(pre);
//    os.unsetf(std::ios::fixed);
//    return os;
//}

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
    RTC::Manager& rtcManager = RTC::Manager::instance();
    std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
    int comPos = nameServer.find(",");
    if (comPos < 0){ comPos = nameServer.length(); }
    nameServer = nameServer.substr(0, comPos);
    RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
    m_robot = hrp::BodyPtr(new hrp::Body());
    if (!loadBodyFromModelLoader(m_robot, prop["model"].c_str(), CosNaming::NamingContext::_duplicate(naming.getRootContext()) )){
        std::cerr << "[" << m_profile.instance_name << "] failed to load model[" << prop["model"] << "]" << std::endl;
        return RTC::RTC_ERROR;
    }
    m_robot_vsafe = hrp::BodyPtr(new hrp::Body(*m_robot)); //copy from m_robot
    // allocate memory for outPorts
    m_qRef.data.length(m_robot->numJoints());
    coil::stringTo(optionalDataLength, prop["seq_optional_data_dim"].c_str());
    loop = 0;
    transition_interpolator = new interpolator(1, m_dt, interpolator::HOFFARBIB, 1);
    transition_interpolator->setName(std::string(m_profile.instance_name)+" transition_interpolator");
    transition_interpolator_ratio = 0;
    double tmp_ratio = 0.0;
    transition_interpolator->clear();
    transition_interpolator->set(&tmp_ratio);
    tmp_ratio = 1.0;
    //    transition_interpolator->setGoal(&tmp_ratio, 3.0, true);
    ROBOT_ALL_DOF = m_robot->numJoints() + 3 + 3;// joints + base_p + base_rpy
    //    q_ip = new interpolator(ROBOT_ALL_DOF, m_dt, interpolator::HOFFARBIB, 1);
    //    q_ip = new interpolator(ROBOT_ALL_DOF, m_dt, interpolator::QUINTICSPLINE, 1);
    q_ip = new interpolator(ROBOT_ALL_DOF, m_dt, interpolator::CUBICSPLINE, 1);
    q_ip->setName(std::string(m_profile.instance_name)+" q_ip");
    q_ip->clear();
    // Generate FIK
    fik = fikPtr(new FullbodyInverseKinematicsSolver(m_robot, std::string(m_profile.instance_name), m_dt));
    body_list.push_back(m_robot);
    body_list.push_back(m_robot_vsafe);

    setupfik(fik, m_robot, prop);

    wbms = boost::shared_ptr<WBMSCore>(new WBMSCore(m_dt));

    invdyn_zmp_filters.setParameter(25, 1/m_dt, Q_BUTTERWORTH);
    final_ref_zmp_filter.setParameter(5, 1/m_dt, Q_BUTTERWORTH);

    avg_q_vel = 1;

    sccp = boost::shared_ptr<CapsuleCollisionChecker>(new CapsuleCollisionChecker(m_robot));

    std::cerr << "[" << m_profile.instance_name << "] onInitialize() OK" << std::endl;
    return RTC::RTC_OK;
}

RTC::ReturnCode_t WholeBodyMasterSlave::onFinalize(){
    return RTC::RTC_OK;
}

RTC::ReturnCode_t WholeBodyMasterSlave::setupfik(fikPtr& fik_in, hrp::BodyPtr& robot_in, RTC::Properties& prop_in){
    coil::vstring ee_conf_all = coil::split(prop_in["end_effectors"], ",");
    size_t prop_num = 10;
    if (ee_conf_all.size() > 0) {
        size_t ee_num = ee_conf_all.size()/prop_num;
        for (size_t i = 0; i < ee_num; i++) {
            std::string ee_name, target_link_name, base_name; // e.g. rleg, RLEG_JOINT5, WAIST
            coil::stringTo(ee_name, ee_conf_all[i*prop_num].c_str());
            coil::stringTo(target_link_name, ee_conf_all[i*prop_num+1].c_str());
            coil::stringTo(base_name, ee_conf_all[i*prop_num+2].c_str());
            ee_name_ikcp_map[ee_name].target_link_name = target_link_name;
            for (size_t j = 0; j < XYZ; j++){ coil::stringTo(ee_name_ikcp_map[ee_name].localPos(j), ee_conf_all[i*prop_num+3+j].c_str()); }
            double tmp_aa[4];
            for (int j = 0; j < 4; j++ ){ coil::stringTo(tmp_aa[j], ee_conf_all[i*prop_num+6+j].c_str()); }
            ee_name_ikcp_map[ee_name].localR = Eigen::AngleAxis<double>(tmp_aa[3], hrp::Vector3(tmp_aa[0], tmp_aa[1], tmp_aa[2])).toRotationMatrix(); // rotation in VRML is represented by axis + angle
            if(robot_in->link(target_link_name)){
              RTCOUT << "End Effector [" << ee_name << "]" << std::endl;
              RTCOUT << "   target_link_name = " << ee_name_ikcp_map[ee_name].target_link_name << ", base = " << base_name << std::endl;
              RTCOUT << "   offset_pos = " << ee_name_ikcp_map[ee_name].localPos.transpose() << "[m]" << std::endl;
              RTCOUT << "   has_toe_joint = " << "fix to false now" << std::endl;
            }else{
              RTCOUT << "Target link [" << target_link_name << "] not found !" << std::endl;
              return RTC::RTC_ERROR;
            }
            contact_states_index_map.insert(std::pair<std::string, size_t>(ee_name, i));////TODO:要移動? //used for check optional data order
        }
    }
    is_legged_robot = (ee_name_ikcp_map.find("rleg") != ee_name_ikcp_map.end() && ee_name_ikcp_map.find("lleg") != ee_name_ikcp_map.end());
    return RTC::RTC_OK;
}

#define TIMECALC 0
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
    if(mode.now()!=MODE_PAUSE){
        if (m_htrfwIn.isNew()){ m_htrfwIn.read(); WBMSCore::WrenchToVector6(m_htrfw.data,wbms->hp_wld_raw.tgt[rf].w); }
        if (m_htlfwIn.isNew()){ m_htlfwIn.read(); WBMSCore::WrenchToVector6(m_htlfw.data,wbms->hp_wld_raw.tgt[lf].w); }
        if (m_htcomIn.isNew()){ m_htcomIn.read(); WBMSCore::Pose3DToWBMSPose3D(m_htcom.data,wbms->hp_wld_raw.tgt[com].abs); }
        if (m_htrfIn.isNew()) { m_htrfIn.read();  WBMSCore::Pose3DToWBMSPose3D(m_htrf.data,wbms->hp_wld_raw.tgt[rf].abs); }
        if (m_htlfIn.isNew()) { m_htlfIn.read();  WBMSCore::Pose3DToWBMSPose3D(m_htlf.data,wbms->hp_wld_raw.tgt[lf].abs); }
        if (m_htrhIn.isNew()) { m_htrhIn.read();  WBMSCore::Pose3DToWBMSPose3D(m_htrh.data,wbms->hp_wld_raw.tgt[rh].abs);}
        if (m_htlhIn.isNew()) { m_htlhIn.read();  WBMSCore::Pose3DToWBMSPose3D(m_htlh.data,wbms->hp_wld_raw.tgt[lh].abs);}
        if (m_htheadIn.isNew()){ m_htheadIn.read(); WBMSCore::Pose3DToWBMSPose3D(m_hthead.data,wbms->hp_wld_raw.tgt[head].abs);}
        if (m_htzmpIn.isNew()){ m_htzmpIn.read();  WBMSCore::Point3DToVector3(m_htzmp.data,wbms->hp_wld_raw.tgt[zmp].abs.p); }
    }

    //khi
    if(m_htlfw.data.force.y == 1 && m_htrfw.data.force.y == 1 && mode.now() == MODE_IDLE){
        std::cerr<<"button call"<<std::endl;
        startWholeBodyMasterSlave();
        std::cerr<<"button call end"<<std::endl;
    }else if(m_htlfw.data.force.y == 1 && m_htrfw.data.force.y == 1 && mode.now() == MODE_WBMS){
        stopWholeBodyMasterSlave();
    }
    
    if ( is_legged_robot ) {
        processTransition();
        mode.update();
//        if(DEBUGP)dbg(mode.now());
        struct timespec startT, endT;
        clock_gettime(CLOCK_REALTIME, &startT);

        if(DEBUGP && TIMECALC){clock_gettime(CLOCK_REALTIME, &endT); std::cout << (double)(endT.tv_sec - startT.tv_sec + (endT.tv_nsec - startT.tv_nsec) * 1e-9) << " @ processTransition" << std::endl;  clock_gettime(CLOCK_REALTIME, &startT);}

        if (mode.isRunning()) {
            if(mode.isInitialize()){
                preProcessForWholeBodyMasterSlave(fik, m_robot);
                idsb.setInitState(m_robot, m_dt);//逆動力学初期化
            }

            processWholeBodyMasterSlave(fik, m_robot, wbms->rp_ref_out);//安全制限つきマスタ・スレーブ

            if(DEBUGP && TIMECALC){clock_gettime(CLOCK_REALTIME, &endT); std::cout << (double)(endT.tv_sec - startT.tv_sec + (endT.tv_nsec - startT.tv_nsec) * 1e-9) << " @ processWholeBodyMasterSlave" << std::endl;  clock_gettime(CLOCK_REALTIME, &startT);}

            //逆動力学
            //        calcAccelerationsForInverseDynamics(m_robot, idsb);
            //        hrp::Vector3 ref_zmp_invdyn;
            //        calcWorldZMPFromInverseDynamics(m_robot, idsb, ref_zmp_invdyn);
            //        ref_zmp_invdyn = invdyn_zmp_filters.passFilter(ref_zmp_invdyn);
            //        updateInvDynStateBuffer(idsb);

            if(DEBUGP && TIMECALC){clock_gettime(CLOCK_REALTIME, &endT); std::cout << (double)(endT.tv_sec - startT.tv_sec + (endT.tv_nsec - startT.tv_nsec) * 1e-9) << " @ calcWorldZMPFromInverseDynamics" << std::endl;  clock_gettime(CLOCK_REALTIME, &startT);}

            processHOFFARBIBFilter(m_robot, m_robot_vsafe);

            hrp::Vector3 com = m_robot_vsafe->calcCM();
            static hrp::Vector3 com_old = com;
            static hrp::Vector3 com_old_old = com_old;
            hrp::Vector3 com_acc = (com - 2*com_old + com_old_old)/(m_dt*m_dt);
            hrp::Vector3 ref_zmp; ref_zmp << com.head(XY)-(com(Z)/G)*com_acc.head(XY), 0;
            if(mode.isInitialize()){ final_ref_zmp_filter.reset(ref_zmp); }
            ref_zmp = final_ref_zmp_filter.passFilter(ref_zmp);
            com_old_old = com_old;
            com_old = com;
            wbms->act_rs.com = com;
            wbms->act_rs.zmp = ref_zmp;

            const std::string tmp[] = {"R_CROTCH_R","R_CROTCH_P","R_CROTCH_Y","R_KNEE_P","R_ANKLE_R","R_ANKLE_P","L_CROTCH_R","L_CROTCH_P","L_CROTCH_Y","L_KNEE_P","L_ANKLE_R","L_ANKLE_P",};
            const std::vector<std::string> lower(tmp, tmp+12);
            // qRef
            for (int i = 0; i < m_qRef.data.length(); i++ ){
              if(!wbms->wp.use_lower && std::find(lower.begin(), lower.end(), m_robot->joint(i)->name) != lower.end()){}// pass through lower limbs
              else{
                m_qRef.data[i] = transition_interpolator_ratio * m_robot_vsafe->joint(i)->q  + (1 - transition_interpolator_ratio) * m_qRef.data[i];
              }
            }
            // basePos
            m_basePos.data.x = transition_interpolator_ratio * m_robot_vsafe->rootLink()->p(0) + (1 - transition_interpolator_ratio) * m_basePos.data.x;
            m_basePos.data.y = transition_interpolator_ratio * m_robot_vsafe->rootLink()->p(1) + (1 - transition_interpolator_ratio) * m_basePos.data.y;
            m_basePos.data.z = transition_interpolator_ratio * m_robot_vsafe->rootLink()->p(2) + (1 - transition_interpolator_ratio) * m_basePos.data.z;
            m_basePos.tm = m_qRef.tm;
            // baseRpy
            hrp::Vector3 baseRpy = hrp::rpyFromRot(m_robot_vsafe->rootLink()->R);
            m_baseRpy.data.r = transition_interpolator_ratio * baseRpy(0) + (1 - transition_interpolator_ratio) * m_baseRpy.data.r;
            m_baseRpy.data.p = transition_interpolator_ratio * baseRpy(1) + (1 - transition_interpolator_ratio) * m_baseRpy.data.p;
            m_baseRpy.data.y = transition_interpolator_ratio * baseRpy(2) + (1 - transition_interpolator_ratio) * m_baseRpy.data.y;
            m_baseRpy.tm = m_qRef.tm;
            // zmp
            hrp::Vector3 rel_ref_zmp = m_robot_vsafe->rootLink()->R.transpose() * (ref_zmp - m_robot_vsafe->rootLink()->p);
            m_zmp.data.x = transition_interpolator_ratio * rel_ref_zmp(0) + (1 - transition_interpolator_ratio) * m_zmp.data.x;
            m_zmp.data.y = transition_interpolator_ratio * rel_ref_zmp(1) + (1 - transition_interpolator_ratio) * m_zmp.data.y;
            m_zmp.data.z = transition_interpolator_ratio * rel_ref_zmp(2) + (1 - transition_interpolator_ratio) * m_zmp.data.z;
            m_zmp.tm = m_qRef.tm;
            // m_optionalData
            if(m_optionalData.data.length() < optionalDataLength){
                m_optionalData.data.length(optionalDataLength);//TODO:これいいのか？
                for(int i=0;i<optionalDataLength;i++)m_optionalData.data[i] = 0;
            }
            m_optionalData.data[contact_states_index_map["rleg"]] = m_optionalData.data[optionalDataLength/2 + contact_states_index_map["rleg"]] = wbms->rp_ref_out.tgt[rf].is_contact;
            m_optionalData.data[contact_states_index_map["lleg"]] = m_optionalData.data[optionalDataLength/2 + contact_states_index_map["lleg"]] = wbms->rp_ref_out.tgt[lf].is_contact;
        }
        wbms->baselinkpose.p = m_robot->rootLink()->p;
        wbms->baselinkpose.rpy = hrp::rpyFromRot(m_robot->rootLink()->R);
    }

    // write
    m_qOut.write();
    m_basePosOut.write();
    m_baseRpyOut.write();
    m_zmpOut.write();
    m_optionalDataOut.write();

#ifdef USE_DEBUG_PORT
    // dbg plot
    m_htcom_dbg.tm = m_qRef.tm;
    WBMSCore::WBMSPose3DToPose3D(wbms->hp_plot.tgt[com].abs, m_htcom_dbg.data);
    m_htcom_dbgOut.write();
    m_htrf_dbg.tm = m_qRef.tm;
    WBMSCore::WBMSPose3DToPose3D(wbms->hp_plot.tgt[rf].abs, m_htrf_dbg.data);
    m_htrf_dbgOut.write();
    m_htlf_dbg.tm = m_qRef.tm;
    WBMSCore::WBMSPose3DToPose3D(wbms->hp_plot.tgt[lf].abs, m_htlf_dbg.data);
    m_htlf_dbgOut.write();
    m_htrh_dbg.tm = m_qRef.tm;
    WBMSCore::WBMSPose3DToPose3D(wbms->hp_plot.tgt[rh].abs, m_htrh_dbg.data);
    m_htrh_dbgOut.write();
    m_htlh_dbg.tm = m_qRef.tm;
    WBMSCore::WBMSPose3DToPose3D(wbms->hp_plot.tgt[lh].abs, m_htlh_dbg.data);
    m_htlh_dbgOut.write();
    m_hthead_dbg.tm = m_qRef.tm;
    WBMSCore::WBMSPose3DToPose3D(wbms->hp_plot.tgt[head].abs, m_hthead_dbg.data);
    m_hthead_dbgOut.write();
    m_htzmp_dbg.tm = m_qRef.tm;
    WBMSCore::Vector3ToPoint3D(wbms->rp_ref_out.tgt[zmp].abs.p,m_rpzmp_dbg.data);
    m_htzmp_dbgOut.write();
    m_htrfw_dbg.tm = m_qRef.tm;
    m_htrfw_dbg.data.length(6);
    WBMSCore::Vector6ToDoubleSeq(wbms->hp_plot.tgt[rf].w, m_htrfw_dbg.data);
    m_htrfw_dbgOut.write();
    m_htlfw_dbg.tm = m_qRef.tm;
    m_htlfw_dbg.data.length(6);
    WBMSCore::Vector6ToDoubleSeq(wbms->hp_plot.tgt[lf].w, m_htlfw_dbg.data);
    m_htlfw_dbgOut.write();
    m_rpcom_dbg.tm = m_qRef.tm;
    WBMSCore::WBMSPose3DToPose3D(wbms->rp_ref_out.tgt[com].abs, m_rpcom_dbg.data);
    m_rpcom_dbgOut.write();
    m_rprf_dbg.tm = m_qRef.tm;
    WBMSCore::WBMSPose3DToPose3D(wbms->rp_ref_out.tgt[rf].abs, m_rprf_dbg.data);
    m_rprf_dbgOut.write();
    m_rplf_dbg.tm = m_qRef.tm;
    WBMSCore::WBMSPose3DToPose3D(wbms->rp_ref_out.tgt[lf].abs, m_rplf_dbg.data);
    m_rplf_dbgOut.write();
    m_rprh_dbg.tm = m_qRef.tm;
    WBMSCore::WBMSPose3DToPose3D(wbms->rp_ref_out.tgt[rh].abs, m_rprh_dbg.data);
    m_rprh_dbgOut.write();
    m_rplh_dbg.tm = m_qRef.tm;
    WBMSCore::WBMSPose3DToPose3D(wbms->rp_ref_out.tgt[lh].abs, m_rplh_dbg.data);
    m_rplh_dbgOut.write();
    m_rphead_dbg.tm = m_qRef.tm;
    WBMSCore::WBMSPose3DToPose3D(wbms->rp_ref_out.tgt[head].abs, m_rphead_dbg.data);
    m_rphead_dbgOut.write();
    m_rpzmp_dbg.tm = m_qRef.tm;
    WBMSCore::Vector3ToPoint3D(wbms->rp_ref_out.tgt[zmp].abs.p, m_rpzmp_dbg.data);
    m_rpzmp_dbgOut.write();
    m_rpdcp_dbg.tm = m_qRef.tm;
    WBMSCore::Vector3ToPoint3D(wbms->cp_dec,m_rpdcp_dbg.data);
    m_rpdcp_dbgOut.write();
    m_rpacp_dbg.tm = m_qRef.tm;
    WBMSCore::Vector3ToPoint3D(wbms->cp_acc,m_rpacp_dbg.data);
    m_rpacp_dbgOut.write();
    m_invdyn_dbg.tm = m_qRef.tm;
    m_invdyn_dbg.data.length(6);
    WBMSCore::Vector6ToDoubleSeq(wbms->invdyn_ft,m_invdyn_dbg.data);
    m_invdyn_dbgOut.write();
#endif
if(TIMECALC){gettimeofday(&t_calc_end, NULL); if(DEBUGP)cout<<"t_last:"<<(double)(t_calc_end.tv_sec - t_calc_start.tv_sec) + (t_calc_end.tv_usec - t_calc_start.tv_usec)/1.0e6<<endl; t_calc_start = t_calc_end;}
loop ++;
return RTC::RTC_OK;
}


void WholeBodyMasterSlave::processTransition(){
    switch(mode.now()){

        case MODE_SYNC_TO_WBMS:
            if(mode.pre() == MODE_IDLE){ double tmp = 1.0; transition_interpolator->setGoal(&tmp, 3.0, true); }
            if (!transition_interpolator->isEmpty() ){
                transition_interpolator->get(&transition_interpolator_ratio, true);
            }else{
                mode.setModeRequest(MODE_WBMS);
            }
            break;

        case MODE_SYNC_TO_IDLE:
            if(mode.pre() == MODE_WBMS || mode.pre() == MODE_PAUSE){ double tmp = 0.0; transition_interpolator->setGoal(&tmp, 3.0, true); }
            if (!transition_interpolator->isEmpty()) {
                transition_interpolator->get(&transition_interpolator_ratio, true);
            }else{
                mode.setModeRequest(MODE_IDLE);
            }
            break;
    }
}


void WholeBodyMasterSlave::preProcessForWholeBodyMasterSlave(fikPtr& fik_in, hrp::BodyPtr& robot_in){
    hrp::Vector3 basePos_heightChecked = hrp::Vector3(m_basePos.data.x, m_basePos.data.y, m_basePos.data.z);//ベースリンク高さ調整により足裏高さ0に
    robot_in->rootLink()->p = basePos_heightChecked;
    for ( int i = 0; i < robot_in->numJoints(); i++ ){ robot_in->joint(i)->q = m_qRef.data[i]; }
    robot_in->calcForwardKinematics();

    //TODO
//    hrp::Vector3 init_foot_mid_coord = (fik_in->getEndEffectorPos("rleg") + fik_in->getEndEffectorPos("lleg")) / 2;
//    if( fabs((double)init_foot_mid_coord(Z)) > 1e-5 ){
//        basePos_heightChecked(Z) -= init_foot_mid_coord(Z);
//        init_foot_mid_coord(Z) = 0;
//        std::cerr<<"["<<m_profile.instance_name<<"] Input basePos height is invalid. Auto modify "<<m_basePos.data.z<<" -> "<<basePos_heightChecked(Z)<<endl;
//    }

    const std::string robot_l_names[4] = {"rleg","lleg","rarm","larm"};

    for(int i=0;i<body_list.size();i++){//初期姿勢でBodyをFK
        body_list[i]->rootLink()->p = basePos_heightChecked;
        body_list[i]->rootLink()->R = hrp::rotFromRpy(m_baseRpy.data.r, m_baseRpy.data.p, m_baseRpy.data.y);
        for ( int j = 0; j < body_list[i]->numJoints(); j++ ){ body_list[i]->joint(j)->q = m_qRef.data[j]; }
        body_list[i]->calcForwardKinematics();
    }

    fik->q_ref << hrp::getQAll(robot_in), robot_in->rootLink()->p, hrp::rpyFromRot(robot_in->rootLink()->R);    //これいらない？


//    fik->setReferenceJointAngles();
//    for(int l=0;l<4;l++){//targetを初期化
//        if(fik_list[i]->ikp.count(robot_l_names[l])){
//            fik_list[i]->ikp[robot_l_names[l]].target_p0 = fik_list[i]->getEndEffectorPos(robot_l_names[l]);
//            fik_list[i]->ikp[robot_l_names[l]].target_r0 = fik_list[i]->getEndEffectorRot(robot_l_names[l]);
//        }
//    }
//    q_ip->clear();//clearは2回目以降しないほうがいい
    init_sync_state.resize(ROBOT_ALL_DOF);
    init_sync_state << hrp::getQAll(robot_in), robot_in->rootLink()->p, hrp::rpyFromRot(robot_in->rootLink()->R);
    q_ip->set(init_sync_state.data());
    wbms->initializeRequest(robot_in, ee_name_ikcp_map);
}


void WholeBodyMasterSlave::processWholeBodyMasterSlave(fikPtr& fik_in, hrp::BodyPtr& robot_in, const HumanPose& pose_ref){
    wbms->update();//////HumanSynchronizerの主要処理
    if(DEBUGP)cout<<"update():"<<wbms->getUpdateTime()<<endl;
    if(DEBUGP)pose_ref.print();
    solveFullbodyIK(fik_in, robot_in, pose_ref.tgt[com].abs, pose_ref.tgt[rf].abs, pose_ref.tgt[lf].abs, pose_ref.tgt[rh].abs, pose_ref.tgt[lh].abs, pose_ref.tgt[head].abs,"processWholeBodyMasterSlave");
}


void WholeBodyMasterSlave::solveFullbodyIK(fikPtr& fik_in, hrp::BodyPtr& robot_in, const WBMSPose3D& com_ref, const WBMSPose3D& rf_ref, const WBMSPose3D& lf_ref, const WBMSPose3D& rh_ref, const WBMSPose3D& lh_ref, const WBMSPose3D& head_ref, const std::string& debug_prefix){
    std::vector<IKConstraint> ikc_list;
    if(wbms->wp.use_lower){
        IKConstraint tmp;
        tmp.target_link_name = "WAIST";
        tmp.localPos = hrp::Vector3::Zero();
        tmp.localR = hrp::Matrix33::Identity();
//        tmp.targetPos = robot_in->rootLink()->p;// will be ignored by selection_vec
//        tmp.targetRpy = com_ref.rpy;// ベースリンクの回転をフリーにはしないほうがいい(omegaの積分誤差で暴れる)
//        tmp.constraint_weight << 0,0,0,1,1,1;
        tmp.targetPos << m_basePos.data.x,m_basePos.data.y,m_basePos.data.z;// will be ignored by selection_vec
        tmp.targetRpy << m_baseRpy.data.r,m_baseRpy.data.p,m_baseRpy.data.y;// ベースリンクの回転をフリーにはしないほうがいい(omegaの積分誤差で暴れる)
        tmp.constraint_weight << 0,0,0,1,1,1;
        tmp.rot_precision = deg2rad(3);
        ikc_list.push_back(tmp);
    }
    if(!wbms->wp.use_lower){
      IKConstraint tmp;
      tmp.target_link_name = "WAIST";
      tmp.localPos = hrp::Vector3::Zero();
      tmp.localR = hrp::Matrix33::Identity();
      tmp.targetPos << m_basePos.data.x, m_basePos.data.y, m_basePos.data.z;// will be ignored by selection_vec
      tmp.targetRpy << m_baseRpy.data.r, m_baseRpy.data.p, m_baseRpy.data.y;// ベースリンクの回転をフリーにはしないほうがいい(omegaの積分誤差で暴れる)
      tmp.constraint_weight << 1,1,1,1,1,1;
      tmp.rot_precision = deg2rad(3);
      ikc_list.push_back(tmp);
    }
    if(wbms->wp.use_lower){
        IKConstraint tmp;
        tmp.target_link_name = ee_name_ikcp_map["rleg"].target_link_name;
        tmp.localPos = ee_name_ikcp_map["rleg"].localPos;
        tmp.localR = ee_name_ikcp_map["rleg"].localR;
        tmp.targetPos = rf_ref.p;
        tmp.targetRpy = rf_ref.rpy;
        if(wbms->rp_ref_out.tgt[rf].is_contact){
            tmp.constraint_weight = hrp::dvector6::Constant(10);
        }else{
            tmp.constraint_weight = hrp::dvector6::Constant(0.1);
        }
        ikc_list.push_back(tmp);
    }
    if(wbms->wp.use_lower){
        IKConstraint tmp;
        tmp.target_link_name = ee_name_ikcp_map["lleg"].target_link_name;
        tmp.localPos = ee_name_ikcp_map["lleg"].localPos;
        tmp.localR = ee_name_ikcp_map["lleg"].localR;
        tmp.targetPos = lf_ref.p;
        tmp.targetRpy = lf_ref.rpy;
        if(wbms->rp_ref_out.tgt[lf].is_contact){
            tmp.constraint_weight = hrp::dvector6::Constant(10);
        }else{
            tmp.constraint_weight = hrp::dvector6::Constant(0.1);
        }
        ikc_list.push_back(tmp);
    }
    const double dist = 0.0;
    {
        IKConstraint tmp;
        tmp.target_link_name = ee_name_ikcp_map["rarm"].target_link_name;
        tmp.localPos = ee_name_ikcp_map["rarm"].localPos;
        tmp.localR = ee_name_ikcp_map["rarm"].localR;
        if((rh_ref.p-lh_ref.p).norm()<dist){
            tmp.targetPos = (rh_ref.p+lh_ref.p)/2 + (rh_ref.p-lh_ref.p).normalized()*dist/2;
        }else{
            tmp.targetPos = rh_ref.p;
        }
        tmp.targetRpy = rh_ref.rpy;
        tmp.constraint_weight = hrp::dvector6::Constant(0.5);
        tmp.pos_precision = 3e-3;
        tmp.rot_precision = deg2rad(3);
        ikc_list.push_back(tmp);
    }{
        IKConstraint tmp;
        tmp.target_link_name = ee_name_ikcp_map["larm"].target_link_name;
        tmp.localPos = ee_name_ikcp_map["larm"].localPos;
        tmp.localR = ee_name_ikcp_map["larm"].localR;
        if((rh_ref.p-lh_ref.p).norm()<dist){
            tmp.targetPos = (rh_ref.p+lh_ref.p)/2 + (lh_ref.p-rh_ref.p).normalized()*dist/2;
        }else{
            tmp.targetPos = lh_ref.p;
        }
        tmp.targetRpy = lh_ref.rpy;
        tmp.constraint_weight = hrp::dvector6::Constant(0.5);
        tmp.pos_precision = 3e-3;
        tmp.rot_precision = deg2rad(3);
        ikc_list.push_back(tmp);
    }if(robot_in->link("HEAD_JOINT1") != NULL){
        IKConstraint tmp;
        tmp.target_link_name = "HEAD_JOINT1";
        tmp.targetRpy = head_ref.rpy;
        tmp.constraint_weight << 0,0,0,0,0.1,0.1;
        tmp.rot_precision = deg2rad(1);
        ikc_list.push_back(tmp);
    }
    // if(robot_in->link("HEAD_P") != NULL){
    //     IKConstraint tmp;
    //     tmp.target_link_name = "HEAD_P";
    //     tmp.targetRpy = head_ref.rpy;
    //     tmp.constraint_weight << 0,0,0,0,0.1,0.1;
    //     tmp.rot_precision = deg2rad(1);
    //     ikc_list.push_back(tmp);
    // }
    if(wbms->rp_ref_out.tgt[rf].is_contact){
        sccp->avoid_priority.head(12).head(6).fill(4);
    }else{
        sccp->avoid_priority.head(12).head(6).fill(3);
    }
    if(wbms->rp_ref_out.tgt[lf].is_contact){
        sccp->avoid_priority.head(12).tail(6).fill(4);
    }else{
        sccp->avoid_priority.head(12).tail(6).fill(3);
    }

//    sccp->checkCollision();

    for(int i=0;i<sccp->collision_info_list.size();i++){
        IKConstraint tmp;
        double val_w = (sccp->collision_info_list[i].dist_safe - sccp->collision_info_list[i].dist_cur)*1e2;
        LIMIT_MAX(val_w, 3);
        tmp.constraint_weight << val_w,val_w,val_w,0,0,0;
//        tmp.constraint_weight << 3,3,3,0,0,0;
        double margin = 1e-3;
        if(sccp->avoid_priority(sccp->collision_info_list[i].id0) > sccp->avoid_priority(sccp->collision_info_list[i].id1)){
            tmp.localPos = sccp->collision_info_list[i].cp1_local;
            tmp.target_link_name = m_robot->joint(sccp->collision_info_list[i].id1)->name;
            tmp.targetPos = sccp->collision_info_list[i].cp0_wld + (sccp->collision_info_list[i].cp1_wld - sccp->collision_info_list[i].cp0_wld).normalized() * (sccp->collision_info_list[i].dist_safe + margin);
            ikc_list.push_back(tmp);
        }else if(sccp->avoid_priority(sccp->collision_info_list[i].id0) < sccp->avoid_priority(sccp->collision_info_list[i].id1)){
            tmp.localPos = sccp->collision_info_list[i].cp0_local;
            tmp.target_link_name = m_robot->joint(sccp->collision_info_list[i].id0)->name;
            tmp.targetPos = sccp->collision_info_list[i].cp1_wld + (sccp->collision_info_list[i].cp0_wld - sccp->collision_info_list[i].cp1_wld).normalized() * (sccp->collision_info_list[i].dist_safe + margin);
            ikc_list.push_back(tmp);
        }else{
            tmp.localPos = sccp->collision_info_list[i].cp1_local;
            tmp.target_link_name = m_robot->joint(sccp->collision_info_list[i].id1)->name;
            tmp.targetPos = sccp->collision_info_list[i].cp0_wld + (sccp->collision_info_list[i].cp1_wld - sccp->collision_info_list[i].cp0_wld).normalized() * (sccp->collision_info_list[i].dist_safe + margin);
            ikc_list.push_back(tmp);
            tmp.localPos = sccp->collision_info_list[i].cp0_local;
            tmp.target_link_name = m_robot->joint(sccp->collision_info_list[i].id0)->name;
            tmp.targetPos = sccp->collision_info_list[i].cp1_wld + (sccp->collision_info_list[i].cp0_wld - sccp->collision_info_list[i].cp1_wld).normalized() * (sccp->collision_info_list[i].dist_safe + margin);
            ikc_list.push_back(tmp);
        }
//        ikc_list[3].constraint_weight =  hrp::dvector6::Constant(1e-4);
//        ikc_list[4].constraint_weight =  hrp::dvector6::Constant(1e-4);
    }

    if(loop%20==0){
        if(sccp->collision_info_list.size()>0){
            std::cout<<"pair:"<<std::endl;
            for(int i=0;i<sccp->collision_info_list.size();i++){
                std::cout<<m_robot->joint(sccp->collision_info_list[i].id0)->name<<" "<<m_robot->joint(sccp->collision_info_list[i].id1)->name<<endl;
            }
        }
    }


    if(wbms->wp.use_lower){
        IKConstraint tmp;
        tmp.target_link_name = "COM";
        tmp.localPos = hrp::Vector3::Zero();
        tmp.localR = hrp::Matrix33::Identity();
        tmp.targetPos = com_ref.p;// COM height will not be constraint
        tmp.targetRpy = hrp::Vector3::Zero();//reference angular momentum
//        tmp.constraint_weight << 10,10,1,1e-6,1e-6,1e-6;
        tmp.constraint_weight << 10,10,0.001,0,0,0;
//        if(fik_in->cur_momentum_around_COM.norm() > 1e9){
//            tmp.constraint_weight << 10,10,1,1e-5,1e-5,1e-10;
//            tmp.rot_precision = 100;//angular momentum precision
//        }else{
//            tmp.constraint_weight << 10,10,1,0,0,0;
//        }
        ikc_list.push_back(tmp);
    }

    if( robot_in->link("CHEST_JOINT0") != NULL) fik_in->dq_weight_all(robot_in->link("CHEST_JOINT0")->jointId) = 0.1;//JAXON
    if( robot_in->link("CHEST_JOINT1") != NULL) fik_in->dq_weight_all(robot_in->link("CHEST_JOINT1")->jointId) = 0.1;
    if( robot_in->link("CHEST_JOINT2") != NULL) fik_in->dq_weight_all(robot_in->link("CHEST_JOINT2")->jointId) = 0.1;
    if( robot_in->link("CHEST_Y") != NULL) fik_in->dq_weight_all(robot_in->link("CHEST_Y")->jointId) = 0.1;//K
    if( robot_in->link("CHEST_P") != NULL) fik_in->dq_weight_all(robot_in->link("CHEST_P")->jointId) = 0.1;
//    if( robot_in->link("RARM_JOINT2") != NULL) robot_in->link("RARM_JOINT2")->ulimit = deg2rad(-30);//脇内側の干渉回避
//    if( robot_in->link("LARM_JOINT2") != NULL) robot_in->link("LARM_JOINT2")->llimit = deg2rad(30);
    if( m_robot->link("RLEG_JOINT3") != NULL) m_robot->link("RLEG_JOINT3")->llimit = deg2rad(10);//膝伸びきり防止のため
    if( m_robot->link("LLEG_JOINT3") != NULL) m_robot->link("LLEG_JOINT3")->llimit = deg2rad(10);
    if( m_robot->link("R_KNEE_P") != NULL) m_robot->link("R_KNEE_P")->llimit = deg2rad(15);//K
    if( m_robot->link("L_KNEE_P") != NULL) m_robot->link("L_KNEE_P")->llimit = deg2rad(15);
    if( m_robot->link("R_WRIST_R") != NULL) m_robot->link("R_WRIST_R")->llimit = deg2rad(-40);
    if( m_robot->link("L_WRIST_R") != NULL) m_robot->link("L_WRIST_R")->llimit = deg2rad(-40);
    if( m_robot->link("R_WRIST_R") != NULL) m_robot->link("R_WRIST_R")->ulimit = deg2rad(40);
    if( m_robot->link("L_WRIST_R") != NULL) m_robot->link("L_WRIST_R")->ulimit = deg2rad(40);
    if( m_robot->link("R_WRIST_P") != NULL) m_robot->link("R_WRIST_P")->llimit = deg2rad(-40);
    if( m_robot->link("L_WRIST_P") != NULL) m_robot->link("L_WRIST_P")->llimit = deg2rad(-40);
    if( m_robot->link("R_WRIST_P") != NULL) m_robot->link("R_WRIST_P")->ulimit = deg2rad(20);
    if( m_robot->link("L_WRIST_P") != NULL) m_robot->link("L_WRIST_P")->ulimit = deg2rad(20);


    if( m_robot->link("CHEST_Y") != NULL) m_robot->link("CHEST_Y")->llimit = deg2rad(-20);
    if( m_robot->link("CHEST_Y") != NULL) m_robot->link("CHEST_Y")->ulimit = deg2rad(20);
    if( m_robot->link("CHEST_P") != NULL) m_robot->link("CHEST_P")->llimit = deg2rad(0);
    if( m_robot->link("CHEST_P") != NULL) m_robot->link("CHEST_P")->ulimit = deg2rad(60);
                                              
    if( m_robot->link("HEAD_Y") != NULL) m_robot->link("HEAD_Y")->llimit = deg2rad(-5);
    if( m_robot->link("HEAD_Y") != NULL) m_robot->link("HEAD_Y")->ulimit = deg2rad(5);
    if( m_robot->link("HEAD_P") != NULL) m_robot->link("HEAD_P")->llimit = deg2rad(0);
    if( m_robot->link("HEAD_P") != NULL) m_robot->link("HEAD_P")->ulimit = deg2rad(60);

    for(int i=0;i<robot_in->numJoints();i++){
        LIMIT_MINMAX(robot_in->joint(i)->q, robot_in->joint(i)->llimit, robot_in->joint(i)->ulimit);
    }

    fik_in->q_ref = init_sync_state;


    // std::string tmp[] = {"R_CROTCH_R","R_CROTCH_P","R_CROTCH_Y","R_KNEE_P","R_ANKLE_R","R_ANKLE_P","L_CROTCH_R","L_CROTCH_P","L_CROTCH_Y","L_KNEE_P","L_ANKLE_R","L_ANKLE_P"};
    // const std::vector<std::string> list(tmp, tmp+12);
    // for(int i=0; i<list.size(); i++){
    //   if( robot_in->link(list[i]) != NULL){
    //     int j=robot_in->link(list[i])->jointId;
    //     fik_in->dq_weight_all(j) = 1e-5;
    //     robot_in->joint(j)->q = m_qRef.data[j];
    //     fik_in->q_ref(j) = m_qRef.data[j];
    //   }
    // }
    // robot_in->rootLink()->p << m_basePos.data.x,m_basePos.data.y,m_basePos.data.z;// will be ignored by selection_vec
    // robot_in->rootLink()->R = hrp::rotFromRpy(hrp::Vector3(m_baseRpy.data.r,m_baseRpy.data.p,m_baseRpy.data.y));// ベースリンクの回転をフリーにはしないほうがいい(omegaの積分誤差で暴れる)


//    fik_in->q_ref_pullback_gain.segment(6+6+3+2, 8*2).fill(0.01);//腕だけ
//    fik_in->dq_ref_pullback.segment(6+6+3+2, 8*2).fill(deg2rad(WBMSparam0.1));//腕だけ
//    fik_in->q_ref_constraint_weight.fill(1);

    struct timespec startT, endT;
    const int IK_MAX_LOOP = 2;
    clock_gettime(CLOCK_REALTIME, &startT);
    int loop_result = fik_in->solveFullbodyIKLoop(robot_in, ikc_list, IK_MAX_LOOP);
    if(loop%100==0){clock_gettime(CLOCK_REALTIME, &endT); std::cout << (endT.tv_sec - startT.tv_sec + (endT.tv_nsec - startT.tv_nsec) * 1e-9) << " @ solveIK"<<loop_result<<"loop" << std::endl;}
}

void WholeBodyMasterSlave::processHOFFARBIBFilter(hrp::BodyPtr& robot_in, hrp::BodyPtr& robot_out){
    hrp::dvector goal_state(ROBOT_ALL_DOF);
    for(int i=0;i<robot_in->numJoints();i++){ goal_state(i) = robot_in->joint(i)->q; }
    goal_state.bottomRows(6).topRows(3) = robot_in->rootLink()->p;
    goal_state.bottomRows(6).bottomRows(3) = hrp::rpyFromRot(robot_in->rootLink()->R);

    double goal_time = 0.0;
//    const double min_goal_time_offset = 0.01;
    const double min_goal_time_offset = 0.1;
    //  const double avg_q_vel = 0.5;
    //  const double avg_q_vel = 1.0;
//    const double avg_q_vel = 1.5;

    static hrp::dvector tmp_v = hrp::dvector::Zero(ROBOT_ALL_DOF);
//    if (!q_ip->isEmpty() ){  q_ip->get(tmp_x, tmp_v, false);}
    for(int i=0;i<robot_in->numJoints();i++){
        if(robot_in->joint(i)->name != "L_HAND" && robot_in->joint(i)->name != "R_HAND" ){ //KHI demo
          double tmp_time = fabs(robot_in->joint(i)->q - robot_out->joint(i)->q) / avg_q_vel;
          LIMIT_MIN(tmp_time, fabs(tmp_v(i))/16);
  //        tmp_time += fabs(tmp_v(i))/16; //加速度8
          if(tmp_time > goal_time){ goal_time = tmp_time; }
        }
    }
    q_ip->setGoal(goal_state.data(), goal_time + min_goal_time_offset, true);
    hrp::dvector ans_state(ROBOT_ALL_DOF);
    double tmp[ROBOT_ALL_DOF];
//    if (!q_ip->isEmpty() ){  q_ip->get(tmp, true);}
    double tmpv[ROBOT_ALL_DOF];
    if (!q_ip->isEmpty() ){  q_ip->get(tmp, tmpv, true);}
    tmp_v = Eigen::Map<hrp::dvector>(tmpv, ROBOT_ALL_DOF);
    ans_state = Eigen::Map<hrp::dvector>(tmp, ROBOT_ALL_DOF);
    for(int i=0;i<robot_out->numJoints();i++){
      if(robot_in->joint(i)->name != "L_HAND" && robot_in->joint(i)->name != "R_HAND" ){ //KHI demo
        robot_out->joint(i)->q = ans_state(i);
      }
    }



    //// KHI demo
    const double hand_max_vel = 180.0/0.4 /180.0*M_PI;// 0.4sec for 180deg
    std::map<std::string, double> joy_inputs;// 0 ~ 1
    joy_inputs["L_HAND"] = m_htlfw.data.force.x;// tmp substitute
    joy_inputs["R_HAND"] = m_htrfw.data.force.x;// tmp substitute
    for(std::map<std::string, double>::iterator it = joy_inputs.begin(); it!=joy_inputs.end(); it++){
      if(robot_out->link(it->first) != NULL){
          LIMIT_MINMAX(it->second, 0.0, 1.0);
          double tgt_hand_q = (-29.0 + (124.0-(-29.0))*it->second ) /180.0*M_PI;
          const double q_old = robot_out->link(it->first)->q;
          LIMIT_MINMAX(tgt_hand_q, q_old - hand_max_vel*m_dt, q_old + hand_max_vel*m_dt);
          robot_out->link(it->first)->q = tgt_hand_q;
      }
    }

    robot_out->rootLink()->p = ans_state.bottomRows(6).topRows(3);
    robot_out->rootLink()->R = hrp::rotFromRpy(ans_state.bottomRows(6).bottomRows(3));
    robot_out->calcForwardKinematics();
}


bool WholeBodyMasterSlave::startWholeBodyMasterSlave(){
    if(mode.now() == MODE_IDLE){
        std::cerr << "[" << m_profile.instance_name << "] startWholeBodyMasterSlave" << std::endl;
        mode.setModeRequest(MODE_SYNC_TO_WBMS);
        while(!transition_interpolator->isEmpty()){ usleep(1000); }
        return true;
    }else{
        std::cerr << "[" << m_profile.instance_name << "] Invalid context to startWholeBodyMasterSlave" << std::endl;
        return false;
    }
}


bool WholeBodyMasterSlave::pauseWholeBodyMasterSlave(){
    if(mode.now() == MODE_WBMS){
        std::cerr << "[" << m_profile.instance_name << "] pauseWholeBodyMasterSlave" << std::endl;
        mode.setModeRequest(MODE_PAUSE);
        return true;
    }else{
        std::cerr << "[" << m_profile.instance_name << "] Invalid context to pauseWholeBodyMasterSlave" << std::endl;
        return false;
    }
}


bool WholeBodyMasterSlave::resumeWholeBodyMasterSlave(){
    if(mode.now() == MODE_PAUSE){
        std::cerr << "[" << m_profile.instance_name << "] resumeWholeBodyMasterSlave" << std::endl;
        mode.setModeRequest(MODE_WBMS);
        avg_q_vel /= 5;
        sleep(5);
        avg_q_vel *= 5;
        return true;
    }else{
        std::cerr << "[" << m_profile.instance_name << "] Invalid context to resumeWholeBodyMasterSlave" << std::endl;
        return false;
    }
}


bool WholeBodyMasterSlave::stopWholeBodyMasterSlave(){
    if(mode.now() == MODE_WBMS || mode.now() == MODE_PAUSE ){
        std::cerr << "[" << m_profile.instance_name << "] stopWholeBodyMasterSlave" << std::endl;
        mode.setModeRequest(MODE_SYNC_TO_IDLE);
        while(!transition_interpolator->isEmpty()){ usleep(1000); }
        return true;
    }else{
        std::cerr << "[" << m_profile.instance_name << "] Invalid context to stopWholeBodyMasterSlave" << std::endl;
        return false;
    }
}


bool WholeBodyMasterSlave::setWholeBodyMasterSlaveParam(const OpenHRP::WholeBodyMasterSlaveService::WholeBodyMasterSlaveParam& i_param){
    std::cerr << "[" << m_profile.instance_name << "] setWholeBodyMasterSlaveParam" << std::endl;
    wbms->wp.auto_swing_foot_landing_threshold = i_param.auto_swing_foot_landing_threshold;
    wbms->wp.foot_vertical_vel_limit_coeff = i_param.foot_vertical_vel_limit_coeff;
    wbms->wp.human_com_height = i_param.human_com_height;
    wbms->wp.set_com_height_fix = i_param.set_com_height_fix;
    wbms->wp.set_com_height_fix_val = i_param.set_com_height_fix_val;
    wbms->wp.swing_foot_height_offset = i_param.swing_foot_height_offset;
    wbms->wp.swing_foot_max_height = i_param.swing_foot_max_height;
    wbms->wp.upper_body_rmc_ratio = i_param.upper_body_rmc_ratio;
    if(mode.now() == MODE_IDLE){
        wbms->wp.use_head = i_param.use_head;
        wbms->wp.use_upper = i_param.use_upper;
        wbms->wp.use_lower = i_param.use_lower;
    }else{
      RTCOUT << "use_head, use_upper, use_lower can be changed in MODE_IDLE" << std::endl;
    }
    return true;
}


bool WholeBodyMasterSlave::getWholeBodyMasterSlaveParam(OpenHRP::WholeBodyMasterSlaveService::WholeBodyMasterSlaveParam& i_param){
    std::cerr << "[" << m_profile.instance_name << "] getWholeBodyMasterSlaveParam" << std::endl;
    i_param.auto_swing_foot_landing_threshold = wbms->wp.auto_swing_foot_landing_threshold;
    i_param.foot_vertical_vel_limit_coeff = wbms->wp.foot_vertical_vel_limit_coeff;
    i_param.human_com_height = wbms->wp.human_com_height;
    i_param.set_com_height_fix = wbms->wp.set_com_height_fix;
    i_param.set_com_height_fix_val = wbms->wp.set_com_height_fix_val;
    i_param.swing_foot_height_offset = wbms->wp.swing_foot_height_offset;
    i_param.swing_foot_max_height = wbms->wp.swing_foot_max_height;
    i_param.upper_body_rmc_ratio = wbms->wp.upper_body_rmc_ratio;
    i_param.use_head = wbms->wp.use_head;
    i_param.use_upper = wbms->wp.use_upper;
    i_param.use_lower = wbms->wp.use_lower;
    return true;
}


RTC::ReturnCode_t WholeBodyMasterSlave::onActivated(RTC::UniqueId ec_id){ std::cerr << "[" << m_profile.instance_name<< "] onActivated(" << ec_id << ")" << std::endl; return RTC::RTC_OK; }
RTC::ReturnCode_t WholeBodyMasterSlave::onDeactivated(RTC::UniqueId ec_id){ std::cerr << "[" << m_profile.instance_name<< "] onDeactivated(" << ec_id << ")" << std::endl; return RTC::RTC_OK; }

extern "C"{
    void WholeBodyMasterSlaveInit(RTC::Manager* manager) {
        RTC::Properties profile(WholeBodyMasterSlave_spec);
        manager->registerFactory(profile, RTC::Create<WholeBodyMasterSlave>, RTC::Delete<WholeBodyMasterSlave>);
    }
};
