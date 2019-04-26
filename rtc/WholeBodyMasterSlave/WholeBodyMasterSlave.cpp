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
        m_debugLevel(0)
{
    m_service0.wholebodymasterslave(this);
}

WholeBodyMasterSlave::~WholeBodyMasterSlave(){}

RTC::ReturnCode_t WholeBodyMasterSlave::onInitialize(){
    RTCOUT << "onInitialize()" << std::endl;
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

    hrp::BodyPtr robot_for_ik = hrp::BodyPtr(new hrp::Body());
    if (!loadBodyFromModelLoader(robot_for_ik, prop["model"].c_str(), CosNaming::NamingContext::_duplicate(naming.getRootContext()) )){
        RTCOUT << "failed to load model[" << prop["model"] << "]" << std::endl;
        return RTC::RTC_ERROR;
    }
    m_robot_vsafe = hrp::BodyPtr(new hrp::Body(*robot_for_ik)); //copy
    RTCOUT << "setup robot model finished" << std::endl;

    fik = fikPtr(new FullbodyInverseKinematicsSolver(robot_for_ik, std::string(m_profile.instance_name), m_dt));
    setupEEIKConstraintFromConf(ee_ikc_map, robot_for_ik, prop);
    is_legged_robot = (ee_ikc_map.find("rleg") != ee_ikc_map.end() && ee_ikc_map.find("lleg") != ee_ikc_map.end());
    RTCOUT << "setup fullbody ik finished" << std::endl;

    wbms = boost::shared_ptr<WBMSCore>(new WBMSCore(m_dt));
    sccp = boost::shared_ptr<CapsuleCollisionChecker>(new CapsuleCollisionChecker(fik->m_robot));
    RTCOUT << "setup main function class finished" << std::endl;

    // allocate memory for outPorts
    m_qRef.data.length(fik->m_robot->numJoints()); // is really needed?
    coil::stringTo(optionalDataLength, prop["seq_optional_data_dim"].c_str());

    output_ratio = 0.0;
    t_ip = new interpolator(1, m_dt, interpolator::HOFFARBIB, 1);
    t_ip->clear();
    t_ip->set(&output_ratio);
    q_ip = new interpolator(fik->numStates(), m_dt, interpolator::CUBICSPLINE, 1); // or HOFFARBIB, QUINTICSPLINE
    q_ip->clear();
    avg_q_vel = hrp::dvector::Constant(fik->numStates(), 1.0); // all joint max avarage vel = 1.0 rad/s
    avg_q_acc = hrp::dvector::Constant(fik->numStates(), 16.0); // all joint max avarage acc = 16.0 rad/s^2
    avg_q_vel.tail(6).fill(std::numeric_limits<double>::max()); // no limit for base link vel
    avg_q_acc.tail(6).fill(std::numeric_limits<double>::max()); // no limit for base link acc
    RTCOUT << "setup interpolator finished" << std::endl;

    //    invdyn_zmp_filters.setParameter(25, 1/m_dt, Q_BUTTERWORTH);
    ref_zmp_filter.setParameter(5, 1/m_dt, Q_BUTTERWORTH);

    RTCOUT << "onInitialize() OK" << std::endl;
    loop = 0;
    return RTC::RTC_OK;
}

RTC::ReturnCode_t WholeBodyMasterSlave::onFinalize(){
    return RTC::RTC_OK;
}

RTC::ReturnCode_t WholeBodyMasterSlave::setupEEIKConstraintFromConf(std::map<std::string, IKConstraint>& _ee_ikc_map, hrp::BodyPtr _robot, RTC::Properties& _prop){
    coil::vstring ee_conf_all = coil::split(_prop["end_effectors"], ",");
    size_t prop_num = 10;
    if (ee_conf_all.size() > 0) {
        size_t ee_num = ee_conf_all.size()/prop_num;
        for (size_t i = 0; i < ee_num; i++) {
            std::string ee_name, target_link_name, base_name; // e.g. rleg, RLEG_JOINT5, WAIST
            coil::stringTo(ee_name, ee_conf_all[i*prop_num].c_str());
            coil::stringTo(target_link_name, ee_conf_all[i*prop_num+1].c_str());
            coil::stringTo(base_name, ee_conf_all[i*prop_num+2].c_str());
            _ee_ikc_map[ee_name].target_link_name = target_link_name;
            for (size_t j = 0; j < XYZ; j++){ coil::stringTo(_ee_ikc_map[ee_name].localPos(j), ee_conf_all[i*prop_num+3+j].c_str()); }
            double tmp_aa[4];
            for (int j = 0; j < 4; j++ ){ coil::stringTo(tmp_aa[j], ee_conf_all[i*prop_num+6+j].c_str()); }
            _ee_ikc_map[ee_name].localR = Eigen::AngleAxis<double>(tmp_aa[3], hrp::Vector3(tmp_aa[0], tmp_aa[1], tmp_aa[2])).toRotationMatrix(); // rotation in VRML is represented by axis + angle
            if(_robot->link(target_link_name)){
              RTCOUT << "End Effector [" << ee_name << "]" << std::endl;
              RTCOUT << "   target_link_name = " << _ee_ikc_map[ee_name].target_link_name << ", base = " << base_name << std::endl;
              RTCOUT << "   offset_pos = " << _ee_ikc_map[ee_name].localPos.transpose() << "[m]" << std::endl;
              RTCOUT << "   has_toe_joint = " << "fix to false now" << std::endl;
            }else{
              RTCOUT << "Target link [" << target_link_name << "] not found !" << std::endl;
              return RTC::RTC_ERROR;
            }
            contact_states_index_map.insert(std::pair<std::string, size_t>(ee_name, i));////TODO:要移動? //used for check optional data order
        }
    }
    return RTC::RTC_OK;
}

#define TIMECALC 1
RTC::ReturnCode_t WholeBodyMasterSlave::onExecute(RTC::UniqueId ec_id){
    if(DEBUGP_ONCE)RTCOUT << "onExecute(" << ec_id << ")" << std::endl;
    struct timeval t_calc_start, t_calc_end;
    if(TIMECALC)gettimeofday(&t_calc_start, NULL);
    if (m_qRefIn.isNew()) { m_qRefIn.read(); }
    if (m_basePosIn.isNew()) { m_basePosIn.read(); }
    if (m_baseRpyIn.isNew()) { m_baseRpyIn.read(); }
    if (m_zmpIn.isNew()) { m_zmpIn.read(); }
    if (m_optionalDataIn.isNew()) { m_optionalDataIn.read(); }

    if( mode.now() != MODE_PAUSE ){ // stop updating input when MODE_PAUSE
        if (m_htrfwIn.isNew()){ m_htrfwIn.read(); wbms->hp_wld_raw.tgt[rf].w = hrp::to_dvector6(m_htrfw.data); }
        if (m_htlfwIn.isNew()){ m_htlfwIn.read(); wbms->hp_wld_raw.tgt[lf].w = hrp::to_dvector6(m_htlfw.data); }
        if (m_htcomIn.isNew()){ m_htcomIn.read(); hrp::Pose3DToWBMSPose3D(m_htcom.data,wbms->hp_wld_raw.tgt[com].abs); }
        if (m_htrfIn.isNew()) { m_htrfIn.read();  hrp::Pose3DToWBMSPose3D(m_htrf.data,wbms->hp_wld_raw.tgt[rf].abs); }
        if (m_htlfIn.isNew()) { m_htlfIn.read();  hrp::Pose3DToWBMSPose3D(m_htlf.data,wbms->hp_wld_raw.tgt[lf].abs); }
        if (m_htrhIn.isNew()) { m_htrhIn.read();  hrp::Pose3DToWBMSPose3D(m_htrh.data,wbms->hp_wld_raw.tgt[rh].abs);}
        if (m_htlhIn.isNew()) { m_htlhIn.read();  hrp::Pose3DToWBMSPose3D(m_htlh.data,wbms->hp_wld_raw.tgt[lh].abs);}
        if (m_htheadIn.isNew()){ m_htheadIn.read(); hrp::Pose3DToWBMSPose3D(m_hthead.data,wbms->hp_wld_raw.tgt[head].abs);}
        if (m_htzmpIn.isNew()){ m_htzmpIn.read(); wbms->hp_wld_raw.tgt[zmp].abs.p = hrp::to_Vector3(m_htzmp.data); }
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
                preProcessForWholeBodyMasterSlave();
                idsb.setInitState(fik->m_robot, m_dt);//逆動力学初期化
            }

            wbms->update();//////HumanSynchronizerの主要処理
            if(DEBUGP)RTCOUT << "update():"<<wbms->getUpdateTime()<<std::endl;
            if(DEBUGP)wbms->rp_ref_out.print();

            solveFullbodyIK(wbms->rp_ref_out.tgt[com].abs, wbms->rp_ref_out.tgt[rf].abs, wbms->rp_ref_out.tgt[lf].abs, wbms->rp_ref_out.tgt[rh].abs, wbms->rp_ref_out.tgt[lh].abs, wbms->rp_ref_out.tgt[head].abs);

            if(DEBUGP && TIMECALC){clock_gettime(CLOCK_REALTIME, &endT); std::cout << (double)(endT.tv_sec - startT.tv_sec + (endT.tv_nsec - startT.tv_nsec) * 1e-9) << " @ processWholeBodyMasterSlave" << std::endl;  clock_gettime(CLOCK_REALTIME, &startT);}

            //逆動力学
            //        calcAccelerationsForInverseDynamics(m_robot, idsb);
            //        hrp::Vector3 ref_zmp_invdyn;
            //        calcWorldZMPFromInverseDynamics(m_robot, idsb, ref_zmp_invdyn);processTransition
            //        ref_zmp_invdyn = invdyn_zmp_filters.passFilter(ref_zmp_invdyn);
            //        updateInvDynStateBuffer(idsb);

            if(DEBUGP && TIMECALC){clock_gettime(CLOCK_REALTIME, &endT); std::cout << (double)(endT.tv_sec - startT.tv_sec + (endT.tv_nsec - startT.tv_nsec) * 1e-9) << " @ calcWorldZMPFromInverseDynamics" << std::endl;  clock_gettime(CLOCK_REALTIME, &startT);}

            processHOFFARBIBFilter(fik->m_robot, m_robot_vsafe);

            hrp::Vector3 com = m_robot_vsafe->calcCM();
            static hrp::Vector3 com_old = com;
            static hrp::Vector3 com_old_old = com_old;
            hrp::Vector3 com_acc = (com - 2*com_old + com_old_old)/(m_dt*m_dt);
            hrp::Vector3 ref_zmp; ref_zmp << com.head(XY)-(com(Z)/G)*com_acc.head(XY), 0;
            if(mode.isInitialize()){ ref_zmp_filter.reset(ref_zmp); }
            ref_zmp = ref_zmp_filter.passFilter(ref_zmp);
            com_old_old = com_old;
            com_old = com;
            wbms->act_rs.com = com;
            wbms->act_rs.zmp = ref_zmp;

            const std::string tmp[] = {"R_CROTCH_R","R_CROTCH_P","R_CROTCH_Y","R_KNEE_P","R_ANKLE_R","R_ANKLE_P","L_CROTCH_R","L_CROTCH_P","L_CROTCH_Y","L_KNEE_P","L_ANKLE_R","L_ANKLE_P",};
            const std::vector<std::string> lower(tmp, tmp+12);
            // qRef
            for (int i = 0; i < m_qRef.data.length(); i++ ){
              if(!wbms->wp.use_lower && std::find(lower.begin(), lower.end(), fik->m_robot->joint(i)->name) != lower.end()){}// pass through lower limbs
              else{
                m_qRef.data[i] = output_ratio * m_robot_vsafe->joint(i)->q  + (1 - output_ratio) * m_qRef.data[i];
              }
            }
//            m_qRef.data = hrp::to_DoubleSeq(output_ratio * hrp::getQAll(m_robot_vsafe)  + (1 - output_ratio) * hrp::to_dvector(m_qRef.data));

            // basePos
            m_basePos.data = hrp::to_Point3D( output_ratio * m_robot_vsafe->rootLink()->p + (1 - output_ratio) * hrp::to_Vector3(m_basePos.data));
            m_basePos.tm = m_qRef.tm;
            // baseRpy
            m_baseRpy.data = hrp::to_Orientation3D( output_ratio * hrp::rpyFromRot(m_robot_vsafe->rootLink()->R) + (1 - output_ratio) * hrp::to_Vector3(m_baseRpy.data));
            m_baseRpy.tm = m_qRef.tm;
            // zmp
            hrp::Vector3 rel_ref_zmp = m_robot_vsafe->rootLink()->R.transpose() * (ref_zmp - m_robot_vsafe->rootLink()->p);
            m_zmp.data = hrp::to_Point3D( output_ratio * rel_ref_zmp + (1 - output_ratio) * hrp::to_Vector3(m_zmp.data));
            m_zmp.tm = m_qRef.tm;
            // m_optionalData
            if(m_optionalData.data.length() < optionalDataLength){
                m_optionalData.data.length(optionalDataLength);//TODO:これいいのか？
                for(int i=0;i<optionalDataLength;i++)m_optionalData.data[i] = 0;
            }
            m_optionalData.data[contact_states_index_map["rleg"]] = m_optionalData.data[optionalDataLength/2 + contact_states_index_map["rleg"]] = wbms->rp_ref_out.tgt[rf].is_contact;
            m_optionalData.data[contact_states_index_map["lleg"]] = m_optionalData.data[optionalDataLength/2 + contact_states_index_map["lleg"]] = wbms->rp_ref_out.tgt[lf].is_contact;
        }
        wbms->baselinkpose.p = fik->m_robot->rootLink()->p;
        wbms->baselinkpose.rpy = hrp::rpyFromRot(fik->m_robot->rootLink()->R);
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
    m_htlh_dbgOut.write();mode
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
            if(mode.pre() == MODE_IDLE){ double tmp = 1.0; t_ip->setGoal(&tmp, 3.0, true); }
            if (!t_ip->isEmpty() ){
                t_ip->get(&output_ratio, true);
            }else{
                mode.setNextMode(MODE_WBMS);
            }
            break;

        case MODE_SYNC_TO_IDLE:
            if(mode.pre() == MODE_WBMS || mode.pre() == MODE_PAUSE){ double tmp = 0.0; t_ip->setGoal(&tmp, 3.0, true); }
            if (!t_ip->isEmpty()) {
                t_ip->get(&output_ratio, true);
            }else{
                mode.setNextMode(MODE_IDLE);
            }
            break;
    }
}


void WholeBodyMasterSlave::preProcessForWholeBodyMasterSlave(){
    hrp::Vector3 basePos_heightChecked = hrp::to_Vector3(m_basePos.data);//ベースリンク高さ調整により足裏高さ0に
    fik->m_robot->rootLink()->p = basePos_heightChecked;
//    for ( int i = 0; i < fik->m_robot->numJoints(); i++ ){ fik->m_robot->joint(i)->q = m_qRef.data[i]; }
    hrp::setQAll(fik->m_robot, hrp::to_dvector(m_qRef.data));
    fik->m_robot->calcForwardKinematics();

    //TODO
//    hrp::Vector3 init_foot_mid_coord = (fik_in->getEndEffectorPos("rleg") + fik_in->getEndEffectorPos("lleg")) / 2;
//    if( fabs((double)init_foot_mid_coord(Z)) > 1e-5 ){
//        basePos_heightChecked(Z) -= init_foot_mid_coord(Z);
//        init_foot_mid_coord(Z) = 0;
//        std::cerr<<"["<<m_profile.instance_name<<"] Input basePos height is invalid. Auto modify "<<m_basePos.data.z<<" -> "<<basePos_heightChecked(Z)<<endl;
//    }

    std::vector<hrp::BodyPtr> body_list;
    body_list.push_back(fik->m_robot);
    body_list.push_back(m_robot_vsafe);
    for(int i=0;i<body_list.size();i++){//初期姿勢でBodyをFK
        hrp::setRobotStateVec(body_list[i], hrp::to_dvector(m_qRef.data), basePos_heightChecked, hrp::rotFromRpy(m_baseRpy.data.r, m_baseRpy.data.p, m_baseRpy.data.y));
        body_list[i]->calcForwardKinematics();
    }

    fik->q_ref = hrp::getRobotStateVec(fik->m_robot);
    q_ip->set(fik->q_ref.data());
    wbms->initializeRequest(fik->m_robot, ee_ikc_map);
}


void WholeBodyMasterSlave::solveFullbodyIK(const WBMSPose3D& com_ref, const WBMSPose3D& rf_ref, const WBMSPose3D& lf_ref, const WBMSPose3D& rh_ref, const WBMSPose3D& lh_ref, const WBMSPose3D& head_ref){
    std::vector<IKConstraint> ikc_list;
    if(wbms->wp.use_lower){
        IKConstraint tmp;
        tmp.target_link_name = "WAIST";
        tmp.localPos = hrp::Vector3::Zero();
        tmp.localR = hrp::Matrix33::Identity();
        tmp.targetPos = hrp::to_Vector3(m_basePos.data);// will be ignored by selection_vec
        tmp.targetRpy = hrp::to_Vector3(m_baseRpy.data);// ベースリンクの回転をフリーにはしないほうがいい(omegaの積分誤差で暴れる)
        tmp.constraint_weight << 0,0,0,1,1,1;
        tmp.rot_precision = deg2rad(3);
        ikc_list.push_back(tmp);
    }
    if(!wbms->wp.use_lower){
      IKConstraint tmp;
      tmp.target_link_name = "WAIST";
      tmp.localPos = hrp::Vector3::Zero();
      tmp.localR = hrp::Matrix33::Identity();
      tmp.targetPos = hrp::to_Vector3(m_basePos.data);// will be ignored by selection_vec
      tmp.targetRpy = hrp::to_Vector3(m_baseRpy.data);// ベースリンクの回転をフリーにはしないほうがいい(omegaの積分誤差で暴れる)
      tmp.constraint_weight << 1,1,1,1,1,1;
      tmp.rot_precision = deg2rad(3);
      ikc_list.push_back(tmp);
    }
    if(wbms->wp.use_lower){
        IKConstraint tmp;
        tmp.target_link_name = ee_ikc_map["rleg"].target_link_name;
        tmp.localPos = ee_ikc_map["rleg"].localPos;
        tmp.localR = ee_ikc_map["rleg"].localR;
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
        tmp.target_link_name = ee_ikc_map["lleg"].target_link_name;
        tmp.localPos = ee_ikc_map["lleg"].localPos;
        tmp.localR = ee_ikc_map["lleg"].localR;
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
        tmp.target_link_name = ee_ikc_map["rarm"].target_link_name;
        tmp.localPos = ee_ikc_map["rarm"].localPos;
        tmp.localR = ee_ikc_map["rarm"].localR;
        if((rh_ref.p-lh_ref.p).norm()<dist){
            tmp.targetPos = (rh_ref.p+lh_ref.p)/2 + (rh_ref.p-lh_ref.p).normalized()*dist/2;
        }else{
            tmp.targetPos = rh_ref.p;
        }
        tmp.targetRpy = rh_ref.rpy;
        tmp.constraint_weight = hrp::dvector6::Constant(0.1);
        tmp.pos_precision = 3e-3;
        tmp.rot_precision = deg2rad(3);
        ikc_list.push_back(tmp);
    }{
        IKConstraint tmp;
        tmp.target_link_name = ee_ikc_map["larm"].target_link_name;
        tmp.localPos = ee_ikc_map["larm"].localPos;
        tmp.localR = ee_ikc_map["larm"].localR;
        if((rh_ref.p-lh_ref.p).norm()<dist){
            tmp.targetPos = (rh_ref.p+lh_ref.p)/2 + (lh_ref.p-rh_ref.p).normalized()*dist/2;
        }else{
            tmp.targetPos = lh_ref.p;
        }
        tmp.targetRpy = lh_ref.rpy;
        tmp.constraint_weight = hrp::dvector6::Constant(0.1);
        tmp.pos_precision = 3e-3;
        tmp.rot_precision = deg2rad(3);
        ikc_list.push_back(tmp);
    }if(fik->m_robot->link("HEAD_JOINT1") != NULL){
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
            tmp.target_link_name = fik->m_robot->joint(sccp->collision_info_list[i].id1)->name;
            tmp.targetPos = sccp->collision_info_list[i].cp0_wld + (sccp->collision_info_list[i].cp1_wld - sccp->collision_info_list[i].cp0_wld).normalized() * (sccp->collision_info_list[i].dist_safe + margin);
            ikc_list.push_back(tmp);
        }else if(sccp->avoid_priority(sccp->collision_info_list[i].id0) < sccp->avoid_priority(sccp->collision_info_list[i].id1)){
            tmp.localPos = sccp->collision_info_list[i].cp0_local;
            tmp.target_link_name = fik->m_robot->joint(sccp->collision_info_list[i].id0)->name;
            tmp.targetPos = sccp->collision_info_list[i].cp1_wld + (sccp->collision_info_list[i].cp0_wld - sccp->collision_info_list[i].cp1_wld).normalized() * (sccp->collision_info_list[i].dist_safe + margin);
            ikc_list.push_back(tmp);
        }else{
            tmp.localPos = sccp->collision_info_list[i].cp1_local;
            tmp.target_link_name = fik->m_robot->joint(sccp->collision_info_list[i].id1)->name;
            tmp.targetPos = sccp->collision_info_list[i].cp0_wld + (sccp->collision_info_list[i].cp1_wld - sccp->collision_info_list[i].cp0_wld).normalized() * (sccp->collision_info_list[i].dist_safe + margin);
            ikc_list.push_back(tmp);
            tmp.localPos = sccp->collision_info_list[i].cp0_local;
            tmp.target_link_name = fik->m_robot->joint(sccp->collision_info_list[i].id0)->name;
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
                std::cout<<fik->m_robot->joint(sccp->collision_info_list[i].id0)->name<<" "<<fik->m_robot->joint(sccp->collision_info_list[i].id1)->name<<endl;
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

    if( fik->m_robot->link("CHEST_JOINT0") != NULL) fik->dq_weight_all(fik->m_robot->link("CHEST_JOINT0")->jointId) = 0.1;//JAXON
    if( fik->m_robot->link("CHEST_JOINT1") != NULL) fik->dq_weight_all(fik->m_robot->link("CHEST_JOINT1")->jointId) = 0.1;
    if( fik->m_robot->link("CHEST_JOINT2") != NULL) fik->dq_weight_all(fik->m_robot->link("CHEST_JOINT2")->jointId) = 0.1;
    if( fik->m_robot->link("CHEST_Y") != NULL) fik->dq_weight_all(fik->m_robot->link("CHEST_Y")->jointId) = 0.1;//K
    if( fik->m_robot->link("CHEST_P") != NULL) fik->dq_weight_all(fik->m_robot->link("CHEST_P")->jointId) = 0.1;
//    if( robot_in->link("RARM_JOINT2") != NULL) robot_in->link("RARM_JOINT2")->ulimit = deg2rad(-30);//脇内側の干渉回避
//    if( robot_in->link("LARM_JOINT2") != NULL) robot_in->link("LARM_JOINT2")->llimit = deg2rad(30);
    if( fik->m_robot->link("RLEG_JOINT3") != NULL) fik->m_robot->link("RLEG_JOINT3")->llimit = deg2rad(10);//膝伸びきり防止のため
    if( fik->m_robot->link("LLEG_JOINT3") != NULL) fik->m_robot->link("LLEG_JOINT3")->llimit = deg2rad(10);
    if( fik->m_robot->link("R_KNEE_P") != NULL) fik->m_robot->link("R_KNEE_P")->llimit = deg2rad(15);//K
    if( fik->m_robot->link("L_KNEE_P") != NULL) fik->m_robot->link("L_KNEE_P")->llimit = deg2rad(15);
    if( fik->m_robot->link("R_WRIST_R") != NULL) fik->m_robot->link("R_WRIST_R")->llimit = deg2rad(-40);
    if( fik->m_robot->link("L_WRIST_R") != NULL) fik->m_robot->link("L_WRIST_R")->llimit = deg2rad(-40);
    if( fik->m_robot->link("R_WRIST_R") != NULL) fik->m_robot->link("R_WRIST_R")->ulimit = deg2rad(40);
    if( fik->m_robot->link("L_WRIST_R") != NULL) fik->m_robot->link("L_WRIST_R")->ulimit = deg2rad(40);
    if( fik->m_robot->link("R_WRIST_P") != NULL) fik->m_robot->link("R_WRIST_P")->llimit = deg2rad(-40);
    if( fik->m_robot->link("L_WRIST_P") != NULL) fik->m_robot->link("L_WRIST_P")->llimit = deg2rad(-40);
    if( fik->m_robot->link("R_WRIST_P") != NULL) fik->m_robot->link("R_WRIST_P")->ulimit = deg2rad(20);
    if( fik->m_robot->link("L_WRIST_P") != NULL) fik->m_robot->link("L_WRIST_P")->ulimit = deg2rad(20);


    if( fik->m_robot->link("CHEST_Y") != NULL) fik->m_robot->link("CHEST_Y")->llimit = deg2rad(-20);
    if( fik->m_robot->link("CHEST_Y") != NULL) fik->m_robot->link("CHEST_Y")->ulimit = deg2rad(20);
    if( fik->m_robot->link("CHEST_P") != NULL) fik->m_robot->link("CHEST_P")->llimit = deg2rad(0);
    if( fik->m_robot->link("CHEST_P") != NULL) fik->m_robot->link("CHEST_P")->ulimit = deg2rad(60);
                                              
    if( fik->m_robot->link("HEAD_Y") != NULL) fik->m_robot->link("HEAD_Y")->llimit = deg2rad(-5);
    if( fik->m_robot->link("HEAD_Y") != NULL) fik->m_robot->link("HEAD_Y")->ulimit = deg2rad(5);
    if( fik->m_robot->link("HEAD_P") != NULL) fik->m_robot->link("HEAD_P")->llimit = deg2rad(0);
    if( fik->m_robot->link("HEAD_P") != NULL) fik->m_robot->link("HEAD_P")->ulimit = deg2rad(60);

    for(int i=0;i<fik->m_robot->numJoints();i++){
        LIMIT_MINMAX(fik->m_robot->joint(i)->q, fik->m_robot->joint(i)->llimit, fik->m_robot->joint(i)->ulimit);
    }

//    fik_in->q_ref = init_sync_state;


    // std::string tmp[] = {"R_CROTCH_R","R_CROTCH_P","R_CROTCH_Y","R_KNEE_P","R_ANKLE_R","R_ANKLE_P","L_CROTCH_R","L_CROTCH_P","L_CROTCH_Y","L_KNEE_P","L_ANKLE_R","L_ANKLE_P"};
    // const std::vector<std::string> list(tmp, tmp+12);
    // for(int i=0; i<list.size(); i++){
    //   if( robot_in->link(list[i]) != NULL){
    //     int j=robot_in->link(list[i])->jointId;
    //     fik_in->dq_weight_all(j) = 1e-5;setNextMode
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
    int loop_result = fik->solveFullbodyIKLoop(ikc_list, IK_MAX_LOOP);
    if(loop%100==0){clock_gettime(CLOCK_REALTIME, &endT); std::cout << (endT.tv_sec - startT.tv_sec + (endT.tv_nsec - startT.tv_nsec) * 1e-9) << " @ solveIK"<<loop_result<<"loop" << std::endl;}
}

void WholeBodyMasterSlave::processHOFFARBIBFilter(hrp::BodyPtr _robot, hrp::BodyPtr _robot_safe){
    double goal_time = 0.0;
    const double min_goal_time_offset = 0.1;

    static hrp::dvector ans_state_vel = hrp::dvector::Zero(fik->numStates());
//    if (!q_ip->isEmpty() ){  q_ip->get(tmp_x, ans_state_vel, false);}

//    for(int i=0;i<_robot->numJoints();i++){
//        if(_robot->joint(i)->name != "L_HAND" && _robot->joint(i)->name != "R_HAND" ){ //KHI demo
//          double tmp_time = fabs(_robot->joint(i)->q - _robot_safe->joint(i)->q) / avg_q_vel(i);
//          LIMIT_MIN(tmp_time, fabs(ans_state_vel(i))/16);
//  //        tmp_time += fabs(ans_state_vel(i))/16; //加速度8
//          if(tmp_time > goal_time){ goal_time = tmp_time; }
//        }
//    }
//
//    q_ip->setGoal(hrp::getRobotStateVec(_robot).data(), goal_time + min_goal_time_offset, true);
//    hrp::dvector ans_state(fik->numStates());
//    double tmp[fik->numStates()], tmpv[fik->numStates()];
//    if (!q_ip->isEmpty() ){  q_ip->get(tmp, tmpv, true);}
//    ans_state_vel = Eigen::Map<hrp::dvector>(tmpv, fik->numStates());
//    ans_state = Eigen::Map<hrp::dvector>(tmp, fik->numStates());


    hrp::dvector estimated_times_from_vel_limit = (hrp::getRobotStateVec(_robot) - hrp::getRobotStateVec(_robot_safe)).array().abs() / avg_q_vel.array();
    hrp::dvector estimated_times_from_acc_limit = ans_state_vel.array() / avg_q_acc.array();
    double longest_estimated_time_from_vel_limit = estimated_times_from_vel_limit.maxCoeff();
    double longest_estimated_time_from_acc_limit = estimated_times_from_acc_limit.maxCoeff();
    goal_time = hrp::Vector3(longest_estimated_time_from_vel_limit, longest_estimated_time_from_acc_limit, min_goal_time_offset).maxCoeff();

    q_ip->setGoal(hrp::getRobotStateVec(_robot).data(), goal_time, true);
    double tmp[fik->numStates()], tmpv[fik->numStates()];
    if (!q_ip->isEmpty() ){  q_ip->get(tmp, tmpv, true);}
    hrp::dvector ans_state = Eigen::Map<hrp::dvector>(tmp, fik->numStates());
    ans_state_vel = Eigen::Map<hrp::dvector>(tmpv, fik->numStates());


//    //// KHI demo
//    const double hand_max_vel = 180.0/0.4 /180.0*M_PI;// 0.4sec for 180deg
//    std::map<std::string, double> joy_inputs;// 0 ~ 1
//    joy_inputs["L_HAND"] = m_htlfw.data.force.x;// tmp substitute
//    joy_inputs["R_HAND"] = m_htrfw.data.force.x;// tmp substitute
//    for(std::map<std::string, double>::iterator it = joy_inputs.begin(); it!=joy_inputs.end(); it++){
//      if(_robot_safe->link(it->first) != NULL){
//          LIMIT_MINMAX(it->second, 0.0, 1.0);
//          double tgt_hand_q = (-29.0 + (124.0-(-29.0))*it->second ) /180.0*M_PI;
//          const double q_old = _robot_safe->link(it->first)->q;
//          LIMIT_MINMAX(tgt_hand_q, q_old - hand_max_vel*m_dt, q_old + hand_max_vel*m_dt);
//          _robot_safe->link(it->first)->q = tgt_hand_q;
//      }
//    }

    hrp::setRobotStateVec(_robot_safe, ans_state);

    _robot_safe->calcForwardKinematics();
}


bool WholeBodyMasterSlave::startWholeBodyMasterSlave(){
    if(mode.now() == MODE_IDLE){
        RTCOUT << "startWholeBodyMasterSlave" << std::endl;
        mode.setNextMode(MODE_SYNC_TO_WBMS);
        while(!t_ip->isEmpty()){ usleep(1000); }
        return true;
    }else{
        RTCOUT << "Invalid context to startWholeBodyMasterSlave" << std::endl;
        return false;
    }
}


bool WholeBodyMasterSlave::pauseWholeBodyMasterSlave(){
    if(mode.now() == MODE_WBMS){
        RTCOUT << "pauseWholeBodyMasterSlave" << std::endl;
        mode.setNextMode(MODE_PAUSE);
        return true;
    }else{
        RTCOUT << "Invalid context to pauseWholeBodyMasterSlave" << std::endl;
        return false;
    }
}


bool WholeBodyMasterSlave::resumeWholeBodyMasterSlave(){
    if(mode.now() == MODE_PAUSE){
        RTCOUT << "resumeWholeBodyMasterSlave" << std::endl;
        mode.setNextMode(MODE_WBMS);
        avg_q_vel /= 5;
        sleep(5);
        avg_q_vel *= 5;
        return true;
    }else{
        RTCOUT << "Invalid context to resumeWholeBodyMasterSlave" << std::endl;
        return false;
    }
}


bool WholeBodyMasterSlave::stopWholeBodyMasterSlave(){
    if(mode.now() == MODE_WBMS || mode.now() == MODE_PAUSE ){
        RTCOUT << "stopWholeBodyMasterSlave" << std::endl;
        mode.setNextMode(MODE_SYNC_TO_IDLE);
        while(!t_ip->isEmpty()){ usleep(1000); }
        return true;
    }else{
        RTCOUT << "Invalid context to stopWholeBodyMasterSlave" << std::endl;
        return false;
    }
}


bool WholeBodyMasterSlave::setWholeBodyMasterSlaveParam(const OpenHRP::WholeBodyMasterSlaveService::WholeBodyMasterSlaveParam& i_param){
    RTCOUT << "setWholeBodyMasterSlaveParam" << std::endl;
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
    RTCOUT << "getWholeBodyMasterSlaveParam" << std::endl;
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


RTC::ReturnCode_t WholeBodyMasterSlave::onActivated(RTC::UniqueId ec_id){ RTCOUT << "onActivated(" << ec_id << ")" << std::endl; return RTC::RTC_OK; }
RTC::ReturnCode_t WholeBodyMasterSlave::onDeactivated(RTC::UniqueId ec_id){ RTCOUT << "onDeactivated(" << ec_id << ")" << std::endl; return RTC::RTC_OK; }

extern "C"{
    void WholeBodyMasterSlaveInit(RTC::Manager* manager) {
        RTC::Properties profile(WholeBodyMasterSlave_spec);
        manager->registerFactory(profile, RTC::Create<WholeBodyMasterSlave>, RTC::Delete<WholeBodyMasterSlave>);
    }
};
