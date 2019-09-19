#include "WholeBodyMasterSlave.h"

#define DEBUGP (loop%200==0)
#define DEBUGP_ONCE (loop==0)

static const char* WholeBodyMasterSlave_spec[] = {
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
    m_qActIn("qAct", m_qAct),
    m_zmpIn("zmpIn", m_zmp),
    m_basePosIn("basePosIn", m_basePos),
    m_baseRpyIn("baseRpyIn", m_baseRpy),
    m_optionalDataIn("optionalData", m_optionalData),
    m_qOut("q", m_qRef),// to abc
    m_zmpOut("zmpOut", m_zmp),
    m_basePosOut("basePosOut", m_basePos),
    m_baseRpyOut("baseRpyOut", m_baseRpy),
    m_optionalDataOut("optionalDataOut", m_optionalData),
    m_actCPIn("actCapturePoint", m_actCP),
    m_actZMPIn("zmp", m_actZMP),
    m_exDataIn("exData", m_exData),
    m_exDataIndexIn("exDataIndex", m_exDataIndex),
    m_WholeBodyMasterSlaveServicePort("WholeBodyMasterSlaveService"),
    m_AutoBalancerServicePort("AutoBalancerService"),
    m_StabilizerServicePort("StabilizerService"),
    m_debugLevel(0)
{
    m_service0.wholebodymasterslave(this);
}

WholeBodyMasterSlave::~WholeBodyMasterSlave(){}

RTC::ReturnCode_t WholeBodyMasterSlave::onInitialize(){
    RTC_INFO_STREAM("onInitialize()");
    bindParameter("debugLevel", m_debugLevel, "0");
    addInPort("qRef", m_qRefIn);// from sh
    addInPort("qAct", m_qActIn);
    addInPort("zmpIn", m_zmpIn);
    addInPort("basePosIn", m_basePosIn);
    addInPort("baseRpyIn", m_baseRpyIn);
    addInPort("optionalData", m_optionalDataIn);
    addOutPort("q", m_qOut);// to abc
    addOutPort("zmpOut", m_zmpOut);
    addOutPort("basePosOut", m_basePosOut);
    addOutPort("baseRpyOut", m_baseRpyOut);
    addOutPort("optionalDataOut", m_optionalDataOut);
    addInPort("actCapturePoint", m_actCPIn);
    addInPort("zmp", m_actZMPIn);
    addInPort("exData", m_exDataIn);
    addInPort("exDataIndex", m_exDataIndexIn);
    m_WholeBodyMasterSlaveServicePort.registerProvider("service0", "WholeBodyMasterSlaveService", m_service0);
    addPort(m_WholeBodyMasterSlaveServicePort);

    m_AutoBalancerServicePort.registerConsumer("service0","AutoBalancerService", m_AutoBalancerServiceConsumer);
    m_StabilizerServicePort.registerConsumer("service0","StabilizerService", m_StabilizerServiceConsumer);
    addPort(m_AutoBalancerServicePort);
    addPort(m_StabilizerServicePort);

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
        RTC_WARN_STREAM("failed to load model[" << prop["model"] << "]");
        return RTC::RTC_ERROR;
    }
    m_robot_act = hrp::BodyPtr(new hrp::Body(*robot_for_ik)); //copy
    m_robot_vsafe = hrp::BodyPtr(new hrp::Body(*robot_for_ik)); //copy
    RTC_INFO_STREAM("setup robot model finished");

    fik = fikPtr(new FullbodyInverseKinematicsSolver(robot_for_ik, std::string(m_profile.instance_name), m_dt));
    setupEEIKConstraintFromConf(ee_ikc_map, robot_for_ik, prop);
    RTC_INFO_STREAM("setup fullbody ik finished");

    wbms = boost::shared_ptr<WBMSCore>(new WBMSCore(m_dt));
    if(fik->m_robot->name() == "RHP4B"){
        for(int i=0; i<fik->m_robot->numJoints(); i++){
            if(fik->m_robot->joint(i)->name.find("-linear-joint") == std::string::npos){
                wbms->wp.use_joints.push_back(fik->m_robot->joint(i)->name);
            }
        }
    }else{
        wbms->wp.use_joints = getJointNameAll(fik->m_robot);
    }

    if(fik->m_robot->name().find("JAXON") != std::string::npos){ // for demo
        wbms->wp.use_targets.push_back("rleg");
        wbms->wp.use_targets.push_back("lleg");
        wbms->wp.use_targets.push_back("rarm");
        wbms->wp.use_targets.push_back("larm");
        wbms->wp.use_targets.push_back("com");
        wbms->wp.use_targets.push_back("head");
    }else{
        wbms->wp.use_targets.push_back("rleg");
        wbms->wp.use_targets.push_back("lleg");
        wbms->wp.use_targets.push_back("rarm");
        wbms->wp.use_targets.push_back("larm");
        wbms->wp.use_targets.push_back("com");
        wbms->wp.use_targets.push_back("head");
    }
    sccp = boost::shared_ptr<CapsuleCollisionChecker>(new CapsuleCollisionChecker(fik->m_robot));
    RTC_INFO_STREAM("setup main function class finished");

    // allocate memory for outPorts
    m_qRef.data.length(fik->m_robot->numJoints()); // is really needed?
    coil::stringTo(optionalDataLength, prop["seq_optional_data_dim"].c_str());

    output_ratio = 0.0;
    t_ip = new interpolator(1, m_dt, interpolator::HOFFARBIB, 1);
    t_ip->clear();
    t_ip->set(&output_ratio);
    q_ip = new interpolator(fik->numStates(), m_dt, interpolator::CUBICSPLINE, 1); // or HOFFARBIB, QUINTICSPLINE
    q_ip->clear();
    if(fik->m_robot->name().find("JAXON") != std::string::npos){
        avg_q_vel = hrp::dvector::Constant(fik->numStates(), 4.0); // all joint max avarage vel = 1.0 rad/s
    }else{
        avg_q_vel = hrp::dvector::Constant(fik->numStates(), 1.0); // all joint max avarage vel = 1.0 rad/s
    }
    avg_q_acc = hrp::dvector::Constant(fik->numStates(), 16.0); // all joint max avarage acc = 16.0 rad/s^2
    avg_q_vel.tail(6).fill(std::numeric_limits<double>::max()); // no limit for base link vel
    avg_q_acc.tail(6).fill(std::numeric_limits<double>::max()); // no limit for base link acc
    RTC_INFO_STREAM("setup interpolator finished");

    ref_zmp_filter.setParameter(20, 1/m_dt, Q_BUTTERWORTH);

    cp_flag = -1;
    lt.fill(0);
    rt.fill(0);

    to_enum["lleg"] = lf;
    to_enum["rleg"] = rf;
    to_enum["larm"] = lh;
    to_enum["rarm"] = rh;
    to_enum["com"] =  com;
    to_enum["head"] = head;

    for ( int i=0; i<ee_names.size(); i++) {
        std::string n = "slave_"+ee_names[i]+"_wrench";
        m_slaveEEWrenchesOut[ee_names[i]] = OTDS_Ptr(new RTC::OutPort<RTC::TimedDoubleSeq>(n.c_str(), m_slaveEEWrenches[ee_names[i]]));
        registerOutPort(n.c_str(), *m_slaveEEWrenchesOut[ee_names[i]]);
        RTC_INFO_STREAM(" registerOutPort " << n);
    }
    for ( int i=0; i<ee_names.size(); i++) {
        std::string n = "local_"+ee_names[i]+"_wrench";
        m_localEEWrenchesIn[ee_names[i]] = ITDS_Ptr(new RTC::InPort<RTC::TimedDoubleSeq>(n.c_str(), m_localEEWrenches[ee_names[i]]));
        registerInPort(n.c_str(), *m_localEEWrenchesIn[ee_names[i]]);
        RTC_INFO_STREAM(" registerInPort " << n);
    }

    tgt_names = ee_names;
    tgt_names.push_back("com");
    tgt_names.push_back("head");
    for ( int i=0; i<tgt_names.size(); i++) {
        std::string n = "master_"+tgt_names[i]+"_pose";
        m_masterTgtPosesIn[tgt_names[i]] = ITP3_Ptr(new RTC::InPort<RTC::TimedPose3D>(n.c_str(), m_masterTgtPoses[tgt_names[i]]));
        registerInPort(n.c_str(), *m_masterTgtPosesIn[tgt_names[i]]);
        RTC_INFO_STREAM(" registerInPort " << n);
    }

    RTC_INFO_STREAM("onInitialize() OK");
    loop = 0;
    return RTC::RTC_OK;
}

RTC::ReturnCode_t WholeBodyMasterSlave::setupEEIKConstraintFromConf(std::map<std::string, IKConstraint>& _ee_ikc_map, hrp::BodyPtr _robot, RTC::Properties& _prop){
    coil::vstring ee_conf_all = coil::split(_prop["end_effectors"], ",");
    size_t prop_num = 10; // limbname + linkname + basename + pos(3) + angleaxis(4)
    if (ee_conf_all.size() > 0) {
        size_t ee_num = ee_conf_all.size()/prop_num;
        for (size_t i = 0; i < ee_num; i++) {
            std::string ee_name, target_link_name, base_name; // e.g. rleg, RLEG_JOINT5, WAIST
            coil::stringTo(ee_name,             ee_conf_all[i*prop_num].c_str());
            coil::stringTo(target_link_name,    ee_conf_all[i*prop_num+1].c_str());
            coil::stringTo(base_name,           ee_conf_all[i*prop_num+2].c_str());
            ee_names.push_back(ee_name);
            _ee_ikc_map[ee_name].target_link_name = target_link_name;
            for (size_t j = 0; j < XYZ; j++){ coil::stringTo(_ee_ikc_map[ee_name].localPos(j), ee_conf_all[i*prop_num+3+j].c_str()); }
            double tmp_aa[4];
            for (int j = 0; j < 4; j++ ){ coil::stringTo(tmp_aa[j], ee_conf_all[i*prop_num+6+j].c_str()); }
            _ee_ikc_map[ee_name].localR = Eigen::AngleAxis<double>(tmp_aa[3], hrp::Vector3(tmp_aa[0], tmp_aa[1], tmp_aa[2])).toRotationMatrix(); // rotation in VRML is represented by axis + angle
            if(_robot->link(target_link_name)){
                RTC_INFO_STREAM("End Effector [" << ee_name << "]");
                RTC_INFO_STREAM("   target_link_name = " << _ee_ikc_map[ee_name].target_link_name << ", base = " << base_name);
                RTC_INFO_STREAM("   offset_pos = " << _ee_ikc_map[ee_name].localPos.transpose() << "[m]");
                RTC_INFO_STREAM("   has_toe_joint = " << "fix to false now");
            }else{
                RTC_WARN_STREAM("Target link [" << target_link_name << "] not found !");
                return RTC::RTC_ERROR;
            }
            contact_states_index_map.insert(std::pair<std::string, size_t>(ee_name, i));////TODO:要移動? //used for check optional data order
        }
    }
    return RTC::RTC_OK;
}

RTC::ReturnCode_t WholeBodyMasterSlave::onExecute(RTC::UniqueId ec_id){
    if(DEBUGP_ONCE) RTC_INFO_STREAM("onExecute(" << ec_id << ")");
    time_report_str.clear();
    clock_gettime(CLOCK_REALTIME, &startT);
    if (m_qRefIn.isNew()) { m_qRefIn.read(); }
    if (m_qActIn.isNew()) { m_qActIn.read(); }
    if (m_basePosIn.isNew()) { m_basePosIn.read(); }
    if (m_baseRpyIn.isNew()) { m_baseRpyIn.read(); }
    if (m_zmpIn.isNew()) { m_zmpIn.read(); }
    if (m_optionalDataIn.isNew()) { m_optionalDataIn.read(); }
    for(int i=0; i<ee_names.size(); i++){ if (m_localEEWrenchesIn[ee_names[i]]->isNew()){ m_localEEWrenchesIn[ee_names[i]]->read(); } }

    // button func
    hrp::dvector ex_data;
    std::vector<std::string> ex_data_index;
    if(m_exDataIn.isNew()){
        m_exDataIn.read();
        ex_data = hrp::to_dvector(m_exData.data);
    }
    if(m_exDataIndexIn.isNew()){
        m_exDataIndexIn.read();
        ex_data_index = hrp::to_string_vector(m_exDataIndex.data);
    }

//    if(m_htlfw.data.force.y >= 1 && m_htrfw.data.force.y >= 1 && mode.now() == MODE_IDLE){
//        std::cerr<<"startWholeBodyMasterSlave() called by button"<<std::endl;
//        startWholeBodyMasterSlave();
//    }else if(m_htlfw.data.force.y >= 1 && m_htrfw.data.force.y >= 1 && mode.now() == MODE_WBMS){
//        std::cerr<<"stopWholeBodyMasterSlave() called by button"<<std::endl;
//        stopWholeBodyMasterSlave();
//    }
//    static bool is_blocking_continuous_hits = false;
//    if(m_htlfw.data.force.z >= 1 && m_htrfw.data.force.z >= 1 && mode.now() == MODE_PAUSE){
//        if(!is_blocking_continuous_hits){
//            std::cerr<<"resumeWholeBodyMasterSlave() called by button"<<std::endl;
//            resumeWholeBodyMasterSlave();
//            is_blocking_continuous_hits = true;
//        }
//    }else if(m_htlfw.data.force.z >= 1 && m_htrfw.data.force.z >= 1 && mode.now() == MODE_WBMS){
//        if(!is_blocking_continuous_hits){
//            std::cerr<<"pauseWholeBodyMasterSlave() called by button"<<std::endl;
//            pauseWholeBodyMasterSlave();
//            is_blocking_continuous_hits = true;
//        }
//    }else{
//        is_blocking_continuous_hits = false;
//    }


    if( mode.now() != MODE_PAUSE ){ // stop updating input when MODE_PAUSE
        for(int i=0; i<tgt_names.size(); i++){
            if (m_masterTgtPosesIn[tgt_names[i]]->isNew()){
                m_masterTgtPosesIn[tgt_names[i]]->read();
                wbms->hp_wld_raw.tgt_by_str(tgt_names[i]).abs = hrp::to_Pose3(m_masterTgtPoses[tgt_names[i]].data);
            }
        }
        if (m_actCPIn.isNew())  { m_actCPIn.read(); rel_act_cp = hrp::to_Vector3(m_actCP.data);}
        if (m_actZMPIn.isNew())  { m_actZMPIn.read(); rel_act_zmp = hrp::to_Vector3(m_actZMP.data);}
    }
    addTimeReport("InPort");

    processTransition();
    mode.update();

    if (mode.isRunning()) {
        if(mode.isInitialize()){
            preProcessForWholeBodyMasterSlave();
            idsb.setInitState(fik->m_robot, m_dt);//逆動力学初期化
        }
        wbms->update();//////HumanSynchronizerの主要処理
        if(DEBUGP)wbms->rp_ref_out.print();
        addTimeReport("MainFunc");

        solveFullbodyIK(wbms->rp_ref_out.tgt[com].abs, wbms->rp_ref_out.tgt[rf].abs, wbms->rp_ref_out.tgt[lf].abs, wbms->rp_ref_out.tgt[rh].abs, wbms->rp_ref_out.tgt[lh].abs, wbms->rp_ref_out.tgt[head].abs);
        addTimeReport("IK");

        smoothingJointAngles(fik->m_robot, m_robot_vsafe);

        hrp::Vector3 com = m_robot_vsafe->calcCM();
        com(Z) += 0.3;//hotfix
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
        wbms->act_rs.st_zmp = m_robot_vsafe->rootLink()->p + m_robot_vsafe->rootLink()->R * rel_act_zmp;

//            if(wbms->rp_ref_out.tgt[rf].is_contact && !wbms->rp_ref_out.tgt[lf].is_contact){
//                ref_zmp.head(XY) = m_robot_vsafe->link(ee_ikc_map["rleg"].target_link_name)->p.head(XY);
////                ref_zmp(X) + 0.05;
//            }
//            if(wbms->rp_ref_out.tgt[lf].is_contact && !wbms->rp_ref_out.tgt[rf].is_contact){
//                ref_zmp.head(XY) = m_robot_vsafe->link(ee_ikc_map["lleg"].target_link_name)->p.head(XY);
////                ref_zmp(X) + 0.05;
//            }

        // qRef
        for (int i = 0; i < m_qRef.data.length(); i++ ){
          if(has(wbms->wp.use_joints, fik->m_robot->joint(i)->name)){
              m_qRef.data[i] = output_ratio * m_robot_vsafe->joint(i)->q  + (1 - output_ratio) * m_qRef.data[i];
          }
        }
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
        m_optionalData.data[contact_states_index_map["rleg"]] = m_optionalData.data[optionalDataLength/2 + contact_states_index_map["rleg"]] = wbms->rp_ref_out.tgt[rf].is_contact();
        m_optionalData.data[contact_states_index_map["lleg"]] = m_optionalData.data[optionalDataLength/2 + contact_states_index_map["lleg"]] = wbms->rp_ref_out.tgt[lf].is_contact();
        addTimeReport("SetOutPut");
    }
    wbms->baselinkpose.p = fik->m_robot->rootLink()->p;
    wbms->baselinkpose.R = fik->m_robot->rootLink()->R;


    std::map<std::string, std::string> to_sname;
    to_sname["lleg"] = "lfsensor";
    to_sname["rleg"] = "rfsensor";
    to_sname["larm"] = "lhsensor";
    to_sname["rarm"] = "rhsensor";
    hrp::setQAll(m_robot_act, hrp::to_dvector(m_qAct.data));
    m_robot_act->calcForwardKinematics();
    for(int i=0;i<ee_names.size();i++){
        hrp::ForceSensor* sensor = m_robot_act->sensor<hrp::ForceSensor>(to_sname[ee_names[i]]);
        hrp::Matrix33 sensorR_wld = sensor->link->R * sensor->localR;
        hrp::Matrix33 sensorR_from_base = m_robot_act->rootLink()->R.transpose() * sensorR_wld;
        const hrp::Vector3 f_wld = sensorR_from_base * hrp::to_dvector(m_localEEWrenches[ee_names[i]].data).head(3);
        const hrp::Vector3 t_wld = sensorR_from_base * hrp::to_dvector(m_localEEWrenches[ee_names[i]].data).tail(3);
        m_slaveEEWrenches[ee_names[i]].data = hrp::to_DoubleSeq( (hrp::dvector6()<<f_wld,t_wld).finished());
        m_slaveEEWrenches[ee_names[i]].tm = m_qRef.tm;
        m_slaveEEWrenchesOut[ee_names[i]]->write();
    }
    // write
    m_qOut.write();
    m_basePosOut.write();
    m_baseRpyOut.write();
    m_zmpOut.write();
    m_optionalDataOut.write();
    addTimeReport("OutPort");
    if(DEBUGP) RTC_INFO_STREAM(time_report_str);
    loop ++;
    return RTC::RTC_OK;
}


void WholeBodyMasterSlave::processTransition(){
    switch(mode.now()){

        case MODE_SYNC_TO_HC:
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
    fik->m_robot->rootLink()->p = hrp::to_Vector3(m_basePos.data);
    hrp::setQAll(fik->m_robot, hrp::to_dvector(m_qRef.data));
    fik->m_robot->calcForwardKinematics();
    double current_foot_height_from_world = std::min(ee_ikc_map["rleg"].getCurrentTargetPos(fik->m_robot)(Z), ee_ikc_map["lleg"].getCurrentTargetPos(fik->m_robot)(Z));
    RTC_INFO_STREAM("current_foot_height_from_world = "<<current_foot_height_from_world<<" will be modified to 0");

    std::vector<hrp::BodyPtr> body_list;
    body_list.push_back(fik->m_robot);
    body_list.push_back(m_robot_vsafe);
    for(int i=0;i<body_list.size();i++){//初期姿勢でBodyをFK
        hrp::setRobotStateVec(body_list[i], hrp::to_dvector(m_qRef.data), hrp::to_Vector3(m_basePos.data) - hrp::Vector3::UnitZ() * current_foot_height_from_world, hrp::rotFromRpy(hrp::to_Vector3(m_baseRpy.data)));
        body_list[i]->calcForwardKinematics();
    }

    fik->q_ref = hrp::getRobotStateVec(fik->m_robot);
    q_ip->set(fik->q_ref.data());
    wbms->initializeRequest(fik->m_robot, ee_ikc_map);
}


void WholeBodyMasterSlave::solveFullbodyIK(const hrp::Pose3& com_ref, const hrp::Pose3& rf_ref, const hrp::Pose3& lf_ref, const hrp::Pose3& rh_ref, const hrp::Pose3& lh_ref, const hrp::Pose3& head_ref){
    std::vector<IKConstraint> ikc_list;
    if(has(wbms->wp.use_targets, "com")){
        IKConstraint tmp;
        tmp.target_link_name = "WAIST";
        tmp.localPos = hrp::Vector3::Zero();
        tmp.localR = hrp::Matrix33::Identity();
        tmp.targetPos = hrp::to_Vector3(m_basePos.data);// will be ignored by selection_vec
        tmp.targetRpy = com_ref.rpy();
        tmp.constraint_weight << 0,0,0,1e-1,1e-1,1e-1;
        tmp.rot_precision = deg2rad(3);
        ikc_list.push_back(tmp);
    }else{
      IKConstraint tmp;
      tmp.target_link_name = "WAIST";
      tmp.localPos = hrp::Vector3::Zero();
      tmp.localR = hrp::Matrix33::Identity();
      tmp.targetPos = hrp::to_Vector3(m_basePos.data);// will be ignored by selection_vec
      tmp.targetRpy = com_ref.rpy();// ベースリンクの回転をフリーにはしないほうがいい(omegaの積分誤差で暴れる)
      tmp.constraint_weight << 1,1,1,1,1,1;
      tmp.rot_precision = deg2rad(3);
      ikc_list.push_back(tmp);
    }
    if(has(wbms->wp.use_targets, "rleg")){
        IKConstraint tmp;
        tmp.target_link_name = ee_ikc_map["rleg"].target_link_name;
        tmp.localPos = ee_ikc_map["rleg"].localPos;
        tmp.localR = ee_ikc_map["rleg"].localR;
        tmp.targetPos = rf_ref.p;
        tmp.targetRpy = rf_ref.rpy();
        tmp.constraint_weight = wbms->rp_ref_out.tgt[rf].is_contact() ? hrp::dvector6::Constant(3) : hrp::dvector6::Constant(0.1);
        ikc_list.push_back(tmp);
    }
    if(has(wbms->wp.use_targets, "lleg")){
        IKConstraint tmp;
        tmp.target_link_name = ee_ikc_map["lleg"].target_link_name;
        tmp.localPos = ee_ikc_map["lleg"].localPos;
        tmp.localR = ee_ikc_map["lleg"].localR;
        tmp.targetPos = lf_ref.p;
        tmp.targetRpy = lf_ref.rpy();
        tmp.constraint_weight = wbms->rp_ref_out.tgt[lf].is_contact() ? hrp::dvector6::Constant(3) : hrp::dvector6::Constant(0.1);
        ikc_list.push_back(tmp);
    }
    if(has(wbms->wp.use_targets, "rarm")){
        IKConstraint tmp;
        tmp.target_link_name = ee_ikc_map["rarm"].target_link_name;
        tmp.localPos = ee_ikc_map["rarm"].localPos;
        tmp.localR = ee_ikc_map["rarm"].localR;
        tmp.targetPos = rh_ref.p;
        tmp.targetRpy = rh_ref.rpy();
        tmp.constraint_weight = hrp::dvector6::Constant(0.1);
        tmp.pos_precision = 3e-3;
        tmp.rot_precision = deg2rad(3);
        ikc_list.push_back(tmp);
    }
    if(has(wbms->wp.use_targets, "larm")){
        IKConstraint tmp;
        tmp.target_link_name = ee_ikc_map["larm"].target_link_name;
        tmp.localPos = ee_ikc_map["larm"].localPos;
        tmp.localR = ee_ikc_map["larm"].localR;
        tmp.targetPos = lh_ref.p;
        tmp.targetRpy = lh_ref.rpy();
        tmp.constraint_weight = hrp::dvector6::Constant(0.1);
        tmp.pos_precision = 3e-3;
        tmp.rot_precision = deg2rad(3);
        ikc_list.push_back(tmp);
    }
    if(has(wbms->wp.use_targets, "head")){
        if(fik->m_robot->link("HEAD_JOINT1") != NULL){
            IKConstraint tmp;
            tmp.target_link_name = "HEAD_JOINT1";
            tmp.targetRpy = head_ref.rpy();
            tmp.constraint_weight << 0,0,0,0,0.1,0.1;
            tmp.rot_precision = deg2rad(1);
            ikc_list.push_back(tmp);
        }
        if(fik->m_robot->link("HEAD_P") != NULL){
            IKConstraint tmp;
            tmp.target_link_name = "HEAD_P";
            tmp.targetRpy = head_ref.rpy();
            tmp.constraint_weight << 0,0,0,0,0.1,0.1;
            tmp.rot_precision = deg2rad(1);
            ikc_list.push_back(tmp);
        }
    }
    if(wbms->rp_ref_out.tgt[rf].is_contact()){
        sccp->avoid_priority.head(12).head(6).fill(4);
    }else{
        sccp->avoid_priority.head(12).head(6).fill(3);
    }
    if(wbms->rp_ref_out.tgt[lf].is_contact()){
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


    if(has(wbms->wp.use_targets, "com")){
        IKConstraint tmp;
        tmp.target_link_name = "COM";
        tmp.localPos = hrp::Vector3::Zero();
        tmp.localR = hrp::Matrix33::Identity();
        tmp.targetPos = com_ref.p;// COM height will not be constraint
        tmp.targetRpy = hrp::Vector3::Zero();//reference angular momentum
        tmp.constraint_weight << 3,3,0.01,0,0,0;
        ikc_list.push_back(tmp);
    }

    if( fik->m_robot->link("CHEST_JOINT0") != NULL) fik->dq_weight_all(fik->m_robot->link("CHEST_JOINT0")->jointId) = 0.01;//JAXON
    if( fik->m_robot->link("CHEST_JOINT1") != NULL) fik->dq_weight_all(fik->m_robot->link("CHEST_JOINT1")->jointId) = 0.01;
    if( fik->m_robot->link("CHEST_JOINT2") != NULL) fik->dq_weight_all(fik->m_robot->link("CHEST_JOINT2")->jointId) = 0.01;
    if( fik->m_robot->link("HEAD_JOINT0") != NULL) fik->m_robot->link("HEAD_JOINT0")->llimit = deg2rad(-30);
    if( fik->m_robot->link("HEAD_JOINT0") != NULL) fik->m_robot->link("HEAD_JOINT0")->ulimit = deg2rad(30);
    if( fik->m_robot->link("HEAD_JOINT1") != NULL) fik->m_robot->link("HEAD_JOINT1")->llimit = deg2rad(-45);
    if( fik->m_robot->link("HEAD_JOINT1") != NULL) fik->m_robot->link("HEAD_JOINT1")->ulimit = deg2rad(10);
    if( fik->m_robot->link("RARM_JOINT2") != NULL) fik->m_robot->link("RARM_JOINT2")->ulimit = deg2rad(-45);//脇内側の干渉回避
    if( fik->m_robot->link("LARM_JOINT2") != NULL) fik->m_robot->link("LARM_JOINT2")->llimit = deg2rad(45);
    if( fik->m_robot->link("RLEG_JOINT3") != NULL) fik->m_robot->link("RLEG_JOINT3")->llimit = deg2rad(40);//膝伸びきり防止のため
    if( fik->m_robot->link("LLEG_JOINT3") != NULL) fik->m_robot->link("LLEG_JOINT3")->llimit = deg2rad(40);
    if( fik->m_robot->link("CHEST_JOINT2") != NULL) fik->dq_weight_all(fik->m_robot->link("CHEST_JOINT2")->jointId) = 0;//実機修理中

    if( fik->m_robot->link("CHEST_Y") != NULL) fik->dq_weight_all(fik->m_robot->link("CHEST_Y")->jointId) = 0.1;//K
    if( fik->m_robot->link("CHEST_P") != NULL) fik->dq_weight_all(fik->m_robot->link("CHEST_P")->jointId) = 0.1;
    if( fik->m_robot->link("R_KNEE_P") != NULL) fik->m_robot->link("R_KNEE_P")->llimit = deg2rad(15);
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

    fik->q_ref.head(m_qRef.data.length()) = hrp::to_dvector(m_qRef.data);//あえてseqからのbaselink poseは信用しない

    for(int i=0; i<fik->m_robot->numJoints(); i++){
        if(!has(wbms->wp.use_joints, fik->m_robot->joint(i)->name)){
            fik->dq_weight_all(i) = 0;
            fik->m_robot->joint(i)->q = m_qRef.data[i];
        }
    }


    if(fik->m_robot->name().find("JAXON") != std::string::npos){
        for(int i=0; i<fik->m_robot->numJoints(); i++){
            if(fik->m_robot->joint(i)->name.find("ARM") != std::string::npos){
                fik->q_ref_constraint_weight(i) = 1e-3;//腕だけ
            }
        }
    }

    const int IK_MAX_LOOP = 1;
    int loop_result = fik->solveFullbodyIKLoop(ikc_list, IK_MAX_LOOP);
}

void WholeBodyMasterSlave::smoothingJointAngles(hrp::BodyPtr _robot, hrp::BodyPtr _robot_safe){
    double goal_time = 0.0;
    const double min_goal_time_offset = 0.3;

    static hrp::dvector ans_state_vel = hrp::dvector::Zero(fik->numStates());

    hrp::dvector estimated_times_from_vel_limit = (hrp::getRobotStateVec(_robot) - hrp::getRobotStateVec(_robot_safe)).array().abs() / avg_q_vel.array();
    hrp::dvector estimated_times_from_acc_limit = ans_state_vel.array().abs() / avg_q_acc.array();
    double longest_estimated_time_from_vel_limit = estimated_times_from_vel_limit.maxCoeff();
    double longest_estimated_time_from_acc_limit = estimated_times_from_acc_limit.maxCoeff();
    goal_time = hrp::Vector3(longest_estimated_time_from_vel_limit, longest_estimated_time_from_acc_limit, min_goal_time_offset).maxCoeff();

    q_ip->setGoal(hrp::getRobotStateVec(_robot).data(), goal_time, true);
    double tmp[fik->numStates()], tmpv[fik->numStates()];
    if (!q_ip->isEmpty() ){  q_ip->get(tmp, tmpv, true);}
    hrp::dvector ans_state = Eigen::Map<hrp::dvector>(tmp, fik->numStates());
    ans_state_vel = Eigen::Map<hrp::dvector>(tmpv, fik->numStates());

    hrp::setRobotStateVec(_robot_safe, ans_state);

    _robot_safe->calcForwardKinematics();
}


bool WholeBodyMasterSlave::startWholeBodyMasterSlave(){
    if(mode.now() == MODE_IDLE){
        RTC_INFO_STREAM("startWholeBodyMasterSlave");
        mode.setNextMode(MODE_SYNC_TO_HC);
        return true;
    }else{
        RTC_WARN_STREAM("Invalid context to startWholeBodyMasterSlave");
        return false;
    }
}


bool WholeBodyMasterSlave::pauseWholeBodyMasterSlave(){
    if(mode.now() == MODE_WBMS){
        RTC_INFO_STREAM("pauseWholeBodyMasterSlave");
        mode.setNextMode(MODE_PAUSE);
        return true;
    }else{
        RTC_WARN_STREAM("Invalid context to pauseWholeBodyMasterSlave");
        return false;
    }
}


bool WholeBodyMasterSlave::resumeWholeBodyMasterSlave(){
    if(mode.now() == MODE_PAUSE){
        RTC_INFO_STREAM("resumeWholeBodyMasterSlave");
        mode.setNextMode(MODE_WBMS);
        return true;
    }else{
        RTC_WARN_STREAM("Invalid context to resumeWholeBodyMasterSlave");
        return false;
    }
}


bool WholeBodyMasterSlave::stopWholeBodyMasterSlave(){
    if(mode.now() == MODE_WBMS || mode.now() == MODE_PAUSE ){
        RTC_INFO_STREAM("stopWholeBodyMasterSlave");
        mode.setNextMode(MODE_SYNC_TO_IDLE);
        return true;
    }else{
        RTC_WARN_STREAM("Invalid context to stopWholeBodyMasterSlave");
        return false;
    }
}
namespace hrp{
//    hrp::Vector2 to_Vector2(const OpenHRP::WholeBodyMasterSlaveService::DblSequence2& in){ return (hrp::Vector2()<< in[0],in[1]).finished(); }
//    hrp::Vector2 to_Vector3(const OpenHRP::WholeBodyMasterSlaveService::DblSequence3& in){ return hrp::dvector::Map(in.get_buffer(), in.length()); }
//    hrp::Vector2 to_Vector4(const OpenHRP::WholeBodyMasterSlaveService::DblSequence4& in){ return (hrp::Vector4()<< in[0],in[1]).finished(); }
    OpenHRP::WholeBodyMasterSlaveService::DblSequence2 to_DblSequence2(const hrp::Vector2& in){
        OpenHRP::WholeBodyMasterSlaveService::DblSequence2 ret; ret.length(in.size()); hrp::Vector2::Map(ret.get_buffer()) = in; return ret; }
    OpenHRP::WholeBodyMasterSlaveService::DblSequence3 to_DblSequence3(const hrp::Vector3& in){
        OpenHRP::WholeBodyMasterSlaveService::DblSequence3 ret; ret.length(in.size()); hrp::Vector3::Map(ret.get_buffer()) = in; return ret; }
    OpenHRP::WholeBodyMasterSlaveService::DblSequence4 to_DblSequence4(const hrp::Vector4& in){
        OpenHRP::WholeBodyMasterSlaveService::DblSequence4 ret; ret.length(in.size()); hrp::Vector4::Map(ret.get_buffer()) = in; return ret; }
}

bool WholeBodyMasterSlave::setParams(const OpenHRP::WholeBodyMasterSlaveService::WholeBodyMasterSlaveParam& i_param){
    RTC_INFO_STREAM("setWholeBodyMasterSlaveParam");
    wbms->wp.auto_com_mode                      = i_param.auto_com_mode;
    wbms->wp.additional_double_support_time     = i_param.additional_double_support_time;
    wbms->wp.auto_com_foot_move_detect_height   = i_param.auto_com_foot_move_detect_height;
    wbms->wp.base_to_hand_min_distance          = i_param.base_to_hand_min_distance;
    wbms->wp.capture_point_extend_ratio         = i_param.capture_point_extend_ratio;
    wbms->wp.foot_collision_avoidance_distance  = i_param.foot_collision_avoidance_distance;
    wbms->wp.human_to_robot_ratio               = i_param.human_to_robot_ratio;
    wbms->wp.max_double_support_width           = i_param.max_double_support_width;
    wbms->wp.upper_body_rmc_ratio               = i_param.upper_body_rmc_ratio;
    wbms->wp.single_foot_zmp_safety_distance    = i_param.single_foot_zmp_safety_distance;
    wbms->wp.swing_foot_height_offset           = i_param.swing_foot_height_offset;
    wbms->wp.com_offset                         = hrp::Vector3::Map(i_param.com_offset.get_buffer());
    wbms->wp.actual_foot_vert_fbio              = hrp::Vector4::Map(i_param.actual_foot_vert_fbio.get_buffer());
    wbms->wp.safety_foot_vert_fbio              = hrp::Vector4::Map(i_param.safety_foot_vert_fbio.get_buffer());
    wbms->wp.use_joints                         = hrp::to_string_vector(i_param.use_joints);
    wbms->wp.use_targets                        = hrp::to_string_vector(i_param.use_targets);
    return true;
}


bool WholeBodyMasterSlave::getParams(OpenHRP::WholeBodyMasterSlaveService::WholeBodyMasterSlaveParam& i_param){
    RTC_INFO_STREAM("getWholeBodyMasterSlaveParam");
    i_param.auto_com_mode                       = wbms->wp.auto_com_mode;
    i_param.additional_double_support_time      = wbms->wp.additional_double_support_time;
    i_param.auto_com_foot_move_detect_height    = wbms->wp.auto_com_foot_move_detect_height;
    i_param.base_to_hand_min_distance           = wbms->wp.base_to_hand_min_distance;
    i_param.capture_point_extend_ratio          = wbms->wp.capture_point_extend_ratio;
    i_param.foot_collision_avoidance_distance   = wbms->wp.foot_collision_avoidance_distance;
    i_param.human_to_robot_ratio                = wbms->wp.human_to_robot_ratio;
    i_param.max_double_support_width            = wbms->wp.max_double_support_width;
    i_param.upper_body_rmc_ratio                = wbms->wp.upper_body_rmc_ratio;
    i_param.single_foot_zmp_safety_distance     = wbms->wp.single_foot_zmp_safety_distance;
    i_param.swing_foot_height_offset            = wbms->wp.swing_foot_height_offset;
    i_param.com_offset                          = hrp::to_DblSequence3(wbms->wp.com_offset);
    i_param.actual_foot_vert_fbio               = hrp::to_DblSequence4(wbms->wp.actual_foot_vert_fbio);
    i_param.safety_foot_vert_fbio               = hrp::to_DblSequence4(wbms->wp.safety_foot_vert_fbio);
    i_param.use_joints                          = hrp::to_StrSequence(wbms->wp.use_joints);
    i_param.use_targets                         = hrp::to_StrSequence(wbms->wp.use_targets);
    return true;
}


RTC::ReturnCode_t WholeBodyMasterSlave::onActivated(RTC::UniqueId ec_id){ RTC_INFO_STREAM("onActivated(" << ec_id << ")"); return RTC::RTC_OK; }
RTC::ReturnCode_t WholeBodyMasterSlave::onDeactivated(RTC::UniqueId ec_id){ RTC_INFO_STREAM("onDeactivated(" << ec_id << ")"); return RTC::RTC_OK; }
RTC::ReturnCode_t WholeBodyMasterSlave::onFinalize(){ return RTC::RTC_OK; }

extern "C"{
    void WholeBodyMasterSlaveInit(RTC::Manager* manager) {
        RTC::Properties profile(WholeBodyMasterSlave_spec);
        manager->registerFactory(profile, RTC::Create<WholeBodyMasterSlave>, RTC::Delete<WholeBodyMasterSlave>);
    }
};
