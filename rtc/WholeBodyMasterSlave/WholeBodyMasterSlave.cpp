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
    m_delayCheckPacketInboundIn("delay_check_packet_inbound", m_delayCheckPacket),
    m_delayCheckPacketOutboundOut("delay_check_packet_outbound", m_delayCheckPacket),
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
    addInPort("delay_check_packet_inbound", m_delayCheckPacketInboundIn);
    addOutPort("delay_check_packet_outbound", m_delayCheckPacketOutboundOut);
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
    wbms->legged = ( has(ee_names,"lleg") || has(ee_names,"rleg") );
    RTC_INFO_STREAM("setup mode as legged robot ? = " << wbms->legged);

    for (int st; st<robot_for_ik->numSensorTypes(); st++){
        RTC_INFO_STREAM("SensorType : "<<st);
        for (int s; s<robot_for_ik->numSensors(st); s++){
            RTC_INFO_STREAM("Sensor id : "<<s);
            robot_for_ik->sensor(st,s)->putInformation(std::cerr);
        }
    }
    
    sccp = boost::shared_ptr<CapsuleCollisionChecker>(new CapsuleCollisionChecker(fik->m_robot));
    RTC_INFO_STREAM("setup main function class finished");

    // allocate memory for outPorts
    m_qRef.data.length(fik->m_robot->numJoints()); // is really needed?
    if(prop["seq_optional_data_dim"].empty()){
        RTC_WARN_STREAM("No seq_optional_data_dim is set ! Exit !!!");
        return RTC::RTC_ERROR;
    }
    coil::stringTo(optionalDataLength, prop["seq_optional_data_dim"].c_str());

    output_ratio = 0.0;
    t_ip = new interpolator(1, m_dt, interpolator::HOFFARBIB, 1);
    t_ip->clear();
    t_ip->set(&output_ratio);
    q_ip = new interpolator(fik->numStates(), m_dt, interpolator::CUBICSPLINE, 1); // or HOFFARBIB, QUINTICSPLINE
    q_ip->clear();
    if(fik->m_robot->name().find("JAXON") != std::string::npos){
        avg_q_vel = hrp::dvector::Constant(fik->numStates(), 1.0);
        avg_q_vel.head(12).fill(4.0); // leg
    }else{
        avg_q_vel = hrp::dvector::Constant(fik->numStates(), 1.0); // all joint max avarage vel = 1.0 rad/s
    }
    if(!wbms->legged){ avg_q_vel = hrp::dvector::Constant(fik->numStates(), 10.0); } // rapid manip 
    avg_q_acc = hrp::dvector::Constant(fik->numStates(), 16.0); // all joint max avarage acc = 16.0 rad/s^2
    avg_q_vel.tail(6).fill(std::numeric_limits<double>::max()); // no limit for base link vel
    avg_q_acc.tail(6).fill(std::numeric_limits<double>::max()); // no limit for base link acc
    RTC_INFO_STREAM("setup interpolator finished");

    ref_zmp_filter.resize(XYZ);
    ref_zmp_filter.setParameter(100, 1/m_dt, Q_BUTTERWORTH);

    tgt_names = ee_names;
    tgt_names.push_back("com");
    tgt_names.push_back("head");
    tgt_names.push_back("rhand");
    tgt_names.push_back("lhand");
    tgt_names.push_back("rfloor");
    tgt_names.push_back("lfloor");

    for (auto ee : ee_names) {
        const std::string n = "slave_"+ee+"_wrench";
        m_slaveEEWrenchesOut[ee] = OTDS_Ptr(new RTC::OutPort<RTC::TimedDoubleSeq>(n.c_str(), m_slaveEEWrenches[ee]));
        registerOutPort(n.c_str(), *m_slaveEEWrenchesOut[ee]);
        RTC_INFO_STREAM(" registerOutPort " << n);
    }
    for (auto ee : ee_names) {
        const std::string n = "master_"+ee+"_wrench";
        m_masterEEWrenchesIn[ee] = ITDS_Ptr(new RTC::InPort<RTC::TimedDoubleSeq>(n.c_str(), m_masterEEWrenches[ee]));
        registerInPort(n.c_str(), *m_masterEEWrenchesIn[ee]);
        RTC_INFO_STREAM(" registerInPort " << n);
    }
    ///////////////////
    for (auto tgt : tgt_names) {
        const std::string n = "master_"+tgt+"_pose";
        m_masterTgtPosesIn[tgt] = ITP3_Ptr(new RTC::InPort<RTC::TimedPose3D>(n.c_str(), m_masterTgtPoses[tgt]));
        registerInPort(n.c_str(), *m_masterTgtPosesIn[tgt]);
        RTC_INFO_STREAM(" registerInPort " << n);
    }
    for (auto tgt : tgt_names) {
        const std::string n = "slave_"+tgt+"_pose";
        m_slaveTgtPosesOut[tgt] = OTP3_Ptr(new RTC::OutPort<RTC::TimedPose3D>(n.c_str(), m_slaveTgtPoses[tgt]));
        registerOutPort(n.c_str(), *m_slaveTgtPosesOut[tgt]);
        RTC_INFO_STREAM(" registerOutPort " << n);
    }
    //////////////
    for (auto ee : ee_names) {
        const std::string n = "local_"+ee+"_wrench";
        m_localEEWrenchesIn[ee] = ITDS_Ptr(new RTC::InPort<RTC::TimedDoubleSeq>(n.c_str(), m_localEEWrenches[ee]));
        registerInPort(n.c_str(), *m_localEEWrenchesIn[ee]);
        RTC_INFO_STREAM(" registerInPort " << n);
    }

    for(auto ee : ee_names){
        ee_f_filter[ee].resize(XYZ);
        ee_f_filter[ee].setParameter(1, 1/m_dt, Q_BUTTERWORTH);
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
                for (int i; i<_robot->numJoints(); i++){    RTC_WARN_STREAM( i << " : " << _robot->joint(i)->name); }
                for (int i; i<_robot->numLinks(); i++){     RTC_WARN_STREAM( i << " : " << _robot->link(i)->name); }
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
    if(m_qRefIn.isNew()                     ){ m_qRefIn.read(); }
    if(m_qActIn.isNew()                     ){ m_qActIn.read(); }
    if(m_basePosIn.isNew()                  ){ m_basePosIn.read(); }
    if(m_baseRpyIn.isNew()                  ){ m_baseRpyIn.read(); }
    if(m_zmpIn.isNew()                      ){ m_zmpIn.read(); }
    if(m_optionalDataIn.isNew()             ){ m_optionalDataIn.read(); }
    if(m_delayCheckPacketInboundIn.isNew()  ){ m_delayCheckPacketInboundIn.read(); m_delayCheckPacketOutboundOut.write();}
    for (auto ee : ee_names) {
        if(m_localEEWrenchesIn[ee]->isNew()){ m_localEEWrenchesIn[ee]->read(); }
    }
    if(wbms->legged){
        wbms->act_rs.act_foot_wrench[0] = hrp::to_dvector(m_localEEWrenches["rleg"].data);
        wbms->act_rs.act_foot_wrench[1] = hrp::to_dvector(m_localEEWrenches["lleg"].data);
        wbms->act_rs.act_foot_pose[0] = ee_ikc_map["rleg"].getCurrentTargetPose(m_robot_vsafe);
        wbms->act_rs.act_foot_pose[1] = ee_ikc_map["lleg"].getCurrentTargetPose(m_robot_vsafe);
    }
    // calc actual state
    std::map<std::string, std::string> to_sname{{"lleg", "lfsensor"}, {"rleg", "rfsensor"}, {"larm", "lhsensor"}, {"rarm", "rhsensor"}};
    hrp::setQAll(m_robot_act, hrp::to_dvector(m_qAct.data));
    m_robot_act->rootLink()->p = m_robot_vsafe->rootLink()->p;
    m_robot_act->rootLink()->R = m_robot_vsafe->rootLink()->R;
    m_robot_act->calcForwardKinematics();
    for(auto ee : ee_names){
        hrp::ForceSensor* sensor = m_robot_act->sensor<hrp::ForceSensor>(to_sname[ee]);
        hrp::dvector6 w_ee_wld = hrp::dvector6::Zero();
        if(sensor){
            hrp::Matrix33 sensorR_wld = sensor->link->R * sensor->localR;
            hrp::Matrix33 sensorR_from_base = m_robot_act->rootLink()->R.transpose() * sensorR_wld;
            const hrp::Vector3 f_sensor_wld = sensorR_from_base * hrp::to_dvector(m_localEEWrenches[ee].data).head(3);
            const hrp::Vector3 t_sensor_wld = sensorR_from_base * hrp::to_dvector(m_localEEWrenches[ee].data).tail(3);
            const hrp::Vector3 sensor_to_ee_vec_wld = ee_ikc_map[ee].getCurrentTargetPos(m_robot_act) - sensor->link->p;
            w_ee_wld << f_sensor_wld, t_sensor_wld - sensor_to_ee_vec_wld.cross(f_sensor_wld);
        }
        m_slaveEEWrenches[ee].data = hrp::to_DoubleSeq( w_ee_wld );
        m_slaveEEWrenches[ee].tm = m_qRef.tm;
        m_slaveEEWrenchesOut[ee]->write();
        m_slaveTgtPoses[ee].data = hrp::to_Pose3D(ee_ikc_map[ee].getCurrentTargetPose(m_robot_act));
        m_slaveTgtPoses[ee].tm = m_qRef.tm;
        m_slaveTgtPosesOut[ee]->write();
    }
    m_slaveTgtPoses["com"].data = hrp::to_Pose3D( (hrp::dvector6()<<m_robot_act->calcCM(),0,0,0).finished());
    m_slaveTgtPoses["com"].tm = m_qRef.tm;
    m_slaveTgtPosesOut["com"]->write();

    static_balancing_com_offset.fill(0);
    for(auto ee : {"larm","rarm"}){
        const hrp::Vector3 use_f = hrp::Vector3::UnitZ() * m_slaveEEWrenches[ee].data[Z];
        hrp::Vector3 use_f_filtered = ee_f_filter[ee].passFilter(use_f);
        const hrp::Vector3 ee_pos_from_com = ee_ikc_map[ee].getCurrentTargetPos(m_robot_vsafe) - wbms->rp_ref_out.tgt[com].abs.p;
        static_balancing_com_offset.head(XY) += - ee_pos_from_com.head(XY) * use_f_filtered(Z) / (-m_robot_vsafe->totalMass() * G);
    }
    if(loop%500==0)dbgv(static_balancing_com_offset);

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

    if( mode.now() != MODE_PAUSE ){ // stop updating input when MODE_PAUSE
        for (auto tgt : tgt_names){
            if (m_masterTgtPosesIn[tgt]->isNew()){
                m_masterTgtPosesIn[tgt]->read();
                if(tgt == "lhand" || tgt == "rhand" || tgt == "lfloor" || tgt == "rfloor" ){
                    // nothing
                }else{
                    wbms->hp_wld_raw.stgt(tgt).abs = hrp::to_Pose3(m_masterTgtPoses[tgt].data);
                }
            }
        }
        for (auto ee : ee_names){
            if (m_masterEEWrenchesIn[ee]->isNew()){
                m_masterEEWrenchesIn[ee]->read();
                wbms->hp_wld_raw.stgt(ee).w = hrp::to_dvector(m_masterEEWrenches[ee].data);
            }
        }
        if (m_actCPIn.isNew()){     m_actCPIn.read();   rel_act_cp  = hrp::to_Vector3(m_actCP.data);}
        if (m_actZMPIn.isNew()){    m_actZMPIn.read();  rel_act_zmp = hrp::to_Vector3(m_actZMP.data);}
    }else{
        for (auto tgt : {"lhand", "rhand"}){ // PAUSE中もボタン読み込みのためにこれらだけは更新・・・
            if (m_masterTgtPosesIn[tgt]->isNew()){ m_masterTgtPosesIn[tgt]->read(); }
        }
    }
    addTimeReport("InPort");

    ///// Button start
    static int button1_flag_count = 0, button2_flag_count = 0;
    if(m_masterTgtPoses["rhand"].data.position.x > 0 && m_masterTgtPoses["lhand"].data.position.x > 0  ){ button1_flag_count++; }else{ button1_flag_count = 0; }
    if(m_masterTgtPoses["rhand"].data.position.x > 0 && m_masterTgtPoses["lhand"].data.position.x <= 0 ){ button2_flag_count++; }else{ button2_flag_count = 0; }
    if(button1_flag_count > 2.0 / m_dt){// keep pless both button for 2s to start
        if(mode.now() == MODE_IDLE){
            RTC_INFO_STREAM("startWholeBodyMasterSlave by Button");
            startWholeBodyMasterSlave();
        }
        if(mode.now() == MODE_WBMS || mode.now() == MODE_PAUSE ){
            RTC_INFO_STREAM("stopWholeBodyMasterSlave by Button");
            stopWholeBodyMasterSlave();
        }
        button1_flag_count = 0;
    }
    if(button2_flag_count > 2.0 / m_dt){// keep pless both button for 2s to start
        if(mode.now() == MODE_WBMS){
            RTC_INFO_STREAM("pauseWholeBodyMasterSlave by Button");
            pauseWholeBodyMasterSlave();
        }
        if(mode.now() == MODE_PAUSE ){
            RTC_INFO_STREAM("resumeWholeBodyMasterSlave by Button");
            resumeWholeBodyMasterSlave();
        }
        button2_flag_count = 0;
    }

    processTransition();
    mode.update();

    if (mode.isRunning()) {
        if(mode.isInitialize()){
            preProcessForWholeBodyMasterSlave();
            idsb.setInitState(fik->m_robot, m_dt);//逆動力学初期化
        }
        wbms->update();//////HumanSynchronizerの主要処理
        if(DEBUGP)RTC_INFO_STREAM(wbms->rp_ref_out);
        addTimeReport("MainFunc");

        solveFullbodyIK(wbms->rp_ref_out);
        addTimeReport("IK");

        // RHP finger
        if(fik->m_robot->name() == "RHP4B"){
            const double hand_max_vel = M_PI/0.4;//0.4s for 180deg
            {
                const double trigger_in = MINMAX_LIMITED(m_masterTgtPoses["lhand"].data.position.y, 0, 1);
                double tgt_hand_q = (-29.0 + (124.0-(-29.0))*trigger_in ) /180.0*M_PI;
                fik->m_robot->link("L_HAND")->q = MINMAX_LIMITED(tgt_hand_q, fik->m_robot->link("L_HAND")->llimit, fik->m_robot->link("L_HAND")->ulimit);
                avg_q_vel(fik->m_robot->link("L_HAND")->jointId) = hand_max_vel;
            }{
                const double trigger_in = MINMAX_LIMITED(m_masterTgtPoses["rhand"].data.position.y, 0, 1);
                double tgt_hand_q = (-29.0 + (124.0-(-29.0))*trigger_in ) /180.0*M_PI;
                fik->m_robot->link("R_HAND")->q = MINMAX_LIMITED(tgt_hand_q, fik->m_robot->link("R_HAND")->llimit, fik->m_robot->link("R_HAND")->ulimit);
                avg_q_vel(fik->m_robot->link("R_HAND")->jointId) = hand_max_vel;
            }
        }

        smoothingJointAngles(fik->m_robot, m_robot_vsafe);

        hrp::Vector3 com = m_robot_vsafe->calcCM();
        static hrp::Vector3 com_old = com;
        static hrp::Vector3 com_old_old = com_old;
        hrp::Vector3 com_acc = (com - 2*com_old + com_old_old)/(m_dt*m_dt);
        hrp::Vector3 ref_zmp; ref_zmp << com.head(XY)-(com(Z)/G)*com_acc.head(XY), 0;
        ref_zmp.head(XY) -= static_balancing_com_offset.head(XY);
//        if(mode.isInitialize()){ ref_zmp_filter.reset(ref_zmp); }
//        ref_zmp = ref_zmp_filter.passFilter(ref_zmp);
        com_old_old = com_old;
        com_old = com;
        wbms->act_rs.ref_com = com;
        wbms->act_rs.ref_zmp = ref_zmp;
        wbms->act_rs.st_zmp = m_robot_vsafe->rootLink()->p + m_robot_vsafe->rootLink()->R * rel_act_zmp;

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
        if(wbms->legged){
            m_optionalData.data[contact_states_index_map["rleg"]] = m_optionalData.data[optionalDataLength/2 + contact_states_index_map["rleg"]] = wbms->rp_ref_out.tgt[rf].is_contact();
            m_optionalData.data[contact_states_index_map["lleg"]] = m_optionalData.data[optionalDataLength/2 + contact_states_index_map["lleg"]] = wbms->rp_ref_out.tgt[lf].is_contact();
        }
        addTimeReport("SetOutPut");

    }
    wbms->baselinkpose.p = fik->m_robot->rootLink()->p;
    wbms->baselinkpose.R = fik->m_robot->rootLink()->R;
    // send back auto detected floor height
    
    if(wbms->legged){
        for(std::string lr : {"l","r"}){
            const hrp::Vector3 floor_pos = wbms->rp_ref_out.stgt(lr+"leg").cnt.p - wbms->rp_ref_out.stgt(lr+"leg").offs.p;
            m_slaveTgtPoses[lr+"floor"].data    = hrp::to_Pose3D( (hrp::dvector6()<<floor_pos,0,0,0).finished());
            m_slaveTgtPoses[lr+"floor"].tm      = m_qRef.tm;
            m_slaveTgtPosesOut[lr+"floor"]->write();
        }
    }

    // write
    m_qOut.write();
    m_basePosOut.write();
    m_baseRpyOut.write();
    m_zmpOut.write();
    m_optionalDataOut.write();
    addTimeReport("OutPort");
    if(DEBUGP)RTC_INFO_STREAM(time_report_str);
    if(DEBUGP)RTC_INFO_STREAM(wbms->ws);
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
    fik->m_robot->rootLink()->p = hrp::to_Vector3(m_basePos.data);
    hrp::setQAll(fik->m_robot, hrp::to_dvector(m_qRef.data));
    fik->m_robot->calcForwardKinematics();
    double current_foot_height_from_world = wbms->legged ? ::min(ee_ikc_map["rleg"].getCurrentTargetPos(fik->m_robot)(Z), ee_ikc_map["lleg"].getCurrentTargetPos(fik->m_robot)(Z)) : 0;
    RTC_INFO_STREAM("current_foot_height_from_world = "<<current_foot_height_from_world<<" will be modified to 0");
    for(auto b : {fik->m_robot, m_robot_vsafe}){//初期姿勢でBodyをFK
        hrp::setRobotStateVec(b, hrp::to_dvector(m_qRef.data), hrp::to_Vector3(m_basePos.data) - hrp::Vector3::UnitZ() * current_foot_height_from_world, hrp::rotFromRpy(hrp::to_Vector3(m_baseRpy.data)));
        b->calcForwardKinematics();
    }
    fik->q_ref = hrp::getRobotStateVec(fik->m_robot);
    q_ip->set(fik->q_ref.data());
    wbms->initializeRequest(fik->m_robot, ee_ikc_map);
}


void WholeBodyMasterSlave::solveFullbodyIK(HumanPose& ref){
    std::vector<IKConstraint> ikc_list;
    if(wbms->legged){ // free baselink, lleg, rleg, larm, rarm setting
        {
            IKConstraint tmp;
            tmp.target_link_name    = fik->m_robot->rootLink()->name;
            tmp.localPos            = hrp::Vector3::Zero();
            tmp.localR              = hrp::Matrix33::Identity();
            tmp.targetPos           = hrp::to_Vector3(m_basePos.data);// will be ignored by selection_vec
            tmp.targetRpy           = ref.stgt("com").abs.rpy();
            tmp.constraint_weight   << 0, 0, 0, 0.1, 0.1, 0.1;
            tmp.rot_precision       = deg2rad(3);
            ikc_list.push_back(tmp);
        }
        for(auto leg : {"lleg","rleg"}){
            if(has(wbms->wp.use_targets, leg)){
                IKConstraint tmp;
                tmp.target_link_name    = ee_ikc_map[leg].target_link_name;
                tmp.localPos            = ee_ikc_map[leg].localPos;
                tmp.localR              = ee_ikc_map[leg].localR;
                tmp.targetPos           = ref.stgt(leg).abs.p;
                tmp.targetRpy           = ref.stgt(leg).abs.rpy();
                tmp.constraint_weight   = wbms->rp_ref_out.tgt[rf].is_contact() ? hrp::dvector6::Constant(3) : hrp::dvector6::Constant(0.1);
                ikc_list.push_back(tmp);
            }
        }
        for(auto arm : {"larm","rarm"}){
            if(has(wbms->wp.use_targets, arm)){
                IKConstraint tmp;
                tmp.target_link_name    = ee_ikc_map[arm].target_link_name;
                tmp.localPos            = ee_ikc_map[arm].localPos;
                tmp.localR              = ee_ikc_map[arm].localR;
                tmp.targetPos           = ref.stgt(arm).abs.p;
                tmp.targetRpy           = ref.stgt(arm).abs.rpy();
                tmp.constraint_weight   = hrp::dvector6::Constant(0.1);
                tmp.pos_precision       = 3e-3;
                tmp.rot_precision       = deg2rad(3);
                ikc_list.push_back(tmp);
            }
        }
        if(has(wbms->wp.use_targets, "com")){
            IKConstraint tmp;
            tmp.target_link_name    = "COM";
            tmp.localPos            = hrp::Vector3::Zero();
            tmp.localR              = hrp::Matrix33::Identity();
            tmp.targetPos           = ref.stgt("com").abs.p + static_balancing_com_offset;// COM height will not be constraint
            tmp.targetRpy           = hrp::Vector3::Zero();//reference angular momentum
            tmp.constraint_weight   << 3,3,0.01,0,0,0;
            ikc_list.push_back(tmp);
        }
    }else{ // fixed baselink, larm, rarm setting
        {
            IKConstraint tmp;
            tmp.target_link_name    = fik->m_robot->rootLink()->name;
            tmp.localPos            = hrp::Vector3::Zero();
            tmp.localR              = hrp::Matrix33::Identity();
            tmp.targetPos           = hrp::to_Vector3(m_basePos.data);// will be ignored by selection_vec
            tmp.targetRpy           = hrp::Vector3::Zero();
            tmp.constraint_weight   << hrp::dvector6::Constant(1);
            tmp.rot_precision       = deg2rad(3);
            ikc_list.push_back(tmp);
        }
        for(auto arm : {"larm","rarm"}){
            if(has(wbms->wp.use_targets, arm)){
                IKConstraint tmp;
                tmp.target_link_name    = ee_ikc_map[arm].target_link_name;
                tmp.localPos            = ee_ikc_map[arm].localPos;
                tmp.localR              = ee_ikc_map[arm].localR;
                tmp.targetPos           = ref.stgt(arm).abs.p;
                tmp.targetRpy           = ref.stgt(arm).abs.rpy();
                tmp.constraint_weight   << 1, 1, 1, 0.1, 0.1, 0.1;
                tmp.pos_precision       = 3e-3;
                tmp.rot_precision       = deg2rad(3);
                ikc_list.push_back(tmp);
            }
        }
    }
    // common head setting
    if(has(wbms->wp.use_targets, "head")){
        if(fik->m_robot->link("HEAD_JOINT1") != NULL){
            IKConstraint tmp;
            tmp.target_link_name = "HEAD_JOINT1";
            tmp.targetRpy = ref.stgt("head").abs.rpy();
            tmp.constraint_weight << 0,0,0,0,0.1,0.1;
            tmp.rot_precision = deg2rad(1);
            ikc_list.push_back(tmp);
        }
        if(fik->m_robot->link("HEAD_P") != NULL){
            IKConstraint tmp;
            tmp.target_link_name = "HEAD_P";
            tmp.targetRpy = ref.stgt("head").abs.rpy();
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

    if( fik->m_robot->link("CHEST_JOINT0") != NULL) fik->dq_weight_all(fik->m_robot->link("CHEST_JOINT0")->jointId) = 1e3;//JAXON
    if( fik->m_robot->link("CHEST_JOINT1") != NULL) fik->dq_weight_all(fik->m_robot->link("CHEST_JOINT1")->jointId) = 1e3;
//    if( fik->m_robot->link("CHEST_JOINT2") != NULL) fik->dq_weight_all(fik->m_robot->link("CHEST_JOINT2")->jointId) = 10;
    if( fik->m_robot->link("CHEST_JOINT2") != NULL) fik->dq_weight_all(fik->m_robot->link("CHEST_JOINT2")->jointId) = 0;//実機修理中

    if( fik->m_robot->link("CHEST_JOINT0") != NULL) fik->m_robot->link("CHEST_JOINT0")->llimit = deg2rad(-8);
    if( fik->m_robot->link("CHEST_JOINT0") != NULL) fik->m_robot->link("CHEST_JOINT0")->ulimit = deg2rad(8);
    if( fik->m_robot->link("CHEST_JOINT1") != NULL) fik->m_robot->link("CHEST_JOINT1")->llimit = deg2rad(1);
    if( fik->m_robot->link("CHEST_JOINT1") != NULL) fik->m_robot->link("CHEST_JOINT1")->ulimit = deg2rad(32);

    if( fik->m_robot->link("HEAD_JOINT0") != NULL) fik->m_robot->link("HEAD_JOINT0")->llimit = deg2rad(-20);
    if( fik->m_robot->link("HEAD_JOINT0") != NULL) fik->m_robot->link("HEAD_JOINT0")->ulimit = deg2rad(20);
    if( fik->m_robot->link("HEAD_JOINT1") != NULL) fik->m_robot->link("HEAD_JOINT1")->llimit = deg2rad(-15);
    if( fik->m_robot->link("HEAD_JOINT1") != NULL) fik->m_robot->link("HEAD_JOINT1")->ulimit = deg2rad(35);

    if( fik->m_robot->link("RARM_JOINT6") != NULL) fik->m_robot->link("RARM_JOINT6")->llimit = deg2rad(-59);
    if( fik->m_robot->link("RARM_JOINT6") != NULL) fik->m_robot->link("RARM_JOINT6")->ulimit = deg2rad(59);
    if( fik->m_robot->link("RARM_JOINT7") != NULL) fik->m_robot->link("RARM_JOINT7")->llimit = deg2rad(-61);
    if( fik->m_robot->link("RARM_JOINT7") != NULL) fik->m_robot->link("RARM_JOINT7")->ulimit = deg2rad(58);

    if( fik->m_robot->link("LARM_JOINT6") != NULL) fik->m_robot->link("LARM_JOINT6")->llimit = deg2rad(-59);
    if( fik->m_robot->link("LARM_JOINT6") != NULL) fik->m_robot->link("LARM_JOINT6")->ulimit = deg2rad(59);
    if( fik->m_robot->link("LARM_JOINT7") != NULL) fik->m_robot->link("LARM_JOINT7")->llimit = deg2rad(-61);
    if( fik->m_robot->link("LARM_JOINT7") != NULL) fik->m_robot->link("LARM_JOINT7")->ulimit = deg2rad(58);

    if( fik->m_robot->link("RARM_JOINT2") != NULL) fik->m_robot->link("RARM_JOINT2")->ulimit = deg2rad(-45);//脇内側の干渉回避
    if( fik->m_robot->link("LARM_JOINT2") != NULL) fik->m_robot->link("LARM_JOINT2")->llimit = deg2rad(45);
    if( fik->m_robot->link("RARM_JOINT2") != NULL) fik->m_robot->link("RARM_JOINT2")->llimit = deg2rad(-89);//肩グルン防止
    if( fik->m_robot->link("LARM_JOINT2") != NULL) fik->m_robot->link("LARM_JOINT2")->ulimit = deg2rad(89);
    if( fik->m_robot->link("RARM_JOINT4") != NULL) fik->m_robot->link("RARM_JOINT4")->ulimit = deg2rad(1);//肘逆折れ
    if( fik->m_robot->link("LARM_JOINT4") != NULL) fik->m_robot->link("LARM_JOINT4")->ulimit = deg2rad(1);
    if( fik->m_robot->link("RLEG_JOINT3") != NULL) fik->m_robot->link("RLEG_JOINT3")->llimit = deg2rad(40);//膝伸びきり防止のため
    if( fik->m_robot->link("LLEG_JOINT3") != NULL) fik->m_robot->link("LLEG_JOINT3")->llimit = deg2rad(40);

    if( fik->m_robot->link("CHEST_Y") != NULL) fik->dq_weight_all(fik->m_robot->link("CHEST_Y")->jointId) = 10;//K
    if( fik->m_robot->link("CHEST_P") != NULL) fik->dq_weight_all(fik->m_robot->link("CHEST_P")->jointId) = 10;
    if( fik->m_robot->link("R_KNEE_P") != NULL) fik->m_robot->link("R_KNEE_P")->llimit = deg2rad(30);//膝伸びきり防止
    if( fik->m_robot->link("L_KNEE_P") != NULL) fik->m_robot->link("L_KNEE_P")->llimit = deg2rad(30);
    // if( fik->m_robot->link("R_WRIST_R") != NULL) fik->m_robot->link("R_WRIST_R")->llimit = deg2rad(-40);
    // if( fik->m_robot->link("L_WRIST_R") != NULL) fik->m_robot->link("L_WRIST_R")->llimit = deg2rad(-40);
    // if( fik->m_robot->link("R_WRIST_R") != NULL) fik->m_robot->link("R_WRIST_R")->ulimit = deg2rad(40);
    // if( fik->m_robot->link("L_WRIST_R") != NULL) fik->m_robot->link("L_WRIST_R")->ulimit = deg2rad(40);
    // if( fik->m_robot->link("R_WRIST_P") != NULL) fik->m_robot->link("R_WRIST_P")->llimit = deg2rad(-40);
    // if( fik->m_robot->link("L_WRIST_P") != NULL) fik->m_robot->link("L_WRIST_P")->llimit = deg2rad(-40);
    // if( fik->m_robot->link("R_WRIST_P") != NULL) fik->m_robot->link("R_WRIST_P")->ulimit = deg2rad(20);
    // if( fik->m_robot->link("L_WRIST_P") != NULL) fik->m_robot->link("L_WRIST_P")->ulimit = deg2rad(20);
    if( fik->m_robot->link("R_WRIST_P") != NULL) fik->m_robot->link("R_WRIST_P")->llimit = deg2rad(-80);
    if( fik->m_robot->link("L_WRIST_P") != NULL) fik->m_robot->link("L_WRIST_P")->llimit = deg2rad(-80);
    if( fik->m_robot->link("R_WRIST_P") != NULL) fik->m_robot->link("R_WRIST_P")->ulimit = deg2rad(45);
    if( fik->m_robot->link("L_WRIST_P") != NULL) fik->m_robot->link("L_WRIST_P")->ulimit = deg2rad(45);
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

    const hrp::dvector estimated_times_from_vel_limit = (hrp::getRobotStateVec(_robot) - hrp::getRobotStateVec(_robot_safe)).array().abs() / avg_q_vel.array();
    const hrp::dvector estimated_times_from_acc_limit = ans_state_vel.array().abs() / avg_q_acc.array();
    const double longest_estimated_time_from_vel_limit = estimated_times_from_vel_limit.maxCoeff();
    const double longest_estimated_time_from_acc_limit = estimated_times_from_acc_limit.maxCoeff();
    goal_time = hrp::Vector3(longest_estimated_time_from_vel_limit, longest_estimated_time_from_acc_limit, min_goal_time_offset).maxCoeff();

    q_ip->setGoal(hrp::getRobotStateVec(_robot).data(), goal_time, true);
    double tmp[fik->numStates()], tmpv[fik->numStates()];
    if (!q_ip->isEmpty() ){  q_ip->get(tmp, tmpv, true);}
    hrp::dvector ans_state = Eigen::Map<hrp::dvector>(tmp, fik->numStates());
    ans_state_vel = Eigen::Map<hrp::dvector>(tmpv, fik->numStates());

    hrp::setRobotStateVec(_robot_safe, ans_state);
    for(int i=0; i<_robot_safe->numJoints(); i++){
        LIMIT_MINMAX(_robot_safe->joint(i)->q, _robot_safe->joint(i)->llimit, _robot_safe->joint(i)->ulimit);
    }

    _robot_safe->calcForwardKinematics();
}


bool WholeBodyMasterSlave::startWholeBodyMasterSlave(){
    if(mode.now() == MODE_IDLE){
        RTC_INFO_STREAM("startWholeBodyMasterSlave");
        mode.setNextMode(MODE_SYNC_TO_WBMS);
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
    wbms->wp.auto_floor_h_mode                  = i_param.auto_floor_h_mode;
    wbms->wp.auto_foot_landing_by_act_cp        = i_param.auto_foot_landing_by_act_cp;
    wbms->wp.auto_foot_landing_by_act_zmp       = i_param.auto_foot_landing_by_act_zmp;
    wbms->wp.additional_double_support_time     = i_param.additional_double_support_time;
    wbms->wp.auto_com_foot_move_detect_height   = i_param.auto_com_foot_move_detect_height;
    wbms->wp.auto_floor_h_detect_fz             = i_param.auto_floor_h_detect_fz;
    wbms->wp.auto_floor_h_reset_fz              = i_param.auto_floor_h_reset_fz;
    wbms->wp.base_to_hand_min_distance          = i_param.base_to_hand_min_distance;
    wbms->wp.capture_point_extend_ratio         = i_param.capture_point_extend_ratio;
    wbms->wp.com_filter_cutoff_hz               = i_param.com_filter_cutoff_hz;
    wbms->wp.foot_collision_avoidance_distance  = i_param.foot_collision_avoidance_distance;
    wbms->wp.foot_landing_vel                   = i_param.foot_landing_vel;
    wbms->wp.force_double_support_com_h         = i_param.force_double_support_com_h;
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
    wbms->wp.CheckSafeLimit();
    return true;
}


bool WholeBodyMasterSlave::getParams(OpenHRP::WholeBodyMasterSlaveService::WholeBodyMasterSlaveParam& i_param){
    RTC_INFO_STREAM("getWholeBodyMasterSlaveParam");
    i_param.auto_com_mode                       = wbms->wp.auto_com_mode;
    i_param.auto_floor_h_mode                   = wbms->wp.auto_floor_h_mode;
    i_param.auto_foot_landing_by_act_cp         = wbms->wp.auto_foot_landing_by_act_cp;
    i_param.auto_foot_landing_by_act_zmp        = wbms->wp.auto_foot_landing_by_act_zmp;
    i_param.additional_double_support_time      = wbms->wp.additional_double_support_time;
    i_param.auto_com_foot_move_detect_height    = wbms->wp.auto_com_foot_move_detect_height;
    i_param.auto_floor_h_detect_fz              = wbms->wp.auto_floor_h_detect_fz;
    i_param.auto_floor_h_reset_fz               = wbms->wp.auto_floor_h_reset_fz;
    i_param.base_to_hand_min_distance           = wbms->wp.base_to_hand_min_distance;
    i_param.capture_point_extend_ratio          = wbms->wp.capture_point_extend_ratio;
    i_param.com_filter_cutoff_hz                = wbms->wp.com_filter_cutoff_hz;
    i_param.foot_collision_avoidance_distance   = wbms->wp.foot_collision_avoidance_distance;
    i_param.foot_landing_vel                    = wbms->wp.foot_landing_vel;
    i_param.force_double_support_com_h          = wbms->wp.force_double_support_com_h;
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
