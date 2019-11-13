#include "HapticController.h"

#define DEBUGP (loop%200==0)
#define DEBUGP_ONCE (loop==0)

static const char* HapticController_spec[] = {
        "implementation_id", "HapticController",
        "type_name",         "HapticController",
        "description",       "hapticcontroller component",
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

HapticController::HapticController(RTC::Manager* manager) : RTC::DataFlowComponentBase(manager),
    m_qRefIn("qRef", m_qRef),
    m_qActIn("qAct", m_qAct),
    m_dqActIn("dqAct", m_dqAct),
    m_qOut("q", m_qRef),
    m_tauOut("tau", m_tau),
    m_teleopOdomOut("teleopOdom", m_teleopOdom),
    m_debugDataOut("debugData", m_debugData),
    m_delayCheckPacketInboundIn("delay_check_packet_inbound", m_delayCheckPacket),
    m_delayCheckPacketOutboundOut("delay_check_packet_outbound", m_delayCheckPacket),
    m_HapticControllerServicePort("HapticControllerService"),
    m_debugLevel(0)
{
    m_service0.hapticcontroller(this);
}

HapticController::~HapticController(){}

RTC::ReturnCode_t HapticController::onInitialize(){
    RTC_INFO_STREAM("onInitialize()");
    bindParameter("debugLevel", m_debugLevel, "0");
    addInPort("qRef", m_qRefIn);
    addInPort("qAct", m_qActIn);
    addInPort("dqAct", m_dqActIn);
    addOutPort("q", m_qOut);
    addOutPort("tau", m_tauOut);
    addOutPort("teleopOdom", m_teleopOdomOut);
    addOutPort("debugData", m_debugDataOut);
    addInPort("delay_check_packet_inbound", m_delayCheckPacketInboundIn);
    addOutPort("delay_check_packet_outbound", m_delayCheckPacketOutboundOut);
    m_HapticControllerServicePort.registerProvider("service0", "HapticControllerService", m_service0);
    addPort(m_HapticControllerServicePort);

    RTC::Properties& prop = getProperties();
    coil::stringTo(m_dt, prop["dt"].c_str());
    RTC::Manager& rtcManager = RTC::Manager::instance();
    std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
    int comPos = nameServer.find(",");
    if (comPos < 0){ comPos = nameServer.length(); }
    nameServer = nameServer.substr(0, comPos);
    RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());

    m_robot = hrp::BodyPtr(new hrp::Body());
    if (!loadBodyFromModelLoader(m_robot, prop["model"].c_str(), CosNaming::NamingContext::_duplicate(naming.getRootContext()) )){
        RTC_WARN_STREAM("failed to load model[" << prop["model"] << "]");
        return RTC::RTC_ERROR;
    }

    setupEEIKConstraintFromConf(ee_ikc_map, m_robot, prop);
    RTC_INFO_STREAM("setupEEIKConstraintFromConf finished");

    const double tmp_init = 0;
    t_ip = new interpolator(1, m_dt, interpolator::LINEAR, 1);
    t_ip->clear();
    t_ip->set(&tmp_init);
    t_ip->get(&output_ratio, false);
    q_ref_ip = new interpolator(1, m_dt, interpolator::LINEAR, 1);
    q_ref_ip->clear();
    q_ref_ip->set(&tmp_init);
    q_ref_ip->get(&q_ref_output_ratio, false);
    default_baselink_h_from_floor = 1.5;
    hcp.baselink_height_from_floor = default_baselink_h_from_floor;
    baselink_h_ip = new interpolator(1, m_dt, interpolator::LINEAR, 1);
    baselink_h_ip->clear();
    baselink_h_ip->set(&default_baselink_h_from_floor);
    baselink_h_ip->get(&baselink_h_from_floor, false);
    RTC_INFO_STREAM("setup interpolator finished");

    dqAct_filter = BiquadIIRFilterVec(m_robot->numJoints());
    dqAct_filter.setParameter(hcp.dqAct_filter_cutoff_hz, 1/m_dt, Q_BUTTERWORTH);
    dqAct_filter.reset(0);
    for ( int i=0; i<ee_names.size(); i++) {
        ee_vel_filter[ee_names[i]] = BiquadIIRFilterVec(6);
        ee_vel_filter[ee_names[i]].setParameter(hcp.ee_vel_filter_cutoff_hz, 1/m_dt, Q_BUTTERWORTH);
        ee_vel_filter[ee_names[i]].reset(0);
        wrench_lpf_for_hpf[ee_names[i]] = BiquadIIRFilterVec(6);
        wrench_lpf_for_hpf[ee_names[i]].setParameter(hcp.wrench_hpf_cutoff_hz, 1/m_dt, Q_BUTTERWORTH);
        wrench_lpf_for_hpf[ee_names[i]].reset(0);
        wrench_lpf[ee_names[i]] = BiquadIIRFilterVec(6);
        wrench_lpf[ee_names[i]].setParameter(hcp.wrench_lpf_cutoff_hz, 1/m_dt, Q_BUTTERWORTH);
        wrench_lpf[ee_names[i]].reset(0);
    }

    for ( int i=0; i<ee_names.size(); i++) {
        std::string n = "slave_"+ee_names[i]+"_wrench";
        m_slaveEEWrenchesIn[ee_names[i]] = ITDS_Ptr(new RTC::InPort<RTC::TimedDoubleSeq>(n.c_str(), m_slaveEEWrenches[ee_names[i]]));
        registerInPort(n.c_str(), *m_slaveEEWrenchesIn[ee_names[i]]);
        m_slaveEEWrenches[ee_names[i]].data = hrp::to_DoubleSeq(hrp::dvector6::Zero()); // avoid non-zero initial input
        RTC_INFO_STREAM(" registerInPort " << n );
    }

    tgt_names = ee_names;
    tgt_names.push_back("com");
    tgt_names.push_back("head");
    for ( int i=0; i<tgt_names.size(); i++) {
        std::string n = "master_"+tgt_names[i]+"_pose";
        m_masterTgtPosesOut[tgt_names[i]] = OTP3_Ptr(new RTC::OutPort<RTC::TimedPose3D>(n.c_str(), m_masterTgtPoses[tgt_names[i]]));
        registerOutPort(n.c_str(), *m_masterTgtPosesOut[tgt_names[i]]);
        RTC_INFO_STREAM(" registerOutPort " << n);
    }

    //debug : This won't be send to slave
    for ( int i=0; i<ee_names.size(); i++) {
        std::string n = "master_"+ee_names[i]+"_wrench_dbg";
        m_masterEEWrenchesOut[ee_names[i]] = OTDS_Ptr(new RTC::OutPort<RTC::TimedDoubleSeq>(n.c_str(), m_masterEEWrenches[ee_names[i]]));
        registerOutPort(n.c_str(), *m_masterEEWrenchesOut[ee_names[i]]);
        m_masterEEWrenches[ee_names[i]].data = hrp::to_DoubleSeq(hrp::dvector6::Zero()); // avoid non-zero initial input
        RTC_INFO_STREAM(" registerOutPort " << n );
    }

    m_tau.data = hrp::to_DoubleSeq(hrp::dvector::Zero(m_robot->numJoints()));

    loop = 0;
    RTC_INFO_STREAM("onInitialize() OK");
    return RTC::RTC_OK;
}

RTC::ReturnCode_t HapticController::setupEEIKConstraintFromConf(std::map<std::string, IKConstraint>& _ee_ikc_map, hrp::BodyPtr _robot, RTC::Properties& _prop){
    std::vector<std::string> ee_conf_all = coil::split(_prop["end_effectors"], ",");
    size_t prop_num = 10; // limbname + linkname + basename + pos(3) + angleaxis(4)
    if (ee_conf_all.size() > 0) {
        size_t ee_num = ee_conf_all.size()/prop_num;
        for (size_t i = 0; i < ee_num; i++) {
            std::string ee_name, target_link_name, base_name; // e.g. rleg, RLEG_JOINT5, WAIST
            ee_name             = ee_conf_all[i*prop_num];
            target_link_name    = ee_conf_all[i*prop_num+1];
            base_name           = ee_conf_all[i*prop_num+2];
            ee_names.push_back(ee_name);
            _ee_ikc_map[ee_name].target_link_name = target_link_name;
            for (size_t j = 0; j < XYZ; j++){ _ee_ikc_map[ee_name].localPos(j) = std::atof( ee_conf_all[i*prop_num+3+j].c_str() ); }
            hrp::Vector4 tmp_aa;
            for (int j = 0; j < 4; j++ ){ tmp_aa(j) = std::atof( ee_conf_all[i*prop_num+6+j].c_str() ); }
            _ee_ikc_map[ee_name].localR = Eigen::AngleAxis<double>(tmp_aa(3), tmp_aa.head(3)).toRotationMatrix(); // rotation in VRML is represented by axis + angle
            jpath_ee[ee_name] = hrp::JointPath(m_robot->rootLink(), m_robot->link(target_link_name));
            if(_robot->link(target_link_name)){
                RTC_INFO_STREAM("End Effector [" << ee_name << "]");
                RTC_INFO_STREAM("   target_link_name = " << _ee_ikc_map[ee_name].target_link_name << ", base = " << base_name);
                RTC_INFO_STREAM("   offset_pos = " << _ee_ikc_map[ee_name].localPos.transpose() << "[m]");
                RTC_INFO_STREAM("   has_toe_joint = " << "fix to false now");
            }else{
                RTC_WARN_STREAM("Target link [" << target_link_name << "] not found !");
                return RTC::RTC_ERROR;
            }
        }
    }
    return RTC::RTC_OK;
}

RTC::ReturnCode_t HapticController::onExecute(RTC::UniqueId ec_id){
    if(DEBUGP_ONCE) RTC_INFO_STREAM("onExecute(" << ec_id << ") dt = " << m_dt);
    if(m_qRefIn.isNew()                     ){ m_qRefIn.read(); }
    if(m_qActIn.isNew()                     ){ m_qActIn.read(); }
    if(m_dqActIn.isNew()                    ){ m_dqActIn.read(); }
    if(m_delayCheckPacketInboundIn.isNew()  ){ m_delayCheckPacketInboundIn.read(); m_delayCheckPacketOutboundOut.write();}
    for (int i=0; i<ee_names.size(); i++) {
        if ( m_slaveEEWrenchesIn[ee_names[i]]->isNew() ) { m_slaveEEWrenchesIn[ee_names[i]]->read(); }
    }
    mode.update();
    processTransition();

    calcCurrentState();
    calcTorque();

    if (mode.isRunning()) {
        if(mode.isInitialize()){
            idsb.setInitState(m_robot, m_dt);//逆動力学初期化
        }

        ///////////////////////////

    }


    m_tau.data = hrp::to_DoubleSeq(hrp::getUAll(m_robot) * output_ratio);

    // write
    m_qOut.write();
    m_tauOut.write();
    loop ++;
    return RTC::RTC_OK;
}

class ContactInfo{
    private:
    public:
    ContactInfo(hrp::Link* l, const hrp::Vector3& lp)
            :target_link_(l),
             is_contact(false),
             local_pos_(lp){}
     ContactInfo(){}
        hrp::Link* target_link_;
        hrp::Vector3 local_pos_;
        hrp::Vector3 world_pos_;
        hrp::Vector3 world_pos_old_;
        hrp::Vector3 world_f_;
        bool is_contact;
        hrp::Vector3 CalcWorldPos(){ world_pos_ = target_link_->p + target_link_->R * local_pos_; return world_pos_;}
        void UpdateState(){world_pos_old_ = world_pos_;}
        friend std::ostream& operator<<(std::ostream& os, const ContactInfo& ci){
            os<< "target_link: "<<ci.target_link_->name<<" local_pos: "<<ci.local_pos_.transpose();
            return os;
        }
};


void HapticController::calcCurrentState(){
    ///// set current joint angles
    for(int i=0;i<m_robot->numJoints();i++)m_robot->joint(i)->q = m_qAct.data[i];
    dqAct_filtered = dqAct_filter.passFilter(hrp::to_dvector(m_dqAct.data));

    ///// calc inverse dynamics (forward kinematics included)
    if(!idsb.is_initialized){ idsb.setInitState(m_robot, m_dt); }
    for(int i=0;i<m_robot->numJoints();i++)idsb.q(i) = m_robot->joint(i)->q;
    idsb.dq.fill(0);
    idsb.ddq.fill(0);
    const hrp::Vector3 g(0, 0, 9.80665* hcp.gravity_compensation_ratio);
    idsb.base_p = m_robot->rootLink()->p;
    idsb.base_v.fill(0);
    idsb.base_dv = g;
    idsb.base_R = hrp::Matrix33::Identity();//m_robot->rootLink()->R;
    idsb.base_dR.fill(0);
    idsb.base_w_hat.fill(0);
    idsb.base_w.fill(0);
    idsb.base_dw.fill(0);
    hrp::Vector3 f_base_wld, t_base_wld;
    calcRootLinkWrenchFromInverseDynamics(m_robot, idsb, f_base_wld, t_base_wld); // torque (m_robot->joint(i)->u) set
    updateInvDynStateBuffer(idsb);

    ///// calc end effector position and velocity
    for(int i=0; i<ee_names.size();i++){
        ee_pose[ee_names[i]] = ee_ikc_map[ee_names[i]].getCurrentTargetPose(m_robot);
    }
    if(ee_pose_old.size()==0){ ee_pose_old = ee_pose; } //init
    for(int i=0; i<ee_names.size();i++){
        ee_vel[ee_names[i]].head(3) = (ee_pose[ee_names[i]].p - ee_pose_old[ee_names[i]].p) / m_dt;// rawは廃止したほうがいい？
        ee_vel[ee_names[i]].tail(3) = ee_pose_old[ee_names[i]].R * hrp::omegaFromRot(ee_pose_old[ee_names[i]].R.transpose() * ee_pose[ee_names[i]].R) / m_dt; // world angular velocity
        ee_vel_filtered[ee_names[i]] = ee_vel_filter[ee_names[i]].passFilter(ee_vel[ee_names[i]]);
    }
    ee_pose_old = ee_pose;

    ///// calc base tp end effector Jacobian
    for(int i=0; i<ee_names.size();i++){
        hrp::JointPath jp(m_robot->rootLink(), m_robot->link(ee_ikc_map[ee_names[i]].target_link_name));
        jp.calcJacobian(J_ee[ee_names[i]], ee_ikc_map[ee_names[i]].localPos);
    }
}


void HapticController::calcTorque(){

//    static std::vector<ContactInfo> civ;
//    if(civ.size() == 0){
//        const double a = 4;
//        civ.push_back(ContactInfo(m_robot->link("RLEG_JOINT5"), stikp[0].localp + hrp::Vector3( 0.1,  0.05, 0)*a ));
//        civ.push_back(ContactInfo(m_robot->link("RLEG_JOINT5"), stikp[0].localp + hrp::Vector3( 0.1, -0.05, 0)*a ));
//        civ.push_back(ContactInfo(m_robot->link("RLEG_JOINT5"), stikp[0].localp + hrp::Vector3(-0.1,  0.05, 0)*a ));
//        civ.push_back(ContactInfo(m_robot->link("RLEG_JOINT5"), stikp[0].localp + hrp::Vector3(-0.1, -0.05, 0)*a ));
//        civ.push_back(ContactInfo(m_robot->link("LLEG_JOINT5"), stikp[1].localp + hrp::Vector3( 0.1,  0.05, 0)*a ));
//        civ.push_back(ContactInfo(m_robot->link("LLEG_JOINT5"), stikp[1].localp + hrp::Vector3( 0.1, -0.05, 0)*a ));
//        civ.push_back(ContactInfo(m_robot->link("LLEG_JOINT5"), stikp[1].localp + hrp::Vector3(-0.1,  0.05, 0)*a ));
//        civ.push_back(ContactInfo(m_robot->link("LLEG_JOINT5"), stikp[1].localp + hrp::Vector3(-0.1, -0.05, 0)*a ));
//
////        civ.push_back(ContactInfo(m_robot->link("RARM_JOINT6"), stikp[2].localp + hrp::Vector3( 0.1,  0.05, 0)*a ));
////        civ.push_back(ContactInfo(m_robot->link("RARM_JOINT6"), stikp[2].localp + hrp::Vector3( 0.1, -0.05, 0)*a ));
////        civ.push_back(ContactInfo(m_robot->link("RARM_JOINT6"), stikp[2].localp + hrp::Vector3(-0.1,  0.05, 0)*a ));
////        civ.push_back(ContactInfo(m_robot->link("RARM_JOINT6"), stikp[2].localp + hrp::Vector3(-0.1, -0.05, 0)*a ));
////        civ.push_back(ContactInfo(m_robot->link("LARM_JOINT6"), stikp[3].localp + hrp::Vector3( 0.1,  0.05, 0)*a ));
////        civ.push_back(ContactInfo(m_robot->link("LARM_JOINT6"), stikp[3].localp + hrp::Vector3( 0.1, -0.05, 0)*a ));
////        civ.push_back(ContactInfo(m_robot->link("LARM_JOINT6"), stikp[3].localp + hrp::Vector3(-0.1,  0.05, 0)*a ));
////        civ.push_back(ContactInfo(m_robot->link("LARM_JOINT6"), stikp[3].localp + hrp::Vector3(-0.1, -0.05, 0)*a ));
//    }
       // calc virtual floor reaction force
//    {
//        for (int i=0; i<civ.size(); i++){
//            civ[i].CalcWorldPos();
//            double vfloor_h;
//            if(civ[i].target_link_->name.find("LEG") != std::string::npos){
//                vfloor_h = m_robot->rootLink()->p(Z) - 1.2;
//            }else{
//                vfloor_h = m_robot->rootLink()->p(Z) - 0.1;
//            }
//            if(civ[i].world_pos_(2) < vfloor_h){
//                civ[i].is_contact = true;
//                const double pos_err = vfloor_h - civ[i].world_pos_(2);
//                const double vel_err = 0 - (civ[i].world_pos_(2) - civ[i].world_pos_old_(2)) / dt;
//                civ[i].world_f_ << 0, 0, 5000 * pos_err + 50 * vel_err;// P,D = ?,10 OUT
//                if(civ[i].world_f_.norm() > 500){
//                    civ[i].world_f_ = civ[i].world_f_.normalized() * 500;
//                }
//                const hrp::Vector3 world_f_rel_base = m_robot->rootLink()->R.transpose() * civ[i].world_f_;
//                hrp::JointPath jp(m_robot->rootLink(), civ[i].target_link_);
//                hrp::dmatrix J_contact;
//                jp.calcJacobian(J_contact, civ[i].local_pos_);
//                hrp::dvector tq_for_virtual_reaction_force = (J_contact.topRows(3)).transpose() * world_f_rel_base;
//                for (int j = 0; j < jp.numJoints(); j++) jp.joint(j)->u += tq_for_virtual_reaction_force(j);
//            }else{
//                civ[i].is_contact = false;
//            }
//            civ[i].UpdateState();
//            }
//        if(loop%500==0)dbgv(hrp::getUAll(m_robot));
//    }

    std::map<std::string, hrp::dvector6> masterEEWrenches;
    for (int i=0; i<ee_names.size(); i++){ masterEEWrenches[ee_names[i]].fill(0); }

    std::map<std::string, bool> is_contact_to_floor;
    {
        std::vector<std::string> tgt;
        tgt.push_back("lleg");
        tgt.push_back("rleg");

        ///// fix ee rot horizontal
        for (int i=0; i<tgt.size();i++){
            hrp::Vector3 diff_rot;
            rats::difference_rotation(diff_rot, ee_pose[tgt[i]].R, hrp::Matrix33::Identity());
            const hrp::Vector3 diff_rot_vel = hrp::Vector3::Zero() - ee_vel_filtered[tgt[i]].tail(3);
            hrp::dvector6 wrench = (hrp::dvector6()<< 0,0,0, diff_rot * hcp.foot_horizontal_pd_gain(0) + diff_rot_vel * hcp.foot_horizontal_pd_gain(1)).finished();
            LIMIT_NORM_V(wrench.tail(3), 50);
            hrp::dvector tq_tmp = J_ee[ee_names[i]].transpose() * wrench;
            for(int j=0; j<jpath_ee[tgt[i]].numJoints(); j++){ jpath_ee[tgt[i]].joint(j)->u += tq_tmp(j); }
            masterEEWrenches[tgt[i]] += wrench;
        }

        ///// virtual floor
        for (int i=0; i<tgt.size();i++){
            const double foot_h_from_floor = ee_pose[tgt[i]].p(Z);
            if(loop%1000==0){
                dbg(foot_h_from_floor);
            }
            if(foot_h_from_floor < 0){
                hrp::dvector6 wrench = hrp::dvector6::Unit(fz) * (-foot_h_from_floor*hcp.floor_pd_gain(0) + (0-ee_vel_filtered[tgt[i]](fz))*hcp.floor_pd_gain(1));
                LIMIT_NORM_V(wrench, 1000);
                const hrp::dvector tq_tmp = J_ee[tgt[i]].transpose() * wrench;
                for(int j=0; j<jpath_ee[tgt[i]].numJoints(); j++){ jpath_ee[tgt[i]].joint(j)->u += tq_tmp(j); }
                masterEEWrenches[tgt[i]] += wrench;
            }
            is_contact_to_floor[tgt[i]] = (foot_h_from_floor < 0 + 0.03);
        }

        ///// virtual back wall
        for (int i=0; i<tgt.size();i++){
            const double wall_x_rel_base = -0.1;
            const double current_x = ee_pose[tgt[i]].p(X) - m_robot->rootLink()->p(X);
            if(current_x < wall_x_rel_base){
                hrp::dvector6 wrench = (hrp::dvector6()<< (wall_x_rel_base - current_x) * 1000,0,0, 0,0,0).finished();
                LIMIT_NORM_V(wrench, 100);
                hrp::dvector tq_tmp = J_ee[tgt[i]].transpose() * wrench;
                for(int j=0; j<jpath_ee[tgt[i]].numJoints(); j++){ jpath_ee[tgt[i]].joint(j)->u += tq_tmp(j); }
                masterEEWrenches[tgt[i]] += wrench;
            }
        }

        ///// keep apart each other
        const double current_dist = ee_pose["lleg"].p(Y) - ee_pose["rleg"].p(Y);
        if(current_dist < hcp.foot_min_distance){
            for (int i=0; i<tgt.size();i++){
                hrp::dvector6 wrench = (hrp::dvector6()<< 0, (tgt[i]=="lleg" ? 1:-1) * (hcp.foot_min_distance - current_dist) * 1000, 0,0,0,0).finished();
                LIMIT_NORM_V(wrench, 50);
                hrp::dvector tq_tmp = J_ee[tgt[i]].transpose() * wrench;
                for (int j=0; j<jpath_ee[tgt[i]].numJoints(); j++){ jpath_ee[tgt[i]].joint(j)->u += tq_tmp(j); }
                masterEEWrenches[tgt[i]] += wrench;
            }
        }

        ///// virtual floor, leg constraint
        static hrp::Vector3 locked_rel_pos = ee_pose["rleg"].p - ee_pose["lleg"].p;
        if(is_contact_to_floor["lleg"] && is_contact_to_floor["rleg"]){
            for (int i=0; i<tgt.size();i++){
                hrp::Vector3 cur_rel_pos = ee_pose["rleg"].p - ee_pose["lleg"].p;
                hrp::Vector2 rleg_wrench = (locked_rel_pos - cur_rel_pos).head(XY) * 1000;
                hrp::dvector6 wrench = (hrp::dvector6()<< (tgt[i]=="rleg" ? 1:-1) * rleg_wrench, 0,0,0,0).finished();
                LIMIT_NORM_V(wrench, 1000);
                hrp::dvector tq_tmp = J_ee[tgt[i]].transpose() * wrench;
                for (int j=0; j<jpath_ee[tgt[i]].numJoints(); j++){ jpath_ee[tgt[i]].joint(j)->u += tq_tmp(j); }
                masterEEWrenches[tgt[i]] += wrench;
            }
        }else{
            locked_rel_pos = ee_pose["rleg"].p - ee_pose["lleg"].p;
        }


        static std::map<std::string, hrp::Vector2> leg_pos_from_floor_origin;
        hrp::Vector2 baselink_pos_from_floor_origin;
        if(loop==0){
            leg_pos_from_floor_origin["lleg"] << 0, 0.15;
            leg_pos_from_floor_origin["rleg"] << 0,- 0.15;
            baselink_pos_from_floor_origin << 0,0;
        }
        if(is_contact_to_floor["lleg"] && !is_contact_to_floor["rleg"] ){ // when lleg has contact to floor, each value can be calculated from lleg
            leg_pos_from_floor_origin["rleg"]   = (ee_pose["rleg"].p        - ee_pose["lleg"].p).head(XY) + leg_pos_from_floor_origin["lleg"];
            baselink_pos_from_floor_origin      = (m_robot->rootLink()->p   - ee_pose["lleg"].p).head(XY) + leg_pos_from_floor_origin["lleg"];
        }
        else if(!is_contact_to_floor["lleg"] && is_contact_to_floor["rleg"] ){ // when rleg has contact to floor, each value can be calculated from rleg
            leg_pos_from_floor_origin["lleg"]   = (ee_pose["lleg"].p        - ee_pose["rleg"].p).head(XY) + leg_pos_from_floor_origin["rleg"];
            baselink_pos_from_floor_origin      = (m_robot->rootLink()->p   - ee_pose["rleg"].p).head(XY) + leg_pos_from_floor_origin["rleg"];
        }
        else if(is_contact_to_floor["lleg"] && is_contact_to_floor["rleg"] ){ // when both has contact to floor, baselink may be calculated from rleg or lleg
            baselink_pos_from_floor_origin      = (m_robot->rootLink()->p   - ee_pose["rleg"].p).head(XY) + leg_pos_from_floor_origin["rleg"];// for example
        }
        else{ // when both in the air, both feet pos won't be update, only baselink will be update
            baselink_pos_from_floor_origin      = (m_robot->rootLink()->p - ee_pose["rleg"].p).head(XY) + leg_pos_from_floor_origin["rleg"];// for example
        }

        //update base link pos from world
        m_robot->rootLink()->p << baselink_pos_from_floor_origin, baselink_h_from_floor;

        m_teleopOdom.data = (RTC::Pose3D){m_robot->rootLink()->p(X),m_robot->rootLink()->p(Y),m_robot->rootLink()->p(Z), 0,0,0};
        m_teleopOdom.tm = m_qRef.tm;
        m_teleopOdomOut.write();


    }



    // calc real robot feedback force
    for (int i=0; i<ee_names.size(); i++){
        if(ee_names[i] == "lleg" || ee_names[i] == "rleg" ) continue; // skip leg force feedback
        const hrp::dvector6 wrench_raw = hrp::to_dvector(m_slaveEEWrenches[ee_names[i]].data);
        const hrp::dvector6 w_hpf = wrench_raw - wrench_lpf_for_hpf[ee_names[i]].passFilter( wrench_raw );
        const hrp::dvector6 w_lpf = wrench_lpf[ee_names[i]].passFilter( wrench_raw );
        wrench_shaped[ee_names[i]] = w_hpf * hcp.wrench_hpf_gain + w_lpf * hcp.wrench_lpf_gain;

        wrench_used[ee_names[i]] = wrench_shaped[ee_names[i]];
        LIMIT_NORM_V(wrench_used[ee_names[i]].head(3),50);
        LIMIT_NORM_V(wrench_used[ee_names[i]].tail(3),5);

        hrp::Vector3 friction_f = - hcp.ee_pos_rot_friction_coeff(0) * ee_vel_filtered[ee_names[i]].head(3);
        hrp::Vector3 friction_t = - hcp.ee_pos_rot_friction_coeff(1) * ee_vel_filtered[ee_names[i]].tail(3);

        if(ee_names[i] == "rarm"){
            m_debugData.data = hrp::to_DoubleSeq((hrp::dvector(6+6+6+6)<<wrench_raw,wrench_shaped[ee_names[i]],w_hpf,w_lpf).finished());
            m_debugData.tm = m_qRef.tm;
            m_debugDataOut.write();
            if(loop%1000==0){
                dbgv(ee_vel_filtered["rarm"]);
                dbgv(hcp.ee_pos_rot_friction_coeff);
                dbg(hcp.ee_vel_filter_cutoff_hz);
            }
        }

        hrp::JointPath jp(m_robot->rootLink(), m_robot->link(ee_ikc_map[ee_names[i]].target_link_name));
        hrp::dmatrix J_base_to_ee;
        jp.calcJacobian(J_base_to_ee, ee_ikc_map[ee_names[i]].localPos);
        hrp::dvector tq_from_feedbackWrench = J_base_to_ee.transpose() * wrench_used[ee_names[i]];
        for (int j = 0; j < jp.numJoints(); j++) jp.joint(j)->u += tq_from_feedbackWrench(j);
        if(loop%1000==0)dbgv(wrench_used[ee_names[i]]);
        masterEEWrenches[ee_names[i]] += wrench_used[ee_names[i]];
    }


    {
        // soft joint limit
        m_robot->link("RARM_JOINT1")->ulimit = deg2rad(-10);
        m_robot->link("LARM_JOINT1")->llimit = deg2rad(10);
        for(int i=0;i<m_robot->numJoints();i++){
            const double soft_ulimit = m_robot->joint(i)->ulimit - deg2rad(15);
            const double soft_llimit = m_robot->joint(i)->llimit + deg2rad(15);
            double soft_jlimit_tq = 0;
            if(m_qAct.data[i] > soft_ulimit){
                soft_jlimit_tq = 100 * (soft_ulimit - m_qAct.data[i]) + 0 * (0 - dqAct_filtered(i));
            }
            if(m_qAct.data[i] < soft_llimit){
                soft_jlimit_tq = 100 * (soft_llimit - m_qAct.data[i]) + 0 * (0 - dqAct_filtered(i));
            }
            m_robot->joint(i)->u += soft_jlimit_tq;
        }
    }

    { // real robot joint friction damper
        hrp::dvector friction_tq = - hcp.q_friction_coeff * dqAct_filtered;
        for (int i=0; i<m_robot->numJoints(); i++){
         LIMIT_MINMAX(friction_tq(i), -10, 10);
         m_robot->joint(i)->u += friction_tq(i);
        }
        if(loop%1000==0)dbgv(friction_tq);
    }


    const hrp::dvector max_torque = (hrp::dvector(m_robot->numJoints()) << 20,40,60,60,20,20, 20,40,60,60,20,20, 40, 15,15,10,10,6,4,4, 15,15,10,10,6,4,4).finished();
    hrp::dvector j_power_coeff = max_torque/max_torque.maxCoeff(); // reduce gain of light inertia joint

    {// calc qref pd control torque
        hrp::dvector q_diff = hrp::to_dvector(m_qRef.data) - hrp::to_dvector(m_qAct.data);
        hrp::dvector dq_diff = - dqAct_filtered;
        hrp::dvector torque = (q_diff * hcp.q_ref_pd_gain(0) + dq_diff * hcp.q_ref_pd_gain(1)) * j_power_coeff;
        for (int i=0; i<m_robot->numJoints(); i++){
            LIMIT_NORM(torque(i), max_torque(i)/5);// pd torque will be limited to 1/5 max torque
            m_robot->joint(i)->u += torque(i);
        }
    }






    for(int i=0;i<m_robot->numJoints();i++){
        LIMIT_NORM(m_robot->joint(i)->u, max_torque(i)*2);// up to x2 max torque
//        double tlimit = m_robot->joint(i)->climit * m_robot->joint(i)->gearRatio * m_robot->joint(i)->torqueConst;////?????
    }

    if(loop%1000==0)dbgv(hrp::getUAll(m_robot));


    // for debug plot
    for (int i=0; i<ee_names.size(); i++){
        m_masterEEWrenches[ee_names[i]].tm = m_qRef.tm;
        m_masterEEWrenches[ee_names[i]].data = hrp::to_DoubleSeq(masterEEWrenches[ee_names[i]]);
        m_masterEEWrenchesOut[ee_names[i]]->write();
    }



    for (int i=0; i<tgt_names.size(); i++){
        if(tgt_names[i] == "com"){
            m_masterTgtPoses[tgt_names[i]].data = hrp::to_Pose3D(hrp::Pose3(m_robot->calcCM(), hrp::Matrix33::Identity()));
        }else if(tgt_names[i] == "head"){
            // dummy
        }else{
            m_masterTgtPoses[tgt_names[i]].data = hrp::to_Pose3D(ee_ikc_map[ee_names[i]].getCurrentTargetPose(m_robot));//四肢揃ってないと危険
        }
        m_masterTgtPoses[tgt_names[i]].tm = m_qRef.tm;
//        m_masterTgtPoses[tgt_names[i]].tm = m_qAct.tm;
        m_masterTgtPosesOut[tgt_names[i]]->write();
    }


}

void HapticController::processTransition(){
    if(!q_ref_ip        ->isEmpty()){q_ref_ip       ->get( &q_ref_output_ratio,     true); }
    if(!baselink_h_ip   ->isEmpty()){baselink_h_ip  ->get( &baselink_h_from_floor,  true); }
    if(!t_ip            ->isEmpty()){t_ip           ->get( &output_ratio,           true); }
    if(mode.now() == MODE_SYNC_TO_HC    && output_ratio == 1.0){ mode.setNextMode(MODE_HC); }
    if(mode.now() == MODE_SYNC_TO_IDLE  && output_ratio == 0.0){ mode.setNextMode(MODE_IDLE); }
}


bool HapticController::startHapticController(){
    if(mode.now() == MODE_IDLE){
        RTC_INFO_STREAM("startHapticController");
        mode.setNextMode(MODE_SYNC_TO_HC);
        const double next_goal = 1.0; t_ip->setGoal(&next_goal, 10.0, true);
        return true;
    }else{
        RTC_WARN_STREAM("Invalid context to startHapticController");
        return false;
    }
}


bool HapticController::pauseHapticController(){
    if(mode.now() == MODE_HC){
        RTC_INFO_STREAM("pauseHapticController");
        mode.setNextMode(MODE_PAUSE);
        return true;
    }else{
        RTC_WARN_STREAM("Invalid context to pauseHapticController");
        return false;
    }
}


bool HapticController::resumeHapticController(){
    if(mode.now() == MODE_PAUSE){
        RTC_INFO_STREAM("resumeHapticController");
        mode.setNextMode(MODE_HC);
        return true;
    }else{
        RTC_WARN_STREAM("Invalid context to resumeHapticController");
        return false;
    }
}


bool HapticController::stopHapticController(){
    if(mode.now() == MODE_HC || mode.now() == MODE_PAUSE ){
        RTC_INFO_STREAM("stopHapticController");
        mode.setNextMode(MODE_SYNC_TO_IDLE);
        const double next_goal = 0.0; t_ip->setGoal(&next_goal, 5.0, true);
        baselink_h_ip->setGoal(&default_baselink_h_from_floor, 5.0, true);
        return true;
    }else{
        RTC_WARN_STREAM("Invalid context to stopHapticController");
        return false;
    }
}

namespace hrp{
    hrp::Vector2 to_Vector2(const OpenHRP::HapticControllerService::DblSequence2& in){ return (hrp::Vector2()<< in[0],in[1]).finished(); }
    OpenHRP::HapticControllerService::DblSequence2 to_DblSequence2(const hrp::Vector2& in){
        OpenHRP::HapticControllerService::DblSequence2 ret; ret.length(2); ret[0] = in(0); ret[1] = in(1); return ret; }
}

bool HapticController::setParams(const OpenHRP::HapticControllerService::HapticControllerParam& i_param){
    RTC_INFO_STREAM("setHapticControllerParam");
    hcp.baselink_height_from_floor  = i_param.baselink_height_from_floor;
    hcp.dqAct_filter_cutoff_hz      = i_param.dqAct_filter_cutoff_hz;
    hcp.ee_vel_filter_cutoff_hz     = i_param.ee_vel_filter_cutoff_hz;
    hcp.foot_min_distance           = i_param.foot_min_distance;
    hcp.force_feedback_ratio        = i_param.force_feedback_ratio;
    hcp.gravity_compensation_ratio  = i_param.gravity_compensation_ratio;
    hcp.q_friction_coeff            = i_param.q_friction_coeff;
    hcp.q_ref_output_ratio_goal     = i_param.q_ref_output_ratio_goal;
    hcp.wrench_hpf_cutoff_hz        = i_param.wrench_hpf_cutoff_hz;
    hcp.wrench_lpf_cutoff_hz        = i_param.wrench_lpf_cutoff_hz;
    hcp.wrench_hpf_gain             = i_param.wrench_hpf_gain;
    hcp.wrench_lpf_gain             = i_param.wrench_lpf_gain;
    hcp.ee_pos_rot_friction_coeff   = hrp::to_Vector2(i_param.ee_pos_rot_friction_coeff);
    hcp.floor_pd_gain               = hrp::to_Vector2(i_param.floor_pd_gain);
    hcp.foot_horizontal_pd_gain     = hrp::to_Vector2(i_param.foot_horizontal_pd_gain);
    hcp.q_ref_pd_gain               = hrp::to_Vector2(i_param.q_ref_pd_gain);
    ///// update process if required
    q_ref_ip->setGoal(&hcp.q_ref_output_ratio_goal, 5.0, true);
    baselink_h_ip->setGoal(&hcp.baselink_height_from_floor, 5.0, true);
    dqAct_filter.setParameter(hcp.dqAct_filter_cutoff_hz, 1/m_dt, Q_BUTTERWORTH);
    dqAct_filter.reset(0);
    for(int i=0;i<ee_names.size();i++){
        ee_vel_filter[ee_names[i]].setParameter(hcp.ee_vel_filter_cutoff_hz, 1/m_dt, Q_BUTTERWORTH);
        ee_vel_filter[ee_names[i]].reset(0);
        wrench_lpf_for_hpf[ee_names[i]].setParameter(hcp.wrench_hpf_cutoff_hz, 1/m_dt, Q_BUTTERWORTH);
        wrench_lpf_for_hpf[ee_names[i]].reset(0);
        wrench_lpf[ee_names[i]].setParameter(hcp.wrench_lpf_cutoff_hz, 1/m_dt, Q_BUTTERWORTH);
        wrench_lpf[ee_names[i]].reset(0);
    }
    return true;
}


bool HapticController::getParams(OpenHRP::HapticControllerService::HapticControllerParam& i_param){
    RTC_INFO_STREAM("getHapticControllerParam");
    i_param.baselink_height_from_floor  = hcp.baselink_height_from_floor;
    i_param.dqAct_filter_cutoff_hz      = hcp.dqAct_filter_cutoff_hz;
    i_param.ee_vel_filter_cutoff_hz     = hcp.ee_vel_filter_cutoff_hz;
    i_param.foot_min_distance           = hcp.foot_min_distance;
    i_param.force_feedback_ratio        = hcp.force_feedback_ratio;
    i_param.gravity_compensation_ratio  = hcp.gravity_compensation_ratio;
    i_param.q_friction_coeff            = hcp.q_friction_coeff;
    i_param.q_ref_output_ratio_goal     = hcp.q_ref_output_ratio_goal;
    i_param.wrench_hpf_cutoff_hz        = hcp.wrench_hpf_cutoff_hz;
    i_param.wrench_lpf_cutoff_hz        = hcp.wrench_lpf_cutoff_hz;
    i_param.wrench_hpf_gain             = hcp.wrench_hpf_gain;
    i_param.wrench_lpf_gain             = hcp.wrench_lpf_gain;
    i_param.ee_pos_rot_friction_coeff   = hrp::to_DblSequence2(hcp.ee_pos_rot_friction_coeff);
    i_param.floor_pd_gain               = hrp::to_DblSequence2(hcp.floor_pd_gain);
    i_param.foot_horizontal_pd_gain     = hrp::to_DblSequence2(hcp.foot_horizontal_pd_gain);
    i_param.q_ref_pd_gain               = hrp::to_DblSequence2(hcp.q_ref_pd_gain);
    return true;
}


RTC::ReturnCode_t HapticController::onActivated(RTC::UniqueId ec_id){ RTC_INFO_STREAM("onActivated(" << ec_id << ")"); return RTC::RTC_OK; }
RTC::ReturnCode_t HapticController::onDeactivated(RTC::UniqueId ec_id){ RTC_INFO_STREAM("onDeactivated(" << ec_id << ")"); return RTC::RTC_OK; }
RTC::ReturnCode_t HapticController::onFinalize(){ return RTC::RTC_OK; }

extern "C"{
    void HapticControllerInit(RTC::Manager* manager) {
        RTC::Properties profile(HapticController_spec);
        manager->registerFactory(profile, RTC::Create<HapticController>, RTC::Delete<HapticController>);
    }
};
