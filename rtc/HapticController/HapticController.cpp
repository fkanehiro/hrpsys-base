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
    m_HapticControllerServicePort("HapticControllerService"),
    m_debugLevel(0)
{
    m_service0.hapticcontroller(this);
}

HapticController::~HapticController(){}

RTC::ReturnCode_t HapticController::onInitialize(){
    RTCOUT << "onInitialize()" << std::endl;
    bindParameter("debugLevel", m_debugLevel, "0");
    addInPort("qRef", m_qRefIn);
    addInPort("qAct", m_qActIn);
    addInPort("dqAct", m_dqActIn);
    addOutPort("q", m_qOut);
    addOutPort("tau", m_tauOut);
    addOutPort("teleopOdom", m_teleopOdomOut);
    addOutPort("debugData", m_debugDataOut);
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
        RTCOUT << "failed to load model[" << prop["model"] << "]" << std::endl;
        return RTC::RTC_ERROR;
    }

    setupEEIKConstraintFromConf(ee_ikc_map, m_robot, prop);
    RTCOUT << "setupEEIKConstraintFromConf finished" << std::endl;

    output_ratio = 0.0;
    t_ip = new interpolator(1, m_dt, interpolator::LINEAR, 1);
    t_ip->clear();
    t_ip->set(&output_ratio);
    RTCOUT << "setup interpolator finished" << std::endl;

    dqAct_filter = BiquadIIRFilterVec2(m_robot->numJoints());
    dqAct_filter.setParameter(hcp.dqAct_filter_cutoff_hz, 1/m_dt, Q_BUTTERWORTH);
    dqAct_filter.reset(0);
    for ( int i=0; i<ee_names.size(); i++) {
        ee_vel_filter[ee_names[i]] = BiquadIIRFilterVec2(6);
        ee_vel_filter[ee_names[i]].setParameter(hcp.ee_vel_filter_cutoff_hz, 1/m_dt, Q_BUTTERWORTH);
        ee_vel_filter[ee_names[i]].reset(0);
        wrench_filter[ee_names[i]] = BiquadIIRFilterVec2(6);
        wrench_filter[ee_names[i]].setParameter(hcp.wrench_filter_cutoff_hz, 1/m_dt, Q_BUTTERWORTH);
        wrench_filter[ee_names[i]].reset(0);
        wrench_lpf_for_hpf[ee_names[i]] = BiquadIIRFilterVec2(6);
        wrench_lpf_for_hpf[ee_names[i]].setParameter(hcp.wrench_hpf_cutoff_hz, 1/m_dt, Q_BUTTERWORTH);
        wrench_lpf_for_hpf[ee_names[i]].reset(0);
        wrench_lpf[ee_names[i]] = BiquadIIRFilterVec2(6);
        wrench_lpf[ee_names[i]].setParameter(hcp.wrench_lpf_cutoff_hz, 1/m_dt, Q_BUTTERWORTH);
        wrench_lpf[ee_names[i]].reset(0);
    }
//    std::vector<std::string> force_sensor_names;
//    for (int i=0; i<m_robot->numSensors(hrp::Sensor::FORCE); i++) {
//        force_sensor_names.push_back(m_robot->sensor(hrp::Sensor::FORCE, i)->name);
//    }

    for ( int i=0; i<ee_names.size(); i++) {
        std::string n = "slave_"+ee_names[i]+"_wrench_in";
        m_feedbackWrenchesIn[ee_names[i]] = ITDS_Ptr(new RTC::InPort<RTC::TimedDoubleSeq>(n.c_str(), m_feedbackWrenches[ee_names[i]]));
        registerInPort(n.c_str(), *m_feedbackWrenchesIn[ee_names[i]]);
        m_feedbackWrenches[ee_names[i]].data = hrp::to_DoubleSeq(hrp::dvector6::Zero()); // avoid non-zero initial input
        RTCOUT << " registerInPort " << n << std::endl;
    }

    tgt_names = ee_names;
    tgt_names.push_back("com");
    tgt_names.push_back("head");
    for ( int i=0; i<tgt_names.size(); i++) {
        std::string n = "master_"+tgt_names[i]+"_pose_out";
        m_eePosesOut[tgt_names[i]] = OTP3_Ptr(new RTC::OutPort<RTC::TimedPose3D>(n.c_str(), m_masterTgtPoses[tgt_names[i]]));
        registerOutPort(n.c_str(), *m_eePosesOut[tgt_names[i]]);
        RTCOUT << " registerOutPort " << n << std::endl;
    }


    m_tau.data = hrp::to_DoubleSeq(hrp::dvector::Zero(m_robot->numJoints()));

    loop = 0;
    RTCOUT << "onInitialize() OK" << std::endl;
    return RTC::RTC_OK;
}

RTC::ReturnCode_t HapticController::setupEEIKConstraintFromConf(std::map<std::string, IKConstraint>& _ee_ikc_map, hrp::BodyPtr _robot, RTC::Properties& _prop){
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
              RTCOUT << "End Effector [" << ee_name << "]" << std::endl;
              RTCOUT << "   target_link_name = " << _ee_ikc_map[ee_name].target_link_name << ", base = " << base_name << std::endl;
              RTCOUT << "   offset_pos = " << _ee_ikc_map[ee_name].localPos.transpose() << "[m]" << std::endl;
              RTCOUT << "   has_toe_joint = " << "fix to false now" << std::endl;
            }else{
              RTCOUT << "Target link [" << target_link_name << "] not found !" << std::endl;
              return RTC::RTC_ERROR;
            }
        }
    }
    return RTC::RTC_OK;
}

RTC::ReturnCode_t HapticController::onExecute(RTC::UniqueId ec_id){
    if(DEBUGP_ONCE) RTCOUT << "onExecute(" << ec_id << ") dt = " << m_dt << std::endl;
    if (m_qRefIn.isNew())   { m_qRefIn.read(); }
    if (m_qActIn.isNew())   { m_qActIn.read(); }
    if (m_dqActIn.isNew())  { m_dqActIn.read(); }
    for (int i=0; i<ee_names.size(); i++) {
        if ( m_feedbackWrenchesIn[ee_names[i]]->isNew() ) { m_feedbackWrenchesIn[ee_names[i]]->read(); }
    }

    processTransition();
    mode.update();

    calcTorque();

    if (mode.isRunning()) {
        if(mode.isInitialize()){
//            preProcessForHapticController();
            idsb.setInitState(m_robot, m_dt);//逆動力学初期化
        }

        ///////////////////////////

    }


    m_tau.data = hrp::to_DoubleSeq(hrp::getUAll(m_robot) * output_ratio);

    // write
    m_qOut.write();
    m_tauOut.write();
    m_teleopOdomOut.write();

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

void HapticController::calcTorque (){

    dqAct_filtered = dqAct_filter.passFilter(hrp::to_dvector(m_dqAct.data));

//    static hrp::dvector q_old = hrp::to_dvector(m_qAct.data);
//    hrp::dvector dq_tmp = (hrp::to_dvector(m_qAct.data) - q_old) / m_dt;
//    dqAct_filtered = dqAct_filter.passFilter(dq_tmp);




    for(int i=0;i<m_robot->numJoints();i++)m_robot->joint(i)->q = m_qAct.data[i];

    if(!idsb.is_initialized){
      idsb.setInitState(m_robot, m_dt);
    }
//    calcAccelerationsForInverseDynamics(m_robot, idsb);
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
    calcRootLinkWrenchFromInverseDynamics(m_robot, idsb, f_base_wld, t_base_wld);

//    if(loop%500==0)std::cerr<<"ref_torque is"<<std::endl;
//    if(loop%500==0)dbgv(hrp::getUAll(m_robot));

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


    {
        //fix ee rot horizontal
        std::vector<std::string> targets;
        targets.push_back("LLEG_JOINT5");
        targets.push_back("RLEG_JOINT5");
        static std::vector<hrp::Vector3> diff_rots_old(targets.size());
        for (int i=0; i<targets.size();i++){
            hrp::Vector3 diff_rot;
            rats::difference_rotation(diff_rot, m_robot->link(targets[i])->R, hrp::Matrix33::Identity());
            hrp::Vector3 diff_rot_d = (diff_rot - diff_rots_old[i])/m_dt;
            hrp::dvector6 wrench;
            wrench << 0,0,0,diff_rot * 1000 + diff_rot_d * 10;
            hrp::JointPath jp(m_robot->rootLink(), m_robot->link(targets[i]));
            hrp::dmatrix J_ee;
            jp.calcJacobian(J_ee);
            hrp::dvector tq_for_fix = J_ee.transpose() * wrench;
            for (int j = 0; j < jp.numJoints(); j++) jp.joint(j)->u += tq_for_fix(j);
            diff_rots_old[i] = diff_rot;
        }
    }

    {
        //keep ee apart each other
        std::vector<std::string> targets;
        targets.push_back("LLEG_JOINT5");
        targets.push_back("RLEG_JOINT5");
        double dist = m_robot->link("LLEG_JOINT5")->p(Y) - m_robot->link("RLEG_JOINT5")->p(Y);
        if(dist < 0.3){
            double err = dist - 0.3;
            for (int i=0; i<targets.size();i++){
                hrp::dvector6 wrench;
                wrench << 0, (i==0 ? -1 : 1) * err*1000,0,0,0,0;
                hrp::JointPath jp(m_robot->rootLink(), m_robot->link(targets[i]));
                hrp::dmatrix J_ee;
                jp.calcJacobian(J_ee);
                hrp::dvector tq_for_apart = J_ee.transpose() * wrench;
                for (int j = 0; j < jp.numJoints(); j++) jp.joint(j)->u += tq_for_apart(j);
            }
        }
    }

    {
        // floor
        std::vector<std::string> targets;
        targets.push_back("LLEG_JOINT5");
        targets.push_back("RLEG_JOINT5");
        for (int i=0; i<targets.size();i++){
            double height = m_robot->link(targets[i])->p(Z) - m_robot->rootLink()->p(Z);
            if(height < -1.0){
                hrp::dvector6 wrench;
                wrench << 0,0, -(height-(-1.0))*10000,0,0,0;

                if(m_robot->link(targets[i])->p(X)<0) wrench(X) += - m_robot->link(targets[i])->p(X) * 1000;

                hrp::JointPath jp(m_robot->rootLink(), m_robot->link(targets[i]));
                hrp::dmatrix J_ee;
                jp.calcJacobian(J_ee);
                hrp::dvector tq_for_apart = J_ee.transpose() * wrench;
                for (int j = 0; j < jp.numJoints(); j++) jp.joint(j)->u += tq_for_apart(j);
            }
        }
    }


    // calc real robot feedback force
    std::vector<hrp::Pose3> ee_pose(ee_names.size());
    for(int i=0; i<ee_pose.size();i++){
        ee_pose[i].p = m_robot->link(ee_ikc_map[ee_names[i]].target_link_name)->p;
        ee_pose[i].R = m_robot->link(ee_ikc_map[ee_names[i]].target_link_name)->R;
    }
    static std::vector<hrp::Pose3> ee_pose_old(ee_pose);
    for (int i=0; i<ee_names.size(); i++){
        const hrp::dvector6 wrench_raw = hrp::to_dvector(m_feedbackWrenches[ee_names[i]].data);
        wrench_filtered[ee_names[i]] = wrench_filter[ee_names[i]].passFilter( wrench_raw );
        const hrp::Vector3 ee_pos_vel = (ee_pose[i].p - ee_pose_old[i].p) / m_dt;
        const hrp::Vector3 ee_rot_vel = ee_pose_old[i].R * hrp::omegaFromRot(ee_pose_old[i].R.transpose() * ee_pose[i].R) / m_dt;
        ee_vel_filtered[ee_names[i]] = ee_vel_filter[ee_names[i]].passFilter((hrp::dvector6()<<ee_pos_vel,ee_rot_vel).finished());

        const hrp::dvector6 w_hpf = wrench_raw - wrench_lpf_for_hpf[ee_names[i]].passFilter( wrench_raw );
        const hrp::dvector6 w_lpf = wrench_lpf[ee_names[i]].passFilter( wrench_raw );
        wrench_shaped[ee_names[i]] = w_hpf * hcp.wrench_hpf_gain + w_lpf * hcp.wrench_lpf_gain;

        wrench_used[ee_names[i]] = wrench_shaped[ee_names[i]];
        LIMIT_NORM(wrench_used[ee_names[i]].head(3),50);
        LIMIT_NORM(wrench_used[ee_names[i]].tail(3),5);


//        if(f.norm() > 1e-6 ){ // avoid zero devide
//            f = f * fabs(1 - 1 * f.normalized().dot(ee_pos_vel) );
//        }
//        if(t.norm() > 1e-6){ // avoid zero devide
//            t = t * fabs(1 - 0.1 * t.normalized().dot(ee_rot_vel) );
//        }

//        hrp::Vector3 friction_f = - hcp.ee_friction_coeff(0) * ee_vel_filtered[ee_names[i]].head(3);
//        hrp::Vector3 friction_t = - hcp.ee_friction_coeff(1) * ee_vel_filtered[ee_names[i]].tail(3);

        if(ee_names[i] == "rarm"){
            m_debugData.data = hrp::to_DoubleSeq((hrp::dvector(6+6+6+6)<<wrench_raw,wrench_shaped[ee_names[i]],w_hpf,w_lpf).finished());
            m_debugData.tm = m_qRef.tm;
            m_debugDataOut.write();
            if(loop%1000==0){
                dbgv(ee_vel_filtered["rarm"]);
                dbgv(hcp.ee_friction_coeff);
                dbg(hcp.ee_vel_filter_cutoff_hz);
            }
        }

        hrp::JointPath jp(m_robot->rootLink(), m_robot->link(ee_ikc_map[ee_names[i]].target_link_name));
        hrp::dmatrix J_base_to_ee;
        jp.calcJacobian(J_base_to_ee, ee_ikc_map[ee_names[i]].localPos);
        hrp::dvector tq_from_feedbackWrench = J_base_to_ee.transpose() * wrench_used[ee_names[i]];
        for (int j = 0; j < jp.numJoints(); j++) jp.joint(j)->u += tq_from_feedbackWrench(j);
        if(loop%1000==0)dbgv(wrench_used[ee_names[i]]);
    }
    ee_pose_old = ee_pose;


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



//    m_debugData.data = hrp::to_DoubleSeq((hrp::dvector(27*2)<<hrp::to_dvector(m_dqAct.data),dqAct_filtered).finished());
//    m_debugData.tm = m_qRef.tm;
//    m_debugDataOut.write();


//    {
//        // real robot joint friction damper
//        const hrp::dvector q_friction_coeff_const = (hrp::dvector(m_robot->numJoints()) << 0,0,0,0,0,0, 0,0,0,0,0,0, 0, 3,3,2,2,1,0.5,0.5, 3,3,2,2,1,0.5,0.5).finished();
//        hrp::dvector friction_tq = - q_friction_coeff_const.array() * dqAct_filtered.array();
////        hrp::dvector friction_tq = - hcp.q_friction_coeff * dqAct_filtered;
//        for (int i=0; i<m_robot->numJoints(); i++){
//         LIMIT_MINMAX(friction_tq(i), -5, 5);
//         m_robot->joint(i)->u += friction_tq(i);
//        }
//        if(loop%1000==0)dbgv(friction_tq);
//    }



    const hrp::dvector max_torque = (hrp::dvector(m_robot->numJoints()) << 30,80,80,80,30,30, 30,80,80,80,30,30, 80, 30,30,25,25,20,10,10, 30,30,25,25,20,10,10).finished();

    for(int i=0;i<m_robot->numJoints();i++){
        LIMIT_MINMAX(m_robot->joint(i)->u, -max_torque(i), max_torque(i));
//        double tlimit = m_robot->joint(i)->climit * m_robot->joint(i)->gearRatio * m_robot->joint(i)->torqueConst;////?????
    }

    if(loop%1000==0)dbgv(hrp::getUAll(m_robot));

    updateInvDynStateBuffer(idsb);




    for (int i=0; i<tgt_names.size(); i++){
        if(tgt_names[i] == "com"){
            m_masterTgtPoses[tgt_names[i]].data = hrp::to_Pose3D(hrp::Pose3(m_robot->calcCM(), hrp::Matrix33::Identity()));
        }else if(tgt_names[i] == "head"){
            // dummy
        }else{
            m_masterTgtPoses[tgt_names[i]].data = hrp::to_Pose3D(ee_ikc_map[ee_names[i]].getCurrentTargetPose(m_robot));//四肢揃ってないと危険
        }
//        m_masterTgtPoses[tgt_names[i]].tm = m_qRef.tm;
        m_masterTgtPoses[tgt_names[i]].tm = m_qAct.tm;
        m_eePosesOut[tgt_names[i]]->write();
    }


    m_teleopOdom.data = (RTC::Pose3D){1.0,2.0,3.0,0.0,0.0,1.57* loop * 0.001};
    m_teleopOdom.tm = m_qRef.tm;
    m_teleopOdomOut.write();

}

void HapticController::processTransition(){
    switch(mode.now()){

        case MODE_SYNC_TO_HC:
            if(mode.pre() == MODE_IDLE){ double tmp = 1.0; t_ip->setGoal(&tmp, 10.0, true); }
            if (!t_ip->isEmpty() ){
                t_ip->get(&output_ratio, true);
            }else{
                mode.setNextMode(MODE_HC);
            }
            break;

        case MODE_SYNC_TO_IDLE:
            if(mode.pre() == MODE_HC || mode.pre() == MODE_PAUSE){ double tmp = 0.0; t_ip->setGoal(&tmp, 10.0, true); }
            if (!t_ip->isEmpty()) {
                t_ip->get(&output_ratio, true);
            }else{
                mode.setNextMode(MODE_IDLE);
            }
            break;
    }
}


//void HapticController::preProcessForHapticController(){
//    hrp::Vector3 basePos_heightChecked = hrp::to_Vector3(m_basePos.data);//ベースリンク高さ調整により足裏高さ0に
//    fik->m_robot->rootLink()->p = basePos_heightChecked;
////    for ( int i = 0; i < fik->m_robot->numJoints(); i++ ){ fik->m_robot->joint(i)->q = m_qRef.data[i]; }
//    hrp::setQAll(fik->m_robot, hrp::to_dvector(m_qRef.data));
//    fik->m_robot->calcForwardKinematics();
//
//    //TODO
////    hrp::Vector3 init_foot_mid_coord = (fik_in->getEndEffectorPos("rleg") + fik_in->getEndEffectorPos("lleg")) / 2;
////    if( fabs((double)init_foot_mid_coord(Z)) > 1e-5 ){
////        basePos_heightChecked(Z) -= init_foot_mid_coord(Z);
////        init_foot_mid_coord(Z) = 0;dqAct_filter
////        std::cerr<<"["<<m_profile.instance_name<<"] Input basePos height is invalid. Auto modify "<<m_basePos.data.z<<" -> "<<basePos_heightChecked(Z)<<endl;
////    }
//
//    std::vector<hrp::BodyPtr> body_list;
//    body_list.push_back(fik->m_robot);
//    body_list.push_back(m_robot_vsafe);
//    for(int i=0;i<body_list.size();i++){//初期姿勢でBodyをFK
//        hrp::setRobotStateVec(body_list[i], hrp::to_dvector(m_qRef.data), basePos_heightChecked, hrp::rotFromRpy(m_baseRpy.data.r, m_baseRpy.data.p, m_baseRpy.data.y));
//        body_list[i]->calcForwardKinematics();
//    }
//
//    fik->q_ref = hrp::getRobotStateVec(fik->m_robot);
//    q_ip->set(fik->q_ref.data());
//    wbms->initializeRequest(fik->m_robot, ee_ikc_map);
//}



bool HapticController::startHapticController(){
    if(mode.now() == MODE_IDLE){
        RTCOUT << "startHapticController" << std::endl;
        mode.setNextMode(MODE_SYNC_TO_HC);
        return true;
    }else{
        RTCOUT << "Invalid context to startHapticController" << std::endl;
        return false;
    }
}


bool HapticController::pauseHapticController(){
    if(mode.now() == MODE_HC){
        RTCOUT << "pauseHapticController" << std::endl;
        mode.setNextMode(MODE_PAUSE);
        return true;
    }else{
        RTCOUT << "Invalid context to pauseHapticController" << std::endl;
        return false;
    }
}


bool HapticController::resumeHapticController(){
    if(mode.now() == MODE_PAUSE){
        RTCOUT << "resumeHapticController" << std::endl;
        mode.setNextMode(MODE_HC);
        return true;
    }else{
        RTCOUT << "Invalid context to resumeHapticController" << std::endl;
        return false;
    }
}


bool HapticController::stopHapticController(){
    if(mode.now() == MODE_HC || mode.now() == MODE_PAUSE ){
        RTCOUT << "stopHapticController" << std::endl;
        mode.setNextMode(MODE_SYNC_TO_IDLE);
        return true;
    }else{
        RTCOUT << "Invalid context to stopHapticController" << std::endl;
        return false;
    }
}


bool HapticController::setParams(const OpenHRP::HapticControllerService::HapticControllerParam& i_param){
    RTCOUT << "setHapticControllerParam" << std::endl;
    hcp.gravity_compensation_ratio  = i_param.gravity_compensation_ratio;
    hcp.dqAct_filter_cutoff_hz      = i_param.dqAct_filter_cutoff_hz;
    {
        LIMIT_MAX(hcp.dqAct_filter_cutoff_hz, 1/m_dt/2);
        dqAct_filter.setParameter(hcp.dqAct_filter_cutoff_hz, 1/m_dt, Q_BUTTERWORTH);
        dqAct_filter.reset(0);
    }
    hcp.force_feedback_ratio        = i_param.force_feedback_ratio;
    hcp.q_friction_coeff            = i_param.q_friction_coeff;
    for(int i=0;i<hcp.ee_friction_coeff.size(); i++){ hcp.ee_friction_coeff(i) = i_param.ee_friction_coeff[i]; }
    hcp.wrench_filter_cutoff_hz     = i_param.wrench_filter_cutoff_hz;
    hcp.ee_vel_filter_cutoff_hz     = i_param.ee_vel_filter_cutoff_hz;
    hcp.wrench_hpf_cutoff_hz        = i_param.wrench_hpf_cutoff_hz;
    hcp.wrench_lpf_cutoff_hz        = i_param.wrench_lpf_cutoff_hz;
    for(int i=0;i<ee_names.size();i++){
        LIMIT_MAX(hcp.wrench_filter_cutoff_hz, 1/m_dt/2);
        LIMIT_MAX(hcp.ee_vel_filter_cutoff_hz, 1/m_dt/2);
        LIMIT_MAX(hcp.wrench_hpf_cutoff_hz, 1/m_dt/2);
        LIMIT_MAX(hcp.wrench_lpf_cutoff_hz, 1/m_dt/2);
        wrench_filter[ee_names[i]].setParameter(hcp.wrench_filter_cutoff_hz, 1/m_dt, Q_BUTTERWORTH);
        ee_vel_filter[ee_names[i]].setParameter(hcp.ee_vel_filter_cutoff_hz, 1/m_dt, Q_BUTTERWORTH);
        wrench_lpf_for_hpf[ee_names[i]].setParameter(hcp.wrench_hpf_cutoff_hz, 1/m_dt, Q_BUTTERWORTH);
        wrench_lpf[ee_names[i]].setParameter(hcp.wrench_lpf_cutoff_hz, 1/m_dt, Q_BUTTERWORTH);
        wrench_filter[ee_names[i]].reset(0);
        ee_vel_filter[ee_names[i]].reset(0);
        wrench_lpf_for_hpf[ee_names[i]].reset(0);
        wrench_lpf[ee_names[i]].reset(0);
    }
    hcp.wrench_hpf_gain             = i_param.wrench_hpf_gain;
    hcp.wrench_lpf_gain             = i_param.wrench_lpf_gain;
    return true;
}


bool HapticController::getParams(OpenHRP::HapticControllerService::HapticControllerParam& i_param){
    RTCOUT << "getHapticControllerParam" << std::endl;
    i_param.gravity_compensation_ratio  = hcp.gravity_compensation_ratio;
    i_param.dqAct_filter_cutoff_hz      = hcp.dqAct_filter_cutoff_hz;
    i_param.wrench_filter_cutoff_hz     = hcp.wrench_filter_cutoff_hz;
    i_param.ee_vel_filter_cutoff_hz     = hcp.ee_vel_filter_cutoff_hz;
    i_param.force_feedback_ratio        = hcp.force_feedback_ratio;
    i_param.q_friction_coeff            = hcp.q_friction_coeff;
    for(int i=0;i<hcp.ee_friction_coeff.size(); i++){ i_param.ee_friction_coeff[i] = hcp.ee_friction_coeff(i); }
    i_param.wrench_hpf_cutoff_hz        = hcp.wrench_hpf_cutoff_hz;
    i_param.wrench_lpf_cutoff_hz        = hcp.wrench_lpf_cutoff_hz;
    i_param.wrench_hpf_gain             = hcp.wrench_hpf_gain;
    i_param.wrench_lpf_gain             = hcp.wrench_lpf_gain;
    return true;
}


RTC::ReturnCode_t HapticController::onActivated(RTC::UniqueId ec_id){ RTCOUT << "onActivated(" << ec_id << ")" << std::endl; return RTC::RTC_OK; }
RTC::ReturnCode_t HapticController::onDeactivated(RTC::UniqueId ec_id){ RTCOUT << "onDeactivated(" << ec_id << ")" << std::endl; return RTC::RTC_OK; }
RTC::ReturnCode_t HapticController::onFinalize(){ return RTC::RTC_OK; }

extern "C"{
    void HapticControllerInit(RTC::Manager* manager) {
        RTC::Properties profile(HapticController_spec);
        manager->registerFactory(profile, RTC::Create<HapticController>, RTC::Delete<HapticController>);
    }
};
