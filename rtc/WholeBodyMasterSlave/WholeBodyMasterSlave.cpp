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
    RTC::Manager& rtcManager = RTC::Manager::instance();
    std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
    int comPos = nameServer.find(",");
    if (comPos < 0){ comPos = nameServer.length(); }
    nameServer = nameServer.substr(0, comPos);
    RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
    m_robot = hrp::BodyPtr(new hrp::Body());
    m_robot_ml = hrp::BodyPtr(new hrp::Body());
    m_robot_vsafe = hrp::BodyPtr(new hrp::Body());
    if (!loadBodyFromModelLoader(m_robot, prop["model"].c_str(), CosNaming::NamingContext::_duplicate(naming.getRootContext()) )){
        std::cerr << "[" << m_profile.instance_name << "] failed to load model[" << prop["model"] << "]" << std::endl;
        return RTC::RTC_ERROR;
    }
    m_robot_ml = hrp::BodyPtr(new hrp::Body(*m_robot)); //copy
    m_robot_vsafe = hrp::BodyPtr(new hrp::Body(*m_robot)); //copy
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
    ROBOT_ALL_DOF = m_robot->numJoints() + 3 + 3;// joints + base_p + base_rpy
    //    q_ip = new interpolator(ROBOT_ALL_DOF, m_dt, interpolator::HOFFARBIB, 1);
    //    q_ip = new interpolator(ROBOT_ALL_DOF, m_dt, interpolator::QUINTICSPLINE, 1);
    q_ip = new interpolator(ROBOT_ALL_DOF, m_dt, interpolator::CUBICSPLINE, 1);
    q_ip->setName(std::string(m_profile.instance_name)+" q_ip");
    q_ip->clear();
    // Generate FIK
    fik = fikPtr(new FullbodyInverseKinematicsSolver(m_robot, std::string(m_profile.instance_name), m_dt));
    fik_ml = fikPtr(new FullbodyInverseKinematicsSolver(m_robot_ml, std::string(m_profile.instance_name), m_dt));
    fik_list.push_back(fik);
    fik_list.push_back(fik_ml);
    body_list.push_back(m_robot);
    body_list.push_back(m_robot_ml);
    body_list.push_back(m_robot_vsafe);

    setupfik(fik, m_robot, prop);
    setupfik(fik_ml, m_robot_ml, prop);

    if (fik->ikp.find("rleg") != fik->ikp.end() && fik->ikp.find("lleg") != fik->ikp.end()) {
        is_legged_robot = true;
    } else {
        is_legged_robot = false;
    }
    hsp = boost::shared_ptr<WBMSCore>(new WBMSCore(m_dt));

    invdyn_zmp_filters.setParameter(25, 1/m_dt, Q_BUTTERWORTH);
    final_ref_zmp_filter.setParameter(5, 1/m_dt, Q_BUTTERWORTH);

    avg_q_vel = 4;

    sccp = boost::shared_ptr<CapsuleCollisionChecker>(new CapsuleCollisionChecker(m_robot));

    std::cerr << "[" << m_profile.instance_name << "] onInitialize() OK" << std::endl;
    return RTC::RTC_OK;
}

RTC::ReturnCode_t WholeBodyMasterSlave::onFinalize(){
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

            eename_ikcp_map[ee_name].target_link_name = ee_target;
            for (size_t j = 0; j < XYZ; j++){ coil::stringTo(eename_ikcp_map[ee_name].localPos(j), end_effectors_str[i*prop_num+3+j].c_str()); }
            double tmp[4];
            for (int j = 0; j < 4; j++ ){ coil::stringTo(tmp[j], end_effectors_str[i*prop_num+6+j].c_str()); }
            eename_ikcp_map[ee_name].localR = Eigen::AngleAxis<double>(tmp[3], hrp::Vector3(tmp[0], tmp[1], tmp[2])).toRotationMatrix();
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
    if(mode.now()!=MODE_PAUSE){
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

    if ( is_legged_robot ) {
        processTransition();
        mode.update();
        if(DEBUGP)dbg(mode.now());
        struct timespec startT, endT;
        clock_gettime(CLOCK_REALTIME, &startT);

        if(DEBUGP && TIMECALC){clock_gettime(CLOCK_REALTIME, &endT); std::cout << (double)(endT.tv_sec - startT.tv_sec + (endT.tv_nsec - startT.tv_nsec) * 1e-9) << " @ processTransition" << std::endl;  clock_gettime(CLOCK_REALTIME, &startT);}

        for(int i=0; i<fik_list.size(); i++){
            fik_list[i]->ikp["rleg"].is_ik_enable = true;
            fik_list[i]->ikp["lleg"].is_ik_enable = true;
            if(fik_list[i]->ikp.count("rarm"))fik_list[i]->ikp["rarm"].is_ik_enable = hsp->WBMSparam.use_rh;
            if(fik_list[i]->ikp.count("larm"))fik_list[i]->ikp["larm"].is_ik_enable = hsp->WBMSparam.use_lh;
        }

        if (mode.isRunning()) {
            if(mode.isInitialize()){
                preProcessForWholeBodyMasterSlave(fik, m_robot);
                hsp->fik_ml = fik_ml;
                hsp->m_robot_ml = m_robot_ml;
                //          hsp->fik_act = fik_vsafe;
                hsp->m_robot_act = m_robot_vsafe;
                //逆動力学初期化
                idsb.setInitState(m_robot, m_dt);
            }

            if(hsp->WBMSparam.is_doctor){
                processWholeBodyMasterSlave(fik, m_robot, hsp->rp_ref_out);//安全制限つきマスタ・スレーブ
            }else{
                processWholeBodyMasterSlave_Raw(fik, m_robot, raw_pose);//生マスタ・スレーブ
            }

            if(DEBUGP && TIMECALC){clock_gettime(CLOCK_REALTIME, &endT); std::cout << (double)(endT.tv_sec - startT.tv_sec + (endT.tv_nsec - startT.tv_nsec) * 1e-9) << " @ processWholeBodyMasterSlave" << std::endl;  clock_gettime(CLOCK_REALTIME, &startT);}

            //逆動力学
            //        calcAccelerationsForInverseDynamics(m_robot, idsb);
            //        hrp::Vector3 ref_zmp_invdyn;
            //        calcWorldZMPFromInverseDynamics(m_robot, idsb, ref_zmp_invdyn);
            //        ref_zmp_invdyn = invdyn_zmp_filters.passFilter(ref_zmp_invdyn);
            //        updateInvDynStateBuffer(idsb);

            if(DEBUGP && TIMECALC){clock_gettime(CLOCK_REALTIME, &endT); std::cout << (double)(endT.tv_sec - startT.tv_sec + (endT.tv_nsec - startT.tv_nsec) * 1e-9) << " @ calcWorldZMPFromInverseDynamics" << std::endl;  clock_gettime(CLOCK_REALTIME, &startT);}

            sccp->checkCollision();

            processHOFFARBIBFilter(m_robot, m_robot_vsafe);

            const hrp::BodyPtr m_robot_for_out = m_robot_vsafe;
            //        const hrp::BodyPtr m_robot_for_out = m_robot;

            //OutPortデータセット
            //        hrp::Vector3 ref_zmp = hsp->rp_ref_out.tgt[zmp].abs.p;

            hrp::Vector3 com = m_robot_for_out->calcCM();
            static hrp::Vector3 com_old = com;
            static hrp::Vector3 com_old_old = com_old;
            hrp::Vector3 com_acc = (com - 2*com_old + com_old_old)/(m_dt*m_dt);
            hrp::Vector3 ref_zmp; ref_zmp << com.head(XY)-(com(Z)/G)*com_acc.head(XY), 0;
            if(mode.isInitialize()){ final_ref_zmp_filter.reset(ref_zmp); }
            ref_zmp = final_ref_zmp_filter.passFilter(ref_zmp);
            com_old_old = com_old;
            com_old = com;
            hsp->act_rs.com = com;
            hsp->act_rs.zmp = ref_zmp;

            // qRef
            for (int i = 0; i < m_qRef.data.length(); i++ ){
                m_qRef.data[i] = transition_interpolator_ratio * m_robot_for_out->joint(i)->q  + (1 - transition_interpolator_ratio) * m_qRef.data[i];
            }
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
            m_optionalData.data[contact_states_index_map["rleg"]] = m_optionalData.data[optionalDataLength/2 + contact_states_index_map["rleg"]] = hsp->rp_ref_out.tgt[rf].is_contact;
            m_optionalData.data[contact_states_index_map["lleg"]] = m_optionalData.data[optionalDataLength/2 + contact_states_index_map["lleg"]] = hsp->rp_ref_out.tgt[lf].is_contact;
        }
        hsp->baselinkpose.p = m_robot->rootLink()->p;
        hsp->baselinkpose.rpy = hrp::rpyFromRot(m_robot->rootLink()->R);
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
    hrp::Vector3 init_foot_mid_coord = (fik_in->getEndEffectorPos("rleg") + fik_in->getEndEffectorPos("lleg")) / 2;
    if( fabs((double)init_foot_mid_coord(Z)) > 1e-5 ){
        basePos_heightChecked(Z) -= init_foot_mid_coord(Z);
        init_foot_mid_coord(Z) = 0;
        std::cerr<<"["<<m_profile.instance_name<<"] Input basePos height is invalid. Auto modify "<<m_basePos.data.z<<" -> "<<basePos_heightChecked(Z)<<endl;
    }
    const std::string robot_l_names[4] = {"rleg","lleg","rarm","larm"};

    for(int i=0;i<body_list.size();i++){//初期姿勢でBodyをFK
        body_list[i]->rootLink()->p = basePos_heightChecked;
        body_list[i]->rootLink()->R = hrp::rotFromRpy(m_baseRpy.data.r, m_baseRpy.data.p, m_baseRpy.data.y);
        for ( int j = 0; j < body_list[i]->numJoints(); j++ ){ body_list[i]->joint(j)->q = m_qRef.data[j]; }
        body_list[i]->calcForwardKinematics();
    }
    for(int i=0;i<fik_list.size();i++){//初期姿勢でBodyをFK
        fik_list[i]->setReferenceJointAngles();
        for(int l=0;l<4;l++){//targetを初期化
            if(fik_list[i]->ikp.count(robot_l_names[l])){
                fik_list[i]->ikp[robot_l_names[l]].target_p0 = fik_list[i]->getEndEffectorPos(robot_l_names[l]);
                fik_list[i]->ikp[robot_l_names[l]].target_r0 = fik_list[i]->getEndEffectorRot(robot_l_names[l]);
            }
        }
    }
//    q_ip->clear();//clearは2回目以降しないほうがいい
    init_sync_state.resize(ROBOT_ALL_DOF);
    init_sync_state.head(robot_in->numJoints()) = hrp::getQAll(robot_in);
    init_sync_state.tail(6).head(3) = robot_in->rootLink()->p;
    init_sync_state.tail(6).tail(3) = hrp::rpyFromRot(robot_in->rootLink()->R);
    q_ip->set(init_sync_state.data());
    hsp->initializeRequest(fik_in, robot_in);
}


void WholeBodyMasterSlave::processWholeBodyMasterSlave(fikPtr& fik_in, hrp::BodyPtr& robot_in, const HumanPose& pose_ref){
    hsp->update();//////HumanSynchronizerの主要処理
    if(DEBUGP)cout<<"update():"<<hsp->getUpdateTime()<<endl;
    if(DEBUGP)pose_ref.print();
    solveFullbodyIKStrictCOM(fik_in, robot_in, pose_ref.tgt[com].abs, pose_ref.tgt[rf].abs, pose_ref.tgt[lf].abs, pose_ref.tgt[rh].abs, pose_ref.tgt[lh].abs, pose_ref.tgt[head].abs,"processWholeBodyMasterSlave");
}


void WholeBodyMasterSlave::processWholeBodyMasterSlave_Raw(fikPtr& fik_in, hrp::BodyPtr& robot_in, HumanPose& pose_ref){
    static BiquadIIRFilterVec pos_filters[num_pose_tgt], rot_filters[num_pose_tgt];
    static unsigned int callnum;
    for(int i=0;i<num_pose_tgt;i++){
        if(callnum == 0){
            pos_filters[i].setParameter(100.0, 1.0/m_dt, Q_NOOVERSHOOT);
            rot_filters[i].setParameter(100.0, 1.0/m_dt, Q_NOOVERSHOOT);
            pos_filters[i].reset(hsp->hp_wld_raw.tgt[i].abs.p);
            rot_filters[i].reset(hsp->hp_wld_raw.tgt[i].abs.rpy);
        }
        pose_ref.tgt[i].abs.p = pos_filters[i].passFilter(hsp->hp_wld_raw.tgt[i].abs.p);
        pose_ref.tgt[i].abs.rpy = rot_filters[i].passFilter(hsp->hp_wld_raw.tgt[i].abs.rpy);
    }
    callnum++;
    if(DEBUGP){ fprintf(stderr,"\x1b[31mmaster-mode:\x1b[39m"); pose_ref.print(); }
    //  solveFullbodyIKStrictCOM(fik_in, robot_in, pose_ref.tgt[com].abs, pose_ref.tgt[rf].abs, pose_ref.tgt[lf].abs, pose_ref.tgt[rh].abs, pose_ref.tgt[lh].abs, pose_ref.tgt[head].abs,"processWholeBodyMasterSlave_Raw");
}

void WholeBodyMasterSlave::calcManipulability(fikPtr& fik_in, hrp::BodyPtr& robot_in){
    const std::string names[4] = {"rleg","lleg","rarm","larm"};
    hrp::dmatrix J,Jinv,Jnull;
    for(int l=0; l<4; l++){
        if(fik_in->ikp.count(names[l])){
            fik_in->ikp[names[l]].manip->calcJacobian(J);
            fik_in->ikp[names[l]].manip->calcJacobianInverseNullspace(J, Jinv, Jnull);
            Eigen::JacobiSVD< Eigen::MatrixXd > svd(J.block(0,0,3,J.cols()), Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::JacobiSVD< Eigen::MatrixXd > svd_rot(J.block(3,0,3,J.cols()), Eigen::ComputeFullU | Eigen::ComputeFullV);
            hsp->manip_mat[l] = svd.matrixU();
            hsp->manip_mat_rot[l] = svd_rot.matrixU();
            for(int i=0;i<3;i++){
                hsp->manip_sv[l](i) = svd.singularValues()(i);
                hsp->manip_sv_rot[l](i) = svd_rot.singularValues()(i);
            }
        }
    }
}

#ifdef USE_NEW_FIK
void WholeBodyMasterSlave::solveFullbodyIKStrictCOM(fikPtr& fik_in, hrp::BodyPtr& robot_in, const WBMSPose3D& com_ref, const WBMSPose3D& rf_ref, const WBMSPose3D& lf_ref, const WBMSPose3D& rh_ref, const WBMSPose3D& lh_ref, const WBMSPose3D& head_ref, const std::string& debug_prefix){
    std::vector<IKConstraint> ikc_list;
    {
        IKConstraint tmp;
        tmp.target_link_name = "WAIST";
        tmp.localPos = hrp::Vector3::Zero();
        tmp.localR = hrp::Matrix33::Identity();
        tmp.targetPos = robot_in->rootLink()->p;// will be ignored by selection_vec
        tmp.targetRpy = com_ref.rpy;// ベースリンクの回転をフリーにはしないほうがいい(omegaの積分誤差で暴れる)
        tmp.constraint_weight << 0,0,0,1,1,1;
        tmp.rot_precision = deg2rad(3);
        ikc_list.push_back(tmp);
    }{
        IKConstraint tmp;
        tmp.target_link_name = eename_ikcp_map["rleg"].target_link_name;
        tmp.localPos = eename_ikcp_map["rleg"].localPos;
        tmp.localR = eename_ikcp_map["rleg"].localR;
        tmp.targetPos = rf_ref.p;
        tmp.targetRpy = rf_ref.rpy;
        if(hsp->rp_ref_out.tgt[rf].is_contact){
            tmp.constraint_weight = hrp::dvector6::Constant(10);
        }else{
            tmp.constraint_weight = hrp::dvector6::Constant(0.1);
        }
        ikc_list.push_back(tmp);
    }{
        IKConstraint tmp;
        tmp.target_link_name = eename_ikcp_map["lleg"].target_link_name;
        tmp.localPos = eename_ikcp_map["lleg"].localPos;
        tmp.localR = eename_ikcp_map["lleg"].localR;
        tmp.targetPos = lf_ref.p;
        tmp.targetRpy = lf_ref.rpy;
        if(hsp->rp_ref_out.tgt[lf].is_contact){
            tmp.constraint_weight = hrp::dvector6::Constant(10);
        }else{
            tmp.constraint_weight = hrp::dvector6::Constant(0.1);
        }
        ikc_list.push_back(tmp);
    }{
        IKConstraint tmp;
        tmp.target_link_name = eename_ikcp_map["rarm"].target_link_name;
        tmp.localPos = eename_ikcp_map["rarm"].localPos;
        tmp.localR = eename_ikcp_map["rarm"].localR;
        tmp.targetPos = rh_ref.p;
        tmp.targetRpy = rh_ref.rpy;
        tmp.constraint_weight = hrp::dvector6::Constant(0.1);
        tmp.pos_precision = 3e-3;
        tmp.rot_precision = deg2rad(3);
        ikc_list.push_back(tmp);
    }{
        IKConstraint tmp;
        tmp.target_link_name = eename_ikcp_map["larm"].target_link_name;
        tmp.localPos = eename_ikcp_map["larm"].localPos;
        tmp.localR = eename_ikcp_map["larm"].localR;
        tmp.targetPos = lh_ref.p;
        tmp.targetRpy = lh_ref.rpy;
        tmp.constraint_weight = hrp::dvector6::Constant(0.1);
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
    for(int i=0;i<sccp->collision_info_list.size();i++){
        IKConstraint tmp;
        tmp.localPos = sccp->collision_info_list[i].cp1_local;
        tmp.target_link_name = m_robot->joint(sccp->collision_info_list[i].id1)->name;
        tmp.targetPos = sccp->collision_info_list[i].cp0_wld + (sccp->collision_info_list[i].cp1_wld - sccp->collision_info_list[i].cp0_wld).normalized() * (sccp->collision_info_list[i].dist_safe + 1e-3);
        tmp.constraint_weight << 1,1,1,0,0,0;
        ikc_list.push_back(tmp);
    }


    {
        IKConstraint tmp;
        tmp.target_link_name = "COM";
        tmp.localPos = hrp::Vector3::Zero();
        tmp.localR = hrp::Matrix33::Identity();
        tmp.targetPos = com_ref.p;// COM height will not be constraint
        tmp.targetRpy = hrp::Vector3::Zero();//reference angular momentum
        tmp.constraint_weight << 10,10,1,1e-6,1e-6,1e-6;
//        tmp.constraint_weight << 10,10,1,0,0,0;
//        if(fik_in->cur_momentum_around_COM.norm() > 1e9){
//            tmp.constraint_weight << 10,10,1,1e-5,1e-5,1e-10;
//            tmp.rot_precision = 100;//angular momentum precision
//        }else{
//            tmp.constraint_weight << 10,10,1,0,0,0;
//        }
        ikc_list.push_back(tmp);
    }

    fik_in->dq_weight_all(robot_in->link("CHEST_JOINT0")->jointId) = 0.1;
    fik_in->dq_weight_all(robot_in->link("CHEST_JOINT1")->jointId) = 0.1;
    fik_in->dq_weight_all(robot_in->link("CHEST_JOINT2")->jointId) = 0.1;
//    if( robot_in->link("RARM_JOINT2") != NULL) robot_in->link("RARM_JOINT2")->ulimit = deg2rad(-40);//脇内側の干渉回避
//    if( robot_in->link("LARM_JOINT2") != NULL) robot_in->link("LARM_JOINT2")->llimit = deg2rad(40);
//    if( robot_in->link("RARM_JOINT2") != NULL) robot_in->link("RARM_JOINT2")->llimit = deg2rad(-140);//肩外側の干渉回避
//    if( robot_in->link("LARM_JOINT2") != NULL) robot_in->link("LARM_JOINT2")->ulimit = deg2rad(140);

    for(int i=0;i<robot_in->numJoints();i++){
        LIMIT_MINMAX(robot_in->joint(i)->q, robot_in->joint(i)->llimit, robot_in->joint(i)->ulimit);
    }

    fik_in->q_ref = init_sync_state;

    fik_in->q_ref_pullback_gain.fill(0.001);
//    fik_in->dq_ref_pullback.fill(deg2rad(0.01));
    fik_in->q_ref_pullback_gain.segment(6+6+3+2, 8*2).fill(0.01);//腕だけ
//    fik_in->dq_ref_pullback.segment(6+6+3+2, 8*2).fill(deg2rad(0.1));//腕だけ

//    fik_in->q_ref_pullback_gain.tail(6) << 0.0,0.0,0.0, 0.001,0.001,0.001;

    struct timespec startT, endT;
    const int IK_MAX_LOOP = 2;
    clock_gettime(CLOCK_REALTIME, &startT);
    int loop_result = fik_in->solveFullbodyIKLoop(robot_in, ikc_list, IK_MAX_LOOP);
    if(loop%100==0){clock_gettime(CLOCK_REALTIME, &endT); std::cout << (endT.tv_sec - startT.tv_sec + (endT.tv_nsec - startT.tv_nsec) * 1e-9) << " @ solveIK"<<loop_result<<"loop" << std::endl;}
}
#else
//旧ver
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
            fik_in->ikp[names[i]].targetPos = refs[i]->p;
            fik_in->ikp[names[i]].targetRpy = hrp::rotFromRpy(refs[i]->rpy);
        }
    }
    //  fik_in->storeCurrentParameters();
    //  fik_in->setReferenceJointAngles();//これ入れると腕ブワーなる
    if( robot_in->link("HEAD_JOINT0") != NULL) robot_in->link("HEAD_JOINT0")->q = head_ref.rpy(y);
    if( robot_in->link("HEAD_JOINT1") != NULL) robot_in->link("HEAD_JOINT1")->q = head_ref.rpy(p);
    if(fik_in->ikp.count("rarm") && fik_in->ikp.count("larm")){
        hrp::Vector3 base2rh = robot_in->rootLink()->R.transpose() * (fik_in->ikp["rarm"].targetPos - robot_in->rootLink()->p);
        hrp::Vector3 base2lh = robot_in->rootLink()->R.transpose() * (fik_in->ikp["larm"].targetPos - robot_in->rootLink()->p);
        if( robot_in->link("CHEST_JOINT0") != NULL){
            robot_in->link("CHEST_JOINT0")->q = (base2lh(Z) - base2rh(Z)) * (10 * D2R / 1.0);
            LIMIT_MINMAX(robot_in->link("CHEST_JOINT0")->q, robot_in->link("CHEST_JOINT0")->llimit, robot_in->link("CHEST_JOINT0")->ulimit);
        }
        if(robot_in->link("CHEST_JOINT2") != NULL){
            robot_in->link("CHEST_JOINT2")->q = (base2rh(X) - base2lh(X)) * (60 * D2R / 1.0);
            LIMIT_MINMAX(robot_in->link("CHEST_JOINT2")->q, robot_in->link("CHEST_JOINT2")->llimit, robot_in->link("CHEST_JOINT2")->ulimit);
        }
    }
    hrp::Vector3 tmp_com_err = hrp::Vector3::Zero();
    struct timespec startT, endT;
    while( 1 ){  //COM 収束ループ
        com_ik_loop++;
        robot_in->rootLink()->p += tmp_com_err;
        robot_in->calcForwardKinematics();
        clock_gettime(CLOCK_REALTIME, &startT);
        for ( std::map<std::string, SimpleFullbodyInverseKinematicsSolver::IKparam>::iterator it = fik_in->ikp.begin(); it != fik_in->ikp.end(); it++ ) {
            if (it->second.is_ik_enable) fik_in->solveLimbIK (it->second, it->first, fik_in->ratio_for_vel, false);
        }
        if(DEBUGP && TIMECALC){clock_gettime(CLOCK_REALTIME, &endT); std::cout << (double)(endT.tv_sec - startT.tv_sec + (endT.tv_nsec - startT.tv_nsec) * 1e-9) << " @ solveLimbIK x 4" << std::endl;  clock_gettime(CLOCK_REALTIME, &startT);}
        tmp_com_err = com_ref.p - robot_in->calcCM();
        if(tmp_com_err.norm() < COM_IK_MAX_ERROR){ break; }
        if(com_ik_loop >= COM_IK_MAX_LOOP){std::cerr << "COM constraint IK MAX loop [="<<COM_IK_MAX_LOOP<<"] exceeded!! @ "<<debug_prefix<< std::endl; break; };
    }
    if(com_ik_loop != 1 && DEBUGP)cout<<"com_ik_loop:"<<com_ik_loop<<" @ "<<debug_prefix<<endl;
}
#endif


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
        double tmp_time = fabs(robot_in->joint(i)->q - robot_out->joint(i)->q) / avg_q_vel;
        LIMIT_MIN(tmp_time, fabs(tmp_v(i))/16);
//        tmp_time += fabs(tmp_v(i))/16; //加速度8
        if(tmp_time > goal_time){ goal_time = tmp_time; }
    }
    q_ip->setGoal(goal_state.data(), goal_time + min_goal_time_offset, true);
    hrp::dvector ans_state(ROBOT_ALL_DOF);
    double tmp[ROBOT_ALL_DOF];
//    if (!q_ip->isEmpty() ){  q_ip->get(tmp, true);}
    double tmpv[ROBOT_ALL_DOF];
    if (!q_ip->isEmpty() ){  q_ip->get(tmp, tmpv, true);}
    tmp_v = Eigen::Map<hrp::dvector>(tmpv, ROBOT_ALL_DOF);
    ans_state = Eigen::Map<hrp::dvector>(tmp, ROBOT_ALL_DOF);
    for(int i=0;i<robot_out->numJoints();i++){ robot_out->joint(i)->q = ans_state(i); }
    robot_out->rootLink()->p = ans_state.bottomRows(6).topRows(3);
    robot_out->rootLink()->R = hrp::rotFromRpy(ans_state.bottomRows(6).bottomRows(3));
    robot_out->calcForwardKinematics();
}


bool WholeBodyMasterSlave::startCountDownForWholeBodyMasterSlave(const double sec){//遅い
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


RTC::ReturnCode_t WholeBodyMasterSlave::onActivated(RTC::UniqueId ec_id){ std::cerr << "[" << m_profile.instance_name<< "] onActivated(" << ec_id << ")" << std::endl; return RTC::RTC_OK; }
RTC::ReturnCode_t WholeBodyMasterSlave::onDeactivated(RTC::UniqueId ec_id){ std::cerr << "[" << m_profile.instance_name<< "] onDeactivated(" << ec_id << ")" << std::endl; return RTC::RTC_OK; }

extern "C"{
    void WholeBodyMasterSlaveInit(RTC::Manager* manager) {
        RTC::Properties profile(WholeBodyMasterSlave_spec);
        manager->registerFactory(profile, RTC::Create<WholeBodyMasterSlave>, RTC::Delete<WholeBodyMasterSlave>);
    }
};
