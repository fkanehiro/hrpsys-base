#ifndef HUMANMASTERSLAVE_H
#define HUMANMASTERSLAVE_H
#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include "interpolator.h"
#include "../TorqueFilter/IIRFilter.h"
#include <hrpModel/Body.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <hrpUtil/Eigen4d.h>
#include <numeric>

#if defined(__cplusplus)
extern "C" {
#endif
#include <stdio.h>
#include <stdlib.h>
#include <qhull/qhull.h>
#include <qhull/mem.h>
#include <qhull/qset.h>
#include <qhull/geom.h>
#include <qhull/merge.h>
#include <qhull/poly.h>
#include <qhull/io.h>
#include <qhull/stat.h>
#if defined(__cplusplus)
}
#endif

#define DEBUG 1
#define dbg(var) std::cout<<#var"= "<<(var)<<std::endl
#define LIMIT_MIN(x,min) (x= ( x<min ? min:x ))
#define LIMIT_MAX(x,max) (x= ( x<max ? x:max ))
#define LIMIT_MINMAX(x,min,max) ((x= (x<min  ? min : x<max ? x : max)))
#define SGN(x) ((x)>=0 ? 1 : -1)

class UTIL_CONST{
  public:
    enum { com, rf, lf, rh, lh, head, zmp, num_pose_tgt };
    enum { num_ee_tgt=4 };
    enum { R, L, LR };
    enum { X, Y, Z, XYZ };
    enum { r, p, y, rpy };
    enum { fx, fy, fz, tx, ty, tz, ft_xyz };
    enum { MIN, MAX, MINMAX };
    double G, D2R, INFMIN, INFMAX, Q_BUTTERWORTH, Q_NOOVERSHOOT;
    UTIL_CONST() :
      G(9.80665),
      D2R( M_PI/180.0),
      Q_BUTTERWORTH(0.707106781),
      Q_NOOVERSHOOT(0.5),
      INFMIN( - std::numeric_limits<double>::max()),
      INFMAX( + std::numeric_limits<double>::max())
    {};
};

class BiquadIIRFilterVec : UTIL_CONST {
  private:
    IIRFilter filters[XYZ];
    hrp::Vector3 ans;
  public:
    BiquadIIRFilterVec(){}
    ~BiquadIIRFilterVec(){}
    void setParameter(const hrp::Vector3& fc_in, const double& HZ, const double& Q = 0.5){ for(int i=0;i<XYZ;i++){ filters[i].setParameterAsBiquad((double)fc_in(i), Q, HZ); } }
    void setParameter(const double& fc_in, const double& HZ, const double& Q = 0.5){ setParameter(hrp::Vector3(fc_in,fc_in,fc_in), HZ, Q); }//overload
    hrp::Vector3 passFilter(const hrp::Vector3& input){ for(int i=0;i<XYZ;i++){ ans(i) = filters[i].passFilter((double)input(i)); } return ans; }
    void reset(const hrp::Vector3& initial_input){ for(int i=0;i<XYZ;i++){ filters[i].reset((double)initial_input(i));} }
};

class WBMSPose3D{
  public:
    hrp::Vector3 p,rpy;
    WBMSPose3D(){ clear(); }
    ~WBMSPose3D(){}
    void clear(){ p = rpy = hrp::Vector3::Zero(); }
};

class PoseTGT{
  public:
    WBMSPose3D abs, offs, cnt;
    hrp::dvector6 w;
    bool is_contact, go_contact;
    PoseTGT(){ clear(); }
    ~PoseTGT(){}
    void clear(){ abs.clear(); offs.clear(); cnt.clear(); w=hrp::dvector6::Zero(); is_contact = go_contact = false;}
};

class HumanPose : UTIL_CONST {
  public:
    std::vector<PoseTGT> tgt;
    HumanPose(){ tgt.resize(num_pose_tgt); }
    ~HumanPose(){cerr<<"HumanPose destructed"<<endl;}
    void clear(){
      for(std::vector<PoseTGT>::iterator it = tgt.begin(); it != tgt.end(); it++){ it->clear(); }
    }
    static void hp_printf(const HumanPose& in) {
      const std::string pcatgt[] = {"c","rf","lf","rh","lh","hd","z"}, wcatgt[] = {"rw","lw"};
      for(int i=0;i<num_pose_tgt;i++){ fprintf(stderr,"\x1b[31m%s\x1b[39m%+05.2f %+05.2f %+05.2f ",pcatgt[i].c_str(),in.tgt[i].abs.p(X),in.tgt[i].abs.p(Y),in.tgt[i].abs.p(Z)); }
//      for(int i=0;i<num_wrench_tgt;i++){ fprintf(stderr,"\x1b[31m%s\x1b[39m%+05.1f %+05.1f %+05.1f %+05.1f %+05.1f %+05.1f ",wcatgt[i].c_str(),in.w[i](fx),in.w[i](fy),in.w[i](fz),in.w[i](tx),in.w[i](ty),in.w[i](tz)); }
      printf("\n");
    }
    void print() const { hp_printf(*this); }
};

class RobotConfig : UTIL_CONST {
  public:
    std::vector< std::vector<hrp::Vector3> > ee_rot_limit;
    RobotConfig(){
      std::vector<hrp::Vector3> init;
      init.resize(MINMAX);
      ee_rot_limit.resize(num_pose_tgt, init);
//      ee_rot_limit[com][MIN] = hrp::Vector3(-10*D2R, -10*D2R, INFMIN);
//      ee_rot_limit[com][MAX] = hrp::Vector3( 10*D2R,  10*D2R, INFMAX);
      ee_rot_limit[com][MIN] = hrp::Vector3(-10*D2R, -10*D2R, -30*D2R);
      ee_rot_limit[com][MAX] = hrp::Vector3( 10*D2R,  10*D2R, 30*D2R);
      ee_rot_limit[rf][MIN] = hrp::Vector3(-30*D2R, -30*D2R, -20*D2R);
      ee_rot_limit[rf][MAX] = hrp::Vector3( 30*D2R,  30*D2R,   5*D2R);
      ee_rot_limit[lf][MIN] = hrp::Vector3(-30*D2R, -30*D2R,  -5*D2R);
      ee_rot_limit[lf][MAX] = hrp::Vector3( 30*D2R,  30*D2R,  20*D2R);
//      ee_rot_limit[rh][MIN] = hrp::Vector3(-30*D2R, -30*D2R, -30*D2R);
//      ee_rot_limit[rh][MAX] = hrp::Vector3( 30*D2R,  30*D2R,  30*D2R);
//      ee_rot_limit[lh][MIN] = hrp::Vector3(-30*D2R, -30*D2R, -30*D2R);
//      ee_rot_limit[lh][MAX] = hrp::Vector3( 30*D2R,  30*D2R,  30*D2R);
      ee_rot_limit[rh][MIN] = hrp::Vector3(INFMIN, INFMIN, INFMIN);
      ee_rot_limit[rh][MAX] = hrp::Vector3(INFMAX, INFMAX, INFMAX);
      ee_rot_limit[lh][MIN] = hrp::Vector3(INFMIN, INFMIN, INFMIN);
      ee_rot_limit[lh][MAX] = hrp::Vector3(INFMAX, INFMAX, INFMAX);
      ee_rot_limit[head][MIN] = hrp::Vector3( 0*D2R, -20*D2R, -40*D2R);
      ee_rot_limit[head][MAX] = hrp::Vector3( 0*D2R,  30*D2R,  40*D2R);
    }
};

#include "../AutoBalancer/AutoBalancer.h"
class WBMSCore : UTIL_CONST {
  private:
    double CNT_F_TH, h2r_ratio, tgt_h2r_ratio, HZ, DT;
    struct timeval t_calc_start, t_calc_end;
    BiquadIIRFilterVec calcacc_v_filters, acc4zmp_v_filters, com_in_filter;
    hrp::Vector3 com_old, com_oldold, comacc, com_CP_ref_old, r_zmp_raw;
    hrp::Vector4 rf_safe_region,lf_safe_region;//前後左右の順
    std::vector<hrp::Vector2> rflf_points, hull_com, hull_dcp, hull_acp;
    std::vector<BiquadIIRFilterVec> tgt_pos_filters,tgt_rot_filters;
    std::vector<cv::Point2f> points,cvhull;
    RobotConfig rc;
    HumanPose rp_ref_out_old, hp_swap_checked;
    unsigned int loop;
    bool is_initial_loop;
  public:
    double H_cur;
    HumanPose hp_wld_raw, hp_plot, rp_ref_out, rp_ref_vel_old;
    FILE *sr_log, *cz_log, *id_log;
    WBMSPose3D baselinkpose;
//    WBMSPose3D ee_contact_pose[4];
    hrp::Vector3 com_vel_old,rh_vel_old, cp_dec, cp_acc;
    std::vector<hrp::Vector3> rf_vert,lf_vert;
    hrp::Vector2 com_forcp_ref,com_vel_forcp_ref;
    hrp::dvector6 invdyn_ft;
    struct WBMSparameters {
      double auto_swing_foot_landing_threshold;
      double foot_vertical_vel_limit_coeff;
      double human_com_height;
      bool is_doctor;
      bool set_com_height_fix;
      double set_com_height_fix_val;
      double swing_foot_height_offset;
      double swing_foot_max_height;
      double upper_body_rmc_ratio;
      bool use_rh,use_lh;
      bool use_head;
      bool use_manipulability_limit;
    };
    struct WBMSparameters WBMSparam;

    typedef boost::shared_ptr<SimpleFullbodyInverseKinematicsSolver> fikPtr;
    fikPtr fik;
    hrp::BodyPtr m_robot;
    double cur_manip_val[4][3];
    hrp::Vector3 manip_direc[4][3];
    hrp::Matrix33 manip_mat[4];
    hrp::Vector3 manip_sv[4];

    hrp::Vector2 cp_acc_old;

    WBMSCore(const double& dt){
      tgt_h2r_ratio = h2r_ratio = 0.96;//human 1.1m vs jaxon 1.06m
      DT = dt;
      HZ = (int)(1.0/DT);
      CNT_F_TH = 20.0;
      loop = 0;
      //////////  hrp::Vector は初期化必要  ///////////
      com_old = com_oldold = comacc = hrp::Vector3::Zero();
      r_zmp_raw = hrp::Vector3::Zero();
      com_vel_old = hrp::Vector3::Zero();
      rh_vel_old = hrp::Vector3::Zero();
      cp_dec = cp_acc = hrp::Vector3::Zero();
      invdyn_ft = hrp::dvector6::Zero();
      tgt_pos_filters.resize(num_pose_tgt);
      tgt_rot_filters.resize(num_pose_tgt);
      for(int i=0;i<tgt_pos_filters.size();i++)tgt_pos_filters[i].setParameter(1.0, HZ, Q_NOOVERSHOOT);//四肢拘束点用(position)
      for(int i=0;i<tgt_rot_filters.size();i++)tgt_rot_filters[i].setParameter(1.0, HZ, Q_NOOVERSHOOT);//四肢拘束点用(Rotation)
      tgt_pos_filters[com].setParameter(1.0, HZ, Q_NOOVERSHOOT);//重心pos用
      tgt_rot_filters[com].setParameter(0.6, HZ, Q_NOOVERSHOOT);//重心rot用
      tgt_pos_filters[rf].setParameter(hrp::Vector3(10.0,10.0,10.0), HZ, Q_NOOVERSHOOT);//右足pos用
      tgt_pos_filters[lf].setParameter(hrp::Vector3(1.0,1.0,1.0), HZ, Q_NOOVERSHOOT);//左足pos用
      calcacc_v_filters.setParameter(5, HZ, Q_BUTTERWORTH);//加速度計算用
      acc4zmp_v_filters.setParameter(5, HZ, Q_BUTTERWORTH);//ZMP生成用ほぼこの値でいい
      com_in_filter.setParameter(1, HZ);
      rf_safe_region = hrp::Vector4(0.02, -0.01,  0.02,  0.01);
      lf_safe_region = hrp::Vector4(0.02, -0.01, -0.01, -0.02);
      rf_vert.push_back(hrp::Vector3(0.13,-0.08,0));
      rf_vert.push_back(hrp::Vector3(-0.10,-0.08,0));
      rf_vert.push_back(hrp::Vector3(-0.10,0.06,0));
      rf_vert.push_back(hrp::Vector3(0.13,0.06,0));
      lf_vert.push_back(hrp::Vector3(0.13,-0.06,0));
      lf_vert.push_back(hrp::Vector3(-0.10,-0.06,0));
      lf_vert.push_back(hrp::Vector3(-0.10,0.08,0));
      lf_vert.push_back(hrp::Vector3(0.13,0.08,0));

      WBMSparam.auto_swing_foot_landing_threshold = 0.02;
      WBMSparam.foot_vertical_vel_limit_coeff = 4.0;
      WBMSparam.human_com_height = 1.00;
      WBMSparam.is_doctor = true;
      WBMSparam.set_com_height_fix = false;
      WBMSparam.set_com_height_fix_val = 0.02;
      WBMSparam.swing_foot_height_offset = 0.01;
      WBMSparam.swing_foot_max_height = 0.5;
//      WBMSparam.upper_body_rmc_ratio = 0.5;
      WBMSparam.upper_body_rmc_ratio = 0.0;
      WBMSparam.use_rh = WBMSparam.use_lh = true;
      WBMSparam.use_head = true;
      WBMSparam.use_manipulability_limit = true;
      rp_ref_out_old.clear();
      rp_ref_out.clear();
      rp_ref_vel_old.clear();
      rflf_points.reserve(8);//あらかじめreserveして高速化
      points.reserve(8);
      hull_com.reserve(6);
      hull_dcp.reserve(6);
      hull_dcp.reserve(6);
      cvhull.reserve(6);
      if(DEBUG){
        sr_log = fopen("/home/ishiguro/HumanSync_support_region.log","w+");
        cz_log = fopen("/home/ishiguro/HumanSync_com_zmp.log","w+");
        id_log = fopen("/home/ishiguro/HumanSync_invdyn.log","w+");
      }
      is_initial_loop = true;
      cout<<"WBMSCore constructed"<<endl;
    }
    ~WBMSCore(){
      if(DEBUG)fclose(sr_log);
      if(DEBUG)fclose(cz_log);
      if(DEBUG)fclose(id_log);
      cout<<"WBMSCore destructed"<<endl;
    }
    double getUpdateTime() const { return (double)(t_calc_end.tv_sec - t_calc_start.tv_sec) + (t_calc_end.tv_usec - t_calc_start.tv_usec)/1.0e6; }
    void initializeHumanPoseFromCurrentInput(){
      std::string ns[7] = {"com","rf","lf","rh","lh","zmp","head"};
      for(int i=0;i<7;i++){
        hp_wld_raw.tgt[i].offs.p   =  hp_wld_raw.tgt[i].abs.p;
        hp_wld_raw.tgt[i].offs.rpy =  hp_wld_raw.tgt[i].abs.rpy;
      }
    }
    void initializeRobotPoseFromHRPBody(fikPtr& fik_in, hrp::BodyPtr& robot_in){
      const std::string robot_l_names[4] = {"rleg","lleg","rarm","larm"};
      const int human_l_names[4] = {rf,lf,rh,lh};
      for(int i=0;i<4;i++){//HumanSynchronizerの初期姿勢オフセットをセット
        if(fik_in->ikp.count(robot_l_names[i])){
          rp_ref_out.tgt[human_l_names[i]].offs.p = fik_in->getEEPos(robot_l_names[i]);
          rp_ref_out.tgt[human_l_names[i]].offs.rpy = hrp::rpyFromRot(fik_in->getEERot(robot_l_names[i]));
          rp_ref_out.tgt[human_l_names[i]].abs = rp_ref_out.tgt[human_l_names[i]].offs;
        }
      }
      rp_ref_out.tgt[com].offs.p = robot_in->calcCM();
      rp_ref_out.tgt[com].offs.rpy = hrp::rpyFromRot(robot_in->rootLink()->R);
      rp_ref_out.tgt[zmp].offs.p(X) = rp_ref_out.tgt[com].offs.p(X);
      rp_ref_out.tgt[zmp].offs.p(Y) = rp_ref_out.tgt[com].offs.p(Y);
      rp_ref_out.tgt[zmp].offs.p(Z) = (fik_in->getEEPos("rleg")(Z)+fik_in->getEEPos("lleg")(Z))/2;
      rp_ref_out.tgt[rf].cnt = rp_ref_out.tgt[rf].offs;
      rp_ref_out.tgt[lf].cnt = rp_ref_out.tgt[lf].offs;
      baselinkpose.p = robot_in->rootLink()->p;
      baselinkpose.rpy = hrp::rpyFromRot(robot_in->rootLink()->R);
      for(int i=0, l[LR]={rf,lf}; i<LR; i++){
        rp_ref_out.tgt[l[i]].is_contact = true;
//        rp_ref_out.tgt[l[i]].cnt = rp_ref_out.tgt[l[i]].abs;
      }
    }
    void initializeRequest(fikPtr& fik_in, hrp::BodyPtr& robot_in){
      loop = 0;
      is_initial_loop = true;
      initializeHumanPoseFromCurrentInput();
      initializeRobotPoseFromHRPBody(fik_in, robot_in);
      rp_ref_out_old = rp_ref_vel_old = rp_ref_out;
    }
    void update(){//////////  メインループ  ////////////
      gettimeofday(&t_calc_start, NULL);
//      if(is_initial_loop){ initialize(); }
      updateParams                        ();
      autoLRSwapCheck                     (hp_wld_raw,hp_swap_checked);//入力の左右反転を常にチェック(＝手足の交差は不可能)
      convertRelHumanPoseToRelRobotPose   (hp_swap_checked, rp_ref_out);
      hp_plot = rp_ref_out;
//      applyVelLimit                       (rp_ref_vel_old, rp_ref_out); rp_ref_vel_old = rp_ref_out;
      if(WBMSparam.set_com_height_fix)rp_ref_out.tgt[com].abs.p(Z) = rp_ref_out.tgt[com].offs.p(Z) + WBMSparam.set_com_height_fix_val;//膝曲げトルクで落ちるときの応急措置
      judgeFootLandOnCommandByFootForce   (hp_wld_raw);//人体足裏反力から各足の接地指令を生成
      lockSwingFootIfZMPOutOfSupportFoot  (rp_ref_out_old, rp_ref_out);//
      limitEEWorkspace                    (rp_ref_out);
//      limitManipulability                 (rp_ref_out);
      setFootContactPoseByGoContact       (rp_ref_out);
      if(DEBUG){ fprintf(cz_log,"com_in: %f %f ",rp_ref_out.tgt[com].abs.p(X),rp_ref_out.tgt[com].abs.p(Y)); }
      applyCOMToSupportRegionLimit        (rp_ref_out.tgt[rf].abs.p, rp_ref_out.tgt[lf].abs.p, rp_ref_out.tgt[com].abs.p);
      setFootRotHorizontalIfGoLanding     (rp_ref_out);

      if(WBMSparam.use_manipulability_limit){limitManipulability                 (rp_ref_out);}

      applyCOMToSupportRegionLimit        (rp_ref_out.tgt[rf].abs.p, rp_ref_out.tgt[lf].abs.p, com_CP_ref_old);//これやらないと支持領域の移動によって1ステップ前のCOM位置はもうはみ出てるかもしれないから

      Vector3ToVector2(rp_ref_out.tgt[com].abs.p,com_forcp_ref);
      static hrp::Vector2 com_forcp_ref_old;
      com_vel_forcp_ref = (com_forcp_ref - com_forcp_ref_old)/DT;
      com_forcp_ref_old = com_forcp_ref;


      applyLPFilter_pre                       (rp_ref_out);
      applyCOMStateLimitByCapturePoint    (rp_ref_out.tgt[com].abs.p, rp_ref_out.tgt[rf].abs.p, rp_ref_out.tgt[lf].abs.p, com_CP_ref_old, rp_ref_out.tgt[com].abs.p);
//      dbg(rp_ref_out.tgt[rf].go_contact);
//      dbg(rp_ref_out.tgt[lf].go_contact);
      applyCOMToSupportRegionLimit        (rp_ref_out.tgt[rf].abs.p, rp_ref_out.tgt[lf].abs.p, rp_ref_out.tgt[com].abs.p);
      applyLPFilter_post                       (rp_ref_out);

      r_zmp_raw = rp_ref_out.tgt[zmp].abs.p;
      applyZMPCalcFromCOM                 (rp_ref_out.tgt[com].abs.p, rp_ref_out.tgt[zmp].abs.p);
      if(DEBUG){
        fprintf(cz_log,"com_ans_zmp: %f %f ",rp_ref_out.tgt[zmp].abs.p(X),rp_ref_out.tgt[zmp].abs.p(Y));
        fprintf(cz_log,"\n");
      }
      overwriteFootZFromFootLandOnCommand (rp_ref_out);
      modifyFootRotAndXYForContact        (rp_ref_out);

      H_cur = rp_ref_out.tgt[com].abs.p(Z) - std::min((double)rp_ref_out.tgt[rf].abs.p(Z), (double)rp_ref_out.tgt[rf].abs.p(Z));
      com_vel_old = (rp_ref_out.tgt[com].abs.p - rp_ref_out_old.tgt[com].abs.p)/DT;
      rh_vel_old = (rp_ref_out.tgt[rh].abs.p - rp_ref_out_old.tgt[rh].abs.p)/DT;
      rp_ref_out_old = rp_ref_out;
      loop++;
      is_initial_loop = false;
      gettimeofday(&t_calc_end, NULL);
    }
    static void Vector3ToVector2(const hrp::Vector3& in, hrp::Vector2& out){ out(X) = in(X); out(Y) = in(Y);}
    static void Vector2ToVector3(const hrp::Vector2& in, hrp::Vector3& out){ out(X) = in(X); out(Y) = in(Y);}
    static void Point3DToVector3(const RTC::Point3D& in, hrp::Vector3& out){ out(X) = in.x; out(Y) = in.y; out(Z) = in.z;}
    static void Oriantation3DToVector3(const RTC::Orientation3D& in, hrp::Vector3& out){ out(X) = in.r; out(Y) = in.p; out(Z) = in.y;}
    static void Vector3ToPoint3D(const hrp::Vector3& in, RTC::Point3D& out){ out.x = in(X); out.y = in(Y); out.z = in(Z);}
    static void Vector3ToOriantation3D(const hrp::Vector3& in, RTC::Orientation3D& out){ out.r = in(X); out.p = in(Y); out.y = in(Z);}
    static void Pose3DToWBMSPose3D(const RTC::Pose3D& in, WBMSPose3D& out){ Point3DToVector3(in.position,out.p); Oriantation3DToVector3(in.orientation,out.rpy); }
    static void WBMSPose3DToPose3D(const WBMSPose3D& in, RTC::Pose3D& out){ Vector3ToPoint3D(in.p,out.position); Vector3ToOriantation3D(in.rpy,out.orientation); }
    static void DoubleSeqToVector6(const RTC::TimedDoubleSeq::_data_seq& in, hrp::dvector6& out){
      if(in.length() == 6){out(fx)=in[0]; out(fy)=in[1]; out(fz)=in[2]; out(tz)=in[3]; out(ty)=in[4]; out(tz)=in[5];
      }else{ std::cerr<<"[WARN] HumanPose::hrp::dvector6 DoubleSeqTohrp::dvector6() invalid data length"<<std::endl; out = hrp::dvector6::Zero(); }
    }
    static void Vector6ToDoubleSeq(const hrp::dvector6& in, RTC::TimedDoubleSeq::_data_seq& out){
      if(out.length() == 6){out[0]=in(fx); out[1]=in(fy); out[2]=in(fz); out[3]=in(tx); out[4]=in(ty); out[5]=in(tz);
      }else{ std::cerr<<"[WARN] HumanPose::hrp::dvector6 DoubleSeqTohrp::dvector6() invalid data length"<<std::endl; }
    }
    static double hrpVector2Cross(const hrp::Vector2& a, const hrp::Vector2& b){ return a(X)*b(Y)-a(Y)*b(X); }

  private:
    void updateParams(){
      updateHumanToRobotRatio(tgt_h2r_ratio);
    }
    void updateHumanToRobotRatio(const double h2r_r_goal){//h2r_ratioに伴って変わる変数の処理もここに書く
      const double step = 0.001;
      if(h2r_r_goal - h2r_ratio > step){h2r_ratio += step; cout<<"Interpolating HumanToRobotRatio as "<<h2r_ratio<<endl;}
      else if(h2r_r_goal - h2r_ratio < (-1)*step){h2r_ratio -= step; cout<<"Interpolating HumanToRobotRatio as "<<h2r_ratio<<endl;}
      else{h2r_ratio = h2r_r_goal;}
    }
    void autoLRSwapCheck(const HumanPose& in, HumanPose& out){
      out = in;
      if(in.tgt[rh].offs.p(Y) > in.tgt[lh].offs.p(Y) ){ out.tgt[rh] = in.tgt[lh]; out.tgt[lh] = in.tgt[rh]; }
      if(in.tgt[rf].offs.p(Y) > in.tgt[lf].offs.p(Y) ){ out.tgt[rf] = in.tgt[lf]; out.tgt[lf] = in.tgt[rf]; }
    }
    void convertRelHumanPoseToRelRobotPose(const HumanPose& in, HumanPose& out){//結局初期指定からの移動量(=Rel)で計算をしてゆく
//      out = in;//ダメゼッタイ
      for(int i=0, l[5]={com,rf,lf,rh,lh}; i<5; i++){
        out.tgt[l[i]].abs.p   =  h2r_ratio * (in.tgt[l[i]].abs.p - in.tgt[l[i]].offs.p)   + out.tgt[l[i]].offs.p;
        out.tgt[l[i]].abs.rpy =  hrp::rpyFromRot( hrp::rotFromRpy(in.tgt[l[i]].abs.rpy) * hrp::rotFromRpy(in.tgt[l[i]].offs.rpy).transpose() * hrp::rotFromRpy(out.tgt[l[i]].offs.rpy) );
      }
      for(int i=0, l[4]={rf,lf,rh,lh}; i<4; i++){
        out.tgt[l[i]].w = in.tgt[l[i]].w;
        out.tgt[l[i]].go_contact = in.tgt[l[i]].go_contact;
      }
      out.tgt[head].abs = in.tgt[head].abs;
      out.tgt[zmp].abs = in.tgt[zmp].abs;//最近使わない
    }
    void judgeFootLandOnCommandByFootForce(HumanPose& in){
      for(int i=0, l[LR]={rf,lf}; i<LR; i++){
        if      (in.tgt[l[i]].is_contact  && in.tgt[l[i]].w(fz)<CNT_F_TH   ){in.tgt[l[i]].is_contact = false;}//足ついた状態から上げる
        else if (!in.tgt[l[i]].is_contact && in.tgt[l[i]].w(fz)>CNT_F_TH+30){in.tgt[l[i]].is_contact = true;}//足浮いた状態から下げる
      }
    }
    void applyLPFilter(HumanPose& tgt){
      if(is_initial_loop){
        for(int i=0, l[1]={com}; i<1; i++){
          tgt_pos_filters[l[i]].reset(tgt.tgt[l[i]].abs.p);
          tgt_rot_filters[l[i]].reset(tgt.tgt[l[i]].abs.rpy);
        }
      }
      for(int i=0, l[1]={com}; i<1; i++){
        tgt.tgt[l[i]].abs.p   = tgt_pos_filters[l[i]].passFilter(tgt.tgt[l[i]].abs.p);
        tgt.tgt[l[i]].abs.rpy = tgt_rot_filters[l[i]].passFilter(tgt.tgt[l[i]].abs.rpy);
      }
//      cout<<"calcVelAccSafeTrajectoryVecML"<<endl;
      for(int i=0, l[4]={rf,lf,rh,lh}; i<4; i++){
        calcVelAccSafeTrajectoryVecML(rp_ref_out_old.tgt[l[i]].abs.p, (rp_ref_out.tgt[l[i]].abs.p - rp_ref_out_old.tgt[l[i]].abs.p)/DT, tgt.tgt[l[i]].abs.p, manip_mat[i], manip_sv[i], tgt.tgt[l[i]].abs.p);
        calcVelAccSafeTrajectoryVec(rp_ref_out_old.tgt[l[i]].abs.rpy, (rp_ref_out.tgt[l[i]].abs.rpy - rp_ref_out_old.tgt[l[i]].abs.rpy)/DT, tgt.tgt[l[i]].abs.rpy, 1.0, 1.0, tgt.tgt[l[i]].abs.rpy);
      }
      for(int i=0, l[1]={head}; i<1; i++){
        calcVelAccSafeTrajectoryVec(rp_ref_out_old.tgt[l[i]].abs.p, (rp_ref_out.tgt[l[i]].abs.p - rp_ref_out_old.tgt[l[i]].abs.p)/DT, tgt.tgt[l[i]].abs.p, 1.0, 1.0, tgt.tgt[l[i]].abs.p);
        calcVelAccSafeTrajectoryVec(rp_ref_out_old.tgt[l[i]].abs.rpy, (rp_ref_out.tgt[l[i]].abs.rpy - rp_ref_out_old.tgt[l[i]].abs.rpy)/DT, tgt.tgt[l[i]].abs.rpy, 1.0, 1.0, tgt.tgt[l[i]].abs.rpy);
      }
    }
    void applyLPFilter_pre(HumanPose& tgt){
//      if(is_initial_loop){
//        for(int i=0, l[1]={com}; i<1; i++){
//          tgt_pos_filters[l[i]].reset(tgt.tgt[l[i]].abs.p);
//          tgt_rot_filters[l[i]].reset(tgt.tgt[l[i]].abs.rpy);
//        }
//      }
//      for(int i=0, l[1]={com}; i<1; i++){
//        tgt.tgt[l[i]].abs.p   = tgt_pos_filters[l[i]].passFilter(tgt.tgt[l[i]].abs.p);
//        tgt.tgt[l[i]].abs.rpy = tgt_rot_filters[l[i]].passFilter(tgt.tgt[l[i]].abs.rpy);
//      }
      for(int i=0, l[4]={rf,lf,rh,lh}; i<4; i++){
        calcVelAccSafeTrajectoryVecML(rp_ref_out_old.tgt[l[i]].abs.p, (rp_ref_out.tgt[l[i]].abs.p - rp_ref_out_old.tgt[l[i]].abs.p)/DT, tgt.tgt[l[i]].abs.p, manip_mat[i], manip_sv[i], tgt.tgt[l[i]].abs.p);
        calcVelAccSafeTrajectoryVec(rp_ref_out_old.tgt[l[i]].abs.rpy, (rp_ref_out.tgt[l[i]].abs.rpy - rp_ref_out_old.tgt[l[i]].abs.rpy)/DT, tgt.tgt[l[i]].abs.rpy, 1.0, 1.0, tgt.tgt[l[i]].abs.rpy);
      }
      for(int i=0, l[1]={head}; i<1; i++){
        calcVelAccSafeTrajectoryVec(rp_ref_out_old.tgt[l[i]].abs.p, (rp_ref_out.tgt[l[i]].abs.p - rp_ref_out_old.tgt[l[i]].abs.p)/DT, tgt.tgt[l[i]].abs.p, 1.0, 1.0, tgt.tgt[l[i]].abs.p);
        calcVelAccSafeTrajectoryVec(rp_ref_out_old.tgt[l[i]].abs.rpy, (rp_ref_out.tgt[l[i]].abs.rpy - rp_ref_out_old.tgt[l[i]].abs.rpy)/DT, tgt.tgt[l[i]].abs.rpy, 1.0, 1.0, tgt.tgt[l[i]].abs.rpy);
      }
    }
    void applyLPFilter_post(HumanPose& tgt){
      if(is_initial_loop){
        for(int i=0, l[1]={com}; i<1; i++){
          tgt_pos_filters[l[i]].reset(tgt.tgt[l[i]].abs.p);
          tgt_rot_filters[l[i]].reset(tgt.tgt[l[i]].abs.rpy);
        }
      }
      for(int i=0, l[1]={com}; i<1; i++){
        tgt.tgt[l[i]].abs.p   = tgt_pos_filters[l[i]].passFilter(tgt.tgt[l[i]].abs.p);
        tgt.tgt[l[i]].abs.rpy = tgt_rot_filters[l[i]].passFilter(tgt.tgt[l[i]].abs.rpy);
      }
    }
    void calcVelAccSafeTrajectoryVec(const hrp::Vector3& pos_cur, const hrp::Vector3& vel_cur, const hrp::Vector3& pos_tgt, const double& max_acc, const double& max_vel, hrp::Vector3& pos_ans){
      if((pos_tgt - pos_cur).norm() > 1.0e-6){
        hrp::Vector3 direc = (pos_tgt - pos_cur).normalized();
        double stop_safe_vel_scalar = sqrt( 2 * max_acc * (pos_tgt - pos_cur).norm() );
        hrp::Vector3 vel_ans = vel_cur + direc * max_acc * DT;
        if(vel_ans.dot(direc) > stop_safe_vel_scalar){
//          vel_ans = vel_ans.normalized() * stop_safe_vel_scalar;
          vel_ans = vel_ans * stop_safe_vel_scalar / vel_ans.dot(direc);
        }
        pos_ans = pos_cur + vel_ans * DT;
      }else{
        pos_ans = pos_tgt;
      }
    }
    void calcVelAccSafeTrajectoryVecML(const hrp::Vector3& pos_cur, const hrp::Vector3& vel_cur, const hrp::Vector3& pos_tgt, const hrp::Matrix33& max_acc_mat, const hrp::Vector3& max_acc_sv, hrp::Vector3& pos_ans){
      if((pos_tgt - pos_cur).norm() > 1.0e-6){
        hrp::Vector3 direc = (pos_tgt - pos_cur).normalized();
        double acc_val = 1.0;
        hrp::Vector3 ref_acc = direc*acc_val;
        hrp::Vector3 mod_acc = max_acc_mat * max_acc_sv.asDiagonal() * max_acc_mat.transpose() * ref_acc;
//        cout<<"ref_acc "<<ref_acc.transpose()<<" mod_acc "<<mod_acc.transpose()<<endl;
//        dbg(max_acc_mat);
//        dbg(max_acc_sv.transpose());
//        dbg((max_acc_mat.transpose() * ref_acc).transpose());
//        dbg((max_acc_sv.asDiagonal() * max_acc_mat.transpose() * ref_acc).transpose());
        double stop_safe_vel_scalar = sqrt( 2 * fabs(mod_acc.dot(direc)) * (pos_tgt - pos_cur).norm() );
        hrp::Vector3 vel_ans = vel_cur + mod_acc * DT;
        if(vel_ans.dot(direc) > stop_safe_vel_scalar){
//          vel_ans = vel_ans.normalized() * stop_safe_vel_scalar;
          vel_ans = vel_ans * stop_safe_vel_scalar / vel_ans.dot(direc);
        }
        pos_ans = pos_cur + vel_ans * DT;
      }else{
        pos_ans = pos_tgt;
      }
    }
    void calcVelAccSafeTrajectory(const hrp::Vector3& pos_cur, const hrp::Vector3& vel_cur, const hrp::Vector3& pos_tgt, const double& max_acc, const double& max_vel, hrp::Vector3& pos_ans){
      hrp::Vector3 direc( SGN(pos_tgt(X)-pos_cur(X)), SGN(pos_tgt(Y)-pos_cur(Y)), SGN(pos_tgt(Z)-pos_cur(Z)));
      hrp::Vector3 vel_ans = vel_cur + direc * max_acc * DT;
      for(int i=0;i<XYZ;i++){
        double stop_safe_vel = sqrt( 2 * max_acc * fabs(pos_tgt(i) - pos_cur(i)) );
        if( direc(i) > 0 ){
          if(vel_ans(i) > stop_safe_vel){ vel_ans(i) = stop_safe_vel; }
        }else{
          if(vel_ans(i) < - stop_safe_vel){ vel_ans(i) = - stop_safe_vel; }
        }
      }
      pos_ans = pos_cur + vel_ans * DT;
    }
    void lockSwingFootIfZMPOutOfSupportFoot(const HumanPose& old, HumanPose& out){
      hrp::Vector2 inside_vec_rf(0,1),inside_vec_lf(0,-1);
      hrp::Vector2 rf2zmp = hrp::Vector2(old.tgt[zmp].abs.p(X),old.tgt[zmp].abs.p(Y)) - hrp::Vector2(old.tgt[rf].abs.p(X),old.tgt[rf].abs.p(Y));
      hrp::Vector2 lf2zmp = hrp::Vector2(old.tgt[zmp].abs.p(X),old.tgt[zmp].abs.p(Y)) - hrp::Vector2(old.tgt[lf].abs.p(X),old.tgt[lf].abs.p(Y));
      if( inside_vec_rf.dot(rf2zmp) / inside_vec_rf.norm() > WBMSparam.auto_swing_foot_landing_threshold ){ out.tgt[lf].go_contact = true; }
      if( inside_vec_lf.dot(lf2zmp) / inside_vec_lf.norm() > WBMSparam.auto_swing_foot_landing_threshold ){ out.tgt[rf].go_contact = true; }
    }
    void limitEEWorkspace(HumanPose& out){
      const double MAX_FW = 0.25;
//      const double MAX_FW = 1000000;//manipulability test
      const double FOOT_2_FOOT_COLLISION_MARGIIN = 0.16;
//      if(!out.tgt[rf].is_contact && out.tgt[lf].is_contact){//右足浮遊時
//        const hrp::Vector2 lf2rf_vec( out.tgt[rf].abs.p(X) - out.tgt[lf].cnt.p(X), out.tgt[rf].abs.p(Y) - out.tgt[lf].cnt.p(Y) );
//        if(lf2rf_vec.norm() > MAX_FW){
//          out.tgt[rf].abs.p(X) = (lf2rf_vec.normalized())(X) * MAX_FW + out.tgt[lf].cnt.p(X);
//          out.tgt[rf].abs.p(Y) = (lf2rf_vec.normalized())(Y) * MAX_FW + out.tgt[lf].cnt.p(Y);
//        }
//        LIMIT_MAX(out.tgt[rf].abs.p(Y), out.tgt[lf].cnt.p(Y) - FOOT_2_FOOT_COLLISION_MARGIIN);
//      }
//      else if(out.tgt[rf].is_contact && !out.tgt[lf].is_contact){//左足浮遊時
//        const hrp::Vector2 rf2lf_vec( out.tgt[lf].abs.p(X) - out.tgt[rf].cnt.p(X), out.tgt[lf].abs.p(Y) - out.tgt[rf].cnt.p(Y) );
//        if(rf2lf_vec.norm() > MAX_FW){
//          out.tgt[lf].abs.p(X) = (rf2lf_vec.normalized())(X) * MAX_FW + out.tgt[rf].cnt.p(X);
//          out.tgt[lf].abs.p(Y) = (rf2lf_vec.normalized())(Y) * MAX_FW + out.tgt[rf].cnt.p(Y);
//        }
//        LIMIT_MIN(out.tgt[lf].abs.p(Y), out.tgt[rf].cnt.p(Y) + FOOT_2_FOOT_COLLISION_MARGIIN);
//      }


      for(int i=0, spl[LR]={rf,lf}, swl[LR]={lf,rf}; i<LR; i++){
        PoseTGT& support_leg = out.tgt[spl[i]];
        PoseTGT& swing_leg = out.tgt[swl[i]];
        hrp::Vector2 inside_vec_baserel = ( spl[i]==rf ? hrp::Vector2(0,+1) : hrp::Vector2(0,-1) );

        hrp::Vector2 sp2sw_vec( swing_leg.abs.p(X) - support_leg.cnt.p(X), swing_leg.abs.p(Y) - support_leg.cnt.p(Y) );
        if(swing_leg.go_contact && sp2sw_vec.norm() > MAX_FW){ sp2sw_vec = sp2sw_vec.normalized() * MAX_FW; }//着地時に足を広げすぎないよう制限
        Eigen::Matrix2d base_rot;
        base_rot = Eigen::Rotation2Dd(baselinkpose.rpy(y));
        hrp::Vector2 sp2sw_vec_baserel = base_rot.transpose() * sp2sw_vec;
        if( sp2sw_vec_baserel.dot(inside_vec_baserel) < FOOT_2_FOOT_COLLISION_MARGIIN){
          sp2sw_vec_baserel += inside_vec_baserel * (FOOT_2_FOOT_COLLISION_MARGIIN - sp2sw_vec_baserel.dot(inside_vec_baserel));
        }
        sp2sw_vec = base_rot * sp2sw_vec_baserel;
        swing_leg.abs.p(X) = sp2sw_vec(X) + support_leg.cnt.p(X);
        swing_leg.abs.p(Y) = sp2sw_vec(Y) + support_leg.cnt.p(Y);
      }



//      if(!out.tgt[rf].is_contact && out.tgt[lf].is_contact){//右足浮遊時
//        PoseTGT& support_leg = out.tgt[lf];
//        PoseTGT& swing_leg = out.tgt[rf];
//        hrp::Vector2 inside_vec_baserel(0, -1);
//
//        hrp::Vector2 sp2sw_vec( swing_leg.abs.p(X) - support_leg.cnt.p(X), swing_leg.abs.p(Y) - support_leg.cnt.p(Y) );
//        if(swing_leg.go_contact && sp2sw_vec.norm() > MAX_FW){ sp2sw_vec = sp2sw_vec.normalized() * MAX_FW; }//着地時に足を広げすぎないよう制限
//        Eigen::Matrix2d base_rot;
//        base_rot = Eigen::Rotation2Dd(baselinkpose.rpy(y));
//        hrp::Vector2 sp2sw_vec_baserel = base_rot.transpose() * sp2sw_vec;
//
//        if( sp2sw_vec_baserel.dot(inside_vec_baserel) < FOOT_2_FOOT_COLLISION_MARGIIN){
//          sp2sw_vec_baserel += inside_vec_baserel * (FOOT_2_FOOT_COLLISION_MARGIIN - sp2sw_vec_baserel.dot(inside_vec_baserel));
//        }
//        sp2sw_vec = base_rot * sp2sw_vec_baserel;
//        swing_leg.abs.p(X) = sp2sw_vec(X) + support_leg.cnt.p(X);
//        swing_leg.abs.p(Y) = sp2sw_vec(Y) + support_leg.cnt.p(Y);
//      }
//      else if(!out.tgt[lf].is_contact && out.tgt[lf].go_contact && out.tgt[rf].is_contact){//左足浮遊時
//        const hrp::Vector2 rf2lf_vec( out.tgt[lf].abs.p(X) - out.tgt[rf].cnt.p(X), out.tgt[lf].abs.p(Y) - out.tgt[rf].cnt.p(Y) );
//        if(rf2lf_vec.norm() > MAX_FW){
//          out.tgt[lf].abs.p(X) = (rf2lf_vec.normalized())(X) * MAX_FW + out.tgt[rf].cnt.p(X);
//          out.tgt[lf].abs.p(Y) = (rf2lf_vec.normalized())(Y) * MAX_FW + out.tgt[rf].cnt.p(Y);
//        }
//        LIMIT_MIN(out.tgt[lf].abs.p(Y), out.tgt[rf].cnt.p(Y) + FOOT_2_FOOT_COLLISION_MARGIIN);
//      }


//      hrp::Vector3 init_base2rh = out.tgt[rh].offs.p - baselinkpose.p_offs;
//      hrp::Vector3 init_base2lh = out.tgt[lh].offs.p - baselinkpose.p_offs;
//      hrp::Matrix33 init_base2rh_R = hrp::rotFromRpy(baselinkpose.rpy_offs).transpose() * hrp::rotFromRpy(out.tgt[rh].offs.rpy);
//      hrp::Matrix33 init_base2lh_R = hrp::rotFromRpy(baselinkpose.rpy_offs).transpose() * hrp::rotFromRpy(out.tgt[lh].offs.rpy);
//      hrp::Vector3 hand_ulimit = hrp::Vector3(0.1,0.1,0.1);
//      hrp::Vector3 hand_llimit = hrp::Vector3(-0.1,-0.1,-0.1);
//      if(!WBMSparam.use_rh){
//        out.tgt[rh].abs.rpy = hrp::rpyFromRot( hrp::rotFromRpy(baselinkpose.rpy) * init_base2rh_R);
//        out.tgt[rh].abs.p = baselinkpose.p + hrp::rotFromRpy(out.tgt[rh].abs.rpy) * init_base2rh;
//      }
//      if(!WBMSparam.use_lh){
//        out.tgt[lh].abs.rpy = hrp::rpyFromRot( hrp::rotFromRpy(baselinkpose.rpy) * init_base2lh_R);
//        out.tgt[lh].abs.p = baselinkpose.p + hrp::rotFromRpy(out.tgt[lh].abs.rpy) * init_base2lh;
//      }
//      for(int i=0;i<3;i++){
//        LIMIT_MINMAX( out.tgt[rh].abs.p(i), (baselinkpose.p+init_base2rh+hand_llimit)(i), (baselinkpose.p+init_base2rh+hand_ulimit)(i) );
//        LIMIT_MINMAX( out.tgt[lh].abs.p(i), (baselinkpose.p+init_base2lh+hand_llimit)(i), (baselinkpose.p+init_base2lh+hand_ulimit)(i) );
//      }
      LIMIT_MINMAX( out.tgt[rf].abs.p(Z), out.tgt[rf].offs.p(Z), out.tgt[rf].offs.p(Z)+WBMSparam.swing_foot_max_height);
      LIMIT_MINMAX( out.tgt[lf].abs.p(Z), out.tgt[lf].offs.p(Z), out.tgt[lf].offs.p(Z)+WBMSparam.swing_foot_max_height);

      for(int i=0, l[2]={rh,lh}; i<2; i++){
        LIMIT_MIN(out.tgt[l[i]].abs.p(X), baselinkpose.p(X));
        LIMIT_MAX(out.tgt[l[i]].abs.p(Z), baselinkpose.p(Z) + 0.4);
        hrp::Vector2 horizontal_dist(out.tgt[l[i]].abs.p(X) - baselinkpose.p(X), out.tgt[l[i]].abs.p(Y) - baselinkpose.p(Y));
        if(horizontal_dist.norm() < 0.5){
          horizontal_dist = 0.5 * horizontal_dist.normalized();
        }
        out.tgt[l[i]].abs.p(X) = baselinkpose.p(X) + horizontal_dist(X);
        out.tgt[l[i]].abs.p(Y) = baselinkpose.p(Y) + horizontal_dist(Y);
      }

      for(int i=0;i<XYZ;i++){ LIMIT_MINMAX( out.tgt[com].abs.rpy(i), rc.ee_rot_limit[com][MIN](i), rc.ee_rot_limit[com][MAX](i) ); }
      LIMIT_MINMAX( out.tgt[com].abs.p(Z), out.tgt[com].offs.p(Z) - 0.15, out.tgt[com].offs.p(Z) + 0.03 );//COM高さ方向の制限
      for(int i=0, l[5]={rf,lf,rh,lh,head}; i<5; i++){
        for(int j=0;j<XYZ;j++){ LIMIT_MINMAX( out.tgt[l[i]].abs.rpy(j), rc.ee_rot_limit[l[i]][MIN](j) + out.tgt[com].abs.rpy(j), rc.ee_rot_limit[l[i]][MAX](j) + out.tgt[com].abs.rpy(j) ); }
      }
      if(!WBMSparam.use_head)out.tgt[head].abs.rpy = hrp::Vector3::Zero();
//      if(mode!=MODE_WBMS)out.tgt[head].abs.rpy = hrp::Vector3::Zero();//頭は動かしてない時は0
    }
    void limitManipulability(HumanPose& out){
      const std::string robot_l_names[4] = {"rleg","lleg","rarm","larm"};
      const int human_l_names[4] = {rf,lf,rh,lh};
      for(int i=0;i<4;i++){
        if(fik->ikp.count(robot_l_names[i])){
          fik->ikp[robot_l_names[i]].target_r0 = hrp::rotFromRpy(out.tgt[human_l_names[i]].abs.rpy);
          fik->ikp[robot_l_names[i]].target_p0 = out.tgt[human_l_names[i]].abs.p;
        }
      }
      for ( std::map<std::string, SimpleFullbodyInverseKinematicsSolver::IKparam>::iterator it = fik->ikp.begin(); it != fik->ikp.end(); it++ ) {
          if (it->second.is_ik_enable) fik->solveLimbIK (it->second, it->first, fik->ratio_for_vel, false);
      }
      for(int i=0;i<4;i++){
        if(fik->ikp.count(robot_l_names[i])){
          out.tgt[human_l_names[i]].abs.p = fik->ikp[robot_l_names[i]].target_link->p + fik->ikp[robot_l_names[i]].target_link->R * fik->ikp[robot_l_names[i]].localPos;
        }
      }
    }
    void setFootContactPoseByGoContact(HumanPose& out){
      for(int i=0, l[LR]={rf,lf}; i<LR; i++){
        if(out.tgt[l[i]].go_contact){
          out.tgt[l[i]].abs.p(X) = out.tgt[l[i]].cnt.p(X);
          out.tgt[l[i]].abs.p(Y) = out.tgt[l[i]].cnt.p(Y);
          out.tgt[l[i]].abs.rpy(y) = out.tgt[l[i]].cnt.rpy(y);
        }else{
//          out.tgt[l[i]].cnt.p(X) = out.tgt[l[i]].abs.p(X);
//          out.tgt[l[i]].cnt.p(Y) = out.tgt[l[i]].abs.p(Y);
//          out.tgt[l[i]].cnt.rpy(y) = out.tgt[l[i]].abs.rpy(y);
          out.tgt[l[i]].cnt.p(X) = rp_ref_out_old.tgt[l[i]].abs.p(X);
          out.tgt[l[i]].cnt.p(Y) = rp_ref_out_old.tgt[l[i]].abs.p(Y);
          out.tgt[l[i]].cnt.rpy(y) = rp_ref_out_old.tgt[l[i]].abs.rpy(y);
        }
        if(out.tgt[l[i]].go_contact){
          out.tgt[l[i]].abs.p(Z) = out.tgt[l[i]].offs.p(Z);
        }else{
          LIMIT_MIN( out.tgt[l[i]].abs.p(Z), out.tgt[l[i]].offs.p(Z)+WBMSparam.swing_foot_height_offset);
        }
      }
    }
    void setFootRotHorizontalIfGoLanding(HumanPose& out){
      if( out.tgt[rf].go_contact){ out.tgt[rf].abs.rpy(r) = out.tgt[rf].abs.rpy(p) = 0; }
      if( out.tgt[lf].go_contact){ out.tgt[lf].abs.rpy(r) = out.tgt[lf].abs.rpy(p) = 0; }
    }
    void overwriteFootZFromFootLandOnCommand(HumanPose& out){
      const double com2rf_dist = fabs((double)out.tgt[rf].abs.p(Y) - (double)out.tgt[com].abs.p(Y));
      const double com2lf_dist = fabs((double)out.tgt[lf].abs.p(Y) - (double)out.tgt[com].abs.p(Y));
      limitGroundContactVelocity(out.tgt[rf].go_contact, com2rf_dist, rp_ref_out_old.tgt[rf], out.tgt[rf], out.tgt[rf].is_contact);
      limitGroundContactVelocity(out.tgt[lf].go_contact, com2lf_dist, rp_ref_out_old.tgt[lf], out.tgt[lf], out.tgt[lf].is_contact);
    }
    void limitGroundContactVelocity(const bool& go_land_in, const double& com2foot_dist, const PoseTGT& f_old_in, PoseTGT& f_in_out, bool& is_f_contact_out){
      double penalty = 1.0 + (0.2 - com2foot_dist) * 5.0;//1.0 ~ 2.0 本当はCPに比例すべき？
      LIMIT_MINMAX( penalty, 1.0, 2.0);
      LIMIT_MIN( f_in_out.abs.p(Z), f_in_out.offs.p(Z));
      double input_vel = (f_in_out.abs.p(Z) - f_old_in.abs.p(Z)) / DT;
      const double vel_limit_k = WBMSparam.foot_vertical_vel_limit_coeff * penalty;//地面から0.02[m]地点で0.02*8=0.16[m/s]出ている計算
      double limit_vel = - vel_limit_k * (f_old_in.abs.p(Z) - f_old_in.offs.p(Z));//sinの時は平均0.05~0.10[m/s]
      const double max_vel_threshold = -0.01;
      LIMIT_MAX( limit_vel, max_vel_threshold);//速度0に近づくと目標になかなか到達しないから
      LIMIT_MIN( input_vel, limit_vel);//着地時の下向きの速度を制限
      f_in_out.abs.p(Z) = f_old_in.abs.p(Z) + input_vel * DT;
      LIMIT_MIN( f_in_out.abs.p(Z), f_in_out.offs.p(Z));
      const double contact_threshold = 0.005;
      is_f_contact_out = ((f_in_out.abs.p(Z)-f_in_out.offs.p(Z)) <= contact_threshold);
    }
    void modifyFootRotAndXYForContact(HumanPose& out){
      calcFootRotAndXYTransitionForContact(out.tgt[rf],rf_vert);
      calcFootRotAndXYTransitionForContact(out.tgt[lf],lf_vert);
    }
    void calcFootRotAndXYTransitionForContact(PoseTGT& foot_in, const std::vector<hrp::Vector3>& act_sole_size){
      std::vector<hrp::Vector3> act_foot_vert;
      double min_height = foot_in.abs.p(Z);
      int min_height_id = 0;
      for(int i=0;i<act_sole_size.size();i++){
        act_foot_vert.push_back(hrp::rotFromRpy(foot_in.abs.rpy) * act_sole_size[i]);
        act_foot_vert[i](Z) += foot_in.abs.p(Z);
        if(act_foot_vert[i](Z) < min_height){
          min_height = act_foot_vert[i](Z);
          min_height_id = i;
        }
      }
      if(act_foot_vert[min_height_id](Z) < foot_in.offs.p(Z)){
        foot_in.abs.p(Z) += foot_in.offs.p(Z) - act_foot_vert[min_height_id](Z);
      }
    }
    bool applyCOMToSupportRegionLimit(const hrp::Vector3& rfin_abs, const hrp::Vector3& lfin_abs, hrp::Vector3& comin_abs){//boost::geometryがUbuntu12だとないから・・・
      hull_com.clear();
      createSupportRegionByFootPos(rfin_abs, lfin_abs, rf_safe_region, lf_safe_region, hull_com);
      hrp::Vector2 cog(comin_abs(X),comin_abs(Y));
      if(!isPointInHullOpenCV(cog,hull_com)){ calcNearestPointOnHull(cog,hull_com,cog); }//外に出たら最近傍点に頭打ち
      comin_abs(X) = cog(X);
      comin_abs(Y) = cog(Y);
      return true;
    }
    bool applyCOMStateLimitByCapturePoint(const hrp::Vector3& com_in, const hrp::Vector3& rfin_abs, const hrp::Vector3& lfin_abs, hrp::Vector3& com_ans_old, hrp::Vector3& com_ans){
      com_ans = com_in;
      if (is_initial_loop)com_ans_old = com_in;
      hrp::Vector3 com_vel = (com_in - com_ans_old)/DT;
      hull_dcp.clear();
      hull_acp.clear();
      hrp::Vector4 marginDelta_for_dcp(+0.001,-0.001,0.001,-0.001);
//      hrp::Vector4 marginDelta_for_acp(+0.001,-0.001,0.001,-0.001);
      hrp::Vector4 marginDelta_for_acp(+0.002,-0.002,0.002,-0.002);
//      hrp::Vector4 marginDelta_for_dcp(+0.001,-0.001,0.001,-0.001);
//      hrp::Vector4 marginDelta_for_acp(+0.01,-0.01,0.01,-0.01);
      createSupportRegionByFootPos(rfin_abs, lfin_abs, rf_safe_region+marginDelta_for_dcp, lf_safe_region+marginDelta_for_dcp, hull_dcp);
      createSupportRegionByFootPos(rfin_abs, lfin_abs, rf_safe_region+marginDelta_for_acp, lf_safe_region+marginDelta_for_acp, hull_acp);
      hrp::Vector2 com_vel_ans_2d;
      regulateCOMVelocityByCapturePointVec( hrp::Vector2(com_ans_old(X),com_ans_old(Y)), hrp::Vector2(com_vel(X),com_vel(Y)), hull_dcp, hull_acp, com_vel_ans_2d);
      com_ans(X) = com_ans_old(X) + com_vel_ans_2d(X) * DT;
      com_ans(Y) = com_ans_old(Y) + com_vel_ans_2d(Y) * DT;
      com_ans_old = com_ans;
      Vector2ToVector3(hrp::Vector2(com_ans(X),com_ans(Y)) + com_vel_ans_2d * sqrt( H_cur / G ), cp_dec);
      Vector2ToVector3(hrp::Vector2(com_ans(X),com_ans(Y)) - com_vel_ans_2d * sqrt( H_cur / G ), cp_acc);
      if(DEBUG){
        fprintf(cz_log,"com_ans: %f %f ",com_ans(X),com_ans(Y));
        fprintf(cz_log,"com_ans_cp_d: %f %f ",cp_dec(X),cp_dec(Y));
        fprintf(cz_log,"com_ans_cp_a: %f %f ",cp_acc(X),cp_acc(Y));
        for(int i=0;i<hull_dcp.size();i++)fprintf(sr_log,"hull_vert%d: %f %f\n",i,hull_dcp[i](X),hull_dcp[i](Y));
        fprintf(sr_log,"hull_vert%d: %f %f\n",(int)hull_dcp.size(),hull_dcp[0](X),hull_dcp[0](Y));//閉じる
        fprintf(sr_log,"\n");
      }
      return true;
    }
    void regulateCOMVelocityByCapturePointVec(const hrp::Vector2& com_pos, const hrp::Vector2& com_vel, const std::vector<hrp::Vector2>& hull_d, const std::vector<hrp::Vector2>& hull_a, hrp::Vector2& com_vel_ans){
      hrp::Vector2 com_vel_decel_ok,com_vel_accel_ok;
      com_vel_decel_ok = com_vel_accel_ok = com_vel_ans = com_vel;

      //減速CP条件(現在のCPを常に両足裏で頭打ち)
      hrp::Vector2 cp_dec_tmp = com_pos + com_vel * sqrt( H_cur / G );
      hrp::Vector2 cp_dec_ragulated;
      if(!isPointInHullOpenCV(cp_dec_tmp,hull_d)){
        calcCrossPointOnHull(com_pos, cp_dec_tmp, hull_d, cp_dec_ragulated);
        com_vel_decel_ok = (cp_dec_ragulated - com_pos) / sqrt( H_cur / G );
      }

      //減速CP条件2(指令のCPに対する現在発揮可能なCP)
//      hrp::Vector2 cp_dec_ref = com_forcp_ref + com_vel_forcp_ref * sqrt( H_cur / G );
//      hrp::Vector2 cp_dec_ref_ragulated = cp_dec_ref;
//      if(!isPointInHullOpenCV(cp_dec_ref,hull_d)){
//        calcCrossPointOnHull(com_pos, cp_dec_ref, hull_d, cp_dec_ref_ragulated);
//      }
//      com_vel_decel_ok = (cp_dec_ref_ragulated - com_pos) / sqrt( H_cur / G );

      //減速CP条件2(指令のCOMに収束する現在発揮可能なCP)
//      hrp::Vector2 cp_dec_ref = com_forcp_ref;
//      hrp::Vector2 cp_dec_ref_ragulated = cp_dec_ref;
//      if(!isPointInHullOpenCV(cp_dec_ref,hull_d)){
//        calcCrossPointOnHull(com_pos, cp_dec_ref, hull_d, cp_dec_ref_ragulated);
//      }
//      hrp::Vector2 vel_max_decel_ok = (cp_dec_ref_ragulated - com_pos) / sqrt( H_cur / G );
//      if( com_vel.dot(com_vel) > vel_max_decel_ok.dot(com_vel)){
//        com_vel_decel_ok = vel_max_decel_ok;
//      }else{
//        com_vel_decel_ok = com_vel;
//      }

      //加速CP条件(ACP使用)
      hrp::Vector2 cp_acc_tmp = com_pos - com_vel * sqrt( H_cur / G );
      hrp::Vector2 cp_acc_ragulated;
      if(!isPointInHullOpenCV(cp_acc_tmp,hull_a)){
         calcCrossPointOnHull(com_pos, cp_acc_tmp, hull_a, cp_acc_ragulated);
         com_vel_accel_ok = (-cp_acc_ragulated + com_pos ) / sqrt( H_cur / G );
      }
      cp_acc_old = cp_acc_ragulated;

      //加速CP条件2(ZMP使用)
      static hrp::Vector2 com_vel_ans_old = com_vel;
//      hrp::Vector2 com_acc = (com_vel - com_vel_ans_old) / DT;
//      hrp::Vector2 zmp_in = com_pos - com_acc / G * H_cur;
//      hrp::Vector2 zmp_regulated;
//      if(!isPointInHullOpenCV(zmp_in,hull_a)){
//        calcCrossPointOnHull(com_pos, zmp_in, hull_a, zmp_regulated);
//        hrp::Vector2 com_acc_regulated = (com_pos - zmp_regulated)*G/H_cur;
//        com_vel_accel_ok = com_vel_ans_old + com_acc_regulated*DT;
//      }

      //加速減速条件マージ
      if( com_vel_decel_ok.dot(com_vel) < com_vel_accel_ok.dot(com_vel)){//normじゃダメ？
        com_vel_ans = com_vel_decel_ok;
      }else{
        com_vel_ans = com_vel_accel_ok;
      }
      com_vel_ans_old = com_vel_ans;
    }
    void createSupportRegionByFootPos(const hrp::Vector3& rfin_abs, const hrp::Vector3& lfin_abs, const hrp::Vector4& rf_mgn, const hrp::Vector4& lf_mgn, std::vector<hrp::Vector2>& hull_ans){
      rflf_points.clear();
      hull_ans.clear();
      for(int i=0;i<2;i++){
        for(int j=2;j<4;j++){
          rflf_points.push_back(hrp::Vector2(rfin_abs(X) + rf_mgn(i),    rfin_abs(Y) + rf_mgn(j)));
          rflf_points.push_back(hrp::Vector2(lfin_abs(X) + lf_mgn(i),    lfin_abs(Y) + lf_mgn(j)));
        }
      }
      makeConvexHullOpenCV(rflf_points, hull_ans);
    }
    void calcWorldZMP(const hrp::Vector3& rfpos, const hrp::Vector3& lfpos, const hrp::dvector6& rfwin, const hrp::dvector6& lfwin, hrp::Vector3& zmp_ans){
      hrp::Vector3 rfzmp,lfzmp;
      const double F_H_OFFSET = 0.03;//地面から6軸センサ原点への高さ
      if( rfwin(fz) > 1.0e-6 ){
        rfzmp(0) = ( - rfwin(ty) - rfwin(fx) * F_H_OFFSET + rfwin(fz) * 0 ) / rfwin(fz) + rfpos(0);
        rfzmp(1) = (   rfwin(tx) - rfwin(fy) * F_H_OFFSET + rfwin(fz) * 0 ) / rfwin(fz) + rfpos(1);
      }
      if( lfwin(fz) > 1.0e-6 ){
        lfzmp(0) = ( - lfwin(ty) - lfwin(fx) * F_H_OFFSET + lfwin(fz) * 0 ) / lfwin(fz) + lfpos(0);
        lfzmp(1) = (   lfwin(tx) - lfwin(fy) * F_H_OFFSET + lfwin(fz) * 0 ) / lfwin(fz) + lfpos(1);
      }
      if( rfwin(fz) > 1.0e-6 || lfwin(fz) > 1.0e-6 ){
        zmp_ans(0) = ( rfzmp(0)*rfwin(fz) + lfzmp(0)*lfwin(fz) ) / ( rfwin(fz) + lfwin(fz));
        zmp_ans(1) = ( rfzmp(1)*rfwin(fz) + lfzmp(1)*lfwin(fz) ) / ( rfwin(fz) + lfwin(fz));
      }else{ zmp_ans(0) = 0; zmp_ans(1) = 0; }
      zmp_ans(2) = 0;
    }
    void calcXYMarginToHull(const hrp::Vector2& check_point, const std::vector<hrp::Vector2>& hull, hrp::Vector4& margin_ans){
      hrp::Vector4 margin_abs;
      hrp::Vector2 cross_pt, anchor_vec[4] = {hrp::Vector2(1,0), hrp::Vector2(-1,0), hrp::Vector2(0,1), hrp::Vector2(0,-1)};//前後左右に伸ばしたアンカーとの交点を見る
      for(int direc=0;direc<4;direc++){
        calcCrossPointOnHull(check_point, check_point+anchor_vec[direc], hull, cross_pt);
        margin_abs(direc) = (cross_pt - check_point).norm();
      }
      margin_ans(0) =  margin_abs(0);
      margin_ans(1) = -margin_abs(1);
      margin_ans(2) =  margin_abs(2);
      margin_ans(3) = -margin_abs(3);
    }
    bool calcCrossPointOnHull(const hrp::Vector2& pt_in_start, const hrp::Vector2& pt_out_goal, const std::vector<hrp::Vector2>& hull, hrp::Vector2& pt_will_cross){
      hrp::Vector2 anchor_vec = pt_out_goal - pt_in_start;
      for(int i=0;i<hull.size();i++){
        int i_nxt = (i!=hull.size()-1 ? i+1 : 0);
        hrp::Vector2 cur_pt = hull[i], nxt_pt = hull[i_nxt];
        hrp::Vector2 cur_edge = nxt_pt - cur_pt;
        double dBunbo = hrpVector2Cross(anchor_vec,cur_edge);
        if( dBunbo != 0.0) {//平行の場合を外す
          hrp::Vector2 vectorAC = hrp::Vector2(hull[i](0),hull[i](1)) - pt_in_start;
          double dR = hrpVector2Cross(vectorAC,cur_edge) / dBunbo;
          double dS = hrpVector2Cross(vectorAC,anchor_vec) / dBunbo;
          if(dR > 1e-9 && dS >= 0.0 && dS <= 1.0){//dRには数値誤差が乗る
            pt_will_cross = pt_in_start + dR * anchor_vec;
            //dist = dR * anchor_vec.norm();
            return true;
          }
        }
      }
      return false;
    }
    double calcNearestPointOnHull(const hrp::Vector2& tgt_pt, const std::vector<hrp::Vector2>& hull, hrp::Vector2& pt_ans){
      double cur_nearest_dist, ans_nearest_dist;
      hrp::Vector2 cur_nearest_pt, ans_nearest_pt;
      for(int i=0;i<hull.size();i++){
        int i_nxt = (i!=hull.size()-1 ? i+1 : 0);
        hrp::Vector2 cur_pt = hull[i], nxt_pt = hull[i_nxt];
        hrp::Vector2 cur_edge = nxt_pt - cur_pt;
        hrp::Vector2 tgt_pt_v = tgt_pt - cur_pt;
        double tgt_pt_projected_length = tgt_pt_v.dot(cur_edge.normalized());
        if(tgt_pt_projected_length > cur_edge.norm() ){//tgt_pt's nearest point is on the　i+1-th vertex
          cur_nearest_pt = nxt_pt;
        }else if(tgt_pt_projected_length < 0 ){//tgt_pt's nearest point is on the　i-th vertex
          cur_nearest_pt = cur_pt;
        }else{//tgt_pt's nearest point is on the line
          cur_nearest_pt = cur_pt + tgt_pt_projected_length * cur_edge.normalized();
        }
        cur_nearest_dist = (tgt_pt - cur_nearest_pt).norm();
        if(i==0){//set first candidate as nearest
          ans_nearest_dist = cur_nearest_dist;
          ans_nearest_pt = cur_nearest_pt;
        }else if( cur_nearest_dist < ans_nearest_dist ){//update nearest candidate
          ans_nearest_dist = cur_nearest_dist;
          ans_nearest_pt = cur_nearest_pt;
        }
      }
      pt_ans = ans_nearest_pt;
      return ans_nearest_dist;
    }
    void makeConvexHullOpenCV(const std::vector<hrp::Vector2>& pts, std::vector<hrp::Vector2>& hull_ans){
      points.clear();
      cvhull.clear();
      hull_ans.clear();
      for(int i=0;i<pts.size();i++){ points.push_back( cv::Point2f( pts[i](0), pts[i](1)) ); }
      cv::convexHull(cv::Mat(points),cvhull,true);
      for(int i=0;i<cvhull.size();i++){ hull_ans.push_back( hrp::Vector2( cvhull[i].x, cvhull[i].y ) );  }
    }
    //void makeConvexHullQHull(const std::vector<hrp::Vector2>& pts, std::vector<hrp::Vector2>& hull_ans){
    //  const int dim = 2;
    //  double points[dim*pts.size()];
    //  for(int i=0;i<pts.size();i++){ points[dim*i] = pts[i](0); points[dim*i+1] = pts[i](1); }
    //  qh_new_qhull( dim, pts.size(), points, false, "qhull ", NULL, stderr);
    //  pointT *point, *pointtemp;
    //  hull_ans.clear();
    //  FORALLpoints hull_ans.push_back( hrp::Vector2(point[0],point[1]) );
    //  qh_freeqhull(!qh_ALL);
    //  //#define qh_ORIENTclock 1
    //}
    bool isPointInHullOpenCV(const hrp::Vector2& pt, const std::vector<hrp::Vector2>& hull){
      cvhull.clear();
      for(int i=0;i<hull.size();i++){ cvhull.push_back( cv::Point2f( hull[i](0), hull[i](1)) ); }
      return (cv::pointPolygonTest(cv::Mat(cvhull), cv::Point2f(pt(0),pt(1)), false) > 0);
    }
    void applyZMPCalcFromCOM(const hrp::Vector3& comin, hrp::Vector3& zmpout){
      comacc = (comin - 2 * com_old + com_oldold)/(DT*DT);
      comacc = acc4zmp_v_filters.passFilter(comacc);
      zmpout(X) = comin(X)-(H_cur/G)*comacc(X);
      zmpout(Y) = comin(Y)-(H_cur/G)*comacc(Y);
      com_oldold = com_old;
      com_old = comin;
    }
    void applyVelLimit(const HumanPose& out_old, HumanPose& out){
      const double EE_MAX_VEL = 2.0;// m/s
      for(int i=0, l[6]={com,rf,lf,rh,lh,head}; i<6; i++){
        hrp::Vector3 diff = out.tgt[i].abs.p  - out_old.tgt[i].abs.p;
        for(int j=0;j<3;j++)LIMIT_MINMAX( diff(j), -EE_MAX_VEL*DT, EE_MAX_VEL*DT);
        out.tgt[i].abs.p = out_old.tgt[i].abs.p + diff;
      }
    }
};

#endif // HUMANMASTERSLAVE_H
