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

#define LIMIT_MIN(x,min) (x= ( x<min ? min:x ))
#define LIMIT_MAX(x,max) (x= ( x<max ? x:max ))
#define LIMIT_MINMAX(x,min,max) ((x= (x<min  ? min : x<max ? x : max)))

//実機だとログ出力で落ちる？getenv("HOME")ダメ？
#define DEBUG 0

namespace myconst{
//  const double INFMIN = std::numeric_limits<double>::min(), INFMAX = std::numeric_limits<double>::max();
  const double INFMIN = - std::numeric_limits<double>::max(), INFMAX = std::numeric_limits<double>::max();
  const double D2R = M_PI/180.0;
}
using namespace myconst;

#define DEF_UTIL_CONST \
    static const enum pose_tgt{ com, rf, lf, rh, lh, head, zmp, num_pose_tgt } p_id;\
    static const enum wrench_tgt{ rfw, lfw, num_wrench_tgt } w_id;\
    static const enum lr_direction{ L, R, LR} lr_direc;\
    static const enum xyz_direction{ X, Y, Z, XYZ} xyz_direc;\
    static const enum rpy_direction{ r, p, y, rpy} rpy_direc;\
    static const enum ft_direction{ fx, fy, fz, tx, ty, tz, ft_xyz} w_direc;\
    static const enum minmax_id{ MIN, MAX, MINMAX} mm_id;\

class BiquadIIRFilterVec{
  private:
    IIRFilter filters[3];
    hrp::Vector3 ans;
  public:
    static const double Q_BUTTERWORTH = 0.707106781, Q_NOOVERSHOOT = 0.5;
    BiquadIIRFilterVec(){
    };
    ~BiquadIIRFilterVec() {};
    void setParameter(const hrp::Vector3& fc_in, const double& HZ, const double& Q = Q_BUTTERWORTH){
     for(int i=0;i<3;i++){
         filters[i].setParameterAsBiquad((double)fc_in(i), Q, HZ);
      }
    };
    void setParameter(const double& fc_in, const double& HZ, const double& Q = Q_BUTTERWORTH){//overload
      setParameter(hrp::Vector3(fc_in,fc_in,fc_in), HZ, Q);
    };
    hrp::Vector3 passFilter(const hrp::Vector3& input){
      for(int i=0;i<3;i++){ ans(i) = filters[i].passFilter((double)input(i)); }
      return ans;
    };
    void init(const hrp::Vector3& initial_input){
      for(int i=0;i<3;i++){ filters[i].reset((double)initial_input(i));}
    }
};

class HRPPose3D{
  public:
    hrp::Vector3 p,p_offs,rpy,rpy_offs;
    HRPPose3D(){ clear(); }
    ~HRPPose3D(){}
    void clear(){
      p = p_offs = rpy = rpy_offs = hrp::Vector3::Zero();
    }
};
class Wrench6{
  public:
    hrp::dvector6 w;
    Wrench6(){ clear(); }
    ~Wrench6(){}
    void clear(){ w = hrp::dvector6::Zero(); }
};

class HumanPose{
    DEF_UTIL_CONST
  public:
    std::vector<HRPPose3D> P;
    std::vector<Wrench6> w;
    HumanPose(){
      P.resize(num_pose_tgt);
      w.resize(num_wrench_tgt);
      clear();
    }
    ~HumanPose(){cerr<<"HumanPose destructed"<<endl;}
    void clear(){
      for(std::vector<HRPPose3D>::iterator it = P.begin(); it != P.end(); it++){ it->clear(); }
      for(std::vector<Wrench6>::iterator it = w.begin(); it != w.end(); it++){ it->clear(); }
    }
    static void hp_printf(const HumanPose& in){
      const std::string pcap[] = {"c","rf","lf","rh","lh","hd","z"}, wcap[] = {"rw","lw"};
      for(int i=0;i<num_pose_tgt;i++){ fprintf(stderr,"\x1b[31m%s\x1b[39m%+05.2f %+05.2f %+05.2f ",pcap[i].c_str(),in.P[i].p(X),in.P[i].p(Y),in.P[i].p(Z)); }
//      for(int i=0;i<num_wrench_tgt;i++){ fprintf(stderr,"\x1b[31m%s\x1b[39m%+05.1f %+05.1f %+05.1f %+05.1f %+05.1f %+05.1f ",wcap[i].c_str(),in.w[i].w(fx),in.w[i].w(fy),in.w[i].w(fz),in.w[i].w(tx),in.w[i].w(ty),in.w[i].w(tz)); }
      printf("\n");
    }
    void print(){ hp_printf(*this); }
};

class RobotConfig{
    DEF_UTIL_CONST
  public:
    std::vector< std::vector<hrp::Vector3> > ee_rot_limit;
    RobotConfig(){
      std::vector<hrp::Vector3> init;
      init.resize(MINMAX);
      ee_rot_limit.resize(num_pose_tgt, init);
      ee_rot_limit[com][MIN] = hrp::Vector3(-30*D2R, -30*D2R, INFMIN);
      ee_rot_limit[com][MAX] = hrp::Vector3( 30*D2R,  30*D2R, INFMAX);
      ee_rot_limit[rf][MIN] = hrp::Vector3(-30*D2R, -30*D2R, -20*D2R);
      ee_rot_limit[rf][MAX] = hrp::Vector3( 30*D2R,  30*D2R,   5*D2R);
      ee_rot_limit[lf][MIN] = hrp::Vector3(-30*D2R, -30*D2R,  -5*D2R);
      ee_rot_limit[lf][MAX] = hrp::Vector3( 30*D2R,  30*D2R,  20*D2R);
      ee_rot_limit[rh][MIN] = hrp::Vector3(-30*D2R, -30*D2R, -30*D2R);
      ee_rot_limit[rh][MAX] = hrp::Vector3( 30*D2R,  30*D2R,  30*D2R);
      ee_rot_limit[lh][MIN] = hrp::Vector3(-30*D2R, -30*D2R, -30*D2R);
      ee_rot_limit[lh][MAX] = hrp::Vector3( 30*D2R,  30*D2R,  30*D2R);
      ee_rot_limit[head][MIN] = hrp::Vector3( 0*D2R, -10*D2R, -40*D2R);
      ee_rot_limit[head][MAX] = hrp::Vector3( 0*D2R,  20*D2R,  40*D2R);
    }
};

class HumanSynchronizer{
    DEF_UTIL_CONST
  private:
    double FUP_TIME;
    int cur_rfup_level,cur_lfup_level;
    double cur_rfup_rad,cur_lfup_rad;
    double FUP_HIGHT;
    double CNT_F_TH;
    hrp::Vector3 init_hp_calibcom;
    double MAXVEL,MAXACC;
    HumanPose rp_ref_out_old, hp_swap_checked, rp_old_ref_Vel;
    bool HumanSyncOn;
    struct timeval t_calc_start, t_calc_end;
    double h2r_ratio, tgt_h2r_ratio;
    double HZ,DT,G;
    hrp::Vector3 com_old,com_oldold,comacc;
    hrp::Vector3 comacc_ref;
    hrp::Vector3 com_CP_ref_old;
    std::vector<BiquadIIRFilterVec> tgt_pos_filters,tgt_rot_filters;
    BiquadIIRFilterVec calcacc_v_filters, acc4zmp_v_filters;
    BiquadIIRFilterVec com_in_filter;
    hrp::Vector3 r_zmp_raw;
    hrp::Vector4 rf_safe_region,lf_safe_region;//前後左右の順
    RobotConfig rc;
    std::vector<hrp::Vector2> rflf_points;
    std::vector<hrp::Vector2> hull_com,hull_dcp,hull_acp;
    std::vector<cv::Point2f> points,cvhull;

  public:
    double tgt_FUP_TIME;
    bool startCountdownForHumanSync;
    bool ht_first_call;
    bool is_rf_contact,is_lf_contact;//足上げ高さ0に到達しているか否か
    bool go_rf_landing,go_lf_landing;//足上げ指令の有無
//    HumanPose rp_wld_initpos;//IKでのリンク原点の位置(足首，手首など)
    bool use_x,use_y,use_z;
    HumanPose hp_wld_raw;
    HumanPose hp_plot;
    HumanPose rp_ref_out;
    FILE *sr_log,*cz_log;
    FILE *id_log;
    hrp::Vector3 init_wld_rp_rfeepos,init_wld_rp_lfeepos;
    hrp::Vector3 init_wld_hp_rfpos,init_wld_hp_lfpos;
    HRPPose3D baselinkpose;
    hrp::Vector3 com_vel_old;
    unsigned int loop;
    hrp::Vector3 pre_cont_rfpos,pre_cont_lfpos, pre_cont_rfrot,pre_cont_lfrot;
    std::vector<hrp::Vector3> rf_vert,lf_vert;
    double countdown_sec;
    hrp::Vector3 cp_dec,cp_acc;
    Wrench6 invdyn_ft;
    hrp::Vector2 com_forcp_ref,com_vel_forcp_ref;

    interpolator *init_zmp_com_offset_interpolator;
    double zmp_com_offset_ip_ratio;
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

    double H_cur;

    HumanSynchronizer(const double& dt){
      tgt_h2r_ratio = h2r_ratio = 0.96;//human 1.1m vs jaxon 1.06m
//      tgt_h2r_ratio = h2r_ratio = 0.62;//human 1.1m vs chidori 0.69m
      DT = dt;
      HZ = (int)(1.0/DT);
      G = 9.80665;
      use_x = use_y = use_z = true;
      cur_rfup_level = cur_lfup_level = 0;
      cur_rfup_rad = cur_lfup_rad = 0;
      is_rf_contact = is_lf_contact = true;
      go_rf_landing = go_lf_landing = false;
      pre_cont_rfpos = pre_cont_lfpos = pre_cont_rfrot = pre_cont_lfrot = hrp::Vector3::Zero();
      init_hp_calibcom = hrp::Vector3::Zero();
      tgt_FUP_TIME = FUP_TIME = 0.4;
      FUP_HIGHT = 0.05;
      CNT_F_TH = 20.0;
      MAXVEL = 0.4*10;
      HumanSyncOn = false;
      ht_first_call = true;
      startCountdownForHumanSync = false;
      countdown_sec = 5.0;
      loop = 0;

      //////////  hrp::Vector は初期化必要  ///////////
      com_old = com_oldold = comacc = hrp::Vector3::Zero();
      comacc_ref = hrp::Vector3::Zero();
      r_zmp_raw = hrp::Vector3::Zero();

      init_wld_rp_rfeepos = init_wld_rp_lfeepos = hrp::Vector3::Zero();
      init_wld_hp_rfpos = init_wld_hp_lfpos = hrp::Vector3::Zero();

      com_vel_old = hrp::Vector3::Zero();
      cp_dec = cp_acc = hrp::Vector3::Zero();

      tgt_pos_filters.resize(num_pose_tgt);
      tgt_rot_filters.resize(num_pose_tgt);
//      for(int i=0;i<tgt_pos_filters.size();i++)tgt_pos_filters[i].setParameter(0.6,HZ);//四肢拘束点用(position)
//      for(int i=0;i<tgt_rot_filters.size();i++)tgt_rot_filters[i].setParameter(1.0,HZ);//四肢拘束点用(Rotation)
//      tgt_pos_filters[0].setParameter(0.6,HZ);//重心pos用
//      tgt_rot_filters[0].setParameter(0.4,HZ);//重心rot用
//      tgt_pos_filters[1].setParameter(hrp::Vector3(0.6,0.6,1.0),HZ);//右足pos用
//      tgt_pos_filters[2].setParameter(hrp::Vector3(0.6,0.6,1.0),HZ);//左足pos用
//      calcacc_v_filters.setParameter(5,HZ);//加速度計算用
//      acc4zmp_v_filters.setParameter(1,HZ);//ZMP生成用ほぼこの値でいい
//      cam_rpy_filter.setParameter(1,HZ);//カメラアングル
      for(int i=0;i<tgt_pos_filters.size();i++)tgt_pos_filters[i].setParameter(1.0, HZ, BiquadIIRFilterVec::Q_NOOVERSHOOT);//四肢拘束点用(position)
      for(int i=0;i<tgt_rot_filters.size();i++)tgt_rot_filters[i].setParameter(1.0, HZ, BiquadIIRFilterVec::Q_NOOVERSHOOT);//四肢拘束点用(Rotation)
      tgt_pos_filters[0].setParameter(1.0, HZ, BiquadIIRFilterVec::Q_NOOVERSHOOT);//重心pos用
      tgt_rot_filters[0].setParameter(0.6, HZ, BiquadIIRFilterVec::Q_NOOVERSHOOT);//重心rot用
      tgt_pos_filters[1].setParameter(hrp::Vector3(5.0,1.0,1.2), HZ, BiquadIIRFilterVec::Q_NOOVERSHOOT);//右足pos用
      tgt_pos_filters[2].setParameter(hrp::Vector3(1.0,1.0,1.2), HZ, BiquadIIRFilterVec::Q_NOOVERSHOOT);//左足pos用
      calcacc_v_filters.setParameter(5, HZ, BiquadIIRFilterVec::Q_BUTTERWORTH);//加速度計算用
      acc4zmp_v_filters.setParameter(5, HZ, BiquadIIRFilterVec::Q_BUTTERWORTH);//ZMP生成用ほぼこの値でいい
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
      WBMSparam.upper_body_rmc_ratio = 0.5;
      WBMSparam.use_rh = WBMSparam.use_lh = true;
      WBMSparam.use_head = true;
      WBMSparam.use_manipulability_limit = false;

      rp_ref_out_old.clear();
      rp_ref_out.clear();

      rflf_points.reserve(8);//あらかじめreserveして高速化
      points.reserve(8);
      hull_com.reserve(6);
      hull_dcp.reserve(6);
      hull_dcp.reserve(6);
      cvhull.reserve(6);


      init_zmp_com_offset_interpolator = new interpolator(1, DT, interpolator::HOFFARBIB, 1);
      init_zmp_com_offset_interpolator->setName("zmp_com_offset_interpolator");
      double start_ratio = 0.0;
      init_zmp_com_offset_interpolator->set(&start_ratio);
      double goal_ratio = 1.0;
      init_zmp_com_offset_interpolator->go(&goal_ratio, 3.0, true);//3s

      if(DEBUG){
        sr_log = fopen("/home/ishiguro/HumanSync_support_region.log","w+");
        cz_log = fopen("/home/ishiguro/HumanSync_com_zmp.log","w+");
        id_log = fopen("/home/ishiguro/HumanSync_invdyn.log","w+");
      }
      cout<<"HumanSynchronizer constructed"<<endl;
    }
    ~HumanSynchronizer(){
      if(DEBUG)fclose(sr_log);
      if(DEBUG)fclose(cz_log);
      if(DEBUG)fclose(id_log);
      delete init_zmp_com_offset_interpolator;
      cout<<"HumanSynchronizer destructed"<<endl;
    }
    const double getRemainingCountDown() const { return countdown_sec; }
    const bool isHumanSyncOn() const { return HumanSyncOn; }
    const double getUpdateTime() const { return (double)(t_calc_end.tv_sec - t_calc_start.tv_sec) + (t_calc_end.tv_usec - t_calc_start.tv_usec)/1.0e6; }
    void setTargetHumanToRobotRatio(const double tgt_h2r_r_in){ tgt_h2r_ratio = tgt_h2r_r_in; }
    void setCurrentInputAsOffset(HumanPose& tgt){
      std::string ns[7] = {"com","rf","lf","rh","lh","zmp","head"};
      for(int i=0;i<7;i++){
        tgt.P[i].p_offs   =  tgt.P[i].p;
        tgt.P[i].rpy_offs =  tgt.P[i].rpy;
      }
    }
    void updateCountDown(){
      if(countdown_sec>0.0){
        countdown_sec -= DT;
      }else if(countdown_sec <= 0){
        HumanSyncOn = true;
        startCountdownForHumanSync = false;
      }
    }
    void update(){//////////  メインループ  ////////////
      gettimeofday(&t_calc_start, NULL);
      updateParams                        ();
//      applyInitHumanZMPToCOMOffset        (hp_rel_raw, init_hp_calibcom, hp_rel_raw);//怪しいので要修正
//      calcWorldZMP                        ((hp_rel_raw.P[rf].p+init_wld_hp_rfpos), (hp_rel_raw.P[lf].p+init_wld_hp_lfpos), hp_rel_raw.w[rfw], hp_rel_raw.w[lfw], hp_rel_raw.P[zmp].p);//足の位置はworldにしないと・・・
      autoLRSwapCheck                     (hp_wld_raw,hp_swap_checked);//入力の左右反転を常にチェック(＝手足の交差は不可能)
      convertRelHumanPoseToRelRobotPose   (hp_swap_checked, rp_ref_out);
      hp_plot = rp_ref_out;
      if(loop==0)rp_ref_out_old = rp_ref_out;


      if(isHumanSyncOn() && WBMSparam.set_com_height_fix)rp_ref_out.P[com].p(Z) = rp_ref_out.P[com].p_offs(Z) + WBMSparam.set_com_height_fix_val;//膝曲げトルクで落ちるときの応急措置
      judgeFootLandOnCommand              (hp_wld_raw.w[rfw], hp_wld_raw.w[lfw], go_rf_landing, go_lf_landing);
      lockSwingFootIfZMPOutOfSupportFoot  (rp_ref_out_old, go_rf_landing, go_lf_landing);//ここ
      applyEEWorkspaceLimit               (rp_ref_out);
      lockFootXYOnContact                 (go_rf_landing, go_lf_landing, rp_ref_out);//根本から改変すべき3
      if(go_rf_landing)rp_ref_out.P[rf].p(Z) = rp_ref_out.P[rf].p_offs(Z);
      if(go_lf_landing)rp_ref_out.P[lf].p(Z) = rp_ref_out.P[lf].p_offs(Z);
      if(!go_rf_landing)rp_ref_out.P[rf].p(Z) += WBMSparam.swing_foot_height_offset;
      if(!go_lf_landing)rp_ref_out.P[lf].p(Z) += WBMSparam.swing_foot_height_offset;

      if(DEBUG)fprintf(cz_log,"com_ref: %f %f ", rp_ref_out.P[com].p(X), rp_ref_out.P[com].p(Y));

      applyCOMToSupportRegionLimit        (rp_ref_out.P[rf].p, rp_ref_out.P[lf].p, rp_ref_out.P[com].p);
      setFootRotHorizontalIfGoLanding     (rp_ref_out_old, rp_ref_out);

//      applyVelLimit                       (rp_ref_out, rp_old_ref_Vel, rp_ref_out);

      applyCOMToSupportRegionLimit        (rp_ref_out.P[rf].p, rp_ref_out.P[lf].p, com_CP_ref_old);//これやらないと支持領域の移動によって1ステップ前のCOM位置はもうはみ出てるかもしれないから

      Vector3ToVector2(rp_ref_out.P[com].p,com_forcp_ref);
      static hrp::Vector2 com_forcp_ref_old;
      com_vel_forcp_ref = (com_forcp_ref - com_forcp_ref_old)/DT;
      com_forcp_ref_old = com_forcp_ref;

      applyCOMStateLimitByCapturePoint    (rp_ref_out.P[com].p, rp_ref_out.P[rf].p, rp_ref_out.P[lf].p, com_CP_ref_old, rp_ref_out.P[com].p);
      applyCOMToSupportRegionLimit        (rp_ref_out.P[rf].p, rp_ref_out.P[lf].p, rp_ref_out.P[com].p);
      applyLPFilter                       (rp_ref_out);

      r_zmp_raw = rp_ref_out.P[zmp].p;
      applyZMPCalcFromCOM                 (rp_ref_out.P[com].p,rp_ref_out.P[zmp].p);


      if(DEBUG)fprintf(cz_log,"com_ans_zmp: %f %f ",rp_ref_out.P[zmp].p(X),rp_ref_out.P[zmp].p(Y));
      if(DEBUG)fprintf(cz_log,"com_ans2: %f %f ",rp_ref_out.P[com].p(X),rp_ref_out.P[com].p(Y));

      overwriteFootZFromFootLandOnCommand (go_rf_landing, go_lf_landing, rp_ref_out);
      modifyFootRotAndXYForContact        (rp_ref_out.P[rf], rp_ref_out.P[lf]);

      if(DEBUG)fprintf(cz_log,"rf: %f %f %f ",rp_ref_out.P[rf].p(X),rp_ref_out.P[rf].p(Y),rp_ref_out.P[rf].p(Z));
      if(DEBUG)fprintf(cz_log,"lf: %f %f %f ",rp_ref_out.P[lf].p(X),rp_ref_out.P[lf].p(Y),rp_ref_out.P[lf].p(Z));
      if(DEBUG)fprintf(cz_log,"\n");



      H_cur = rp_ref_out.P[com].p(Z) - std::min((double)rp_ref_out.P[rf].p(Z), (double)rp_ref_out.P[rf].p(Z));
      com_vel_old = (rp_ref_out.P[com].p - rp_ref_out_old.P[com].p)/DT;
      rp_ref_out_old = rp_ref_out;
      loop++;
      gettimeofday(&t_calc_end, NULL);
    }
    void calibInitHumanCOMFromZMP(){
      init_hp_calibcom(X) = r_zmp_raw(X);
      init_hp_calibcom(Y) = r_zmp_raw(Y);
    }
    static void Vector3ToVector2(const hrp::Vector3& in, hrp::Vector2& out){ out(X) = in(X); out(Y) = in(Y);}
    static void Vector2ToVector3(const hrp::Vector2& in, hrp::Vector3& out){ out(X) = in(X); out(Y) = in(Y);}
    static void Point3DToVector3(const RTC::Point3D& in, hrp::Vector3& out){ out(X) = in.x; out(Y) = in.y; out(Z) = in.z;}
    static void Oriantation3DToVector3(const RTC::Orientation3D& in, hrp::Vector3& out){ out(X) = in.r; out(Y) = in.p; out(Z) = in.y;}
    static void Vector3ToPoint3D(const hrp::Vector3& in, RTC::Point3D& out){ out.x = in(X); out.y = in(Y); out.z = in(Z);}
    static void Vector3ToOriantation3D(const hrp::Vector3& in, RTC::Orientation3D& out){ out.r = in(X); out.p = in(Y); out.y = in(Z);}
    static void Pose3DToHRPPose3D(const RTC::Pose3D& in, HRPPose3D& out){ Point3DToVector3(in.position,out.p); Oriantation3DToVector3(in.orientation,out.rpy); }
    static void HRPPose3DToPose3D(const HRPPose3D& in, RTC::Pose3D& out){ Vector3ToPoint3D(in.p,out.position); Vector3ToOriantation3D(in.rpy,out.orientation); }
    static void DoubleSeqToWrench6(const RTC::TimedDoubleSeq::_data_seq& in, Wrench6& out){
      if(in.length() == 6){out.w(fx)=in[0]; out.w(fy)=in[1]; out.w(fz)=in[2]; out.w(tz)=in[3]; out.w(ty)=in[4]; out.w(tz)=in[5];
      }else{ std::cerr<<"[WARN] HumanPose::Wrench6 DoubleSeqToWrench6() invalid data length"<<std::endl; out.clear(); }
    }
    static void Wrench6ToDoubleSeq(const Wrench6& in, RTC::TimedDoubleSeq::_data_seq& out){
      if(out.length() == 6){out[0]=in.w(fx); out[1]=in.w(fy); out[2]=in.w(fz); out[3]=in.w(tx); out[4]=in.w(ty); out[5]=in.w(tz);
      }else{ std::cerr<<"[WARN] HumanPose::Wrench6 DoubleSeqToWrench6() invalid data length"<<std::endl; }
    }
    static double hrpVector2Cross(const hrp::Vector2& a, const hrp::Vector2& b){ return a(X)*b(Y)-a(Y)*b(X); }

  private:
    void updateParams(){
//      tgt_h2r_ratio = H_cur/WBMSparam.human_com_height;
      updateHumanToRobotRatio(tgt_h2r_ratio);
    }
    void updateHumanToRobotRatio(const double h2r_r_goal){//h2r_ratioに伴って変わる変数の処理もここに書く
      const double step = 0.001;
      if(h2r_r_goal - h2r_ratio > step){h2r_ratio += step; cout<<"Interpolating HumanToRobotRatio as "<<h2r_ratio<<endl;}
      else if(h2r_r_goal - h2r_ratio < (-1)*step){h2r_ratio -= step; cout<<"Interpolating HumanToRobotRatio as "<<h2r_ratio<<endl;}
      else{h2r_ratio = h2r_r_goal;}
      init_wld_hp_rfpos = init_wld_rp_rfeepos/h2r_ratio;
      init_wld_hp_lfpos = init_wld_rp_lfeepos/h2r_ratio;
    }
    void autoLRSwapCheck(const HumanPose& in, HumanPose& out){
      out = in;
      if(in.P[rh].p_offs(Y) > in.P[lh].p_offs(Y) ){
        out.P[rh] = in.P[lh];
        out.P[lh] = in.P[rh];
      }
      if(in.P[rf].p_offs(Y) > in.P[lf].p_offs(Y) ){
        out.P[rf] = in.P[lf];
        out.P[lf] = in.P[rf];
      }
    }
    void applyInitHumanZMPToCOMOffset(const HumanPose& in, const hrp::Vector3& com_offs, HumanPose& out){
      if(isHumanSyncOn()){
        if(!init_zmp_com_offset_interpolator->isEmpty()){
          init_zmp_com_offset_interpolator->get(&zmp_com_offset_ip_ratio, true);
        }
      }else{
        zmp_com_offset_ip_ratio = 0;
      }
      out = in;
      out.P[com].p = in.P[com].p + zmp_com_offset_ip_ratio * com_offs;
    }
    void applyLPFilter(HumanPose& tgt){
      const int ns[] = {com,rf,lf,rh,lh,head}, num_ns = sizeof(ns)/sizeof(int);
      if(loop==0){
        cout<<"init LPF"<<endl;
        for(int i=0;i<num_ns;i++){ tgt_pos_filters[ns[i]].init(tgt.P[ns[i]].p);  }// tgt_pos_filters[ns[i]].passFilter(tgt.P[ns[i]].p);  }
        for(int i=0;i<num_ns;i++){ tgt_rot_filters[ns[i]].init(tgt.P[ns[i]].rpy); }//tgt_pos_filters[ns[i]].passFilter(tgt.P[ns[i]].rpy);}
      }
      for(int i=0;i<num_ns;i++){ tgt.P[ns[i]].p   = tgt_pos_filters[ns[i]].passFilter(tgt.P[ns[i]].p);  }
      for(int i=0;i<num_ns;i++){ tgt.P[ns[i]].rpy = tgt_rot_filters[ns[i]].passFilter(tgt.P[ns[i]].rpy);}
    }
    void convertRelHumanPoseToRelRobotPose(const HumanPose& hp_in, HumanPose& rp_out){//結局初期指定からの移動量(=Rel)で計算をしてゆく
      const int ns[] = {com,rf,lf,rh,lh,zmp}, num_ns = sizeof(ns)/sizeof(int);
      for(int i=0;i<num_ns;i++){
        rp_out.P[ns[i]].p   =  h2r_ratio * (hp_in.P[ns[i]].p - hp_in.P[ns[i]].p_offs)   + rp_out.P[ns[i]].p_offs;
        rp_out.P[ns[i]].rpy =  hrp::rpyFromRot( hrp::rotFromRpy(hp_in.P[ns[i]].rpy) * hrp::rotFromRpy(hp_in.P[ns[i]].rpy_offs).transpose() * hrp::rotFromRpy(rp_out.P[ns[i]].rpy_offs) );
      }
      rp_out.P[head] = hp_in.P[head];
      rp_out.w[rfw]  = hp_in.w[rfw];
      rp_out.w[lfw]  = hp_in.w[lfw];
    }
    void judgeFootLandOnCommand(const Wrench6& rfw_in, const Wrench6& lfw_in, bool& rfloc_ans, bool& lfloc_ans){
      if(isHumanSyncOn()){
        if      (rfloc_ans  && rfw_in.w(fz)<CNT_F_TH   ){rfloc_ans = false;}//右足ついた状態から上げる
        else if (!rfloc_ans && rfw_in.w(fz)>CNT_F_TH+30){rfloc_ans = true;}//右足浮いた状態から下げる
        if      (lfloc_ans  && lfw_in.w(fz)<CNT_F_TH   ){lfloc_ans = false;}//左足ついた状態から上げる
        else if (!lfloc_ans && lfw_in.w(fz)>CNT_F_TH+30){lfloc_ans = true;}//左足浮いた状態から下げる
      }
      //      if(!rfloc_ans && !lfloc_ans){rfloc_ans = lfloc_ans = true;}//両足ジャンプは禁止
    }
    void lockSwingFootIfZMPOutOfSupportFoot(const HumanPose& tgt, bool& rfcs_ans, bool& lfcs_ans){
      hrp::Vector2 inside_vec_rf(0,1),inside_vec_lf(0,-1);
      hrp::Vector2 rf2zmp = hrp::Vector2(tgt.P[zmp].p(X),tgt.P[zmp].p(Y)) - hrp::Vector2(tgt.P[rf].p(X),tgt.P[rf].p(Y));
      hrp::Vector2 lf2zmp = hrp::Vector2(tgt.P[zmp].p(X),tgt.P[zmp].p(Y)) - hrp::Vector2(tgt.P[lf].p(X),tgt.P[lf].p(Y));
      if( inside_vec_rf.dot(rf2zmp) / inside_vec_rf.norm() > WBMSparam.auto_swing_foot_landing_threshold ){ lfcs_ans = true; }
      if( inside_vec_lf.dot(lf2zmp) / inside_vec_lf.norm() > WBMSparam.auto_swing_foot_landing_threshold ){ rfcs_ans = true; }
    }
    void setFootRotHorizontalIfGoLanding(const HumanPose& rp_ref_out_old, HumanPose& rp_ref_out){
      if(go_rf_landing){ rp_ref_out.P[rf].rpy(r) = rp_ref_out.P[rf].rpy(p) = 0; }
      if(go_lf_landing){ rp_ref_out.P[lf].rpy(r) = rp_ref_out.P[lf].rpy(p) = 0; }
    }
    void overwriteFootZFromFootLandOnCommand(const bool& rf_goland_in, const bool& lf_goland_in, HumanPose& tgt){
      const double com2rf_dist = fabs((double)tgt.P[rf].p(Y) - (double)tgt.P[com].p(Y));
      const double com2lf_dist = fabs((double)tgt.P[lf].p(Y) - (double)tgt.P[com].p(Y));
      limitGroundContactVelocity(rf_goland_in, com2rf_dist, rp_ref_out_old.P[rf], tgt.P[rf], is_rf_contact);
      limitGroundContactVelocity(lf_goland_in, com2lf_dist, rp_ref_out_old.P[lf], tgt.P[lf], is_lf_contact);
    }
    void limitGroundContactVelocity(const bool& go_land_in, const double& com2foot_dist, const HRPPose3D& f_old_in, HRPPose3D& f_in_out, bool& is_f_contact_out){
      double penalty = 1.0 + (0.2 - com2foot_dist) * 5.0;//1.0 ~ 2.0 本当はCPに比例すべき？
      LIMIT_MINMAX( penalty, 1.0, 2.0);
      LIMIT_MIN( f_in_out.p(Z), f_in_out.p_offs(Z));
      double input_vel = (f_in_out.p(Z) - f_old_in.p(Z)) / DT;
      const double vel_limit_k = WBMSparam.foot_vertical_vel_limit_coeff * penalty;//地面から0.02[m]地点で0.02*8=0.16[m/s]出ている計算
      double limit_vel = - vel_limit_k * (f_old_in.p(Z) - f_old_in.p_offs(Z));//sinの時は平均0.05~0.10[m/s]
      const double max_vel_threshold = -0.01;
      LIMIT_MAX( limit_vel, max_vel_threshold);//速度0に近づくと目標になかなか到達しないから
      LIMIT_MIN( input_vel, limit_vel);//着地時の下向きの速度を制限
      f_in_out.p(Z) = f_old_in.p(Z) + input_vel * DT;
      LIMIT_MIN( f_in_out.p(Z), f_in_out.p_offs(Z));
      const double contact_threshold = 0.005;
      is_f_contact_out = ((f_in_out.p(Z)-f_in_out.p_offs(Z)) <= contact_threshold);
    }
    void modifyFootRotAndXYForContact(HRPPose3D& rf_in, HRPPose3D& lf_in){
      calcFootRotAndXYTransitionForContact(rf_in,rf_vert);
      calcFootRotAndXYTransitionForContact(lf_in,lf_vert);
    }
    void calcFootRotAndXYTransitionForContact(HRPPose3D& foot_in, const std::vector<hrp::Vector3>& act_sole_size){
      std::vector<hrp::Vector3> act_foot_vert;
      double min_height = foot_in.p(Z);
      int min_height_id = 0;
      for(int i=0;i<act_sole_size.size();i++){
        act_foot_vert.push_back(hrp::rotFromRpy(foot_in.rpy) * act_sole_size[i]);
        act_foot_vert[i](Z) += foot_in.p(Z);
        if(act_foot_vert[i](Z) < min_height){
          min_height = act_foot_vert[i](Z);
          min_height_id = i;
        }
      }
      if(act_foot_vert[min_height_id](Z) < foot_in.p_offs(Z)){
        foot_in.p(Z) += foot_in.p_offs(Z) - act_foot_vert[min_height_id](Z);
      }
    }
    void lockFootXYOnContact(const bool& rfcs_in, const bool& lfcs_in, HumanPose& tgt){
      if(rfcs_in){
        tgt.P[rf].p(X) = pre_cont_rfpos(X);
        tgt.P[rf].p(Y) = pre_cont_rfpos(Y);
        tgt.P[rf].rpy(y) = pre_cont_rfrot(Z);
      }else{
        pre_cont_rfpos(X) = tgt.P[rf].p(X);
        pre_cont_rfpos(Y) = tgt.P[rf].p(Y);
        pre_cont_rfrot(Z) = tgt.P[rf].rpy(y);
      }
      if(lfcs_in){
        tgt.P[lf].p(X) = pre_cont_lfpos(X);
        tgt.P[lf].p(Y) = pre_cont_lfpos(Y);
        tgt.P[lf].rpy(y) = pre_cont_lfrot(Z);
      }else{
        pre_cont_lfpos(X) = tgt.P[lf].p(X);
        pre_cont_lfpos(Y) = tgt.P[lf].p(Y);
        pre_cont_lfrot(Z) = tgt.P[lf].rpy(y);
      }
    }
    void applyEEWorkspaceLimit(HumanPose& tgt){
      const double MAX_FW = 0.25;
//      const double MAX_FW = 0.5;
//      const double MAX_FW = 1000000;//manipulability test
      const double FOOT_2_FOOT_COLLISION_MARGIIN = 0.16;
      if(!is_rf_contact && is_lf_contact){//右足浮遊時
        const hrp::Vector2 lf2rf_vec( tgt.P[rf].p(X)-pre_cont_lfpos(X), tgt.P[rf].p(Y)-pre_cont_lfpos(Y) );
        if(lf2rf_vec.norm() > MAX_FW){
          tgt.P[rf].p(X) = (lf2rf_vec.normalized())(X) * MAX_FW + pre_cont_lfpos(X);
          tgt.P[rf].p(Y) = (lf2rf_vec.normalized())(Y) * MAX_FW + pre_cont_lfpos(Y);
        }
        LIMIT_MAX(tgt.P[rf].p(Y), pre_cont_lfpos(Y) - FOOT_2_FOOT_COLLISION_MARGIIN);
      }
      else if(is_rf_contact && !is_lf_contact){//左足浮遊時
        const hrp::Vector2 rf2lf_vec( tgt.P[lf].p(X)-pre_cont_rfpos(X), tgt.P[lf].p(Y)-pre_cont_rfpos(Y) );
        if(rf2lf_vec.norm() > MAX_FW){
          tgt.P[lf].p(X) = (rf2lf_vec.normalized())(X) * MAX_FW + pre_cont_rfpos(X);
          tgt.P[lf].p(Y) = (rf2lf_vec.normalized())(Y) * MAX_FW + pre_cont_rfpos(Y);
        }
        LIMIT_MIN(tgt.P[lf].p(Y), pre_cont_rfpos(Y) + FOOT_2_FOOT_COLLISION_MARGIIN);
      }
      hrp::Vector3 init_base2rh = tgt.P[rh].p_offs - baselinkpose.p_offs;
      hrp::Vector3 init_base2lh = tgt.P[lh].p_offs - baselinkpose.p_offs;
      hrp::Matrix33 init_base2rh_R = hrp::rotFromRpy(baselinkpose.rpy_offs).transpose() * hrp::rotFromRpy(tgt.P[rh].rpy_offs);
      hrp::Matrix33 init_base2lh_R = hrp::rotFromRpy(baselinkpose.rpy_offs).transpose() * hrp::rotFromRpy(tgt.P[lh].rpy_offs);
      hrp::Vector3 hand_ulimit = hrp::Vector3(0.1,0.1,0.1);
      hrp::Vector3 hand_llimit = hrp::Vector3(-0.1,-0.1,-0.1);
      if(!WBMSparam.use_rh){
        tgt.P[rh].rpy = hrp::rpyFromRot( hrp::rotFromRpy(baselinkpose.rpy) * init_base2rh_R);
        tgt.P[rh].p = baselinkpose.p + hrp::rotFromRpy(tgt.P[rh].rpy) * init_base2rh;
      }
      if(!WBMSparam.use_lh){
        tgt.P[lh].rpy = hrp::rpyFromRot( hrp::rotFromRpy(baselinkpose.rpy) * init_base2lh_R);
        tgt.P[lh].p = baselinkpose.p + hrp::rotFromRpy(tgt.P[lh].rpy) * init_base2lh;
      }
      for(int i=0;i<3;i++){
        LIMIT_MINMAX( tgt.P[rh].p(i), (baselinkpose.p+init_base2rh+hand_llimit)(i), (baselinkpose.p+init_base2rh+hand_ulimit)(i) );
        LIMIT_MINMAX( tgt.P[lh].p(i), (baselinkpose.p+init_base2lh+hand_llimit)(i), (baselinkpose.p+init_base2lh+hand_ulimit)(i) );
      }
      LIMIT_MINMAX( tgt.P[rf].p(Z), tgt.P[rf].p_offs(Z), tgt.P[rf].p_offs(Z)+WBMSparam.swing_foot_max_height);
      LIMIT_MINMAX( tgt.P[lf].p(Z), tgt.P[lf].p_offs(Z), tgt.P[lf].p_offs(Z)+WBMSparam.swing_foot_max_height);

      for(int i=0;i<XYZ;i++){ LIMIT_MINMAX( tgt.P[com].rpy(i), rc.ee_rot_limit[com][MIN](i), rc.ee_rot_limit[com][MAX](i) ); }
      LIMIT_MINMAX( tgt.P[com].p(Z), tgt.P[com].p_offs(Z) - 0.15, tgt.P[com].p_offs(Z) + 0.03 );//COM高さ方向の制限
      const int ns[] = {rf,lf,rh,lh,head};
      for(int i=0;i<sizeof(ns)/sizeof(int);i++){
        for(int j=0;j<XYZ;j++){ LIMIT_MINMAX( tgt.P[ns[i]].rpy(j), rc.ee_rot_limit[ns[i]][MIN](j) + tgt.P[com].rpy(j), rc.ee_rot_limit[ns[i]][MAX](j) + tgt.P[com].rpy(j) ); }
      }
      if(!WBMSparam.use_head)tgt.P[head].rpy = hrp::Vector3::Zero();
      if(!isHumanSyncOn())tgt.P[head].rpy = hrp::Vector3::Zero();
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
      if (loop==0)com_ans_old = com_in;
      hrp::Vector3 com_vel = (com_in - com_ans_old)/DT;
      hull_dcp.clear();
      hull_acp.clear();
      hrp::Vector4 marginDelta_for_dcp(+0.001,-0.001,0.001,-0.001);
//      hrp::Vector4 marginDelta_for_acp(+0.005,-0.005,0.005,-0.005);
      hrp::Vector4 marginDelta_for_acp(+0.01,-0.01,0.01,-0.01);
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
//      hrp::Vector2 cp_dec_tmp = com_pos + com_vel * sqrt( H_cur / G );
//      hrp::Vector2 cp_dec_ragulated;
//      if(!isPointInHullOpenCV(cp_dec_tmp,hull_d)){
//        calcCrossPointOnHull(com_pos, cp_dec_tmp, hull_d, cp_dec_ragulated);
//        com_vel_decel_ok = (cp_dec_ragulated - com_pos) / sqrt( H_cur / G );
//      }

      //減速CP条件2(指令のCPに対する現在発揮可能なCP)
      hrp::Vector2 cp_dec_ref = com_forcp_ref + com_vel_forcp_ref * sqrt( H_cur / G );
      hrp::Vector2 cp_dec_ref_ragulated = cp_dec_ref;
      if(!isPointInHullOpenCV(cp_dec_ref,hull_d)){
        calcCrossPointOnHull(com_pos, cp_dec_ref, hull_d, cp_dec_ref_ragulated);
      }
      com_vel_decel_ok = (cp_dec_ref_ragulated - com_pos) / sqrt( H_cur / G );

//      //加速CP条件(ACP使用)
//      hrp::Vector2 cp_acc_tmp = com_pos - com_vel * sqrt( H_cur / G );
//      hrp::Vector2 cp_acc_ragulated;
//      if(!isPointInHullOpenCV(cp_acc_tmp,hull_a)){
//         calcCrossPointOnHull(com_pos, cp_acc_tmp, hull_a, cp_acc_ragulated);
//         com_vel_accel_ok = (-cp_acc_ragulated + com_pos ) / sqrt( H_cur / G );
//      }

      //加速CP条件2(ZMP使用)
      static hrp::Vector2 com_vel_ans_old = com_vel;
      hrp::Vector2 com_acc = (com_vel - com_vel_ans_old) / DT;
      hrp::Vector2 zmp_in = com_pos - com_acc / G * H_cur;
      hrp::Vector2 zmp_regulated;
      if(!isPointInHullOpenCV(zmp_in,hull_a)){
        calcCrossPointOnHull(com_pos, zmp_in, hull_a, zmp_regulated);
        hrp::Vector2 com_acc_regulated = (com_pos - zmp_regulated)*G/H_cur;
        com_vel_accel_ok = com_vel_ans_old + com_acc_regulated*DT;
      }

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
    void calcWorldZMP(const hrp::Vector3& rfpos, const hrp::Vector3& lfpos, const Wrench6& rfwin, const Wrench6& lfwin, hrp::Vector3& zmp_ans);
    void calcXYMarginToHull(const hrp::Vector2& check_point, const std::vector<hrp::Vector2>& hull, hrp::Vector4& margin_ans);
    bool calcCrossPointOnHull(const hrp::Vector2& pt_in_start, const hrp::Vector2& pt_out_goal, const std::vector<hrp::Vector2>& hull, hrp::Vector2& pt_will_cross);
    double calcNearestPointOnHull(const hrp::Vector2& tgt_pt, const std::vector<hrp::Vector2>& hull, hrp::Vector2& pt_ans);
    void makeConvexHullOpenCV(const std::vector<hrp::Vector2>& pts, std::vector<hrp::Vector2>& hull_ans);
    void makeConvexHullQHull(const std::vector<hrp::Vector2>& pts, std::vector<hrp::Vector2>& hull_ans);
    bool isPointInHullOpenCV(const hrp::Vector2& pt, const std::vector<hrp::Vector2>& hull);
    void applyZMPCalcFromCOM(const hrp::Vector3& comin, hrp::Vector3& zmpout);
//    void applyVelLimit(const HumanPose& in, HumanPose& out_old, HumanPose& out);
    void applyCOMZMPXYZLock(HumanPose& tgt);
};

#endif // HUMANMASTERSLAVE_H
