// -*- C++ -*-
/*!
 * @file  AutoBalancer.h
 * @brief autobalancer component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef AUTOBALANCER_H
#define AUTOBALANCER_H

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <hrpModel/Body.h>
#include "../ImpedanceController/JointPathEx.h"
#include "../ImpedanceController/RatsMatrix.h"
#include "GaitGenerator.h"
// Service implementation headers
// <rtc-template block="service_impl_h">
#include "AutoBalancerService_impl.h"
#include "interpolator.h"
#include "../TorqueFilter/IIRFilter.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <hrpUtil/Eigen4d.h>

#if defined(__cplusplus)
extern "C"
{
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

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

//実機だとログ出力で落ちる？getenv("HOME")ダメ？
#define DEBUG 0


class BiquadIIRFilterVec
{
private:
    IIRFilter x_filters,y_filters,z_filters;
    hrp::Vector3 ans;
    std::vector<double> fb_coeffs, ff_coeffs;
public:
    BiquadIIRFilterVec(const std::string& error_prefix = ""){
    };
    ~BiquadIIRFilterVec() {};
    bool setParameter(const double fc_in, const double HZ){
      double fc = tan(fc_in * M_PI / HZ) / (2 * M_PI);
      double denom = 1 + (2 * sqrt(2) * M_PI * fc) + 4 * M_PI * M_PI * fc*fc;
      ff_coeffs.resize(3);
      fb_coeffs.resize(3);
      ff_coeffs[0] = (4 * M_PI * M_PI * fc*fc) / denom;
      ff_coeffs[1] = (8 * M_PI * M_PI * fc*fc) / denom;
      ff_coeffs[2] = (4 * M_PI * M_PI * fc*fc) / denom;
      fb_coeffs[0] = 1.0;
      fb_coeffs[1] = (8 * M_PI * M_PI * fc*fc - 2) / denom;
      fb_coeffs[2] = (1 - (2 * sqrt(2) * M_PI * fc) + 4 * M_PI * M_PI * fc*fc) / denom;
      return (
        x_filters.setParameter(2,fb_coeffs,ff_coeffs) &&
        y_filters.setParameter(2,fb_coeffs,ff_coeffs) &&
        z_filters.setParameter(2,fb_coeffs,ff_coeffs)
      );
    };
    hrp::Vector3 passFilter(const hrp::Vector3& input){
      ans(0) = x_filters.passFilter(input(0));
      ans(1) = y_filters.passFilter(input(1));
      ans(2) = z_filters.passFilter(input(2));
      return ans;
    };
};

class margin_t{
  public:
    hrp::Vector3 pt;
    double x_u, x_l, y_u, y_l;
    margin_t(){pt = hrp::Vector3::Zero(); x_u = x_l = y_u = y_l = 0;}
    bool isRagulated(){return (x_u == 0 || x_l == 0 || y_u == 0 || y_l == 0);}
};



class HumanPose{
  private: hrp::Vector3 zero;
  public:
    hrp::Vector3 com,rf,lf,rh,lh;
    hrp::Vector3 zmp;
    int idsize;
    typedef struct{ hrp::Vector3 f; hrp::Vector3 t; }Wrench6;
    Wrench6 rfw,lfw;
    hrp::Vector3 neck;
    HumanPose() : idsize(10){
      clear();
    }
    ~HumanPose(){cerr<<"HumanPose destructed"<<endl;}
    void clear(){
      for(int i=0;i<idsize;i++){ Seq(i) = hrp::Vector3::Zero(); }
    }
    hrp::Vector3& Seq(const int i){
      switch(i){
        case 0: return com;
        case 1: return rf;
        case 2: return lf;
        case 3: return rh;
        case 4: return lh;
        case 5: return zmp;
        case 6: return rfw.f;
        case 7: return rfw.t;
        case 8: return lfw.f;
        case 9: return lfw.t;
        default: std::cerr <<"[WARN] HumanPose::getSeq(int) invalid index"<<std::endl;return zero;
      }
    }
    const hrp::Vector3& Seq(const int i) const{//const指定オブジェクトからオーバーロードする時のためのオーバーライド
      switch(i){
        case 0: return com;
        case 1: return rf;
        case 2: return lf;
        case 3: return rh;
        case 4: return lh;
        case 5: return zmp;
        case 6: return rfw.f;
        case 7: return rfw.t;
        case 8: return lfw.f;
        case 9: return lfw.t;
        default: std::cerr <<"[WARN] HumanPose::getSeq(int) invalid index"<<std::endl;return zero;
      }
    }
    static void hp_printf(HumanPose& in){
      const std::string cap[] = {"c","rf","lf","rh","lh","z","rw","","lw",""};
      for(int i=0;i<6;i++){ printf("\x1b[31m%s\x1b[39m%+05.2f %+05.2f %+05.2f ",cap[i].c_str(),in.Seq(i)(0),in.Seq(i)(1),in.Seq(i)(2)); }
      for(int i=6;i<10;i++){ printf("\x1b[31m%s\x1b[39m%+05.1f %+05.1f %+05.1f ",cap[i].c_str(),in.Seq(i)(0),in.Seq(i)(1),in.Seq(i)(2)); }
      printf("\n");
    }
    void print(){ hp_printf(*this); }
};



class HumanSynchronizer{
  private:
    double FUP_TIME;
    int cur_rfup_level,cur_lfup_level;
    double cur_rfup_rad,cur_lfup_rad;
    double FUP_HIGHT;
    double CNT_F_TH;
    hrp::Vector3 pre_cont_rfpos,pre_cont_lfpos;
    hrp::Vector3 init_hp_calibcom;
    double MAXVEL,MAXACC;
    HumanPose hp_wld_raw;
    HumanPose hp_wld_initpos;
    HumanPose hp_rel_raw;
    HumanPose rp_ref_out_old;
    int countdown_num;
    bool HumanSyncOn;
    struct timeval t_calc_start, t_calc_end;
    double h2r_ratio, tgt_h2r_ratio;
    double cmmr, tgt_cmmr;
    double HZ,DT,G;
    hrp::Vector3 com_old,com_oldold,comacc;
    hrp::Vector3 comacc_ref;
    std::vector<BiquadIIRFilterVec> iir_v_filters;
    BiquadIIRFilterVec calcacc_v_filters, acc4zmp_v_filters;
    BiquadIIRFilterVec cam_rpy_filter;
    hrp::Vector3 r_zmp_raw;
    hrp::Vector4 rf_safe_region,lf_safe_region;//前後左右の順

  public:
    double tgt_FUP_TIME;
    bool startCountdownForHumanSync;
    bool ht_first_call;
    bool is_rf_contact,is_lf_contact;//足上げ高さ0に到達しているか否か
    bool go_rf_landing,go_lf_landing;//足上げ指令の有無
    HumanPose rp_wld_initpos;//IKでのリンク原点の位置(足首，手首など)
    bool use_x,use_y,use_z;
    HumanPose rp_ref_out;
    FILE *sr_log,*cz_log;
    bool use_rh,use_lh;
    hrp::Vector3 init_wld_rp_rfeepos,init_wld_rp_lfeepos;
    hrp::Vector3 init_wld_hp_rfpos,init_wld_hp_lfpos;
    hrp::Vector3 current_basepos,init_basepos;
    hrp::Vector3 com_vel_old;
    unsigned int loop;
    hrp::Vector3 cam_pos,cam_rpy,cam_pos_filtered,cam_rpy_filtered;


    interpolator *init_zmp_com_offset_interpolator;
    double zmp_com_offset_ip_ratio;

    HumanSynchronizer(){
      tgt_h2r_ratio = h2r_ratio = 0.96;//human 1.1m vs jaxon 1.06m
//      tgt_h2r_ratio = h2r_ratio = 0.62;//human 1.1m vs chidori 0.69m
//      tgt_h2r_ratio = h2r_ratio = 0.69;//human 1.0(with heavy foot sensor) vs chidori 0.69
//      tgt_h2r_ratio = h2r_ratio = 1.06;//human 1.0(with heavy foot sensor) vs jaxon 1.06
      HZ = 500;
      DT = 1.0/HZ;
      G = 9.80665;
      tgt_cmmr = cmmr = 1.0;
      use_x = use_y = use_z = true;
      cur_rfup_level = cur_lfup_level = 0;
      cur_rfup_rad = cur_lfup_rad = 0;
      is_rf_contact = is_lf_contact = true;
      go_rf_landing = go_lf_landing = false;
      pre_cont_rfpos = pre_cont_lfpos = hrp::Vector3::Zero();
      init_hp_calibcom = hrp::Vector3::Zero();
      tgt_FUP_TIME = FUP_TIME = 0.4;
      FUP_HIGHT = 0.05;
      CNT_F_TH = 20.0;
      MAXVEL = 0.004;
      HumanSyncOn = false;
      ht_first_call = true;
      startCountdownForHumanSync = false;
      countdown_num = 5*HZ;
      loop = 0;

      //////////  hrp::Vector は初期化必要  ///////////
      com_old = com_oldold = comacc = hrp::Vector3::Zero();
      comacc_ref = hrp::Vector3::Zero();
      r_zmp_raw = hrp::Vector3::Zero();

      init_wld_rp_rfeepos = init_wld_rp_lfeepos = hrp::Vector3::Zero();
      init_wld_hp_rfpos = init_wld_hp_lfpos = hrp::Vector3::Zero();
      current_basepos = init_basepos = hrp::Vector3::Zero();

      com_vel_old = hrp::Vector3::Zero();
      cam_pos = cam_rpy = cam_pos_filtered = cam_rpy_filtered = hrp::Vector3::Zero();

      iir_v_filters.resize(5);
      for(int i=0;i<iir_v_filters.size();i++)iir_v_filters[i].setParameter(0.5,HZ);//四肢拘束点用
      iir_v_filters[0].setParameter(0.4,HZ);//重心用
      calcacc_v_filters.setParameter(5,HZ);//加速度計算用
      acc4zmp_v_filters.setParameter(1,HZ);//ZMP生成用ほぼこの値でいい
      cam_rpy_filter.setParameter(1,HZ);//カメラアングル

      rf_safe_region = hrp::Vector4(0.02, -0.01,  0.02,  0.01);
      lf_safe_region = hrp::Vector4(0.02, -0.01, -0.01, -0.02);
//      rf_safe_region = hrp::Vector4(0.02, -0.01,  0.02,  0.0);
//      lf_safe_region = hrp::Vector4(0.02, -0.01, -0.0, -0.02);

      use_rh = use_lh = false;

      rp_ref_out_old.clear();
      rp_ref_out.clear();

      init_zmp_com_offset_interpolator = new interpolator(1, DT, interpolator::HOFFARBIB, 1);
      init_zmp_com_offset_interpolator->setName("zmp_com_offset_interpolator");
      double start_ratio = 0.0;
      init_zmp_com_offset_interpolator->set(&start_ratio);
      double goal_ratio = 1.0;
      init_zmp_com_offset_interpolator->go(&goal_ratio, 3.0, true);//3s

      if(DEBUG){
//        std::string home_path(std::getenv("HOME"));
//        sr_log = fopen((home_path+"/HumanSync_support_region.log").c_str(),"w+");
//        cz_log = fopen((home_path+"/HumanSync_com_zmp.log").c_str(),"w+");
        sr_log = fopen("/home/ishiguro/HumanSync_support_region.log","w+");
        cz_log = fopen("/home/ishiguro/HumanSync_com_zmp.log","w+");
      }
      cout<<"HumanSynchronizer constructed"<<endl;
    }
    ~HumanSynchronizer(){
      if(DEBUG)fclose(sr_log);
      if(DEBUG)fclose(cz_log);
      delete init_zmp_com_offset_interpolator;
      cout<<"HumanSynchronizer destructed"<<endl;
    }

    void readInput(const HumanPose& raw_in){ hp_wld_raw = raw_in; }
    void setInitOffsetPose(){ hp_wld_initpos = hp_wld_raw; }
    const int getRemainingCountDown() const { return countdown_num; }
    const bool isHumanSyncOn() const { return HumanSyncOn; }
    const double getUpdateTime() const { return (double)(t_calc_end.tv_sec - t_calc_start.tv_sec) + (t_calc_end.tv_usec - t_calc_start.tv_usec)/1000000.0; }
    void setTargetHumanToRobotRatio(const double tgt_h2r_r_in){ tgt_h2r_ratio = tgt_h2r_r_in; }
    void setTargetCOMMoveModRatio(const double tgt_mod_ratio){ tgt_cmmr = tgt_mod_ratio; }
    void updateCountDown(){
      if(countdown_num>0){
        countdown_num--;
      }else if(countdown_num==0){
        HumanSyncOn = true;
        startCountdownForHumanSync = false;
      }
    }
    void update(){//////////  メインループ  ////////////
      gettimeofday(&t_calc_start, NULL);
      updateHumanToRobotRatio             (tgt_h2r_ratio);
      updateCOMModRatio                   (tgt_cmmr);
      removeInitOffsetPose                (hp_wld_raw, hp_wld_initpos, hp_rel_raw);
      applyInitHumanZMPToCOMOffset        (hp_rel_raw, init_hp_calibcom, hp_rel_raw);
//      lockFootXYOnContact                 (go_rf_landing, go_lf_landing, hp_rel_raw);//根本から改変すべき2
      calcWorldZMP                        ((hp_rel_raw.rf+init_wld_hp_rfpos), (hp_rel_raw.lf+init_wld_hp_lfpos), hp_rel_raw.rfw, hp_rel_raw.lfw, hp_rel_raw.zmp);//足の位置はworldにしないと・・・
      convertRelHumanPoseToRelRobotPose   (hp_rel_raw, rp_ref_out);
      applyCOMMoveModRatio                (rp_ref_out);
      judgeFootLandOnCommand              (hp_wld_raw.rfw, hp_wld_raw.lfw, go_rf_landing, go_lf_landing);//fwはすでに1000->100Hzにフィルタリングされている
      lockSwingFootIfZMPOutOfSupportFoot  (rp_ref_out_old, go_rf_landing, go_lf_landing);//ここ

      applyEEWorkspaceLimit               (rp_ref_out);
      lockFootXYOnContact                 (go_rf_landing, go_lf_landing, rp_ref_out);//根本から改変すべき3

      applyCOMToSupportRegionLimit        (rp_ref_out.rf+init_wld_rp_rfeepos, rp_ref_out.lf+init_wld_rp_lfeepos, rp_ref_out.com);
      applyLPFilter                       (rp_ref_out);
      applyCOMStateLimitByCapturePoint    (rp_ref_out.com, rp_ref_out_old.com, com_vel_old, rp_ref_out.rf+init_wld_rp_rfeepos, rp_ref_out.lf+init_wld_rp_lfeepos, rp_ref_out.com);

      if(cam_rpy(1) > 20*M_PI/180)cam_rpy(1) = 20*M_PI/180;
      if(cam_rpy(1) < -20*M_PI/180)cam_rpy(1) = -20*M_PI/180;
      if(cam_rpy(2) > 20*M_PI/180)cam_rpy(2) = 20*M_PI/180;
      if(cam_rpy(2) < -20*M_PI/180)cam_rpy(2) = -20*M_PI/180;
      cam_rpy_filtered = cam_rpy_filter.passFilter(cam_rpy);

      r_zmp_raw = rp_ref_out.zmp;
      applyZMPCalcFromCOM                 (rp_ref_out.com,rp_ref_out.zmp);
//      lockSwingFootIfZMPOutOfSupportFoot  (rp_ref_out, go_rf_landing, go_lf_landing);//ここじゃ判定遅い
      applyVelLimit                       (rp_ref_out, rp_ref_out_old, rp_ref_out);
      applyCOMZMPXYZLock                  (rp_ref_out);
      overwriteFootZFromFootLandOnCommand (go_rf_landing, go_lf_landing, rp_ref_out);
      com_vel_old = (rp_ref_out.com - rp_ref_out_old.com)/DT;
      rp_ref_out_old = rp_ref_out;
      loop++;
      gettimeofday(&t_calc_end, NULL);
    }
    void calibInitHumanCOMFromZMP(){
      init_hp_calibcom(0) = r_zmp_raw(0);
      init_hp_calibcom(1) = r_zmp_raw(1);
    }
    void calcWorldZMP(const hrp::Vector3& rfpos, const hrp::Vector3& lfpos, const HumanPose::Wrench6& rfwin, const HumanPose::Wrench6& lfwin, hrp::Vector3& zmp_ans){
      hrp::Vector3 rfzmp,lfzmp;
      const double F_H_OFFSET = 0.03;//地面から6軸センサ原点への高さ
      if( rfwin.f(2) > 1.0e-6 ){
        rfzmp(0) = ( - rfwin.t(1) - rfwin.f(0) * F_H_OFFSET + rfwin.f(2) * 0 ) / rfwin.f(2) + rfpos(0);
        rfzmp(1) = (   rfwin.t(0) - rfwin.f(1) * F_H_OFFSET + rfwin.f(2) * 0 ) / rfwin.f(2) + rfpos(1);
      }
      if( lfwin.f(2) > 1.0e-6 ){
        lfzmp(0) = ( - lfwin.t(1) - lfwin.f(0) * F_H_OFFSET + lfwin.f(2) * 0 ) / lfwin.f(2) + lfpos(0);
        lfzmp(1) = (   lfwin.t(0) - lfwin.f(1) * F_H_OFFSET + lfwin.f(2) * 0 ) / lfwin.f(2) + lfpos(1);
      }
      if( rfwin.f(2) > 1.0e-6 || lfwin.f(2) > 1.0e-6 ){
        zmp_ans(0) = ( rfzmp(0)*rfwin.f(2) + lfzmp(0)*lfwin.f(2) ) / ( rfwin.f(2) + lfwin.f(2));
        zmp_ans(1) = ( rfzmp(1)*rfwin.f(2) + lfzmp(1)*lfwin.f(2) ) / ( rfwin.f(2) + lfwin.f(2));
      }else{ zmp_ans(0) = 0; zmp_ans(1) = 0; }
      zmp_ans(2) = 0;
    }
    static void Point3DToVector3(const RTC::Point3D& in, hrp::Vector3& out){ out(0) = in.x; out(1) = in.y; out(2) = in.z;}
    static void Oriantation3DToVector3(const RTC::Orientation3D& in, hrp::Vector3& out){ out(0) = in.r; out(1) = in.p; out(2) = in.y;}
    static void Pose3DToVector3(const RTC::Pose3D& in, hrp::Vector3& out_pos, hrp::Vector3& out_rpy){ Point3DToVector3(in.position,out_pos); Oriantation3DToVector3(in.orientation,out_rpy); }
    static void DoubleSeqToWrench6(const RTC::TimedDoubleSeq::_data_seq& in, HumanPose::Wrench6& out){
      if(in.length() == 6){out.f(0)=in[0]; out.f(1)=in[1]; out.f(2)=in[2]; out.t(0)=in[3]; out.t(1)=in[4]; out.t(2)=in[5];
      }else{ std::cerr<<"[WARN] HumanPose::Wrench6 DoubleSeqToWrench6() invalid data length"<<std::endl; out.f.Zero(); out.t.Zero(); }
    }
    static double hrpVector2Cross(const hrp::Vector2& a, const hrp::Vector2& b){ return a(0)*b(1)-a(1)*b(0); }

  private:
    void updateHumanToRobotRatio(const double h2r_r_goal){//h2r_ratioに伴って変わる変数の処理もここに書く
      const double step = 0.001;
      if(h2r_r_goal - h2r_ratio > step){h2r_ratio += step; cout<<"Interpolating HumanToRobotRatio as "<<h2r_ratio<<endl;}
      else if(h2r_r_goal - h2r_ratio < (-1)*step){h2r_ratio -= step; cout<<"Interpolating HumanToRobotRatio as "<<h2r_ratio<<endl;}
      else{h2r_ratio = h2r_r_goal;}
      init_wld_hp_rfpos = init_wld_rp_rfeepos/h2r_ratio;
      init_wld_hp_lfpos = init_wld_rp_lfeepos/h2r_ratio;
    }
    void updateCOMModRatio(const double cmmr_goal){
      const double step = 0.001;
      if(cmmr_goal - cmmr > step){cmmr += step; cout<<"Interpolating COMMoveModRatio as "<<cmmr<<endl;}
      else if(cmmr_goal - cmmr < (-1)*step){cmmr -= step; cout<<"Interpolating COMMoveModRatio as "<<cmmr<<endl;}
      else{cmmr = cmmr_goal;}
    };
    void removeInitOffsetPose(const HumanPose& abs_in, const HumanPose& init_offset, HumanPose& rel_out){
      for(int i=0;i<rel_out.idsize-5;i++){
        rel_out.Seq(i) =  abs_in.Seq(i) - init_offset.Seq(i);
      }
      rel_out.rfw = abs_in.rfw;
      rel_out.lfw = abs_in.lfw;
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
      out.com = in.com + zmp_com_offset_ip_ratio * com_offs;
    }
    void applyLPFilter(HumanPose& tgt){
      for(int i=0;i<iir_v_filters.size();i++){ tgt.Seq(i) = iir_v_filters[i].passFilter(tgt.Seq(i)); }
    }
    void applyLPFilter_o(const HumanPose& raw_in, const HumanPose& out_old, HumanPose& filt_out){
      double FNUM = 0.01;//0.01でも荒い?
      for(int i=0;i<filt_out.idsize;i++){
        filt_out.Seq(i) = (1-FNUM) * out_old.Seq(i) + FNUM * raw_in.Seq(i);
      }
    }
    void convertRelHumanPoseToRelRobotPose(const HumanPose& hp_in, HumanPose& rp_out){//結局初期指定からの移動量(=Rel)で計算をしてゆく
      for(int i=0;i<rp_out.idsize-4;i++){
        rp_out.Seq(i) =  h2r_ratio * hp_in.Seq(i);
      }
      rp_out.rfw  = hp_in.rfw;
      rp_out.lfw  = hp_in.lfw;
    }
    void applyCOMMoveModRatio(HumanPose& tgt){//
      tgt.com = cmmr * tgt.com;
    }
    void judgeFootLandOnCommand(const HumanPose::Wrench6& rfw_in, const HumanPose::Wrench6& lfw_in, bool& rfloc_ans, bool& lfloc_ans){
      if(HumanSyncOn){
        if      (rfloc_ans  && rfw_in.f(2)<CNT_F_TH   ){rfloc_ans = false;}//右足ついた状態から上げる
        else if (!rfloc_ans && rfw_in.f(2)>CNT_F_TH+30){rfloc_ans = true;}//右足浮いた状態から下げる
        if      (lfloc_ans  && lfw_in.f(2)<CNT_F_TH   ){lfloc_ans = false;}//左足ついた状態から上げる
        else if (!lfloc_ans && lfw_in.f(2)>CNT_F_TH+30){lfloc_ans = true;}//左足浮いた状態から下げる
      }
      if(!rfloc_ans && !lfloc_ans){rfloc_ans = lfloc_ans = true;}//両足ジャンプは禁止
    }
    void lockSwingFootIfZMPOutOfSupportFoot(const HumanPose& tgt, bool& rfcs_ans, bool& lfcs_ans){
      if(rfcs_ans && !lfcs_ans){
        if(tgt.zmp(1) > tgt.rf(1) + init_wld_rp_rfeepos(1) + 0.02){
          lfcs_ans = true;
          if(loop%100==0)cout<<"LF Auto lock"<<endl;
        }
      }
      if(!rfcs_ans && lfcs_ans){
        if(tgt.zmp(1) < tgt.lf(1) + init_wld_rp_lfeepos(1) - 0.02){
          rfcs_ans = true;
          if(loop%100==0)cout<<"RF Auto lock"<<endl;
        }
      }
    }
    void overwriteFootZFromFootLandOnCommand(const bool& rf_goland_in, const bool& lf_goland_in, HumanPose& tgt){
      if(is_rf_contact&&is_lf_contact)FUP_TIME = tgt_FUP_TIME;//両足接地しているタイミングでのみ更新
      double RFDOWN_TIME = fabs(tgt.rf(1) - tgt.com(1))*4 + FUP_TIME;//下ろす速度は遊脚の重心からの距離に比例したペナルティ
      double LFDOWN_TIME = fabs(tgt.lf(1) - tgt.com(1))*4 + FUP_TIME;
      calcFootUpCurveAndJudgeFootContact2(rf_goland_in, FUP_TIME, RFDOWN_TIME, cur_rfup_rad, tgt.rf, is_rf_contact);
      calcFootUpCurveAndJudgeFootContact2(lf_goland_in, FUP_TIME, LFDOWN_TIME, cur_lfup_rad, tgt.lf, is_lf_contact);
    }
    void calcFootUpCurveAndJudgeFootContact(const bool& go_land_in, int& cur_fup_count_in, hrp::Vector3& f_height_out, bool& is_f_contact_out){
      int FUP_COUNT = (int)(HZ*FUP_TIME);
      if(cur_fup_count_in > FUP_COUNT){cur_fup_count_in = FUP_COUNT;} if(cur_fup_count_in < 0){cur_fup_count_in = 0;}
      if(!go_land_in){//足上げ命令入力時
        if(cur_fup_count_in < FUP_COUNT){ cur_fup_count_in++; }
      }else{//足下げ命令入力時
        if(cur_fup_count_in > 0){ cur_fup_count_in--; }
      }
      f_height_out(2) = FUP_HIGHT/2*(1-cos(M_PI*cur_fup_count_in/FUP_COUNT));
      is_f_contact_out = (cur_fup_count_in <= 0);
    }
    void calcFootUpCurveAndJudgeFootContact2(const bool& go_land_in, const double& fup_time_in, const double& fdown_time_in,  double& cur_fup_phase_in,hrp::Vector3& f_height_out, bool& is_f_contact_out){
      const double omega_up = M_PI / fup_time_in;
      const double omega_down = M_PI / fdown_time_in;
      if(!go_land_in){//足上げ命令入力時
        cur_fup_phase_in += omega_up * DT;
      }else{//足下げ命令入力時
        cur_fup_phase_in -= omega_down * DT;
      }
      if(cur_fup_phase_in > M_PI){cur_fup_phase_in = M_PI;}
      else if(cur_fup_phase_in < 0){cur_fup_phase_in = 0;}
      f_height_out(2) = FUP_HIGHT/2*(1-cos(cur_fup_phase_in));
      is_f_contact_out = (cur_fup_phase_in <= 0);
    }
    void lockFootXYOnContact(const bool& rfcs_in, const bool& lfcs_in, HumanPose& tgt){
      if(rfcs_in){
        tgt.rf(0) = pre_cont_rfpos(0);
        tgt.rf(1) = pre_cont_rfpos(1);
      }else{
        pre_cont_rfpos(0) = tgt.rf(0);
        pre_cont_rfpos(1) = tgt.rf(1);
      }
      if(lfcs_in){
        tgt.lf(0) = pre_cont_lfpos(0);
        tgt.lf(1) = pre_cont_lfpos(1);
      }else{
        pre_cont_lfpos(0) = tgt.lf(0);
        pre_cont_lfpos(1) = tgt.lf(1);
      }
    }
    void applyEEWorkspaceLimit(HumanPose& tgt){
      const double MAX = 0.15;
      if(!is_rf_contact && is_lf_contact){//右足浮遊時
        const hrp::Vector2 lf2rf_vec( tgt.rf(0)-pre_cont_lfpos(0), tgt.rf(1)-pre_cont_lfpos(1) );
        if(lf2rf_vec.norm() > MAX){
          tgt.rf(0) = (lf2rf_vec.normalized())(0) * MAX + pre_cont_lfpos(0);
          tgt.rf(1) = (lf2rf_vec.normalized())(1) * MAX + pre_cont_lfpos(1);
        }
        if(tgt.rf(1) > pre_cont_lfpos(1) + 0.05 ) tgt.rf(1) = pre_cont_lfpos(1) + 0.05;
      }
      else if(is_rf_contact && !is_lf_contact){//左足浮遊時
        const hrp::Vector2 rf2lf_vec( tgt.lf(0)-pre_cont_rfpos(0), tgt.lf(1)-pre_cont_rfpos(1) );
        if(rf2lf_vec.norm() > MAX){
          tgt.lf(0) = (rf2lf_vec.normalized())(0) * MAX + pre_cont_rfpos(0);
          tgt.lf(1) = (rf2lf_vec.normalized())(1) * MAX + pre_cont_rfpos(1);
        }
        if(tgt.lf(1) < pre_cont_rfpos(1) - 0.05 ) tgt.lf(1) = pre_cont_rfpos(1) - 0.05;
      }
      hrp::Vector3 hand_ulimit = hrp::Vector3(0.1,0.1,0.1);
      hrp::Vector3 hand_llimit = hrp::Vector3(-0.1,-0.1,-0.1);
      for(int i=0;i<3;i++){
        if      (tgt.rh(i) > hand_ulimit(i) + current_basepos(i) - init_basepos(i) ){ tgt.rh(i) = hand_ulimit(i) + current_basepos(i) - init_basepos(i); }
        else if (tgt.rh(i) < hand_llimit(i) + current_basepos(i) - init_basepos(i) ){ tgt.rh(i) = hand_llimit(i) + current_basepos(i) - init_basepos(i); }
        if      (tgt.lh(i) > hand_ulimit(i) + current_basepos(i) - init_basepos(i) ){ tgt.lh(i) = hand_ulimit(i) + current_basepos(i) - init_basepos(i); }
        else if (tgt.lh(i) < hand_llimit(i) + current_basepos(i) - init_basepos(i) ){ tgt.lh(i) = hand_llimit(i) + current_basepos(i) - init_basepos(i); }
      }
    }
    bool applyCOMToSupportRegionLimit(const hrp::Vector3& rfin_abs, const hrp::Vector3& lfin_abs, hrp::Vector3& comin_abs){//boost::geometryがUbuntu12だとないから・・・
      std::vector<hrp::Vector2> hull;
      createSupportRegionByFootPos(rfin_abs, lfin_abs, rf_safe_region, lf_safe_region, hull);
      hrp::Vector2 cog(comin_abs(0),comin_abs(1));
      if(!isPointInHullOpenCV(cog,hull)){ calcNearestPointOnHull(cog,hull,cog); }//外に出たら最近傍点に頭打ち
      comin_abs(0) = cog(0);
      comin_abs(1) = cog(1);
      if( comin_abs(2) > 0.03 ) comin_abs(2) = 0.03;
      if( comin_abs(2) < -0.15 ) comin_abs(2) = -0.15;
      return true;
    }
    bool applyCOMStateLimitByCapturePoint(const hrp::Vector3& com_in, const hrp::Vector3& com_old, const hrp::Vector3& com_vel_old, const hrp::Vector3& rfin_abs, const hrp::Vector3& lfin_abs, hrp::Vector3& com_ans){
      com_ans = com_in;
      hrp::Vector3 com_vel = (com_in - com_old)/DT;
      hrp::Vector3 com_acc = (com_vel - com_vel_old)/DT;//命名規則
      hrp::Vector3 com_vel_zmpOk, com_vel_zmpOk_cpOk;
      double accel[4] = {0.05, -0.03, 0.02, -0.02};
      double decel[4] = {0.04, -0.02, 0.01, -0.01};
      comacc_ref = calcacc_v_filters.passFilter(com_acc);
      const double H = rp_wld_initpos.com(2);
//      double maxacc[2],minacc[2];
//      maxacc[0] = (com_old(0) - accel[1]) * (G/H); if(maxacc[0] < 0){maxacc[0]=0;}
//      minacc[0] = (com_old(0) - accel[0]) * (G/H); if(minacc[0] > 0){minacc[0]=0;}
//      maxacc[1] = (com_old(1) - (accel[3]-0.1)) * (G/H); if(maxacc[1] < 0){maxacc[1]=0;}
//      minacc[1] = (com_old(1) - (accel[2]+0.1)) * (G/H); if(minacc[1] > 0){minacc[1]=0;}
//      if(com_vel(0) > maxacc[0] * DT + com_vel_old(0)){com_vel(0) = maxacc[0] * DT + com_vel_old(0); cout<<"maxacc[0] over"<<endl;}
//      if(com_vel(0) < minacc[0] * DT + com_vel_old(0)){com_vel(0) = minacc[0] * DT + com_vel_old(0); cout<<"minacc[0] over"<<endl;}
//      if(com_vel(1) > maxacc[1] * DT + com_vel_old(1)){com_vel(1) = maxacc[1] * DT + com_vel_old(1); cout<<"maxacc[1] over"<<endl;}
//      if(com_vel(1) < minacc[1] * DT + com_vel_old(1)){com_vel(1) = minacc[1] * DT + com_vel_old(1); cout<<"minacc[1] over"<<endl;}
//      if(comacc_ref(0) > maxacc[0]){com_vel(0) = maxacc[0] * DT + com_vel_old(0); cout<<"maxacc[0] over"<<endl;}
//      if(comacc_ref(0) < minacc[0]){com_vel(0) = minacc[0] * DT + com_vel_old(0); cout<<"minacc[0] over"<<endl;}
//      if(comacc_ref(1) > maxacc[1]){com_vel(1) = maxacc[1] * DT + com_vel_old(1); cout<<"maxacc[1] over"<<endl;}
//      if(comacc_ref(1) < minacc[1]){com_vel(1) = minacc[1] * DT + com_vel_old(1); cout<<"minacc[1] over"<<endl;}
//      cout<<"comacc_ref(1) "<<comacc_ref(1)<<" minacc[1] "<<minacc[1]<<endl;
////      cout<<"maxacc "<<maxacc[1]<<" minacc "<<minacc[1]<< " acc " << (com_vel(1)-com_vel_old(1))/DT <<endl;
      std::vector<hrp::Vector2> hull;
      createSupportRegionByFootPos(rfin_abs, lfin_abs, rf_safe_region, lf_safe_region, hull);
      hrp::Vector2 com_vel_ans_2d;
//      regulateCOMVelocityByCapturePointVec( hrp::Vector2(com_old(0),com_old(1)), hrp::Vector2(com_vel(0),com_vel(1)), hull, com_vel_ans_2d);
      regulateCOMVelocityByCapturePointXY( hrp::Vector2(com_old(0),com_old(1)), hrp::Vector2(com_vel(0),com_vel(1)), hull, com_vel_ans_2d);//XY独立に計算する手法のほうがエッジで引っかからない・・・
      com_ans(0) = com_old(0) + com_vel_ans_2d(0) * DT;
      com_ans(1) = com_old(1) + com_vel_ans_2d(1) * DT;
      return true;
    }
    void regulateCOMVelocityByCapturePointVec(const hrp::Vector2& com_pos, const hrp::Vector2& com_vel, const std::vector<hrp::Vector2>& hull, hrp::Vector2& com_vel_ans){
      com_vel_ans = com_vel;
      const double H = rp_wld_initpos.com(2);
      hrp::Vector2 cp = com_pos + com_vel * sqrt( H / G );
      hrp::Vector2 cp_ragulated;
      if(!isPointInHullOpenCV(cp,hull)){
        calcCrossPointOnHull(com_pos, cp, hull, cp_ragulated);
        //calcNearestPointOnHull(cp, hull, cp_ragulated);
        com_vel_ans = (cp_ragulated - com_pos) / sqrt( H / G );
      }
    }
    void regulateCOMVelocityByCapturePointXY(const hrp::Vector2& com_pos, const hrp::Vector2& com_vel, const std::vector<hrp::Vector2>& hull, hrp::Vector2& com_vel_ans){
      com_vel_ans = com_vel;
      const double H = rp_wld_initpos.com(2);
      hrp::Vector4 margin_from_4maxcp;
      calcXYMarginToHull(com_pos, hull, margin_from_4maxcp);
      hrp::Vector3 max_com_vel,min_com_vel;
      max_com_vel(0) = margin_from_4maxcp(0) / sqrt( H / G );
      min_com_vel(0) = margin_from_4maxcp(1) / sqrt( H / G );
      max_com_vel(1) = margin_from_4maxcp(2) / sqrt( H / G );
      min_com_vel(1) = margin_from_4maxcp(3) / sqrt( H / G );
      if(com_vel(0) > max_com_vel(0)){ com_vel_ans(0) = max_com_vel(0); if(loop%100==0)cout<<"F cp over"<<endl;}
      if(com_vel(0) < min_com_vel(0)){ com_vel_ans(0) = min_com_vel(0); if(loop%100==0)cout<<"B cp over"<<endl;}
      if(com_vel(1) > max_com_vel(1)){ com_vel_ans(1) = max_com_vel(1); if(loop%100==0)cout<<"L cp over"<<endl;}
      if(com_vel(1) < min_com_vel(1)){ com_vel_ans(1) = min_com_vel(1); if(loop%100==0)cout<<"R cp over"<<endl;}
    }
    void createSupportRegionByFootPos(const hrp::Vector3& rfin_abs, const hrp::Vector3& lfin_abs, const hrp::Vector4& rf_mgn, const hrp::Vector4& lf_mgn, std::vector<hrp::Vector2>& hull_ans){
      std::vector<hrp::Vector2> points;
      for(int i=0;i<2;i++){
        for(int j=2;j<4;j++){
          points.push_back(hrp::Vector2(rfin_abs(0) + rf_mgn(i),    rfin_abs(1) + rf_mgn(j)));
          points.push_back(hrp::Vector2(lfin_abs(0) + lf_mgn(i),    lfin_abs(1) + lf_mgn(j)));
        }
      }
      makeConvexHullOpenCV(points, hull_ans);
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
      std::vector<cv::Point2f> points,hull;
      for(int i=0;i<pts.size();i++){ points.push_back( cv::Point2f( pts[i](0), pts[i](1)) ); }
      cv::convexHull(cv::Mat(points),hull,true);
      hull_ans.clear();
      for(int i=0;i<hull.size();i++){ hull_ans.push_back( hrp::Vector2( hull[i].x, hull[i].y ) );  }
    }
    void makeConvexHullQHull(const std::vector<hrp::Vector2>& pts, std::vector<hrp::Vector2>& hull_ans){
      const int dim = 2;
      double points[dim*pts.size()];
      for(int i=0;i<pts.size();i++){ points[dim*i] = pts[i](0); points[dim*i+1] = pts[i](1); }
      qh_new_qhull( dim, pts.size(), points, false, "qhull ", NULL, stderr);
      pointT *point, *pointtemp;
      hull_ans.clear();
      FORALLpoints hull_ans.push_back( hrp::Vector2(point[0],point[1]) );
      qh_freeqhull(!qh_ALL);
      //#define qh_ORIENTclock 1
    }
    bool isPointInHullOpenCV(const hrp::Vector2& pt, const std::vector<hrp::Vector2>& hull){
      std::vector<cv::Point2f> cvhull;
      for(int i=0;i<hull.size();i++){ cvhull.push_back( cv::Point2f( hull[i](0), hull[i](1)) ); }
      return (cv::pointPolygonTest(cv::Mat(cvhull), cv::Point2f(pt(0),pt(1)), false) > 0);
    }
    void applyZMPCalcFromCOM(const hrp::Vector3& comin, hrp::Vector3& zmpout){
      comacc = (comin - 2 * com_old + com_oldold)/(DT*DT);
      const double MAXACC = 5;
      if(comacc(0)>MAXACC)comacc(0)=MAXACC;else if(comacc(0)<-MAXACC)comacc(0)=-MAXACC;
      if(comacc(1)>MAXACC)comacc(1)=MAXACC;else if(comacc(1)<-MAXACC)comacc(1)=-MAXACC;
      comacc = acc4zmp_v_filters.passFilter(comacc);
      zmpout(0) = comin(0)-(rp_wld_initpos.com(2)/G)*comacc(0);
      zmpout(1) = comin(1)-(rp_wld_initpos.com(2)/G)*comacc(1);
      if(DEBUG)fprintf(cz_log,"%f %f %f %f %f %f %f\n",(double)loop/HZ,comin(0),comin(1),rp_ref_out.zmp(0),rp_ref_out.zmp(1),zmpout(0),zmpout(1));
      com_oldold = com_old;
      com_old = comin;
    }
    void applyVelLimit(const HumanPose& in, const HumanPose& in_old, HumanPose& out){
      for(int i=0;i<3;i++){if(in.com(i) - in_old.com(i) > MAXVEL){out.com(i) = in_old.com(i) + MAXVEL;}else if(in.com(i) - in_old.com(i) < -MAXVEL){out.com(i) = in_old.com(i) - MAXVEL;}}
      for(int i=0;i<3;i++){if(in.rf(i)  - in_old.rf(i)  > MAXVEL){out.rf(i)  = in_old.rf(i)  + MAXVEL;}else if(in.rf(i)  - in_old.rf(i)  < -MAXVEL){out.rf(i)  = in_old.rf(i)  - MAXVEL;}}
      for(int i=0;i<3;i++){if(in.lf(i)  - in_old.lf(i)  > MAXVEL){out.lf(i)  = in_old.lf(i)  + MAXVEL;}else if(in.lf(i)  - in_old.lf(i)  < -MAXVEL){out.lf(i)  = in_old.lf(i)  - MAXVEL;}}
      for(int i=0;i<3;i++){if(in.rh(i)  - in_old.rh(i)  > MAXVEL){out.rh(i)  = in_old.rh(i)  + MAXVEL;}else if(in.rh(i)  - in_old.rh(i)  < -MAXVEL){out.rh(i)  = in_old.rh(i)  - MAXVEL;}}
      for(int i=0;i<3;i++){if(in.lh(i)  - in_old.lh(i)  > MAXVEL){out.lh(i)  = in_old.lh(i)  + MAXVEL;}else if(in.lh(i)  - in_old.lh(i)  < -MAXVEL){out.lh(i)  = in_old.lh(i)  - MAXVEL;}}
    }
    void applyCOMZMPXYZLock(HumanPose& tgt){
        if(!use_x){tgt.com(0) = 0;tgt.zmp(0) = 0;}
        if(!use_y){tgt.com(1) = 0;tgt.zmp(1) = 0;}
        if(!use_z){tgt.com(2) = 0;}
    }
};


// </rtc-template>

using namespace RTC;

class AutoBalancer
  : public RTC::DataFlowComponentBase
{
 public:
  AutoBalancer(RTC::Manager* manager);
  virtual ~AutoBalancer();

  // The initialize action (on CREATED->ALIVE transition)
  // formaer rtc_init_entry()
 virtual RTC::ReturnCode_t onInitialize();

  // The finalize action (on ALIVE->END transition)
  // formaer rtc_exiting_entry()
  virtual RTC::ReturnCode_t onFinalize();

  // The startup action when ExecutionContext startup
  // former rtc_starting_entry()
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  // The shutdown action when ExecutionContext stop
  // former rtc_stopping_entry()
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  // The activated action (Active state entry action)
  // former rtc_active_entry()
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  // The deactivated action (Active state exit action)
  // former rtc_active_exit()
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  // The execution action that is invoked periodically
  // former rtc_active_do()
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  // The aborting action when main logic error occurred.
  // former rtc_aborting_entry()
  // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  // The error action in ERROR state
  // former rtc_error_do()
  // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  // The reset action that is invoked resetting
  // This is same but different the former rtc_init_entry()
  // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);

  // The state update action that is invoked after onExecute() action
  // no corresponding operation exists in OpenRTm-aist-0.2.0
  // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  // The action that is invoked when execution context's rate is changed
  // no corresponding operation exists in OpenRTm-aist-0.2.0
  // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);
  bool goPos(const double& x, const double& y, const double& th);
  bool goVelocity(const double& vx, const double& vy, const double& vth);
  bool goStop();
  bool emergencyStop ();
  bool setFootSteps(const OpenHRP::AutoBalancerService::FootstepSequence& fs, CORBA::Long overwrite_fs_idx);
  bool setFootSteps(const OpenHRP::AutoBalancerService::FootstepsSequence& fss, CORBA::Long overwrite_fs_idx);
  bool setFootStepsWithParam(const OpenHRP::AutoBalancerService::FootstepSequence& fs, const OpenHRP::AutoBalancerService::StepParamSequence& sps, CORBA::Long overwrite_fs_idx);
  bool setFootStepsWithParam(const OpenHRP::AutoBalancerService::FootstepsSequence& fss, const OpenHRP::AutoBalancerService::StepParamsSequence& spss, CORBA::Long overwrite_fs_idx);
  void waitFootSteps();
  void waitFootStepsEarly(const double tm);
  bool startAutoBalancer(const ::OpenHRP::AutoBalancerService::StrSequence& limbs);
  bool stopAutoBalancer();
  bool setGaitGeneratorParam(const OpenHRP::AutoBalancerService::GaitGeneratorParam& i_param);
  bool getGaitGeneratorParam(OpenHRP::AutoBalancerService::GaitGeneratorParam& i_param);
  bool setAutoBalancerParam(const OpenHRP::AutoBalancerService::AutoBalancerParam& i_param);
  bool getAutoBalancerParam(OpenHRP::AutoBalancerService::AutoBalancerParam& i_param);
  bool getFootstepParam(OpenHRP::AutoBalancerService::FootstepParam& i_param);
  bool adjustFootSteps(const OpenHRP::AutoBalancerService::Footstep& rfootstep, const OpenHRP::AutoBalancerService::Footstep& lfootstep);
  bool getRemainingFootstepSequence(OpenHRP::AutoBalancerService::FootstepSequence_out o_footstep, CORBA::Long& o_current_fs_idx);
  bool getGoPosFootstepsSequence(const double& x, const double& y, const double& th, OpenHRP::AutoBalancerService::FootstepsSequence_out o_footstep);
  bool releaseEmergencyStop();
  bool startHumanSyncAfter5sec();
  bool setHumanToRobotRatio(const double h2r);
  bool setCOMMoveModRatio(const double cmr);
  bool setFootUpTime(const double fupt);
  bool setAllowedXYZSync(const bool x_on,const bool y_on,const bool z_on);
  bool stopHumanSync();

 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  
  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  TimedDoubleSeq m_qRef;
  InPort<TimedDoubleSeq> m_qRefIn;
  TimedPoint3D m_basePos;
  InPort<TimedPoint3D> m_basePosIn;
  TimedOrientation3D m_baseRpy;
  InPort<TimedOrientation3D> m_baseRpyIn;
  TimedPoint3D m_zmp;
  InPort<TimedPoint3D> m_zmpIn;
  TimedDoubleSeq m_optionalData;
  InPort<TimedDoubleSeq> m_optionalDataIn;
  std::vector<TimedDoubleSeq> m_ref_force;
  std::vector<InPort<TimedDoubleSeq> *> m_ref_forceIn;
  TimedLong m_emergencySignal;
  InPort<TimedLong> m_emergencySignalIn;
  // for debug
  TimedPoint3D m_cog;
  //for human tracker
  TimedPoint3D m_htzmp;
  InPort<TimedPoint3D> m_htzmpIn;
  TimedDoubleSeq m_htrfw;
  InPort<TimedDoubleSeq> m_htrfwIn;
  TimedDoubleSeq m_htlfw;
  InPort<TimedDoubleSeq> m_htlfwIn;
  TimedPoint3D m_htcom;
  InPort<TimedPoint3D> m_htcomIn;
  TimedPoint3D m_htrf;
  InPort<TimedPoint3D> m_htrfIn;
  TimedPoint3D m_htlf;
  InPort<TimedPoint3D> m_htlfIn;
  TimedPoint3D m_htrh;
  InPort<TimedPoint3D> m_htrhIn;
  TimedPoint3D m_htlh;
  InPort<TimedPoint3D> m_htlhIn;
  TimedPoint3D m_actzmp;
  InPort<TimedPoint3D> m_actzmpIn;
  TimedPose3D m_htcam;
  InPort<TimedPose3D> m_htcamIn;
  
  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  OutPort<TimedDoubleSeq> m_qOut;
  RTC::OutPort<RTC::TimedPoint3D> m_zmpOut;
  OutPort<TimedPoint3D> m_basePosOut;
  OutPort<TimedOrientation3D> m_baseRpyOut;
  TimedDoubleSeq m_baseTform;
  OutPort<TimedDoubleSeq> m_baseTformOut;
  TimedPose3D m_basePose;
  OutPort<TimedPose3D> m_basePoseOut;
  TimedAcceleration3D m_accRef;
  OutPort<TimedAcceleration3D> m_accRefOut;
  TimedBooleanSeq m_contactStates;
  OutPort<TimedBooleanSeq> m_contactStatesOut;
  TimedDoubleSeq m_toeheelRatio;
  OutPort<TimedDoubleSeq> m_toeheelRatioOut;
  TimedDoubleSeq m_controlSwingSupportTime;
  OutPort<TimedDoubleSeq> m_controlSwingSupportTimeOut;
  TimedBoolean m_walkingStates;
  OutPort<TimedBoolean> m_walkingStatesOut;
  TimedPoint3D m_sbpCogOffset;
  OutPort<TimedPoint3D> m_sbpCogOffsetOut;
  std::vector<TimedDoubleSeq> m_force;
  std::vector<OutPort<TimedDoubleSeq> *> m_ref_forceOut;
  std::vector<TimedPoint3D> m_limbCOPOffset;
  std::vector<OutPort<TimedPoint3D> *> m_limbCOPOffsetOut;
  // for debug
  OutPort<TimedPoint3D> m_cogOut;
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  RTC::CorbaPort m_AutoBalancerServicePort;

  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  AutoBalancerService_impl m_service0;

  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  
  // </rtc-template>

 private:
  struct ABCIKparam {
    hrp::Vector3 target_p0, localPos, adjust_interpolation_target_p0, adjust_interpolation_org_p0;
    hrp::Matrix33 target_r0, localR, adjust_interpolation_target_r0, adjust_interpolation_org_r0;
    rats::coordinates target_end_coords;
    hrp::Link* target_link;
    hrp::JointPathExPtr manip;
    double avoid_gain, reference_gain;
    size_t pos_ik_error_count, rot_ik_error_count;
    bool is_active, has_toe_joint;
  };
  void getCurrentParameters();
  void getTargetParameters();
  bool solveLimbIKforLimb (ABCIKparam& param, const std::string& limb_name);
  void solveLimbIK();
  void startABCparam(const ::OpenHRP::AutoBalancerService::StrSequence& limbs);
  void stopABCparam();
  void waitABCTransition();
  hrp::Matrix33 OrientRotationMatrix (const hrp::Matrix33& rot, const hrp::Vector3& axis1, const hrp::Vector3& axis2);
  void fixLegToCoords (const hrp::Vector3& fix_pos, const hrp::Matrix33& fix_rot);
  void startWalking ();
  void stopWalking ();
  void copyRatscoords2Footstep(OpenHRP::AutoBalancerService::Footstep& out_fs, const rats::coordinates& in_fs);
  // static balance point offsetting
  void static_balance_point_proc_one(hrp::Vector3& tmp_input_sbp, const double ref_com_height);
  void calc_static_balance_point_from_forces(hrp::Vector3& sb_point, const hrp::Vector3& tmpcog, const double ref_com_height, std::vector<hrp::Vector3>& tmp_forces);
  hrp::Vector3 calc_vel_from_hand_error (const rats::coordinates& tmp_fix_coords);
  bool isOptionalDataContact (const std::string& ee_name)
  {
      return (std::fabs(m_optionalData.data[contact_states_index_map[ee_name]]-1.0)<0.1)?true:false;
  };
  bool calc_inital_support_legs(const double& y, std::vector<rats::coordinates>& initial_support_legs_coords, std::vector<rats::leg_type>& initial_support_legs, rats::coordinates& start_ref_coords);

  // for gg
  typedef boost::shared_ptr<rats::gait_generator> ggPtr;
  ggPtr gg;
  bool gg_is_walking, gg_solved;
  // for abc
  hrp::Vector3 ref_cog, ref_zmp, prev_imu_sensor_pos, prev_imu_sensor_vel, hand_fix_initial_offset;
  enum {BIPED, TROT, PACE, CRAWL, GALLOP} gait_type;
  enum {MODE_IDLE, MODE_ABC, MODE_SYNC_TO_IDLE, MODE_SYNC_TO_ABC} control_mode, return_control_mode;
  std::map<std::string, ABCIKparam> ikp;
  std::map<std::string, size_t> contact_states_index_map;
  std::map<std::string, hrp::VirtualForceSensorParam> m_vfs;
  std::vector<std::string> sensor_names, leg_names, ee_vec;
  hrp::dvector qorg, qrefv;
  hrp::Vector3 current_root_p, target_root_p;
  hrp::Matrix33 current_root_R, target_root_R;
  rats::coordinates fix_leg_coords;
  std::vector<hrp::Vector3> default_zmp_offsets;
  double m_dt, move_base_gain;
  hrp::BodyPtr m_robot;
  coil::Mutex m_mutex;

  double transition_interpolator_ratio, transition_time, zmp_transition_time, adjust_footstep_transition_time, leg_names_interpolator_ratio;
  interpolator *zmp_offset_interpolator;
  interpolator *transition_interpolator;
  interpolator *adjust_footstep_interpolator;
  interpolator *leg_names_interpolator;
  hrp::Vector3 input_zmp, input_basePos;
  hrp::Matrix33 input_baseRot;

  // static balance point offsetting
  hrp::Vector3 sbp_offset, sbp_cog_offset;
  enum {MODE_NO_FORCE, MODE_REF_FORCE} use_force;
  std::vector<hrp::Vector3> ref_forces;

  unsigned int m_debugLevel;
  bool is_legged_robot, is_stop_mode, has_ik_failed, is_hand_fix_mode, is_hand_fix_initial;
  int loop, ik_error_debug_print_freq;
  bool graspless_manip_mode;
  std::string graspless_manip_arm;
  hrp::Vector3 graspless_manip_p_gain;
  rats::coordinates graspless_manip_reference_trans_coords;
  double pos_ik_thre, rot_ik_thre;

  //for HumanSynchronizer
  boost::shared_ptr<HumanSynchronizer> hsp;
  HumanPose hp_raw_data;

};

extern "C"
{
  void AutoBalancerInit(RTC::Manager* manager);
};

#endif // IMPEDANCE_H
