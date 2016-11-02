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
public:
    BiquadIIRFilterVec(const std::string& error_prefix = ""){
    };
    ~BiquadIIRFilterVec() {};
    bool setParameter(const double fc_in, const double HZ){
      std::vector<double> fb_coeffs, ff_coeffs;
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




class HumanPose{
  private: hrp::Vector3 zero;
  public:
    hrp::Vector3 com,rf,lf,rh,lh;
    hrp::Vector3 zmp;
    int idsize;
    typedef struct{ hrp::Vector3 f; hrp::Vector3 t; }Wrench6;
    Wrench6 rfw,lfw;
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
//      return Seq(i);//呼べない・・・うーん
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
    double FUP_HIGHT;
    double CNT_F_TH;
    hrp::Vector3 pre_cont_rfpos,pre_cont_lfpos;
    hrp::Vector3 init_hp_calibcom;
    double MAXVEL,MAXACC;
    HumanPose hp_wld_raw;
    HumanPose hp_wld_initpos;
    HumanPose hp_rel_raw;
    HumanPose hp_filtered;
    HumanPose rp_ref_out_old;
    int countdown_num;
    bool HumanSyncOn;
    struct timeval t_calc_start, t_calc_end;
    unsigned int loop;
    double h2r_ratio, tgt_h2r_ratio;
    double cmmr, tgt_cmmr;
    double HZ,DT,G;
    hrp::Vector3 com_old,com_oldold,comacc;
    std::vector<BiquadIIRFilterVec> iir_v_filters;
    BiquadIIRFilterVec comacc_v_filters;
    hrp::Vector3 r_zmp_raw;

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
    hrp::Vector3 init_wld_rp_rfpos,init_wld_rp_lfpos;
    hrp::Vector3 init_wld_hp_rfpos,init_wld_hp_lfpos;
    hrp::Vector3 current_basepos,init_basepos;


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
      is_rf_contact = is_lf_contact = true;
      go_rf_landing = go_lf_landing = false;
      pre_cont_rfpos = pre_cont_lfpos = hrp::Vector3::Zero();
      init_hp_calibcom.Zero();
      tgt_FUP_TIME = FUP_TIME = 0.4;
      FUP_HIGHT = 0.05;
      CNT_F_TH = 20.0;
      MAXVEL = 0.004;
      HumanSyncOn = false;
      ht_first_call = true;
      startCountdownForHumanSync = false;
      countdown_num = 5*HZ;
      loop = 0;

      com_old = com_oldold = comacc = hrp::Vector3::Zero();
      r_zmp_raw = hrp::Vector3::Zero();

      init_wld_rp_rfpos = init_wld_rp_lfpos = hrp::Vector3::Zero();
      init_wld_hp_rfpos = init_wld_hp_lfpos = hrp::Vector3::Zero();
      current_basepos = init_basepos = hrp::Vector3::Zero();

      iir_v_filters.resize(5);
      for(int i=0;i<iir_v_filters.size();i++)iir_v_filters[i].setParameter(0.5,HZ);
      comacc_v_filters.setParameter(0.5,HZ);

//      use_rh = use_lh = false;
      use_rh = use_lh = true;

//      zmp_com_offset_ip_ratio = 0;


      init_zmp_com_offset_interpolator = new interpolator(1, DT, interpolator::HOFFARBIB, 1);
      init_zmp_com_offset_interpolator->setName("zmp_com_offset_interpolator");
      double start_ratio = 0.0;
      init_zmp_com_offset_interpolator->set(&start_ratio);
      double goal_ratio = 1.0;
      init_zmp_com_offset_interpolator->go(&goal_ratio, 3.0, true);//3s

      if(DEBUG){
        std::string home_path(std::getenv("HOME"));
        sr_log = fopen((home_path+"/HumanSync_support_region.log").c_str(),"w+");
        cz_log = fopen((home_path+"/HumanSync_com_zmp.log").c_str(),"w+");
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
      lockFootXYOnContact                 (go_rf_landing, go_lf_landing, hp_rel_raw);//根本から改変すべき2
      hp_filtered = hp_rel_raw;

//      applyLPFilterToEE(hp_rel_raw, hp_filtered);//次に使う.rf,.lfのためにEEだけは先にフィルターしておかないといけない説もある
      calcWorldZMP                        ((hp_filtered.rf+init_wld_hp_rfpos), (hp_filtered.lf+init_wld_hp_lfpos), hp_filtered.rfw, hp_filtered.lfw, hp_filtered.zmp);//足の位置はworldにしないと・・・
      convertRelHumanPoseToRelRobotPose   (hp_filtered,rp_ref_out);
      applyCOMMoveModRatio                (rp_ref_out);
      judgeFootLandOnCommand              (hp_wld_raw.rfw, hp_wld_raw.lfw, go_rf_landing, go_lf_landing);//frwはすでに1000->100Hzにフィルタリングされている

      applyEEWorkspaceLimit               (rp_ref_out);

      applyCOMToSupportRegionLimit        (rp_ref_out.rf+init_wld_rp_rfpos, rp_ref_out.lf+init_wld_rp_lfpos, rp_ref_out.com);

      applyLPFilter                       (rp_ref_out);
//      applyLPFilter_o                       (rp_ref_out, rp_ref_out_old, rp_ref_out);
      r_zmp_raw = rp_ref_out.zmp;
      applyZMPCalcFromCOM                 (rp_ref_out.com,rp_ref_out.zmp);
      lockSwingFootIfZMPOutOfSupportFoot  (rp_ref_out, go_rf_landing, go_lf_landing);//

      applyVelLimit                       (rp_ref_out, rp_ref_out_old, rp_ref_out);
      applyCOMZMPXYZLock                  (rp_ref_out);
      overwriteFootZFromFootLandOnCommand (go_rf_landing, go_lf_landing, rp_ref_out);
      rp_ref_out_old = rp_ref_out;
//      fprintf(log,"%f %f %f %f %f %f\n", rp_ref_out.com(0), rp_ref_out.com(1), rp_ref_out.zmp(0), rp_ref_out.zmp(1), hp_rel_raw.com(0), hp_rel_raw.com(1));
      loop++;
      gettimeofday(&t_calc_end, NULL);
    }
    void calibInitHumanCOMFromZMP(){
//      init_hp_calibcom(0) = hp_filtered.zmp(0);
//      init_hp_calibcom(1) = hp_filtered.zmp(1);
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
    static hrp::Vector3 Point3DToVector3(const RTC::TimedPoint3D& in){ hrp::Vector3 out(in.data.x,in.data.y,in.data.z); return out; }
    static HumanPose::Wrench6 DoubleSeqToWrench6(const RTC::TimedDoubleSeq& in){
      HumanPose::Wrench6 out;
      if(in.data.length() == 6){out.f(0)=in.data[0]; out.f(1)=in.data[1]; out.f(2)=in.data[2]; out.t(0)=in.data[3]; out.t(1)=in.data[4]; out.t(2)=in.data[5];
      }else{ std::cerr<<"[WARN] HumanPose::Wrench6 DoubleSeqToWrench6() invalid data length"<<std::endl; out.f.Zero(); out.t.Zero(); }
      return out;
    }

  private:
    void updateHumanToRobotRatio(const double h2r_r_goal){//h2r_ratioに伴って変わる変数の処理もここに書く
      const double step = 0.001;
      if(h2r_r_goal - h2r_ratio > step){h2r_ratio += step; cout<<"Interpolating HumanToRobotRatio as "<<h2r_ratio<<endl;}
      else if(h2r_r_goal - h2r_ratio < (-1)*step){h2r_ratio -= step; cout<<"Interpolating HumanToRobotRatio as "<<h2r_ratio<<endl;}
      else{h2r_ratio = h2r_r_goal;}
      init_wld_hp_rfpos = init_wld_rp_rfpos/h2r_ratio;
      init_wld_hp_lfpos = init_wld_rp_lfpos/h2r_ratio;
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
        if(tgt.zmp(1) > tgt.rf(1) + init_wld_rp_rfpos(1) + 0.02){
          lfcs_ans = true;
          cout<<"LF lock"<<endl;
        }
      }
      if(!rfcs_ans && lfcs_ans){
        if(tgt.zmp(1) < tgt.lf(1) + init_wld_rp_lfpos(1) - 0.02){
          rfcs_ans = true;
          cout<<"RF lock"<<endl;
        }
      }
    }
    void calcFootUpCurveAndJudgeFootContact(const bool& go_land_in, int& cur_fup_count_in, hrp::Vector3& f_height_out, bool& is_f_contact_out){
      int FUP_COUNT = (int)(HZ*FUP_TIME);
      if(cur_fup_count_in > FUP_COUNT){cur_fup_count_in = FUP_COUNT;} if(cur_fup_count_in < 0){cur_fup_count_in = 0;}
      if(!go_land_in){//足下げ命令入力時
        if(cur_fup_count_in < FUP_COUNT){ cur_fup_count_in++; }
      }else{//足上げ命令入力時
        if(cur_fup_count_in > 0){ cur_fup_count_in--; }
      }
      f_height_out(2) = FUP_HIGHT/2*(1-cos(M_PI*cur_fup_count_in/FUP_COUNT));
      is_f_contact_out = (cur_fup_count_in <= 0);
    }
    void overwriteFootZFromFootLandOnCommand(const bool& rf_goland_in, const bool& lf_goland_in, HumanPose& tgt){
      if(is_rf_contact&&is_lf_contact)FUP_TIME = tgt_FUP_TIME;//両足設置しているタイミングでのみ更新
      calcFootUpCurveAndJudgeFootContact(rf_goland_in, cur_rfup_level, tgt.rf, is_rf_contact);
      calcFootUpCurveAndJudgeFootContact(lf_goland_in, cur_lfup_level, tgt.lf, is_lf_contact);
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
      if(!is_rf_contact && is_lf_contact){//右足浮遊時
        if(tgt.rf(1) + rp_wld_initpos.rf(1) > tgt.lf(1) + rp_wld_initpos.lf(1) - 0.15)tgt.rf(1) = tgt.lf(1) + rp_wld_initpos.lf(1) - rp_wld_initpos.rf(1) - 0.15;
      }
      else if(is_rf_contact && !is_lf_contact){//左足浮遊時
        if(tgt.lf(1) + rp_wld_initpos.lf(1) < tgt.rf(1) + rp_wld_initpos.rf(1) + 0.15)tgt.lf(1) = tgt.rf(1) + rp_wld_initpos.rf(1) - rp_wld_initpos.lf(1) + 0.15;
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
      const double XUMARGIN = 0.04;//CHIDORI
      const double XLMARGIN = -0.02;
      const double YUMARGIN = 0.01;
      const double YLMARGIN = -0.01;
//      const double XUMARGIN = 0.02;//JAXON
//      const double XLMARGIN = -0.01;
//      const double YUMARGIN = 0.0;
//      const double YLMARGIN = -0.0;
//      const double XUMARGIN = 0.10;//test
//      const double XLMARGIN = -0.05;
//      const double YUMARGIN = 0.03;
//      const double YLMARGIN = -0.03;
      std::vector<hrp::Vector2> convex_hull;
      if(lfin_abs(0)>rfin_abs(0)){
        convex_hull.push_back(hrp::Vector2(lfin_abs(0) + XLMARGIN, lfin_abs(1) + YUMARGIN));//LFの左下
        convex_hull.push_back(hrp::Vector2(lfin_abs(0) + XUMARGIN, lfin_abs(1) + YUMARGIN));//LFの左上
        convex_hull.push_back(hrp::Vector2(lfin_abs(0) + XUMARGIN, lfin_abs(1) + YLMARGIN));//LFの右上
        convex_hull.push_back(hrp::Vector2(rfin_abs(0) + XUMARGIN, rfin_abs(1) + YLMARGIN));//RFの右上
        convex_hull.push_back(hrp::Vector2(rfin_abs(0) + XLMARGIN, rfin_abs(1) + YLMARGIN));//RFの右下
        convex_hull.push_back(hrp::Vector2(rfin_abs(0) + XLMARGIN, rfin_abs(1) + YUMARGIN));//RFの左下
        convex_hull.push_back(hrp::Vector2(lfin_abs(0) + XLMARGIN, lfin_abs(1) + YUMARGIN));//LFの左下
      }else{
        convex_hull.push_back(hrp::Vector2(lfin_abs(0) + XLMARGIN, lfin_abs(1) + YLMARGIN));//LFの右下
        convex_hull.push_back(hrp::Vector2(lfin_abs(0) + XLMARGIN, lfin_abs(1) + YUMARGIN));//LFの左下
        convex_hull.push_back(hrp::Vector2(lfin_abs(0) + XUMARGIN, lfin_abs(1) + YUMARGIN));//LFの左上
        convex_hull.push_back(hrp::Vector2(rfin_abs(0) + XUMARGIN, rfin_abs(1) + YUMARGIN));//RFの左上
        convex_hull.push_back(hrp::Vector2(rfin_abs(0) + XUMARGIN, rfin_abs(1) + YLMARGIN));//RFの右上
        convex_hull.push_back(hrp::Vector2(rfin_abs(0) + XLMARGIN, rfin_abs(1) + YLMARGIN));//RFの右下
        convex_hull.push_back(hrp::Vector2(lfin_abs(0) + XLMARGIN, lfin_abs(1) + YLMARGIN));//LFの右下
      }
      hrp::Vector2 check_point(comin_abs(0),comin_abs(1));
      hrp::Vector2 ans_point = check_point;
      enum point_state_t {PT_IN_NO_PATTERN = -1, PT_IN_REGION, PT_FIX_TO_EDGE, PT_FIX_TO_VERTEX};
      point_state_t pt_state = PT_IN_REGION;

      for(int i=0;i<convex_hull.size()-1;i++){//対象の点が凸包の内部に存在するかチェックする
        hrp::Vector2 cur_vert(convex_hull[i](0),convex_hull[i](1));
        hrp::Vector2 next_vert(convex_hull[i+1](0),convex_hull[i+1](1));
        hrp::Vector2 edge_v = next_vert - cur_vert;
        hrp::Vector2 tgt_pt_v = check_point - cur_vert;
        if((edge_v(0)*tgt_pt_v(1)-edge_v(1)*tgt_pt_v(0)) > 0){
          pt_state = PT_IN_NO_PATTERN;
//          cout<<"COM out of s_region"<<endl;
          break;
        }
      }

      if(pt_state != PT_IN_REGION){
        int ans_lid=-1;
        for(int i=0;i<convex_hull.size()-1;i++){//対象の点がある線分への垂線を有するかチェックする
          hrp::Vector2 cur_vert(convex_hull[i](0),convex_hull[i](1));
          hrp::Vector2 next_vert(convex_hull[i+1](0),convex_hull[i+1](1));
          hrp::Vector2 edge_v = next_vert - cur_vert;
          hrp::Vector2 tgt_pt_v = check_point - cur_vert;
          //ある線分への垂線を有し，かつ外側(時計回りエッジに対して左側)に存在するなら，対象の点からそのエッジへの垂線の交点が最近傍点
          if(edge_v.dot(tgt_pt_v)/edge_v.norm() > 0 && edge_v.dot(tgt_pt_v)/edge_v.norm() < edge_v.norm() && (edge_v(0)*tgt_pt_v(1)-edge_v(1)*tgt_pt_v(0)) > 0){
            ans_lid = i;
            ans_point = cur_vert + edge_v.normalized() * (edge_v.dot(tgt_pt_v)/edge_v.norm());
            pt_state = PT_FIX_TO_EDGE;
//            cout<<"point will on the line:"<<ans_lid<<" point:"<<ans_point(0)<<","<<ans_point(1)<<endl;
            break;
          }
        }
        if(pt_state != PT_FIX_TO_EDGE){//対象の点が線分への垂線を持たなければ答えを頂点に絞ってチェックする
          double cur_min_dis = (check_point - convex_hull[0]).norm();
          int ans_pid = 0;
          for(int i=1;i<convex_hull.size();i++){
            if((check_point - convex_hull[i]).norm() < cur_min_dis){
              cur_min_dis = (check_point - convex_hull[i]).norm();
              ans_pid = i;
            }
          }
          ans_point(0) = convex_hull[ans_pid](0);
          ans_point(1) = convex_hull[ans_pid](1);
//          cout<<"point will on the vertex:"<<ans_pid<<" point:"<<ans_point(0)<<","<<ans_point(1)<<endl;
          pt_state = PT_FIX_TO_VERTEX;
        }
      }

//      if(loop%50==0){//ログ
//        if(pt_state != PT_IN_REGION){
//          std::cout<<"COM out of Support Region pt_state="<<pt_state<<std::endl;
//        }
//        for(int i=0;i<convex_hull.size();i++){
//          fprintf(sr_log,"%f %f %f\n",convex_hull[i](0), convex_hull[i](1),(double)loop/HZ);
//        }
//        fprintf(sr_log,"\n");
//        fprintf(cz_log,"%f %f %f %f %f %f %f %f %f",(double)loop/HZ,comin_abs(0),comin_abs(1),comin_abs(2),rp_ref_out.zmp(0),rp_ref_out.zmp(1),rp_ref_out.zmp(2),ans_point(0),ans_point(1));
//        fprintf(cz_log," %f %f %f %f",rp_ref_out.zmp(1),hpf_zmp(1),ans_point(1),ans_point(1) + hpf_zmp(1));
//        fprintf(cz_log,"\n\n");
//      }
      comin_abs(0) = ans_point(0);
      comin_abs(1) = ans_point(1);
      return true;
    }
    void applyZMPCalcFromCOM(const hrp::Vector3& comin, hrp::Vector3& zmpout){
      comacc = (comin - 2 * com_old + com_oldold)/(DT*DT);
      const double MAXACC = 5;
      if(comacc(0)>MAXACC)comacc(0)=MAXACC;else if(comacc(0)<-MAXACC)comacc(0)=-MAXACC;
      if(comacc(1)>MAXACC)comacc(1)=MAXACC;else if(comacc(1)<-MAXACC)comacc(1)=-MAXACC;
      comacc = comacc_v_filters.passFilter(comacc);
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
