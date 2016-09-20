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


// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">


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

    double FNUM;
    int FUPLEVELNUM;
    int cur_rfup_level,cur_lfup_level;
    double FUPHIGHT;
    double CNT_F_TH;
    hrp::Vector3 pre_cont_rfpos,pre_cont_lfpos;
    hrp::Vector3 init_wld_rp_compos;
    hrp::Vector3 init_wld_hp_rfpos,init_wld_hp_lfpos;
    hrp::Vector3 init_wld_rp_rfpos,init_wld_rp_lfpos;
    hrp::Vector3 init_hp_calibcom;
    double MAXVEL,MAXACC;
    HumanPose hp_wld_raw;
    HumanPose hp_wld_initpos;
    HumanPose hp_rel_raw;
    HumanPose hp_filtered;
    HumanPose rp_ref_out_old;
    int countdown_num;
    bool HumanSyncOn;
    bool is_rf_in_air,is_lf_in_air;
    struct timeval t_calc_start, t_calc_end;
    unsigned int loop;
    double h2r_ratio, tgt_h2r_ratio;
    double cmmr, tgt_cmmr;
    hrp::Vector3 lpf_zmp;

  public:

    bool startCountdownForHumanSync;
    bool ht_first_call;
    bool is_rf_contact,is_lf_contact;
    HumanPose rp_wld_initpos;
    bool use_x,use_y,use_z;
    hrp::Vector3 init_wld_rp_basepos;
    HumanPose rp_ref_out;
    FILE *sr_log,*cz_log;

    HumanSynchronizer(){
      tgt_h2r_ratio = h2r_ratio = 0.96;//human 1.1m vs jaxon 1.06m
//      tgt_h2r_ratio = h2r_ratio = 0.62;//human 1.1m vs chidori 0.69m
//      tgt_h2r_ratio = h2r_ratio = 0.69;//human 1.0(with heavy foot sensor) vs chidori 0.69
//      tgt_h2r_ratio = h2r_ratio = 1.06;//human 1.0(with heavy foot sensor) vs jaxon 1.06
      tgt_cmmr = cmmr = 1.0;
      use_x = true; use_y = true; use_z = true;
      cur_rfup_level = 0;       cur_lfup_level = 0;
      is_rf_contact = true;     is_lf_contact = true;
      pre_cont_rfpos = hrp::Vector3::Zero();    pre_cont_lfpos = hrp::Vector3::Zero();
      init_wld_hp_rfpos = hrp::Vector3(0,-0.1/h2r_ratio,0);//h2r_ratioは可変なので後の値変更に注意
      init_wld_hp_lfpos = hrp::Vector3(0, 0.1/h2r_ratio,0);
      init_wld_rp_rfpos = hrp::Vector3(0,-0.1,0);
      init_wld_rp_lfpos = hrp::Vector3(0, 0.1,0);
      init_hp_calibcom.Zero();
      lpf_zmp.Zero();
      FNUM = 0.01;//0.01でも荒い?
      FUPLEVELNUM = 200;
      FUPHIGHT = 0.05;
      CNT_F_TH = 20.0;
      MAXVEL = 0.004;
      HumanSyncOn = false;
      ht_first_call = true;
      startCountdownForHumanSync = false;
      countdown_num = 5*500;
      loop = 0;
      is_rf_in_air = false; is_lf_in_air = false;
//      std::string home_path(std::getenv("HOME"));
//      sr_log = fopen((home_path+"/HumanSync_support_region.log").c_str(),"w+");
//      cz_log = fopen((home_path+"/HumanSync_com_zmp.log").c_str(),"w+");
      cout<<"HumanSynchronizer constructed"<<endl;
    }
    ~HumanSynchronizer(){
//      fclose(sr_log);
//      fclose(cz_log);
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
    void update(){
      gettimeofday(&t_calc_start, NULL);
      updateHumanToRobotRatio(tgt_h2r_ratio);
      updateCOMModRatio(tgt_cmmr);
      removeInitOffsetPose(hp_wld_raw, hp_wld_initpos, hp_rel_raw);
      applyInitHumanCOMOffset(hp_rel_raw, init_hp_calibcom, hp_rel_raw);
      applyLPFilter(hp_rel_raw, hp_filtered, hp_filtered);
      lockFootXYOnContact(hp_filtered, is_rf_contact, is_lf_contact, hp_filtered);//根本から改変すべき
      calcWorldZMP((hp_filtered.rf+init_wld_hp_rfpos), (hp_filtered.lf+init_wld_hp_lfpos), hp_filtered.rfw, hp_filtered.lfw, hp_filtered.zmp);//足の位置はworldにしないと・・・
      convertRelHumanPoseToRelRobotPose(hp_filtered,rp_ref_out);
      applyCOMMoveModRatio(rp_ref_out,rp_ref_out);
      judgeFootContactStates(hp_wld_raw.rfw, hp_wld_raw.lfw, is_rf_contact, is_lf_contact);//frwはすでに1000->100Hzにフィルタリングされている
      overwriteFootZFromContactStates(rp_ref_out, is_rf_contact, is_lf_contact, rp_ref_out);
//      lockFootXYOnContact(rp_ref_out, is_rf_contact, is_lf_contact, rp_ref_out);//ここで改変するとLPFかかってないからジャンプする
      applyWorkspaceLimit(rp_ref_out, rp_ref_out);
      applyVelLimit(rp_ref_out, rp_ref_out_old, rp_ref_out);
      applyCOMZMPXYZLock(rp_ref_out, rp_ref_out);

      applyCOMToSupportRegionLimit(rp_ref_out.lf+init_wld_rp_lfpos,rp_ref_out.rf+init_wld_rp_rfpos,rp_ref_out.com);

      rp_ref_out_old = rp_ref_out;
//      fprintf(log,"%f %f %f %f %f %f\n", rp_ref_out.com(0), rp_ref_out.com(1), rp_ref_out.zmp(0), rp_ref_out.zmp(1), hp_rel_raw.com(0), hp_rel_raw.com(1));
      loop++;
      gettimeofday(&t_calc_end, NULL);
    }
    void calibInitHumanCOMFromZMP(){
      init_hp_calibcom(0) = hp_filtered.zmp(0);
      init_hp_calibcom(1) = hp_filtered.zmp(1);
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
      init_wld_hp_rfpos = hrp::Vector3(0,-0.1/h2r_ratio,0);
      init_wld_hp_lfpos = hrp::Vector3(0, 0.1/h2r_ratio,0);
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
    void applyInitHumanCOMOffset(const HumanPose& in, const hrp::Vector3& com_offs, HumanPose& out){
      out = in;
      out.com = in.com + com_offs;
    }
//    void startHumanSync(){
//      countdown_flag = true;
//      setInitOffsetPose();
//      HumanSyncOn = true;
//    }
    void applyLPFilter(const HumanPose& raw_in, const HumanPose& out_old, HumanPose& filt_out){
      for(int i=0;i<filt_out.idsize;i++){
//      for(int i=0;i<filt_out.idsize-5;i++){//ここではrfw,zmpとかはフィルターしないほうがいい
        filt_out.Seq(i) = (1-FNUM) * out_old.Seq(i) + FNUM * raw_in.Seq(i);
      }
//      for(int i=filt_out.idsize-5;i<filt_out.idsize;i++){//ここではrfw,zmpとかはフィルターしないほうがいい
//        filt_out.Seq(i) = raw_in.Seq(i);
//      }
    }
    void convertRelHumanPoseToRelRobotPose(const HumanPose& hp_in, HumanPose& rp_out){//結局初期指定からの移動量(=Rel)で計算をしてゆく
      for(int i=0;i<rp_out.idsize-4;i++){
        rp_out.Seq(i) =  h2r_ratio * hp_in.Seq(i);
      }
      rp_out.rfw  = hp_in.rfw;
      rp_out.lfw  = hp_in.lfw;
    }
    void applyCOMMoveModRatio(const HumanPose& in, HumanPose& out){//結局初期指定からの移動量(=Rel)で計算をしてゆく
      out = in;
      out.com = cmmr * in.com;
    }
    void judgeFootContactStates(const HumanPose::Wrench6& rfw_in, const HumanPose::Wrench6& lfw_in, bool& rfcs_ans, bool& lfcs_ans){
      if      (rfcs_ans  && lfcs_ans && rfw_in.f(2)<CNT_F_TH   ){rfcs_ans = false;}//右足ついた状態から上げる
      else if (!rfcs_ans && lfcs_ans && rfw_in.f(2)>CNT_F_TH+30){rfcs_ans = true;}//右足浮いた状態から下げる
      if      (lfcs_ans  && rfcs_ans && lfw_in.f(2)<CNT_F_TH   ){lfcs_ans = false;}//左足ついた状態から上げる
      else if (!lfcs_ans && rfcs_ans && lfw_in.f(2)>CNT_F_TH+30){lfcs_ans = true;}//左足浮いた状態から下げる


      //else if (rfw_in.f(2)>CNT_F_TH && lfw_in.f(2)<CNT_F_TH){rfcs_ans = true;   lfcs_ans = false;}//右足接地かつ左足浮遊
      //else                                              {rfcs_ans = true;   lfcs_ans = true;}//両足接地または両足浮遊
    }
    void overwriteFootZFromContactStates(const HumanPose& in, const bool& rfcs_in, const bool& lfcs_in, HumanPose& out){
      out = in;
      if(!rfcs_in){//右足浮遊時
        if(cur_rfup_level < FUPLEVELNUM){
          cur_rfup_level++;
          out.rf(2) = FUPHIGHT/2*(1-cos(M_PI*cur_rfup_level/FUPLEVELNUM));
        }else{
          out.rf(2) = FUPHIGHT;
        }
      }else{
        if(cur_rfup_level > 0){
          cur_rfup_level--;
          out.rf(2) = FUPHIGHT/2*(1-cos(M_PI*cur_rfup_level/FUPLEVELNUM));
        }else{
          out.rf(2) = 0;
        }
      }
      if(!lfcs_in){//左足浮遊時
        if(cur_lfup_level < FUPLEVELNUM){
          cur_lfup_level++;
          out.lf(2) = FUPHIGHT/2*(1-cos(M_PI*cur_lfup_level/FUPLEVELNUM));
        }else{
          out.lf(2) = FUPHIGHT;
        }
      }else{
        if(cur_lfup_level > 0){
          cur_lfup_level--;
          out.lf(2) = FUPHIGHT/2*(1-cos(M_PI*cur_lfup_level/FUPLEVELNUM));
        }else{
          out.lf(2) = 0;
        }
      }
    }
    void lockFootXYOnContact(const HumanPose& in, const bool& rfcs_in, const bool& lfcs_in, HumanPose& out){
      out = in;
      if(rfcs_in){
        out.rf(0) = pre_cont_rfpos(0);
        out.rf(1) = pre_cont_rfpos(1);
      }else{
        pre_cont_rfpos(0) = in.rf(0);
        pre_cont_rfpos(1) = in.rf(1);
      }
      if(lfcs_in){
        out.lf(0) = pre_cont_lfpos(0);
        out.lf(1) = pre_cont_lfpos(1);
      }else{
        pre_cont_lfpos(0) = in.lf(0);
        pre_cont_lfpos(1) = in.lf(1);
      }
//      out.rf(0) = 0; out.rf(1) = 0; out.lf(0) = 0; out.lf(1) = 0;//足固定テスト
    }
    void applyWorkspaceLimit(const HumanPose& in, HumanPose& out){
      if(!is_rf_contact && is_lf_contact){//右足浮遊時
        if(in.rf(1) + rp_wld_initpos.rf(1) > in.lf(1) + rp_wld_initpos.lf(1) - 0.15)out.rf(1) = in.lf(1) + rp_wld_initpos.lf(1) - rp_wld_initpos.rf(1) - 0.15;
      }
      else if(is_rf_contact && !is_lf_contact){//左足浮遊時
        if(in.lf(1) + rp_wld_initpos.lf(1) < in.rf(1) + rp_wld_initpos.rf(1) + 0.15)out.lf(1) = in.rf(1) + rp_wld_initpos.rf(1) - rp_wld_initpos.lf(1) + 0.15;
      }
    }
//    bool applyCOMToSupportRegionLimit_boost_geometry(const hrp::Vector3& lfin_abs, const hrp::Vector3& rfin_abs, hrp::Vector3& comin_abs){
////      ishiguro(ロボット体内のboostが古くて対応してない！！！)
//      #include <boost/assert.hpp>
//      #include <boost/assign/list_of.hpp>
//      #include <boost/geometry.hpp>
//      #include <boost/geometry/geometries/point_xy.hpp>
//      #include <boost/geometry/geometries/box.hpp>
//      #include <boost/geometry/geometries/polygon.hpp>
//      #include <boost/geometry/algorithms/disjoint.hpp>
//      #include <boost/geometry/geometries/geometries.hpp>
//      namespace bg = boost::geometry;
//      typedef bg::model::d2::point_xy<double> point;
//      typedef bg::model::polygon<point> polygon;
//      polygon lr_region,s_region;
//      const double XUMARGIN = 0.04;//CHIDORI
//      const double XLMARGIN = -0.02;
//      const double YUMARGIN = 0.01;
//      const double YLMARGIN = -0.01;
//      bg::model::linestring<point> s_line = boost::assign::list_of<point>
//        (lfin_abs(0) + XLMARGIN, lfin_abs(1) + YLMARGIN)//LFの右下
//        (lfin_abs(0) + XLMARGIN, lfin_abs(1) + YUMARGIN)//LFの左下
//        (lfin_abs(0) + XUMARGIN, lfin_abs(1) + YUMARGIN)//LFの左上
//        (lfin_abs(0) + XUMARGIN, lfin_abs(1) + YLMARGIN)//LFの右上
//        (rfin_abs(0) + XLMARGIN, rfin_abs(1) + YLMARGIN)//RFの右下
//        (rfin_abs(0) + XLMARGIN, rfin_abs(1) + YUMARGIN)//RFの左下
//        (rfin_abs(0) + XUMARGIN, rfin_abs(1) + YUMARGIN)//RFの左上
//        (rfin_abs(0) + XUMARGIN, rfin_abs(1) + YLMARGIN)//RFの右上
//             ;
//      bg::convex_hull(s_line, s_region);
//      point com2d(comin_abs(0),comin_abs(1));
//      Eigen::Vector2d ans_point(comin_abs(0),comin_abs(1));
//      enum point_state_t {PT_IN_NO_PATTERN = -1, PT_IN_REGION, PT_FIX_TO_EDGE, PT_FIX_TO_VERTEX};
//      point_state_t pt_state = PT_IN_NO_PATTERN;
//      if(bg::within(com2d, s_region)){//対象の点が凸包の内部に存在するかチェックする
//        pt_state = PT_IN_REGION;
//      }else{
//        Eigen::Vector2d check_point(com2d.x(),com2d.y());
//        int ans_lid=-1;
//        for(int i=0;i<s_region.outer().size()-1;i++){//対象の点がある線分への垂線を有するかチェックする
//          Eigen::Vector2d cur_vert(s_region.outer()[i].x(),s_region.outer()[i].y());
//          Eigen::Vector2d next_vert(s_region.outer()[i+1].x(),s_region.outer()[i+1].y());
//          Eigen::Vector2d edge_v = next_vert - cur_vert;
//          Eigen::Vector2d tgt_pt_v = check_point - cur_vert;
//          //ある線分への垂線を有し，かつ外側(時計回りエッジに対して左側)に存在するなら，対象の点からそのエッジへの垂線の交点が最近傍点
//          if(edge_v.dot(tgt_pt_v)/edge_v.norm() > 0 && edge_v.dot(tgt_pt_v)/edge_v.norm() < edge_v.norm() && (edge_v(0)*tgt_pt_v(1)-edge_v(1)*tgt_pt_v(0)) > 0){
//            ans_lid = i;
//            ans_point = cur_vert + edge_v.normalized() * (edge_v.dot(tgt_pt_v)/edge_v.norm());
//            pt_state = PT_FIX_TO_EDGE;
//            break;
//          }
//        }
//        if(pt_state == PT_FIX_TO_EDGE){
//          cout<<"point will on the line:"<<ans_lid<<" point:"<<ans_point(0)<<","<<ans_point(1)<<endl;
//        }else{//対象の点が線分への垂線を持たなければ答えを頂点に絞ってチェックする
//          double cur_min_dis = bg::distance(com2d, s_region.outer()[0]);
//          int ans_pid = 0;
//          for(int i=1;i<s_region.outer().size();i++){
//            if(bg::distance(com2d, s_region.outer()[i]) < cur_min_dis){
//              cur_min_dis = bg::distance(com2d, s_region.outer()[i]);
//              ans_pid = i;
//            }
//          }
//          ans_point(0) = s_region.outer()[ans_pid].x();
//          ans_point(1) = s_region.outer()[ans_pid].y();
//          cout<<"point will on the vertex:"<<ans_pid<<" point:"<<ans_point(0)<<","<<ans_point(1)<<endl;
//          pt_state = PT_FIX_TO_VERTEX;
//        }
//      }
//      if(loop%50==0){
//        if(pt_state != PT_IN_REGION){
//          std::cout<<"COM out of Support Region pt_state="<<pt_state<<std::endl;
//        }
//        for(int i=0;i<s_region.outer().size();i++){
//          fprintf(sr_log,"%f %f %f\n",s_region.outer().at(i).x(), s_region.outer().at(i).y(),(double)loop/500.0);
//        }
//        fprintf(sr_log,"\n");
//        fprintf(cz_log,"%f %f %f %f %f %f %f %f %f",(double)loop/500.0,comin_abs(0),comin_abs(1),comin_abs(2),rp_ref_out.zmp(0),rp_ref_out.zmp(1),rp_ref_out.zmp(2),ans_point(0),ans_point(1));
//        fprintf(cz_log,"\n\n");
//      }
//      comin_abs(0) = ans_point(0);
//      comin_abs(1) = ans_point(1);
//      return true;
//    }
    bool applyCOMToSupportRegionLimit(const hrp::Vector3& lfin_abs, const hrp::Vector3& rfin_abs, hrp::Vector3& comin_abs){
//      const double XUMARGIN = 0.04;//CHIDORI
//      const double XLMARGIN = -0.02;
//      const double YUMARGIN = 0.01;
//      const double YLMARGIN = -0.01;
      const double XUMARGIN = 0.02;//JAXON
      const double XLMARGIN = -0.01;
      const double YUMARGIN = 0.0;
      const double YLMARGIN = -0.0;
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
//      double FNUM_Z = 0.005;
      double FNUM_Z = 0.01;
      lpf_zmp = (1-FNUM_Z) * lpf_zmp + FNUM_Z * rp_ref_out.zmp;
      hrp::Vector3 hpf_zmp = rp_ref_out.zmp - lpf_zmp;
//      rp_ref_out.zmp(0) = ans_point(0) + hpf_zmp(0);
//      rp_ref_out.zmp(1) = ans_point(1) + hpf_zmp(1);

//      if(loop%50==0){//ログ
//        if(pt_state != PT_IN_REGION){
//          std::cout<<"COM out of Support Region pt_state="<<pt_state<<std::endl;
//        }
//        for(int i=0;i<convex_hull.size();i++){
//          fprintf(sr_log,"%f %f %f\n",convex_hull[i](0), convex_hull[i](1),(double)loop/500.0);
//        }
//        fprintf(sr_log,"\n");
//        fprintf(cz_log,"%f %f %f %f %f %f %f %f %f",(double)loop/500.0,comin_abs(0),comin_abs(1),comin_abs(2),rp_ref_out.zmp(0),rp_ref_out.zmp(1),rp_ref_out.zmp(2),ans_point(0),ans_point(1));
//        fprintf(cz_log," %f %f %f %f",rp_ref_out.zmp(1),hpf_zmp(1),ans_point(1),ans_point(1) + hpf_zmp(1));
//        fprintf(cz_log,"\n\n");
//      }
      comin_abs(0) = ans_point(0);
      comin_abs(1) = ans_point(1);
//      rp_ref_out.zmp(0) = comin_abs(0);
//      rp_ref_out.zmp(1) = comin_abs(1);
      return true;
    }
    void applyVelLimit(const HumanPose& in, const HumanPose& in_old, HumanPose& out){
      for(int i=0;i<3;i++){if(in.com(i) - in_old.com(i) > MAXVEL){out.com(i) = in_old.com(i) + MAXVEL;}else if(in.com(i) - in_old.com(i) < -MAXVEL){out.com(i) = in_old.com(i) - MAXVEL;}}
      for(int i=0;i<3;i++){if(in.rf(i)  - in_old.rf(i)  > MAXVEL){out.rf(i)  = in_old.rf(i)  + MAXVEL;}else if(in.rf(i)  - in_old.rf(i)  < -MAXVEL){out.rf(i)  = in_old.rf(i)  - MAXVEL;}}
      for(int i=0;i<3;i++){if(in.lf(i)  - in_old.lf(i)  > MAXVEL){out.lf(i)  = in_old.lf(i)  + MAXVEL;}else if(in.lf(i)  - in_old.lf(i)  < -MAXVEL){out.lf(i)  = in_old.lf(i)  - MAXVEL;}}
      for(int i=0;i<3;i++){if(in.rh(i)  - in_old.rh(i)  > MAXVEL){out.rh(i)  = in_old.rh(i)  + MAXVEL;}else if(in.rh(i)  - in_old.rh(i)  < -MAXVEL){out.rh(i)  = in_old.rh(i)  - MAXVEL;}}
      for(int i=0;i<3;i++){if(in.lh(i)  - in_old.lh(i)  > MAXVEL){out.lh(i)  = in_old.lh(i)  + MAXVEL;}else if(in.lh(i)  - in_old.lh(i)  < -MAXVEL){out.lh(i)  = in_old.lh(i)  - MAXVEL;}}
    }
    void applyCOMZMPXYZLock(const HumanPose& in, HumanPose& out){
      out = in;
        if(!use_x){out.com(0) = 0;out.zmp(0) = 0;}
        if(!use_y){out.com(1) = 0;out.zmp(1) = 0;}
        if(!use_z){out.com(2) = 0;}
        out.rh(0)=0;
        out.rh(1)=out.com(1);
        out.rh(2)=0;
        out.lh(0)=0;
        out.lh(1)=out.com(1);
        out.lh(2)=0;
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
  TimedDoubleSeq m_legMargin;
  InPort<TimedDoubleSeq> m_legMarginIn;
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
