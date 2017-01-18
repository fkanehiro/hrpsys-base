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

#define LIMIT_MINMAX(x,min,max) ((x= (x<min  ? min : x<max ? x : max)))

//実機だとログ出力で落ちる？getenv("HOME")ダメ？
#define DEBUG 0

class BiquadIIRFilterVec{
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

class HRPPose3D{
  private:
    hrp::Vector3 wld;
  public:
    hrp::Vector3 p,offs,rpy;
    hrp::Matrix33 R;
    HRPPose3D(){ clear(); }
    ~HRPPose3D(){}
    void clear(){
      p = offs = rpy = wld = hrp::Vector3::Zero();
      R = hrp::Matrix33::Zero();
    }
    const hrp::Vector3& wldp(){
      wld = p + offs;
      return wld;
    }
};
class Wrench6{
  public:
    hrp::Vector3 f,t;
    Wrench6(){ clear(); }
    ~Wrench6(){}
    void clear(){ f = t = hrp::Vector3::Zero(); }
};

class HumanPose{
  public:
    std::map<std::string, HRPPose3D> P;
    std::map<std::string, Wrench6> w;
    HumanPose(){
      P["com"] = HRPPose3D();
      P["rf"] = HRPPose3D();
      P["lf"] = HRPPose3D();
      P["rh"] = HRPPose3D();
      P["lh"] = HRPPose3D();
      P["zmp"] = HRPPose3D();
      P["head"] = HRPPose3D();
      w["rfw"] = Wrench6();
      w["lfw"] = Wrench6();
      clear();
    }
    ~HumanPose(){cerr<<"HumanPose destructed"<<endl;}
    void clear(){
      for(std::map<std::string, HRPPose3D>::iterator it = P.begin(); it != P.end(); it++){ it->second.clear(); }
      for(std::map<std::string, Wrench6>::iterator it = w.begin(); it != w.end(); it++){ it->second.clear(); }
    }
    static void hp_printf(const HumanPose& in){
      const std::string ns[] = {"com","rf","lf","rh","lh","zmp","rfw","lfw"};
      const std::string cap[] = {"c","rf","lf","rh","lh","z","rw","lw"};
      for(int i=0;i<6;i++){ printf("\x1b[31m%s\x1b[39m%+05.2f %+05.2f %+05.2f ",cap[i].c_str(),in.getP(ns[i]).p(0),in.getP(ns[i]).p(1),in.getP(ns[i]).p(2)); }
      for(int i=6;i<8;i++){ printf("\x1b[31m%s\x1b[39m%+05.1f %+05.1f %+05.1f %+05.1f %+05.1f %+05.1f ",cap[i].c_str(),in.getw(ns[i]).f(0),in.getw(ns[i]).f(1),in.getw(ns[i]).f(2),in.getw(ns[i]).t(0),in.getw(ns[i]).t(1),in.getw(ns[i]).t(2)); }
      printf("\n");
    }
    HRPPose3D& getP(const std::string& key){///Safe accessor
      if (P.find(key) != P.end() ) { return P.find(key)->second; }
      else { cerr<<"\n\x1b[31m[FATAL] HumanPose Invalid Key \""<<key<<"\" !!\x1b[39m\n"<<endl; exit(-1); return P.end()->second; }
    }
    const HRPPose3D& getP(const std::string& key) const {
      if (P.find(key) != P.end() ) { return P.find(key)->second; }
      else { cerr<<"\n\x1b[31m[FATAL] HumanPose Invalid Key \""<<key<<"\" !!\x1b[39m\n"<<endl; exit(-1); return P.end()->second; }
    }
    Wrench6& getw(const std::string& key){
      if (w.find(key) != w.end() ) { return w.find(key)->second; }
      else { cerr<<"\n\x1b[31m[FATAL] HumanPose Invalid Key \""<<key<<"\" !!\x1b[39m\n"<<endl; exit(-1); return w.end()->second; }
    }
    const Wrench6& getw(const std::string& key) const {
      if (w.find(key) != w.end() ) { return w.find(key)->second; }
      else { cerr<<"\n\x1b[31m[FATAL] HumanPose Invalid Key \""<<key<<"\" !!\x1b[39m\n"<<endl; exit(-1); return w.end()->second; }
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
    std::vector<BiquadIIRFilterVec> tgt_pos_filters,tgt_rot_filters;
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
    hrp::Vector3 cam_pos_filtered,cam_rpy_filtered;
    HRPPose3D head_cam_pose;
    hrp::Vector3 pre_cont_rfpos,pre_cont_lfpos;


    interpolator *init_zmp_com_offset_interpolator;
    double zmp_com_offset_ip_ratio;

    HumanSynchronizer(){
      tgt_h2r_ratio = h2r_ratio = 0.96;//human 1.1m vs jaxon 1.06m
//      tgt_h2r_ratio = h2r_ratio = 0.62;//human 1.1m vs chidori 0.69m
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
      MAXVEL = 2.0;
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
      cam_pos_filtered = cam_rpy_filtered = hrp::Vector3::Zero();

      tgt_pos_filters.resize(5);
      tgt_rot_filters.resize(5);
//      for(int i=0;i<iir_v_filters.size();i++)iir_v_filters[i].setParameter(0.5,HZ);//四肢拘束点用
      for(int i=0;i<tgt_pos_filters.size();i++)tgt_pos_filters[i].setParameter(0.7,HZ);//四肢拘束点用(position)
      for(int i=0;i<tgt_rot_filters.size();i++)tgt_rot_filters[i].setParameter(0.7,HZ);//四肢拘束点用(Rotation)
      tgt_pos_filters[0].setParameter(0.4,HZ);//重心用
      tgt_rot_filters[0].setParameter(0.4,HZ);//重心用
      calcacc_v_filters.setParameter(5,HZ);//加速度計算用
      acc4zmp_v_filters.setParameter(1,HZ);//ZMP生成用ほぼこの値でいい
      cam_rpy_filter.setParameter(1,HZ);//カメラアングル

      rf_safe_region = hrp::Vector4(0.02, -0.01*3,  0.02,  0.01);
      lf_safe_region = hrp::Vector4(0.02, -0.01*3, -0.01, -0.02);
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
//      applyInitHumanZMPToCOMOffset        (hp_rel_raw, init_hp_calibcom, hp_rel_raw);//怪しいので要修正
      calcWorldZMP                        ((hp_rel_raw.getP("rf").p+init_wld_hp_rfpos), (hp_rel_raw.getP("lf").p+init_wld_hp_lfpos), hp_rel_raw.getw("rfw"), hp_rel_raw.getw("lfw"), hp_rel_raw.getP("zmp").p);//足の位置はworldにしないと・・・
      convertRelHumanPoseToRelRobotPose   (hp_rel_raw, rp_ref_out);

      applyCOMMoveModRatio                (rp_ref_out);
      judgeFootLandOnCommand              (hp_wld_raw.getw("rfw"), hp_wld_raw.getw("lfw"), go_rf_landing, go_lf_landing);//fwはすでに1000->100Hzにフィルタリングされている
      lockSwingFootIfZMPOutOfSupportFoot  (rp_ref_out_old, go_rf_landing, go_lf_landing);//ここ
      applyEEWorkspaceLimit               (rp_ref_out);
      lockFootXYOnContact                 (go_rf_landing, go_lf_landing, rp_ref_out);//根本から改変すべき3

      applyCOMToSupportRegionLimit        (rp_ref_out.getP("rf").p, rp_ref_out.getP("lf").p, rp_ref_out.getP("com"));
      applyLPFilter                       (rp_ref_out);
      applyCOMStateLimitByCapturePoint    (rp_ref_out.getP("com").p, rp_ref_out_old.getP("com").p, com_vel_old, rp_ref_out.getP("rf").p, rp_ref_out.getP("lf").p, rp_ref_out.getP("com").p);

      cam_rpy_filtered = cam_rpy_filter.passFilter(head_cam_pose.rpy);
      r_zmp_raw = rp_ref_out.getP("zmp").p;
      applyZMPCalcFromCOM                 (rp_ref_out.getP("com").p,rp_ref_out.getP("zmp").p);
      applyVelLimit                       (rp_ref_out, rp_ref_out_old, rp_ref_out);
      applyCOMZMPXYZLock                  (rp_ref_out);
      overwriteFootZFromFootLandOnCommand (go_rf_landing, go_lf_landing, rp_ref_out);
      com_vel_old = (rp_ref_out.getP("com").p - rp_ref_out_old.getP("com").p)/DT;

      rp_ref_out_old = rp_ref_out;
      loop++;
      gettimeofday(&t_calc_end, NULL);
    }
    void calibInitHumanCOMFromZMP(){
      init_hp_calibcom(0) = r_zmp_raw(0);
      init_hp_calibcom(1) = r_zmp_raw(1);
    }
    static void Point3DToVector3(const RTC::Point3D& in, hrp::Vector3& out){ out(0) = in.x; out(1) = in.y; out(2) = in.z;}
    static void Oriantation3DToVector3(const RTC::Orientation3D& in, hrp::Vector3& out){ out(0) = in.r; out(1) = in.p; out(2) = in.y;}
    static void Pose3DToHRPPose3D(const RTC::Pose3D& in, HRPPose3D& out_pose){ Point3DToVector3(in.position,out_pose.p); Oriantation3DToVector3(in.orientation,out_pose.rpy); }
    static void DoubleSeqToWrench6(const RTC::TimedDoubleSeq::_data_seq& in, Wrench6& out){
      if(in.length() == 6){out.f(0)=in[0]; out.f(1)=in[1]; out.f(2)=in[2]; out.t(0)=in[3]; out.t(1)=in[4]; out.t(2)=in[5];
      }else{ std::cerr<<"[WARN] HumanPose::Wrench6 DoubleSeqToWrench6() invalid data length"<<std::endl; out.clear(); }
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
      std::string ns[5] = {"com","rf","lf","rh","lh"};
      for(int i=0;i<5;i++){
        rel_out.getP(ns[i]).p = abs_in.getP(ns[i]).p - init_offset.getP(ns[i]).p;
        rel_out.getP(ns[i]).rpy = abs_in.getP(ns[i]).rpy;
      }
      rel_out.getw("rfw") = abs_in.getw("rfw");
      rel_out.getw("lfw") = abs_in.getw("lfw");
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
      out.getP("com").p = in.getP("com").p + zmp_com_offset_ip_ratio * com_offs;
    }
    void applyLPFilter(HumanPose& tgt){
      std::string ns[5] = {"com","rf","lf","rh","lh"};
      for(int i=0;i<tgt_pos_filters.size();i++){ tgt.getP(ns[i]).p = tgt_pos_filters[i].passFilter(tgt.getP(ns[i]).p); }
      for(int i=0;i<tgt_rot_filters.size();i++){ tgt.getP(ns[i]).rpy = tgt_rot_filters[i].passFilter(tgt.getP(ns[i]).rpy); }
    }
    void convertRelHumanPoseToRelRobotPose(const HumanPose& hp_in, HumanPose& rp_out){//結局初期指定からの移動量(=Rel)で計算をしてゆく
      std::string ns[6] = {"com","rf","lf","rh","lh","zmp"};
      for(int i=0;i<6;i++){
        rp_out.getP(ns[i]).p =  h2r_ratio * hp_in.getP(ns[i]).p + rp_out.getP(ns[i]).offs;
        rp_out.getP(ns[i]).rpy =  hp_in.getP(ns[i]).rpy;
      }
      rp_out.getw("rfw")  = hp_in.getw("rfw");
      rp_out.getw("lfw")  = hp_in.getw("lfw");
    }
    void applyCOMMoveModRatio(HumanPose& tgt){//
      tgt.getP("com").p = cmmr * tgt.getP("com").p;
    }
    void judgeFootLandOnCommand(const Wrench6& rfw_in, const Wrench6& lfw_in, bool& rfloc_ans, bool& lfloc_ans){
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
        if(tgt.getP("zmp").p(1) > tgt.getP("rf").p(1) + 0.03){
          lfcs_ans = true;
          if(loop%100==0)cout<<"LF Auto lock"<<endl;
        }
      }
      if(!rfcs_ans && lfcs_ans){
        if(tgt.getP("zmp").p(1) < tgt.getP("lf").p(1) - 0.03){
          rfcs_ans = true;
          if(loop%100==0)cout<<"RF Auto lock"<<endl;
        }
      }
    }
    void overwriteFootZFromFootLandOnCommand(const bool& rf_goland_in, const bool& lf_goland_in, HumanPose& tgt){
      if(is_rf_contact&&is_lf_contact)FUP_TIME = tgt_FUP_TIME;//両足接地しているタイミングでのみ更新
      double RFDOWN_TIME = fabs(tgt.getP("rf").p(1) - tgt.getP("com").p(1))*2 + FUP_TIME;//下ろす速度は遊脚の重心からの距離に比例したペナルティ
      double LFDOWN_TIME = fabs(tgt.getP("lf").p(1) - tgt.getP("com").p(1))*2 + FUP_TIME;
      LIMIT_MINMAX( RFDOWN_TIME, FUP_TIME, 0.8);
      LIMIT_MINMAX( LFDOWN_TIME, FUP_TIME, 0.8);
      calcFootUpCurveAndJudgeFootContact(rf_goland_in, FUP_TIME, RFDOWN_TIME, cur_rfup_rad, tgt.getP("rf"), is_rf_contact);
      calcFootUpCurveAndJudgeFootContact(lf_goland_in, FUP_TIME, LFDOWN_TIME, cur_lfup_rad, tgt.getP("lf"), is_lf_contact);
    }
    void calcFootUpCurveAndJudgeFootContact(const bool& go_land_in, const double& fup_time_in, const double& fdown_time_in,  double& cur_fup_phase_in, HRPPose3D& f_height_out, bool& is_f_contact_out){
      const double omega_up = M_PI / fup_time_in;
      const double omega_down = M_PI / fdown_time_in;
      if(!go_land_in){//足上げ命令入力時
        cur_fup_phase_in += omega_up * DT;
      }else{//足下げ命令入力時
        cur_fup_phase_in -= omega_down * DT;
      }
      if(cur_fup_phase_in > M_PI){cur_fup_phase_in = M_PI;}
      else if(cur_fup_phase_in < 0){cur_fup_phase_in = 0;}
      double fup_height = f_height_out.p(2)-f_height_out.offs(2);
      LIMIT_MINMAX( fup_height, 0.03, 0.10 );
      f_height_out.p(2) = fup_height/2 * (1-cos(cur_fup_phase_in)) + f_height_out.offs(2);
      is_f_contact_out = (cur_fup_phase_in <= 0);
    }
    void lockFootXYOnContact(const bool& rfcs_in, const bool& lfcs_in, HumanPose& tgt){
      if(rfcs_in){
        tgt.getP("rf").p(0) = pre_cont_rfpos(0);
        tgt.getP("rf").p(1) = pre_cont_rfpos(1);
      }else{
        pre_cont_rfpos(0) = tgt.getP("rf").p(0);
        pre_cont_rfpos(1) = tgt.getP("rf").p(1);
      }
      if(lfcs_in){
        tgt.getP("lf").p(0) = pre_cont_lfpos(0);
        tgt.getP("lf").p(1) = pre_cont_lfpos(1);
      }else{
        pre_cont_lfpos(0) = tgt.getP("lf").p(0);
        pre_cont_lfpos(1) = tgt.getP("lf").p(1);
      }
    }
    void applyEEWorkspaceLimit(HumanPose& tgt){
      const double MAX = 0.30;
      if(!is_rf_contact && is_lf_contact){//右足浮遊時
        const hrp::Vector2 lf2rf_vec( tgt.getP("rf").p(0)-pre_cont_lfpos(0), tgt.getP("rf").p(1)-pre_cont_lfpos(1) );
        if(lf2rf_vec.norm() > MAX){
          tgt.getP("rf").p(0) = (lf2rf_vec.normalized())(0) * MAX + pre_cont_lfpos(0);
          tgt.getP("rf").p(1) = (lf2rf_vec.normalized())(1) * MAX + pre_cont_lfpos(1);
        }
        if(tgt.getP("rf").p(1) > pre_cont_lfpos(1) + 0.05 ) tgt.getP("rf").p(1) = pre_cont_lfpos(1) + 0.05;
      }
      else if(is_rf_contact && !is_lf_contact){//左足浮遊時
        const hrp::Vector2 rf2lf_vec( tgt.getP("lf").p(0)-pre_cont_rfpos(0), tgt.getP("lf").p(1)-pre_cont_rfpos(1) );
        if(rf2lf_vec.norm() > MAX){
          tgt.getP("lf").p(0) = (rf2lf_vec.normalized())(0) * MAX + pre_cont_rfpos(0);
          tgt.getP("lf").p(1) = (rf2lf_vec.normalized())(1) * MAX + pre_cont_rfpos(1);
        }
        if(tgt.getP("lf").p(1) < pre_cont_rfpos(1) - 0.05 ) tgt.getP("lf").p(1) = pre_cont_rfpos(1) - 0.05;
      }
      hrp::Vector3 base2rh = tgt.getP("rh").offs - init_basepos;
      hrp::Vector3 base2lh = tgt.getP("lh").offs - init_basepos;
      hrp::Vector3 hand_ulimit = hrp::Vector3(0.1,0.1,0.1);
      hrp::Vector3 hand_llimit = hrp::Vector3(-0.1,-0.1,-0.1);
      for(int i=0;i<3;i++){
        LIMIT_MINMAX( tgt.getP("rh").p(i), (current_basepos+base2rh+hand_llimit)(i), (current_basepos+base2rh+hand_ulimit)(i) );
        LIMIT_MINMAX( tgt.getP("lh").p(i), (current_basepos+base2lh+hand_llimit)(i), (current_basepos+base2lh+hand_ulimit)(i) );
        LIMIT_MINMAX( tgt.getP("rf").p(2), tgt.getP("rf").offs(2), tgt.getP("rf").offs(2)+0.10);
        LIMIT_MINMAX( tgt.getP("lf").p(2), tgt.getP("lf").offs(2), tgt.getP("lf").offs(2)+0.10);
      }
      LIMIT_MINMAX( head_cam_pose.rpy(1), -20*M_PI/180, 20*M_PI/180 );
      LIMIT_MINMAX( head_cam_pose.rpy(2), -20*M_PI/180, 20*M_PI/180 );
    }
    bool applyCOMToSupportRegionLimit(const hrp::Vector3& rfin_abs, const hrp::Vector3& lfin_abs, HRPPose3D& comin_abs){//boost::geometryがUbuntu12だとないから・・・
      std::vector<hrp::Vector2> hull;
      createSupportRegionByFootPos(rfin_abs, lfin_abs, rf_safe_region, lf_safe_region, hull);
      hrp::Vector2 cog(comin_abs.p(0),comin_abs.p(1));
      if(!isPointInHullOpenCV(cog,hull)){ calcNearestPointOnHull(cog,hull,cog); }//外に出たら最近傍点に頭打ち
      comin_abs.p(0) = cog(0);
      comin_abs.p(1) = cog(1);
      LIMIT_MINMAX( comin_abs.p(2), comin_abs.offs(2) - 0.15, comin_abs.offs(2) + 0.03 );
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
      const double H = rp_wld_initpos.getP("com").p(2);
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
      const double H = rp_wld_initpos.getP("com").p(2);
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
      const double H = rp_wld_initpos.getP("com").p(2);
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
    void calcWorldZMP(const hrp::Vector3& rfpos, const hrp::Vector3& lfpos, const Wrench6& rfwin, const Wrench6& lfwin, hrp::Vector3& zmp_ans);
    void createSupportRegionByFootPos(const hrp::Vector3& rfin_abs, const hrp::Vector3& lfin_abs, const hrp::Vector4& rf_mgn, const hrp::Vector4& lf_mgn, std::vector<hrp::Vector2>& hull_ans);
    void calcXYMarginToHull(const hrp::Vector2& check_point, const std::vector<hrp::Vector2>& hull, hrp::Vector4& margin_ans);
    bool calcCrossPointOnHull(const hrp::Vector2& pt_in_start, const hrp::Vector2& pt_out_goal, const std::vector<hrp::Vector2>& hull, hrp::Vector2& pt_will_cross);
    double calcNearestPointOnHull(const hrp::Vector2& tgt_pt, const std::vector<hrp::Vector2>& hull, hrp::Vector2& pt_ans);
    void makeConvexHullOpenCV(const std::vector<hrp::Vector2>& pts, std::vector<hrp::Vector2>& hull_ans);
    void makeConvexHullQHull(const std::vector<hrp::Vector2>& pts, std::vector<hrp::Vector2>& hull_ans);
    bool isPointInHullOpenCV(const hrp::Vector2& pt, const std::vector<hrp::Vector2>& hull);
    void applyZMPCalcFromCOM(const hrp::Vector3& comin, hrp::Vector3& zmpout);
    void applyVelLimit(const HumanPose& in, const HumanPose& in_old, HumanPose& out);
    void applyCOMZMPXYZLock(HumanPose& tgt);
};



#endif // HUMANMASTERSLAVE_H
