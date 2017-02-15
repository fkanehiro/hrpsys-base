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

#define LIMIT_MIN(x,min) (x= ( x<min ? min:x ))
#define LIMIT_MAX(x,max) (x= ( x<max ? x:max ))
#define LIMIT_MINMAX(x,min,max) ((x= (x<min  ? min : x<max ? x : max)))

//実機だとログ出力で落ちる？getenv("HOME")ダメ？
#define DEBUG 0

namespace myconst{
  const int X = 0, Y = 1, Z = 2, XYZ = 3;
  const int r = 0, p = 1, y = 2, rpy = 3;
  const int L = 0, R = 1, LR = 2;
  const int MIN = 0, MAX = 1, MINMAX = 2;
  const double INFMIN = std::numeric_limits<double>::min(), INFMAX = std::numeric_limits<double>::max();
  const double D2R = M_PI/180.0;
}
using namespace myconst;

class BiquadIIRFilterVec{
  private:
    IIRFilter filters[3];
    hrp::Vector3 ans;
    std::vector<double> fb_coeffs, ff_coeffs;
  public:
    BiquadIIRFilterVec(const std::string& error_prefix = ""){
    };
    ~BiquadIIRFilterVec() {};
    void setParameter(const hrp::Vector3 fc_in, const double HZ){
      ff_coeffs.resize(3);
      fb_coeffs.resize(3);
      for(int i=0;i<3;i++){
        const double fc = tan(fc_in(i) * M_PI / HZ) / (2 * M_PI);
        const double denom = 1 + (2 * sqrt(2) * M_PI * fc) + 4 * M_PI * M_PI * fc*fc;
        ff_coeffs[0] = (4 * M_PI * M_PI * fc*fc) / denom;
        ff_coeffs[1] = (8 * M_PI * M_PI * fc*fc) / denom;
        ff_coeffs[2] = (4 * M_PI * M_PI * fc*fc) / denom;
        fb_coeffs[0] = 1.0;
        fb_coeffs[1] = (8 * M_PI * M_PI * fc*fc - 2) / denom;
        fb_coeffs[2] = (1 - (2 * sqrt(2) * M_PI * fc) + 4 * M_PI * M_PI * fc*fc) / denom;
        filters[i].setParameter(2,fb_coeffs,ff_coeffs);
      }
    };
    void setParameter(const double fc_in, const double HZ){
      setParameter(hrp::Vector3(fc_in,fc_in,fc_in), HZ);
    };
    hrp::Vector3 passFilter(const hrp::Vector3& input){
      for(int i=0;i<3;i++){ ans(i) = filters[i].passFilter(input(i)); }
      return ans;
    };
};

class HRPPose3D{
  public:
    hrp::Vector3 p,p_offs,rpy,rpy_offs;
    hrp::Matrix33 R;
    HRPPose3D(){ clear(); }
    ~HRPPose3D(){}
    void clear(){
      p = p_offs = rpy = hrp::Vector3::Zero();
      R = hrp::Matrix33::Zero();
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
  private:
    std::map<std::string, HRPPose3D> P;
    std::map<std::string, Wrench6> w;
  public:
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
      for(int i=0;i<6;i++){ printf("\x1b[31m%s\x1b[39m%+05.2f %+05.2f %+05.2f ",cap[i].c_str(),in.getP(ns[i]).p(X),in.getP(ns[i]).p(Y),in.getP(ns[i]).p(Z)); }
      for(int i=6;i<8;i++){ printf("\x1b[31m%s\x1b[39m%+05.1f %+05.1f %+05.1f %+05.1f %+05.1f %+05.1f ",cap[i].c_str(),in.getw(ns[i]).f(X),in.getw(ns[i]).f(Y),in.getw(ns[i]).f(Z),in.getw(ns[i]).t(X),in.getw(ns[i]).t(Y),in.getw(ns[i]).t(Z)); }
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

class RobotConfig{
  public:
    std::map<std::string, std::vector<hrp::Vector3> > ee_rot_limit;
    RobotConfig(){
      ee_rot_limit["com"].resize(MINMAX);
      ee_rot_limit["com"][MIN] = hrp::Vector3(-30*D2R, -30*D2R, INFMIN);
      ee_rot_limit["com"][MAX] = hrp::Vector3( 30*D2R,  30*D2R, INFMAX);
      ee_rot_limit["rf"].resize(MINMAX);
      ee_rot_limit["rf"][MIN] = hrp::Vector3(-30*D2R, -30*D2R, -20*D2R);
      ee_rot_limit["rf"][MAX] = hrp::Vector3( 30*D2R,  30*D2R,   5*D2R);
      ee_rot_limit["lf"].resize(MINMAX);
      ee_rot_limit["lf"][MIN] = hrp::Vector3(-30*D2R, -30*D2R,  -5*D2R);
      ee_rot_limit["lf"][MAX] = hrp::Vector3( 30*D2R,  30*D2R,  20*D2R);
      ee_rot_limit["rh"].resize(MINMAX);
      ee_rot_limit["rh"][MIN] = hrp::Vector3(-30*D2R, -30*D2R, -30*D2R);
      ee_rot_limit["rh"][MAX] = hrp::Vector3( 30*D2R,  30*D2R,  30*D2R);
      ee_rot_limit["lh"].resize(MINMAX);
      ee_rot_limit["lh"][MIN] = hrp::Vector3(-30*D2R, -30*D2R, -30*D2R);
      ee_rot_limit["lh"][MAX] = hrp::Vector3( 30*D2R,  30*D2R,  30*D2R);
    }
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
    RobotConfig rc;

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
    hrp::Vector3 pre_cont_rfpos,pre_cont_lfpos, pre_cont_rfrot,pre_cont_lfrot;
    std::vector<hrp::Vector3> rf_vert,lf_vert;

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
      pre_cont_rfpos = pre_cont_lfpos = pre_cont_rfrot = pre_cont_lfrot = hrp::Vector3::Zero();
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
      for(int i=0;i<tgt_pos_filters.size();i++)tgt_pos_filters[i].setParameter(0.6,HZ);//四肢拘束点用(position)
      for(int i=0;i<tgt_rot_filters.size();i++)tgt_rot_filters[i].setParameter(1.0,HZ);//四肢拘束点用(Rotation)
      tgt_pos_filters[0].setParameter(0.4,HZ);//重心pos用
      tgt_rot_filters[0].setParameter(0.4,HZ);//重心rot用
      tgt_pos_filters[1].setParameter(hrp::Vector3(0.6,0.6,1.0),HZ);//右足pos用
      tgt_pos_filters[2].setParameter(hrp::Vector3(0.6,0.6,1.0),HZ);//左足pos用
      calcacc_v_filters.setParameter(5,HZ);//加速度計算用
      acc4zmp_v_filters.setParameter(1,HZ);//ZMP生成用ほぼこの値でいい
      cam_rpy_filter.setParameter(1,HZ);//カメラアングル

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

      use_rh = use_lh = false;
//      use_rh = use_lh = true;

      rp_ref_out_old.clear();
      rp_ref_out.clear();

      init_zmp_com_offset_interpolator = new interpolator(1, DT, interpolator::HOFFARBIB, 1);
      init_zmp_com_offset_interpolator->setName("zmp_com_offset_interpolator");
      double start_ratio = 0.0;
      init_zmp_com_offset_interpolator->set(&start_ratio);
      double goal_ratio = 1.0;
      init_zmp_com_offset_interpolator->go(&goal_ratio, 3.0, true);//3s

      if(DEBUG){
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
      if(loop==0)rp_ref_out_old = rp_ref_out;

      applyCOMMoveModRatio                (rp_ref_out);
      judgeFootLandOnCommand              (hp_wld_raw.getw("rfw"), hp_wld_raw.getw("lfw"), go_rf_landing, go_lf_landing);//fwはすでに1000->100Hzにフィルタリングされている
      lockSwingFootIfZMPOutOfSupportFoot  (rp_ref_out_old, go_rf_landing, go_lf_landing);//ここ
      applyEEWorkspaceLimit               (rp_ref_out);
      lockFootXYOnContact                 (go_rf_landing, go_lf_landing, rp_ref_out);//根本から改変すべき3
      if(go_rf_landing)rp_ref_out.getP("rf").p(Z) = rp_ref_out.getP("rf").p_offs(Z);
      if(go_lf_landing)rp_ref_out.getP("lf").p(Z) = rp_ref_out.getP("lf").p_offs(Z);

      applyCOMToSupportRegionLimit        (rp_ref_out.getP("rf").p, rp_ref_out.getP("lf").p, rp_ref_out.getP("com"));

      setFootRotHorizontalIfGoLanding     (rp_ref_out_old, rp_ref_out);
      applyLPFilter                       (rp_ref_out);
      applyCOMStateLimitByCapturePoint    (rp_ref_out.getP("com").p, rp_ref_out_old.getP("com").p, com_vel_old, rp_ref_out.getP("rf").p, rp_ref_out.getP("lf").p, rp_ref_out.getP("com").p);

      cam_rpy_filtered = cam_rpy_filter.passFilter(head_cam_pose.rpy);
      r_zmp_raw = rp_ref_out.getP("zmp").p;
      applyZMPCalcFromCOM                 (rp_ref_out.getP("com").p,rp_ref_out.getP("zmp").p);
      applyVelLimit                       (rp_ref_out, rp_ref_out_old, rp_ref_out);
      applyCOMZMPXYZLock                  (rp_ref_out);
      overwriteFootZFromFootLandOnCommand (go_rf_landing, go_lf_landing, rp_ref_out);
      modifyFootRotAndXYForContact        (rp_ref_out.getP("rf"), rp_ref_out.getP("lf"));
      com_vel_old = (rp_ref_out.getP("com").p - rp_ref_out_old.getP("com").p)/DT;

      rp_ref_out_old = rp_ref_out;
      loop++;
      gettimeofday(&t_calc_end, NULL);
    }
    void calibInitHumanCOMFromZMP(){
      init_hp_calibcom(X) = r_zmp_raw(X);
      init_hp_calibcom(Y) = r_zmp_raw(Y);
    }
    static void Point3DToVector3(const RTC::Point3D& in, hrp::Vector3& out){ out(X) = in.x; out(Y) = in.y; out(Z) = in.z;}
    static void Oriantation3DToVector3(const RTC::Orientation3D& in, hrp::Vector3& out){ out(X) = in.r; out(Y) = in.p; out(Z) = in.y;}
    static void Pose3DToHRPPose3D(const RTC::Pose3D& in, HRPPose3D& out_pose){ Point3DToVector3(in.position,out_pose.p); Oriantation3DToVector3(in.orientation,out_pose.rpy); }
    static void DoubleSeqToWrench6(const RTC::TimedDoubleSeq::_data_seq& in, Wrench6& out){
      if(in.length() == 6){out.f(X)=in[0]; out.f(Y)=in[1]; out.f(Z)=in[2]; out.t(X)=in[3]; out.t(Y)=in[4]; out.t(Z)=in[5];
      }else{ std::cerr<<"[WARN] HumanPose::Wrench6 DoubleSeqToWrench6() invalid data length"<<std::endl; out.clear(); }
    }
    static double hrpVector2Cross(const hrp::Vector2& a, const hrp::Vector2& b){ return a(X)*b(Y)-a(Y)*b(X); }

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
        rel_out.getP(ns[i]).p   = abs_in.getP(ns[i]).p   - init_offset.getP(ns[i]).p;
        rel_out.getP(ns[i]).rpy = abs_in.getP(ns[i]).rpy - init_offset.getP(ns[i]).rpy;
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
        rp_out.getP(ns[i]).p   =  h2r_ratio * hp_in.getP(ns[i]).p   + rp_out.getP(ns[i]).p_offs;
        rp_out.getP(ns[i]).rpy =  hrp::rpyFromRot( hrp::rotFromRpy(hp_in.getP(ns[i]).rpy) * hrp::rotFromRpy(rp_out.getP(ns[i]).rpy_offs) );
//        rp_out.getP(ns[i]).rpy =         hrp::rpyFromRot(        hrp::rotFromRpy(rp_out.getP(ns[i]).rpy_offs) * hrp::rotFromRpy(hp_in.getP(ns[i]).rpy) );
      }
      rp_out.getw("rfw")  = hp_in.getw("rfw");
      rp_out.getw("lfw")  = hp_in.getw("lfw");
    }
    void applyCOMMoveModRatio(HumanPose& tgt){//
      tgt.getP("com").p = cmmr * tgt.getP("com").p;
    }
    void judgeFootLandOnCommand(const Wrench6& rfw_in, const Wrench6& lfw_in, bool& rfloc_ans, bool& lfloc_ans){
      if(isHumanSyncOn()){
        if      (rfloc_ans  && rfw_in.f(Z)<CNT_F_TH   ){rfloc_ans = false;}//右足ついた状態から上げる
        else if (!rfloc_ans && rfw_in.f(Z)>CNT_F_TH+30){rfloc_ans = true;}//右足浮いた状態から下げる
        if      (lfloc_ans  && lfw_in.f(Z)<CNT_F_TH   ){lfloc_ans = false;}//左足ついた状態から上げる
        else if (!lfloc_ans && lfw_in.f(Z)>CNT_F_TH+30){lfloc_ans = true;}//左足浮いた状態から下げる
      }
//      if(!rfloc_ans && !lfloc_ans){rfloc_ans = lfloc_ans = true;}//両足ジャンプは禁止
    }
    void lockSwingFootIfZMPOutOfSupportFoot(const HumanPose& tgt, bool& rfcs_ans, bool& lfcs_ans){
//      if(rfcs_ans && !lfcs_ans){
        if(tgt.getP("zmp").p(Y) > tgt.getP("rf").p(Y) + 0.03){
          lfcs_ans = true;
          if(loop%100==0)cout<<"LF Auto lock"<<endl;
        }
//      }
//      if(!rfcs_ans && lfcs_ans){
        if(tgt.getP("zmp").p(Y) < tgt.getP("lf").p(Y) - 0.03){
          rfcs_ans = true;
          if(loop%100==0)cout<<"RF Auto lock"<<endl;
        }
//      }
    }
    void setFootRotHorizontalIfGoLanding(const HumanPose& rp_ref_out_old, HumanPose& rp_ref_out){
//      if(rp_ref_out_old.getP("rf").p(Z)-rp_ref_out_old.getP("rf").p_offs(Z)<0.05){ rp_ref_out.getP("rf").rpy(X) = rp_ref_out.getP("rf").rpy(Y) = 0; }
//      if(rp_ref_out_old.getP("lf").p(Z)-rp_ref_out_old.getP("lf").p_offs(Z)<0.05){ rp_ref_out.getP("lf").rpy(X) = rp_ref_out.getP("lf").rpy(Y) = 0; }
      if(go_rf_landing){ rp_ref_out.getP("rf").rpy(X) = rp_ref_out.getP("rf").rpy(Y) = 0; }
      if(go_lf_landing){ rp_ref_out.getP("lf").rpy(X) = rp_ref_out.getP("lf").rpy(Y) = 0; }
    }
    void overwriteFootZFromFootLandOnCommand(const bool& rf_goland_in, const bool& lf_goland_in, HumanPose& tgt){
//      if(is_rf_contact&&is_lf_contact)FUP_TIME = tgt_FUP_TIME;//両足接地しているタイミングでのみ更新
//      double RFDOWN_TIME = fabs(tgt.getP("rf").p(Y) - tgt.getP("com").p(Y))*2 + FUP_TIME;//下ろす速度は遊脚の重心からの距離に比例したペナルティ
//      double LFDOWN_TIME = fabs(tgt.getP("lf").p(Y) - tgt.getP("com").p(Y))*2 + FUP_TIME;
//      LIMIT_MINMAX( RFDOWN_TIME, FUP_TIME, 0.8);
//      LIMIT_MINMAX( LFDOWN_TIME, FUP_TIME, 0.8);
//      calcFootUpCurveAndJudgeFootContact(rf_goland_in, FUP_TIME, RFDOWN_TIME, cur_rfup_rad, tgt.getP("rf"), is_rf_contact);
//      calcFootUpCurveAndJudgeFootContact(lf_goland_in, FUP_TIME, LFDOWN_TIME, cur_lfup_rad, tgt.getP("lf"), is_lf_contact);
      const double com2rf_dist = fabs(tgt.getP("rf").p(Y) - tgt.getP("com").p(Y));
      const double com2lf_dist = fabs(tgt.getP("lf").p(Y) - tgt.getP("com").p(Y));
      limitGroundContactVelocity(rf_goland_in, com2rf_dist, rp_ref_out_old.getP("rf"), tgt.getP("rf"), is_rf_contact);
      limitGroundContactVelocity(lf_goland_in, com2lf_dist, rp_ref_out_old.getP("lf"), tgt.getP("lf"), is_lf_contact);
    }
//    void calcFootUpCurveAndJudgeFootContact(const bool& go_land_in, const double& fup_time_in, const double& fdown_time_in,  double& cur_fup_phase_in, HRPPose3D& f_height_out, bool& is_f_contact_out){
//      const double omega_up = M_PI / fup_time_in;
//      const double omega_down = M_PI / fdown_time_in;
//      if(!go_land_in){//足上げ命令入力時
//        cur_fup_phase_in += omega_up * DT;
//      }else{//足下げ命令入力時
//        cur_fup_phase_in -= omega_down * DT;
//      }
//      LIMIT_MINMAX( cur_fup_phase_in, 0, M_PI);
//      double fup_height = f_height_out.p(Z)-f_height_out.p_offs(Z);
//      LIMIT_MINMAX( fup_height, 0.03, 0.10 );
//      f_height_out.p(Z) = fup_height/2 * (1-cos(cur_fup_phase_in)) + f_height_out.p_offs(Z);
//      is_f_contact_out = (cur_fup_phase_in <= 0);
//    }
    void limitGroundContactVelocity(const bool& go_land_in, const double& com2foot_dist, const HRPPose3D& f_old_in, HRPPose3D& f_in_out, bool& is_f_contact_out){
      double penalty = 1.0 + (0.2 - com2foot_dist) * 5.0;//1.0 ~ 2.0
      LIMIT_MINMAX( penalty, 1.0, 2.0);
      LIMIT_MIN( f_in_out.p(Z), f_in_out.p_offs(Z));
      double input_vel = (f_in_out.p(Z) - f_old_in.p(Z)) / DT;
      const double vel_limit_k = 8.0 * penalty;//地面から0.02[m]地点で0.02*8=0.16[m/s]出ている計算
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
        tgt.getP("rf").p(X) = pre_cont_rfpos(X);
        tgt.getP("rf").p(Y) = pre_cont_rfpos(Y);
        tgt.getP("rf").rpy(Z) = pre_cont_rfrot(Z);
      }else{
        pre_cont_rfpos(X) = tgt.getP("rf").p(X);
        pre_cont_rfpos(Y) = tgt.getP("rf").p(Y);
        pre_cont_rfrot(Z) = tgt.getP("rf").rpy(Z);
      }
      if(lfcs_in){
        tgt.getP("lf").p(X) = pre_cont_lfpos(X);
        tgt.getP("lf").p(Y) = pre_cont_lfpos(Y);
        tgt.getP("lf").rpy(Z) = pre_cont_lfrot(Z);
      }else{
        pre_cont_lfpos(X) = tgt.getP("lf").p(X);
        pre_cont_lfpos(Y) = tgt.getP("lf").p(Y);
        pre_cont_lfrot(Z) = tgt.getP("lf").rpy(Z);
      }
    }
    void applyEEWorkspaceLimit(HumanPose& tgt){
      const double MAX_FW = 0.30;
      const double FOOT_2_FOOT_COLLISION_MARGIIN = 0.16;
      if(!is_rf_contact && is_lf_contact){//右足浮遊時
        const hrp::Vector2 lf2rf_vec( tgt.getP("rf").p(X)-pre_cont_lfpos(X), tgt.getP("rf").p(Y)-pre_cont_lfpos(Y) );
        if(lf2rf_vec.norm() > MAX_FW){
          tgt.getP("rf").p(X) = (lf2rf_vec.normalized())(X) * MAX_FW + pre_cont_lfpos(X);
          tgt.getP("rf").p(Y) = (lf2rf_vec.normalized())(Y) * MAX_FW + pre_cont_lfpos(Y);
        }
        LIMIT_MAX(tgt.getP("rf").p(Y), pre_cont_lfpos(Y) - FOOT_2_FOOT_COLLISION_MARGIIN);
      }
      else if(is_rf_contact && !is_lf_contact){//左足浮遊時
        const hrp::Vector2 rf2lf_vec( tgt.getP("lf").p(X)-pre_cont_rfpos(X), tgt.getP("lf").p(Y)-pre_cont_rfpos(Y) );
        if(rf2lf_vec.norm() > MAX_FW){
          tgt.getP("lf").p(X) = (rf2lf_vec.normalized())(X) * MAX_FW + pre_cont_rfpos(X);
          tgt.getP("lf").p(Y) = (rf2lf_vec.normalized())(Y) * MAX_FW + pre_cont_rfpos(Y);
        }
        LIMIT_MIN(tgt.getP("lf").p(Y), pre_cont_rfpos(Y) + FOOT_2_FOOT_COLLISION_MARGIIN);
      }
      hrp::Vector3 base2rh = tgt.getP("rh").p_offs - init_basepos;
      hrp::Vector3 base2lh = tgt.getP("lh").p_offs - init_basepos;
      hrp::Vector3 hand_ulimit = hrp::Vector3(0.1,0.1,0.1);
      hrp::Vector3 hand_llimit = hrp::Vector3(-0.1,-0.1,-0.1);
      for(int i=0;i<3;i++){
        LIMIT_MINMAX( tgt.getP("rh").p(i), (current_basepos+base2rh+hand_llimit)(i), (current_basepos+base2rh+hand_ulimit)(i) );
        LIMIT_MINMAX( tgt.getP("lh").p(i), (current_basepos+base2lh+hand_llimit)(i), (current_basepos+base2lh+hand_ulimit)(i) );
      }
      LIMIT_MINMAX( tgt.getP("rf").p(Z), tgt.getP("rf").p_offs(Z), tgt.getP("rf").p_offs(Z)+0.10);
      LIMIT_MINMAX( tgt.getP("lf").p(Z), tgt.getP("lf").p_offs(Z), tgt.getP("lf").p_offs(Z)+0.10);


      std::string ns[5] = {"com","rf","lf","rh","lh"};

//
//      for(int i=0;i<5;i++){
//        LIMIT_MINMAX( tgt.getP(ns[i]).rpy(X), -30*M_PI/180, 30*M_PI/180 );
//        LIMIT_MINMAX( tgt.getP(ns[i]).rpy(Y), -30*M_PI/180, 30*M_PI/180 );
//      }
//      LIMIT_MINMAX( tgt.getP("rf").rpy(Z), -30*M_PI/180+tgt.getP("com").rpy(Z), 30*M_PI/180+tgt.getP("com").rpy(Z) );
//      LIMIT_MINMAX( tgt.getP("lf").rpy(Z), -30*M_PI/180+tgt.getP("com").rpy(Z), 30*M_PI/180+tgt.getP("com").rpy(Z) );
//      LIMIT_MINMAX( tgt.getP("rh").rpy(Z), -30*M_PI/180+tgt.getP("com").rpy(Z), 30*M_PI/180+tgt.getP("com").rpy(Z) );
//      LIMIT_MINMAX( tgt.getP("lh").rpy(Z), -30*M_PI/180+tgt.getP("com").rpy(Z), 30*M_PI/180+tgt.getP("com").rpy(Z) );

      for(int i=0;i<XYZ;i++){ LIMIT_MINMAX( tgt.getP("com").rpy(i), rc.ee_rot_limit["com"][MIN](i), rc.ee_rot_limit["com"][MAX](i) ); }
      for(int i=1;i<5;i++){
        for(int j=0;j<XYZ;j++){ LIMIT_MINMAX( tgt.getP(ns[i]).rpy(j), rc.ee_rot_limit[ns[i]][MIN](j) + tgt.getP("com").rpy(j), rc.ee_rot_limit[ns[i]][MAX](j) + tgt.getP("com").rpy(j) ); }
      }

      LIMIT_MINMAX( head_cam_pose.rpy(Y), -20*M_PI/180, 20*M_PI/180 );
      LIMIT_MINMAX( head_cam_pose.rpy(Z), -20*M_PI/180, 20*M_PI/180 );
    }
    bool applyCOMToSupportRegionLimit(const hrp::Vector3& rfin_abs, const hrp::Vector3& lfin_abs, HRPPose3D& comin_abs){//boost::geometryがUbuntu12だとないから・・・
      std::vector<hrp::Vector2> hull;
      createSupportRegionByFootPos(rfin_abs, lfin_abs, rf_safe_region, lf_safe_region, hull);
      hrp::Vector2 cog(comin_abs.p(X),comin_abs.p(Y));
      if(!isPointInHullOpenCV(cog,hull)){ calcNearestPointOnHull(cog,hull,cog); }//外に出たら最近傍点に頭打ち
      comin_abs.p(X) = cog(X);
      comin_abs.p(Y) = cog(Y);
      LIMIT_MINMAX( comin_abs.p(Z), comin_abs.p_offs(Z) - 0.15, comin_abs.p_offs(Z) + 0.03 );//COM高さ方向の制限
      return true;
    }
    bool applyCOMStateLimitByCapturePoint(const hrp::Vector3& com_in, const hrp::Vector3& com_old, const hrp::Vector3& com_vel_old, const hrp::Vector3& rfin_abs, const hrp::Vector3& lfin_abs, hrp::Vector3& com_ans){
      com_ans = com_in;
      hrp::Vector3 com_vel = (com_in - com_old)/DT;
//      hrp::Vector3 com_acc = (com_vel - com_vel_old)/DT;//命名規則
//      hrp::Vector3 com_vel_zmpOk, com_vel_zmpOk_cpOk;
//      double accel[4] = {0.05, -0.03, 0.02, -0.02};
//      double decel[4] = {0.04, -0.02, 0.01, -0.01};
//      comacc_ref = calcacc_v_filters.passFilter(com_acc);
      const double H = rp_wld_initpos.getP("com").p(Z);
      std::vector<hrp::Vector2> hull;
      hrp::Vector4 marginDelta(+0.001,-0.001,0.001,-0.001);
      createSupportRegionByFootPos(rfin_abs, lfin_abs, rf_safe_region+marginDelta, lf_safe_region+marginDelta, hull);
      hrp::Vector2 com_vel_ans_2d;
      regulateCOMVelocityByCapturePointVec( hrp::Vector2(com_old(X),com_old(Y)), hrp::Vector2(com_vel(X),com_vel(Y)), hull, com_vel_ans_2d);
//      regulateCOMVelocityByCapturePointXY( hrp::Vector2(com_old(X),com_old(Y)), hrp::Vector2(com_vel(X),com_vel(Y)), hull, com_vel_ans_2d);//XY独立に計算する手法のほうがエッジで引っかからない・・・
      com_ans(X) = com_old(X) + com_vel_ans_2d(X) * DT;
      com_ans(Y) = com_old(Y) + com_vel_ans_2d(Y) * DT;
      return true;
    }
    void regulateCOMVelocityByCapturePointVec(const hrp::Vector2& com_pos, const hrp::Vector2& com_vel, const std::vector<hrp::Vector2>& hull, hrp::Vector2& com_vel_ans){
      com_vel_ans = com_vel;
      const double H = rp_wld_initpos.getP("com").p(Z);
      hrp::Vector2 cp = com_pos + com_vel * sqrt( H / G );
      hrp::Vector2 cp_ragulated;
      if(!isPointInHullOpenCV(cp,hull)){
        calcCrossPointOnHull(com_pos, cp, hull, cp_ragulated);
        //calcNearestPointOnHull(cp, hull, cp_ragulated);
        com_vel_ans = (cp_ragulated - com_pos) / sqrt( H / G );
      }
    }
//    void regulateCOMVelocityByCapturePointXY(const hrp::Vector2& com_pos, const hrp::Vector2& com_vel, const std::vector<hrp::Vector2>& hull, hrp::Vector2& com_vel_ans){
//      com_vel_ans = com_vel;
//      const double H = rp_wld_initpos.getP("com").p(Z);
//      enum Direc{F,B,L,R} direc;
//      hrp::Vector4 margin_from_4maxcp;
//      calcXYMarginToHull(com_pos, hull, margin_from_4maxcp);
//      hrp::Vector4 com_vel_decel_limit,com_vel_accel_limit;
//      const double accel_cp_tolerance = 0.01;//境界上でcom_vel_accel_limitが0になって抜け出せなくなるのを防ぐため
//      com_vel_decel_limit(F) = margin_from_4maxcp(F) / sqrt( H / G );
//      com_vel_decel_limit(B) = margin_from_4maxcp(B) / sqrt( H / G );
//      com_vel_decel_limit(L) = margin_from_4maxcp(L) / sqrt( H / G );
//      com_vel_decel_limit(R) = margin_from_4maxcp(R) / sqrt( H / G );
//      com_vel_accel_limit(F) = (-margin_from_4maxcp(B)+accel_cp_tolerance) / sqrt( H / G );
//      com_vel_accel_limit(B) = (-margin_from_4maxcp(F)-accel_cp_tolerance) / sqrt( H / G );
//      com_vel_accel_limit(L) = (-margin_from_4maxcp(R)+accel_cp_tolerance) / sqrt( H / G );
//      com_vel_accel_limit(R) = (-margin_from_4maxcp(L)-accel_cp_tolerance) / sqrt( H / G );
//      LIMIT_MINMAX( com_vel_ans(X), com_vel_decel_limit(B), com_vel_decel_limit(F) );
//      LIMIT_MINMAX( com_vel_ans(Y), com_vel_decel_limit(R), com_vel_decel_limit(L) );
//      LIMIT_MINMAX( com_vel_ans(X), com_vel_accel_limit(B), com_vel_accel_limit(F) );//加速時のCP
//      LIMIT_MINMAX( com_vel_ans(Y), com_vel_accel_limit(R), com_vel_accel_limit(L) );
//    }
    void createSupportRegionByFootPos(const hrp::Vector3& rfin_abs, const hrp::Vector3& lfin_abs, const hrp::Vector4& rf_mgn, const hrp::Vector4& lf_mgn, std::vector<hrp::Vector2>& hull_ans){
      std::vector<hrp::Vector2> points;
      for(int i=0;i<2;i++){
        for(int j=2;j<4;j++){
          points.push_back(hrp::Vector2(rfin_abs(X) + rf_mgn(i),    rfin_abs(Y) + rf_mgn(j)));
          points.push_back(hrp::Vector2(lfin_abs(X) + lf_mgn(i),    lfin_abs(Y) + lf_mgn(j)));
        }
      }
//      static unsigned int edge_time = 0;
//      if(rp_ref_out_old.getP("rf").rpy(Y)>0.001){
//        points.clear();
//        double mod = rfin_abs(X) + 0.13 * edge_time * 0.002;
////        LIMIT_MAX( mod, rfin_abs(X) + 0.13 );
//        for(int i=0;i<2;i++){
//          for(int j=2;j<4;j++){
//            points.push_back(hrp::Vector2(rfin_abs(X) + 0.13,    rfin_abs(Y) + rf_mgn(j)));
//            points.push_back(hrp::Vector2(lfin_abs(X) + lf_mgn(i),    lfin_abs(Y) + lf_mgn(j)));
//          }
//        }
//        edge_time++;
//      }
      makeConvexHullOpenCV(points, hull_ans);
    }
    void calcWorldZMP(const hrp::Vector3& rfpos, const hrp::Vector3& lfpos, const Wrench6& rfwin, const Wrench6& lfwin, hrp::Vector3& zmp_ans);
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
