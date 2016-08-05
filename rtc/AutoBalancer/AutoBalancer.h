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

class humanpose_t{
  public:
    hrp::Vector3 com,rf,lf,rh,lh;
    hrp::Vector3 zmp;
    Eigen::VectorXd rfw,lfw;
    humanpose_t(){ clear(); }
    ~humanpose_t(){cout<<"humanpose_t destructed"<<endl;}
    void clear(){
      com =   hrp::Vector3::Zero();
      rf = hrp::Vector3::Zero();
      lf = hrp::Vector3::Zero();
      rh = hrp::Vector3::Zero();
      lh = hrp::Vector3::Zero();
      zmp =   hrp::Vector3::Zero();
      rfw.resize(6); rfw = Eigen::VectorXd::Zero(6);
      lfw.resize(6); lfw = Eigen::VectorXd::Zero(6);
    }
    static void hp_printf(const humanpose_t& in){
      printf("\x1b[31mcom:\x1b[39m %+05.2f %+05.2f %+05.2f ",in.com(0),in.com(1),in.com(2));
      printf("\x1b[31mrf:\x1b[39m %+05.2f %+05.2f %+05.2f ",in.rf(0),in.rf(1),in.rf(2));
      printf("\x1b[31mlf:\x1b[39m %+05.2f %+05.2f %+05.2f ",in.lf(0),in.lf(1),in.lf(2));
      printf("\x1b[31mrh:\x1b[39m %+05.2f %+05.2f %+05.2f ",in.rh(0),in.rh(1),in.rh(2));
      printf("\x1b[31mlh:\x1b[39m %+05.2f %+05.2f %+05.2f ",in.lh(0),in.lh(1),in.lh(2));
      printf("\x1b[31mzmp:\x1b[39m %+05.2f %+05.2f %+05.2f ",in.zmp(0),in.zmp(1),in.zmp(2));
      printf("\x1b[31mrfw:\x1b[39m %+04.0f %+04.0f %+05.0f %+04.0f %+04.0f %+04.0f ",in.rfw(0),in.rfw(1),in.rfw(2),in.rfw(3),in.rfw(4),in.rfw(5));
      printf("\x1b[31mlfw:\x1b[39m %+04.0f %+04.0f %+05.0f %+04.0f %+04.0f %+04.0f ",in.lfw(0),in.lfw(1),in.lfw(2),in.lfw(3),in.lfw(4),in.lfw(5));
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
    humanpose_t hp_wld_raw;
    humanpose_t hp_wld_initpos;
    humanpose_t hp_rel_raw;
    humanpose_t hp_filtered;
    humanpose_t rp_ref_out_old;
    int countdown_num;
    bool countdown_flag;
    bool HumanSyncOn;

  public:
    bool startCountdownForHumanSync;
    bool ht_first_call;
    bool is_rf_contact,is_lf_contact;


    humanpose_t rp_wld_initpos;
    double h2r_ratio;
    hrp::Vector3 init_wld_rp_basepos;

    humanpose_t rp_ref_out;

    HumanSynchronizer(){
//      h2r_ratio = 0.96;//human 1.1m vs jaxon 1.06m
//      h2r_ratio = 0.62;//human 1.1m vs chidori 0.69m
      h2r_ratio = 0.69;//human 1.0(with heavy foot sensor) vs chidori 0.69
      //h2r_ratio = 1.06;//human 1.0(with heavy foot sensor) vs jaxon 1.06
      cur_rfup_level = 0;       cur_lfup_level = 0;
      is_rf_contact = true;     is_lf_contact = true;
      pre_cont_rfpos.Zero();    pre_cont_lfpos.Zero();
      init_wld_hp_rfpos = hrp::Vector3(0,-0.1/h2r_ratio,0);
      init_wld_hp_lfpos = hrp::Vector3(0, 0.1/h2r_ratio,0);
      init_wld_rp_rfpos = hrp::Vector3(0,-0.1,0);
      init_wld_rp_lfpos = hrp::Vector3(0, 0.1,0);
      init_hp_calibcom.Zero();
      FNUM = 0.01;
      FUPLEVELNUM = 200;
      FUPHIGHT = 0.05;
      CNT_F_TH = 20.0;
//      MAXVEL = 0.004;
      MAXVEL = 0.001;
      HumanSyncOn = false;
      ht_first_call = true;
      countdown_flag = false;
      countdown_num = 5*500;
    }

    ~HumanSynchronizer(){cout<<"HumanSynchronizer destructed"<<endl;}


    void readInput(const humanpose_t& raw_in){ hp_wld_raw = raw_in; }

    int getRemainingCountDown(){ return countdown_num; }
    bool isHumanSyncOn(){ return HumanSyncOn; }

    void setInitOffsetPose(){ hp_wld_initpos = hp_wld_raw; }



//    void startCountDown(int sec_x_dt){
//      countdown_num = sec_x_dt;
//    }

    void updateCountDown(){
      if(countdown_num>0){
        countdown_num--;
      }else if(countdown_num==0){
        HumanSyncOn = true;
        startCountdownForHumanSync = false;
      }
    }

    void update(){
      removeInitOffsetPose(hp_wld_raw, hp_wld_initpos, hp_rel_raw);

      hp_rel_raw.com += init_hp_calibcom;

      applyLPFilter(hp_rel_raw, hp_filtered, hp_filtered);
      calcWorldZMP((hp_filtered.rf+init_wld_hp_rfpos), (hp_filtered.lf+init_wld_hp_lfpos), hp_filtered.rfw, hp_filtered.lfw, hp_filtered.zmp);//足の位置はworldにしないと・・・

      convertRelHumanPoseToWldRobotPose(hp_filtered,rp_ref_out);
      judgeFootContactStates(rp_ref_out.rfw, rp_ref_out.lfw, is_rf_contact, is_lf_contact);
      overwriteFootZFromContactStates(rp_ref_out, is_rf_contact, is_lf_contact, rp_ref_out);
      lockFootXYOnContact(rp_ref_out, is_rf_contact, is_lf_contact, rp_ref_out);
      applyWorkspaceLimit(rp_ref_out, rp_ref_out);
      applyCOMToSupportRegionLimit(rp_ref_out, rp_ref_out);
      applyVelLimit(rp_ref_out, rp_ref_out_old, rp_ref_out);
      rp_ref_out_old = rp_ref_out;
    }

    void calibInitHumanComPos(){
      init_hp_calibcom(0) = hp_filtered.zmp(0);
      init_hp_calibcom(1) = hp_filtered.zmp(1);
    }


    void calcWorldZMP(const hrp::Vector3& rfpos, const hrp::Vector3& lfpos,
        const Eigen::VectorXd& rfwin, const Eigen::VectorXd& lfwin, hrp::Vector3& zmp_ans){
      hrp::Vector3 rfzmp,lfzmp;
      const double F_H_OFFSET = 0.03;//地面から6軸センサ原点への高さ
      if( rfwin[2] > 1.0e-6 ){
        rfzmp(0) = ( - rfwin[4] - rfwin[0] * F_H_OFFSET + rfwin[2] * 0 ) / rfwin[2] + rfpos(0);
        rfzmp(1) = (   rfwin[3] - rfwin[1] * F_H_OFFSET + rfwin[2] * 0 ) / rfwin[2] + rfpos(1);
      }
      if( lfwin[2] > 1.0e-6 ){
        lfzmp(0) = ( - lfwin[4] - lfwin[0] * F_H_OFFSET + lfwin[2] * 0 ) / lfwin[2] + lfpos(0);
        lfzmp(1) = (   lfwin[3] - lfwin[1] * F_H_OFFSET + lfwin[2] * 0 ) / lfwin[2] + lfpos(1);
      }
      if( rfwin[2] > 1.0e-6 || lfwin[2] > 1.0e-6 ){
        zmp_ans(0) = ( rfzmp(0)*rfwin[2] + lfzmp(0)*lfwin[2] ) / ( rfwin[2] + lfwin[2]);
        zmp_ans(1) = ( rfzmp(1)*rfwin[2] + lfzmp(1)*lfwin[2] ) / ( rfwin[2] + lfwin[2]);
      }else{ zmp_ans(0) = 0; zmp_ans(1) = 0; }
      zmp_ans(2) = 0;
    }
    static hrp::Vector3 Point3DToVector3(const RTC::Point3D& in){ hrp::Vector3 out(in.x,in.y,in.z); return out; }


  private:

    void startHumanSync(){
      countdown_flag = true;
      setInitOffsetPose();
      HumanSyncOn = true;
    }

    void removeInitOffsetPose(const humanpose_t& abs_in, const humanpose_t& init_offset, humanpose_t& rel_out){
      rel_out.com = abs_in.com - init_offset.com;
      rel_out.rf  = abs_in.rf  - init_offset.rf;
      rel_out.lf  = abs_in.lf  - init_offset.lf;
      rel_out.rh  = abs_in.rh  - init_offset.rh;
      rel_out.lh  = abs_in.lh  - init_offset.lh;
      rel_out.rfw = abs_in.rfw;
      rel_out.lfw = abs_in.lfw;
    }
    void applyLPFilter(const humanpose_t& raw_in, const humanpose_t& out_old, humanpose_t& filt_out){
      filt_out.com = (1-FNUM) * out_old.com + FNUM * raw_in.com;
      filt_out.rf  = (1-FNUM) * out_old.rf  + FNUM * raw_in.rf;
      filt_out.lf  = (1-FNUM) * out_old.lf  + FNUM * raw_in.lf;
      filt_out.rh  = (1-FNUM) * out_old.rh  + FNUM * raw_in.rh;
      filt_out.lh  = (1-FNUM) * out_old.lh  + FNUM * raw_in.lh;
      filt_out.zmp = (1-FNUM) * out_old.zmp + FNUM * raw_in.zmp;
      filt_out.rfw = (1-FNUM) * out_old.rfw + FNUM * raw_in.rfw;
      filt_out.lfw = (1-FNUM) * out_old.lfw + FNUM * raw_in.lfw;
    }
    void convertRelHumanPoseToWldRobotPose(const humanpose_t& hp_in, humanpose_t& rp_out){//Wldと言いつつまだRel
      rp_out.com  = h2r_ratio * hp_in.com;//+ init_wld_rp_compos;
      rp_out.rf   = h2r_ratio * hp_in.rf;//+ init_wld_rp_rfpos;
      rp_out.lf   = h2r_ratio * hp_in.lf;//+ init_wld_rp_lfpos;
      rp_out.rh   = h2r_ratio * hp_in.rh;
      rp_out.lh   = h2r_ratio * hp_in.lh;
      rp_out.zmp  = h2r_ratio * hp_in.zmp;
      rp_out.rfw  = hp_in.rfw;
      rp_out.lfw  = hp_in.lfw;
    }
    void judgeFootContactStates(const Eigen::VectorXd& rfw_in, const Eigen::VectorXd& lfw_in, bool& rfcs_ans, bool& lfcs_ans){
      //他のプログラムは足に荷重時に力センサの値がZ軸マイナスに出るっぽい？(のでTODO FIX)
      if      (rfw_in(2)<CNT_F_TH && lfw_in(2)>CNT_F_TH){rfcs_ans = false;  lfcs_ans = true;}//右足浮遊かつ左足接地
      else if (rfw_in(2)>CNT_F_TH && lfw_in(2)<CNT_F_TH){rfcs_ans = true;   lfcs_ans = false;}//右足接地かつ左足浮遊
      else                                              {rfcs_ans = true;   lfcs_ans = true;}//両足接地または両足浮遊
    }

    void overwriteFootZFromContactStates(const humanpose_t& in, const bool& rfcs_in, const bool& lfcs_in, humanpose_t& out){
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
    void lockFootXYOnContact(const humanpose_t& in, const bool& rfcs_in, const bool& lfcs_in, humanpose_t& out){
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
//      out.rf(0) = 0;//足固定テスト
//      out.rf(1) = 0;
//      out.lf(0) = 0;
//      out.lf(1) = 0;
    }
    void applyWorkspaceLimit(const humanpose_t& in, humanpose_t& out){}

    void applyCOMToSupportRegionLimit(const humanpose_t& in, humanpose_t& out){
      out = in;
      double x_upper_region,x_lower_region,y_upper_region,y_lower_region;
      if(rp_wld_initpos.rf(0) + in.rf(0) > rp_wld_initpos.lf(0) + in.lf(0)){
        x_upper_region = rp_wld_initpos.rf(0) + in.rf(0) + 0.13;
        x_lower_region = rp_wld_initpos.lf(0) + in.lf(0) - 0.08;
      }else{
        x_upper_region = rp_wld_initpos.lf(0) + in.lf(0) + 0.13;
        x_lower_region = rp_wld_initpos.rf(0) + in.rf(0) - 0.08;
      }
      y_upper_region = rp_wld_initpos.lf(1) + in.lf(1) + 0.03;
      y_lower_region = rp_wld_initpos.rf(1) + in.rf(1) - 0.03;
      if(in.com(0) < x_lower_region){ out.com(0) = x_lower_region; }
      else if(in.com(0) > x_upper_region){ out.com(0) = x_upper_region; }
      if(in.com(1) < y_lower_region){ out.com(1) = y_lower_region; }
      else if(in.com(1) > y_upper_region){ out.com(1) = y_upper_region; }
    }

    void applyVelLimit(const humanpose_t& in, const humanpose_t& in_old, humanpose_t& out){
      for(int i=0;i<3;i++){if(in.com(i) - in_old.com(i) > MAXVEL){out.com(i) = in_old.com(i) + MAXVEL;}else if(in.com(i) - in_old.com(i) < -MAXVEL){out.com(i) = in_old.com(i) - MAXVEL;}}
//      for(int i=0;i<3;i++){if(in.rf(i)  - in_old.rf(i)  > MAXVEL){out.rf(i)  = in_old.rf(i)  + MAXVEL;}else if(in.rf(i)  - in_old.rf(i)  < -MAXVEL){out.rf(i)  = in_old.rf(i)  - MAXVEL;}}
//      for(int i=0;i<3;i++){if(in.lf(i)  - in_old.lf(i)  > MAXVEL){out.lf(i)  = in_old.lf(i)  + MAXVEL;}else if(in.lf(i)  - in_old.lf(i)  < -MAXVEL){out.lf(i)  = in_old.lf(i)  - MAXVEL;}}
      for(int i=0;i<3;i++){if(in.rf(i)  - in_old.rf(i)  > MAXVEL/2){out.rf(i)  = in_old.rf(i)  + MAXVEL/2;}else if(in.rf(i)  - in_old.rf(i)  < -MAXVEL/2){out.rf(i)  = in_old.rf(i)  - MAXVEL/2;}}
      for(int i=0;i<3;i++){if(in.lf(i)  - in_old.lf(i)  > MAXVEL/2){out.lf(i)  = in_old.lf(i)  + MAXVEL/2;}else if(in.lf(i)  - in_old.lf(i)  < -MAXVEL/2){out.lf(i)  = in_old.lf(i)  - MAXVEL/2;}}
      for(int i=0;i<3;i++){if(in.rh(i)  - in_old.rh(i)  > MAXVEL){out.rh(i)  = in_old.rh(i)  + MAXVEL;}else if(in.rh(i)  - in_old.rh(i)  < -MAXVEL){out.rh(i)  = in_old.rh(i)  - MAXVEL;}}
      for(int i=0;i<3;i++){if(in.lh(i)  - in_old.lh(i)  > MAXVEL){out.lh(i)  = in_old.lh(i)  + MAXVEL;}else if(in.lh(i)  - in_old.lh(i)  < -MAXVEL){out.lh(i)  = in_old.lh(i)  - MAXVEL;}}
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
  bool solveLimbIKforLimb (ABCIKparam& param);
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
  humanpose_t hp_raw_data;

};

extern "C"
{
  void AutoBalancerInit(RTC::Manager* manager);
};

#endif // IMPEDANCE_H
