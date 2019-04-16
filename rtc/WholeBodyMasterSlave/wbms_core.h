#ifndef WBMS_CORE_H
#define WBMS_CORE_H
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
#include <hrpCollision/DistFuncs.h>

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

//#include "../AutoBalancer/AutoBalancer.h"
#include "../AutoBalancer/FullbodyInverseKinematicsSolver.h"
#define USE_NEW_FIK

#define DEBUG 0
#define dbg(var) std::cout<<#var"= "<<(var)<<std::endl
#define dbgn(var) std::cout<<#var"= "<<std::endl<<(var)<<std::endl
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
        enum { XY = 2 };
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
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        BiquadIIRFilterVec(){}
        ~BiquadIIRFilterVec(){}
        void setParameter(const hrp::Vector3& fc_in, const double& HZ, const double& Q = 0.5){ for(int i=0;i<XYZ;i++){ filters[i].setParameterAsBiquad((double)fc_in(i), Q, HZ); } }
        void setParameter(const double& fc_in, const double& HZ, const double& Q = 0.5){ setParameter(hrp::Vector3(fc_in,fc_in,fc_in), HZ, Q); }//overload
        hrp::Vector3 passFilter(const hrp::Vector3& input){ for(int i=0;i<XYZ;i++){ ans(i) = filters[i].passFilter((double)input(i)); } return ans; }
        void reset(const hrp::Vector3& initial_input){ for(int i=0;i<XYZ;i++){ filters[i].reset((double)initial_input(i));} }
};

class WBMSPose3D{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        hrp::Vector3 p,rpy;
        WBMSPose3D(){ clear(); }
        ~WBMSPose3D(){}
        void clear(){ p = rpy = hrp::Vector3::Zero(); }
};

class PoseTGT{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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

namespace hrp{
    class Sphere{
        public:
            double r;
            hrp::Vector3 local_pos, cur_pos;

            Sphere(const hrp::Vector3 p_in = hrp::Vector3::Zero(), const double r_in = 0){
                local_pos = cur_pos = p_in;
                r = r_in;
            };
    };
}


inline double SegSegDist2(const Point& u0, const Point& u, const Point& v0, const Point& v, Point& cp0, Point& cp1){
    Point    w = u0 - v0;
    double    a = u|u;        // always >= 0
    double    b = u|v;
    double    c = v|v;        // always >= 0
    double    d = u|w;
    double    e = v|w;
    double    D = a*c - b*b;       // always >= 0
    double    sc, sN, sD = D;      // sc = sN / sD, default sD = D >= 0
    double    tc, tN, tD = D;      // tc = tN / tD, default tD = D >= 0

    // compute the line parameters of the two closest points
#define EPS 1e-8
    if (D < EPS) { // the lines are almost parallel
        sN = 0.0;        // force using point P0 on segment S1
        sD = 1.0;        // to prevent possible division by 0.0 later
        tN = e;
        tD = c;
    }
    else {                // get the closest points on the infinite lines
        sN = (b*e - c*d);
        tN = (a*e - b*d);
        if (sN < 0.0) {       // sc < 0 => the s=0 edge is visible
            sN = 0.0;
            tN = e;
            tD = c;
        }
        else if (sN > sD) {  // sc > 1 => the s=1 edge is visible
            sN = sD;
            tN = e + b;
            tD = c;
        }
    }

    if (tN < 0.0) {           // tc < 0 => the t=0 edge is visible
        tN = 0.0;
        // recompute sc for this edge
        if (-d < 0.0)
            sN = 0.0;
        else if (-d > a)
            sN = sD;
        else {
            sN = -d;
            sD = a;
        }
    }
    else if (tN > tD) {      // tc > 1 => the t=1 edge is visible
        tN = tD;
        // recompute sc for this edge
        if ((-d + b) < 0.0)
            sN = 0;
        else if ((-d + b) > a)
            sN = sD;
        else {
            sN = (-d + b);
            sD = a;
        }
    }
    // finally do the division to get sc and tc
    sc = (fabs(sN) < EPS ? 0.0 : sN / sD);
    tc = (fabs(tN) < EPS ? 0.0 : tN / tD);

    cp0 = u0 + sc * u;
    cp1 = v0 + tc * v;

    // get the difference of the two closest points
    Point dP = cp0 - cp1;

    return dP.Magnitude();   // return the closest distance
}

class Capsule{
    public:
        hrp::Vector3 p0, p1;
        double r;
        Capsule(const hrp::Vector3 _p0 = hrp::Vector3::Zero(), const hrp::Vector3 _p1 = hrp::Vector3::Zero(), const double _r = 0){
            p0 = _p0; p1 = _p1; r = _r;
        }
};

class CollisionInfo{
    public:
        int id0, id1;
        hrp::Vector3 cp0_local, cp1_local, cp0_wld, cp1_wld;
        double dist_safe, dist_cur;
        CollisionInfo(const int _id0 = 0, const int _id1 = 0,
                const hrp::Vector3 _cp0_local = hrp::Vector3::Zero(), const hrp::Vector3 _cp1_local = hrp::Vector3::Zero(),
                const hrp::Vector3 _cp0_wld = hrp::Vector3::Zero(), const hrp::Vector3 _cp1_wld = hrp::Vector3::Zero(),
                const double _dist_safe = 0, const double _dist_cur = 0){
            id0 = _id0; id1 = _id1; cp0_local = _cp0_local; cp1_local = _cp1_local;  cp0_wld = _cp0_wld; cp1_wld = _cp1_wld; dist_safe = _dist_safe; dist_cur = _dist_cur;
        }
};

typedef std::vector<Capsule> CapsuleArray;

class CapsuleCollisionChecker {
    private:
        const hrp::BodyPtr m_robot;
        std::vector<CapsuleArray> capsule_array_list_local, capsule_array_list_wld;
    public:
        std::vector<CollisionInfo> collision_info_list;
        Eigen::MatrixXi check_pair_mat;
        hrp::ivector avoid_priority;
        bool hasStr(const std::string& str, const std::string key){ return str.find(key) != std::string::npos; }
        CapsuleCollisionChecker(hrp::BodyPtr robot):
            m_robot(robot){
            capsule_array_list_local.resize(m_robot->numJoints());
            for(int i=0; i<m_robot->numJoints(); i++){
                if(hasStr(m_robot->joint(i)->name,"LEG_JOINT0")){
                    capsule_array_list_local[i].push_back(Capsule(hrp::Vector3(0,0,0), m_robot->joint(i)->child->b, 0.099));
                }
                if(hasStr(m_robot->joint(i)->name,"LLEG_JOINT2")){
                    capsule_array_list_local[i].push_back(Capsule(hrp::Vector3(0,0,0), m_robot->joint(i)->child->b, 0.095));
                    capsule_array_list_local[i].push_back(Capsule(hrp::Vector3(0,0.05,0), m_robot->joint(i)->child->b, 0.095));
                }
                if(hasStr(m_robot->joint(i)->name,"RLEG_JOINT2")){
                    capsule_array_list_local[i].push_back(Capsule(hrp::Vector3(0,0,0), m_robot->joint(i)->child->b, 0.095));
                    capsule_array_list_local[i].push_back(Capsule(hrp::Vector3(0,-0.05,0), m_robot->joint(i)->child->b, 0.095));
                }
                if(hasStr(m_robot->joint(i)->name,"LEG_JOINT3")){
                    capsule_array_list_local[i].push_back(Capsule(hrp::Vector3(0.05,0,0), m_robot->joint(i)->child->b, 0.095));
                }
                if(hasStr(m_robot->joint(i)->name,"ARM_JOINT2") || hasStr(m_robot->joint(i)->name,"ARM_JOINT3") || hasStr(m_robot->joint(i)->name,"ARM_JOINT4") || hasStr(m_robot->joint(i)->name,"ARM_JOINT5") || hasStr(m_robot->joint(i)->name,"ARM_JOINT6")){
                    capsule_array_list_local[i].push_back(Capsule(hrp::Vector3(0,0,0), m_robot->joint(i)->child->b, 0.09));
                }
                if(hasStr(m_robot->joint(i)->name,"ARM_JOINT7")){
                    capsule_array_list_local[i].push_back(Capsule(hrp::Vector3(0,0,0), hrp::Vector3(0,0,-0.35), 0.1));
                }
                if(hasStr(m_robot->joint(i)->name,"CHEST_JOINT1") || hasStr(m_robot->joint(i)->name,"CHEST_JOINT2")){
                    capsule_array_list_local[i].push_back(Capsule(hrp::Vector3(0,0,0), m_robot->joint(i)->child->b, 0.2));
                    capsule_array_list_local[i].push_back(Capsule(hrp::Vector3(-0.1,0,0), m_robot->joint(i)->child->b + hrp::Vector3(-0.1,0,0), 0.2));
                    capsule_array_list_local[i].push_back(Capsule(hrp::Vector3(-0.2,0,0), m_robot->joint(i)->child->b + hrp::Vector3(-0.2,0,0), 0.2));
                }
            };

            for(int i=0; i<m_robot->numJoints(); i++){
                for(int j=0; j<capsule_array_list_local[i].size(); j++){
                    std::cout<<m_robot->joint(i)->name<<" capsule["<<j<<"] "<<capsule_array_list_local[i][j].p0.transpose()<<" - "<<capsule_array_list_local[i][j].p1.transpose()<<" r: "<<capsule_array_list_local[i][j].r<<std::endl;
                }
            }
            capsule_array_list_wld = capsule_array_list_local;
            check_pair_mat = Eigen::MatrixXi::Ones(capsule_array_list_wld.size(),capsule_array_list_wld.size());
            for(int me=0;me<m_robot->numJoints();me++){
                for(int you=me; you<m_robot->numJoints();you++){
                    if(me == you) check_pair_mat(me, you) = 0;
                    if(m_robot->joint(me)->name.find("RLEG_JOINT") != std::string::npos && m_robot->joint(you)->name.find("RLEG_JOINT") != std::string::npos) check_pair_mat(me, you) = 0;
                    if(m_robot->joint(me)->name.find("LLEG_JOINT") != std::string::npos && m_robot->joint(you)->name.find("LLEG_JOINT") != std::string::npos) check_pair_mat(me, you) = 0;
                    if(m_robot->joint(me)->name.find("RARM_JOINT") != std::string::npos && m_robot->joint(you)->name.find("RARM_JOINT") != std::string::npos) check_pair_mat(me, you) = 0;
                    if(m_robot->joint(me)->name.find("LARM_JOINT") != std::string::npos && m_robot->joint(you)->name.find("LARM_JOINT") != std::string::npos) check_pair_mat(me, you) = 0;
//                    if(m_robot->link(me)->parent == m_robot->link(you) || m_robot->link(you)->parent == m_robot->link(me)) check_pair_mat(me, you) = 0;//実機とシミュで挙動違う？
                    if(m_robot->joint(me)->parent == m_robot->joint(you) || m_robot->joint(you)->parent == m_robot->joint(me)) check_pair_mat(me, you) = 0;

                    if(m_robot->joint(me)->name.find("LEG_JOINT0") != std::string::npos && m_robot->joint(you)->name.find("CHEST_JOINT") != std::string::npos) check_pair_mat(me, you) = 0;
                    if(m_robot->joint(me)->name.find("LEG_JOINT1") != std::string::npos && m_robot->joint(you)->name.find("CHEST_JOINT") != std::string::npos) check_pair_mat(me, you) = 0;
                    if(m_robot->joint(me)->name.find("LEG_JOINT2") != std::string::npos && m_robot->joint(you)->name.find("CHEST_JOINT") != std::string::npos) check_pair_mat(me, you) = 0;
                    if(m_robot->joint(me)->name.find("CHEST_JOINT") != std::string::npos && m_robot->joint(you)->name.find("ARM_JOINT0") != std::string::npos) check_pair_mat(me, you) = 0;
                    if(m_robot->joint(me)->name.find("CHEST_JOINT") != std::string::npos && m_robot->joint(you)->name.find("ARM_JOINT1") != std::string::npos) check_pair_mat(me, you) = 0;
                    if(m_robot->joint(me)->name.find("CHEST_JOINT") != std::string::npos && m_robot->joint(you)->name.find("ARM_JOINT2") != std::string::npos) check_pair_mat(me, you) = 0;
                }
            }
            dbgn(check_pair_mat);
            avoid_priority = hrp::ivector::Zero(m_robot->numJoints());

            for(int i=0;i<m_robot->numJoints();i++){
                if(m_robot->joint(i)->name.find("ARM_JOINT") != std::string::npos) avoid_priority(i) = 1;
                if(m_robot->joint(i)->name.find("CHEST_JOINT") != std::string::npos) avoid_priority(i) = 2;
                if(m_robot->joint(i)->name.find("LEG_JOINT") != std::string::npos) avoid_priority(i) = 3;
            }

        }
        ~CapsuleCollisionChecker(){cerr<<"CapsuleCollisionChecker destructed"<<endl;}
        void update(){
            for(int i=0;i<m_robot->numJoints();i++){
                for(int j=0;j<capsule_array_list_wld[i].size();j++){
                capsule_array_list_wld[i][j].p0 = m_robot->joint(i)->p + m_robot->joint(i)->R * capsule_array_list_local[i][j].p0;
                capsule_array_list_wld[i][j].p1 = m_robot->joint(i)->p + m_robot->joint(i)->R * capsule_array_list_local[i][j].p1;
                }
            };
        }
        Point Vec3ToPoint(const hrp::Vector3& in){ return Point(in(0),in(1),in(2)); }
        bool checkCollision(){
            update();
            collision_info_list.clear();
            for(int me=0;me<m_robot->numJoints();me++){
                for(int you=me; you<m_robot->numJoints();you++){

                    for(int mec=0;mec<capsule_array_list_wld[me].size();mec++){
                        for(int youc=0;youc<capsule_array_list_wld[you].size();youc++){
                            if(capsule_array_list_wld[me][mec].r > 0 && capsule_array_list_wld[you][youc].r > 0 && check_pair_mat(me,you)){
                                Point cp0_ans, cp1_ans;
                                double dist_ans = SegSegDist2(Vec3ToPoint(capsule_array_list_wld[me][mec].p0), Vec3ToPoint(capsule_array_list_wld[me][mec].p1-capsule_array_list_wld[me][mec].p0),
                                        Vec3ToPoint(capsule_array_list_wld[you][youc].p0), Vec3ToPoint(capsule_array_list_wld[you][youc].p1-capsule_array_list_wld[you][youc].p0), cp0_ans, cp1_ans);
                                double dist_safe = capsule_array_list_wld[me][mec].r + capsule_array_list_wld[you][youc].r;
                                if (dist_ans < dist_safe){
                                    hrp::Vector3 cp0_wld_tmp = hrp::Vector3(cp0_ans.x,cp0_ans.y,cp0_ans.z);
                                    hrp::Vector3 cp1_wld_tmp = hrp::Vector3(cp1_ans.x,cp1_ans.y,cp1_ans.z);
                                    collision_info_list.push_back(CollisionInfo(me, you,
                                            m_robot->joint(me)->R.transpose() * (cp0_wld_tmp - m_robot->joint(me)->p), m_robot->joint(you)->R.transpose() * (cp1_wld_tmp - m_robot->joint(you)->p),
                                            cp0_wld_tmp, cp1_wld_tmp, dist_safe, dist_ans));
                                }
                            }
                        }
                    }

                }
            };
            return (collision_info_list.size() > 0 );
        }
//        bool checkCapsuleArray2CapsuleArray(const Capsule){
//            if(capsule_array_list_wld[me].r > 0 && capsule_array_list_wld[you].r > 0 && check_pair_mat(me,you)){
//                Point cp0_ans, cp1_ans;
//                double dist_ans = SegSegDist2(Vec3ToPoint(capsule_array_list_wld[me].p0), Vec3ToPoint(capsule_array_list_wld[me].p1-capsule_array_list_wld[me].p0),
//                        Vec3ToPoint(capsule_array_list_wld[you].p0), Vec3ToPoint(capsule_array_list_wld[you].p1-capsule_array_list_wld[you].p0), cp0_ans, cp1_ans);
//                double dist_safe = capsule_array_list_wld[me].r + capsule_array_list_wld[you].r;
//                if (dist_ans < dist_safe){
//                    hrp::Vector3 cp0_wld_tmp = hrp::Vector3(cp0_ans.x,cp0_ans.y,cp0_ans.z);
//                    hrp::Vector3 cp1_wld_tmp = hrp::Vector3(cp1_ans.x,cp1_ans.y,cp1_ans.z);
//                    collision_info_list.push_back(CollisionInfo(me, you,
//                            m_robot->joint(me)->R.transpose() * (cp0_wld_tmp - m_robot->joint(me)->p), m_robot->joint(you)->R.transpose() * (cp1_wld_tmp - m_robot->joint(you)->p),
//                            cp0_wld_tmp, cp1_wld_tmp, dist_safe, dist_ans));
//                }
//            }
//        }
};

class WBMSCore : UTIL_CONST {
    private:
        double CNT_F_TH, h2r_ratio, tgt_h2r_ratio, HZ, DT;
        struct timeval t_calc_start, t_calc_end;
        BiquadIIRFilterVec calcacc_v_filters, acc4zmp_v_filters, com_in_filter;
        hrp::Vector3 com_old, com_oldold, comacc, com_CP_ref_old, r_zmp_raw;
        std::vector<hrp::Vector2> rflf_points, hull_com, hull_dcp, hull_acp;
        std::vector<BiquadIIRFilterVec> tgt_pos_filters,tgt_rot_filters;
        std::vector<cv::Point2f> points,cvhull;
        //    RobotConfig rc;
        HumanPose rp_ref_out_old, hp_swap_checked;
        unsigned int loop;
        bool is_initial_loop;
        bool cp_force_go_contact[LR];
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        double H_cur;
        HumanPose hp_wld_raw, hp_plot, rp_ref_out, rp_ref_vel_old;
        FILE *sr_log, *cz_log, *id_log;
        WBMSPose3D baselinkpose;
        //    WBMSPose3D ee_contact_pose[4];
        hrp::Vector3 com_vel_old,rh_vel_old, cp_dec, cp_acc;
        std::vector<hrp::Vector3> foot_vert_act[LR], foot_vert_safe[LR], foot_vert_check[LR];
        hrp::Vector2 com_forcp_ref,com_vel_forcp_ref;
        hrp::dvector6 invdyn_ft;
        hrp::Vector4 foot_vert_act_fblr[LR], foot_vert_safe_fblr[LR], foot_vert_check_fblr[LR];
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
                bool disable_lower;
        };
        struct WBMSparameters WBMSparam;
        struct ActualRobotState {
                hrp::Vector3 com, zmp;
        }act_rs;

        typedef boost::shared_ptr<FullbodyInverseKinematicsSolver> fikPtr;
        fikPtr fik_ml;
        hrp::BodyPtr m_robot_ml;
        //    fikPtr fik_act;
        hrp::BodyPtr m_robot_act;
        double cur_manip_val[4][3];
        hrp::Vector3 manip_direc[4][3];
        hrp::Matrix33 manip_mat[4];
        hrp::Vector3 manip_sv[4];
        hrp::Matrix33 manip_mat_rot[4];
        hrp::Vector3 manip_sv_rot[4];

        hrp::Vector2 cp_acc_old;
        //    WBMSPose3D ee_pose_old[4];

        WBMSCore(const double& dt){
            tgt_h2r_ratio = h2r_ratio = 0.96;//human 1.1m vs jaxon 1.06m
            //      tgt_h2r_ratio = h2r_ratio = 1.0;
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
            tgt_pos_filters[rf].setParameter(hrp::Vector3(1.0,1.0,1.0), HZ, Q_NOOVERSHOOT);//右足pos用
            tgt_pos_filters[lf].setParameter(hrp::Vector3(1.0,1.0,1.0), HZ, Q_NOOVERSHOOT);//左足pos用
            calcacc_v_filters.setParameter(5, HZ, Q_BUTTERWORTH);//加速度計算用
            acc4zmp_v_filters.setParameter(5, HZ, Q_BUTTERWORTH);//ZMP生成用ほぼこの値でいい
            com_in_filter.setParameter(1, HZ);
            foot_vert_act_fblr[R] << 0.13, -0.10,  0.06, -0.08;
            foot_vert_act_fblr[L] << 0.13, -0.10,  0.08, -0.06;
            foot_vert_check_fblr[R] << 0.02+0.002, -0.01-0.002,  0.02+0.002,  0.01-0.002;
            foot_vert_check_fblr[L] << 0.02+0.002, -0.01-0.002, -0.01+0.002, -0.02-0.002;
            //      foot_vert_check_fblr[R] << 0.13, -0.10,  0.06, -0.08;
            //      foot_vert_check_fblr[L] << 0.13, -0.10,  0.08, -0.06;
            foot_vert_safe_fblr[R] << 0.02, -0.01,  0.02,  0.01;
            foot_vert_safe_fblr[L] << 0.02, -0.01, -0.01, -0.02;
            for(int i=0;i<LR;i++){
                make_rect_verts(foot_vert_act_fblr[i], foot_vert_act[i]);
                make_rect_verts(foot_vert_check_fblr[i], foot_vert_check[i]);
                make_rect_verts(foot_vert_safe_fblr[i], foot_vert_safe[i]);
            }
            cp_force_go_contact[R] = false;
            cp_force_go_contact[L] = false;

            //      WBMSparam.auto_swing_foot_landing_threshold = 0.02;
            WBMSparam.auto_swing_foot_landing_threshold = 0.04;
            WBMSparam.foot_vertical_vel_limit_coeff = 4.0;
            WBMSparam.human_com_height = 1.00;
            WBMSparam.is_doctor = true;
            WBMSparam.set_com_height_fix = false;
            WBMSparam.set_com_height_fix_val = 0.02;
            WBMSparam.swing_foot_height_offset = 0.02;
            WBMSparam.swing_foot_max_height = 0.5;
            WBMSparam.upper_body_rmc_ratio = 0.0;
            WBMSparam.use_rh = WBMSparam.use_lh = true;
            WBMSparam.use_head = true;
            WBMSparam.use_manipulability_limit = true;
            WBMSparam.disable_lower = true;
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
        static void make_rect_verts(const hrp::Vector4& ForwardBackLeftRight, std::vector<hrp::Vector3>& out){
            out.clear();
            out.push_back(hrp::Vector3(ForwardBackLeftRight(0), ForwardBackLeftRight(2),  0));//左前
            out.push_back(hrp::Vector3(ForwardBackLeftRight(1), ForwardBackLeftRight(2),  0));//左後
            out.push_back(hrp::Vector3(ForwardBackLeftRight(1), ForwardBackLeftRight(3),  0));//右後
            out.push_back(hrp::Vector3(ForwardBackLeftRight(0), ForwardBackLeftRight(3),  0));//右前
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
                    rp_ref_out.tgt[human_l_names[i]].offs.p = fik_in->getEndEffectorPos(robot_l_names[i]);
                    rp_ref_out.tgt[human_l_names[i]].offs.rpy = hrp::rpyFromRot(fik_in->getEndEffectorRot(robot_l_names[i]));
                    rp_ref_out.tgt[human_l_names[i]].abs = rp_ref_out.tgt[human_l_names[i]].offs;
                }
            }
            rp_ref_out.tgt[com].offs.p = act_rs.com = act_rs.zmp = robot_in->calcCM();
            com_CP_ref_old = rp_ref_out.tgt[com].offs.p;
            rp_ref_out.tgt[com].offs.rpy = hrp::rpyFromRot(robot_in->rootLink()->R);
            rp_ref_out.tgt[zmp].offs.p(X) = rp_ref_out.tgt[com].offs.p(X);
            rp_ref_out.tgt[zmp].offs.p(Y) = rp_ref_out.tgt[com].offs.p(Y);
            rp_ref_out.tgt[zmp].offs.p(Z) = (fik_in->getEndEffectorPos("rleg")(Z)+fik_in->getEndEffectorPos("lleg")(Z))/2;
            rp_ref_out.tgt[rf].cnt = rp_ref_out.tgt[rf].offs;
            rp_ref_out.tgt[lf].cnt = rp_ref_out.tgt[lf].offs;
            baselinkpose.p = robot_in->rootLink()->p;
            baselinkpose.rpy = hrp::rpyFromRot(robot_in->rootLink()->R);
            for(int i=0, l[LR]={rf,lf}; i<LR; i++){
                rp_ref_out.tgt[l[i]].is_contact = true;
            }
            H_cur = rp_ref_out.tgt[com].offs.p(Z) - std::min((double)rp_ref_out.tgt[rf].offs.p(Z), (double)rp_ref_out.tgt[rf].offs.p(Z));
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
            updateParams                        ();
            autoLRSwapCheck                     (hp_wld_raw,hp_swap_checked);//入力の左右反転を常にチェック(＝手足の交差は不可能)
            convertRelHumanPoseToRelRobotPose   (hp_swap_checked, rp_ref_out);
            hp_plot = rp_ref_out;
            if(WBMSparam.set_com_height_fix)rp_ref_out.tgt[com].abs.p(Z) = rp_ref_out.tgt[com].offs.p(Z) + WBMSparam.set_com_height_fix_val;//膝曲げトルクで落ちるときの応急措置
            judgeFootLandOnCommandByFootForce   (hp_wld_raw);//人体足裏反力から各足の接地指令を生成
            lockSwingFootIfZMPOutOfSupportFoot  (rp_ref_out_old, rp_ref_out);//
            limitEEWorkspace                    (rp_ref_out);
            setFootContactPoseByGoContact       (rp_ref_out);
            if(DEBUG){ fprintf(cz_log,"com_in: %f %f ",rp_ref_out.tgt[com].abs.p(X),rp_ref_out.tgt[com].abs.p(Y)); }
            applyCOMToSupportRegionLimit        (rp_ref_out.tgt[rf].abs.p, rp_ref_out.tgt[lf].abs.p, rp_ref_out.tgt[com].abs.p);

            overwriteFootZFromFootLandOnCommand (rp_ref_out);
            setFootRotHorizontalIfGoLanding     (rp_ref_out);

            for(int i=0, l[2]={rf,lf}; i<2; i++){
                double fheight = rp_ref_out_old.tgt[l[i]].abs.p(Z) - rp_ref_out_old.tgt[l[i]].offs.p(Z);
                double horizontal_max_vel = 0 + fheight * 20;
                for(int j=0;j<XY;j++)LIMIT_MINMAX( rp_ref_out.tgt[l[i]].abs.p(j), rp_ref_out_old.tgt[l[i]].abs.p(j)-horizontal_max_vel*DT, rp_ref_out_old.tgt[l[i]].abs.p(j)+horizontal_max_vel*DT);
                double vertical_max_vel = 0 + fheight * 10;
                LIMIT_MIN( rp_ref_out.tgt[l[i]].abs.p(Z), rp_ref_out_old.tgt[l[i]].abs.p(Z)-vertical_max_vel*DT);
            }

            //      limitManipulability                 (rp_ref_out);

            applyCOMToSupportRegionLimit        (rp_ref_out.tgt[rf].abs.p, rp_ref_out.tgt[lf].abs.p, com_CP_ref_old);//これやらないと支持領域の移動によって1ステップ前のCOM位置はもうはみ出てるかもしれないから

            Vector3ToVector2(rp_ref_out.tgt[com].abs.p,com_forcp_ref);
            static hrp::Vector2 com_forcp_ref_old;
            com_vel_forcp_ref = (com_forcp_ref - com_forcp_ref_old)/DT;
            com_forcp_ref_old = com_forcp_ref;

            applyLPFilter_pre                       (rp_ref_out);
            applyCOMStateLimitByCapturePoint    (rp_ref_out.tgt[com].abs.p, rp_ref_out.tgt[rf].abs.p, rp_ref_out.tgt[lf].abs.p, com_CP_ref_old, rp_ref_out.tgt[com].abs.p);
            //      surpressGoContactChattering         (rp_ref_out);
            //      setFootContactPoseByGoContact       (rp_ref_out);
            applyCOMToSupportRegionLimit        (rp_ref_out.tgt[rf].abs.p, rp_ref_out.tgt[lf].abs.p, rp_ref_out.tgt[com].abs.p);
            applyLPFilter_post                       (rp_ref_out);

            r_zmp_raw = rp_ref_out.tgt[zmp].abs.p;
            applyZMPCalcFromCOM                 (rp_ref_out.tgt[com].abs.p, rp_ref_out.tgt[zmp].abs.p);
            if(DEBUG){
                fprintf(cz_log,"com_ans_zmp: %f %f ",rp_ref_out.tgt[zmp].abs.p(X),rp_ref_out.tgt[zmp].abs.p(Y));
                fprintf(cz_log,"\n");
            }

            //      overwriteFootZFromFootLandOnCommand (rp_ref_out);
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
            for(int i=0, l[6]={com,rf,lf,rh,lh,head}; i<6; i++){
                out.tgt[l[i]].abs.p   =  h2r_ratio * (in.tgt[l[i]].abs.p - in.tgt[l[i]].offs.p)   + out.tgt[l[i]].offs.p;
                out.tgt[l[i]].abs.rpy =  hrp::rpyFromRot( hrp::rotFromRpy(in.tgt[l[i]].abs.rpy) * hrp::rotFromRpy(in.tgt[l[i]].offs.rpy).transpose() * hrp::rotFromRpy(out.tgt[l[i]].offs.rpy) );
            }
            for(int i=0, l[4]={rf,lf,rh,lh}; i<4; i++){
                out.tgt[l[i]].w = in.tgt[l[i]].w;
                out.tgt[l[i]].go_contact = in.tgt[l[i]].go_contact;
            }
            out.tgt[zmp].abs = in.tgt[zmp].abs;//最近使わない
        }
        void judgeFootLandOnCommandByFootForce(HumanPose& in){
            for(int i=0, l[LR]={rf,lf}; i<LR; i++){
                if      (in.tgt[l[i]].is_contact  && in.tgt[l[i]].w(fz)<CNT_F_TH   ){in.tgt[l[i]].is_contact = false;}//足ついた状態から上げる
                else if (!in.tgt[l[i]].is_contact && in.tgt[l[i]].w(fz)>CNT_F_TH+30){in.tgt[l[i]].is_contact = true;}//足浮いた状態から下げる
            }
        }
        void applyLPFilter_pre(HumanPose& tgt){
            const std::string robot_l_names[4] = {"rleg","lleg","rarm","larm"};
            for(int i=0, l[4]={rf,lf,rh,lh}; i<4; i++){
                //        calcVelAccSafeTrajectoryVec(fik_act->getEndEffectorPos(robot_l_names[i]), (fik_act->getEndEffectorPos(robot_l_names[i]) - ee_pose_old[i].p)/DT, tgt.tgt[l[i]].abs.p, 3.0, 1.0, tgt.tgt[l[i]].abs.p);//何故かダメ
                //        calcVelAccSafeTrajectoryVec(hrp::rpyFromRot(fik_act->getEndEffectorRot(robot_l_names[i])), (hrp::rpyFromRot(fik_act->getEndEffectorRot(robot_l_names[i])) - ee_pose_old[i].rpy)/DT, tgt.tgt[l[i]].abs.rpy, 3.0, 1.0, tgt.tgt[l[i]].abs.rpy);
                //        calcVelAccSafeTrajectoryVec(fik_act->getEndEffectorPos(robot_l_names[i]), (rp_ref_out.tgt[l[i]].abs.p - rp_ref_out_old.tgt[l[i]].abs.p)/DT, tgt.tgt[l[i]].abs.p, 3.0, 1.0, tgt.tgt[l[i]].abs.p);
                //        calcVelAccSafeTrajectoryVec(hrp::rpyFromRot(fik_act->getEndEffectorRot(robot_l_names[i])), (rp_ref_out.tgt[l[i]].abs.rpy - rp_ref_out_old.tgt[l[i]].abs.rpy)/DT, tgt.tgt[l[i]].abs.rpy, 3.0, 1.0, tgt.tgt[l[i]].abs.rpy);
            }
            for(int i=0, l[1]={head}; i<1; i++){
                //        calcVelAccSafeTrajectoryVec(rp_ref_out_old.tgt[l[i]].abs.p, (rp_ref_out.tgt[l[i]].abs.p - rp_ref_out_old.tgt[l[i]].abs.p)/DT, tgt.tgt[l[i]].abs.p, 2.0, 1.0, tgt.tgt[l[i]].abs.p);
                //        calcVelAccSafeTrajectoryVec(rp_ref_out_old.tgt[l[i]].abs.rpy, (rp_ref_out.tgt[l[i]].abs.rpy - rp_ref_out_old.tgt[l[i]].abs.rpy)/DT, tgt.tgt[l[i]].abs.rpy, 2.0, 1.0, tgt.tgt[l[i]].abs.rpy);
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
            if((pos_tgt - pos_cur).norm() > 1e-6){
                for(int i=0;i<3;i++){
                    double stop_safe_vel = SGN(pos_tgt(i) - pos_cur(i)) * sqrt( 2 * max_acc * fabs(pos_tgt(i) - pos_cur(i)) );
                    double vel_ans;
                    if(vel_cur(i) > stop_safe_vel){
                        vel_ans = vel_cur(i) - max_acc * DT;
                    }else{
                        vel_ans = vel_cur(i) + max_acc * DT;
                    }
                    pos_ans(i) = pos_cur(i) + vel_ans * DT;
                }
            }else{
                pos_ans = pos_tgt;
            }
        }
        void calcVelAccSafeTrajectoryVecML(const hrp::Vector3& pos_cur, const hrp::Vector3& vel_cur, const hrp::Vector3& pos_tgt, const double& acc_base, const hrp::Matrix33& max_acc_mat, const hrp::Vector3& max_acc_sv, hrp::Vector3& pos_ans){
            if((pos_tgt - pos_cur).norm() > 1e-6){
                hrp::Vector3 direc = (pos_tgt - pos_cur).normalized();
                hrp::Vector3 ref_acc = direc*acc_base;
                hrp::Vector3 mod_acc = max_acc_mat * max_acc_sv.asDiagonal() * max_acc_mat.transpose() * ref_acc;
                double stop_safe_vel_scalar = sqrt( 2 * fabs(mod_acc.dot(direc)) * (pos_tgt - pos_cur).norm() );
                hrp::Vector3 vel_ans = vel_cur + mod_acc * DT;
                if(vel_ans.dot(direc) > stop_safe_vel_scalar){
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
            hrp::Vector2 inside_vec_rf(0,1),inside_vec_lf(0,-1);//TODO implement
            //      hrp::Vector2 rf2zmp = hrp::Vector2(old.tgt[zmp].abs.p(X),old.tgt[zmp].abs.p(Y)) - hrp::Vector2(old.tgt[rf].abs.p(X),old.tgt[rf].abs.p(Y));
            //      hrp::Vector2 lf2zmp = hrp::Vector2(old.tgt[zmp].abs.p(X),old.tgt[zmp].abs.p(Y)) - hrp::Vector2(old.tgt[lf].abs.p(X),old.tgt[lf].abs.p(Y));
            hrp::Vector2 rf2zmp = act_rs.zmp.head(XY) - old.tgt[rf].abs.p.head(XY);
            hrp::Vector2 lf2zmp = act_rs.zmp.head(XY) - old.tgt[lf].abs.p.head(XY);
            if( inside_vec_rf.dot(rf2zmp) / inside_vec_rf.norm() > WBMSparam.auto_swing_foot_landing_threshold ){ out.tgt[lf].go_contact = true; }
            if( inside_vec_lf.dot(lf2zmp) / inside_vec_lf.norm() > WBMSparam.auto_swing_foot_landing_threshold ){ out.tgt[rf].go_contact = true; }
        }
        void limitEEWorkspace(HumanPose& out){
            const double MAX_FW = 0.25;
            //      const double MAX_FW = 1000000;//manipulability test
            const double FOOT_2_FOOT_COLLISION_MARGIIN = 0.16;

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
//            LIMIT_MINMAX( out.tgt[rf].abs.p(Z), out.tgt[rf].offs.p(Z), out.tgt[rf].offs.p(Z)+WBMSparam.swing_foot_max_height);
//            LIMIT_MINMAX( out.tgt[lf].abs.p(Z), out.tgt[lf].offs.p(Z), out.tgt[lf].offs.p(Z)+WBMSparam.swing_foot_max_height);

//            const double base2hand_min = 0.4;
            const double base2hand_min = 0.0;
            for(int i=0, l[2]={rh,lh}; i<2; i++){
                //        LIMIT_MIN(out.tgt[l[i]].abs.p(X), baselinkpose.p(X));
                //        LIMIT_MAX(out.tgt[l[i]].abs.p(Z), baselinkpose.p(Z) + 0.4);
                hrp::Vector2 horizontal_dist(out.tgt[l[i]].abs.p(X) - baselinkpose.p(X), out.tgt[l[i]].abs.p(Y) - baselinkpose.p(Y));
                if(horizontal_dist.norm() < base2hand_min){
                    horizontal_dist = base2hand_min * horizontal_dist.normalized();
                }
                out.tgt[l[i]].abs.p(X) = baselinkpose.p(X) + horizontal_dist(X);
                out.tgt[l[i]].abs.p(Y) = baselinkpose.p(Y) + horizontal_dist(Y);
            }
            LIMIT_MINMAX( out.tgt[com].abs.p(Z), out.tgt[com].offs.p(Z) - 0.15, out.tgt[com].offs.p(Z) + 0.03 );//COM高さ方向の制限
            //      for(int i=0;i<XYZ;i++){ LIMIT_MINMAX( out.tgt[com].abs.rpy(i), rc.ee_rot_limit[com][MIN](i), rc.ee_rot_limit[com][MAX](i) ); }
            //      for(int i=0, l[5]={rf,lf,rh,lh,head}; i<5; i++){
            //        for(int j=0;j<XYZ;j++){ LIMIT_MINMAX( out.tgt[l[i]].abs.rpy(j), rc.ee_rot_limit[l[i]][MIN](j) + out.tgt[com].abs.rpy(j), rc.ee_rot_limit[l[i]][MAX](j) + out.tgt[com].abs.rpy(j) ); }
            //      }
            if(!WBMSparam.use_head)out.tgt[head].abs.rpy = hrp::Vector3::Zero();
        }
        void limitManipulability(HumanPose& out){
            const std::string robot_l_names[4] = {"rleg","lleg","rarm","larm"};
            const int human_l_names[4] = {rf,lf,rh,lh};
            m_robot_ml->rootLink()->p = m_robot_act->rootLink()->p;
            m_robot_ml->rootLink()->R = m_robot_act->rootLink()->R;
            //      for (int i=0;i<m_robot_ml->numJoints();i++){ m_robot_ml->joint(i)->q = m_robot_act->joint(i)->q; }
            m_robot_ml->calcForwardKinematics();
            for(int i=0;i<4;i++){
                if(fik_ml->ikp.count(robot_l_names[i])){
                    fik_ml->ikp[robot_l_names[i]].target_r0 = hrp::rotFromRpy(out.tgt[human_l_names[i]].abs.rpy);
                    fik_ml->ikp[robot_l_names[i]].target_p0 = out.tgt[human_l_names[i]].abs.p;
                }
            }
            const int pre_ik_loop = 2;
            for(int i=0;i<pre_ik_loop;i++){
                for ( std::map<std::string, FullbodyInverseKinematicsSolver::IKparam>::iterator it = fik_ml->ikp.begin(); it != fik_ml->ikp.end(); it++ ) {
                    if (it->second.is_ik_enable) fik_ml->solveLimbIK (it->second, it->first, fik_ml->ratio_for_vel, false);
                }
            }
            for(int i=0;i<4;i++){
                if(fik_ml->ikp.count(robot_l_names[i])){
                    out.tgt[human_l_names[i]].abs.p = fik_ml->getEndEffectorPos(robot_l_names[i]);
                    out.tgt[human_l_names[i]].abs.rpy = hrp::rpyFromRot(fik_ml->getEndEffectorRot(robot_l_names[i]));
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
                    out.tgt[l[i]].cnt.p(X) = rp_ref_out_old.tgt[l[i]].abs.p(X);
                    out.tgt[l[i]].cnt.p(Y) = rp_ref_out_old.tgt[l[i]].abs.p(Y);
                    out.tgt[l[i]].cnt.rpy(y) = rp_ref_out_old.tgt[l[i]].abs.rpy(y);
                }
            }
        }
        void setFootRotHorizontalIfGoLanding(HumanPose& out){
            if( out.tgt[rf].go_contact){ out.tgt[rf].abs.rpy(r) = out.tgt[rf].abs.rpy(p) = 0; }
            if( out.tgt[lf].go_contact){ out.tgt[lf].abs.rpy(r) = out.tgt[lf].abs.rpy(p) = 0; }
        }
        void overwriteFootZFromFootLandOnCommand(HumanPose& out){
            //      limitGroundContactVelocity(out.tgt[rf].go_contact, rp_ref_out_old.tgt[rf], out.tgt[rf], out.tgt[rf].is_contact);
            //      limitGroundContactVelocity(out.tgt[lf].go_contact, rp_ref_out_old.tgt[lf], out.tgt[lf], out.tgt[lf].is_contact);
            for(int i=0, l[LR]={rf,lf}; i<LR; i++){
                if(out.tgt[l[i]].go_contact || cp_force_go_contact[i]){
                    out.tgt[l[i]].abs.p(Z) = out.tgt[l[i]].offs.p(Z);
                }else{
                    LIMIT_MIN( out.tgt[l[i]].abs.p(Z), out.tgt[l[i]].offs.p(Z)+WBMSparam.swing_foot_height_offset);
                }
                const double contact_threshold = 0.005;
                out.tgt[l[i]].is_contact = ((out.tgt[l[i]].abs.p(Z)-out.tgt[l[i]].offs.p(Z)) <= contact_threshold);
            }
        }
        //    void limitGroundContactVelocity(const bool& go_land_in, const PoseTGT& f_old_in, PoseTGT& f_in_out, bool& is_f_contact_out){
        //      LIMIT_MIN( f_in_out.abs.p(Z), f_in_out.offs.p(Z));
        //      double input_vel = (f_in_out.abs.p(Z) - f_old_in.abs.p(Z)) / DT;
        //      const double vel_limit_k = WBMSparam.foot_vertical_vel_limit_coeff;//地面から0.02[m]地点で0.02*8=0.16[m/s]出ている計算
        //      double limit_vel = - vel_limit_k * (f_old_in.abs.p(Z) - f_old_in.offs.p(Z));//sinの時は平均0.05~0.10[m/s]
        //      const double max_vel_threshold = -0.01;
        //      LIMIT_MAX( limit_vel, max_vel_threshold);//速度0に近づくと目標になかなか到達しないから
        //      LIMIT_MIN( input_vel, limit_vel);//着地時の下向きの速度を制限
        //      f_in_out.abs.p(Z) = f_old_in.abs.p(Z) + input_vel * DT;
        //      LIMIT_MIN( f_in_out.abs.p(Z), f_in_out.offs.p(Z));
        //      const double contact_threshold = 0.005;
        //      is_f_contact_out = ((f_in_out.abs.p(Z)-f_in_out.offs.p(Z)) <= contact_threshold);
        //    }
        void modifyFootRotAndXYForContact(HumanPose& out){
            calcFootRotAndXYTransitionForContact(out.tgt[rf],foot_vert_act[R]);
            calcFootRotAndXYTransitionForContact(out.tgt[lf],foot_vert_act[L]);
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
            createSupportRegionByFootPos(rfin_abs, lfin_abs, foot_vert_safe_fblr[R], foot_vert_safe_fblr[L], hull_com);
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
            hrp::Vector4 marginDelta_for_acp(+0.010,-0.010,0.010,-0.010);
            createSupportRegionByFootPos(rfin_abs, lfin_abs, foot_vert_safe_fblr[R]+marginDelta_for_dcp, foot_vert_safe_fblr[L]+marginDelta_for_dcp, hull_dcp);
            createSupportRegionByFootPos(rfin_abs, lfin_abs, foot_vert_safe_fblr[R]+marginDelta_for_acp, foot_vert_safe_fblr[L]+marginDelta_for_acp, hull_acp);
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
            //      //減速CP条件(現在のCPを常に両足裏で頭打ち)
            //      hrp::Vector2 cp_dec_tmp = com_pos + com_vel * sqrt( H_cur / G );
            //      hrp::Vector2 cp_dec_ragulated;
            //      if(!isPointInHullOpenCV(cp_dec_tmp,hull_d)){
            //        calcCrossPointOnHull(com_pos, cp_dec_tmp, hull_d, cp_dec_ragulated);
            //        com_vel_decel_ok = (cp_dec_ragulated - com_pos) / sqrt( H_cur / G );
            //      }

            const double foot_ave_vel = 2.0;
            double lf_landing_delay = (rp_ref_out.tgt[lf].abs.p[Z] - rp_ref_out.tgt[lf].offs.p[Z]) / foot_ave_vel;
            double rf_landing_delay = (rp_ref_out.tgt[rf].abs.p[Z] - rp_ref_out.tgt[rf].offs.p[Z]) / foot_ave_vel;
            if(com_vel(Y)>0 ){ rf_landing_delay = 0; }
            if(com_vel(Y)<0 ){ lf_landing_delay = 0; }
            double foot_landing_delay = std::max(rf_landing_delay, lf_landing_delay);
            //      const double foot_landing_delay = 0;//debug
            //減速CP条件(現在のCPを常に両足裏で頭打ち)

            //      hrp::Vector2 inside_vec_rf(0,1),inside_vec_lf(0,-1);//TODO implement
            //      if(com_vel(Y)>1e-5 || fabs(com_vel(Y))<=1e-5 ){
            //        foot_landing_delay = 0;
            //      }
            hrp::Vector2 cp_dec_ragulated = com_pos + com_vel * sqrt( H_cur / G ) + com_vel * foot_landing_delay;
            if(!isPointInHullOpenCV(cp_dec_ragulated,hull_d)){
                calcCrossPointOnHull(com_pos, cp_dec_ragulated, hull_d, cp_dec_ragulated);
                com_vel_decel_ok = (cp_dec_ragulated - com_pos) / ( sqrt( H_cur / G ) + foot_landing_delay);
            }

#define F(x) (x == R ? rf : lf)

            std::vector<hrp::Vector2> f_hull_wld[LR], f_hull_wld_ans[LR];
            for(int i=0;i<foot_vert_check[R].size();i++){ f_hull_wld[R].push_back(foot_vert_check[R][i].head(XY) + rp_ref_out.tgt[F(R)].abs.p.head(XY)); }
            for(int i=0;i<foot_vert_check[L].size();i++){ f_hull_wld[L].push_back(foot_vert_check[L][i].head(XY) + rp_ref_out.tgt[F(L)].abs.p.head(XY)); }
            //      for(int i=0;i<foot_vert_safe[R].size();i++){ f_hull_wld[R].push_back(foot_vert_safe[R][i].head(XY) + rp_ref_out.tgt[F(R)].abs.p.head(XY)); }
            //      for(int i=0;i<foot_vert_safe[L].size();i++){ f_hull_wld[L].push_back(foot_vert_safe[L][i].head(XY) + rp_ref_out.tgt[F(L)].abs.p.head(XY)); }
            makeConvexHullOpenCV(f_hull_wld[R], f_hull_wld_ans[R]);
            makeConvexHullOpenCV(f_hull_wld[L], f_hull_wld_ans[L]);
            //      if(!isPointInHullOpenCV(cp_dec_ragulated, f_hull_wld_ans[R])){ rp_ref_out.tgt[lf].go_contact = true; }
            //      if(!isPointInHullOpenCV(cp_dec_ragulated, f_hull_wld_ans[L])){ rp_ref_out.tgt[rf].go_contact = true; }
            if(!isPointInHullOpenCV(cp_dec_ragulated, f_hull_wld_ans[R])){ cp_force_go_contact[L] = true; }else{ cp_force_go_contact[L] = false; }
            if(!isPointInHullOpenCV(cp_dec_ragulated, f_hull_wld_ans[L])){ cp_force_go_contact[R] = true; }else{ cp_force_go_contact[R] = false; }

            //加速CP条件(ACP使用)
            hrp::Vector2 cp_acc_tmp = com_pos - com_vel * sqrt( H_cur / G );
            hrp::Vector2 cp_acc_ragulated;
            if(!isPointInHullOpenCV(cp_acc_tmp,hull_a)){
                calcCrossPointOnHull(com_pos, cp_acc_tmp, hull_a, cp_acc_ragulated);
                com_vel_accel_ok = (-cp_acc_ragulated + com_pos ) / sqrt( H_cur / G );
            }
            cp_acc_old = cp_acc_ragulated;

            static hrp::Vector2 com_vel_ans_old = com_vel;
            //加速減速条件マージ
            if( com_vel_decel_ok.dot(com_vel) < com_vel_accel_ok.dot(com_vel)){//normじゃダメ？
                com_vel_ans = com_vel_decel_ok;
            }else{
                com_vel_ans = com_vel_accel_ok;
            }
            com_vel_ans_old = com_vel_ans;
        }
        //    void regulateCOMVelocityByCapturePointVec(const hrp::Vector2& com_pos, const hrp::Vector2& com_vel, const std::vector<hrp::Vector2>& hull_d, const std::vector<hrp::Vector2>& hull_a, hrp::Vector2& com_vel_ans){
        //      hrp::Vector2 com_vel_decel_ok,com_vel_accel_ok;
        //      com_vel_decel_ok = com_vel_accel_ok = com_vel_ans = com_vel;
        //      //減速CP条件(現在のCPを常に両足裏で頭打ち)
        //      hrp::Vector2 cp_dec_tmp = com_pos + com_vel * sqrt( H_cur / G );
        //      hrp::Vector2 cp_dec_ragulated;
        //      if(!isPointInHullOpenCV(cp_dec_tmp,hull_d)){
        //        calcCrossPointOnHull(com_pos, cp_dec_tmp, hull_d, cp_dec_ragulated);
        //        com_vel_decel_ok = (cp_dec_ragulated - com_pos) / sqrt( H_cur / G );
        //      }
        //
        //      //減速CP条件2(指令のCPに対する現在発揮可能なCP)
        ////      hrp::Vector2 cp_dec_ref = com_forcp_ref + com_vel_forcp_ref * sqrt( H_cur / G );
        ////      hrp::Vector2 cp_dec_ref_ragulated = cp_dec_ref;
        ////      if(!isPointInHullOpenCV(cp_dec_ref,hull_d)){
        ////        calcCrossPointOnHull(com_pos, cp_dec_ref, hull_d, cp_dec_ref_ragulated);
        ////      }
        ////      com_vel_decel_ok = (cp_dec_ref_ragulated - com_pos) / sqrt( H_cur / G );
        //
        //      //減速CP条件2(指令のCOMに収束する現在発揮可能なCP)
        ////      hrp::Vector2 cp_dec_ref = com_forcp_ref;
        ////      hrp::Vector2 cp_dec_ref_ragulated = cp_dec_ref;
        ////      if(!isPointInHullOpenCV(cp_dec_ref,hull_d)){
        ////        calcCrossPointOnHull(com_pos, cp_dec_ref, hull_d, cp_dec_ref_ragulated);
        ////      }
        ////      hrp::Vector2 vel_max_decel_ok = (cp_dec_ref_ragulated - com_pos) / sqrt( H_cur / G );
        ////      if( com_vel.dot(com_vel) > vel_max_decel_ok.dot(com_vel)){
        ////        com_vel_decel_ok = vel_max_decel_ok;
        ////      }else{
        ////        com_vel_decel_ok = com_vel;
        ////      }
        //
        //      //加速CP条件(ACP使用)
        //      hrp::Vector2 cp_acc_tmp = com_pos - com_vel * sqrt( H_cur / G );
        //      hrp::Vector2 cp_acc_ragulated;
        //      if(!isPointInHullOpenCV(cp_acc_tmp,hull_a)){
        //         calcCrossPointOnHull(com_pos, cp_acc_tmp, hull_a, cp_acc_ragulated);
        //         com_vel_accel_ok = (-cp_acc_ragulated + com_pos ) / sqrt( H_cur / G );
        //      }
        //      cp_acc_old = cp_acc_ragulated;
        //
        //      //加速CP条件2(ZMP使用)
        //      static hrp::Vector2 com_vel_ans_old = com_vel;
        ////      hrp::Vector2 com_acc = (com_vel - com_vel_ans_old) / DT;
        ////      hrp::Vector2 zmp_in = com_pos - com_acc / G * H_cur;
        ////      hrp::Vector2 zmp_regulated;
        ////      if(!isPointInHullOpenCV(zmp_in,hull_a)){
        ////        calcCrossPointOnHull(com_pos, zmp_in, hull_a, zmp_regulated);
        ////        hrp::Vector2 com_acc_regulated = (com_pos - zmp_regulated)*G/H_cur;
        ////        com_vel_accel_ok = com_vel_ans_old + com_acc_regulated*DT;
        ////      }
        //
        //      //加速減速条件マージ
        //      if( com_vel_decel_ok.dot(com_vel) < com_vel_accel_ok.dot(com_vel)){//normじゃダメ？
        //        com_vel_ans = com_vel_decel_ok;
        //      }else{
        //        com_vel_ans = com_vel_accel_ok;
        //      }
        //      com_vel_ans_old = com_vel_ans;
        //    }
        void surpressGoContactChattering(HumanPose& out){
            //      static std::deque<int> buffer[LR];
            static int count[LR];
            for(int i=0;i<LR;i++){
                if(out.tgt[F(i)].go_contact){ count[i]++; }
                else{ count[i]--; }
                LIMIT_MINMAX(count[i],0,100);
                if(count[i]>0){ out.tgt[F(i)].go_contact = true; }
            }
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

#endif // WBMS_CORE_H
