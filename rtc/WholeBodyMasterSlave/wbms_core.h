#ifndef WBMS_CORE_H
#define WBMS_CORE_H

#include "interpolator.h"
#include <hrpUtil/Eigen4d.h>
#include <hrpCollision/DistFuncs.h>
#include "../ImpedanceController/JointPathEx.h"
#include "../AutoBalancer/FullbodyInverseKinematicsSolver.h"
// geometry
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/core/eigen.hpp>

// geometry
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/multi/geometries/multi_point.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/polygon.hpp>
//#include <boost/assign/list_of.hpp>
namespace bg = boost::geometry;
typedef bg::model::d2::point_xy<double> bg_point;
typedef bg::model::multi_point<bg_point> bg_multi_point;
typedef bg::model::linestring<bg_point> bg_linestring;
typedef bg::model::polygon<bg_point> bg_polygon;

//#include<Eigen/StdVector>
//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix2d)
//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3d)
//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector3d)


#define F(x) (x == R ? rf : lf)


#define DEBUG 0

inline bg_point to_bg_point(const hrp::Vector2& hrp_point2d){ return bg_point(hrp_point2d(X), hrp_point2d(Y)); }
inline hrp::Vector2 to_Vector2(const bg_point& bg_point2d){ return hrp::Vector2(bg_point2d.x(), bg_point2d.y()); }
inline bg_polygon to_bg_hull(const hrp::dmatrix& hrp_hull){
    if(hrp_hull.rows() != 2){ std::cerr << "Invalid input for to_bg_hull" << std::endl; dbgn(hrp_hull); }
    bg_polygon hull_bg;
    hull_bg.outer().resize(hrp_hull.cols());
    for(int i=0; i<hrp_hull.cols(); i++){
        hull_bg.outer()[i] = bg_point(hrp_hull.col(i)(X), hrp_hull.col(i)(Y));
    }
    return hull_bg;
}

class LIP_model{
    private:
        double G, dt;
    public:
        LIP_model(){};
        double com_height;
        std::deque<hrp::Vector3> com;
        void update(const hrp::Vector3 com_new){ com.pop_back(); com.push_front(com_new); }
        hrp::Vector3 vel(){ return (com[1] - com[0]) / dt ; }
        hrp::Vector3 DCM(){ return com[0] + vel() * sqrt( com_height / G ); }
        hrp::Vector3 CCM(){ return com[0] - vel() * sqrt( com_height / G ); }
        hrp::Vector3 acc(){ return (com[2] - 2* com[1] + com[0]) / ( dt * dt ); }
//        hrp::Vector3 ZMP(){ return (hrp::Vector3() << com[0].head(XY) - acc().head(XY) * ( com_height / G ), com[0](Z) - com_height).finished; }
};


class BiquadIIRFilterVec{
    private:
        IIRFilter filters[XYZ];
        hrp::Vector3 ans;
    public:
        BiquadIIRFilterVec(){}
        ~BiquadIIRFilterVec(){}
        void setParameter(const hrp::Vector3& fc_in, const double& HZ, const double& Q = 0.5){ for(int i=0;i<XYZ;i++){ filters[i].setParameterAsBiquad((double)fc_in(i), Q, HZ); } }
        void setParameter(const double& fc_in, const double& HZ, const double& Q = 0.5){
            setParameter(hrp::Vector3(fc_in,fc_in,fc_in), HZ, Q);
        }//overload
        hrp::Vector3 passFilter(const hrp::Vector3& input){
            for(int i=0;i<XYZ;i++){
                ans(i) = filters[i].passFilter((double)input(i));
            }
            return ans;
        }
        void reset(const hrp::Vector3& initial_input){ for(int i=0;i<XYZ;i++){ filters[i].reset((double)initial_input(i));} }
};


class PoseTGT{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        hrp::Pose3 abs, offs, cnt;
        hrp::dvector6 w;
        bool is_contact, go_contact;
        PoseTGT(){ reset(); }
        ~PoseTGT(){}
        void reset(){ abs.reset(); offs.reset(); cnt.reset(); w.fill(0); is_contact = go_contact = false;}
};

class HumanPose{
    public:
        std::vector<PoseTGT> tgt;
        HumanPose(){ tgt.resize(num_pose_tgt); }
        ~HumanPose(){cerr<<"HumanPose destructed"<<endl;}
        void reset(){
            for(std::vector<PoseTGT>::iterator it = tgt.begin(); it != tgt.end(); it++){ it->reset(); }
        }
        static void hp_printf(const HumanPose& in) {
            const std::string pcatgt[] = {"c","rf","lf","rh","lh","hd","z"}, wcatgt[] = {"rw","lw"};
            for(int i=0;i<num_pose_tgt;i++){ fprintf(stderr,"\x1b[31m%s\x1b[39m%+05.2f %+05.2f %+05.2f ",pcatgt[i].c_str(),in.tgt[i].abs.p(X),in.tgt[i].abs.p(Y),in.tgt[i].abs.p(Z)); }
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
                if(hasStr(m_robot->joint(i)->name,"LEEIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix2d)G_JOINT3")){
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

class WBMSCore{
    private:
        double HZ, DT;
        BiquadIIRFilterVec calcacc_v_filters, acc4zmp_v_filters, com_in_filter;
        hrp::Vector3 com_old, com_oldold, comacc, com_CP_ref_old;
        std::vector<BiquadIIRFilterVec> tgt_pos_filters,tgt_rpy_filters;
        HumanPose rp_ref_out_old, hp_swap_checked;
        unsigned int loop;
        bool is_initial_loop;
        bool cp_force_go_contact[LR];
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        double H_cur;
        HumanPose hp_wld_raw, hp_plot, rp_ref_out, rp_ref_vel_old;
        FILE *sr_log, *cz_log, *id_log;
        hrp::Pose3 baselinkpose;
        hrp::Vector3 cp_dec, cp_acc;
        hrp::dmatrix foot_vert3d_act[LR], foot_vert3d_check[LR]; // A = [p0 p1 p2 p3]
        hrp::Vector2 com_forcp_ref,com_vel_forcp_ref;
        hrp::dvector6 invdyn_ft;
        hrp::Vector4 foot_vert_act_fblr[LR], foot_vert_safe_fblr[LR], foot_vert_check_fblr[LR];
        struct WBMSparameters {
            double auto_swing_foot_landing_threshold;
            double human_to_robot_ratio;
            bool set_com_height_fix;
            double set_com_height_fix_val;
            double swing_foot_height_offset;
            double upper_body_rmc_ratio;
            bool use_head;
            bool use_upper;
            bool use_lower;
            std::vector<std::string> use_joints;
            std::vector<std::string> use_targets;
        } wp;
        struct ActualRobotState {
            hrp::Vector3 com, zmp;
        } act_rs;

        WBMSCore(const double& dt){
            DT = dt;
            HZ = (int)(1.0/DT);
            loop = 0;
            //////////  hrp::Vector は初期化必要  ///////////
            com_old = com_oldold = comacc = hrp::Vector3::Zero();
            cp_dec = cp_acc = hrp::Vector3::Zero();
            invdyn_ft = hrp::dvector6::Zero();
            tgt_pos_filters.resize(num_pose_tgt);
            tgt_rpy_filters.resize(num_pose_tgt);
            for(int i=0;i<tgt_pos_filters.size();i++)tgt_pos_filters[i].setParameter(1.0, HZ, Q_NOOVERSHOOT);//四肢拘束点用(position)
            for(int i=0;i<tgt_rpy_filters.size();i++)tgt_rpy_filters[i].setParameter(1.0, HZ, Q_NOOVERSHOOT);//四肢拘束点用(Rotation)
            tgt_pos_filters[com].setParameter(1.0, HZ, Q_NOOVERSHOOT);//重心pos用
            tgt_rpy_filters[com].setParameter(0.6, HZ, Q_NOOVERSHOOT);//重心rot用
            tgt_pos_filters[rf].setParameter(hrp::Vector3(1.0,1.0,1.0), HZ, Q_NOOVERSHOOT);//右足pos用
            tgt_pos_filters[lf].setParameter(hrp::Vector3(1.0,1.0,1.0), HZ, Q_NOOVERSHOOT);//左足pos用
            calcacc_v_filters.setParameter(5, HZ, Q_BUTTERWORTH);//加速度計算用
            acc4zmp_v_filters.setParameter(5, HZ, Q_BUTTERWORTH);//ZMP生成用ほぼこの値でいい
            com_in_filter.setParameter(1, HZ);
            foot_vert_act_fblr[R] << 0.13, -0.10,  0.06, -0.08;
            foot_vert_act_fblr[L] << 0.13, -0.10,  0.08, -0.06;
            foot_vert_check_fblr[R] << 0.02+0.002, -0.01-0.002,  0.02+0.002,  0.01-0.002;
            foot_vert_check_fblr[L] << 0.02+0.002, -0.01-0.002, -0.01+0.002, -0.02-0.002;
            foot_vert_safe_fblr[R] << 0.02, -0.01,  0.02,  0.01;
            foot_vert_safe_fblr[L] << 0.02, -0.01, -0.01, -0.02;
            for(int i=0;i<LR;i++){
                foot_vert3d_act[i]      = make_rect_3d(foot_vert_act_fblr[i]);
                foot_vert3d_check[i]    = make_rect_3d(foot_vert_check_fblr[i]);
            }
            cp_force_go_contact[R] = false;
            cp_force_go_contact[L] = false;

            wp.auto_swing_foot_landing_threshold = 0.04;
            wp.human_to_robot_ratio = 1.0;//human 1.1m vs jaxon 1.06m
            wp.set_com_height_fix = false;
            wp.set_com_height_fix_val = 0.02;
            wp.swing_foot_height_offset = 0.02;
            wp.upper_body_rmc_ratio = 0.0;
            wp.use_head = true;
            wp.use_upper = true;
            wp.use_lower = true;
            rp_ref_out_old.reset();
            rp_ref_out.reset();
            rp_ref_vel_old.reset();
            if(DEBUG){
                sr_log = fopen("/home/ishiguro/HumanSync_support_region.log","w+");
                cz_log = fopen("/home/ishiguro/HumanSync_com_zmp.log","w+");
                id_log = fopen("/home/ishiguro/HumanSync_invdyn.log","w+");
            }
            is_initial_loop = true;
            cout<<"WBMSCore constructed"<<endl;
        }
        hrp::dmatrix make_rect_3d(const hrp::Vector4& ForwardBackLeftRight){
            return (hrp::dmatrix(3,4) << // (XYZ,4)とかの#defineマクロ効かない
                hrp::Vector3(ForwardBackLeftRight(0), ForwardBackLeftRight(2), 0),//左前
                hrp::Vector3(ForwardBackLeftRight(1), ForwardBackLeftRight(2), 0),//左後
                hrp::Vector3(ForwardBackLeftRight(1), ForwardBackLeftRight(3), 0),//右後
                hrp::Vector3(ForwardBackLeftRight(0), ForwardBackLeftRight(3), 0)).finished();//右前
        }
        ~WBMSCore(){
            if(DEBUG)fclose(sr_log);
            if(DEBUG)fclose(cz_log);
            if(DEBUG)fclose(id_log);
            cout<<"WBMSCore destructed"<<endl;
        }
        void initializeHumanPoseFromCurrentInput(){
            std::string ns[7] = {"com","rf","lf","rh","lh","zmp","head"};
            for(int i=0;i<7;i++){
                hp_wld_raw.tgt[i].offs = hp_wld_raw.tgt[i].abs;
            }
        }
        void initializeRobotPoseFromHRPBody(const hrp::BodyPtr robot_in, std::map<std::string, IKConstraint>& _ee_ikc_map){
            const std::string robot_l_names[4] = {"rleg","lleg","rarm","larm"};
            const int human_l_names[4] = {rf,lf,rh,lh};
            for(int i=0;i<4;i++){//HumanSynchronizerの初期姿勢オフセットをセット
                if(_ee_ikc_map.count(robot_l_names[i])){
                    rp_ref_out.tgt[human_l_names[i]].offs.p = _ee_ikc_map[robot_l_names[i]].getCurrentTargetPos(robot_in);//          fik_in->getEndEffectorPos(robot_l_names[i]);
                    rp_ref_out.tgt[human_l_names[i]].offs.R = _ee_ikc_map[robot_l_names[i]].getCurrentTargetRot(robot_in);
                    rp_ref_out.tgt[human_l_names[i]].abs = rp_ref_out.tgt[human_l_names[i]].offs;
                }
            }
            rp_ref_out.tgt[com].offs.p = act_rs.com = act_rs.zmp = robot_in->calcCM();
            com_CP_ref_old = rp_ref_out.tgt[com].offs.p;
            rp_ref_out.tgt[com].offs.R = robot_in->rootLink()->R;
            rp_ref_out.tgt[zmp].offs.p(X) = rp_ref_out.tgt[com].offs.p(X);
            rp_ref_out.tgt[zmp].offs.p(Y) = rp_ref_out.tgt[com].offs.p(Y);
            rp_ref_out.tgt[zmp].offs.p(Z) = (rp_ref_out.tgt[rf].offs.p(Z) + rp_ref_out.tgt[lf].offs.p(Z)) / 2;
            rp_ref_out.tgt[rf].cnt = rp_ref_out.tgt[rf].offs;
            rp_ref_out.tgt[lf].cnt = rp_ref_out.tgt[lf].offs;
            baselinkpose.p = robot_in->rootLink()->p;
            baselinkpose.R = robot_in->rootLink()->R;
            for(int i=0, l[LR]={rf,lf}; i<LR; i++){
                rp_ref_out.tgt[l[i]].is_contact = true;
            }
            H_cur = rp_ref_out.tgt[com].offs.p(Z) - std::min((double)rp_ref_out.tgt[rf].offs.p(Z), (double)rp_ref_out.tgt[rf].offs.p(Z));
        }
        void initializeRequest(hrp::BodyPtr robot_in, std::map<std::string, IKConstraint>& _ee_ikc_map){
            loop = 0;
            is_initial_loop = true;
            initializeHumanPoseFromCurrentInput();
            initializeRobotPoseFromHRPBody(robot_in, _ee_ikc_map);
            rp_ref_out_old = rp_ref_vel_old = rp_ref_out;
        }
        void update(){//////////  メインループ  ////////////
            autoLRSwapCheck                     (hp_wld_raw, hp_swap_checked);//入力の左右反転を常にチェック(＝手足の交差は不可能)(動作中に入れ替わると余計なお世話かも)
            convertHumanToRobot                 (hp_swap_checked, rp_ref_out);
            hp_plot = rp_ref_out;
            if(wp.set_com_height_fix) rp_ref_out.tgt[com].abs.p(Z) = rp_ref_out.tgt[com].offs.p(Z) + wp.set_com_height_fix_val;//膝曲げトルクで落ちるときの応急措置
            lockSwingFootIfZMPOutOfSupportFoot  (rp_ref_out_old, rp_ref_out);//
            limitEEWorkspace                    (rp_ref_out);
            setFootContactPoseByGoContact       (rp_ref_out);
            applyCOMToSupportRegionLimit        (rp_ref_out.tgt[rf].abs, rp_ref_out.tgt[lf].abs, rp_ref_out.tgt[com].abs.p);
            overwriteFootZFromFootLandOnCommand (rp_ref_out);
            setFootRotHorizontalIfGoLanding     (rp_ref_out);
            limitFootVelNearGround              (rp_ref_out);
            applyCOMToSupportRegionLimit        (rp_ref_out.tgt[rf].abs, rp_ref_out.tgt[lf].abs, com_CP_ref_old);//これやらないと支持領域の移動によって1ステップ前のCOM位置はもうはみ出てるかもしれないから
            com_forcp_ref = rp_ref_out.tgt[com].abs.p.head(XY);
            static hrp::Vector2 com_forcp_ref_old;
            com_vel_forcp_ref = (com_forcp_ref - com_forcp_ref_old)/DT;
            com_forcp_ref_old = com_forcp_ref;
            applyCOMStateLimitByCapturePoint    (rp_ref_out.tgt[com].abs.p, rp_ref_out.tgt[rf].abs, rp_ref_out.tgt[lf].abs, com_CP_ref_old, rp_ref_out.tgt[com].abs.p);
            applyCOMToSupportRegionLimit        (rp_ref_out.tgt[rf].abs, rp_ref_out.tgt[lf].abs, rp_ref_out.tgt[com].abs.p);
//            applyLPFilter_post                       (rp_ref_out);
            applyZMPCalcFromCOM                 (rp_ref_out.tgt[com].abs.p, rp_ref_out.tgt[zmp].abs.p);
            modifyFootRotAndXYForContact        (rp_ref_out);
            H_cur = rp_ref_out.tgt[com].abs.p(Z) - std::min((double)rp_ref_out.tgt[rf].abs.p(Z), (double)rp_ref_out.tgt[lf].abs.p(Z));
            rp_ref_out_old = rp_ref_out;
            loop++;
            is_initial_loop = false;
        }

    private:
        void autoLRSwapCheck(const HumanPose& in, HumanPose& out){
            out = in;
            if(in.tgt[rh].offs.p(Y) > in.tgt[lh].offs.p(Y) ){ out.tgt[rh] = in.tgt[lh]; out.tgt[lh] = in.tgt[rh]; }
            if(in.tgt[rf].offs.p(Y) > in.tgt[lf].offs.p(Y) ){ out.tgt[rf] = in.tgt[lf]; out.tgt[lf] = in.tgt[rf]; }
        }
        void convertHumanToRobot(const HumanPose& in, HumanPose& out){//結局初期指定からの移動量(=Rel)で計算をしてゆく
            //      out = in;//ダメゼッタイ
            for(int i=0, l[6]={com,rf,lf,rh,lh,head}; i<6; i++){
                out.tgt[l[i]].abs.p = wp.human_to_robot_ratio * (in.tgt[l[i]].abs.p - in.tgt[l[i]].offs.p) + out.tgt[l[i]].offs.p;
                out.tgt[l[i]].abs.R = in.tgt[l[i]].abs.R * in.tgt[l[i]].offs.R.transpose() * out.tgt[l[i]].offs.R;
            }
            for(int i=0, l[4]={rf,lf,rh,lh}; i<4; i++){
                out.tgt[l[i]].w = in.tgt[l[i]].w;
                out.tgt[l[i]].go_contact = in.tgt[l[i]].go_contact;
            }
            out.tgt[zmp].abs = in.tgt[zmp].abs;//最近使わない
        }
        void lockSwingFootIfZMPOutOfSupportFoot(const HumanPose& old, HumanPose& out){
            hrp::Vector3 inside_vec_rf = old.tgt[rf].abs.R * hrp::Vector3(0,1,0);
            hrp::Vector3 inside_vec_lf = old.tgt[lf].abs.R * hrp::Vector3(0,-1,0);
            hrp::Vector2 rf2zmp = act_rs.zmp.head(XY) - old.tgt[rf].abs.p.head(XY);
            hrp::Vector2 lf2zmp = act_rs.zmp.head(XY) - old.tgt[lf].abs.p.head(XY);
            if( inside_vec_rf.head(XY).dot(rf2zmp) / inside_vec_rf.norm() > wp.auto_swing_foot_landing_threshold ){ out.tgt[lf].go_contact = true; }//2D,3D混ざって微妙
            if( inside_vec_lf.head(XY).dot(lf2zmp) / inside_vec_lf.norm() > wp.auto_swing_foot_landing_threshold ){ out.tgt[rf].go_contact = true; }
        }
        void limitEEWorkspace(HumanPose& out){
            const double MAX_FW = 0.25;
            const double FOOT_2_FOOT_COLLISION_MARGIIN = 0.16;
            for(int i=0, spl[LR]={rf,lf}, swl[LR]={lf,rf}; i<LR; i++){
                PoseTGT& support_leg = out.tgt[spl[i]];
                PoseTGT& swing_leg = out.tgt[swl[i]];
                hrp::Vector2 inside_vec_baserel = ( spl[i]==rf ? hrp::Vector2(0,+1) : hrp::Vector2(0,-1) );
                hrp::Vector2 sp2sw_vec = swing_leg.abs.p.head(XY) - support_leg.cnt.p.head(XY);
                if(swing_leg.go_contact && sp2sw_vec.norm() > MAX_FW){ sp2sw_vec = sp2sw_vec.normalized() * MAX_FW; }//着地時に足を広げすぎないよう制限
                Eigen::Matrix2d base_rot;
                base_rot = Eigen::Rotation2Dd(baselinkpose.rpy()(y));
                hrp::Vector2 sp2sw_vec_baserel = base_rot.transpose() * sp2sw_vec;
                if( sp2sw_vec_baserel.dot(inside_vec_baserel) < FOOT_2_FOOT_COLLISION_MARGIIN){
                    sp2sw_vec_baserel += inside_vec_baserel * (FOOT_2_FOOT_COLLISION_MARGIIN - sp2sw_vec_baserel.dot(inside_vec_baserel));
                }
                sp2sw_vec = base_rot * sp2sw_vec_baserel;
                swing_leg.abs.p.head(XY) = sp2sw_vec + support_leg.cnt.p.head(XY);
            }
            const double base2hand_min = 0.4;// for jaxon demo
            for(int i=0, l[2]={rh,lh}; i<2; i++){
                hrp::Vector2 horizontal_dist = out.tgt[l[i]].abs.p.head(XY) - baselinkpose.p.head(XY);
                if(horizontal_dist.norm() < base2hand_min){
                    out.tgt[l[i]].abs.p.head(XY) = baselinkpose.p.head(XY) + base2hand_min * horizontal_dist.normalized();
                }
            }
        }
        void setFootContactPoseByGoContact(HumanPose& out){
            for(int i=0, l[LR]={rf,lf}; i<LR; i++){
                if(out.tgt[l[i]].go_contact){
                    out.tgt[l[i]].abs.p.head(XY) = out.tgt[l[i]].cnt.p.head(XY);
                    out.tgt[l[i]].abs.setRpy(out.tgt[l[i]].abs.rpy()(r), out.tgt[l[i]].abs.rpy()(p), out.tgt[l[i]].cnt.rpy()(y));

                }else{
                    out.tgt[l[i]].cnt.p.head(XY) = out.tgt[l[i]].abs.p.head(XY);
                    out.tgt[l[i]].cnt.setRpy(out.tgt[l[i]].cnt.rpy()(r), out.tgt[l[i]].cnt.rpy()(p), out.tgt[l[i]].abs.rpy()(y));
                }
            }
        }
        bool applyCOMToSupportRegionLimit(const hrp::Pose3& rfin_abs, const hrp::Pose3& lfin_abs, hrp::Vector3& comin_abs){//boost::geometryがUbuntu12だとないから・・・
            hrp::dmatrix hull_com = createSupportRegionByFootPos(rfin_abs, lfin_abs, foot_vert_safe_fblr[R], foot_vert_safe_fblr[L]);
            if(!isPointInHull2D(comin_abs.head(XY), hull_com)){
                comin_abs.head(XY) = calcNearestPointOnHull(comin_abs.head(XY), hull_com);
            }//外に出たら最近傍点に頭打ち
            return true;
        }
        void overwriteFootZFromFootLandOnCommand(HumanPose& out){
            for(int i=0, l[LR]={rf,lf}; i<LR; i++){
                if(out.tgt[l[i]].go_contact || cp_force_go_contact[i]){
                    out.tgt[l[i]].abs.p(Z) = out.tgt[l[i]].offs.p(Z);
                }else{
                    LIMIT_MIN( out.tgt[l[i]].abs.p(Z), out.tgt[l[i]].offs.p(Z)+wp.swing_foot_height_offset);
                }
                const double contact_threshold = 0.005;
                out.tgt[l[i]].is_contact = ((out.tgt[l[i]].abs.p(Z)-out.tgt[l[i]].offs.p(Z)) <= contact_threshold);
            }
        }
        void setFootRotHorizontalIfGoLanding(HumanPose& out){
            if( out.tgt[rf].go_contact || cp_force_go_contact[R]){
                out.tgt[rf].abs.setRpy(0, 0, out.tgt[rf].abs.rpy()(y));
            }
            if( out.tgt[lf].go_contact || cp_force_go_contact[L]){
                out.tgt[lf].abs.setRpy(0, 0, out.tgt[lf].abs.rpy()(y));
            }
        }
        void limitFootVelNearGround(HumanPose& out){
            for(int i=0, l[2]={rf,lf}; i<2; i++){
                double fheight = rp_ref_out_old.tgt[l[i]].abs.p(Z) - rp_ref_out_old.tgt[l[i]].offs.p(Z);
                double horizontal_max_vel = 0 + fheight * 20;
                for(int j=0; j<XY; j++){
                    LIMIT_MINMAX( out.tgt[l[i]].abs.p(j), rp_ref_out_old.tgt[l[i]].abs.p(j) - horizontal_max_vel * DT, rp_ref_out_old.tgt[l[i]].abs.p(j) + horizontal_max_vel * DT);
                }
                double vertical_max_vel = 0 + fheight * 10;
                LIMIT_MIN( out.tgt[l[i]].abs.p(Z), rp_ref_out_old.tgt[l[i]].abs.p(Z) - vertical_max_vel * DT);
            }
        }
        void modifyFootRotAndXYForContact(HumanPose& out){
            avoidFootSinkIntoFloor(out.tgt[rf],foot_vert3d_act[R]);
            avoidFootSinkIntoFloor(out.tgt[lf],foot_vert3d_act[L]);
        }
        void avoidFootSinkIntoFloor(PoseTGT& foot_in, const hrp::dmatrix& act_sole_vert){
            hrp::dmatrix act_sole_vert_abs = (foot_in.abs.R * act_sole_vert).colwise() + foot_in.abs.p;// translate and rotate each cols
            double min_height = act_sole_vert_abs.row(Z).minCoeff(); // pick up min Z height
            if(min_height < foot_in.offs.p(Z)){
                foot_in.abs.p(Z) += (foot_in.offs.p(Z) - min_height);
            }
        }
        bool applyCOMStateLimitByCapturePoint(const hrp::Vector3& com_in, const hrp::Pose3& rfin_abs, const hrp::Pose3& lfin_abs, hrp::Vector3& com_ans_old, hrp::Vector3& com_ans){
            com_ans = com_in;
            if (is_initial_loop)com_ans_old = com_in;
            hrp::Vector3 com_vel = (com_in - com_ans_old)/DT;
            hrp::Vector4 marginDelta_for_dcp(+0.001,-0.001,0.001,-0.001);
            hrp::Vector4 marginDelta_for_acp(+0.010,-0.010,0.010,-0.010);
            hrp::dmatrix hull_dcp = createSupportRegionByFootPos(rfin_abs, lfin_abs, foot_vert_safe_fblr[R]+marginDelta_for_dcp, foot_vert_safe_fblr[L]+marginDelta_for_dcp);
            hrp::dmatrix hull_acp = createSupportRegionByFootPos(rfin_abs, lfin_abs, foot_vert_safe_fblr[R]+marginDelta_for_acp, foot_vert_safe_fblr[L]+marginDelta_for_acp);
            hrp::Vector2 com_vel_ans_2d;
            regulateCOMVelocityByCapturePointVec( com_ans_old.head(XY), com_vel.head(XY), hull_dcp, hull_acp, com_vel_ans_2d);
            com_ans.head(XY) = com_ans_old.head(XY) + com_vel_ans_2d * DT;
            com_ans_old = com_ans;
            cp_dec.head(XY) = com_ans.head(XY) + com_vel_ans_2d * sqrt( H_cur / G );
            cp_acc.head(XY) = com_ans.head(XY) - com_vel_ans_2d * sqrt( H_cur / G );
            return true;
        }
        void regulateCOMVelocityByCapturePointVec(const hrp::Vector2& com_pos, const hrp::Vector2& com_vel, const hrp::dmatrix& hull_d, const hrp::dmatrix& hull_a, hrp::Vector2& com_vel_ans){
            hrp::Vector2 com_vel_decel_ok,com_vel_accel_ok;
            com_vel_decel_ok = com_vel_accel_ok = com_vel_ans = com_vel;

            const double foot_ave_vel = 2.0;
            double lf_landing_delay = (rp_ref_out.tgt[lf].abs.p[Z] - rp_ref_out.tgt[lf].offs.p[Z]) / foot_ave_vel;
            double rf_landing_delay = (rp_ref_out.tgt[rf].abs.p[Z] - rp_ref_out.tgt[rf].offs.p[Z]) / foot_ave_vel;
            if(com_vel(Y)>0 ){ rf_landing_delay = 0; }
            if(com_vel(Y)<0 ){ lf_landing_delay = 0; }
            double foot_landing_delay = std::max(rf_landing_delay, lf_landing_delay);
            //減速CP条件(現在のCPを常に両足裏で頭打ち)
            //      hrp::Vector2 inside_vec_rf(0,1),inside_vec_lf(0,-1);//TODO implement
            //      if(com_vel(Y)>1e-5 || fabs(com_vel(Y))<=1e-5 ){
            //        foot_landing_delay = 0;
            //      }
            hrp::Vector2 cp_dec_ragulated = com_pos + com_vel * ( sqrt( H_cur / G ) + foot_landing_delay);
            if(!isPointInHull2D(cp_dec_ragulated, hull_d)){
                calcCrossPointOnHull(com_pos, cp_dec_ragulated, hull_d, cp_dec_ragulated);
                com_vel_decel_ok = (cp_dec_ragulated - com_pos) / ( sqrt( H_cur / G ) + foot_landing_delay);
            }

            //ついでに減速CPによる強制遊脚着地も判定
            hrp::dmatrix foot_vert3d_check_wld[LR], one_foot_hull2d[LR];
            for(int i=0; i<LR; i++){
                foot_vert3d_check_wld[i] = (rp_ref_out.tgt[F(i)].abs.R * foot_vert3d_check[i]).colwise() + rp_ref_out.tgt[F(i)].abs.p; // rotate and translate in 3D
                one_foot_hull2d[i] = makeConvexHull2D( foot_vert3d_check_wld[i].topRows(XY) ); // project into 2D and make convex hull
            }
            cp_force_go_contact[L] = !isPointInHull2D(cp_dec_ragulated, one_foot_hull2d[R]); // if CapturePoint go out of R sole region, L foot must be go contact
            cp_force_go_contact[R] = !isPointInHull2D(cp_dec_ragulated, one_foot_hull2d[L]); // if CapturePoint go out of L sole region, R foot must be go contact

            //加速CP条件(ACP使用)
            hrp::Vector2 cp_acc_ragulated = com_pos - com_vel * sqrt( H_cur / G );
            if(!isPointInHull2D(cp_acc_ragulated, hull_a)){
                calcCrossPointOnHull(com_pos, cp_acc_ragulated, hull_a, cp_acc_ragulated);
                com_vel_accel_ok = (-cp_acc_ragulated + com_pos ) / sqrt( H_cur / G );
            }

            //加速減速条件マージ com_vel進行方向をより減速させるものを採用？
            com_vel_ans = (com_vel_decel_ok.dot(com_vel) < com_vel_accel_ok.dot(com_vel)) ? com_vel_decel_ok : com_vel_accel_ok; //normじゃダメ？
        }
        hrp::dmatrix createSupportRegionByFootPos(const hrp::Pose3& rfin_abs, const hrp::Pose3& lfin_abs, const hrp::Vector4& rf_mgn, const hrp::Vector4& lf_mgn){
            hrp::dmatrix rf_sole_verts_abs = (rfin_abs.R * make_rect_3d(rf_mgn)).colwise() + rfin_abs.p;
            hrp::dmatrix lf_sole_verts_abs = (lfin_abs.R * make_rect_3d(lf_mgn)).colwise() + lfin_abs.p;
            hrp::dmatrix both_sole_verts_abs = (hrp::dmatrix(3, rf_sole_verts_abs.cols()+lf_sole_verts_abs.cols()) << rf_sole_verts_abs, lf_sole_verts_abs).finished();
            return makeConvexHull2D(both_sole_verts_abs.topRows(XY));
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
        bool calcCrossPointOnHull(const hrp::Vector2& inside_start_pt, const hrp::Vector2& outside_goal_pt, const hrp::dmatrix& hull, hrp::Vector2& ans_cross_pt){
            if(hull.rows() != 2){ std::cerr << "Invalid input for calcCrossPointOnHull" << std::endl; dbgn(hull); }
            bg_linestring anchor_vec;
            anchor_vec.push_back(to_bg_point(inside_start_pt));
            anchor_vec.push_back(to_bg_point(outside_goal_pt));
            bg_multi_point bg_ans_cross_pt;
            bg::intersection(anchor_vec, to_bg_hull(hull), bg_ans_cross_pt);
            switch(bg_ans_cross_pt.size()){
                case 0:
                    return false;
                case 1:
                    ans_cross_pt = to_Vector2(bg_ans_cross_pt[0]);
                    return true;
                default:
                    std::cerr << "Number of the cross point must be 0 or 1 (current = " << bg_ans_cross_pt.size() << ") something wrong!" << std::endl;
                    return false;
            }
        }
        hrp::Vector2 calcNearestPointOnHull(const hrp::Vector2& tgt_pt, const hrp::dmatrix& hull){
            if(hull.rows() != 2){ std::cerr << "Invalid input for calcNearestPointOnHull" << std::endl; dbgn(hull); }
            double cur_nearest_dist, ans_nearest_dist;
            hrp::Vector2 cur_nearest_pt, ans_nearest_pt;
            for(int i=0; i<hull.cols()-1; i++){// first and end point in hull are same
                const hrp::Vector2 cur_pt = hull.col(i), nxt_pt = hull.col(i+1);
                const hrp::Vector2 cur_edge = nxt_pt - cur_pt;
                const hrp::Vector2 tgt_pt_v = tgt_pt - cur_pt;
                double cur_pt_to_projected_tgt_pt = tgt_pt_v.dot(cur_edge.normalized()); // Distance from cur_pt to the projected tgt_pt on to cur_edge
                LIMIT_MINMAX(cur_pt_to_projected_tgt_pt, 0, cur_edge.norm()); // limit the nearest point onto the line segment "cur_edge"
                cur_nearest_pt = cur_pt + cur_pt_to_projected_tgt_pt * cur_edge.normalized();
                cur_nearest_dist = (tgt_pt - cur_nearest_pt).norm();
                if( cur_nearest_dist < ans_nearest_dist || i==0 ){//update nearest candidate
                    ans_nearest_dist = cur_nearest_dist;
                    ans_nearest_pt = cur_nearest_pt;
                }
            }
            return ans_nearest_pt;
        }
        hrp::dmatrix makeConvexHull2D(const hrp::dmatrix& pts_2d){
//            return makeConvexHull2D_OpenCV(pts_2d);
            return makeConvexHull2D_Boost(pts_2d);
        }
        bool isPointInHull2D(const hrp::Vector2& pt, const hrp::dmatrix& hull){
//            return isPointInHull2D_OpenCV(pt, hull);
            return isPointInHull2D_Boost(pt, hull);
        }
        hrp::dmatrix makeConvexHull2D_OpenCV(const hrp::dmatrix& pts_2d){
            if(pts_2d.rows() != 2){ std::cerr << "Invalid input for makeConvexHull2D_OpenCV" << std::endl; dbgn(pts_2d); }
            cv::Mat pts_2d_cv;
            std::vector<cv::Point2f> points(pts_2d.cols()), hull_2d_cv;
            for(int i=0; i<pts_2d.cols(); i++){
                points[i] = cv::Point2f(pts_2d.col(i)(X), pts_2d.col(i)(Y));
            }
            cv::convexHull( cv::Mat(points), hull_2d_cv, true);
            hrp::dmatrix hull_2d(2, hull_2d_cv.size());
            for(int i=0; i<hull_2d_cv.size(); i++){
                hull_2d.col(i) << hull_2d_cv[i].x,  hull_2d_cv[i].y;
            }
            return hull_2d;
        }
        hrp::dmatrix makeConvexHull2D_Boost(const hrp::dmatrix& pts_2d){
            if(pts_2d.rows() != 2){ std::cerr << "Invalid input for makeConvexHull_Boost" << std::endl; dbgn(pts_2d); }
            bg_multi_point tmp;
            tmp.resize(pts_2d.cols());
            for(int i=0; i<pts_2d.cols(); i++){
                tmp[i] = to_bg_point(pts_2d.col(i));
            }
            bg_polygon hull_bg;
            bg::convex_hull(tmp, hull_bg);
            hrp::dmatrix hull_2d(2, hull_bg.outer().size());
            for(int i=0; i<hull_bg.outer().size(); i++){
                hull_2d.col(i) << to_Vector2(hull_bg.outer()[i]);
            }
            return hull_2d;
        }
        bool isPointInHull2D_OpenCV(const hrp::Vector2& pt, const hrp::dmatrix& hull){
            if(hull.rows() != 2){ std::cerr << "Invalid input for isPointInHull2dOpenCV" << std::endl; dbgn(hull); }
            std::vector<cv::Point2f> hull_cv(hull.cols());
            for(int i=0; i<hull.cols(); i++){
                hull_cv[i] = cv::Point2f(hull.col(i)(X), hull.col(i)(Y));
            }
            return (cv::pointPolygonTest(cv::Mat(hull_cv), cv::Point2f(pt(X),pt(Y)), false) > 0);
        }
        bool isPointInHull2D_Boost(const hrp::Vector2& pt, const hrp::dmatrix& hull){
            if(hull.rows() != 2){ std::cerr << "Invalid input for isPointInHull2D_Boost" << std::endl; dbgn(hull); }
            return bg::within(to_bg_point(pt), to_bg_hull(hull));
        }
        void applyLPFilter_post(HumanPose& tgt){
            if(is_initial_loop){
                for(int i=0, l[1]={com}; i<1; i++){
                    tgt_pos_filters[l[i]].reset(tgt.tgt[l[i]].abs.p);
                    tgt_rpy_filters[l[i]].reset(tgt.tgt[l[i]].abs.rpy());
                }
            }
            for(int i=0, l[1]={com}; i<1; i++){
                tgt.tgt[l[i]].abs.p   = tgt_pos_filters[l[i]].passFilter(tgt.tgt[l[i]].abs.p);
                tgt.tgt[l[i]].abs.setRpy(tgt_rpy_filters[l[i]].passFilter(tgt.tgt[l[i]].abs.rpy()));
            }
        }
        void applyZMPCalcFromCOM(const hrp::Vector3& comin, hrp::Vector3& zmpout){
            comacc = (comin - 2 * com_old + com_oldold)/(DT*DT);
            comacc = acc4zmp_v_filters.passFilter(comacc);
            zmpout.head(XY) = comin.head(XY)-(H_cur/G)*comacc.head(XY);
            com_oldold = com_old;
            com_old = comin;
        }
};


#endif // WBMS_CORE_H
