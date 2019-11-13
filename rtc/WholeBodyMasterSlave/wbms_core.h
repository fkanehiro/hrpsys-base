#ifndef WBMS_CORE_H
#define WBMS_CORE_H

#include <hrpUtil/Eigen4d.h>
#include <hrpCollision/DistFuncs.h>
#include "../SequencePlayer/interpolator.h"
#include "../ImpedanceController/JointPathEx.h"
#include "../AutoBalancer/FullbodyInverseKinematicsSolver.h"
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

class PoseTGT{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        hrp::Pose3 abs, offs;
        hrp::dvector6 w;
        bool go_contact;
        PoseTGT(){ reset(); }
        ~PoseTGT(){}
        void reset(){ abs.reset(); offs.reset(); w.fill(0); go_contact = false;}
        bool is_contact(){
            const double contact_threshold = 0.005;
            return ((abs.p(Z) - offs.p(Z)) < contact_threshold);
        }
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
        PoseTGT&                        foot    (const int lr)      { assert(lr == R || lr == L); return (lr == R) ? tgt[rf] : tgt[lf]; }
        const PoseTGT&                  foot    (const int lr) const{ assert(lr == R || lr == L); return (lr == R) ? tgt[rf] : tgt[lf]; }
        PoseTGT&                        hand    (const int lr)      { assert(lr == R || lr == L); return (lr == R) ? tgt[rh] : tgt[lh]; }
        const PoseTGT&                  hand    (const int lr) const{ assert(lr == R || lr == L); return (lr == R) ? tgt[rh] : tgt[lh]; }
        PoseTGT& tgt_by_str(const std::string ln){
            assert(ln == "com" || ln == "head" || ln == "lleg" || ln == "rleg" || ln == "larm" || ln == "rarm" );
            if      (ln == "lleg"){ return tgt[lf]; }
            else if (ln == "rleg"){ return tgt[rf]; }
            else if (ln == "larm"){ return tgt[lh]; }
            else if (ln == "rarm"){ return tgt[rh]; }
            else if (ln == "com" ){ return tgt[com]; }
            else                  { return tgt[head]; }
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
        hrp::Vector3 com_old, com_oldold, comacc, com_CP_ref_old;
        HumanPose rp_ref_out_old;
        unsigned int loop;
        bool is_initial_loop;
        bool cp_force_go_contact[LR];
        int zmp_force_go_contact_count[LR];
        BiquadIIRFilterVec acc4zmp_v_filters, com_filter;
        double com_filter_cutoff_hz_old;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        double H_cur;
        HumanPose hp_wld_raw, rp_ref_out;
        hrp::Pose3 baselinkpose;
        hrp::Vector3 cp_dec, cp_acc;
        hrp::Vector4 act_foot_vert_fblr[LR], safe_foot_vert_fblr[LR];
        hrp::Vector2 com_forcp_ref,com_vel_forcp_ref;
        class WBMSParams {
            public:
                bool auto_com_mode;
                double additional_double_support_time;
                double auto_com_foot_move_detect_height;
                double base_to_hand_min_distance;
                double capture_point_extend_ratio;
                double com_filter_cutoff_hz;
                double foot_collision_avoidance_distance;
                double human_to_robot_ratio;
                double max_double_support_width;
                double upper_body_rmc_ratio;
                double single_foot_zmp_safety_distance;
                double swing_foot_height_offset;
                hrp::Vector3 com_offset;
                hrp::Vector4 actual_foot_vert_fbio;
                hrp::Vector4 safety_foot_vert_fbio;
                std::vector<std::string> use_joints;
                std::vector<std::string> use_targets;
            WBMSParams(){
                auto_com_mode                       = false;
                additional_double_support_time      = 0.5;
                auto_com_foot_move_detect_height    = 0.03;
                base_to_hand_min_distance           = 0.5;
                capture_point_extend_ratio          = 1.0;
                com_filter_cutoff_hz                = 1.5;
                foot_collision_avoidance_distance   = 0.16;
                human_to_robot_ratio                = 1.0;//human 1.1m vs jaxon 1.06m
                max_double_support_width            = 0.4;
                upper_body_rmc_ratio                = 0.0;
                single_foot_zmp_safety_distance     = 0.04;
                swing_foot_height_offset            = 0.02;
                com_offset                          << 0, 0, 0;
                actual_foot_vert_fbio               << 0.13, -0.10,  0.06, -0.08;
                safety_foot_vert_fbio               << 0.02, -0.02,  0.02,  0.01;
            }
        } wp;
        struct ActualRobotState {
            hrp::Vector3 com, zmp, st_zmp;
        } act_rs;

        WBMSCore(const double& dt){
            DT = dt;
            HZ = (int)(1.0/DT);
            loop = 0;
            com_old = com_oldold = comacc = hrp::Vector3::Zero();
            cp_dec = cp_acc = hrp::Vector3::Zero();
            acc4zmp_v_filters.resize(XYZ);
            acc4zmp_v_filters.setParameter(5, HZ, Q_BUTTERWORTH);//ZMP生成用ほぼこの値でいい
            com_filter.resize(XYZ);
            com_filter.setParameter(wp.com_filter_cutoff_hz, HZ, Q_NOOVERSHOOT);
            for(int lr=0; lr<LR; lr++){
                act_foot_vert_fblr[lr]          = fbio2fblr(wp.actual_foot_vert_fbio, lr);
                safe_foot_vert_fblr[lr]         = fbio2fblr(wp.safety_foot_vert_fbio, lr);
                cp_force_go_contact[lr]         = false;
                zmp_force_go_contact_count[lr]  = 0;
            }
            rp_ref_out_old.reset();
            rp_ref_out.reset();
            is_initial_loop = true;
            cout<<"WBMSCore constructed"<<endl;
        }
        ~WBMSCore(){ cout<<"WBMSCore destructed"<<endl; }
        hrp::dmatrix make_rect_3d(const hrp::Vector4& ForwardBackLeftRight){ // A = [p0 p1 p2 p3]
            return (hrp::dmatrix(3,4) << // (XYZ,4)とかの#defineマクロ効かない
                hrp::Vector3(ForwardBackLeftRight(0), ForwardBackLeftRight(2), 0),//左前
                hrp::Vector3(ForwardBackLeftRight(1), ForwardBackLeftRight(2), 0),//左後
                hrp::Vector3(ForwardBackLeftRight(1), ForwardBackLeftRight(3), 0),//右後
                hrp::Vector3(ForwardBackLeftRight(0), ForwardBackLeftRight(3), 0)).finished();//右前
        }
        hrp::Vector4 offset_fblr(const hrp::Vector4& in, const double offset){
            return (hrp::Vector4()<< in(0)+offset, in(1)-offset, in(2)+offset, in(3)-offset).finished();//右前
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
                    rp_ref_out.tgt[human_l_names[i]].abs = rp_ref_out.tgt[human_l_names[i]].offs = _ee_ikc_map[robot_l_names[i]].getCurrentTargetPose(robot_in);//          fik_in->getEndEffectorPos(robot_l_names[i]);
                }
            }
            rp_ref_out.tgt[com].offs.p = act_rs.com = act_rs.zmp = act_rs.st_zmp = robot_in->calcCM();
            com_CP_ref_old = rp_ref_out.tgt[com].offs.p;
            rp_ref_out.tgt[com].offs.R = robot_in->rootLink()->R;
            rp_ref_out.tgt[zmp].offs.p.head(XY) = rp_ref_out.tgt[com].offs.p.head(XY);
            rp_ref_out.tgt[zmp].offs.p(Z) = (rp_ref_out.tgt[rf].offs.p(Z) + rp_ref_out.tgt[lf].offs.p(Z)) / 2;
            baselinkpose.p = robot_in->rootLink()->p;
            baselinkpose.R = robot_in->rootLink()->R;
            H_cur = rp_ref_out.tgt[com].offs.p(Z) - std::min((double)rp_ref_out.tgt[rf].offs.p(Z), (double)rp_ref_out.tgt[rf].offs.p(Z));
        }
        void initializeRequest(hrp::BodyPtr robot_in, std::map<std::string, IKConstraint>& _ee_ikc_map){
            loop = 0;
            is_initial_loop = true;
            initializeHumanPoseFromCurrentInput();
            initializeRobotPoseFromHRPBody(robot_in, _ee_ikc_map);
            rp_ref_out_old = rp_ref_out;
        }
        void update(){//////////  メインループ  ////////////
            convertHumanToRobot                 (hp_wld_raw, rp_ref_out);
            setAutoCOMMode                      (hp_wld_raw, rp_ref_out_old, rp_ref_out);

            lockSwingFootIfZMPOutOfSupportFoot  (rp_ref_out_old, rp_ref_out);//
            limitEEWorkspace                    (rp_ref_out_old, rp_ref_out);
            setFootContactPoseByGoContact       (rp_ref_out_old, rp_ref_out);
            limitFootVelNearGround              (rp_ref_out_old, rp_ref_out);
            avoidFootSinkIntoFloor              (rp_ref_out);

            applyCOMToSupportRegionLimit        (rp_ref_out.tgt[rf].abs, rp_ref_out.tgt[lf].abs, rp_ref_out.tgt[com].abs.p);
            applyCOMToSupportRegionLimit        (rp_ref_out.tgt[rf].abs, rp_ref_out.tgt[lf].abs, com_CP_ref_old);//これやらないと支持領域の移動によって1ステップ前のCOM位置はもうはみ出てるかもしれないから
            com_forcp_ref = rp_ref_out.tgt[com].abs.p.head(XY);
            static hrp::Vector2 com_forcp_ref_old;
            com_vel_forcp_ref = (com_forcp_ref - com_forcp_ref_old)/DT;
            com_forcp_ref_old = com_forcp_ref;
            applyCOMStateLimitByCapturePoint    (rp_ref_out.tgt[com].abs.p, rp_ref_out.tgt[rf].abs, rp_ref_out.tgt[lf].abs, com_CP_ref_old, rp_ref_out.tgt[com].abs.p);
            applyCOMToSupportRegionLimit        (rp_ref_out.tgt[rf].abs, rp_ref_out.tgt[lf].abs, rp_ref_out.tgt[com].abs.p);
            applyZMPCalcFromCOM                 (rp_ref_out.tgt[com].abs.p, rp_ref_out.tgt[zmp].abs.p);//結局STに送るZMPは最終段で計算するからこれ意味ない
            H_cur = rp_ref_out.tgt[com].abs.p(Z) - std::min((double)rp_ref_out.tgt[rf].abs.p(Z), (double)rp_ref_out.tgt[lf].abs.p(Z));
            rp_ref_out_old = rp_ref_out;

            ///// COMだけはフィルターいるか・・・
            if(is_initial_loop || wp.com_filter_cutoff_hz != com_filter_cutoff_hz_old){
                com_filter.setParameter(wp.com_filter_cutoff_hz, HZ, Q_NOOVERSHOOT);
                com_filter.reset(rp_ref_out.tgt[com].abs.p);
                com_filter_cutoff_hz_old = wp.com_filter_cutoff_hz;
            }
            rp_ref_out.tgt[com].abs.p = com_filter.passFilter(rp_ref_out.tgt[com].abs.p);

            loop++;
            is_initial_loop = false;
        }

    private:
        void convertHumanToRobot(const HumanPose& in, HumanPose& out){//結局初期指定からの移動量(=Rel)で計算をcp_decしてゆく
            //      out = in;//ダメゼッタイ
            for(int i=0, l[6]={com,rf,lf,rh,lh,head}; i<6; i++){
                out.tgt[l[i]].abs.p = wp.human_to_robot_ratio * (in.tgt[l[i]].abs.p - in.tgt[l[i]].offs.p) + out.tgt[l[i]].offs.p;
                out.tgt[l[i]].abs.R = in.tgt[l[i]].abs.R * in.tgt[l[i]].offs.R.transpose() * out.tgt[l[i]].offs.R;
            }
            for(int i=0, l[4]={rf,lf,rh,lh}; i<4; i++){
                out.tgt[l[i]].w = in.tgt[l[i]].w;
                out.tgt[l[i]].go_contact = in.tgt[l[i]].go_contact;
            }
            out.tgt[com].abs.p += wp.com_offset;
            out.tgt[zmp].abs = in.tgt[zmp].abs;//最近使わない
        }
        void setAutoCOMMode(const HumanPose& human, const HumanPose& old, HumanPose& out){
            if(wp.auto_com_mode){
                const bool rf_is_up = (human.tgt[rf].abs.p(Z) - human.tgt[rf].offs.p(Z) > wp.auto_com_foot_move_detect_height);
                const bool lf_is_up = (human.tgt[lf].abs.p(Z) - human.tgt[lf].offs.p(Z) > wp.auto_com_foot_move_detect_height);
                if      ( rf_is_up && !lf_is_up){ out.tgt[com].abs.p.head(XY) = old.tgt[lf].abs.p.head(XY); }
                else if (!rf_is_up &&  lf_is_up){ out.tgt[com].abs.p.head(XY) = old.tgt[rf].abs.p.head(XY); }
                else                            { out.tgt[com].abs.p.head(XY) = (old.tgt[rf].abs.p.head(XY) + old.tgt[lf].abs.p.head(XY)) / 2; }
            }
        }
        void lockSwingFootIfZMPOutOfSupportFoot(const HumanPose& old, HumanPose& out){
            hrp::Vector2 to_opposite_foot[LR], ref_zmp_from_foot[LR], act_zmp_from_foot[LR];
            for(int lr=0; lr<LR; lr++){
                to_opposite_foot[lr] = old.foot(OPPOSITE(lr)).abs.p.head(XY) - old.foot(lr).abs.p.head(XY);
                if(to_opposite_foot[lr].norm() < 1e-3){ std::cerr << "to_opposite_foot[lr].norm() < 1e-3 :" << to_opposite_foot[lr].transpose()<<std::endl; }
                ///// lock by ref_zmp
                ref_zmp_from_foot[lr] = act_rs.zmp.head(XY) - old.foot(lr).abs.p.head(XY);
                out.foot(OPPOSITE(lr)).go_contact |= ( ref_zmp_from_foot[lr].dot(to_opposite_foot[lr]) / to_opposite_foot[lr].norm() > wp.single_foot_zmp_safety_distance );
                ///// lock by acts_zmp
                act_zmp_from_foot[lr] = act_rs.st_zmp.head(XY) - old.foot(lr).abs.p.head(XY);
                if( act_zmp_from_foot[lr].dot(to_opposite_foot[lr]) / to_opposite_foot[lr].norm() > wp.single_foot_zmp_safety_distance ){
                    zmp_force_go_contact_count[OPPOSITE(lr)] = 1;
                }else if(zmp_force_go_contact_count[OPPOSITE(lr)] > 0 && zmp_force_go_contact_count[OPPOSITE(lr)] < HZ * wp.additional_double_support_time ){
                    zmp_force_go_contact_count[OPPOSITE(lr)]++;
                }else{
                    zmp_force_go_contact_count[OPPOSITE(lr)] = 0;
                }
                out.foot(OPPOSITE(lr)).go_contact |= (zmp_force_go_contact_count[OPPOSITE(lr)] > 0);
                ///// lock by ref cp
                out.foot(OPPOSITE(lr)).go_contact |= cp_force_go_contact[OPPOSITE(lr)];
            }
        }
        void limitEEWorkspace(const HumanPose& old, HumanPose& out){
            for(int lr=0; lr<LR; lr++){
                const PoseTGT&  support_leg = old.foot(lr);
                PoseTGT&        swing_leg   = out.foot(OPPOSITE(lr));
                hrp::Vector2 inside_vec_baserel = ( lr==R ? hrp::Vector2(0,+1) : hrp::Vector2(0,-1) );
                hrp::Vector2 sp2sw_vec = swing_leg.abs.p.head(XY) - support_leg.abs.p.head(XY);
                Eigen::Matrix2d base_rot;
                base_rot = Eigen::Rotation2Dd(baselinkpose.rpy()(y));
                hrp::Vector2 sp2sw_vec_baserel = base_rot.transpose() * sp2sw_vec;
                if( sp2sw_vec_baserel.dot(inside_vec_baserel) < wp.foot_collision_avoidance_distance){
                    sp2sw_vec_baserel += inside_vec_baserel * (wp.foot_collision_avoidance_distance - sp2sw_vec_baserel.dot(inside_vec_baserel));
                }
                sp2sw_vec = base_rot * sp2sw_vec_baserel;
                swing_leg.abs.p.head(XY) = sp2sw_vec + support_leg.abs.p.head(XY);
            }
            for(int lr=0; lr<LR; lr++){
                hrp::Vector2 horizontal_dist = out.hand(lr).abs.p.head(XY) - baselinkpose.p.head(XY);
                if(horizontal_dist.norm() < wp.base_to_hand_min_distance){
                    out.hand(lr).abs.p.head(XY) = baselinkpose.p.head(XY) + wp.base_to_hand_min_distance * horizontal_dist.normalized();
                }
            }
        }
        void setFootContactPoseByGoContact(const HumanPose& old, HumanPose& out){
            for(int lr=0; lr<LR; lr++){
                if(out.foot(lr).go_contact){
                    out.foot(lr).abs.p.head(XY) = old.foot(lr).abs.p.head(XY);
                    out.foot(lr).abs.p(Z) = out.foot(lr).offs.p(Z);
                    out.foot(lr).abs.setRpy(0, 0, old.foot(lr).abs.rpy()(y));
                    /////着地時に足を広げすぎないよう制限
                    const hrp::Vector2 support_to_swing = out.foot(lr).abs.p.head(XY) - old.foot(OPPOSITE(lr)).abs.p.head(XY);
                    if(support_to_swing.norm() > wp.max_double_support_width){
                        out.foot(lr).abs.p.head(XY) = old.foot(OPPOSITE(lr)).abs.p.head(XY) + support_to_swing.normalized() * wp.max_double_support_width;
                    }
                }else{
                    LIMIT_MIN( out.foot(lr).abs.p(Z), out.foot(lr).offs.p(Z)+wp.swing_foot_height_offset);
                }
            }
        }
        void limitFootVelNearGround(const HumanPose& old, HumanPose& out){
            for(int lr=0; lr<LR; lr++){
                const double fheight = old.foot(lr).abs.p(Z) - old.foot(lr).offs.p(Z);
                const double horizontal_max_vel = 0 + fheight * 20;
                for(int j=0; j<XY; j++){
                    LIMIT_MINMAX( out.foot(lr).abs.p(j), old.foot(lr).abs.p(j) - horizontal_max_vel * DT, old.foot(lr).abs.p(j) + horizontal_max_vel * DT);
                }
                const double vertical_max_vel = 0 + fheight * 10;
                LIMIT_MIN( out.foot(lr).abs.p(Z), old.foot(lr).abs.p(Z) - vertical_max_vel * DT);
            }
        }
        void avoidFootSinkIntoFloor(HumanPose& out){
            for(int lr=0; lr<LR; lr++){
                hrp::dmatrix act_sole_vert_abs = (out.foot(lr).abs.R * make_rect_3d(act_foot_vert_fblr[lr])).colwise() + out.foot(lr).abs.p;// translate and rotate each cols
                const double min_height = act_sole_vert_abs.row(Z).minCoeff(); // pick up min Z height
                if(min_height < out.foot(lr).offs.p(Z)){
                    out.foot(lr).abs.p(Z) += (out.foot(lr).offs.p(Z) - min_height);
                }
            }
        }
        bool applyCOMToSupportRegionLimit(const hrp::Pose3& rfin_abs, const hrp::Pose3& lfin_abs, hrp::Vector3& comin_abs){//boost::geometryがUbuntu12だとないから・・・
            const hrp::dmatrix hull_com = createSupportRegionByFootPos(rfin_abs, lfin_abs, safe_foot_vert_fblr[R], safe_foot_vert_fblr[L]);
            if(!isPointInHull2D(comin_abs.head(XY), hull_com)){
                comin_abs.head(XY) = calcNearestPointOnHull(comin_abs.head(XY), hull_com);
            }//外に出たら最近傍点に頭打ち
            return true;
        }
        bool applyCOMStateLimitByCapturePoint(const hrp::Vector3& com_in, const hrp::Pose3& rfin_abs, const hrp::Pose3& lfin_abs, hrp::Vector3& com_ans_old, hrp::Vector3& com_ans){
            com_ans = com_in;
            if (is_initial_loop)com_ans_old = com_in;
            hrp::Vector3 com_vel = (com_in - com_ans_old)/DT;
            hrp::dmatrix hull_dcp = createSupportRegionByFootPos(rfin_abs, lfin_abs, offset_fblr(safe_foot_vert_fblr[R], 0.001),    offset_fblr(safe_foot_vert_fblr[L], 0.001));
            hrp::dmatrix hull_acp = createSupportRegionByFootPos(rfin_abs, lfin_abs, offset_fblr(safe_foot_vert_fblr[R], 0.01),     offset_fblr(safe_foot_vert_fblr[L], 0.01));
            hrp::Vector2 com_vel_ans_2d = regulateCOMVelocityByCapturePointVec( com_ans_old.head(XY), com_vel.head(XY), hull_dcp, hull_acp);
            com_ans.head(XY) = com_ans_old.head(XY) + com_vel_ans_2d * DT;
            com_ans_old = com_ans;
            cp_dec.head(XY) = com_ans.head(XY) + com_vel_ans_2d * sqrt( H_cur / G );
            cp_acc.head(XY) = com_ans.head(XY) - com_vel_ans_2d * sqrt( H_cur / G );
            return true;
        }
        hrp::Vector2 regulateCOMVelocityByCapturePointVec(const hrp::Vector2& com_pos, const hrp::Vector2& com_vel, const hrp::dmatrix& hull_d, const hrp::dmatrix& hull_a){
            hrp::Vector2 com_vel_decel_ok, com_vel_accel_ok;
            com_vel_decel_ok = com_vel_accel_ok = com_vel;

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
            cp_dec_ragulated *= wp.capture_point_extend_ratio;
            if(!isPointInHull2D(cp_dec_ragulated, hull_d)){
                calcCrossPointOnHull(com_pos, cp_dec_ragulated, hull_d, cp_dec_ragulated);
                com_vel_decel_ok = (cp_dec_ragulated - com_pos) / ( sqrt( H_cur / G ) + foot_landing_delay);
            }
            ///// ついでに減速CPによる強制遊脚着地も判定
            hrp::dmatrix foot_vert3d_check_wld[LR], one_foot_hull2d[LR], one_foot_vert_3d_for_check[LR];
            for(int lr=0; lr<LR; lr++){
                one_foot_vert_3d_for_check[lr]  = make_rect_3d(offset_fblr(safe_foot_vert_fblr[lr], 0.002));// 数値誤差で内外判定ずれるので少し大きい凸包で判定
                foot_vert3d_check_wld[lr]       = (rp_ref_out.foot(lr).abs.R * one_foot_vert_3d_for_check[lr]).colwise() + rp_ref_out.foot(lr).abs.p; // rotate and translate in 3D
                one_foot_hull2d[lr]             = makeConvexHull2D( foot_vert3d_check_wld[lr].topRows(XY) ); // project into 2D and make convex hull
                cp_force_go_contact[OPPOSITE(lr)] = !isPointInHull2D(cp_dec_ragulated, one_foot_hull2d[lr]); // if CapturePoint go out of R sole region, L foot must be go contact
            }
            ///// 加速CP条件(ACP使用)
            hrp::Vector2 cp_acc_ragulated = com_pos - com_vel * sqrt( H_cur / G );
            cp_acc_ragulated *= wp.capture_point_extend_ratio;
            if(!isPointInHull2D(cp_acc_ragulated, hull_a)){
                calcCrossPointOnHull(com_pos, cp_acc_ragulated, hull_a, cp_acc_ragulated);
                com_vel_accel_ok = (-cp_acc_ragulated + com_pos ) / sqrt( H_cur / G );
            }
            ///// 加速減速条件マージ com_vel進行方向をより減速させるものを採用？
            return (com_vel_decel_ok.dot(com_vel) < com_vel_accel_ok.dot(com_vel)) ? com_vel_decel_ok : com_vel_accel_ok; //normじゃダメ？
        }
        hrp::dmatrix createSupportRegionByFootPos(const hrp::Pose3& rfin_abs, const hrp::Pose3& lfin_abs, const hrp::Vector4& rf_mgn, const hrp::Vector4& lf_mgn){
            hrp::dmatrix rf_sole_verts_abs = (rfin_abs.R * make_rect_3d(rf_mgn)).colwise() + rfin_abs.p;
            hrp::dmatrix lf_sole_verts_abs = (lfin_abs.R * make_rect_3d(lf_mgn)).colwise() + lfin_abs.p;
            hrp::dmatrix both_sole_verts_abs = (hrp::dmatrix(3, rf_sole_verts_abs.cols()+lf_sole_verts_abs.cols()) << rf_sole_verts_abs, lf_sole_verts_abs).finished();
            return makeConvexHull2D(both_sole_verts_abs.topRows(XY));
        }
        bool calcCrossPointOnHull(const hrp::Vector2& inside_start_pt, const hrp::Vector2& outside_goal_pt, const hrp::dmatrix& hull, hrp::Vector2& ans_cross_pt){
            if(hull.rows() != 2){ std::cerr << "Invalid input for calcCrossPointOnHull" << std::endl; dbgn(hull); }
            bg_linestring anchor_vec;
            anchor_vec.push_back(to_bg_point(inside_start_pt));
            anchor_vec.push_back(to_bg_point(outside_goal_pt));
            bg_multi_point bg_ans_cross_pt;
            bg::intersection(anchor_vec, to_bg_hull(hull), bg_ans_cross_pt);
            switch(bg_ans_cross_pt.size()){
                case 0: return false;
                case 1: ans_cross_pt = to_Vector2(bg_ans_cross_pt[0]); return true;
                default: std::cerr << "Number of the cross point must be 0 or 1 (current = " << bg_ans_cross_pt.size() << ") something wrong!" << std::endl; return false;
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
        hrp::dmatrix makeConvexHull2D(const hrp::dmatrix& pts_2d){ return makeConvexHull2D_Boost(pts_2d); }
        bool isPointInHull2D(const hrp::Vector2& pt, const hrp::dmatrix& hull){ return isPointInHull2D_Boost(pt, hull); }
        hrp::dmatrix makeConvexHull2D_Boost(const hrp::dmatrix& pts_2d){  // A = [p0 p1 p2 p3 ... pn p0]
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
        bool isPointInHull2D_Boost(const hrp::Vector2& pt, const hrp::dmatrix& hull){
            if(hull.rows() != 2){ std::cerr << "Invalid input for isPointInHull2D_Boost" << std::endl; dbgn(hull); }
            return bg::within(to_bg_point(pt), to_bg_hull(hull));
        }
        void applyZMPCalcFromCOM(const hrp::Vector3& comin, hrp::Vector3& zmpout){
            comacc = (comin - 2 * com_old + com_oldold)/(DT*DT);
            comacc = acc4zmp_v_filters.passFilter(comacc);
            zmpout.head(XY) = comin.head(XY)-(H_cur/G)*comacc.head(XY);
            com_oldold = com_old;
            com_old = comin;
        }
        hrp::Vector4 fbio2fblr(const hrp::Vector4& fbio_in, const int lr){// convert front_back_in_out to front_back_left_right order
            assert(lr == R || lr == L);
            if(lr == R){
                return fbio_in;
            }else{
                return (hrp::Vector4()<<fbio_in(0), fbio_in(1), -fbio_in(3), -fbio_in(2)).finished();
            }
        }
};


#endif // WBMS_CORE_H
