#ifndef MYUTILS_H
#define MYUTILS_H

#include <hrpModel/Body.h>
#include <hrpModel/Link.h>
#include <hrpUtil/Eigen3d.h>
#include <hrpUtil/Eigen4d.h>
#include "../RobotHardware/defs.h"

enum { R, L, LR };
//enum {X, Y, Z}; //already exist in RobotHardware/defs.h
enum { XY = 2 };
enum { XYZ = 3 };
enum { r, p, y, rpy };
enum { fx, fy, fz, tx, ty, tz, ft_xyz };
enum { com, rf, lf, rh, lh, head, zmp, num_pose_tgt };
enum { num_ee_tgt=4 };

static const double G = 9.80665;
static const double Q_NOOVERSHOOT = 0.5;
static const double Q_BUTTERWORTH = 0.707106781;

#define dbg(var) std::cout<<#var"= "<<(var)<<std::endl
#define dbgn(var) std::cout<<#var"= "<<std::endl<<(var)<<std::endl
#define dbgv(var) std::cout<<#var"= "<<(var.transpose())<<std::endl
#define RTCOUT std::cerr << "[" << m_profile.instance_name << "] "

#define LIMIT_MIN(x,min) (x= ( x<min ? min:x ))
#define LIMIT_MAX(x,max) (x= ( x<max ? x:max ))
#define LIMIT_MINMAX(x,min,max) ((x= (x<min  ? min : x<max ? x : max)))
#define eps_eq(a, b, c)  (fabs((a)-(b)) <= c)

//inline bool eps_eq(const double& a, const double& b, const double& eps = 0.001){ return fabs((a)-(b)) <= eps;}
//inline void LIMIT_MIN(int& x, const int& min){ x = ( x<min ? min:x );}
//inline void LIMIT_MIN(double& x, const double& min){ x = ( x<min ? min:x );}
//inline void LIMIT_MAX(int& x, const int& max){ x = ( x<max ? x:max );}
//inline void LIMIT_MAX(double& x, const double& max){ x = ( x<max ? x:max );}
//inline void LIMIT_MINMAX(int& x, const int& min, const int& max){ x = (x<min  ? min : (x<max ? x : max));}
//inline void LIMIT_MINMAX(double& x, const double& min, const double& max){ x = (x<min  ? min : (x<max ? x : max));}


namespace hrp{
    class Pose3{
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            hrp::Vector3 p;
            hrp::Matrix33 R;
            Pose3(){ p.fill(0); R.setIdentity();}
            Pose3(const double& _X, const double& _Y, const double& _Z, const double& _r, const double& _p, const double& _y){ p << _X,_Y,_Z; R = hrp::rotFromRpy(_r,_p,_y); }
            Pose3(const hrp::dvector6& _xyz_rpy){ p = _xyz_rpy.head(3); R = hrp::rotFromRpy(_xyz_rpy.tail(3)); }
            Pose3(const hrp::Vector3& _xyz, const hrp::Vector3& _rpy){ p = _xyz; R = hrp::rotFromRpy(_rpy); }
            Pose3(const hrp::Vector3& _xyz, const hrp::Matrix33& _R){ p = _xyz; R = _R; }
            hrp::Vector3 rpy() const{ return hrp::rpyFromRot(R); }
            void setRpy(const double& _r, const double& _p, const double& _y) { R = rotFromRpy(_r,_p,_y); }
            void setRpy(const hrp::Vector3& _rpy) { R = rotFromRpy(_rpy); }
            hrp::dvector6 to_dvector6() const{ return (hrp::dvector6() << p, hrp::rpyFromRot(R)).finished(); }
            void reset(){ p.fill(0); R.setIdentity(); }
    };

    inline hrp::Vector3         to_Vector3      (const RTC::Point3D& in)        { return hrp::Vector3(in.x, in.y, in.z); }
    inline hrp::Vector3         to_Vector3      (const RTC::Orientation3D& in)  { return hrp::Vector3(in.r, in.p, in.y); }
    inline hrp::dvector6        to_dvector6     (const RTC::Pose3D& in)         { return (hrp::dvector6() << in.position.x, in.position.y, in.position.z, in.orientation.r, in.orientation.p, in.orientation.y).finished(); }
    inline hrp::Pose3           to_Pose3        (const RTC::Pose3D& in)         { return hrp::Pose3(in.position.x, in.position.y, in.position.z, in.orientation.r, in.orientation.p, in.orientation.y); }
    inline hrp::dvector6        to_dvector6     (const OpenHRP::Wrench& in)     { return (hrp::dvector6() << in.force.x, in.force.y, in.force.z, in.torque.x, in.torque.y, in.torque.z).finished(); }
    inline hrp::dvector         to_dvector      (const RTC::TimedDoubleSeq::_data_seq& in) { return hrp::dvector::Map(in.get_buffer(), in.length()); }

    inline RTC::Point3D         to_Point3D      (const hrp::Vector3& in)        { return (RTC::Point3D){in(X),in(Y),in(Z)}; }
    inline RTC::Orientation3D   to_Orientation3D(const hrp::Vector3& in)        { return (RTC::Orientation3D){in(X),in(Y),in(Z)}; }
    inline RTC::Pose3D          to_Pose3D       (const hrp::dvector6& in)       { return (RTC::Pose3D){in(X),in(Y),in(Z),in(r),in(p),in(y)}; }
    inline RTC::Pose3D          to_Pose3D       (const hrp::Pose3& in)          { return (RTC::Pose3D){in.p(X),in.p(Y),in.p(Z),in.rpy()(r),in.rpy()(p),in.rpy()(y)}; }
    inline OpenHRP::Wrench      to_Wrench       (const hrp::dvector6& in)       { return (OpenHRP::Wrench){in(X),in(Y),in(Z),in(r),in(p),in(y)}; }
    inline RTC::TimedDoubleSeq::_data_seq  to_DoubleSeq(const hrp::dvector& in) { RTC::TimedDoubleSeq::_data_seq out(in.size()); hrp::dvector::Map(out.get_buffer(), in.size()) = in; return out; }

    inline hrp::dvector getQAll         (const hrp::BodyPtr _robot){ hrp::dvector tmp(_robot->numJoints()); for(int i=0;i<_robot->numJoints();i++){ tmp(i) = _robot->joint(i)->q; } return tmp; }
    inline void         setQAll         (hrp::BodyPtr       _robot, const hrp::dvector& in){ assert(in.size() <= _robot->numJoints()); for(int i=0;i<_robot->numJoints();i++){ _robot->joint(i)->q = in(i); } }
    inline hrp::dvector getRobotStateVec(const hrp::BodyPtr _robot){ return (hrp::dvector(_robot->numJoints()+6) << getQAll(_robot), _robot->rootLink()->p, hrp::rpyFromRot(_robot->rootLink()->R)).finished(); }
    inline void         setRobotStateVec(hrp::BodyPtr       _robot, const hrp::dvector& _q_bpos_brpy){ assert(_q_bpos_brpy.size() == _robot->numJoints()+6); for(int i=0;i<_robot->numJoints();i++){ _robot->joint(i)->q = _q_bpos_brpy(i); }; _robot->rootLink()->p = _q_bpos_brpy.tail(6).head(3); _robot->rootLink()->R = hrp::rotFromRpy(_q_bpos_brpy.tail(6).tail(3)); }
    inline void         setRobotStateVec(hrp::BodyPtr       _robot, const hrp::dvector& _q, const hrp::Vector3& _bpos, const hrp::Vector3& _brpy){ assert(_q.size() == _robot->numJoints()); for(int i=0;i<_robot->numJoints();i++){ _robot->joint(i)->q = _q(i); }; _robot->rootLink()->p = _bpos; _robot->rootLink()->R = hrp::rotFromRpy(_brpy); }
    inline void         setRobotStateVec(hrp::BodyPtr       _robot, const hrp::dvector& _q, const hrp::Vector3& _bpos, const hrp::Matrix33& _bR){ assert(_q.size() == _robot->numJoints()); for(int i=0;i<_robot->numJoints();i++){ _robot->joint(i)->q = _q(i); }; _robot->rootLink()->p = _bpos; _robot->rootLink()->R = _bR; }

    inline hrp::Vector3 omegaFromRotEx(const hrp::Matrix33& r) {//copy from JointPathEx.cpp
        using ::std::numeric_limits;
        double alpha = (r(0,0) + r(1,1) + r(2,2) - 1.0) / 2.0;
        if(fabs(alpha - 1.0) < 1.0e-12) {   //th=0,2PI;
            return hrp::Vector3::Zero();
        } else {
            double th = acos(alpha);
            double s = sin(th);
            if (s < numeric_limits<double>::epsilon()) {   //th=PI
                return hrp::Vector3( sqrt((r(0,0)+1)*0.5)*th, sqrt((r(1,1)+1)*0.5)*th, sqrt((r(2,2)+1)*0.5)*th );
            }
            double k = -0.5 * th / s;
            return hrp::Vector3( (r(1,2) - r(2,1)) * k, (r(2,0) - r(0,2)) * k, (r(0,1) - r(1,0)) * k );
        }
    }
    inline double hrpVector2Cross(const hrp::Vector2& a, const hrp::Vector2& b){ return a(X)*b(Y)-a(Y)*b(X); }
    inline hrp::dmatrix to_SelectionMat(const hrp::dvector& in) {
        hrp::dmatrix ret = hrp::dmatrix::Zero( (in.array() > 0.0).count(), in.size() );
        if(ret.rows() != 0 && ret.cols() != 0){
            for (int row=0, col=0; col< in.size(); col++){
                if(in(col) > 0.0){
                    ret(row, col) = 1;
                    row++;
                }
            }
        }
        return ret;
    }
}


inline std::ostream& operator<<(std::ostream& os, hrp::Pose3& in){
    os << "p = " << in.p.transpose() << std::endl;
    os << "R =\n" << in.R << std::endl;
    os << "(rpy) = " << hrp::rpyFromRot(in.R).transpose() << std::endl;
    return os;
}

#endif //  MYUTILS_H
