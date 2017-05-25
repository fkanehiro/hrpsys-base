/* -*- coding:utf-8-unix; mode:c++; -*- */
#ifndef RMC_H_
#define RMC_H_
#include <iostream>
#include <vector>
#include <map>
#include <algorithm>
#include <boost/lambda/lambda.hpp>
#include <boost/shared_ptr.hpp>
#include <hrpUtil/MatrixSolvers.h>
#include <hrpModel/Body.h>
#include <hrpModel/Link.h>
#include <hrpModel/JointPath.h>

namespace rats
{
    enum JointShared {unique, shared};
    struct ConstraintValue {
        hrp::JointPathPtr joint_path;
        size_t num_joints;
        size_t num_shared;
        std::vector<std::pair<size_t, JointShared> > joint_id;
        hrp::dmatrix jacobian;
        hrp::dmatrix jacobian_pinv;
        hrp::dmatrix jacobian_shared;
        hrp::dmatrix jacobian_pinv_shared;
        hrp::dmatrix MH;
        hrp::dmatrix MHpinv;
        hrp::dmatrix r;
        hrp::dvector dq;
        // hrp::dvector dq_shared;
    };

    template<class T>
    void printVector(const std::vector<T> &v)
    {
        for (int i = 0; i < v.size(); ++i) {
            std::cout << v[i] << " ";
        }
        std::cout << std::endl;
    }

    class RMController
    {
    private:
        hrp::dmatrix s_;
        std::map<std::string, ConstraintValue> constraints_;
        std::vector<size_t> free_id_; // initialize to 0 ~ numjoints-1
    public:
        void setSelectionMatrix(const hrp::dvector6 Svec);
        hrp::dvector6 getSelectionVector() { return hrp::dvector::Ones(s_.rows()).transpose() * s_; }
        bool addConstraintLink(const hrp::BodyPtr m_robot, const std::string &name);
        bool removeConstraintLink(const hrp::BodyPtr m_robot, const std::string &name);
        bool removeConstraintLink(const hrp::BodyPtr m_robot, const std::string &name, std::map<std::string, hrp::dvector6> &xi_ref);
        void calcConstraintMatrix(const hrp::Vector3 root_p, const hrp::dmatrix Jpl);
        void rmControl(hrp::BodyPtr &m_robot, const hrp::Vector3 Pref, const hrp::Vector3 Lref,
                       const std::map<std::string, hrp::dvector6> &xi_ref, const hrp::Vector3 ref_basePos,
                       const hrp::Matrix33 &ref_baseRot, const double dt);
        RMController(const hrp::BodyPtr m_robot)
        {
            for (size_t i = 0; i < m_robot->numJoints(); ++i) {
                free_id_.push_back(i);
            }
        };
        RMController(const hrp::BodyPtr m_robot, const hrp::dvector6 Svec)
        {
            for (size_t i = 0; i < m_robot->numJoints(); ++i) {
                free_id_.push_back(i);
            }
            setSelectionMatrix(Svec);
        };
        ~RMController() {};

    };
}
#endif /*RMC_H_*/
