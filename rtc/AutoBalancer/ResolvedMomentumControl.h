/* -*- coding:utf-8-unix mode:c++ -*- */
#ifndef RMC_H_
#define RMC_H_
#include <cmath>
#include <iostream>
#include <vector>
#include <map>
#include <algorithm>
#include <boost/assign.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/shared_ptr.hpp>
#include <hrpUtil/MatrixSolvers.h>
#include "../ImpedanceController/JointPathEx.h"

namespace rats
{
    struct ConstraintValue {
        hrp::JointPathPtr joint_path;
        size_t first_id;
        size_t num_joints;
        hrp::dvector6 xi_ref;
        hrp::dmatrix jacobian_inv;
        hrp::dmatrix inertia_mat;
        // hrp::dmatrix r;
        Eigen::Matrix<double, 6, 6> r;
        hrp::dvector dq;
    };

    class RMController
    {
    private:
        hrp::dmatrix s_;
        std::map<std::string, ConstraintValue> constraints_;
        std::vector<size_t> free_id_; // 0 ~ numjoints-1 で初期化
    public:
        RMController(const hrp::BodyPtr m_robot, const hrp::dvector6 SVec)//, const std::string &limb)
        {
            // initSelectionVector(m_robot);
            // setSelectionMatrix(Svec);
            // addConstraintLimb(limb);
            for (size_t i = 0; i < m_robot->numJoints(); ++i) {
                free_id_.push_back(i);
            }
            const int num_selects = (SVec.array() != 0).count();
            s_.resize(num_selects, 6);
            for (size_t i = 0; i < num_selects; ++i) {
                if (SVec(i) != 0) s_.row(i) = SVec(i) * hrp::dmatrix::Identity(6,6).row(i);
            }
        };
        ~RMController() {};
        void setSelectionMatrix(const hrp::dvector6 SVec);
        void addConstraintLimb(const hrp::BodyPtr m_robot, const std::string &name);
        void addConstraintLink(const hrp::BodyPtr m_robot, const std::string &name);
        void removeConstraintLimb(const std::string &limb);
        void removeConstraintLink(const std::string &link);
        void calcConstraintMatrix(const hrp::Vector3 root_p, const hrp::dmatrix Jpl);
        void rmControl(hrp::BodyPtr &m_robot, const hrp::Vector3 Pref, const hrp::Vector3 Lref,
                       const hrp::dvector6 vref, hrp::Vector3 &ref_basePos, hrp::Matrix33 &ref_baseRot,
                       const double dt);

    };
}
#endif /*RMC_H_*/
