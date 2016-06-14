/* -*- coding:utf-8-unix mode:c++ -*- */
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
    struct ConstraintValue {
        hrp::JointPathPtr joint_path;
        size_t first_id;
        size_t num_joints;
        hrp::dmatrix jacobian_inv;
        hrp::dmatrix inertia_mat;
        hrp::dmatrix r;
        hrp::dvector dq;
    };

    class RMController
    {
    private:
        hrp::dmatrix s_;
        std::map<std::string, ConstraintValue> constraints_;
        std::vector<size_t> free_id_; // initialize to 0 ~ numjoints-1
    public:
        void setSelectionMatrix(const hrp::dvector6 Svec);
        bool addConstraintLimb(const hrp::BodyPtr m_robot, const std::string &name);
        bool addConstraintLink(const hrp::BodyPtr m_robot, const std::string &name);
        bool removeConstraintLimb(const std::string &name);
        bool removeConstraintLink(const std::string &name);
        bool removeConstraintLink(const std::string &name, std::map<std::string, hrp::dvector6> &xi_ref);
        void calcConstraintMatrix(const hrp::Vector3 root_p, const hrp::dmatrix Jpl);
        void rmControl(hrp::BodyPtr &m_robot, const hrp::Vector3 Pref, const hrp::Vector3 Lref,
                       const std::map<std::string, hrp::dvector6> xi_ref, hrp::Vector3 &ref_basePos,
                       hrp::Matrix33 &ref_baseRot, const double dt);
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
