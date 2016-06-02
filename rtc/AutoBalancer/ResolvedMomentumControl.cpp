/* -*- coding:utf-8-unix; mode:c++; -*- */
#include "ResolvedMomentumControl.h"

namespace rats
{
    void RMController::setSelectionMatrix(const hrp::dvector6 SVec)
    {
        const size_t num_selects = (SVec.array() != 0).count();
        s_.resize(num_selects, 6);
        for (size_t i = 0; i < num_selects; ++i) {
            if (SVec(i) != 0) s_.row(i) = SVec(i) * hrp::dmatrix::Identity(6,6).row(i);
        }
    }

    // void RMController::addConstraintLimb(const hrp::BodyPtr m_robot, const std::string &limb)
    // {
    //     if (constraints_.count(limb) == 1) {
    //         std::cerr << limb << "has already been added to constraints_" << std::endl;
    //         return;
    //     }
    //     RTC::Properties& prop =  getProperties();
    //     size_t prop_num = 10;
    //     coil::vstring end_effectors_str = coil::split(prop["end_effectors"], ",");
    //     if (end_effectors_str.size() > 0) {
    //         size_t num = end_effectors_str.size() / prop_num;
    //         for (size_t i = 0; i < num; i++) {
    //             std::string ee_name, ee_target, ee_base;
    //             coil::stringTo(ee_name, end_effectors_str[i*prop_num].c_str());
    //             if (ee_name == limb) {
    //                 ConstraintValue cs;
    //                 coil::stringTo(ee_target, end_effectors_str[i*prop_num+1].c_str());
    //                 coil::stringTo(ee_base, end_effectors_str[i*prop_num+2].c_str());
    //                 hrp::JointPathPtr jpp = hrp::JointPathPtr(new hrp::JointPath(m_robot->link(ee_base), m_robot->link(ee_target)));
    //                 cs.joint_path = jpp;
    //                 cs.num_joints = jpp->numJoints();
    //                 size_t last_id = jpp->endLink()->index - 1;
    //                 cs.first_id = last_id - (cs.num_joints - 1);
    //                 constraints_.insert(map<std::string, ConstraintValue>::value_type(ee_name, cs));
    //                 // free_id_.erase(std::remove_if(free_id_.begin(), free_id_.end(),
    //                 //                               [] (size_t id) (return (cs.first_id <= id && id <= last_id))), // lambda eq
    //                 // free_id_.erase(std::remove_if(free_id_.begin(), free_id_.end(),
    //                 //                               (cs.first_id <= _1 && _1 <= last_id)),
    //                 //                vec.end());
    //                 return;
    //             }
    //         }
    //         std::cerr << "Robot doesn't have " << limb << std::endl;
    //     }
    // }

    // void RMController::addConstraintLimb(const hrp::BodyPtr m_robot, std::map<std::string, AutoBalancer::ABCIKparam> ikp, const std::string &limb)
    // {
    //     if (constraints_.count(limb) == 1) {
    //         std::cerr << limb << "has already been added to constraints_" << std::endl;
    //         return;
    //     }

    //     if (ikp.find(limb)) {
    //         ConstraintValue cs;
    //         // hrp::JointPathPtr jpp = hrp::JointPathPtr(new hrp::JointPath(m_robot->link(ee_base), m_robot->link(ee_target)));
    //         hrp::JointPathExPtr jpp = ikp[limb].manip;
    //         cs.joint_path = jpp;
    //         cs.num_joints = jpp->numJoints();
    //         size_t last_id = jpp->endLink()->index - 1;
    //         cs.first_id = last_id - (cs.num_joints - 1);
    //         constraints_.insert(map<std::string, ConstraintValue>::value_type(limb, cs));
    //         // free_id_.erase(std::remove_if(free_id_.begin(), free_id_.end(),
    //         //                               [] (size_t id) (return (cs.first_id <= id && id <= last_id))), // lambda eq
    //         free_id_.erase(std::remove_if(free_id_.begin(), free_id_.end(),
    //                                       (cs.first_id <= boost::lambda::_1 && boost::lambda::_1 <= last_id)),
    //                        free_id_.end());
    //         return;
    //     }
    //     std::cerr << "Robot doesn't have " << limb << std::endl;
    // }

    void RMController::addConstraintLink(const hrp::BodyPtr m_robot, const std::string &name)
    {
        if (constraints_.count(name) == 1) {
            std::cerr << name << " has already been added to constraints_" << std::endl;
            return;
        }

        hrp::Link* link = m_robot->link(name);
        if (!link) {
            std::cerr << "Robot doesn't have " << name << std::endl;
            return;
        }

        hrp::Link* root = link;
        while (!root->isRoot()) root = root->parent;
        ConstraintValue cs;
        hrp::JointPathPtr jpp = hrp::JointPathPtr(new hrp::JointPath(root, link));
        cs.joint_path = jpp;
        cs.num_joints = jpp->numJoints();
        size_t last_id = jpp->endLink()->index - 1;
        cs.first_id = last_id - (cs.num_joints - 1);
        constraints_.insert(std::map<std::string, ConstraintValue>::value_type(name, cs));
        free_id_.erase(std::remove_if(free_id_.begin(), free_id_.end(),
                                      (cs.first_id <= boost::lambda::_1 && boost::lambda::_1 <= last_id)),
                       free_id_.end());
        std::cerr << "Succeeded to add " << name << " to constraints_" << std::endl;
    }

    void RMController::removeConstraintLimb(const std::string &limb)
    {

    }

    void RMController::calcConstraintMatrix(const hrp::Vector3 root_p, const hrp::dmatrix Jpl)
    {
        std::map<std::string, ConstraintValue>::iterator it = constraints_.begin();
        while (it != constraints_.end()) {
            (*it).second.jacobian_inv = ((*it).second.joint_path->Jacobian()).inverse(); // 6自由度以上のときにどうするか
            (*it).second.inertia_mat = Jpl.block(0, (*it).second.first_id, 6, (*it).second.num_joints) * (*it).second.jacobian_inv;
            // ワールドで見ていないんじゃないか
            (*it).second.r << hrp::Matrix33::Identity(), -hrp::hat(root_p - (*it).second.joint_path->endLink()->p),
                hrp::Matrix33::Zero(), hrp::Matrix33::Identity();
            ++it;
        }
    }

    void RMController::rmControl(hrp::BodyPtr &m_robot, const hrp::Vector3 Pref, const hrp::Vector3 Lref,
                                 const hrp::dvector6 vref, hrp::Vector3 &ref_basePos, hrp::Matrix33 &ref_baseRot,
                                 const double dt)

    {
        // Step2, Step3
        hrp::dmatrix Jp;
        hrp::dmatrix Jl;
        m_robot->calcCMJacobian(NULL, Jp); // Eq.1 upper [M/m E r]
        m_robot->calcAngularMomentumJacobian(NULL, Jl); // Eq.1 lower [H 0 I]
        Jp *= m_robot->totalMass();
        hrp::dmatrix Jpl(Jp.rows() + Jl.rows(), Jp.cols());
        Jpl << Jp, Jl;
        hrp::dvector refmom(6);
        refmom << Pref, Lref;

        calcConstraintMatrix(m_robot->rootLink()->p, Jpl);
        hrp::dmatrix MHb = Jpl.block(0, m_robot->numJoints(), 6, 6);
        std::map<std::string, ConstraintValue>::iterator it = constraints_.begin();
        while (it != constraints_.end()) {
            MHb -= (*it).second.inertia_mat * (*it).second.r;
            // refmom -= (*it).second.inertia_mat * (*it).second.xi_ref;
            refmom -= (*it).second.inertia_mat * vref;
            ++it;
        }

        size_t free_dof = free_id_.size();
        hrp::dmatrix MHfree(6, free_dof);
        for (size_t i = 0; i < free_dof; ++i) {
            MHfree.col(i) = Jpl.col(free_id_[i]);
        }
        hrp::dmatrix MHbf(6, 6 + free_dof);
        MHbf << MHb, MHfree;

        hrp::dmatrix A;
        hrp::dvector y;
        A = s_ * MHbf;
        y = s_ * refmom;

        // Step4
        hrp::dvector dq_free;
        dq_free.resize(free_dof);
        for(size_t i = 0; i < free_dof; ++i) {
            dq_free(i) = m_robot->joint(free_id_[i])->dq; // reference
        }

        hrp::dvector xi_b(6);
        xi_b << m_robot->rootLink()->v, m_robot->rootLink()->w; // reference

        hrp::dvector val_vec(6 + free_dof);
        val_vec << xi_b, dq_free;

        hrp::dmatrix Ainv;
        hrp::calcPseudoInverse(A, Ainv);
        val_vec = Ainv * y + (hrp::dmatrix::Identity(6 + free_dof, 6 + free_dof) - Ainv * A) * val_vec;

        xi_b = val_vec.segment(0, 6);
        dq_free = val_vec.segment(6, free_dof);

        // Step5
        it = constraints_.begin();
        while (it != constraints_.end()) {
            // (*it).second.dq = (*it).second.jacobian_inv * ((*it).second.xi_ref - (*it).second.r * xi_b);
            (*it).second.dq = (*it).second.jacobian_inv * (vref - (*it).second.r * xi_b);
            ++it;
        }

        // Step6
        hrp::dvector dq(m_robot->numJoints());
        // free
        for (size_t i = 0; i < free_dof; ++i) {
            dq(free_id_[i]) = dq_free(i);
        }

        // constraints
        it = constraints_.begin();
        while (it != constraints_.end()) {
            dq.segment((*it).second.first_id, (*it).second.num_joints) = (*it).second.dq;
            ++it;
        }

        ref_basePos = m_robot->rootLink()->p + xi_b.segment(0, 3) * dt;
        hrp::Vector3 omega = xi_b.segment(3, 3);
        if (omega.norm() != 0) ref_baseRot = m_robot->rootLink()->R * hrp::rodrigues(omega.normalized(), omega.norm() * dt);

        for (size_t i = 0; i < m_robot->numJoints(); ++i) {
            m_robot->joint(i)->q += dq(i) * dt;
        }
        std::cerr << "dq: " << dq.transpose() << std::endl; // debug
    }
}
