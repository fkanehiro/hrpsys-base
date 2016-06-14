/* -*- coding:utf-8-unix; mode:c++; -*- */
#include "ResolvedMomentumControl.h"

namespace rats
{
    void RMController::setSelectionMatrix(const hrp::dvector6 Svec)
    {
        const size_t num_selects = (Svec.array() != 0).count();
        s_.resize(num_selects, 6);
        for (size_t i = 0; i < num_selects; ++i) {
            if (Svec(i) != 0) s_.row(i) = Svec(i) * hrp::dmatrix::Identity(6,6).row(i);
        }
        std::cerr << "[RMC] Updated selection matrix" << std::endl;
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

    bool RMController::addConstraintLink(const hrp::BodyPtr m_robot, const std::string &name)
    {
        if (constraints_.count(name) == 1) {
            std::cerr << "[RMC] " << name << " has already been added to constraints_" << std::endl;
            return false;
        }

        hrp::Link* link = m_robot->link(name);
        if (!link) {
            std::cerr << "[RMC] Robot doesn't have " << name << std::endl;
            return false;
        }

        hrp::Link* root = link;
        while (!root->isRoot()) root = root->parent;
        ConstraintValue cs;
        hrp::JointPathPtr jpp = hrp::JointPathPtr(new hrp::JointPath(root, link));
        cs.joint_path = jpp;
        cs.num_joints = jpp->numJoints();
        size_t last_id = jpp->endLink()->index - 1;
        cs.first_id = last_id - (cs.num_joints - 1);
        cs.r.resize(6, 6);

        constraints_.insert(std::map<std::string, ConstraintValue>::value_type(name, cs));
        free_id_.erase(std::remove_if(free_id_.begin(), free_id_.end(),
                                      (cs.first_id <= boost::lambda::_1 && boost::lambda::_1 <= last_id)),
                       free_id_.end());
        std::cerr << "[RMC] Succeeded to add " << name << " to constraints_" << std::endl;
        return true;
    }

    // bool RMController::removeConstraintLimb(const std::string &name)
    // {

    // }

    bool RMController::removeConstraintLink(const std::string &name)
    {
        if (constraints_.count(name) == 0) {
            std::cerr << "[RMC] " << name << " hasn't been added to constraints_" << std::endl;
            return false;
        }

        std::vector<size_t> id;
        for (size_t i = constraints_[name].first_id, j = 0; j < constraints_[name].num_joints; ++i, ++j) {
            id.push_back(i);
        }

        std::vector<size_t>::iterator it = free_id_.begin();
        while (*it < *(id.begin())) ++it;
        free_id_.insert(it, id.begin(), id.end());

        constraints_.erase(name);
        std::cerr << "[RMC] Succeeded to remove " << name << " from constraints_" << std::endl;
        return true;
    }

    bool RMController::removeConstraintLink(const std::string &name, std::map<std::string, hrp::dvector6> &xi_ref)
    {
        if (removeConstraintLink(name)) {
            xi_ref.erase(name);
            return true;
        }
        return false;
    }

    void RMController::calcConstraintMatrix(const hrp::Vector3 root_p, const hrp::dmatrix Jpl)
    {
        for (std::map<std::string, ConstraintValue>::iterator it = constraints_.begin(); it != constraints_.end(); ++it) {
            (*it).second.jacobian_inv = ((*it).second.joint_path->Jacobian()).inverse(); // 6自由度以上のときにどうするか
            (*it).second.inertia_mat = Jpl.block(0, (*it).second.first_id, 6, (*it).second.num_joints) * (*it).second.jacobian_inv;
            (*it).second.r << hrp::Matrix33::Identity(), -hrp::hat(root_p - (*it).second.joint_path->endLink()->p),
                              hrp::Matrix33::Zero(), hrp::Matrix33::Identity();
        }
    }

    void RMController::rmControl(hrp::BodyPtr &m_robot, const hrp::Vector3 Pref, const hrp::Vector3 Lref,
                                 const std::map<std::string, hrp::dvector6> xi_ref, hrp::Vector3 &ref_basePos,
                                 hrp::Matrix33 &ref_baseRot, const double dt)

    {
        const double EPS = 1e-6;
        // Step2, Step3
        hrp::dmatrix Jp;
        hrp::dmatrix Jl;
        m_robot->calcCMJacobian(NULL, Jp); // Eq.1 upper [M/m E r]
        m_robot->calcAngularMomentumJacobian(NULL, Jl); // Eq.1 lower [H 0 I]
        Jp *= m_robot->totalMass();
        hrp::dmatrix Jpl(Jp.rows() + Jl.rows(), Jp.cols());
        Jpl << Jp, Jl;
        hrp::dvector6 refmom;
        refmom << Pref, Lref;

        calcConstraintMatrix(m_robot->rootLink()->p, Jpl);
        hrp::dmatrix MHb = Jpl.block(0, m_robot->numJoints(), 6, 6);
        std::map<std::string, ConstraintValue>::iterator it;
        for (it = constraints_.begin(); it != constraints_.end(); ++it) {
            MHb -= (*it).second.inertia_mat * (*it).second.r;
            refmom -= (*it).second.inertia_mat * xi_ref.at((*it).first);
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

        hrp::dvector xi_b(6); // updated
        {
            hrp::Vector3 root_v, root_w;
            root_v = (ref_basePos - m_robot->rootLink()->p) / dt;
            hrp::dmatrix dRRT = ((ref_baseRot - m_robot->rootLink()->R) / dt) * (m_robot->rootLink()->R).transpose();
            root_w(0) = (-dRRT(1, 2) + dRRT(2, 1)) / 2.0;
            root_w(1) = (-dRRT(2, 0) + dRRT(0, 2)) / 2.0;
            root_w(2) = (-dRRT(0, 1) + dRRT(1, 0)) / 2.0;

            xi_b << root_v, root_w; // reference
        }

        hrp::dvector val_vec(6 + free_dof);
        val_vec << xi_b, dq_free;

        hrp::dmatrix Ainv;
        hrp::calcPseudoInverse(A, Ainv);
        val_vec = Ainv * y + (hrp::dmatrix::Identity(6 + free_dof, 6 + free_dof) - Ainv * A) * val_vec;

        xi_b = val_vec.segment(0, 6);
        dq_free = val_vec.segment(6, free_dof);

        it = constraints_.begin();
        // std::cerr << "xi_b: " << xi_b.transpose() << std::endl;
        // std::cerr << "eq.(6) right: " << (MHbf * val_vec + (*it).second.inertia_mat * xi_ref.at((*it).first) + (*(++it)).second.inertia_mat * xi_ref.at((*it).first)).transpose() << std::endl;

        // Step5
        for (it = constraints_.begin(); it != constraints_.end(); ++it) {
            (*it).second.dq = (*it).second.jacobian_inv * (xi_ref.at((*it).first) - (*it).second.r * xi_b);
        }

        it = constraints_.begin();
        std::cerr << "ee speed(eq.3): " << ((*it).second.jacobian_inv * (*it).second.dq + (*it).second.r * xi_b).transpose() << std::endl;
        // std::cerr << "eq.13 right: " << (xi_ref.at((*it).first) - (*it).second.r * xi_b).transpose() << std::endl;

        // Step6
        hrp::dvector dq(m_robot->numJoints());
        // free
        for (size_t i = 0; i < free_dof; ++i) {
            dq(free_id_[i]) = dq_free(i);
        }

        // constraints
        for (it = constraints_.begin(); it != constraints_.end(); ++it) {
            dq.segment((*it).second.first_id, (*it).second.num_joints) = (*it).second.dq;
        }

        ref_basePos = m_robot->rootLink()->p + xi_b.segment(0, 3) * dt;
        ref_baseRot = m_robot->rootLink()->R;
        hrp::Vector3 omega = xi_b.segment(3, 3);
        if (omega.norm() > EPS) ref_baseRot *= hrp::rodrigues(omega.normalized(), omega.norm() * dt);

        for (size_t i = 0; i < m_robot->numJoints(); ++i) {
            m_robot->joint(i)->q += dq(i) * dt;
        }
    }
}
