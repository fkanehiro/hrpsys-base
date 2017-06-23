/* -*- coding:utf-8-unix; mode:c++; -*- */
#include "ResolvedMomentumControl.h"

namespace rats
{
    void RMController::setSelectionMatrix(const hrp::dvector6 &Svec)
    {
        const size_t num_selects = (Svec.array() != 0).count();
        s_ = hrp::dmatrix::Zero(num_selects, 6);
        size_t i = 0;
        for (size_t j = 0; j < 6; ++j) {
            if (Svec(j) != 0) {
                s_(i, j) = Svec(j);
                ++i;
            }
        }
        std::cerr << "[RMC] Updated selection matrix by " << Svec.transpose() << std::endl;
    }

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

        ConstraintValue cs;
        cs.joint_path = hrp::JointPathPtr(new hrp::JointPath(m_robot->rootLink(), link));
        cs.num_joints = cs.joint_path->numJoints();
        cs.num_shared = 0;

        for (hrp::Link* added_limb = m_robot->link(name); !added_limb->isRoot(); added_limb = added_limb->parent) {
            // Strage shared joint id
            JointShared tmp_shared = unique;
            bool is_shared = false;
            for (std::map<std::string, ConstraintValue>::iterator it = constraints_.begin(); it != constraints_.end(); ++it) {
                size_t i = 0;
                while (i < (*it).second.num_joints && (*it).second.joint_id[i].first != added_limb->jointId) ++i;
                if (i != (*it).second.num_joints) {
                    if ((*it).second.joint_id[i].second == unique) ++(*it).second.num_shared;
                    (*it).second.joint_id[i].second = tmp_shared = shared;
                }
                (*it).second.jacobian.resize(6, (*it).second.num_joints - (*it).second.num_shared);
                (*it).second.jacobian_pinv.resize((*it).second.num_joints - (*it).second.num_shared, 6);
                (*it).second.jacobian_shared.resize(6, (*it).second.num_shared);
                (*it).second.jacobian_pinv_shared.resize((*it).second.num_shared, 6);
                (*it).second.MH.resize(6, (*it).second.num_joints - (*it).second.num_shared);
                (*it).second.dq.resize((*it).second.num_joints - (*it).second.num_shared);
            }

            cs.joint_id.push_back(std::make_pair(added_limb->jointId, tmp_shared));
            free_id_.erase(std::remove(free_id_.begin(), free_id_.end(), added_limb->jointId), free_id_.end());
            if (tmp_shared == shared) {
                ++cs.num_shared;
                free_id_.push_back(added_limb->jointId);
            }
            std::sort(free_id_.begin(), free_id_.end());
        }

        cs.r.resize(6, 6);
        cs.jacobian.resize(6, cs.num_joints - cs.num_shared);
        cs.jacobian_pinv.resize(cs.num_joints - cs.num_shared, 6);
        cs.jacobian_shared.resize(6, cs.num_shared);
        cs.jacobian_pinv_shared.resize(cs.num_shared, 6);
        std::sort(cs.joint_id.begin(), cs.joint_id.end());
        cs.MH.resize(6, cs.num_joints - cs.num_shared);
        cs.dq.resize(cs.num_joints - cs.num_shared);

        constraints_.insert(std::map<std::string, ConstraintValue>::value_type(name, cs));
        std::cerr << "[RMC] Succeeded to add " << name << " to constraints_" << std::endl;
        return true;
    }

    bool RMController::removeConstraintLink(const hrp::BodyPtr m_robot, const std::string &name)
    {
        if (constraints_.count(name) == 0) {
            std::cerr << "[RMC] " << name << " hasn't been added to constraints_" << std::endl;
            return false;
        }

        std::vector<std::string> constraint_list;
        for (std::map<std::string, ConstraintValue>::iterator it = constraints_.begin(); it != constraints_.end(); ++it) {
            if ((*it).first != name) constraint_list.push_back((*it).first);
        }
        free_id_.clear();
        constraints_.clear();
        for (size_t i = 0; i < m_robot->numJoints(); ++i) {
            free_id_.push_back(i);
        }
        for (size_t i = 0; i < constraint_list.size(); ++i) {
            addConstraintLink(m_robot, constraint_list[i]);
        }

        std::cerr << "[RMC] Succeeded to remove " << name << " from constraints_" << std::endl;
        return true;
    }

    bool RMController::removeConstraintLink(const hrp::BodyPtr m_robot, const std::string &name, std::map<std::string, hrp::dvector6> &xi_ref)
    {
        if (removeConstraintLink(m_robot, name)) {
            xi_ref.erase(name);
            return true;
        }
        return false;
    }

    void RMController::calcConstraintMatrix(const hrp::Vector3 &root_p, const hrp::dmatrix &Jpl)
    {
        for (std::map<std::string, ConstraintValue>::iterator it = constraints_.begin(); it != constraints_.end(); ++it) {
            hrp::dmatrix jointpath_jacobian = (*it).second.joint_path->Jacobian();
            hrp::dmatrix pinv;
            hrp::calcPseudoInverse(jointpath_jacobian, pinv);
            size_t u = 0, s = 0;
            for (size_t i = 0; i < (*it).second.num_joints; ++i) {
                if ((*it).second.joint_id[i].second == unique) {
                    (*it).second.jacobian.col(u) = jointpath_jacobian.col(i);
                    (*it).second.jacobian_pinv.row(u) = pinv.row(i);
                    (*it).second.MH.col(u) = Jpl.col((*it).second.joint_id[i].first);
                    ++u;
                } else {
                    (*it).second.jacobian_shared.col(s) = jointpath_jacobian.col(i);
                    (*it).second.jacobian_pinv_shared.row(s) = pinv.row(i);
                    ++s;
                }
            }
            (*it).second.MHpinv = (*it).second.MH * (*it).second.jacobian_pinv;
            (*it).second.r << hrp::Matrix33::Identity(), -hrp::hat((*it).second.joint_path->endLink()->p - root_p),
                              hrp::Matrix33::Zero(), hrp::Matrix33::Identity();
        }
    }

    void RMController::rmControl(hrp::BodyPtr &m_robot, const hrp::Vector3 &Pref, const hrp::Vector3 &Lref,
                                 const std::map<std::string, hrp::dvector6> &xi_ref, const hrp::Vector3 &ref_basePos,
                                 const hrp::Matrix33 &ref_baseRot, const hrp::dvector &dq_ref, const double dt)

    {
        const double EPS = 1e-6;
        // Step2, Step3
        hrp::dmatrix Jp;
        hrp::dmatrix Jl;
        m_robot->calcCM();
        m_robot->calcCMJacobian(NULL, Jp); // Eq.1 upper [M/m E -r]
        m_robot->calcAngularMomentumJacobian(NULL, Jl); // Eq.1 lower [H 0 I]
        Jp *= m_robot->totalMass();
        hrp::dmatrix Jpl(Jp.rows() + Jl.rows(), Jp.cols());
        Jpl << Jp, Jl;
        hrp::dvector6 refmom;
        refmom << Pref, Lref;

        calcConstraintMatrix(m_robot->rootLink()->p, Jpl);
        hrp::dmatrix MHbody = Jpl.block(0, m_robot->numJoints(), 6, 6);
        size_t free_dof = free_id_.size();
        hrp::dmatrix MHfree(6, free_dof);
        for (size_t i = 0; i < free_dof; ++i) {
            MHfree.col(i) = Jpl.col(free_id_[i]);
        }

        std::map<std::string, ConstraintValue>::iterator it;
        for (it = constraints_.begin(); it != constraints_.end(); ++it) {
            // set reference velocity of constraint joints
            for (size_t i = 0, j = 0; i < (*it).second.num_joints; ++i) {
                if ((*it).second.joint_id[i].second == unique) {
                    (*it).second.dq(j++) = dq_ref((*it).second.joint_id[i].first);
                }
            }

            MHbody -= (*it).second.MHpinv * (*it).second.r;
            refmom -= ((*it).second.MHpinv * xi_ref.at((*it).first) + ((*it).second.MH - (*it).second.MHpinv * (*it).second.jacobian) * (*it).second.dq);
            hrp::dmatrix MHshared = (*it).second.MHpinv * (*it).second.jacobian_shared;

            for (size_t i = 0, j = 0; i < (*it).second.num_joints; ++i) {
                if ((*it).second.joint_id[j].second == shared) {
                    size_t k = 0;
                    while ((*it).second.joint_id[i].first != free_id_[k]) ++k;
                    MHfree.col(k) -= MHshared.col(j++);
                }
            }
        }

        hrp::dmatrix MHbf(6, 6 + free_dof);
        MHbf << MHbody, MHfree;

        hrp::dmatrix A;
        hrp::dvector y;
        A = s_ * MHbf;
        y = s_ * refmom;

        // Step4
        hrp::dvector dq_free(free_dof);
        for (size_t i = 0; i < free_dof; ++i) {
            dq_free(i) = dq_ref(free_id_[i]); // reference joint velocity
        }

        hrp::dvector xi_b(6);
        {
            hrp::Vector3 root_v, root_omega;
            root_v = (ref_basePos - m_robot->rootLink()->p);
            hrp::Matrix33 dR = m_robot->rootLink()->R.transpose() * ref_baseRot;
            root_omega = hrp::omegaFromRot(dR);
            xi_b << root_v, root_omega; // reference
        }

        hrp::dvector val_vec(6 + free_dof);
        val_vec << xi_b, dq_free;

        hrp::dmatrix Apinv;
        hrp::calcPseudoInverse(A, Apinv);
        val_vec = Apinv * y + (hrp::dmatrix::Identity(6 + free_dof, 6 + free_dof) - Apinv * A) * val_vec;

        xi_b = val_vec.segment(0, 6);
        dq_free = val_vec.segment(6, free_dof);

        // Step5
        for (it = constraints_.begin(); it != constraints_.end(); ++it) {
            size_t i = 0;
            hrp::dvector dq_shared((*it).second.num_shared);

            for (size_t j = 0; j < (*it).second.num_joints; ++j) {
                if ((*it).second.joint_id[j].second == shared) {
                    size_t k = 0;
                    while ((*it).second.joint_id[j].first != free_id_[k]) ++k;
                    dq_shared(i++) = dq_free(k);
                }
            }

            if ((*it).second.num_shared == 0) (*it).second.dq = (*it).second.jacobian_pinv * (xi_ref.at((*it).first) - (*it).second.r * xi_b) +
                                                  (hrp::dmatrix::Identity((*it).second.num_joints, (*it).second.num_joints) - (*it).second.jacobian_pinv * (*it).second.jacobian) * (*it).second.dq;
            else (*it).second.dq = (*it).second.jacobian_pinv * (xi_ref.at((*it).first) - (*it).second.r * xi_b - (*it).second.jacobian_shared * dq_shared) +
                     (hrp::dmatrix::Identity((*it).second.num_joints - (*it).second.num_shared, (*it).second.num_joints - (*it).second.num_shared) - (*it).second.jacobian_pinv * (*it).second.jacobian) * (*it).second.dq;
        }

        // Step6
        hrp::dvector dq(m_robot->numJoints());
        // free
        for (size_t i = 0; i < free_dof; ++i) {
            dq(free_id_[i]) = dq_free(i);
        }

        // constraints
        for (it = constraints_.begin(); it != constraints_.end(); ++it) {
            size_t i = 0;
            for (size_t j = 0; j < (*it).second.num_joints; ++j) {
                if ((*it).second.joint_id[j].second == unique) dq((*it).second.joint_id[j].first) = (*it).second.dq[i++];
            }
        }

        // update robot states
        m_robot->rootLink()->p += xi_b.segment(0, 3);
        hrp::Vector3 omega = xi_b.segment(3, 3);
        if (omega.norm() > EPS) m_robot->rootLink()->R *= hrp::rodrigues(omega.normalized(), omega.norm());

        for (size_t i = 0; i < m_robot->numJoints(); ++i) {
            m_robot->joint(i)->dq = dq(i) / dt;
            m_robot->joint(i)->q += dq(i);
        }
        m_robot->calcForwardKinematics();
    }
}
