// -*- C++ -*-
/*!
 * @file  ZMPDistributor.h
 * @brief ZMP distribution
 * @date  $Date$
 *
 * $Id$
 */

#ifndef ZMP_DISTRIBUTOR_H
#define ZMP_DISTRIBUTOR_H

#include <hrpModel/Body.h>
#include <iostream>
#include <iterator>
#include "../ImpedanceController/JointPathEx.h"
#include "../TorqueFilter/IIRFilter.h"
#include <hrpUtil/MatrixSolvers.h>

#ifdef USE_QPOASES
#include <qpOASES.hpp>
using namespace qpOASES;
#endif

class FootSupportPolygon
{
    std::vector<std::vector<Eigen::Vector2d> > foot_vertices; // RLEG, LLEG
public:
    FootSupportPolygon () {};
    bool inside_foot (size_t idx)
    {
        return true;
    };
    Eigen::Vector2d get_foot_vertex (const size_t foot_idx, const size_t vtx_idx)
    {
        return foot_vertices[foot_idx][vtx_idx];
    };
    void set_vertices (const std::vector<std::vector<Eigen::Vector2d> >& vs) { foot_vertices = vs; };
    void get_vertices (std::vector<std::vector<Eigen::Vector2d> >& vs) { vs = foot_vertices; };
    void print_vertices (const std::string& str)
    {
        std::cerr << "[" << str << "]     ";
        for (size_t i = 0; i < foot_vertices.size(); i++) {
            std::cerr << "vs = ";
            for (size_t j = 0; j < foot_vertices[i].size(); j++) {
                std::cerr << "[" << foot_vertices[i][j](0) << " " << foot_vertices[i][j](1) << "] ";
            }
            std::cerr << ((i==foot_vertices.size()-1)?"[m]":"[m], ");
        }
        std::cerr << std::endl;;
    }
};

//

class SimpleZMPDistributor
{
    FootSupportPolygon fs, fs_mgn;
    double leg_inside_margin, leg_outside_margin, leg_front_margin, leg_rear_margin, wrench_alpha_blending;
    boost::shared_ptr<FirstOrderLowPassFilter<double> > alpha_filter;
    std::vector<Eigen::Vector2d> convex_hull;
    enum projected_point_region {LEFT, MIDDLE, RIGHT};
public:
    enum leg_type {RLEG, LLEG, RARM, LARM, BOTH, ALL};
    SimpleZMPDistributor (const double _dt) : wrench_alpha_blending (0.5)
    {
        alpha_filter = boost::shared_ptr<FirstOrderLowPassFilter<double> >(new FirstOrderLowPassFilter<double>(1e7, _dt, 0.5)); // [Hz], Almost no filter by default
    };

    inline bool is_inside_foot (const hrp::Vector3& leg_pos, const bool is_lleg, const double margin = 0.0)
    {
        if (is_lleg) return (leg_pos(1) >= (-1 * leg_inside_margin + margin)) && (leg_pos(1) <= (leg_outside_margin - margin));
        else return (leg_pos(1) <= (leg_inside_margin - margin)) && (leg_pos(1) >= (-1 * leg_outside_margin + margin));
    };
    inline bool is_front_of_foot (const hrp::Vector3& leg_pos, const double margin = 0.0)
    {
        return leg_pos(0) >= (leg_front_margin - margin);
    };
    inline bool is_rear_of_foot (const hrp::Vector3& leg_pos, const double margin = 0.0)
    {
        return leg_pos(0) <= (-1 * leg_rear_margin + margin);
    };
    inline bool is_inside_support_polygon (Eigen::Vector2d& p, const hrp::Vector3& offset = hrp::Vector3::Zero(), const bool& truncate_p = false, const std::string& str = "")
    {
      // set any inner point(ip) and binary search two vertices(convex_hull[v_a], convex_hull[v_b]) between which p is.
      p -= offset.head(2);
      size_t n_ch = convex_hull.size();
      Eigen::Vector2d ip = (convex_hull[0] + convex_hull[n_ch/3] + convex_hull[2*n_ch/3]) / 3.0;
      size_t v_a = 0, v_b = n_ch;
      while (v_a + 1 < v_b) {
        size_t v_c = (v_a + v_b) / 2;
        if (calcCrossProduct(convex_hull[v_a], convex_hull[v_c], ip) > 0) {
          if (calcCrossProduct(convex_hull[v_a], p, ip) > 0 && calcCrossProduct(convex_hull[v_c], p, ip) < 0) v_b = v_c;
          else v_a = v_c;
        } else {
          if (calcCrossProduct(convex_hull[v_a], p, ip) < 0 && calcCrossProduct(convex_hull[v_c], p, ip) > 0) v_a = v_c;
          else v_b = v_c;
        }
      }
      v_b %= n_ch;
      if (calcCrossProduct(convex_hull[v_a], convex_hull[v_b], p) >= 0) {
        p += offset.head(2);
        return true;
      } else {
        if (truncate_p) {
          if (!calc_closest_boundary_point(p, v_a, v_b)) std::cerr << "[" << str << "]   Cannot calculate closest boundary point on the convex hull" << std::endl;
        }
        p += offset.head(2);
        return false;
      }
    };
    void print_params (const std::string& str)
    {
        std::cerr << "[" << str << "]   leg_inside_margin = " << leg_inside_margin << "[m], leg_outside_margin = " << leg_outside_margin << "[m], leg_front_margin = " << leg_front_margin << "[m], leg_rear_margin = " << leg_rear_margin << "[m]" << std::endl;
        std::cerr << "[" << str << "]   wrench_alpha_blending = " << wrench_alpha_blending << ", alpha_cutoff_freq = " << alpha_filter->getCutOffFreq() << "[Hz]" << std::endl;
    }
    void print_vertices (const std::string& str)
    {
        fs.print_vertices(str);
    };
    // Compare Vector2d for sorting lexicographically
    static bool compare_eigen2d(const Eigen::Vector2d& lv, const Eigen::Vector2d& rv)
    {
      return lv(0) < rv(0) || (lv(0) == rv(0) && lv(1) < rv(1));
    };
    // Calculate 2D convex hull based on Andrew's algorithm
    // Assume that the order of vs, ee, and cs is the same
    void calc_convex_hull (const std::vector<std::vector<Eigen::Vector2d> >& vs, const std::vector<bool>& cs, const std::vector<hrp::Vector3>& ee_pos, const std::vector <hrp::Matrix33>& ee_rot)
    {
      // transform coordinate
      std::vector<Eigen::Vector2d>  tvs;
      hrp::Vector3 tpos;
      tvs.reserve(cs.size()*vs[0].size());
      for (size_t i = 0; i < cs.size(); i++) {
        if (cs[i]) {
          for (size_t j = 0; j < vs[i].size(); j++) {
            tpos = ee_pos[i] + ee_rot[i] * hrp::Vector3(vs[i][j](0), vs[i][j](1), 0.0);
            tvs.push_back(tpos.head(2));
          }
        }
      }
      // calculate convex hull
      int n_tvs = tvs.size(), n_ch = 0;
      convex_hull.resize(2*n_tvs);
      std::sort(tvs.begin(), tvs.end(), compare_eigen2d);
      for (int i = 0; i < n_tvs; convex_hull[n_ch++] = tvs[i++])
        while (n_ch >= 2 && calcCrossProduct(convex_hull[n_ch-1], tvs[i], convex_hull[n_ch-2]) <= 0) n_ch--;
      for (int i = n_tvs-2, j = n_ch+1; i >= 0; convex_hull[n_ch++] = tvs[i--])
        while (n_ch >= j && calcCrossProduct(convex_hull[n_ch-1], tvs[i], convex_hull[n_ch-2]) <= 0) n_ch--;
      convex_hull.resize(n_ch-1);
    };
    // Calculate closest boundary point on the convex hull
    bool calc_closest_boundary_point (Eigen::Vector2d& p, size_t& right_idx, size_t& left_idx) {
      size_t n_ch = convex_hull.size();
      Eigen::Vector2d cur_closest_point;
      for (size_t i; i < n_ch; i++) {
        switch(calcProjectedPoint(cur_closest_point, p, convex_hull[left_idx], convex_hull[right_idx])) {
        case MIDDLE:
          p = cur_closest_point;
          return true;
        case LEFT:
          right_idx = left_idx;
          left_idx = (left_idx + 1) % n_ch;
          if ((p - convex_hull[right_idx]).dot(convex_hull[left_idx] - convex_hull[right_idx]) <= 0) {
            p = cur_closest_point;
            return true;
          }
        case RIGHT:
          left_idx = right_idx;
          right_idx = (right_idx - 1) % n_ch;
          if ((p - convex_hull[left_idx]).dot(convex_hull[right_idx] - convex_hull[left_idx]) <= 0) {
            p = cur_closest_point;
            return true;
          }
        }
      }
      return false;
    }
    // setter
    void set_wrench_alpha_blending (const double a) { wrench_alpha_blending = a; };
    void set_leg_front_margin (const double a) { leg_front_margin = a; };
    void set_leg_rear_margin (const double a) { leg_rear_margin = a; };
    void set_leg_inside_margin (const double a) { leg_inside_margin = a; };
    void set_leg_outside_margin (const double a) { leg_outside_margin = a; };
    void set_alpha_cutoff_freq (const double a) { alpha_filter->setCutOffFreq(a); };
    void set_vertices (const std::vector<std::vector<Eigen::Vector2d> >& vs)
    {
        fs.set_vertices(vs);
        // leg_inside_margin = fs.get_foot_vertex(0, 0)(1);
        // leg_front_margin = fs.get_foot_vertex(0, 0)(0);
        // leg_rear_margin = std::fabs(fs.get_foot_vertex(0, 3)(0));
    };
    void set_vertices_from_margin_params ()
    {
        std::vector<std::vector<Eigen::Vector2d> > vec;
        // RLEG
        {
            std::vector<Eigen::Vector2d> tvec;
            tvec.push_back(Eigen::Vector2d(leg_front_margin, leg_inside_margin));
            tvec.push_back(Eigen::Vector2d(leg_front_margin, -1*leg_outside_margin));
            tvec.push_back(Eigen::Vector2d(-1*leg_rear_margin, -1*leg_outside_margin));
            tvec.push_back(Eigen::Vector2d(-1*leg_rear_margin, leg_inside_margin));
            vec.push_back(tvec);
        }
        // LLEG
        {
            std::vector<Eigen::Vector2d> tvec;
            tvec.push_back(Eigen::Vector2d(leg_front_margin, leg_outside_margin));
            tvec.push_back(Eigen::Vector2d(leg_front_margin, -1*leg_inside_margin));
            tvec.push_back(Eigen::Vector2d(-1*leg_rear_margin, -1*leg_inside_margin));
            tvec.push_back(Eigen::Vector2d(-1*leg_rear_margin, leg_outside_margin));
            vec.push_back(tvec);
        }
        // {
        //     std::vector<Eigen::Vector2d> tvec;
        //     tvec.push_back(Eigen::Vector2d(leg_front_margin, leg_inside_margin));
        //     tvec.push_back(Eigen::Vector2d(leg_front_margin, -1*leg_outside_margin));
        //     tvec.push_back(Eigen::Vector2d(-1*leg_rear_margin, -1*leg_outside_margin));
        //     tvec.push_back(Eigen::Vector2d(-1*leg_rear_margin, leg_inside_margin));
        //     vec.push_back(tvec);
        // }
        // {
        //     std::vector<Eigen::Vector2d> tvec;
        //     tvec.push_back(Eigen::Vector2d(leg_front_margin, leg_inside_margin));
        //     tvec.push_back(Eigen::Vector2d(leg_front_margin, -1*leg_outside_margin));
        //     tvec.push_back(Eigen::Vector2d(-1*leg_rear_margin, -1*leg_outside_margin));
        //     tvec.push_back(Eigen::Vector2d(-1*leg_rear_margin, leg_inside_margin));
        //     vec.push_back(tvec);
        // }
        set_vertices(vec);
    };
    // Set vertices only for cp_check_margin for now
    void set_vertices_from_margin_params (const std::vector<double>& margin)
    {
      std::vector<std::vector<Eigen::Vector2d> > vec;
      // RLEG
      {
        std::vector<Eigen::Vector2d> tvec;
        tvec.push_back(Eigen::Vector2d(leg_front_margin - margin[0], leg_inside_margin - margin[2]));
        tvec.push_back(Eigen::Vector2d(leg_front_margin - margin[0], -1*(leg_outside_margin - margin[3])));
        tvec.push_back(Eigen::Vector2d(-1*(leg_rear_margin - margin[1]), -1*(leg_outside_margin - margin[3])));
        tvec.push_back(Eigen::Vector2d(-1*(leg_rear_margin - margin[1]), leg_inside_margin - margin[2]));
        vec.push_back(tvec);
      }
      // LLEG
      {
        std::vector<Eigen::Vector2d> tvec;
        tvec.push_back(Eigen::Vector2d(leg_front_margin - margin[0], leg_inside_margin - margin[3]));
        tvec.push_back(Eigen::Vector2d(leg_front_margin - margin[0], -1*(leg_outside_margin - margin[2])));
        tvec.push_back(Eigen::Vector2d(-1*(leg_rear_margin - margin[1]), -1*(leg_outside_margin - margin[2])));
        tvec.push_back(Eigen::Vector2d(-1*(leg_rear_margin - margin[1]), leg_inside_margin - margin[3]));
        vec.push_back(tvec);
      }
      fs_mgn.set_vertices(vec);
    };
    // getter
    double get_wrench_alpha_blending () { return wrench_alpha_blending; };
    double get_leg_front_margin () { return leg_front_margin; };
    double get_leg_rear_margin () { return leg_rear_margin; };
    double get_leg_inside_margin () { return leg_inside_margin; };
    double get_leg_outside_margin () { return leg_outside_margin; };
    double get_alpha_cutoff_freq () { return alpha_filter->getCutOffFreq(); };
    void get_vertices (std::vector<std::vector<Eigen::Vector2d> >& vs) { fs.get_vertices(vs); };
    void get_margined_vertices (std::vector<std::vector<Eigen::Vector2d> >& vs) { fs_mgn.get_vertices(vs); };
    //
    double calcAlpha (const hrp::Vector3& tmprefzmp,
                      const std::vector<hrp::Vector3>& ee_pos,
                      const std::vector<hrp::Matrix33>& ee_rot,
                      const std::vector<std::string>& ee_name)
    {
        double alpha;
        size_t l_idx, r_idx;
        for (size_t i = 0; i < ee_name.size(); i++) {
            if (ee_name[i]=="rleg") r_idx = i;
            else l_idx = i;
        }
        hrp::Vector3 l_local_zmp = ee_rot[l_idx].transpose() * (tmprefzmp-ee_pos[l_idx]);
        hrp::Vector3 r_local_zmp = ee_rot[r_idx].transpose() * (tmprefzmp-ee_pos[r_idx]);

        // std::cerr << "a " << l_local_zmp(0) << " " << l_local_zmp(1) << std::endl;
        // std::cerr << "b " << r_local_zmp(0) << " " << r_local_zmp(1) << std::endl;
        if ( is_inside_foot(l_local_zmp, true) && !is_front_of_foot(l_local_zmp) && !is_rear_of_foot(l_local_zmp)) { // new_refzmp is inside lfoot
            alpha = 0.0;
        } else if ( is_inside_foot(r_local_zmp, false) && !is_front_of_foot(r_local_zmp) && !is_rear_of_foot(r_local_zmp)) { // new_refzmp is inside rfoot
            alpha = 1.0;
        } else {
            hrp::Vector3 ledge_foot;
            hrp::Vector3 redge_foot;
            // lleg
            if (is_inside_foot(l_local_zmp, true) && is_front_of_foot(l_local_zmp)) {
                ledge_foot = hrp::Vector3(leg_front_margin, l_local_zmp(1), 0.0);
            } else if (!is_inside_foot(l_local_zmp, true) && is_front_of_foot(l_local_zmp)) {
                ledge_foot = hrp::Vector3(leg_front_margin, -1 * leg_inside_margin, 0.0);
            } else if (!is_inside_foot(l_local_zmp, true) && !is_front_of_foot(l_local_zmp) && !is_rear_of_foot(l_local_zmp)) {
                ledge_foot = hrp::Vector3(l_local_zmp(0), -1 * leg_inside_margin, 0.0);
            } else if (!is_inside_foot(l_local_zmp, true) && is_rear_of_foot(l_local_zmp)) {
                ledge_foot = hrp::Vector3(-1 * leg_rear_margin, -1 * leg_inside_margin, 0.0);
            } else {
                ledge_foot = hrp::Vector3(-1 * leg_rear_margin, l_local_zmp(1), 0.0);
            }
            ledge_foot = ee_rot[l_idx] * ledge_foot + ee_pos[l_idx];
            // rleg
            if (is_inside_foot(r_local_zmp, false) && is_front_of_foot(r_local_zmp)) {
                redge_foot = hrp::Vector3(leg_front_margin, r_local_zmp(1), 0.0);
            } else if (!is_inside_foot(r_local_zmp, false) && is_front_of_foot(r_local_zmp)) {
                redge_foot = hrp::Vector3(leg_front_margin, leg_inside_margin, 0.0);
            } else if (!is_inside_foot(r_local_zmp, false) && !is_front_of_foot(r_local_zmp) && !is_rear_of_foot(r_local_zmp)) {
                redge_foot = hrp::Vector3(r_local_zmp(0), leg_inside_margin, 0.0);
            } else if (!is_inside_foot(r_local_zmp, false) && is_rear_of_foot(r_local_zmp)) {
                redge_foot = hrp::Vector3(-1 * leg_rear_margin, leg_inside_margin, 0.0);
            } else {
                redge_foot = hrp::Vector3(-1 * leg_rear_margin, r_local_zmp(1), 0.0);
            }
            redge_foot = ee_rot[r_idx] * redge_foot + ee_pos[r_idx];
            // calc alpha
            hrp::Vector3 difp = redge_foot - ledge_foot;
            alpha = difp.dot(tmprefzmp-ledge_foot)/difp.squaredNorm();
        }
        // limit
        if (alpha>1.0) alpha = 1.0;
        if (alpha<0.0) alpha = 0.0;
        return alpha;
    };

    double calcAlphaFromCOP (const hrp::Vector3& tmprefzmp,
                             const std::vector<hrp::Vector3>& cop_pos,
                             const std::vector<std::string>& ee_name)
    {
        size_t l_idx, r_idx;
        for (size_t i = 0; i < ee_name.size(); i++) {
            if (ee_name[i]=="rleg") r_idx = i;
            else l_idx = i;
        }
        hrp::Vector3 difp = (cop_pos[r_idx]-cop_pos[l_idx]);
        double alpha = difp.dot(tmprefzmp - cop_pos[l_idx])/difp.dot(difp);

        // limit
        if (alpha>1.0) alpha = 1.0;
        if (alpha<0.0) alpha = 0.0;
        return alpha;
    };

    void calcAlphaVector (std::vector<double>& alpha_vector,
                          std::vector<double>& fz_alpha_vector,
                          const std::vector<hrp::Vector3>& ee_pos,
                          const std::vector<hrp::Matrix33>& ee_rot,
                          const std::vector<std::string>& ee_name,
                          const hrp::Vector3& new_refzmp, const hrp::Vector3& ref_zmp)
    {
        double fz_alpha =  calcAlpha(ref_zmp, ee_pos, ee_rot, ee_name);
        double tmpalpha = calcAlpha(new_refzmp, ee_pos, ee_rot, ee_name), alpha;
        // LPF
        alpha = alpha_filter->passFilter(tmpalpha);
        // Blending
        fz_alpha = wrench_alpha_blending * fz_alpha + (1-wrench_alpha_blending) * alpha;
        for (size_t i = 0; i < ee_name.size(); i++) {
            alpha_vector[i]= (ee_name[i]=="rleg") ? alpha : 1-alpha;
            fz_alpha_vector[i]=(ee_name[i]=="rleg") ? fz_alpha : 1-fz_alpha;
        }
    };

    void calcAlphaVectorFromCOP (std::vector<double>& alpha_vector,
                                 std::vector<double>& fz_alpha_vector,
                                 const std::vector<hrp::Vector3>& cop_pos,
                                 const std::vector<std::string>& ee_name,
                                 const hrp::Vector3& new_refzmp, const hrp::Vector3& ref_zmp)
    {
        double fz_alpha =  calcAlphaFromCOP(ref_zmp, cop_pos, ee_name);
        double tmpalpha = calcAlphaFromCOP(new_refzmp, cop_pos, ee_name), alpha;
        // LPF
        alpha = alpha_filter->passFilter(tmpalpha);
        // Blending
        fz_alpha = wrench_alpha_blending * fz_alpha + (1-wrench_alpha_blending) * alpha;
        for (size_t i = 0; i < ee_name.size(); i++) {
            alpha_vector[i]= (ee_name[i]=="rleg") ? alpha : 1-alpha;
            fz_alpha_vector[i]=(ee_name[i]=="rleg") ? fz_alpha : 1-fz_alpha;
        }
    };

    void calcAlphaVectorFromCOPDistanceCommon (std::vector<double>& alpha_vector,
                                               const std::vector<hrp::Vector3>& cop_pos,
                                               const std::vector<std::string>& ee_name,
                                               const hrp::Vector3& ref_zmp)
    {
        std::vector<double> distances;
        double tmpdistance = 0;
        for (size_t i = 0; i < ee_name.size(); i++) {
            distances.push_back((cop_pos[i]-ref_zmp).norm());
            tmpdistance += (cop_pos[i]-ref_zmp).norm();
        }
        for (size_t i = 0; i < ee_name.size(); i++) {
            alpha_vector[i] = tmpdistance/distances[i];
        }
    };

    void calcAlphaVectorFromCOPDistance (std::vector<double>& alpha_vector,
                                         std::vector<double>& fz_alpha_vector,
                                         const std::vector<hrp::Vector3>& cop_pos,
                                         const std::vector<std::string>& ee_name,
                                         const hrp::Vector3& new_refzmp, const hrp::Vector3& ref_zmp)
    {
        calcAlphaVectorFromCOPDistanceCommon(alpha_vector, cop_pos, ee_name, new_refzmp);
        calcAlphaVectorFromCOPDistanceCommon(fz_alpha_vector, cop_pos, ee_name, ref_zmp);
        for (size_t i = 0; i < ee_name.size(); i++) {
            fz_alpha_vector[i] = wrench_alpha_blending * fz_alpha_vector[i] + (1-wrench_alpha_blending) * alpha_vector[i];
        }
    };

    void distributeZMPToForceMoments (std::vector<hrp::Vector3>& ref_foot_force, std::vector<hrp::Vector3>& ref_foot_moment,
                                      const std::vector<hrp::Vector3>& ee_pos,
                                      const std::vector<hrp::Vector3>& cop_pos,
                                      const std::vector<hrp::Matrix33>& ee_rot,
                                      const std::vector<std::string>& ee_name,
                                      const std::vector<double>& limb_gains,
                                      const std::vector<double>& toeheel_ratio,
                                      const hrp::Vector3& new_refzmp, const hrp::Vector3& ref_zmp,
                                      const double total_fz, const double dt, const bool printp = true, const std::string& print_str = "")
    {
        std::vector<double> alpha_vector(2), fz_alpha_vector(2);
        calcAlphaVector(alpha_vector, fz_alpha_vector, ee_pos, ee_rot, ee_name, new_refzmp, ref_zmp);
        ref_foot_force[0] = hrp::Vector3(0,0, fz_alpha_vector[0] * total_fz);
        ref_foot_force[1] = hrp::Vector3(0,0, fz_alpha_vector[1] * total_fz);

        hrp::Vector3 tau_0 = hrp::Vector3::Zero();
#if 0
        double gamma = fz_alpha;
        double beta = m_wrenches[1].data[2] / ( m_wrenches[0].data[2] + m_wrenches[1].data[2] );
        beta = isnan(beta) ? 0 : beta;
        double steepness = 8; // change ration from alpha to beta (steepness >= 4)
        double r = - 1/(1+exp(-6*steepness*(gamma-1+1/steepness))) + 1/(1+exp(-6*steepness*(gamma-1/steepness)));
        fz_alpha = r * beta + ( 1 - r ) * gamma;
        //       alpha = fz_alpha;

        ref_foot_force[0] = hrp::Vector3(0,0, fz_alpha * total_fz);
        ref_foot_force[1] = hrp::Vector3(0,0, (1-fz_alpha) * total_fz);
        if (DEBUGP) {
            std::cerr << "[" << m_profile.instance_name << "] slip st parameters" << std::endl;
            std::cerr << "[" << m_profile.instance_name << "]   " << ref_foot_force[1](2) << " " << ref_foot_force[0](2) << "   a:"<< steepness << " beta:" << beta << " gamma:" << gamma << " r:" << r << " fz_alpha:" << fz_alpha <<  " alpha:" << alpha << std::endl;
        }
#endif

        for (size_t i = 0; i < 2; i++) {
            tau_0 -= (cop_pos[i] - new_refzmp).cross(ref_foot_force[i]);
        }
        {
            // Foot-distribution-coords frame =>
//             hrp::Vector3 foot_dist_coords_y = (cop_pos[1] - cop_pos[0]); // e_y'
//             foot_dist_coords_y(2) = 0.0;
//             foot_dist_coords_y.normalize();
//             hrp::Vector3 foot_dist_coords_x = hrp::Vector3(foot_dist_coords_y.cross(hrp::Vector3::UnitZ())); // e_x'
//             hrp::Matrix33 foot_dist_coords_rot;
//             foot_dist_coords_rot(0,0) = foot_dist_coords_x(0);
//             foot_dist_coords_rot(1,0) = foot_dist_coords_x(1);
//             foot_dist_coords_rot(2,0) = foot_dist_coords_x(2);
//             foot_dist_coords_rot(0,1) = foot_dist_coords_y(0);
//             foot_dist_coords_rot(1,1) = foot_dist_coords_y(1);
//             foot_dist_coords_rot(2,1) = foot_dist_coords_y(2);
//             foot_dist_coords_rot(0,2) = 0;
//             foot_dist_coords_rot(1,2) = 0;
//             foot_dist_coords_rot(2,2) = 1;
//             hrp::Vector3 tau_0_f = foot_dist_coords_rot.transpose() * tau_0; // tau_0'
//             // x
//             //         // right
//             //         if (tau_0_f(0) > 0) ref_foot_moment[0](0) = tau_0_f(0);
//             //         else ref_foot_moment[0](0) = 0;
//             //         // left
//             //         if (tau_0_f(0) > 0) ref_foot_moment[1](0) = 0;
//             //         else ref_foot_moment[1](0) = tau_0_f(0);
//             ref_foot_moment[0](0) = tau_0_f(0) * alpha;
//             ref_foot_moment[1](0) = tau_0_f(0) * (1-alpha);
//             // y
//             ref_foot_moment[0](1) = tau_0_f(1) * alpha;
//             ref_foot_moment[1](1) = tau_0_f(1) * (1-alpha);
//             ref_foot_moment[0](2) = ref_foot_moment[1](2) = 0.0;
//             // <= Foot-distribution-coords frame
//             // Convert foot-distribution-coords frame => actual world frame
//             ref_foot_moment[0] = foot_dist_coords_rot * ref_foot_moment[0];
//             ref_foot_moment[1] = foot_dist_coords_rot * ref_foot_moment[1];
            //
          ref_foot_moment[0](0) = tau_0(0) * alpha_vector[0];
          ref_foot_moment[1](0) = tau_0(0) * alpha_vector[1];
          ref_foot_moment[0](1) = tau_0(1) * alpha_vector[0];
          ref_foot_moment[1](1) = tau_0(1) * alpha_vector[1];
          ref_foot_moment[0](2) = ref_foot_moment[1](2)= 0.0;
        }
#if 0
        {
            // Foot-distribution-coords frame =>
            hrp::Vector3 foot_dist_coords_y = (cop_pos[1] - cop_pos[0]); // e_y'
            foot_dist_coords_y(2) = 0.0;
            foot_dist_coords_y.normalize();
            hrp::Vector3 foot_dist_coords_x = hrp::Vector3(foot_dist_coords_y.cross(hrp::Vector3::UnitZ())); // e_x'
            hrp::Matrix33 foot_dist_coords_rot;
            foot_dist_coords_rot(0,0) = foot_dist_coords_x(0);
            foot_dist_coords_rot(1,0) = foot_dist_coords_x(1);
            foot_dist_coords_rot(2,0) = foot_dist_coords_x(2);
            foot_dist_coords_rot(0,1) = foot_dist_coords_y(0);
            foot_dist_coords_rot(1,1) = foot_dist_coords_y(1);
            foot_dist_coords_rot(2,1) = foot_dist_coords_y(2);
            foot_dist_coords_rot(0,2) = 0;
            foot_dist_coords_rot(1,2) = 0;
            foot_dist_coords_rot(2,2) = 1;
            tau_0 = hrp::Vector3::Zero();
            //
            hrp::dvector fvec(3);
            fvec(0) = total_fz;
            fvec(1) = tau_0(0);
            fvec(2) = tau_0(1);
            hrp::dmatrix Gmat(3,6);
            Gmat(0,0) = 1.0; Gmat(0,1) = 0.0; Gmat(0,2) = 0.0;
            Gmat(0,3) = 1.0; Gmat(0,4) = 0.0; Gmat(0,5) = 0.0;
            Gmat(1,0) = (cop_pos[0](1)-new_refzmp(1));
            Gmat(1,1) = 1.0;
            Gmat(1,2) = 0.0;
            Gmat(1,3) = (cop_pos[1](1)-new_refzmp(1));
            Gmat(1,4) = 1.0;
            Gmat(1,5) = 0.0;
            Gmat(2,0) = -(cop_pos[0](0)-new_refzmp(0));
            Gmat(2,1) = 0.0;
            Gmat(2,2) = 1.0;
            Gmat(2,3) = -(cop_pos[1](0)-new_refzmp(0));
            Gmat(2,4) = 0.0;
            Gmat(2,5) = 1.0;
            hrp::dmatrix Wmat(6,6);
            for (size_t i = 0; i < 6; i++) {
                for (size_t j = 0; j < 6; j++) {
                    Wmat(i,j) = 0.0;
                }
            }
            double beta_r =0 , beta_l =0;
            double kk = 8.0;
            double alpha_r = 0.9;
            double alpha_l = 0.1;
            if (alpha > alpha_r) beta_r = std::pow((alpha/alpha_r-1.0), kk)/std::pow((1.0/alpha_r-1.0), kk);
            else beta_r = 0;
            if (alpha < alpha_l) beta_l = std::pow((alpha/alpha_l-1.0), kk);
            else beta_l = 0;
            Wmat(0,0) = alpha;
            Wmat(1,1) = beta_r;
            Wmat(2,2) = alpha;
            Wmat(3,3) = (1-alpha);
            Wmat(5,5) = (1-alpha);
            Wmat(4,4) = beta_l;
            // if (printp) {
            //     std::cerr << Wmat << std::endl;
            // }
            hrp::dmatrix Gmat_ret(6,3);
            hrp::calcSRInverse(Gmat, Gmat_ret, 0.0, Wmat);
            hrp::dvector fmvec(6);
            fmvec = Gmat_ret* fvec;
            ref_foot_force[0] = hrp::Vector3(0,0,fmvec(0));
            ref_foot_force[1] = hrp::Vector3(0,0,fmvec(3));
            ref_foot_moment[0] = hrp::Vector3(fmvec(1),fmvec(2),0);
            ref_foot_moment[1] = hrp::Vector3(fmvec(4),fmvec(5),0);
            // <= Foot-distribution-coords frame
            // Convert foot-distribution-coords frame => actual world frame
            ref_foot_moment[0] = foot_dist_coords_rot * ref_foot_moment[0];
            ref_foot_moment[1] = foot_dist_coords_rot * ref_foot_moment[1];
        }
#endif

        if (printp) {
            //std::cerr << "[" << print_str << "]   alpha = " << alpha << ", fz_alpha = " << fz_alpha << std::endl;
            std::cerr << "[" << print_str << "]   "
                      << "total_tau    = " << hrp::Vector3(tau_0).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[Nm]" << std::endl;
            std::cerr << "[" << print_str << "]   "
                      << "ref_force_R  = " << hrp::Vector3(ref_foot_force[0]).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N]" << std::endl;
            std::cerr << "[" << print_str << "]   "
                      << "ref_force_L  = " << hrp::Vector3(ref_foot_force[1]).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N]" << std::endl;
            std::cerr << "[" << print_str << "]   "
                      << "ref_moment_R = " << hrp::Vector3(ref_foot_moment[0]).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[Nm]" << std::endl;
            std::cerr << "[" << print_str << "]   "
                      << "ref_moment_L = " << hrp::Vector3(ref_foot_moment[1]).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[Nm]" << std::endl;
        }
    };

#ifdef USE_QPOASES
    void solveForceMomentQPOASES (std::vector<hrp::dvector>& fret,
                                  const size_t state_dim,
                                  const size_t ee_num,
                                  const hrp::dmatrix& Hmat,
                                  const hrp::dvector& gvec)
    {
        real_t* H = new real_t[state_dim*state_dim];
        real_t* g = new real_t[state_dim];
        real_t* lb = new real_t[state_dim];
        real_t* ub = new real_t[state_dim];
        for (size_t i = 0; i < state_dim; i++) {
            for (size_t j = 0; j < state_dim; j++) {
                H[i*state_dim+j] = Hmat(i,j);
            }
            g[i] = gvec(i);
            lb[i] = 0.0;
            ub[i] = 1e10;
        }
        QProblemB example( state_dim );
        Options options;
        //options.enableFlippingBounds = BT_FALSE;
        options.initialStatusBounds = ST_INACTIVE;
        options.numRefinementSteps = 1;
        options.enableCholeskyRefactorisation = 1;
        //options.printLevel = PL_LOW;
        options.printLevel = PL_NONE;
        example.setOptions( options );
        /* Solve first QP. */
        int nWSR = 10;
        example.init( H,g,lb,ub, nWSR,0 );
        real_t* xOpt = new real_t[state_dim];
        example.getPrimalSolution( xOpt );
        size_t state_dim_one = state_dim / ee_num;
        for (size_t fidx = 0; fidx < ee_num; fidx++) {
            for (size_t i = 0; i < state_dim_one; i++) {
                fret[fidx](i) = xOpt[i+state_dim_one*fidx];
            }
        }
        delete[] H;
        delete[] g;
        delete[] lb;
        delete[] ub;
        delete[] xOpt;
    };

    void distributeZMPToForceMomentsQP (std::vector<hrp::Vector3>& ref_foot_force, std::vector<hrp::Vector3>& ref_foot_moment,
                                        const std::vector<hrp::Vector3>& ee_pos,
                                        const std::vector<hrp::Vector3>& cop_pos,
                                        const std::vector<hrp::Matrix33>& ee_rot,
                                        const std::vector<std::string>& ee_name,
                                        const std::vector<double>& limb_gains,
                                        const std::vector<double>& toeheel_ratio,
                                        const hrp::Vector3& new_refzmp, const hrp::Vector3& ref_zmp,
                                        const double total_fz, const double dt, const bool printp = true, const std::string& print_str = "",
                                        const bool use_cop_distribution = false)
    {
        size_t ee_num = ee_name.size();
        std::vector<double> alpha_vector(ee_num), fz_alpha_vector(ee_num);
        if ( use_cop_distribution ) {
            //calcAlphaVectorFromCOP(alpha_vector, fz_alpha_vector, cop_pos, ee_name, new_refzmp, ref_zmp);
            calcAlphaVectorFromCOPDistance(alpha_vector, fz_alpha_vector, cop_pos, ee_name, new_refzmp, ref_zmp);
        } else {
            calcAlphaVector(alpha_vector, fz_alpha_vector, ee_pos, ee_rot, ee_name, new_refzmp, ref_zmp);
        }

        // QP
        double norm_weight = 1e-7;
        double cop_weight = 1e-3;
        double ref_force_weight = 0;// 1e-3;
        hrp::dvector total_fm(3);
        total_fm(0) = total_fz;
        total_fm(1) = 0;
        total_fm(2) = 0;
        size_t state_dim = 4*ee_num, state_dim_one = 4; // TODO
        //
        std::vector<hrp::dvector> ff(ee_num, hrp::dvector(state_dim_one));
        std::vector<hrp::dmatrix> mm(ee_num, hrp::dmatrix(3, state_dim_one));
        //
        hrp::dmatrix Hmat = hrp::dmatrix::Zero(state_dim,state_dim);
        hrp::dvector gvec = hrp::dvector::Zero(state_dim);
        double alpha_thre = 1e-20;
        // fz_alpha inversion for weighing matrix
        for (size_t i = 0; i < fz_alpha_vector.size(); i++) {
            fz_alpha_vector[i] *= limb_gains[i];
            fz_alpha_vector[i] = (fz_alpha_vector[i] < alpha_thre) ? 1/alpha_thre : 1/fz_alpha_vector[i];
        }
        for (size_t j = 0; j < fz_alpha_vector.size(); j++) {
            for (size_t i = 0; i < state_dim_one; i++) {
                Hmat(i+j*state_dim_one,i+j*state_dim_one) = norm_weight * fz_alpha_vector[j];
            }
        }
        hrp::dmatrix Gmat(3,state_dim);
        for (size_t i = 0; i < state_dim; i++) {
            Gmat(0,i) = 1.0;
        }
        for (size_t fidx = 0; fidx < ee_num; fidx++) {
            for (size_t i = 0; i < state_dim_one; i++) {
                hrp::Vector3 fpos = ee_rot[fidx]*hrp::Vector3(fs.get_foot_vertex(fidx,i)(0), fs.get_foot_vertex(fidx,i)(1), 0) + ee_pos[fidx];
                mm[fidx](0,i) = 1.0;
                mm[fidx](1,i) = -(fpos(1)-cop_pos[fidx](1));
                mm[fidx](2,i) = (fpos(0)-cop_pos[fidx](0));
                Gmat(1,i+state_dim_one*fidx) = -(fpos(1)-new_refzmp(1));
                Gmat(2,i+state_dim_one*fidx) = (fpos(0)-new_refzmp(0));
            }
            //std::cerr << "fpos " << fpos[0] << " " << fpos[1] << std::endl;
        }
        Hmat += Gmat.transpose() * Gmat;
        gvec += -1 * Gmat.transpose() * total_fm;
        // std::cerr << "Gmat " << std::endl;
        // std::cerr << Gmat << std::endl;
        // std::cerr << "total_fm " << std::endl;
        // std::cerr << total_fm << std::endl;
        //
        {
            hrp::dmatrix Kmat = hrp::dmatrix::Zero(ee_num,state_dim);
            hrp::dmatrix KW = hrp::dmatrix::Zero(ee_num, ee_num);
            hrp::dvector reff(ee_num);
            for (size_t j = 0; j < ee_num; j++) {
                for (size_t i = 0; i < state_dim_one; i++) {
                    Kmat(j,i+j*state_dim_one) = 1.0;
                }
                reff(j) = ref_foot_force[j](2);// total_fz/2.0;
                KW(j,j) = ref_force_weight;
            }
            Hmat += Kmat.transpose() * KW * Kmat;
            gvec += -1 * Kmat.transpose() * KW * reff;
        }
        {
            hrp::dmatrix Cmat = hrp::dmatrix::Zero(ee_num*2,state_dim);
            hrp::dmatrix CW = hrp::dmatrix::Zero(ee_num*2,ee_num*2);
            hrp::Vector3 fpos;
            for (size_t j = 0; j < ee_num; j++) {
                for (size_t i = 0; i < state_dim_one; i++) {
                    fpos = ee_rot[j]*hrp::Vector3(fs.get_foot_vertex(j,i)(0), fs.get_foot_vertex(j,i)(1), 0) + ee_pos[j];
                    Cmat(j*2,  i+j*state_dim_one) = fpos(0) - cop_pos[j](0);
                    Cmat(j*2+1,i+j*state_dim_one) = fpos(1) - cop_pos[j](1);
                }
                CW(j*2,j*2) = CW(j*2+1,j*2+1) = cop_weight;
            }
            Hmat += Cmat.transpose() * CW * Cmat;
        }
        // std::cerr << "H " << Hmat << std::endl;
        // std::cerr << "g " << gvec << std::endl;
        solveForceMomentQPOASES(ff, state_dim, ee_num, Hmat, gvec);
        hrp::dvector tmpv(3);
        for (size_t fidx = 0; fidx < ee_num; fidx++) {
            tmpv = mm[fidx] * ff[fidx];
            ref_foot_force[fidx] = hrp::Vector3(0,0,tmpv(0));
            ref_foot_moment[fidx] = -1*hrp::Vector3(tmpv(1),tmpv(2),0);
        }
        if (printp) {
            std::cerr << "[" << print_str << "] force moment distribution " << (use_cop_distribution ? "(QP COP)" : "(QP)") << std::endl;
            //std::cerr << "[" << print_str << "]   alpha = " << alpha << ", fz_alpha = " << fz_alpha << std::endl;
            // std::cerr << "[" << print_str << "]   "
            //           << "total_tau    = " << hrp::Vector3(tau_0).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[Nm]" << std::endl;
            for (size_t i = 0; i < ee_num; i++) {
                std::cerr << "[" << print_str << "]   "
                          << "ref_force  [" << ee_name[i] << "] " << hrp::Vector3(ref_foot_force[i]).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N]" << std::endl;
                std::cerr << "[" << print_str << "]   "
                          << "ref_moment [" << ee_name[i] << "] " << hrp::Vector3(ref_foot_moment[i]).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[Nm]" << std::endl;
            }
        }
    };
#else
    void distributeZMPToForceMomentsQP (std::vector<hrp::Vector3>& ref_foot_force, std::vector<hrp::Vector3>& ref_foot_moment,
                                        const std::vector<hrp::Vector3>& ee_pos,
                                        const std::vector<hrp::Vector3>& cop_pos,
                                        const std::vector<hrp::Matrix33>& ee_rot,
                                        const std::vector<std::string>& ee_name,
                                        const std::vector<double>& limb_gains,
                                        const std::vector<double>& toeheel_ratio,
                                        const hrp::Vector3& new_refzmp, const hrp::Vector3& ref_zmp,
                                        const double total_fz, const double dt, const bool printp = true, const std::string& print_str = "",
                                        const bool use_cop_distribution = false)
    {
        distributeZMPToForceMoments(ref_foot_force, ref_foot_moment,
                                    ee_pos, cop_pos, ee_rot, ee_name, limb_gains, toeheel_ratio,
                                    new_refzmp, ref_zmp,
                                    total_fz, dt, printp, print_str);
    };
#endif // USE_QPOASES

    // Solve A * x = b => x = W A^T (A W A^T)-1 b
    // => x = W^{1/2} Pinv(A W^{1/2}) b
    void calcWeightedLinearEquation(hrp::dvector& ret, const hrp::dmatrix& A, const hrp::dmatrix& W, const hrp::dvector& b)
    {
        hrp::dmatrix W2 = hrp::dmatrix::Zero(W.rows(), W.cols());
        for (size_t i = 0; i < W.rows(); i++) W2(i,i) = std::sqrt(W(i,i));
        hrp::dmatrix Aw = A*W2;
        hrp::dmatrix Aw_inv = hrp::dmatrix::Zero(A.cols(), A.rows());
        hrp::calcPseudoInverse(Aw, Aw_inv);
        ret = W2 * Aw_inv * b;
        //ret = W2 * Aw.colPivHouseholderQr().solve(b);
    };

    void distributeZMPToForceMomentsPseudoInverse (std::vector<hrp::Vector3>& ref_foot_force, std::vector<hrp::Vector3>& ref_foot_moment,
                                                   const std::vector<hrp::Vector3>& ee_pos,
                                                   const std::vector<hrp::Vector3>& cop_pos,
                                                   const std::vector<hrp::Matrix33>& ee_rot,
                                                   const std::vector<std::string>& ee_name,
                                                   const std::vector<double>& limb_gains,
                                                   const std::vector<double>& toeheel_ratio,
                                                   const hrp::Vector3& new_refzmp, const hrp::Vector3& ref_zmp,
                                                   const double total_fz, const double dt, const bool printp = true, const std::string& print_str = "",
                                                   const bool use_cop_distribution = true, const std::vector<bool> is_contact_list = std::vector<bool>())
    {
        size_t ee_num = ee_name.size();
        std::vector<double> alpha_vector(ee_num), fz_alpha_vector(ee_num);
        calcAlphaVectorFromCOPDistance(alpha_vector, fz_alpha_vector, cop_pos, ee_name, new_refzmp, ref_zmp);
        if (printp) {
            std::cerr << "[" << print_str << "] force moment distribution (Pinv)" << std::endl;
        }

        // ref_foot_force and ref_foot_moment should be set
        double norm_weight = 1e-7;
        double norm_moment_weight = 1e2;
        size_t total_wrench_dim = 5;
        size_t state_dim = 6*ee_num;

        // Temporarily set ref_foot_force and ref_foot_moment based on reference values
        {
            size_t total_wrench_dim = 5;
            //size_t total_wrench_dim = 3;
            hrp::dmatrix Wmat = hrp::dmatrix::Identity(state_dim/2, state_dim/2);
            hrp::dmatrix Gmat = hrp::dmatrix::Zero(total_wrench_dim, state_dim/2);
            for (size_t j = 0; j < ee_num; j++) {
                if (total_wrench_dim == 3) {
                    Gmat(0,3*j+2) = 1.0;
                } else {
                    for (size_t k = 0; k < 3; k++) Gmat(k,3*j+k) = 1.0;
                }
            }
            for (size_t i = 0; i < total_wrench_dim; i++) {
                for (size_t j = 0; j < ee_num; j++) {
                    if ( i == total_wrench_dim-2 ) { // Nx
                        Gmat(i,3*j+1) = -(cop_pos[j](2) - ref_zmp(2));
                        Gmat(i,3*j+2) = (cop_pos[j](1) - ref_zmp(1));
                    } else if ( i == total_wrench_dim-1 ) { // Ny
                        Gmat(i,3*j) = (cop_pos[j](2) - ref_zmp(2));
                        Gmat(i,3*j+2) = -(cop_pos[j](0) - ref_zmp(0));
                    }
                }
            }
            for (size_t j = 0; j < ee_num; j++) {
                for (size_t i = 0; i < 3; i++) {
                    if (i != 2 && ee_num == 2)
                        Wmat(i+j*3, i+j*3) = 0;
                    else
                        Wmat(i+j*3, i+j*3) = Wmat(i+j*3, i+j*3) * limb_gains[j];
                }
            }
            if (printp) {
                std::cerr << "[" << print_str << "]   Wmat(diag) = [";
                for (size_t j = 0; j < ee_num; j++) {
                    for (size_t i = 0; i < 3; i++) {
                        std::cerr << Wmat(i+j*3, i+j*3) << " ";
                    }
                }
                std::cerr << "]" << std::endl;
            }
            hrp::dvector ret(state_dim/2);
            hrp::dvector total_wrench = hrp::dvector::Zero(total_wrench_dim);
            total_wrench(total_wrench_dim-3) = total_fz;
            calcWeightedLinearEquation(ret, Gmat, Wmat, total_wrench);
            for (size_t fidx = 0; fidx < ee_num; fidx++) {
                ref_foot_force[fidx] = hrp::Vector3(ret(3*fidx), ret(3*fidx+1), ret(3*fidx+2));
                ref_foot_moment[fidx] = hrp::Vector3::Zero();
            }
            // std::cerr << "GmatRef" << std::endl;
            // std::cerr << Gmat << std::endl;
            // std::cerr << "WmatRef" << std::endl;
            // std::cerr << Wmat << std::endl;
            // std::cerr << "ret" << std::endl;
            // std::cerr << ret << std::endl;
        }
        if (printp) {
            for (size_t i = 0; i < ee_num; i++) {
                std::cerr << "[" << print_str << "]   "
                          << "ref_force  [" << ee_name[i] << "] " << hrp::Vector3(ref_foot_force[i]).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N], "
                          << "ref_moment [" << ee_name[i] << "] " << hrp::Vector3(ref_foot_moment[i]).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[Nm]" << std::endl;
            }
        }

        // Calculate force/moment distribution matrix and vector
        //   We assume F = G f, calculate F and G. f is absolute here.
        //   1. Calculate F (total_wrench)
        hrp::dvector total_wrench = hrp::dvector::Zero(total_wrench_dim);
        hrp::Vector3 ref_total_force = hrp::Vector3::Zero();
        for (size_t fidx = 0; fidx < ee_num; fidx++) {
            double tmp_tau_x = -(cop_pos[fidx](2)-ref_zmp(2)) * ref_foot_force[fidx](1)
                + (cop_pos[fidx](1)-ref_zmp(1)) * ref_foot_force[fidx](2)
                + ref_foot_moment[fidx](0);
            total_wrench(3) -= tmp_tau_x;
            double tmp_tau_y = (cop_pos[fidx](2)-ref_zmp(2)) * ref_foot_force[fidx](0)
                - (cop_pos[fidx](0)-ref_zmp(0)) * ref_foot_force[fidx](2)
                + ref_foot_moment[fidx](1);
            total_wrench(4) -= tmp_tau_y;
            ref_total_force += ref_foot_force[fidx];
        }
        total_wrench(3) -= -(ref_zmp(2) - new_refzmp(2)) * ref_total_force(1) + (ref_zmp(1) - new_refzmp(1)) * ref_total_force(2);
        total_wrench(4) -= (ref_zmp(2) - new_refzmp(2)) * ref_total_force(0) - (ref_zmp(0) - new_refzmp(0)) * ref_total_force(2);
        //   2. Calculate G (Gmat)
        hrp::dmatrix Gmat = hrp::dmatrix::Zero(total_wrench_dim, state_dim);
        for (size_t j = 0; j < ee_num; j++) {
            for (size_t k = 0; k < total_wrench_dim; k++) Gmat(k,6*j+k) = 1.0;
        }
        for (size_t i = 0; i < total_wrench_dim; i++) {
            for (size_t j = 0; j < ee_num; j++) {
                if ( i == 3 ) { // Nx
                    Gmat(i,6*j+1) = -(cop_pos[j](2) - new_refzmp(2));
                    Gmat(i,6*j+2) = (cop_pos[j](1) - new_refzmp(1));
                } else if ( i == 4) { // Ny
                    Gmat(i,6*j) = (cop_pos[j](2) - new_refzmp(2));
                    Gmat(i,6*j+2) = -(cop_pos[j](0) - new_refzmp(0));
                }
            }
        }
        // Calc rotation matrix to introduce local frame
        //   G = [tmpsubG_0, tmpsubG_1, ...]
        //   R = diag[tmpR_0,    tmpR_1,    ...]
        //   f = R f_l
        //   f : absolute, f_l : local
        //   G f = G R f_l
        //   G R = [tmpsubG_0 tmpR_0, tmpsubG_1 tmpR_1, ...] -> inserted to Gmat
        hrp::dmatrix tmpsubG = hrp::dmatrix::Zero(total_wrench_dim, 6);
        hrp::dmatrix tmpR = hrp::dmatrix::Zero(6,6);
        for (size_t ei = 0; ei < ee_num; ei++) {
            for (size_t i = 0; i < total_wrench_dim; i++) {
                for (size_t j = 0; j < 6; j++) {
                    tmpsubG(i,j) = Gmat(i,6*ei+j);
                }
            }
            for (size_t i = 0; i < 3; i++) {
                for (size_t j = 0; j < 3; j++) {
                    tmpR(i,j) = tmpR(i+3,j+3) = ee_rot[ei](i,j);
                }
            }
            tmpsubG = tmpsubG * tmpR;
            for (size_t i = 0; i < total_wrench_dim; i++) {
                for (size_t j = 0; j < 6; j++) {
                    Gmat(i,6*ei+j) = tmpsubG(i,j);
                }
            }
        }

        // Calc weighting matrix
        hrp::dmatrix Wmat = hrp::dmatrix::Zero(state_dim, state_dim);
        for (size_t j = 0; j < ee_num; j++) {
            for (size_t i = 0; i < 3; i++) {
                Wmat(i+j*6, i+j*6) = fz_alpha_vector[j] * limb_gains[j];
                Wmat(i+j*6+3, i+j*6+3) = (1.0/norm_moment_weight) * fz_alpha_vector[j] * limb_gains[j];
                // Set local Y moment
                //   If toeheel_ratio is 0, toe and heel contact and local Y moment should be 0.
                if (i == 1) {
                    Wmat(i+j*6+3, i+j*6+3) = toeheel_ratio[j] * Wmat(i+j*6+3, i+j*6+3);
                }
                // In actual swing phase, X/Y momoment should be 0.
                if (!is_contact_list.empty()) {
                  if (!is_contact_list[j]) Wmat(i+j*6+3, i+j*6+3) = 0;
                }
            }
        }
        if (printp) {
            std::cerr << "[" << print_str << "]   newWmat(diag) = [";
            for (size_t j = 0; j < ee_num; j++) {
                for (size_t i = 0; i < 6; i++) {
                    std::cerr << Wmat(i+j*6, i+j*6) << " ";
                }
            }
            std::cerr << "]" << std::endl;
        }
        // std::cerr << "total_wrench" << std::endl;
        // std::cerr << total_wrench << std::endl;
        // std::cerr << "Gmat" << std::endl;
        // std::cerr << Gmat << std::endl;
        // std::cerr << "Wmat" << std::endl;
        // std::cerr << Wmat << std::endl;

        // Solve
        hrp::dvector ret(state_dim);
        calcWeightedLinearEquation(ret, Gmat, Wmat, total_wrench);

        // Extract force and moment with converting local frame -> absolute frame (ret is local frame)
        for (size_t fidx = 0; fidx < ee_num; fidx++) {
            ref_foot_force[fidx] += ee_rot[fidx] * hrp::Vector3(ret(6*fidx), ret(6*fidx+1), ret(6*fidx+2));
            ref_foot_moment[fidx] += ee_rot[fidx] * hrp::Vector3(ret(6*fidx+3), ret(6*fidx+4), ret(6*fidx+5));
        }
        if (printp) {
            for (size_t i = 0; i < ee_num; i++) {
                std::cerr << "[" << print_str << "]   "
                          << "new_ref_force  [" << ee_name[i] << "] " << hrp::Vector3(ref_foot_force[i]).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N], "
                          << "new_ref_moment [" << ee_name[i] << "] " << hrp::Vector3(ref_foot_moment[i]).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[Nm]" << std::endl;
            }
        }
    };

    void distributeZMPToForceMomentsPseudoInverse2 (std::vector<hrp::Vector3>& ref_foot_force, std::vector<hrp::Vector3>& ref_foot_moment,
                                                   const std::vector<hrp::Vector3>& ee_pos,
                                                   const std::vector<hrp::Vector3>& cop_pos,
                                                   const std::vector<hrp::Matrix33>& ee_rot,
                                                   const std::vector<std::string>& ee_name,
                                                   const std::vector<double>& limb_gains,
                                                   const std::vector<double>& toeheel_ratio,
                                                   const hrp::Vector3& new_refzmp, const hrp::Vector3& ref_zmp,
                                                   const hrp::Vector3& total_force, const hrp::Vector3& total_moment,
                                                   const std::vector<hrp::dvector6>& ee_forcemoment_distribution_weight,
                                                   const double total_fz, const double dt, const bool printp = true, const std::string& print_str = "")
    {
#define FORCE_MOMENT_DIFF_CONTROL

        size_t ee_num = ee_name.size();
        std::vector<double> alpha_vector(ee_num), fz_alpha_vector(ee_num);
        calcAlphaVectorFromCOPDistance(alpha_vector, fz_alpha_vector, cop_pos, ee_name, new_refzmp, ref_zmp);
        if (printp) {
            std::cerr << "[" << print_str << "] force moment distribution (Pinv2) ";
#ifdef FORCE_MOMENT_DIFF_CONTROL
            std::cerr << "(FORCE_MOMENT_DIFF_CONTROL)" << std::endl;
#else
            std::cerr << "(NOT FORCE_MOMENT_DIFF_CONTROL)" << std::endl;
#endif
        }
        // ref_foot_force and ref_foot_moment should be set
        size_t state_dim = 6*ee_num;
        if (printp) {
            std::cerr << "[" << print_str << "]   fz_alpha_vector [";
            for (size_t j = 0; j < ee_num; j++) {
                std::cerr << fz_alpha_vector[j] << " ";
            }
            std::cerr << std::endl;
        }

        hrp::dvector total_wrench = hrp::dvector::Zero(6);
#ifndef FORCE_MOMENT_DIFF_CONTROL
        total_wrench(0) = total_force(0);
        total_wrench(1) = total_force(1);
        total_wrench(2) = total_force(2);
#endif
        total_wrench(3) = total_moment(0);
        total_wrench(4) = total_moment(1);
        total_wrench(5) = total_moment(2);
#ifdef FORCE_MOMENT_DIFF_CONTROL
        for (size_t fidx = 0; fidx < ee_num; fidx++) {
            double tmp_tau_x = -(cop_pos[fidx](2)-new_refzmp(2)) * ref_foot_force[fidx](1) + (cop_pos[fidx](1)-new_refzmp(1)) * ref_foot_force[fidx](2);
            total_wrench(3) -= tmp_tau_x;
            double tmp_tau_y = (cop_pos[fidx](2)-new_refzmp(2)) * ref_foot_force[fidx](0) - (cop_pos[fidx](0)-new_refzmp(0)) * ref_foot_force[fidx](2);
            total_wrench(4) -= tmp_tau_y;
        }
#endif

        hrp::dmatrix Wmat = hrp::dmatrix::Zero(state_dim, state_dim);
        hrp::dmatrix Gmat = hrp::dmatrix::Zero(6, state_dim);
        for (size_t j = 0; j < ee_num; j++) {
            for (size_t k = 0; k < 6; k++) Gmat(k,6*j+k) = 1.0;
        }
        for (size_t i = 0; i < 6; i++) {
            for (size_t j = 0; j < ee_num; j++) {
                if ( i == 3 ) { // Nx
                    Gmat(i,6*j+1) = -(cop_pos[j](2) - new_refzmp(2));
                    Gmat(i,6*j+2) = (cop_pos[j](1) - new_refzmp(1));
                } else if ( i == 4 ) { // Ny
                    Gmat(i,6*j) = (cop_pos[j](2) - new_refzmp(2));
                    Gmat(i,6*j+2) = -(cop_pos[j](0) - new_refzmp(0));
                }
            }
        }
        for (size_t j = 0; j < ee_num; j++) {
            for (size_t i = 0; i < 3; i++) {
                Wmat(i+j*6, i+j*6) = ee_forcemoment_distribution_weight[j][i] * fz_alpha_vector[j] * limb_gains[j];
                //double norm_moment_weight = 1e2;
                //Wmat(i+j*6+3, i+j*6+3) = ee_forcemoment_distribution_weight[j][i+3] * (1.0/norm_moment_weight) * fz_alpha_vector[j] * limb_gains[j];
                Wmat(i+j*6+3, i+j*6+3) = ee_forcemoment_distribution_weight[j][i+3] * fz_alpha_vector[j] * limb_gains[j];
            }
        }
        if (printp) {
            std::cerr << "[" << print_str << "]   newWmat(diag) = [";
            for (size_t j = 0; j < ee_num; j++) {
                for (size_t i = 0; i < 6; i++) {
                    std::cerr << Wmat(i+j*6, i+j*6) << " ";
                }
            }
            std::cerr << std::endl;
            // std::cerr << "[" << print_str << "]   Gmat";
            // std::cerr << Gmat << std::endl;
            // std::cerr << "Wmat" << std::endl;
            // std::cerr << Wmat << std::endl;
        }

        hrp::dvector ret(state_dim);
        // Consider 6DOF total wrench (Fx, Fy, Fz, Mx, My, Mz)
        hrp::dmatrix selection_matrix = hrp::dmatrix::Identity(6,6);
        // Consdier 3DOF total wrench (Fz, Mx, My)
//         hrp::dmatrix selection_matrix = hrp::dmatrix::Zero(3,6);
//         selection_matrix(0,2) = 1.0;
//         selection_matrix(1,3) = 1.0;
//         selection_matrix(2,4) = 1.0;
        {
            hrp::dvector selected_total_wrench = hrp::dvector::Zero(selection_matrix.rows());
            hrp::dmatrix selected_Gmat = hrp::dmatrix::Zero(selection_matrix.rows(), Gmat.cols());
            selected_total_wrench = selection_matrix * total_wrench;
            selected_Gmat = selection_matrix * Gmat;
            calcWeightedLinearEquation(ret, selected_Gmat, Wmat, selected_total_wrench);
        }

        if (printp) {
            hrp::dvector tmpretv(total_wrench.size());
            tmpretv = Gmat * ret - total_wrench;
            std::cerr << "[" << print_str << "]   "
                      << "test_diff " << tmpretv.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N][Nm]" << std::endl;
            std::cerr << "[" << print_str << "]   "
                      << "total_wrench " << total_wrench.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N][Nm]" << std::endl;
            for (size_t i = 0; i < ee_num; i++) {
                std::cerr << "[" << print_str << "]   "
                          << "ref_force  [" << ee_name[i] << "] " << hrp::Vector3(ref_foot_force[i]).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N], "
                          << "ref_moment [" << ee_name[i] << "] " << hrp::Vector3(ref_foot_moment[i]).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[Nm]" << std::endl;
            }
            for (size_t i = 0; i < ee_num; i++) {
#ifdef FORCE_MOMENT_DIFF_CONTROL
                std::cerr << "[" << print_str << "]   "
                          << "dif_ref_force  [" << ee_name[i] << "] " << hrp::Vector3(hrp::Vector3(ret(6*i), ret(6*i+1), ret(6*i+2))).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N], "
                          << "dif_ref_moment [" << ee_name[i] << "] " << hrp::Vector3(hrp::Vector3(ret(6*i+3), ret(6*i+4), ret(6*i+5))).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[Nm]" << std::endl;
#else
                std::cerr << "[" << print_str << "]   "
                          << "dif_ref_force  [" << ee_name[i] << "] " << hrp::Vector3(hrp::Vector3(ret(6*i), ret(6*i+1), ret(6*i+2))-ref_foot_force[i]).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N], "
                          << "dif_ref_moment [" << ee_name[i] << "] " << hrp::Vector3(hrp::Vector3(ret(6*i+3), ret(6*i+4), ret(6*i+5))-ref_foot_moment[i]).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[Nm]" << std::endl;
#endif
            }
        }
        for (size_t fidx = 0; fidx < ee_num; fidx++) {
#ifdef FORCE_MOMENT_DIFF_CONTROL
            ref_foot_force[fidx] += hrp::Vector3(ret(6*fidx), ret(6*fidx+1), ret(6*fidx+2));
            ref_foot_moment[fidx] += hrp::Vector3(ret(6*fidx+3), ret(6*fidx+4), ret(6*fidx+5));
#else
            ref_foot_force[fidx] = hrp::Vector3(ret(6*fidx), ret(6*fidx+1), ret(6*fidx+2));
            ref_foot_moment[fidx] = hrp::Vector3(ret(6*fidx+3), ret(6*fidx+4), ret(6*fidx+5));
#endif
        }
        if (printp){
            for (size_t i = 0; i < ee_num; i++) {
                std::cerr << "[" << print_str << "]   "
                          << "new_ref_force  [" << ee_name[i] << "] " << hrp::Vector3(ref_foot_force[i]).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N], "
                          << "new_ref_moment [" << ee_name[i] << "] " << hrp::Vector3(ref_foot_moment[i]).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[Nm]" << std::endl;
            }
        }
    };

  double calcCrossProduct(const Eigen::Vector2d& a, const Eigen::Vector2d& b, const Eigen::Vector2d& o)
  {
    return (a(0) - o(0)) * (b(1) - o(1)) - (a(1) - o(1)) * (b(0) - o(0));
  };

  projected_point_region calcProjectedPoint(Eigen::Vector2d& ret, const Eigen::Vector2d& target, const Eigen::Vector2d& a, const Eigen::Vector2d& b){
    Eigen::Vector2d v1 = target - b;
    Eigen::Vector2d v2 = a - b;
    double v2_norm = v2.norm();
    if ( v2_norm == 0 ) {
        ret = a;
        return LEFT;
    } else {
        double ratio = v1.dot(v2) / (v2_norm * v2_norm);
        if (ratio < 0){
            ret = b;
            return RIGHT;
        } else if (ratio > 1){
            ret = a;
            return LEFT;
        } else {
            ret = b + ratio * v2;
            return MIDDLE;
        }
    }
  };
};

#endif // ZMP_DISTRIBUTOR_H
