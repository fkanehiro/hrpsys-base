/* -*- coding:utf-8-unix; mode:c++; -*- */

#include "GaitGenerator.h"
#include <numeric>

namespace rats
{
#ifndef rad2deg
#define rad2deg(rad) (rad * 180 / M_PI)
#endif
#ifndef deg2rad
#define deg2rad(deg) (deg * M_PI / 180)
#endif
  void cycloid_midpoint (hrp::Vector3& ret,
                         const double ratio, const hrp::Vector3& start,
                         const hrp::Vector3& goal, const double height,
                         const double default_top_ratio)
  {
    hrp::Vector3 u ( goal - start );
    hrp::Vector3 uz (0,0, ratio * u(2));
    u(2) = 0.0;
    double pth = 2 * M_PI * ratio, norm_u = u.norm();
    if ( !eps_eq(norm_u, 0.0,1e-3*0.01) )
      u =  u.normalized();
    /* check ratio vs 0.5 for default_top_ratio blending */
    hrp::Vector3 cycloid_point( ((0.5 > ratio) ? ( 2 * default_top_ratio * norm_u ) : ( 2 * (1 - default_top_ratio) * norm_u )) * ( pth - sin(pth) ) / (2 * M_PI) -
			   ((0.5 > ratio) ? 0.0 : (norm_u * (1 - 2 * default_top_ratio)) ), // local x
			   0, // local y
			   ( 0.5 * height * ( 1 - cos(pth) )) ); // local z
    hrp::Vector3 v(hrp::Vector3(0,0,1).cross(u));
    hrp::Matrix33 dvm;
    dvm << u(0), v(0), 0,
      u(1), v(1), 0,
      u(2), v(2), 1;
    ret = dvm * cycloid_point + start + uz;
  };
  void multi_mid_coords (coordinates& ret, const std::vector<coordinates>& cs)
  {
      if (cs.size() == 1) {
          ret = cs.front();
      } else {
          std::vector<coordinates> tmp_mid_coords;
          double ratio = (1.0 - 1.0 / cs.size());
          for (size_t i = 1; i < cs.size(); i++) {
              coordinates tmp;
              mid_coords(tmp, ratio, cs.front(), cs.at(i));
              tmp_mid_coords.push_back(tmp);
          }
          multi_mid_coords(ret, tmp_mid_coords);
      }
      return;
  };

  /* member function implementation for refzmp_generator */
  void refzmp_generator::push_refzmp_from_footstep_nodes_for_dual (const std::vector<step_node>& fns,
                                                                   const std::vector<step_node>& _support_leg_steps,
                                                                   const std::vector<step_node>& _swing_leg_steps)
  {
    hrp::Vector3 rzmp;
    std::vector<hrp::Vector3> dzl;
    hrp::Vector3 ret_zmp;
    hrp::Vector3 tmp_zero = hrp::Vector3::Zero();
    std::vector<hrp::Vector3> foot_x_axises;
    double sum_of_weight = 0.0;
    for (std::vector<step_node>::const_iterator it = _support_leg_steps.begin(); it != _support_leg_steps.end(); it++) {
        dzl.push_back((it->worldcoords.rot * default_zmp_offsets[it->l_r] + it->worldcoords.pos) * zmp_weight_map[it->l_r]);
        sum_of_weight += zmp_weight_map[it->l_r];
    }
    for (std::vector<step_node>::const_iterator it = _swing_leg_steps.begin(); it != _swing_leg_steps.end(); it++) {
        dzl.push_back((it->worldcoords.rot * default_zmp_offsets[it->l_r] + it->worldcoords.pos) * zmp_weight_map[it->l_r]);
        sum_of_weight += zmp_weight_map[it->l_r];
        foot_x_axises.push_back( hrp::Vector3(it->worldcoords.rot * hrp::Vector3::UnitX()) );
    }
    foot_x_axises_list.push_back(foot_x_axises);
    rzmp = std::accumulate(dzl.begin(), dzl.end(), tmp_zero) / sum_of_weight;
    refzmp_cur_list.push_back( rzmp );
    std::vector<leg_type> swing_leg_types;
    for (size_t i = 0; i < fns.size(); i++) {
        swing_leg_types.push_back(fns.at(i).l_r);
    }
    swing_leg_types_list.push_back( swing_leg_types );
    step_count_list.push_back(static_cast<size_t>(fns.front().step_time/dt));
    //std::cerr << "double " << (fns[fs_index].l_r==RLEG?LLEG:RLEG) << " [" << refzmp_cur_list.back()(0) << " " << refzmp_cur_list.back()(1) << " " << refzmp_cur_list.back()(2) << "]" << std::endl;
  };

  void refzmp_generator::push_refzmp_from_footstep_nodes_for_single (const std::vector<step_node>& fns, const std::vector<step_node>& _support_leg_steps)
  {
    // support leg = prev fns l_r
    // swing leg = fns l_r
    hrp::Vector3 rzmp, tmp_zero=hrp::Vector3::Zero();
    std::vector<hrp::Vector3> dzl;
    std::vector<hrp::Vector3> foot_x_axises;
    double sum_of_weight = 0.0;

    for (std::vector<step_node>::const_iterator it = _support_leg_steps.begin(); it != _support_leg_steps.end(); it++) {
        dzl.push_back((it->worldcoords.rot * default_zmp_offsets[it->l_r] + it->worldcoords.pos) * zmp_weight_map[it->l_r]);
        sum_of_weight += zmp_weight_map[it->l_r];
        foot_x_axises.push_back( hrp::Vector3(it->worldcoords.rot * hrp::Vector3::UnitX()) );
    }
    rzmp = std::accumulate(dzl.begin(), dzl.end(), tmp_zero) / sum_of_weight;
    refzmp_cur_list.push_back( rzmp );
    foot_x_axises_list.push_back(foot_x_axises);
    std::vector<leg_type> swing_leg_types;
    for (size_t i = 0; i< fns.size(); i++) {
        swing_leg_types.push_back(fns.at(i).l_r);
    }
    swing_leg_types_list.push_back( swing_leg_types );
    step_count_list.push_back(static_cast<size_t>(fns.front().step_time/dt));
    //std::cerr << "single " << fns[fs_index-1].l_r << " [" << refzmp_cur_list.back()(0) << " " << refzmp_cur_list.back()(1) << " " << refzmp_cur_list.back()(2) << "]" << std::endl;
  };

  void refzmp_generator::calc_current_refzmp (hrp::Vector3& ret, std::vector<hrp::Vector3>& swing_foot_zmp_offsets, const double default_double_support_ratio_before, const double default_double_support_ratio_after, const double default_double_support_static_ratio_before, const double default_double_support_static_ratio_after)
  {
    size_t cnt = one_step_count - refzmp_count; // current counter (0 -> one_step_count)
    size_t double_support_count_half_before = default_double_support_ratio_before * one_step_count;
    size_t double_support_count_half_after = default_double_support_ratio_after * one_step_count;
    size_t double_support_static_count_half_before = default_double_support_static_ratio_before * one_step_count;
    size_t double_support_static_count_half_after = default_double_support_static_ratio_after * one_step_count;
    for (size_t i = 0; i < swing_leg_types_list[refzmp_index].size(); i++) {
        swing_foot_zmp_offsets.push_back(default_zmp_offsets[swing_leg_types_list[refzmp_index].at(i)]);
    }
    double zmp_diff = 0.0; // difference between total swing_foot_zmp_offset and default_zmp_offset
    //if (cnt==0) std::cerr << "z " << refzmp_index << " " << refzmp_cur_list.size() << " " << fs_index << " " << (refzmp_index == refzmp_cur_list.size()-2) << " " << is_final_double_support_set << std::endl;

    // Calculate swing foot zmp offset for toe heel zmp transition
    if (use_toe_heel_transition &&
        !(is_start_double_support_phase() || is_end_double_support_phase())) { // Do not use toe heel zmp transition during start and end double support period because there is no swing foot
        if (thp_ptr->is_between_phases(cnt, SOLE0)) {
            double ratio = thp_ptr->calc_phase_ratio(cnt+1, SOLE0);
            swing_foot_zmp_offsets.front()(0) = (1-ratio)*swing_foot_zmp_offsets.front()(0) + ratio*toe_zmp_offset_x;
        } else if (thp_ptr->is_between_phases(cnt, HEEL2SOLE, SOLE2)) {
            double ratio = thp_ptr->calc_phase_ratio(cnt, HEEL2SOLE, SOLE2);
            swing_foot_zmp_offsets.front()(0) = ratio*swing_foot_zmp_offsets.front()(0) + (1-ratio)*heel_zmp_offset_x;
        } else if (thp_ptr->is_between_phases(cnt, SOLE0, SOLE2TOE)) {
            swing_foot_zmp_offsets.front()(0) = toe_zmp_offset_x;
        } else if (thp_ptr->is_between_phases(cnt, SOLE2HEEL, HEEL2SOLE)) {
            swing_foot_zmp_offsets.front()(0) = heel_zmp_offset_x;
        } else if (thp_ptr->is_between_phases(cnt, SOLE2TOE, SOLE2HEEL)) {
            double ratio = thp_ptr->calc_phase_ratio(cnt, SOLE2TOE, SOLE2HEEL);
            swing_foot_zmp_offsets.front()(0) = ratio * heel_zmp_offset_x + (1-ratio) * toe_zmp_offset_x;
        }
        zmp_diff = swing_foot_zmp_offsets.front()(0)-default_zmp_offsets[swing_leg_types_list[refzmp_index].front()](0);
        if ((is_second_phase() && ( cnt < double_support_count_half_before )) ||
            (is_second_last_phase() && ( cnt > one_step_count - double_support_count_half_after ))) {
            // "* 0.5" is for double supprot period
            zmp_diff *= 0.5;
        }
    }

    // Calculate total reference ZMP
    if (is_start_double_support_phase() || is_end_double_support_phase()) {
      ret = refzmp_cur_list[refzmp_index];
    } else if ( cnt < double_support_static_count_half_before ) { // Start double support static period
      hrp::Vector3 current_support_zmp = refzmp_cur_list[refzmp_index];
      hrp::Vector3 prev_support_zmp = refzmp_cur_list[refzmp_index-1] + zmp_diff * foot_x_axises_list[refzmp_index-1].front();
      double ratio = (is_second_phase()?1.0:0.5);
      ret = (1 - ratio) * current_support_zmp + ratio * prev_support_zmp;
    } else if ( cnt > one_step_count - double_support_static_count_half_after ) { // End double support static period
      hrp::Vector3 current_support_zmp = refzmp_cur_list[refzmp_index+1] + zmp_diff * foot_x_axises_list[refzmp_index+1].front();
      hrp::Vector3 prev_support_zmp = refzmp_cur_list[refzmp_index];
      double ratio = (is_second_last_phase()?1.0:0.5);
      ret = (1 - ratio) * prev_support_zmp + ratio * current_support_zmp;
    } else if ( cnt < double_support_count_half_before ) { // Start double support period
      hrp::Vector3 current_support_zmp = refzmp_cur_list[refzmp_index];
      hrp::Vector3 prev_support_zmp = refzmp_cur_list[refzmp_index-1] + zmp_diff * foot_x_axises_list[refzmp_index-1].front();
      double ratio = ((is_second_phase()?1.0:0.5) / (double_support_count_half_before-double_support_static_count_half_before)) * (double_support_count_half_before-cnt);
      ret = (1 - ratio) * current_support_zmp + ratio * prev_support_zmp;
    } else if ( cnt > one_step_count - double_support_count_half_after ) { // End double support period
      hrp::Vector3 current_support_zmp = refzmp_cur_list[refzmp_index+1] + zmp_diff * foot_x_axises_list[refzmp_index+1].front();
      hrp::Vector3 prev_support_zmp = refzmp_cur_list[refzmp_index];
      double ratio = ((is_second_last_phase()?1.0:0.5) / (double_support_count_half_after-double_support_static_count_half_after)) * (cnt - 1 - (one_step_count - double_support_count_half_after));
      ret = (1 - ratio) * prev_support_zmp + ratio * current_support_zmp;
    } else {
      ret = refzmp_cur_list[refzmp_index];
    }
  };

  void refzmp_generator::update_refzmp (const std::vector< std::vector<step_node> >& fnsl)
  {
    if ( 1 <= refzmp_count ) {
      refzmp_count--;
    } else {
      refzmp_index++;
      refzmp_count = one_step_count = step_count_list[refzmp_index];
      //std::cerr << "fs " << fs_index << "/" << fnl.size() << " rf " << refzmp_index << "/" << refzmp_cur_list.size() << " flg " << std::endl;
    }
  };

  /* member function implementation for leg_coords_generator */
  void leg_coords_generator::calc_current_swing_leg_steps (std::vector<step_node>& rets, const double step_height, const double _current_toe_angle, const double _current_heel_angle)
  {
    /* match the src step order and the dst step order */
    std::sort(swing_leg_src_steps.begin(), swing_leg_src_steps.end(),
              ((&boost::lambda::_1->* &step_node::l_r) < (&boost::lambda::_2->* &step_node::l_r)));
    std::sort(swing_leg_dst_steps.begin(), swing_leg_dst_steps.end(),
              ((&boost::lambda::_1->* &step_node::l_r) < (&boost::lambda::_2->* &step_node::l_r)));
    for (std::vector<step_node>::iterator it1 = swing_leg_src_steps.begin(), it2 = swing_leg_dst_steps.begin();
         it1 != swing_leg_src_steps.end() && it2 != swing_leg_dst_steps.end();
         it1++, it2++) {
      coordinates ret;
      switch (default_orbit_type) {
      case SHUFFLING:
        mid_coords(ret, swing_rot_ratio, it1->worldcoords, it2->worldcoords);
        break;
      case CYCLOID:
        cycloid_midcoords(ret, it1->worldcoords, it2->worldcoords, step_height);
        break;
      case RECTANGLE:
        rectangle_midcoords(ret, it1->worldcoords, it2->worldcoords, step_height);
        break;
      case STAIR:
        stair_midcoords(ret, it1->worldcoords, it2->worldcoords, step_height);
        break;
      case CYCLOIDDELAY:
        cycloid_delay_midcoords(ret, it1->worldcoords, it2->worldcoords, step_height);
        break;
      case CYCLOIDDELAYKICK:
        cycloid_delay_kick_midcoords(ret, it1->worldcoords, it2->worldcoords, step_height);
        break;
      case CROSS:
        cross_delay_midcoords(ret, it1->worldcoords, it2->worldcoords, step_height, it1->l_r);
        break;
      default: break;
      }
      if (std::fabs(step_height) > 1e-3*10) {
          if (swing_leg_src_steps.size() == 1) /* only biped or crawl because there is only one toe_heel_interpolator */
              modif_foot_coords_for_toe_heel_phase(ret, _current_toe_angle, _current_heel_angle);
      }
      rets.push_back(step_node(it1->l_r, ret, 0, 0, 0, 0));
    }
  };

  void leg_coords_generator::calc_ratio_from_double_support_ratio (const double default_double_support_ratio_before, const double default_double_support_ratio_after)
  {
    int support_len_before = one_step_count * default_double_support_ratio_before;
    int support_len_after = one_step_count * default_double_support_ratio_after;
    // int support_len = 2*static_cast<int>(one_step_count * default_double_support_ratio * 0.5);
    int swing_len = one_step_count - support_len_before - support_len_after;
    int current_swing_len = lcg_count - support_len_before;
    double tmp_current_swing_time;
    int current_swing_count = (one_step_count - lcg_count); // 0->one_step_count
    if ( current_swing_count < support_len_before ) { // First double support period
      swing_ratio = swing_rot_ratio = 0.0;
      tmp_current_swing_time = current_swing_len * dt - swing_len * dt;
      is_swing_phase = false;
    } else if ( current_swing_count >= support_len_before+swing_len ) { // Last double support period
      swing_ratio = swing_rot_ratio = 1.0;
      tmp_current_swing_time = current_swing_len * dt + (support_len_before + support_len_after + next_one_step_count) * dt;
      is_swing_phase = false;
    } else {
      if (current_swing_count == support_len_before) {
          double tmp = 0.0;
          swing_foot_rot_ratio_interpolator->clear();
          swing_foot_rot_ratio_interpolator->set(&tmp);
          tmp = 1.0;
          // int reduced_swing_len = 0.95*swing_len; // For margin from early landing
          // swing_foot_rot_ratio_interpolator->go(&tmp, dt * reduced_swing_len);
          //swing_foot_rot_ratio_interpolator->go(&tmp, dt * swing_len);
          swing_foot_rot_ratio_interpolator->setGoal(&tmp, dt * swing_len);
          swing_foot_rot_ratio_interpolator->sync();
      }
      if (!swing_foot_rot_ratio_interpolator->isEmpty()) {
          swing_foot_rot_ratio_interpolator->get(&swing_rot_ratio, true);
      } else {
          swing_foot_rot_ratio_interpolator->get(&swing_rot_ratio, false);
      }
      tmp_current_swing_time = current_swing_len * dt;
      swing_ratio = static_cast<double>(current_swing_count-support_len_before)/swing_len;
      //std::cerr << "gp " << swing_ratio << " " << swing_rot_ratio << std::endl;
      if (current_step_height > 0.0) is_swing_phase = true;
      else is_swing_phase = false;
    }
    for (std::vector<leg_type>::const_iterator it = support_leg_types.begin(); it != support_leg_types.end(); it++) {
        current_swing_time.at(*it) = (lcg_count + default_double_support_ratio_before * next_one_step_count) * dt;
    }
    for (std::vector<leg_type>::const_iterator it = swing_leg_types.begin(); it != swing_leg_types.end(); it++) {
        if (current_step_height > 0.0) {
            current_swing_time.at(*it) = tmp_current_swing_time;
        } else {
            current_swing_time.at(*it) = (lcg_count + default_double_support_ratio_before * next_one_step_count) * dt;
        }
    }
    //std::cerr << "sl " << support_leg << " " << current_swing_time[support_leg==RLEG?0:1] << " " << current_swing_time[support_leg==RLEG?1:0] << " " << tmp_current_swing_time << " " << lcg_count << std::endl;
  };

  double leg_coords_generator::calc_interpolated_toe_heel_angle (const toe_heel_phase start_phase, const toe_heel_phase goal_phase, const double start, const double goal)
  {
      double tmp_ip_ratio;
      size_t current_count = one_step_count - lcg_count;
      if (thp_ptr->is_phase_starting(current_count, start_phase)) {
          toe_heel_interpolator->clear();
          toe_heel_interpolator->set(&start);
          //toe_heel_interpolator->go(&goal, thp_ptr->calc_phase_period(start_phase, goal_phase, dt));
          toe_heel_interpolator->setGoal(&goal, thp_ptr->calc_phase_period(start_phase, goal_phase, dt));
          toe_heel_interpolator->sync();
      }
      if (!toe_heel_interpolator->isEmpty()) {
          toe_heel_interpolator->get(&tmp_ip_ratio, true);
      } else {
          toe_heel_interpolator->get(&tmp_ip_ratio, false);
      }
      return tmp_ip_ratio;
  };

  void leg_coords_generator::modif_foot_coords_for_toe_heel_phase (coordinates& org_coords, const double _current_toe_angle, const double _current_heel_angle)
  {
      coordinates new_coords;
      size_t current_count = one_step_count - lcg_count;
      double dif_angle = 0.0;
      hrp::Vector3 ee_local_pivot_pos(hrp::Vector3(0,0,0));
      if ( thp_ptr->is_between_phases(current_count, SOLE0, SOLE2TOE) ) {
          dif_angle = calc_interpolated_toe_heel_angle(SOLE0, SOLE2TOE, 0.0, _current_toe_angle);
          ee_local_pivot_pos(0) = toe_pos_offset_x;
      } else if ( thp_ptr->is_between_phases(current_count, SOLE2HEEL, HEEL2SOLE) ) {
          dif_angle = calc_interpolated_toe_heel_angle(SOLE2HEEL, HEEL2SOLE, -1 * _current_heel_angle, 0.0);
          ee_local_pivot_pos(0) = heel_pos_offset_x;
      } else if ( thp_ptr->is_between_phases(current_count, SOLE2TOE, SOLE2HEEL) ) {
          // If SOLE1 phase does not exist, interpolate toe => heel smoothly, without 0 velocity phase.
          if ( thp_ptr->is_no_SOLE1_phase() ) {
              dif_angle = calc_interpolated_toe_heel_angle(SOLE2TOE, SOLE2HEEL, _current_toe_angle, -1 * _current_heel_angle);
              double tmpd = (-1*_current_heel_angle-_current_toe_angle);
              if (std::fabs(tmpd) > 1e-5) {
                  ee_local_pivot_pos(0) = (heel_pos_offset_x - toe_pos_offset_x) * (dif_angle - _current_toe_angle) / tmpd + toe_pos_offset_x;
              }
          } else {
              if ( thp_ptr->is_between_phases(current_count, SOLE2TOE, TOE2SOLE) ) {
                  dif_angle = calc_interpolated_toe_heel_angle(SOLE2TOE, TOE2SOLE, _current_toe_angle, 0.0);
                  ee_local_pivot_pos(0) = toe_pos_offset_x;
              } else if ( thp_ptr->is_between_phases(current_count, SOLE1, SOLE2HEEL) ) {
                  dif_angle = calc_interpolated_toe_heel_angle(SOLE1, SOLE2HEEL, 0.0, -1 * _current_heel_angle);
                  ee_local_pivot_pos(0) = heel_pos_offset_x;
              }
          }
      }
      foot_dif_rot_angle = (dif_angle > 0.0 ? deg2rad(dif_angle) : 0.0);
      if (use_toe_joint && dif_angle > 0.0) dif_angle = 0.0;
      Eigen::AngleAxis<double> tmpr(deg2rad(dif_angle), hrp::Vector3::UnitY());
      rotm3times(new_coords.rot, org_coords.rot, tmpr.toRotationMatrix());
      new_coords.pos = org_coords.pos + org_coords.rot * ee_local_pivot_pos - new_coords.rot * ee_local_pivot_pos;
      org_coords = new_coords;
  };

  void leg_coords_generator::cycloid_midcoords (coordinates& ret, const coordinates& start,
                                                                const coordinates& goal, const double height) const
  {
    mid_coords(ret, swing_rot_ratio, start, goal);
    cycloid_midpoint (ret.pos, swing_ratio, start.pos, goal.pos, height, default_top_ratio);
  };

  void leg_coords_generator::rectangle_midcoords (coordinates& ret, const coordinates& start,
                                                                  const coordinates& goal, const double height)
  {
    mid_coords(ret, swing_rot_ratio, start, goal);
    rdtg.get_trajectory_point(ret.pos, hrp::Vector3(start.pos), hrp::Vector3(goal.pos), height);
  };

  void leg_coords_generator::stair_midcoords (coordinates& ret, const coordinates& start,
                                                              const coordinates& goal, const double height)
  {
    mid_coords(ret, swing_rot_ratio, start, goal);
    sdtg.get_trajectory_point(ret.pos, hrp::Vector3(start.pos), hrp::Vector3(goal.pos), height);
  };

  void leg_coords_generator::cycloid_delay_midcoords (coordinates& ret, const coordinates& start,
                                                                      const coordinates& goal, const double height)
  {
    mid_coords(ret, swing_rot_ratio, start, goal);
    cdtg.get_trajectory_point(ret.pos, hrp::Vector3(start.pos), hrp::Vector3(goal.pos), height);
  };

  void leg_coords_generator::cycloid_delay_kick_midcoords (coordinates& ret, const coordinates& start,
                                                                      const coordinates& goal, const double height)
  {
    mid_coords(ret, swing_rot_ratio, start, goal);
    cdktg.set_start_rot(hrp::Matrix33(start.rot));
    cdktg.get_trajectory_point(ret.pos, hrp::Vector3(start.pos), hrp::Vector3(goal.pos), height);
  };

  void leg_coords_generator::cross_delay_midcoords (coordinates& ret, const coordinates& start,
                                                    const coordinates& goal, const double height, leg_type lr)
  {
    mid_coords(ret, swing_rot_ratio, start, goal);
    crdtg.set_swing_leg(lr);
    crdtg.get_trajectory_point(ret.pos, hrp::Vector3(start.pos), hrp::Vector3(goal.pos), height);
  };

  bool leg_coords_generator::is_same_footstep_nodes(const std::vector<step_node>& fns_1, const std::vector<step_node>& fns_2)
  {
      bool matching_flag = true;
      if (fns_1.size() == fns_2.size()) {
          for (std::vector<step_node>::const_iterator it1 = fns_1.begin(); it1 != fns_1.end(); it1++) {
              std::vector<step_node>::const_iterator it2 = std::find_if(fns_2.begin(), fns_2.end(), (&boost::lambda::_1->* &step_node::l_r == it1->l_r));
              if (it2 == fns_2.end()) {
                  matching_flag = false;
                  break;
              }
          }
      } else {
          matching_flag = false;
      }
      return matching_flag;
  };

  void leg_coords_generator::update_leg_steps (const std::vector< std::vector<step_node> >& fnsl, const double default_double_support_ratio_before, const double default_double_support_ratio_after)
  {
    if (!foot_ratio_interpolator->isEmpty()) {
        foot_ratio_interpolator->get(&foot_midcoords_ratio, true);
    }

    // Get current swing coords, support coords, and support leg parameters
    size_t current_footstep_index = (footstep_index < fnsl.size() - 1 ? footstep_index : fnsl.size()-1);
    swing_leg_dst_steps = fnsl[current_footstep_index];
    if (footstep_index != 0) { // If not initial step, support_leg_coords is previous swing_leg_dst_coords // why we need this?
        support_leg_steps = support_leg_steps_list[current_footstep_index];
    }
    support_leg_types.clear();
    for (std::vector<step_node>::iterator it = support_leg_steps.begin(); it != support_leg_steps.end(); it++) {
        support_leg_types.push_back(it->l_r);
    }
    swing_leg_types.clear();
    for (std::vector<step_node>::iterator it = swing_leg_dst_steps.begin(); it != swing_leg_dst_steps.end(); it++) {
        swing_leg_types.push_back(it->l_r);
    }
    if (current_footstep_index > 0) {
      if (is_same_footstep_nodes(fnsl[current_footstep_index], fnsl[current_footstep_index-1])) {
            swing_leg_src_steps = swing_leg_dst_steps_list[current_footstep_index-1];
        } else {
            /* current swing leg src coords = (previout support leg coords + previous swing leg dst coords) - current support leg coords */
            std::vector<step_node> tmp_swing_leg_src_steps = support_leg_steps_list[current_footstep_index-1];
            std::copy(swing_leg_dst_steps_list[current_footstep_index-1].begin(),
                      swing_leg_dst_steps_list[current_footstep_index-1].end(),
                      std::back_inserter(tmp_swing_leg_src_steps));
            for (size_t i = 0; i < support_leg_steps.size(); i++) {
                std::vector<step_node>::iterator it = std::remove_if(tmp_swing_leg_src_steps.begin(), tmp_swing_leg_src_steps.end(), (&boost::lambda::_1->* &step_node::l_r == support_leg_steps.at(i).l_r));
                tmp_swing_leg_src_steps.erase(it, tmp_swing_leg_src_steps.end());
            }
            swing_leg_src_steps = tmp_swing_leg_src_steps;
        }
    }

    calc_ratio_from_double_support_ratio(default_double_support_ratio_before, default_double_support_ratio_after);
    swing_leg_steps.clear();
    calc_current_swing_leg_steps(swing_leg_steps, current_step_height, current_toe_angle, current_heel_angle);
    if ( 1 <= lcg_count ) {
      lcg_count--;
    } else {
      //std::cerr << "gp " << footstep_index << std::endl;
      if (footstep_index < fnsl.size() - 1) {
        footstep_index++;
      }
      if (footstep_index < fnsl.size() - 1) {
        current_step_height = fnsl[footstep_index].front().step_height;
        current_toe_angle = fnsl[footstep_index].front().toe_angle;
        current_heel_angle = fnsl[footstep_index].front().heel_angle;
      } else {
        current_step_height = current_toe_angle = current_heel_angle = 0.0;
      }
      if (footstep_index < fnsl.size()) {
        one_step_count = static_cast<size_t>(fnsl[footstep_index].front().step_time/dt);
      }
      if (footstep_index + 1 < fnsl.size()) {
        next_one_step_count = static_cast<size_t>(fnsl[footstep_index+1].front().step_time/dt);
      }
      lcg_count = one_step_count;
      rdtg.reset(one_step_count, default_double_support_ratio_before, default_double_support_ratio_after);
      sdtg.reset(one_step_count, default_double_support_ratio_before, default_double_support_ratio_after);
      cdtg.reset(one_step_count, default_double_support_ratio_before, default_double_support_ratio_after);
      cdktg.reset(one_step_count, default_double_support_ratio_before, default_double_support_ratio_after);
      crdtg.reset(one_step_count, default_double_support_ratio_before, default_double_support_ratio_after);
      reset_foot_ratio_interpolator();
    }
  };

  /* member function implementation for gait_generator */
  void gait_generator::initialize_gait_parameter (const hrp::Vector3& cog,
                                                  const std::vector<step_node>& initial_support_leg_steps,
                                                  const std::vector<step_node>& initial_swing_leg_dst_steps,
                                                  const double delay)
  {
    /* clear all gait_parameter */
    size_t one_step_len = footstep_nodes_list.front().front().step_time / dt;
    finalize_count = 0;
    for (std::vector<step_node>::iterator it_fns = footstep_nodes_list.front().begin(); it_fns != footstep_nodes_list.front().end(); it_fns++) {
        for (std::vector<step_node>::const_iterator it_init = initial_swing_leg_dst_steps.begin(); it_init != initial_swing_leg_dst_steps.end(); it_init++) {
            if (it_fns->l_r == it_init->l_r) {
                /* initial_swing_leg_dst_steps has dummy step_height, step_time, toe_angle and heel_angle. */
                it_fns->worldcoords = it_init->worldcoords;
                break;
            }
        }
    }
    rg.reset(one_step_len);
    rg.push_refzmp_from_footstep_nodes_for_dual(footstep_nodes_list.front(), initial_support_leg_steps, initial_swing_leg_dst_steps);
    if ( preview_controller_ptr != NULL ) {
      delete preview_controller_ptr;
      preview_controller_ptr = NULL;
    }
    //preview_controller_ptr = new preview_dynamics_filter<preview_control>(dt, cog(2) - refzmp_cur_list[0](2), refzmp_cur_list[0]);
    preview_controller_ptr = new preview_dynamics_filter<extended_preview_control>(dt, cog(2) - rg.get_refzmp_cur()(2), rg.get_refzmp_cur(), gravitational_acceleration);
    lcg.reset(one_step_len, footstep_nodes_list.at(1).front().step_time/dt, initial_swing_leg_dst_steps, initial_swing_leg_dst_steps, initial_support_leg_steps, default_double_support_ratio_swing_before, default_double_support_ratio_swing_after);
    /* make another */
    lcg.set_swing_support_steps_list(footstep_nodes_list);
    for (size_t i = 1; i < footstep_nodes_list.size()-1; i++) {
        rg.push_refzmp_from_footstep_nodes_for_single(footstep_nodes_list.at(i), lcg.get_support_leg_steps_idx(i));
    }
    rg.push_refzmp_from_footstep_nodes_for_dual(footstep_nodes_list.back(),
                                                lcg.get_support_leg_steps_idx(footstep_nodes_list.size()-1),
                                                lcg.get_swing_leg_dst_steps_idx(footstep_nodes_list.size()-1));
    emergency_flg = IDLING;
  };

  bool gait_generator::proc_one_tick ()
  {
    hrp::Vector3 rzmp;
    std::vector<hrp::Vector3> sfzos;
    bool refzmp_exist_p = rg.get_current_refzmp(rzmp, sfzos, default_double_support_ratio_before, default_double_support_ratio_after, default_double_support_static_ratio_before, default_double_support_static_ratio_after);
    if (!refzmp_exist_p) {
      finalize_count++;
      rzmp = prev_que_rzmp;
      sfzos = prev_que_sfzos;
    } else {
      prev_que_rzmp = rzmp;
      prev_que_sfzos = sfzos;
    }
    bool solved = preview_controller_ptr->update(refzmp, cog, swing_foot_zmp_offsets, rzmp, sfzos, (refzmp_exist_p || finalize_count < preview_controller_ptr->get_delay()-default_step_time/dt));
    /* update refzmp */
    if ( lcg.get_lcg_count() == static_cast<size_t>(footstep_nodes_list[lcg.get_footstep_index()][0].step_time/dt * 0.5) - 1 ) { // Almost middle of step time
      if (velocity_mode_flg != VEL_IDLING && lcg.get_footstep_index() > 0) {
        std::vector< std::vector<coordinates> > cv;
        calc_next_coords_velocity_mode(cv, lcg.get_footstep_index() + 1);
        if (velocity_mode_flg == VEL_ENDING) velocity_mode_flg = VEL_IDLING;
        std::vector<leg_type> cur_leg;
        for (size_t i = 0; i < footstep_nodes_list[lcg.get_footstep_index()].size(); i++) {
            cur_leg.push_back(footstep_nodes_list[lcg.get_footstep_index()].at(i).l_r);
        }
        overwrite_footstep_nodes_list.push_back(boost::assign::list_of(step_node(cur_leg.front()==RLEG?LLEG:RLEG, cv[0][0], lcg.get_default_step_height(), default_step_time, lcg.get_toe_angle(), lcg.get_heel_angle())));
        overwrite_footstep_nodes_list.push_back(boost::assign::list_of(step_node(cur_leg.front(), cv[1][0], lcg.get_default_step_height(), default_step_time, lcg.get_toe_angle(), lcg.get_heel_angle())));
        overwrite_footstep_nodes_list.push_back(boost::assign::list_of(step_node(cur_leg.front()==RLEG?LLEG:RLEG, cv[2][0], lcg.get_default_step_height(), default_step_time, lcg.get_toe_angle(), lcg.get_heel_angle())));
        overwrite_refzmp_queue(overwrite_footstep_nodes_list);
        overwrite_footstep_nodes_list.clear();
      } else if ( !overwrite_footstep_nodes_list.empty() && // If overwrite_footstep_node_list exists
                  (lcg.get_footstep_index() < footstep_nodes_list.size()-1) &&  // If overwrite_footstep_node_list is specified and current footstep is not last footstep.
                  get_overwritable_index() == overwrite_footstep_index ) {
        overwrite_refzmp_queue(overwrite_footstep_nodes_list);
        overwrite_footstep_nodes_list.clear();
      } else if (emergency_flg == EMERGENCY_STOP && lcg.get_footstep_index() > 0) {
        leg_type cur_leg = footstep_nodes_list[lcg.get_footstep_index()].front().l_r;
        overwrite_footstep_nodes_list.push_back(boost::assign::list_of(step_node(cur_leg==RLEG?LLEG:RLEG, footstep_nodes_list[lcg.get_footstep_index()-1].front().worldcoords, 0, default_step_time, 0, 0)));
        overwrite_footstep_nodes_list.push_back(boost::assign::list_of(step_node(cur_leg, footstep_nodes_list[lcg.get_footstep_index()].front().worldcoords, 0, default_step_time, 0, 0)));
        overwrite_footstep_nodes_list.push_back(boost::assign::list_of(step_node(cur_leg==RLEG?LLEG:RLEG, footstep_nodes_list[lcg.get_footstep_index()-1].front().worldcoords, 0, default_step_time, 0, 0)));
        overwrite_refzmp_queue(overwrite_footstep_nodes_list);
        overwrite_footstep_nodes_list.clear();
        emergency_flg = STOPPING;
      }
    }
    rg.update_refzmp(footstep_nodes_list);
    // { // debug
    //   double cart_zmp[3];
    //   preview_controller_ptr->get_cart_zmp(cart_zmp);
    //   std::cerr << "(list " << std::endl;
    //   std::cerr << ":cog "; print_vector(std::cerr, cog);
    //   std::cerr << ":refzmp "; print_vector(std::cerr, refzmp);
    //   std::cerr << ":cart-zmp "; print_vector(std::cerr, cart_zmp, 3);
    //   std::cerr << ")" << std::endl;
    // }

    /* update swing_leg_coords, support_leg_coords */
    if ( solved ) {
      lcg.update_leg_steps(footstep_nodes_list, default_double_support_ratio_swing_before, default_double_support_ratio_swing_after);
    } else if (finalize_count>0) {
      lcg.clear_interpolators();
    }
    return solved;
  };

  /* generate vector of step_node from :go-pos params
   *  x, y and theta are simply divided by using stride params
   *  unit system -> x [mm], y [mm], theta [deg]
   */
  void gait_generator::go_pos_param_2_footstep_nodes_list (const double goal_x, const double goal_y, const double goal_theta, /* [mm] [mm] [deg] */
                                                           const std::vector<coordinates>& initial_support_legs_coords, coordinates start_ref_coords,
                                                           const std::vector<leg_type>& initial_support_legs)
  {
    coordinates goal_ref_coords(start_ref_coords);
    goal_ref_coords.pos += goal_ref_coords.rot * hrp::Vector3(goal_x, goal_y, 0.0);
    goal_ref_coords.rotate(deg2rad(goal_theta), hrp::Vector3(0,0,1));
    std::cerr << "start ref coords" << std::endl;
    std::cerr << "  pos =" << std::endl;
    std::cerr << start_ref_coords.pos.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << std::endl;
    std::cerr << "  rot =" << std::endl;
    std::cerr << start_ref_coords.rot.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "    [", "]")) << std::endl;
    std::cerr << "goal ref midcoords" << std::endl;
    std::cerr << "  pos =" << std::endl;
    std::cerr << goal_ref_coords.pos.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << std::endl;
    std::cerr << "  rot =" << std::endl;
    std::cerr << goal_ref_coords.rot.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "    [", "]")) << std::endl;

    /* initialize */
    clear_footstep_nodes_list();
    // For initial double support period
    std::vector<step_node> initial_footstep_nodes;
    for (size_t i = 0; i < initial_support_legs.size(); i++) {
        initial_footstep_nodes.push_back(step_node(initial_support_legs.at(i), initial_support_legs_coords.at(i), 0, default_step_time, 0, 0));
    }
    footstep_nodes_list.push_back(initial_footstep_nodes);

    /* footstep generation loop */
    hrp::Vector3 dp, dr;
    start_ref_coords.difference(dp, dr, goal_ref_coords);
    dp = start_ref_coords.rot.transpose() * dp;
    dr = start_ref_coords.rot.transpose() * dr;
    while ( !(eps_eq(std::sqrt(dp(0)*dp(0)+dp(1)*dp(1)), 0.0, 1e-3*0.1) && eps_eq(dr(2), 0.0, deg2rad(0.5))) ) {
      set_velocity_param(dp(0)/default_step_time, dp(1)/default_step_time, rad2deg(dr(2))/default_step_time);
      append_footstep_list_velocity_mode();
      start_ref_coords = footstep_nodes_list.back().front().worldcoords;
      start_ref_coords.pos += start_ref_coords.rot * hrp::Vector3(footstep_param.leg_default_translate_pos[footstep_nodes_list.back().front().l_r] * -1.0);
      start_ref_coords.difference(dp, dr, goal_ref_coords);
      dp = start_ref_coords.rot.transpose() * dp;
      dr = start_ref_coords.rot.transpose() * dr;
    }
    for (size_t i = 0; i < optional_go_pos_finalize_footstep_num; i++) {
        append_go_pos_step_nodes(start_ref_coords, calc_counter_leg_types_from_footstep_nodes(footstep_nodes_list.back(), all_limbs));
    }

    /* finalize */
    //   Align last foot
    append_go_pos_step_nodes(start_ref_coords, calc_counter_leg_types_from_footstep_nodes(footstep_nodes_list.back(), all_limbs));
    //   Check align
    coordinates final_step_coords1 = footstep_nodes_list[footstep_nodes_list.size()-2].front().worldcoords; // Final coords in footstep_node_list
    coordinates final_step_coords2 = start_ref_coords; // Final coords calculated from start_ref_coords + translate pos
    final_step_coords2.pos += final_step_coords2.rot * hrp::Vector3(footstep_param.leg_default_translate_pos[footstep_nodes_list[footstep_nodes_list.size()-2].front().l_r]);
    final_step_coords1.difference(dp, dr, final_step_coords2);
    if ( !(eps_eq(dp.norm(), 0.0, 1e-3*0.1) && eps_eq(dr.norm(), 0.0, deg2rad(0.5))) ) { // If final_step_coords1 != final_step_coords2, add steps to match final_step_coords1 and final_step_coords2
        append_go_pos_step_nodes(start_ref_coords, calc_counter_leg_types_from_footstep_nodes(footstep_nodes_list.back(), all_limbs));
    }
    //   For Last double support period
    append_finalize_footstep();
    print_footstep_nodes_list();
  };

  void gait_generator::go_single_step_param_2_footstep_nodes_list (const double goal_x, const double goal_y, const double goal_z, const double goal_theta,
                                                             const std::string& tmp_swing_leg,
                                                             const coordinates& _support_leg_coords)
  {
    leg_type _swing_leg = (tmp_swing_leg == "rleg") ? RLEG : LLEG;
    step_node sn0((_swing_leg == RLEG) ? LLEG : RLEG, _support_leg_coords, lcg.get_default_step_height(), default_step_time, lcg.get_toe_angle(), lcg.get_heel_angle());
    footstep_nodes_list.push_back(boost::assign::list_of(sn0));
    step_node sn1(_swing_leg, _support_leg_coords, lcg.get_default_step_height(), default_step_time, lcg.get_toe_angle(), lcg.get_heel_angle());
    hrp::Vector3 trs(2.0 * footstep_param.leg_default_translate_pos[_swing_leg] + hrp::Vector3(goal_x, goal_y, goal_z));
    sn1.worldcoords.pos += sn1.worldcoords.rot * trs;
    sn1.worldcoords.rotate(deg2rad(goal_theta), hrp::Vector3(0,0,1));
    footstep_nodes_list.push_back(boost::assign::list_of(sn1));
    footstep_nodes_list.push_back(boost::assign::list_of(sn0));
  };

  void gait_generator::initialize_velocity_mode (const coordinates& _ref_coords,
						 const double vel_x, const double vel_y, const double vel_theta)
  {
    velocity_mode_flg = VEL_DOING;
    /* initialize */
    leg_type current_leg = (vel_y > 0.0) ? RLEG : LLEG;
    clear_footstep_nodes_list();
    set_velocity_param (vel_x, vel_y, vel_theta);
    append_go_pos_step_nodes(_ref_coords, boost::assign::list_of(current_leg));
    append_footstep_list_velocity_mode();
    append_footstep_list_velocity_mode();
    append_footstep_list_velocity_mode();
  };

  void gait_generator::finalize_velocity_mode ()
  {
    if (velocity_mode_flg == VEL_DOING) velocity_mode_flg = VEL_ENDING;
  };

  void gait_generator::calc_ref_coords_trans_vector_velocity_mode (coordinates& ref_coords, hrp::Vector3& trans, double& dth, const std::vector<step_node>& sup_fns)
  {
    ref_coords = sup_fns.front().worldcoords;
    hrp::Vector3 tmpv(footstep_param.leg_default_translate_pos[sup_fns.front().l_r] * -1.0); /* not fair to every support legs */
    ref_coords.pos += ref_coords.rot * tmpv;
    double dx = vel_param.velocity_x + offset_vel_param.velocity_x, dy = vel_param.velocity_y + offset_vel_param.velocity_y;
    dth = vel_param.velocity_theta + offset_vel_param.velocity_theta;
    /* velocity limitation by stride parameters <- this should be based on footstep candidates */
    dx  = std::max(-1 * footstep_param.stride_bwd_x / default_step_time, std::min(footstep_param.stride_fwd_x / default_step_time, dx ));
    dy  = std::max(-1 * footstep_param.stride_y     / default_step_time, std::min(footstep_param.stride_y     / default_step_time, dy ));
    dth = std::max(-1 * footstep_param.stride_theta / default_step_time, std::min(footstep_param.stride_theta / default_step_time, dth));
    /* inside step limitation */
    if (use_inside_step_limitation) {
        if (vel_param.velocity_y > 0) {
            if (std::count_if(sup_fns.begin(), sup_fns.end(), (&boost::lambda::_1->* &step_node::l_r == LLEG || &boost::lambda::_1->* &step_node::l_r == LARM)) > 0) dy *= 0.5;
        } else {
            if (std::count_if(sup_fns.begin(), sup_fns.end(), (&boost::lambda::_1->* &step_node::l_r == RLEG || &boost::lambda::_1->* &step_node::l_r == RARM)) > 0) dy *= 0.5;
        }
        if (vel_param.velocity_theta > 0) {
            if (std::count_if(sup_fns.begin(), sup_fns.end(), (&boost::lambda::_1->* &step_node::l_r == LLEG || &boost::lambda::_1->* &step_node::l_r == LARM)) > 0) dth *= 0.5;
        } else {
            if (std::count_if(sup_fns.begin(), sup_fns.end(), (&boost::lambda::_1->* &step_node::l_r == RLEG || &boost::lambda::_1->* &step_node::l_r == RARM)) > 0) dth *= 0.5;
        }
    }
    trans = hrp::Vector3(dx * default_step_time, dy * default_step_time, 0);
    dth = deg2rad(dth * default_step_time);
  };

  void gait_generator::append_footstep_list_velocity_mode ()
  {
    coordinates ref_coords;
    hrp::Vector3 trans;
    double dth;
    calc_ref_coords_trans_vector_velocity_mode(ref_coords, trans, dth, footstep_nodes_list.back());

    ref_coords.pos += ref_coords.rot * trans;
    ref_coords.rotate(dth, hrp::Vector3(0,0,1));
    append_go_pos_step_nodes(ref_coords, calc_counter_leg_types_from_footstep_nodes(footstep_nodes_list.back(), all_limbs));
  };

  void gait_generator::calc_next_coords_velocity_mode (std::vector< std::vector<coordinates> >& ret_list, const size_t idx)
  {
    coordinates ref_coords;
    hrp::Vector3 trans;
    double dth;
    calc_ref_coords_trans_vector_velocity_mode(ref_coords, trans, dth, footstep_nodes_list[idx-1]);

    std::vector<leg_type> cur_sup_legs, next_sup_legs;
    for (size_t i = 0; i < footstep_nodes_list[idx-1].size(); i++) cur_sup_legs.push_back(footstep_nodes_list[idx-1].at(i).l_r);
    next_sup_legs = calc_counter_leg_types_from_footstep_nodes(footstep_nodes_list[idx-1], all_limbs);

    for (size_t i = 0; i < 3; i++) {
      std::vector<coordinates> ret;
      std::vector<leg_type> forcused_sup_legs;
      switch( i % 2) {
      case 0: forcused_sup_legs = next_sup_legs; break;
      case 1: forcused_sup_legs = cur_sup_legs; break;
      }
      for (size_t j = 0; j < forcused_sup_legs.size(); j++) {
          ret.push_back(ref_coords);
          if ( velocity_mode_flg != VEL_ENDING ) {
              ret[j].pos += ret[j].rot * trans;
              ret[j].rotate(dth, hrp::Vector3(0,0,1));
          }
          ret[j].pos += ret[j].rot * footstep_param.leg_default_translate_pos[forcused_sup_legs.at(j)];
      }
      ret_list.push_back(ret);
    }
  };

  void gait_generator::overwrite_refzmp_queue(const std::vector< std::vector<step_node> >& fnsl)
  {
    /* clear footstep and refzmp after footstep_index + 1, it means we do not modify current step */
    size_t idx = get_overwritable_index();
    footstep_nodes_list.erase(footstep_nodes_list.begin()+idx, footstep_nodes_list.end());

    /* add new next steps ;; the number of next steps is fnsl.size() */
    footstep_nodes_list.insert(footstep_nodes_list.end(), fnsl.begin(), fnsl.end());

    /* remove refzmp after idx for allocation of new refzmp by push_refzmp_from_footstep_nodes */
    rg.remove_refzmp_cur_list_over_length(idx);
    /* remove refzmp in preview contoroller queue */
    preview_controller_ptr->remove_preview_queue(lcg.get_lcg_count());

    /* reset index and counter */
    rg.set_indices(idx);
    rg.set_refzmp_count(static_cast<size_t>(fnsl[0][0].step_time/dt));
    lcg.set_swing_support_steps_list(footstep_nodes_list);
    /* reset refzmp */
    for (size_t i = 0; i < fnsl.size(); i++) {
        if (emergency_flg == EMERGENCY_STOP)
            rg.push_refzmp_from_footstep_nodes_for_dual(footstep_nodes_list[idx+i],
                                                        lcg.get_swing_leg_dst_steps_idx(footstep_nodes_list.size()-1),
                                                        lcg.get_support_leg_steps_idx(footstep_nodes_list.size()-1));
        else {
            if (i==fnsl.size()-1) {
                rg.push_refzmp_from_footstep_nodes_for_dual(footstep_nodes_list[fnsl.size()-1],
                                                            lcg.get_swing_leg_dst_steps_idx(footstep_nodes_list.size()-1),
                                                            lcg.get_support_leg_steps_idx(footstep_nodes_list.size()-1));
            } else {
                rg.push_refzmp_from_footstep_nodes_for_single(footstep_nodes_list[idx+i], lcg.get_support_leg_steps_idx(idx+i));
            }
        }
    }
    /* fill preview controller queue by new refzmp */
    hrp::Vector3 rzmp;
    bool not_solved = true;
    while (not_solved) {
      std::vector<hrp::Vector3> sfzos;
      bool refzmp_exist_p = rg.get_current_refzmp(rzmp, sfzos, default_double_support_ratio_before, default_double_support_ratio_after, default_double_support_static_ratio_before, default_double_support_static_ratio_after);
      not_solved = !preview_controller_ptr->update(refzmp, cog, swing_foot_zmp_offsets, rzmp, sfzos, refzmp_exist_p);
      rg.update_refzmp(footstep_nodes_list);
    }
  };

  const std::vector<leg_type> gait_generator::calc_counter_leg_types_from_footstep_nodes(const std::vector<step_node>& fns, std::vector<std::string> _all_limbs) const {
    std::vector<std::string> fns_names, cntr_legs_names;
    for (std::vector<step_node>::const_iterator it = fns.begin(); it != fns.end(); it++) {
        fns_names.push_back(leg_type_map.find(it->l_r)->second);
    }
    std::sort(_all_limbs.begin(), _all_limbs.end());
    std::sort(fns_names.begin(), fns_names.end());
    std::set_difference(_all_limbs.begin(), _all_limbs.end(), /* all candidates for legs */
                        fns_names.begin(), fns_names.end(),   /* support legs */
                        std::back_inserter(cntr_legs_names)); /* swing   legs */
    std::vector<leg_type> ret;
    for (std::vector<std::string>::const_iterator it = cntr_legs_names.begin(); it != cntr_legs_names.end(); it++) {
        std::map<leg_type, std::string>::const_iterator dst = std::find_if(leg_type_map.begin(), leg_type_map.end(), (&boost::lambda::_1->* &std::map<leg_type, std::string>::value_type::second == *it));
        ret.push_back(dst->first);
    }
    return ret;
  };
}

