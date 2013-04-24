/* -*- coding:utf-8-unix; mode:c++; -*- */

#include "GaitGenerator.h"
namespace rats
{
#ifndef rad2deg
#define rad2deg(rad) (rad * 180 / M_PI)
#endif
#ifndef deg2rad
#define deg2rad(deg) (deg * M_PI / 180)
#endif
  /* member function implementation for refzmp_generator */
  void gait_generator::refzmp_generator::push_refzmp_from_footstep_list_for_dual (const std::vector<step_node>& fnl,
                                                                                  const coordinates& _support_leg_coords,
                                                                                  const coordinates& _swing_leg_coords)
  {
    hrp::Vector3 rzmp;
    hrp::Vector3 dz0, dz1, ret_zmp;
    leg_type spl = (fnl[fs_index].l_r == WC_RLEG) ? WC_LLEG :WC_RLEG;
    _support_leg_coords.rotate_vector(dz0, (spl == WC_RLEG)?
                                      default_zmp_offsets[0] : default_zmp_offsets[1]);
    _swing_leg_coords.rotate_vector(dz1, (spl == WC_RLEG)?
                                    default_zmp_offsets[1] : default_zmp_offsets[0]);
    dz0 += _support_leg_coords.pos;
    dz1 += _swing_leg_coords.pos;
    rzmp = (dz0 + dz1) / 2.0;
    refzmp_cur_list.push_back( rzmp );
    fs_index++;
  };

  void gait_generator::refzmp_generator::push_refzmp_from_footstep_list_for_single (const std::vector<step_node>& fnl)
  {
    hrp::Vector3 rzmp;
    coordinates tmp(fnl[fs_index-1].worldcoords);
    tmp.rotate_vector(rzmp, (fnl[fs_index-1].l_r == WC_RLEG)?
                      default_zmp_offsets[0] : default_zmp_offsets[1]);
    rzmp += tmp.pos;
    refzmp_cur_list.push_back( rzmp );
    if (fs_index < fnl.size()) fs_index++;
  };

  void gait_generator::refzmp_generator::calc_current_refzmp (hrp::Vector3& ret, const double default_double_support_ratio, const size_t one_step_len) const
  {
    size_t cnt = one_step_len - refzmp_count;
    double margine_count = 0.5 * default_double_support_ratio * one_step_len;
    if ( cnt < margine_count ) {
      double ratio = (-0.5 / margine_count) * (cnt - margine_count);
      ret = (1 - ratio) * refzmp_cur_list[refzmp_index] + ratio * ((refzmp_index > 0) ? refzmp_cur_list[refzmp_index-1] : refzmp_cur_list[refzmp_index]);
    } else if ( cnt > one_step_len - margine_count ) {
      double ratio = (0.5 / margine_count) * (cnt - (one_step_len - margine_count));
      ret = (1 - ratio) * refzmp_cur_list[refzmp_index] + ratio * ((refzmp_index + 1 <= refzmp_cur_list.size() - 1) ? refzmp_cur_list[refzmp_index+1] : refzmp_cur_list[refzmp_index]);
    } else {
      ret = refzmp_cur_list[refzmp_index];
    }
  };

  void gait_generator::refzmp_generator::update_refzmp (const std::vector<step_node>& fnl, const size_t one_step_len)
  {
    if ( 1 <= refzmp_count ) {
      refzmp_count--;
    } else {
      //std::cerr << "fs " << fs_index << "/" << fnl.size() << " rf " << refzmp_index << "/" << refzmp_cur_list.size() << " flg " << std::endl;
      if ( fnl.size() - 1 == fs_index ) {
        push_refzmp_from_footstep_list_for_dual(fnl, fnl[fs_index-1].worldcoords, fnl[fs_index-2].worldcoords);
      } else if ( fnl.size () - 1 > fs_index ) {
        push_refzmp_from_footstep_list_for_single(fnl);
      }
      refzmp_index++;
      refzmp_count = one_step_len;
    }
  };

  /* member function implementation for leg_coords_generator */
  void gait_generator::leg_coords_generator::calc_current_swing_leg_coords (coordinates& ret,
                                                                            const double ratio, const double step_height,
                                                                            const orbit_type type) const
  {
    switch (type) {
    case SHUFFLING:
      mid_coords(ret, ratio, swing_leg_src_coords, swing_leg_dst_coords);
      break;
    case CYCLOID:
      cycloid_midcoords(ret, ratio, swing_leg_src_coords, swing_leg_dst_coords, step_height);
      break;
    default: break;
    }
  };

  double gait_generator::leg_coords_generator::calc_ratio_from_double_support_ratio (const double default_double_support_ratio, const size_t one_step_len) const
  {
    double narrow_one_step_len = (1.0 - default_double_support_ratio) * one_step_len;
    double tmp_ratio = 1.0 - (gp_count - 0.5 * default_double_support_ratio * one_step_len) / narrow_one_step_len;
    double ret;
    if ( tmp_ratio < 0.0 ) ret = 0.0;
    else if ( tmp_ratio > 1.0 ) ret = 1.0;
    else ret = tmp_ratio;
    return ret;
  };

  void gait_generator::leg_coords_generator::cycloid_midpoint (hrp::Vector3& ret,
                                                               const double ratio, const hrp::Vector3& start,
                                                               const hrp::Vector3& goal, const double height) const
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

  void gait_generator::leg_coords_generator::cycloid_midcoords (coordinates& ret,
                                                                const double ratio, const coordinates& start,
                                                                const coordinates& goal, const double height) const
  {
    mid_coords(ret, ratio, start, goal);
    cycloid_midpoint (ret.pos, ratio, start.pos, goal.pos, height);
  };

  void gait_generator::leg_coords_generator::update_leg_coords (const std::vector<step_node>& fnl, const double default_double_support_ratio, const size_t one_step_len, const orbit_type type, const bool force_height_zero)
  {
    swing_ratio = calc_ratio_from_double_support_ratio(default_double_support_ratio, one_step_len);
    rot_ratio = 1.0 - (double)gp_count / one_step_len;
    if ( 0 == gp_index ) {
      swing_leg_dst_coords = fnl[gp_index].worldcoords;
      support_leg = fnl[gp_index+1].l_r;
    } else if (gp_index < fnl.size() - 1) {
      swing_leg_dst_coords = fnl[gp_index].worldcoords;
      support_leg_coords = fnl[gp_index-1].worldcoords;
      support_leg = fnl[gp_index-1].l_r;
    } else {
      swing_leg_dst_coords = fnl[fnl.size()-1].worldcoords;
      support_leg_coords = fnl[fnl.size()-2].worldcoords;
      support_leg = fnl[fnl.size()-2].l_r;
    }
    calc_current_swing_leg_coords(swing_leg_coords, swing_ratio, current_step_height, type);
    if ( 1 <= gp_count ) {
      gp_count--;
    } else {
      //std::cerr << "gp " << gp_index << std::endl;
      if (gp_index < fnl.size() - 1) {
        swing_leg_src_coords = support_leg_coords;
        gp_index++;
      }
      if (gp_index < fnl.size() - 1) {
        if (force_height_zero) current_step_height = 0.0;
        else current_step_height = default_step_height;
      } else {
        current_step_height = 0.0;
      }
      gp_count = one_step_len;
    }
  };

  /* member function implementation for gait_generator */
  void gait_generator::initialize_gait_parameter (const hrp::Vector3& cog,
                                                  const coordinates& initial_support_leg_coords,
                                                  const coordinates& initial_swing_leg_dst_coords,
                                                  const double delay)
  {
    /* clear all gait_parameter */
    one_step_len = default_step_time / dt;
    footstep_node_list[0].worldcoords = initial_swing_leg_dst_coords;
    rg.reset(one_step_len);
    rg.push_refzmp_from_footstep_list_for_dual(footstep_node_list, initial_support_leg_coords, initial_swing_leg_dst_coords);
    if ( preview_controller_ptr != NULL ) {
      delete preview_controller_ptr;
      preview_controller_ptr = NULL;
    }
    //preview_controller_ptr = new preview_dynamics_filter<preview_control>(dt, cog(2) - refzmp_cur_list[0](2), refzmp_cur_list[0]);
    preview_controller_ptr = new preview_dynamics_filter<extended_preview_control>(dt, cog(2) - rg.get_refzmp_cur()(2), rg.get_refzmp_cur());
    lcg.reset(one_step_len, initial_swing_leg_dst_coords, initial_swing_leg_dst_coords, initial_support_leg_coords);
    /* make another */
    rg.push_refzmp_from_footstep_list_for_single(footstep_node_list);
    emergency_flg = IDLING;
  };

  bool gait_generator::proc_one_tick (const orbit_type type)
  {
    hrp::Vector3 rzmp;
    bool refzmp_exist_p = rg.get_current_refzmp(rzmp, default_double_support_ratio, one_step_len);
    bool solved = preview_controller_ptr->update(refzmp, cog, rzmp, refzmp_exist_p);
    /* update refzmp */
    if ( lcg.get_gp_index() > 0 && lcg.get_gp_count() == static_cast<size_t>(one_step_len / 2) - 1 ) {
      if (velocity_mode_flg != VEL_IDLING) {
        std::vector<coordinates> cv;
        calc_next_coords_velocity_mode(cv, lcg.get_gp_index() + 1);
        if (velocity_mode_flg == VEL_ENDING) velocity_mode_flg = VEL_IDLING;
        overwrite_refzmp_queue(cv);
      } else if (emergency_flg == EMERGENCY_STOP) {
        std::vector<coordinates> cv;
        cv.push_back(footstep_node_list[lcg.get_gp_index()-1].worldcoords);
        cv.push_back(footstep_node_list[lcg.get_gp_index()].worldcoords);
        cv.push_back(footstep_node_list[lcg.get_gp_index()-1].worldcoords);
        overwrite_refzmp_queue(cv);
        emergency_flg = STOPPING;
      }
    }
    rg.update_refzmp(footstep_node_list, one_step_len);
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
      lcg.update_leg_coords(footstep_node_list, default_double_support_ratio, one_step_len, type, (emergency_flg == STOPPING));
    }
    return solved;
  };

  /* generate vector of step_node from :go-pos params
   *  x, y and theta are simply divided by using stride params
   *  unit system -> x [mm], y [mm], theta [deg]
   */
  void gait_generator::go_pos_param_2_footstep_list (const double goal_x, const double goal_y, const double goal_theta,
                                                     const coordinates& _foot_midcoords, const leg_type start_leg)
  {
    coordinates foot_midcoords(_foot_midcoords); /* foot_midcoords is modified during loop */
    coordinates goal_foot_midcoords(_foot_midcoords);
    goal_foot_midcoords.translate(hrp::Vector3(goal_x, goal_y, 0.0));
    goal_foot_midcoords.rotate(deg2rad(goal_theta), hrp::Vector3(0,0,1));
    std::cerr << "current fp";
    _foot_midcoords.print_eus_coordinates(std::cerr);
    std::cerr << "goal fp";
    goal_foot_midcoords.print_eus_coordinates(std::cerr);

    /* initialize */
    clear_footstep_node_list();
    append_go_pos_step_node(foot_midcoords, start_leg);

    /* footstep generation loop */
    hrp::Vector3 dp, dr;
    foot_midcoords.difference(dp, dr, goal_foot_midcoords);
    dp = foot_midcoords.rot.transpose() * dp;
    dr = foot_midcoords.rot.transpose() * dr;
    while ( !(eps_eq(dp.norm(), 0.0, 1e-3*0.1) && eps_eq(dr.norm(), 0.0, deg2rad(0.5))) ) {
      set_velocity_param(dp(0)/default_step_time, dp(1)/default_step_time, rad2deg(dr(2))/default_step_time);
      append_footstep_list_velocity_mode();
      foot_midcoords = footstep_node_list.back().worldcoords;
      foot_midcoords.translate(hrp::Vector3(footstep_param.leg_default_translate_pos[(footstep_node_list.back().l_r == WC_RLEG) ? 0 : 1] * -1.0));
      foot_midcoords.difference(dp, dr, goal_foot_midcoords);
      dp = foot_midcoords.rot.transpose() * dp;
      dr = foot_midcoords.rot.transpose() * dr;
    }

    /* finalize */
    append_go_pos_step_node(foot_midcoords, (footstep_node_list.back().l_r == WC_RLEG ? WC_LLEG : WC_RLEG));
    append_go_pos_step_node(foot_midcoords, (footstep_node_list.back().l_r == WC_RLEG ? WC_LLEG : WC_RLEG));
  };

  void gait_generator::go_single_step_param_2_footstep_list (const double goal_x, const double goal_y, const double goal_z, const double goal_theta,
                                                             const std::string& tmp_swing_leg,
                                                             const coordinates& _support_leg_coords)
  {
    leg_type _swing_leg = (tmp_swing_leg == ":rleg") ? WC_RLEG : WC_LLEG;
    step_node sn0((_swing_leg == WC_RLEG) ? WC_LLEG : WC_RLEG, _support_leg_coords);
    footstep_node_list.push_back(sn0);
    step_node sn1(_swing_leg, _support_leg_coords);
    hrp::Vector3 trs(2.0 * footstep_param.leg_default_translate_pos[(_swing_leg == WC_RLEG) ? 0 : 1] + hrp::Vector3(goal_x, goal_y, goal_z));
    sn1.worldcoords.translate(trs);
    sn1.worldcoords.rotate(deg2rad(goal_theta), hrp::Vector3(0,0,1));
    footstep_node_list.push_back(sn1);
    footstep_node_list.push_back(sn0);
  };

  void gait_generator::initialize_velocity_mode (const coordinates& _foot_midcoords,
						 const double vel_x, const double vel_y, const double vel_theta)
  {
    velocity_mode_flg = VEL_DOING;
    /* initialize */
    leg_type current_leg = (vel_y > 0.0) ? WC_RLEG : WC_LLEG;
    clear_footstep_node_list();
    set_velocity_param (vel_x, vel_y, vel_theta);
    append_go_pos_step_node(_foot_midcoords, current_leg);
    append_footstep_list_velocity_mode();
    append_footstep_list_velocity_mode();
    append_footstep_list_velocity_mode();
  };

  void gait_generator::finalize_velocity_mode ()
  {
    if (velocity_mode_flg == VEL_DOING) velocity_mode_flg = VEL_ENDING;
  };

  void gait_generator::calc_foot_midcoords_trans_vector_velocity_mode (coordinates& foot_midcoords, hrp::Vector3& trans, double& dth, const step_node& sn)
  {
    foot_midcoords = sn.worldcoords;
    hrp::Vector3 tmpv(footstep_param.leg_default_translate_pos[(sn.l_r == WC_RLEG) ? 0 : 1] * -1.0);
    foot_midcoords.translate(tmpv);
    double dx = vel_param.velocity_x + offset_vel_param.velocity_x, dy = vel_param.velocity_y + offset_vel_param.velocity_y;
    dth = vel_param.velocity_theta + offset_vel_param.velocity_theta;
    /* velocity limitation by stride parameters <- this should be based on footstep candidates */
    if (footstep_param.stride_x / default_step_time < fabs(dx))
      dx = footstep_param.stride_x * ((dx > 0.0) ? 1.0 : -1.0) / default_step_time;
    if (footstep_param.stride_y / default_step_time < fabs(dy))
      dy = footstep_param.stride_y * ((dy > 0.0) ? 1.0 : -1.0) / default_step_time;
    if (footstep_param.stride_theta / default_step_time < fabs(dth))
      dth = footstep_param.stride_theta * ((dth > 0.0) ? 1.0 : -1.0) / default_step_time;
    /* inside step limitation */
    if (use_inside_step_limitation) {
      if (vel_param.velocity_y > 0) {
        if (sn.l_r == WC_LLEG) dy *= 0.5;
      } else {
        if (sn.l_r == WC_RLEG) dy *= 0.5;
      }
      if (vel_param.velocity_theta > 0) {
        if (sn.l_r == WC_LLEG) dth *= 0.5;
      } else {
        if (sn.l_r == WC_RLEG) dth *= 0.5;
      }
    }
    trans = hrp::Vector3(dx * default_step_time, dy * default_step_time, 0);
    dth = deg2rad(dth * default_step_time);
  };

  void gait_generator::append_footstep_list_velocity_mode ()
  {
    coordinates foot_midcoords;
    hrp::Vector3 trans;
    double dth;
    calc_foot_midcoords_trans_vector_velocity_mode(foot_midcoords, trans, dth, footstep_node_list.back());

    foot_midcoords.translate(trans);
    foot_midcoords.rotate(dth, hrp::Vector3(0,0,1));
    append_go_pos_step_node(foot_midcoords, (( footstep_node_list.back().l_r == WC_RLEG ) ? WC_LLEG : WC_RLEG));
  };

  void gait_generator::calc_next_coords_velocity_mode (std::vector<coordinates>& ret, const size_t idx)
  {
    coordinates foot_midcoords;
    hrp::Vector3 trans;
    double dth;
    calc_foot_midcoords_trans_vector_velocity_mode(foot_midcoords, trans, dth, footstep_node_list[idx-1]);

    for (size_t i = 0; i < 3; i++) {
      ret.push_back(foot_midcoords);
      if ( velocity_mode_flg != VEL_ENDING ) {
        ret[i].translate(trans);
        ret[i].rotate(dth, hrp::Vector3(0,0,1));
      }
      ret[i].translate(footstep_param.leg_default_translate_pos[(footstep_node_list[idx-1].l_r == WC_RLEG) ? (1 + i)%2 : i%2]);
    }
  };

  void gait_generator::overwrite_refzmp_queue(const std::vector<coordinates>& cv)
  {
    /* clear footstep and refzmp after gp_index + 1, it means we do not modify current step */
    size_t idx = lcg.get_gp_index() + 1;
    /* reset index and counter */
    rg.set_indices(idx);
    rg.set_refzmp_count(one_step_len);

    /* add new next steps ;; the number of next steps is cv.size() */
    for (size_t i = 0; i < cv.size(); i++ ) {
      if ( footstep_node_list.size() - 1 >= idx + i) /* if footstep_node_list[idx] and footstep_node_list[idx+1]  exists */
        footstep_node_list[idx + i].worldcoords = cv[i];
      else
        footstep_node_list.push_back(step_node(footstep_node_list[lcg.get_gp_index()-1 + i].l_r, cv[i]));
    }

    /* remove steps after newly added steps */
    while ( footstep_node_list.size() > idx + cv.size())//
      footstep_node_list.pop_back();
    /* remove refzmp after idx for allocation of new refzmp by push_refzmp_from_footstep_list */
    rg.remove_refzmp_cur_list_over_length(idx);
    /* remove refzmp in preview contoroller queue */
    preview_controller_ptr->remove_preview_queue(lcg.get_gp_count());

    /* reset refzmp */
    for (size_t i = 0; i < cv.size()-1; i++) {
      if (emergency_flg == EMERGENCY_STOP)
        rg.push_refzmp_from_footstep_list_for_dual(footstep_node_list, cv[i%2], cv[(i+1)%2]);
      else
        rg.push_refzmp_from_footstep_list_for_single(footstep_node_list);
    }
    if (emergency_flg == EMERGENCY_STOP)
      rg.push_refzmp_from_footstep_list_for_dual(footstep_node_list, cv[0], cv[1]);
    /* fill preview controller queue by new refzmp */
    hrp::Vector3 rzmp;
    bool not_solved = true;
    while (not_solved) {
      bool refzmp_exist_p = rg.get_current_refzmp(rzmp, default_double_support_ratio, one_step_len);
      not_solved = !preview_controller_ptr->update(refzmp, cog, rzmp, refzmp_exist_p);
      rg.update_refzmp(footstep_node_list, one_step_len);
    }
  };
}

/* samples */
#ifdef HAVE_MAIN
using namespace rats;
#include <cstdio>

void test0 ()
{
  double dt = 0.005; /* [s] */
  std::vector<hrp::Vector3> leg_pos; /* default footstep transformations are necessary */
  leg_pos.push_back(hrp::Vector3(0,1e-3*-105,0)); /* :rleg */
  leg_pos.push_back(hrp::Vector3(0,1e-3* 105,0)); /* :lleg */
  gait_generator gg(dt, leg_pos, 1e-3*150, 1e-3*50, 10);

  /* this is c++ version example of test3, test6, test7 and test8 in euslib/demo/nozawa/motion/test-gait-generator.l */

  // std::cerr << "test3" << std::endl;
  // gg.go_pos_param_2_footstep_list(300, 100, 20, coordinates());
  // gg.print_footstep_list();

  // std::cerr << "test6" << std::endl;
  // gg.go_pos_param_2_footstep_list(500, 0, 0, coordinates());
  // gg.print_footstep_list();

  // std::cerr << "test7" << std::endl;
  // gg.go_pos_param_2_footstep_list(0, 500, 0, coordinates());
  // gg.print_footstep_list();

  // std::cerr << "test8" << std::endl;
  // gg.go_pos_param_2_footstep_list(0, 0, 90, coordinates());
  // gg.print_footstep_list();

  // {
  //   /* this is c++ version example of test5 in euslib/demo/nozawa/motion/test-gait-generator.l */
  //   /* default parameter */
  //   std::vector<hrp::Vector3> default_zmp_offsets;
  //   default_zmp_offsets.push_back(hrp::Vector3(0));
  //   default_zmp_offsets.push_back(hrp::Vector3(0));

  //   /* initialize gait_generator instance */
  //   robot_ptr rb(getenv("ROBOT"));
  //   rb->reset_pose();
  //   rb->fix_leg_to_coords(":both", coordinates());
  //   hrp::Vector3 cog(rb->calc_com());
  //   gg.set_default_zmp_offsets(default_zmp_offsets);
  //   gg.set_default_step_time(1.5);
  //   coordinates initial_support_leg_coords(hrp::Vector3(0, 105, 0)), initial_swing_leg_dst_coords(hrp::Vector3(0, -105, 0));
  //   /* initialize sample footstep_list */
  //   gg.clear_footstep_node_list();
  //   gg.append_footstep_node(":rleg", initial_swing_leg_dst_coords);
  //   gg.append_footstep_node(":lleg", initial_support_leg_coords);
  //   gg.append_footstep_node(":rleg", coordinates(hrp::Vector3(50, -105, 0)));
  //   gg.append_footstep_node(":lleg", coordinates(hrp::Vector3(100, 105, 0)));
  //   gg.append_footstep_node(":rleg", coordinates(hrp::Vector3(150, -105, 0)));
  //   gg.append_footstep_node(":lleg", coordinates(hrp::Vector3(200, 105, 0)));
  //   gg.append_footstep_node(":rleg", coordinates(hrp::Vector3(250, -105, 0)));
  //   gg.append_footstep_node(":lleg", coordinates(hrp::Vector3(250, 105, 0)));
  //   gg.initialize_gait_parameter(cog, initial_support_leg_coords, initial_swing_leg_dst_coords);
  //   while ( !gg.proc_one_tick(gait_generator::CYCLOID) );
  //   gg.print_footstep_list();

  //   /* make step and dump */
  //   size_t i = 0;
  //   std::string fname("/tmp/plot.dat");
  //   FILE* fp = fopen(fname.c_str(), "w");
  //   while ( gg.proc_one_tick(gait_generator::CYCLOID) ) {
  //     fprintf(fp, "%f ", i * dt);
  //     for (size_t ii = 0; ii < 3; ii++) {
  //       fprintf(fp, "%f %f %f %f ",
  //               gg.get_refzmp()(ii),
  //               gg.get_cog()(ii),
  //               gg.get_support_leg_coords().pos(ii),
  //               gg.get_swing_leg_coords().pos(ii));
  //     }
  //     fprintf(fp, "\n");
  //     i++;
  //   }
  //   fclose(fp);

  //   /* plot */
  //   FILE* gp[3];
  //   std::string titles[3] = {"X", "Y", "Z"};
  //   for (size_t ii = 0; ii < 3; ii++) {
  //     gp[ii] = popen("gnuplot", "w");
  //     fprintf(gp[ii], "set title \"%s\"\n", titles[ii].c_str());
  //     fprintf(gp[ii], "plot \"%s\" using 1:%d with lines title \"refzmp\"\n", fname.c_str(), ( ii * 4 + 2));
  //     fprintf(gp[ii], "replot \"%s\" using 1:%d with lines title \"cog\"\n", fname.c_str(), ( ii * 4 + 3));
  //     fprintf(gp[ii], "replot \"%s\" using 1:%d with lines title \"support\"\n", fname.c_str(), ( ii * 4 + 4));
  //     fprintf(gp[ii], "replot \"%s\" using 1:%d with lines title \"swing\"\n", fname.c_str(), ( ii * 4 + 5));
  //     fflush(gp[ii]);
  //   }
  //   double tmp;
  //   std::cin >> tmp;
  //   for (size_t j = 0; j < 3; j++) pclose(gp[j]);
  // }

  {
    /* this is update sample */
    /* default parameter */
    std::vector<hrp::Vector3> default_zmp_offsets;
    default_zmp_offsets.push_back(hrp::Vector3::Zero());
    default_zmp_offsets.push_back(hrp::Vector3::Zero());

    /* initialize gait_generator instance */
    //robot_ptr rb(getenv("ROBOT"));
    //rb->reset_pose();
    //rb->fix_leg_to_coords(":both", coordinates());
    //hrp::Vector3 cog(rb->calc_com());
    hrp::Vector3 cog(6.785, 1.54359, 806.831);// param for HRP2JSK reset-pose cog
    cog *= 1e-3;
    gg.set_default_zmp_offsets(default_zmp_offsets);
    gg.set_default_step_time(1.0);
    coordinates initial_support_leg_coords(hrp::Vector3(0, 1e-3*105, 0)), initial_swing_leg_dst_coords(hrp::Vector3(0, 1e-3*-105, 0));
    /* initialize sample footstep_list */
    gg.clear_footstep_node_list();
    gg.append_footstep_node(":rleg", initial_swing_leg_dst_coords);
    gg.append_footstep_node(":lleg", initial_support_leg_coords);
    gg.append_footstep_node(":rleg", coordinates(hrp::Vector3(50*1e-3, -105*1e-3, 0)));
    gg.append_footstep_node(":lleg", coordinates(hrp::Vector3(100*1e-3, 105*1e-3, 0)));
    gg.append_footstep_node(":rleg", coordinates(hrp::Vector3(150*1e-3, -105*1e-3, 0)));
    gg.append_footstep_node(":lleg", coordinates(hrp::Vector3(200*1e-3, 105*1e-3, 0)));
    gg.append_footstep_node(":rleg", coordinates(hrp::Vector3(250*1e-3, -105*1e-3, 0)));
    gg.append_footstep_node(":lleg", coordinates(hrp::Vector3(250*1e-3, 105*1e-3, 0)));
    gg.initialize_gait_parameter(cog, initial_support_leg_coords, initial_swing_leg_dst_coords);
    while ( !gg.proc_one_tick(gait_generator::CYCLOID) );
    gg.print_footstep_list();

    /* make step and dump */
    size_t i = 0;
    std::string fname("/tmp/plot.dat");
    FILE* fp = fopen(fname.c_str(), "w");
    while ( gg.proc_one_tick(gait_generator::CYCLOID) ) {
      //std::cerr << gg.lcg.gp_count << std::endl;
      // if ( gg.lcg.gp_index == 4 && gg.lcg.gp_count == 100) {
      //   //std::cerr << gg.lcg.gp_index << std::endl;
      //   gg.update_refzmp_queue(coordinates(hrp::Vector3(150, 105, 0)), coordinates(hrp::Vector3(150, -105, 0)));
      // }
      fprintf(fp, "%f ", i * dt);
      for (size_t ii = 0; ii < 3; ii++) {
        fprintf(fp, "%f %f %f %f ",
                gg.get_refzmp()(ii),
                gg.get_cog()(ii),
                gg.get_support_leg_coords().pos(ii),
                gg.get_swing_leg_coords().pos(ii));
      }
      fprintf(fp, "\n");
      i++;
    }
    fclose(fp);

    /* plot */
    FILE* gp[3];
    std::string titles[3] = {"X", "Y", "Z"};
    for (size_t ii = 0; ii < 3; ii++) {
      gp[ii] = popen("gnuplot", "w");
      fprintf(gp[ii], "set title \"%s\"\n", titles[ii].c_str());
      fprintf(gp[ii], "plot \"%s\" using 1:%d with lines title \"refzmp\"\n", fname.c_str(), ( ii * 4 + 2));
      fprintf(gp[ii], "replot \"%s\" using 1:%d with lines title \"cog\"\n", fname.c_str(), ( ii * 4 + 3));
      // fprintf(gp[ii], "replot \"%s\" using 1:%d with lines title \"support\"\n", fname.c_str(), ( ii * 4 + 4));
      // fprintf(gp[ii], "replot \"%s\" using 1:%d with lines title \"swing\"\n", fname.c_str(), ( ii * 4 + 5));
      fflush(gp[ii]);
    }
    double tmp;
    std::cin >> tmp;
    for (size_t j = 0; j < 3; j++) pclose(gp[j]);
  }
};

// // go_pos->footstep_list
// void test1 (const double xx, const double yy, const double th, const std::string start_leg = "",
//             const double stride_x = 150, const double stride_y = 50, const double stride_th = 10)
// {
//   robot_ptr rb(getenv("ROBOT"));
//   rb->reset_pose();
//   rb->fix_leg_to_coords(":both", coordinates());
//   double dt = 0.005; /* [s] */
//   std::vector<hrp::Vector3> leg_pos; /* default footstep transformations are necessary */
//   leg_pos.push_back(rb->get_end_coords(":rleg").pos); /* :rleg */
//   leg_pos.push_back(rb->get_end_coords(":lleg").pos); /* :lleg */
//   gait_generator gg(dt, leg_pos, stride_x, stride_y, stride_th); // for HRP2
//   if ( start_leg == "" ) {
//     gg.go_pos_param_2_footstep_list(xx, yy, th, coordinates());
//   } else {
//     gg.go_pos_param_2_footstep_list(xx, yy, th, coordinates(), (start_leg == ":rleg" ? gait_generator::WC_RLEG : gait_generator::WC_LLEG));
//   }
//   gg.print_footstep_list();
// }

// void test2 ()
// {
//   robot_ptr rb(getenv("ROBOT"));
//   rb->reset_pose();
//   rb->fix_leg_to_coords(":both", coordinates());
//   double dt = 0.005; /* [s] */
//   std::vector<hrp::Vector3> leg_pos; /* default footstep transformations are necessary */
//   leg_pos.push_back(rb->get_end_coords(":rleg").pos); /* :rleg */
//   leg_pos.push_back(rb->get_end_coords(":lleg").pos); /* :lleg */
//   gait_generator gg(dt, leg_pos, 150, 50, 10); // for HRP2
//   coordinates spc, swc;
//   std::string leg = ":rleg";
//   double goal_x = 0.15, goal_y = 0, goal_z = 0, goal_theta = 0;
//   rb->get_end_coords(spc, (leg == ":rleg") ? ":lleg" : ":rleg");
//   rb->get_end_coords(swc, (leg == ":rleg") ? ":rleg" : ":lleg");
//   gg.clear_footstep_node_list();
//   gg.go_single_step_param_2_footstep_list(goal_x * 1000.0, goal_y * 1000.0, goal_z * 1000.0, goal_theta,
//                                           leg, spc);
//   std::vector<hrp::Vector3> default_zmp_offsets;
//   default_zmp_offsets.push_back(hrp::Vector3(0));
//   default_zmp_offsets.push_back(hrp::Vector3(0));
//   gg.set_default_zmp_offsets(default_zmp_offsets);
//   gg.initialize_gait_parameter(rb->calc_com(), swc, spc);
//   std::vector<hrp::Vector3> cog_v, spc_v, swc_v;
//   while ( !gg.proc_one_tick(gait_generator::CYCLOID) );
//   while ( gg.proc_one_tick(gait_generator::CYCLOID) ) {
//     std::cerr << "(list :cog ";
//     print_vector(std::cerr, gg.get_cog(), false);
//     std::cerr << " " << gg.get_support_leg() << " ";
//     gg.get_support_leg_coords().print_eus_coordinates(std::cerr, false);
//     std::cerr << " " << gg.get_swing_leg() << " ";
//     gg.get_swing_leg_coords().print_eus_coordinates(std::cerr, false);
//     std::cerr << " )" << std::endl;
//   }
// }

int main(int argc, char* argv[])
{
  if (argc == 1) {
    test0();
  }//  else {
  //   if (argc == 4) test1(atof(argv[1]), atof(argv[2]), atof(argv[3]));
  //   else test1(atof(argv[1]), atof(argv[2]), atof(argv[3]), std::string(argv[4]));
  // }
  return 0;
}
#endif /* HAVE_MAIN */
