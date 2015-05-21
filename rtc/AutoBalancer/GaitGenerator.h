/* -*- mode:c++ -*- */
#ifndef GAITGENERATOR_H
#define GAITGENERATOR_H
#include "PreviewController.h"
#include "../ImpedanceController/RatsMatrix.h"
#include "interpolator.h"
#include <vector>
#include <queue>

namespace rats
{
  class gait_generator
  {

  public:
    enum orbit_type {SHUFFLING, CYCLOID, RECTANGLE, STAIR};
    enum leg_type {RLEG, LLEG, BOTH};

#ifndef HAVE_MAIN
  private:
#endif

    struct step_node
    {
      leg_type l_r;
      coordinates worldcoords;
      double step_height;
      step_node (const leg_type _l_r, const coordinates& _worldcoords, const double _step_height)
        : l_r(_l_r), worldcoords(_worldcoords), step_height(_step_height) {};
      step_node (const std::string& _l_r, const coordinates& _worldcoords, const double _step_height)
        : l_r((_l_r == "rleg") ? RLEG : LLEG), worldcoords(_worldcoords), step_height(_step_height) {};
    };
    friend std::ostream &operator<<(std::ostream &os, const step_node &sn)
    {
      os << "footstep" << std::endl;
      os << "  name = [" << ((sn.l_r==LLEG)?std::string("lleg"):std::string("rleg")) << "]" << std::endl;
      os << "  pos =" << std::endl;
      os << (sn.worldcoords.pos).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << std::endl;
      os << "  rot =" << std::endl;
      os << (sn.worldcoords.rot).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "    [", "]")) << std::endl;
      os << "  step_height = " << sn.step_height << "[m]" << std::endl;
      return os;
    };

    /* footstep parameter */
    struct footstep_parameter
    {
      /* translate pos is translate position of a leg from default foot_midcoords
       *   vector -> (list rleg-pos[mm] lleg-pos[mm] )
       */
      std::vector<hrp::Vector3> leg_default_translate_pos;
      /* stride params indicate max stride ( [mm], [mm], [deg] ) */
      double stride_fwd_x, stride_y, stride_theta, stride_bwd_x;
      footstep_parameter (const std::vector<hrp::Vector3>& _leg_pos,
                          const double _stride_fwd_x, const double _stride_y, const double _stride_theta, const double _stride_bwd_x)
        : leg_default_translate_pos(_leg_pos),
          stride_fwd_x(_stride_fwd_x), stride_y(_stride_y), stride_theta(_stride_theta), stride_bwd_x(_stride_bwd_x)  {};
    };

    /* velocity parameter for velocity mode */
    struct velocity_mode_parameter
    {
      /* velocity is [mm/s], [mm/s], [deg/s] */
      double velocity_x, velocity_y, velocity_theta;
      void set (const double _vx, const double _vy, const double _vth)
      {
        velocity_x = _vx;
        velocity_y = _vy;
        velocity_theta = _vth;
      };
      velocity_mode_parameter ()
	:velocity_x(0), velocity_y(0), velocity_theta(0) {};
    };

    /* Phase name of toe heel contact.
     *   SOLE0 : Sole contact (start). Foot angle is 0.
     *   SOLE2TOE : Transition of foot angle (0 -> toe_angle).
     *   TOE2SOLE : Transition of foot angle (toe_angle -> 0).
     *   SOLE1 : Foot_angle is 0.
     *   SOLE2HEEL : Transition of foot angle (0 -> -1 * heel_angle).
     *   HEEL2SOLE : Transition of foot angle (-1 * heel_angle -> 0).
     *   SOLE2 : Sole contact (end). Foot angle is 0.
     */
    enum toe_heel_phase {SOLE0, SOLE2TOE, TOE2SOLE, SOLE1, SOLE2HEEL, HEEL2SOLE, SOLE2, NUM_TH_PHASES};

    /* Manager for toe heel phase. */
    class toe_heel_phase_counter
    {
        double toe_heel_phase_ratio[NUM_TH_PHASES];
        size_t toe_heel_phase_count[NUM_TH_PHASES], total_count;
        bool calc_toe_heel_phase_count_from_raio ()
        {
            double ratio_sum = 0.0;
            for (size_t i = 0; i < NUM_TH_PHASES; i++) {
                ratio_sum += toe_heel_phase_ratio[i];
                toe_heel_phase_count[i] = static_cast<size_t>(total_count * ratio_sum);
            }
        };
    public:
        toe_heel_phase_counter () : total_count(0)
        {
            double ratio[NUM_TH_PHASES] = {0.05,0.25,0.2,0.0,0.2,0.25,0.05};
            set_toe_heel_phase_ratio(ratio);
        };
        // setter
        void set_total_count (const size_t _count)
        {
            total_count = _count;
            calc_toe_heel_phase_count_from_raio();
        };
        bool set_toe_heel_phase_ratio (const double* ratio)
        {
            for (size_t i = 0; i < NUM_TH_PHASES; i++) toe_heel_phase_ratio[i] = ratio[i];
        };
        // getter
        void get_toe_heel_phase_ratio (double* ratio)
        {
            for (size_t i = 0; i < NUM_TH_PHASES; i++) ratio[i] = toe_heel_phase_ratio[i];
        };
        int get_NUM_TH_PHASES () { return NUM_TH_PHASES; };
        // functions for checking phase and calculating phase time and ratio
        bool is_phase_starting (const size_t current_count, const toe_heel_phase _phase)
        {
            return (current_count == toe_heel_phase_count[_phase]);
        };
        bool is_between_phases (const size_t current_count, const toe_heel_phase phase0, const toe_heel_phase phase1)
        {
            return (toe_heel_phase_count[phase0] <= current_count) && (current_count < toe_heel_phase_count[phase1]);
        };
        bool is_between_phases (const size_t current_count, const toe_heel_phase phase1)
        {
            return (current_count < toe_heel_phase_count[phase1]);
        };
        bool is_no_SOLE1_phase () { return toe_heel_phase_count[TOE2SOLE] == toe_heel_phase_count[SOLE1]; };
        double calc_phase_period (const toe_heel_phase start_phase, const toe_heel_phase goal_phase, const double _dt)
        {
            return _dt * (toe_heel_phase_count[goal_phase]-toe_heel_phase_count[start_phase]);
        };
        double calc_phase_ratio (const size_t current_count, const toe_heel_phase start_phase, const toe_heel_phase goal_phase)
        {
            return static_cast<double>(current_count-toe_heel_phase_count[start_phase]) / (toe_heel_phase_count[goal_phase]-toe_heel_phase_count[start_phase]);
        };
        double calc_phase_ratio (const size_t current_count, const toe_heel_phase goal_phase)
        {
            return static_cast<double>(current_count) / (toe_heel_phase_count[goal_phase]);
        };
    };

    /* refzmp_generator to generate current refzmp from footstep_node_list */
    class refzmp_generator
    {
#ifdef HAVE_MAIN
    public:
#endif
      std::vector<hrp::Vector3> refzmp_cur_list;
      std::vector<hrp::Vector3> foot_x_axis_list; // Swing foot x axis list according to refzmp_cur_list
      std::vector<leg_type> swing_leg_list; // Swing leg list according to refzmp_cur_list
      std::vector<hrp::Vector3> default_zmp_offsets; /* list of RLEG and LLEG */
      size_t fs_index, refzmp_index, refzmp_count;
      double toe_zmp_offset_x, heel_zmp_offset_x; // [m]
      toe_heel_phase_counter* thp_ptr;
      bool use_toe_heel_transition, is_final_double_support_set;
      void calc_current_refzmp (hrp::Vector3& ret, hrp::Vector3& swing_foot_zmp_offset, const double default_double_support_ratio, const double default_double_support_static_ratio, const size_t one_step_len) const;
      const bool is_start_double_support_phase () const { return refzmp_index == 0; };
      const bool is_second_phase () const { return refzmp_index == 1; };
      const bool is_second_last_phase () const { return ((refzmp_index == refzmp_cur_list.size()-2) && is_final_double_support_set); };
      const bool is_end_double_support_phase () const { return refzmp_index == refzmp_cur_list.size() - 1; };
#ifndef HAVE_MAIN
    public:
#endif
      refzmp_generator(toe_heel_phase_counter* _thp_ptr)
        : refzmp_cur_list(), foot_x_axis_list(), swing_leg_list(), default_zmp_offsets(),
          fs_index(0), refzmp_index(0), refzmp_count(0),
          toe_zmp_offset_x(0), heel_zmp_offset_x(0),
          thp_ptr(_thp_ptr), use_toe_heel_transition(false), is_final_double_support_set(false)
      {
          default_zmp_offsets.push_back(hrp::Vector3::Zero());
          default_zmp_offsets.push_back(hrp::Vector3::Zero());
      };
      ~refzmp_generator()
      {
          thp_ptr = NULL;
      };
      void remove_refzmp_cur_list_over_length (const size_t len)
      {
        while ( refzmp_cur_list.size() > len) refzmp_cur_list.pop_back();
        while ( foot_x_axis_list.size() > len) foot_x_axis_list.pop_back();
        while ( swing_leg_list.size() > len) swing_leg_list.pop_back();
      };
      void reset (const size_t _refzmp_count)
      {
        set_indices(0);
        set_refzmp_count(_refzmp_count);
        refzmp_cur_list.clear();
        foot_x_axis_list.clear();
        swing_leg_list.clear();
        is_final_double_support_set = false;
      };
      void push_refzmp_from_footstep_list_for_dual (const std::vector<step_node>& fnl,
                                                    const coordinates& _support_leg_coords,
                                                    const coordinates& _swing_leg_coords);
      void push_refzmp_from_footstep_list_for_single (const std::vector<step_node>& fnl);
      void update_refzmp (const std::vector<step_node>& fnl, const size_t one_step_len);
      // setter
      void set_indices (const size_t idx) { fs_index = refzmp_index = idx; };
      void set_refzmp_count(const size_t _refzmp_count) { refzmp_count = _refzmp_count; };
      void set_default_zmp_offsets(const std::vector<hrp::Vector3>& tmp) { default_zmp_offsets = tmp; };
      void set_toe_zmp_offset_x (const double _off) { toe_zmp_offset_x = _off; };
      void set_heel_zmp_offset_x (const double _off) { heel_zmp_offset_x = _off; };
      void set_use_toe_heel_transition (const double _u) { use_toe_heel_transition = _u; };
      // getter
      bool get_current_refzmp (hrp::Vector3& rzmp, hrp::Vector3& swing_foot_zmp_offset, const double default_double_support_ratio, const double default_double_support_static_ratio, const size_t one_step_len) const
      {
        if (refzmp_cur_list.size() > refzmp_index ) calc_current_refzmp(rzmp, swing_foot_zmp_offset, default_double_support_ratio, default_double_support_static_ratio, one_step_len);
        return refzmp_cur_list.size() > refzmp_index;
      };
      const hrp::Vector3& get_refzmp_cur () { return refzmp_cur_list.front(); };
      const hrp::Vector3& get_default_zmp_offset (const leg_type lt) { return default_zmp_offsets[lt]; };
      double get_toe_zmp_offset_x () const { return toe_zmp_offset_x; };
      double get_heel_zmp_offset_x () const { return heel_zmp_offset_x; };
      bool get_use_toe_heel_transition () const { return use_toe_heel_transition; };
    };

    class delay_hoffarbib_trajectory_generator
    {
    private:
      hrp::Vector3 pos, vel, acc; // [m], [m/s], [m/s^2]
      double _dt; // [s]
      // Implement hoffarbib to configure remain_time;
      void hoffarbib_interpolation (const double tmp_remain_time, const hrp::Vector3& tmp_goal)
      {
        hrp::Vector3 jerk = (-9.0/ tmp_remain_time) * acc +
          (-36.0 / (tmp_remain_time * tmp_remain_time)) * vel +
          (60.0 / (tmp_remain_time * tmp_remain_time * tmp_remain_time)) * (tmp_goal - pos);
        acc = acc + _dt * jerk;
        vel = vel + _dt * acc;
        pos = pos + _dt * vel;
      };
    protected:
      double time_offset; // [s]
      double final_distance_weight;
      size_t total_count, current_count, double_support_count_half; // time/dt
      virtual hrp::Vector3 interpolate_antecedent_path (const hrp::Vector3& start, const hrp::Vector3& goal, const double height, const double tmp_ratio) = 0;
    public:
      delay_hoffarbib_trajectory_generator () : time_offset(0.35), final_distance_weight(1.0), total_count(0), current_count(0), double_support_count_half(0) {};
      ~delay_hoffarbib_trajectory_generator() { };
      void set_dt (const double __dt) { _dt = __dt; };
      void set_swing_trajectory_delay_time_offset (const double _time_offset) { time_offset = _time_offset; };
      void set_swing_trajectory_final_distance_weight (const double _final_distance_weight) { final_distance_weight = _final_distance_weight; };
      void reset (const size_t _one_step_len, const double default_double_support_ratio)
      {
        total_count = _one_step_len;
        current_count = 0;
        double_support_count_half = (default_double_support_ratio*total_count/2.0);
      };
      void get_trajectory_point (hrp::Vector3& ret, const hrp::Vector3& start, const hrp::Vector3& goal, const double height)
      {
        if ( double_support_count_half <= current_count && current_count < total_count - double_support_count_half ) { // swing phase
          size_t swing_remain_count = total_count - current_count - double_support_count_half;
          size_t swing_total_count = total_count - double_support_count_half*2;
          if (swing_remain_count*_dt > time_offset) { // antecedent path is still interpolating
            hoffarbib_interpolation (time_offset, interpolate_antecedent_path(start, goal, height, ((swing_total_count - swing_remain_count) / (swing_total_count - time_offset/_dt))));
          } else if (swing_remain_count > 0) { // antecedent path already reached to goal
            hoffarbib_interpolation (swing_remain_count*_dt, goal);
          } else {
            pos = goal;
          }
        } else if ( current_count < double_support_count_half ) { // first double support phase
          pos = start;
          vel = hrp::Vector3::Zero();
          acc = hrp::Vector3::Zero();
        } else { // last double support phase
          pos = goal;
          vel = hrp::Vector3::Zero();
          acc = hrp::Vector3::Zero();
        }
        ret = pos;
        current_count++;
      };
      double get_swing_trajectory_delay_time_offset () { return time_offset; };
      double get_swing_trajectory_final_distance_weight () { return final_distance_weight; };
      // interpolate path vector
      //   tmp_ratio : ratio value [0, 1]
      //   org_point_vec : vector of via points
      //   e.g., move tmp_ratio from 0 to 1 => move point from org_point_vec.front() to org_point_vec.back()
      hrp::Vector3 interpolate_antecedent_path_base (const double tmp_ratio, const std::vector<hrp::Vector3> org_point_vec)
      {
        std::vector<hrp::Vector3> point_vec;
        std::vector<double> distance_vec;
        double total_path_length = 0;
        point_vec.push_back(org_point_vec.front());
        // remove distance-zero points
        for (size_t i = 0; i < org_point_vec.size()-1; i++) {
          double tmp_distance = (org_point_vec[i+1]-org_point_vec[i]).norm();
          if (i==org_point_vec.size()-2) tmp_distance*=final_distance_weight;
          if ( tmp_distance > 1e-5 ) {
            point_vec.push_back(org_point_vec[i+1]);
            distance_vec.push_back(tmp_distance);
            total_path_length += tmp_distance;
          }
        }
        if ( total_path_length < 1e-5 ) { // if total path is zero, return goal point.
          return org_point_vec.back();
        }
        // point_vec        : [p0, p1, ..., pN-1, pN]
        // distance_vec     : [  d0, ...,     dN-1  ]
        // sum_distance_vec : [l0, l1, ..., lN-1, lN] <= lj = \Sum_{i=0}^{j-1} di
        std::vector<double> sum_distance_vec;
        sum_distance_vec.push_back(0);
        double tmp_dist = 0;
        for (size_t i = 0; i < distance_vec.size(); i++) {
          sum_distance_vec.push_back(tmp_dist + distance_vec[i]);
          tmp_dist += distance_vec[i];
        }
        // select current segment in which 'tmp_ratio' is included
        double current_length = tmp_ratio * total_path_length;
        for (size_t i = 0; i < sum_distance_vec.size(); i++) {
          if ( (sum_distance_vec[i] <= current_length) && (current_length <= sum_distance_vec[i+1]) ) {
            double tmpr = ((current_length - sum_distance_vec[i]) / distance_vec[i]);
            return ((1-tmpr) * point_vec[i] + tmpr * point_vec[1+i]);
          }
        }
        // if illegal tmp-ratio
        if (current_length < 0) return org_point_vec.front();
        else org_point_vec.back();
      };
    };

    class rectangle_delay_hoffarbib_trajectory_generator : public delay_hoffarbib_trajectory_generator
    {
      hrp::Vector3 interpolate_antecedent_path (const hrp::Vector3& start, const hrp::Vector3& goal, const double height, const double tmp_ratio)
      {
        std::vector<hrp::Vector3> rectangle_path;
        double max_height = std::max(start(2), goal(2))+height;
        rectangle_path.push_back(start);
        rectangle_path.push_back(hrp::Vector3(start(0), start(1), max_height));
        rectangle_path.push_back(hrp::Vector3(goal(0), goal(1), max_height));
        rectangle_path.push_back(goal);
        return interpolate_antecedent_path_base(tmp_ratio, rectangle_path);
      };
    };

    class stair_delay_hoffarbib_trajectory_generator : public delay_hoffarbib_trajectory_generator
    {
      hrp::Vector3 way_point_offset;
      hrp::Vector3 interpolate_antecedent_path (const hrp::Vector3& start, const hrp::Vector3& goal, const double height, const double tmp_ratio)
      {
        std::vector<hrp::Vector3> path;
        double max_height = std::max(start(2), goal(2))+height;
        hrp::Vector3 diff_vec = goal - start;
        diff_vec(2) = 0.0; // projection on horizontal plane
        path.push_back(start);
        // currently way_point_offset(1) is not used.
        //if ( diff_vec.norm() > 1e-4 && (goal(2) - start(2)) > way_point_offset(2) ) {
        if ( diff_vec.norm() > 1e-4 && (goal(2) - start(2)) > 0.02) {
          path.push_back(hrp::Vector3(start+-1*way_point_offset(0)*diff_vec.normalized()+hrp::Vector3(0,0,way_point_offset(2)+max_height-start(2))));
        }
        path.push_back(hrp::Vector3(start(0), start(1), max_height));
        path.push_back(hrp::Vector3(goal(0), goal(1), max_height));
        //if ( diff_vec.norm() > 1e-4 && (start(2) - goal(2)) > way_point_offset(2) ) {
        if ( diff_vec.norm() > 1e-4 && (start(2) - goal(2)) > 0.02) {
          path.push_back(hrp::Vector3(goal+way_point_offset(0)*diff_vec.normalized()+hrp::Vector3(0,0,way_point_offset(2)+max_height-goal(2))));
        }
        // if (height > 20 * 1e-3) {
        //   path.push_back(hrp::Vector3(goal(0), goal(1), 20*1e-3+goal(2)));
        // }
        path.push_back(goal);
        return interpolate_antecedent_path_base(tmp_ratio, path);
      };
    public:
      stair_delay_hoffarbib_trajectory_generator () : delay_hoffarbib_trajectory_generator(), way_point_offset(hrp::Vector3(0.03, 0.0, 0.0)) {};
      ~stair_delay_hoffarbib_trajectory_generator () {};
      void set_stair_trajectory_way_point_offset (const hrp::Vector3 _offset) { way_point_offset = _offset; };
      hrp::Vector3 get_stair_trajectory_way_point_offset() { return way_point_offset; };
    };

    /* leg_coords_generator to generate current swing_leg_coords and support_leg_coords from footstep_node_list */
    class leg_coords_generator
    {
#ifdef HAVE_MAIN
    public:
#endif
      coordinates swing_leg_dst_coords, support_leg_coords, swing_leg_coords, swing_leg_src_coords;
      double default_step_height, default_top_ratio, current_step_height, swing_ratio, rot_ratio, _dt, current_swing_time[2];
      size_t gp_index, gp_count, total_count;
      leg_type support_leg;
      orbit_type default_orbit_type;
      rectangle_delay_hoffarbib_trajectory_generator rdtg;
      stair_delay_hoffarbib_trajectory_generator sdtg;
      toe_heel_phase_counter* thp_ptr;
      interpolator* foot_ratio_interpolator;
      // Parameters for toe-heel contact
      interpolator* toe_heel_interpolator;
      double toe_pos_offset_x, heel_pos_offset_x, toe_angle, heel_angle, foot_dif_rot_angle;
      bool use_toe_joint;
      void calc_current_swing_leg_coords (coordinates& ret,
                                          const double ratio, const double step_height);
      double calc_interpolated_toe_heel_angle (const toe_heel_phase start_phase, const toe_heel_phase goal_phase, const double start, const double goal);
      void modif_foot_coords_for_toe_heel_phase (coordinates& org_coords);
      void cycloid_midcoords (coordinates& ret,
                              const double ratio, const coordinates& start,
                              const coordinates& goal, const double height) const;
      void cycloid_midpoint (hrp::Vector3& ret,
                             const double ratio, const hrp::Vector3& start,
                             const hrp::Vector3& goal, const double height) const;
      void rectangle_midcoords (coordinates& ret,
                                const double ratio, const coordinates& start,
                                const coordinates& goal, const double height);
      void stair_midcoords (coordinates& ret,
                            const double ratio, const coordinates& start,
                            const coordinates& goal, const double height);
      double calc_ratio_from_double_support_ratio (const double default_double_support_ratio, const size_t one_step_len);
#ifndef HAVE_MAIN
    public:
#endif
      leg_coords_generator(const double __dt, toe_heel_phase_counter* _thp_ptr)
        : swing_leg_dst_coords(), support_leg_coords(), swing_leg_coords(), swing_leg_src_coords(),
          default_step_height(0.05), default_top_ratio(0.5), current_step_height(0.0), swing_ratio(0), rot_ratio(0), _dt(__dt), gp_index(0), gp_count(0), support_leg(RLEG), default_orbit_type(CYCLOID),
          thp_ptr(_thp_ptr),
          foot_ratio_interpolator(NULL), toe_heel_interpolator(NULL),
          toe_pos_offset_x(0.0), heel_pos_offset_x(0.0), toe_angle(0.0), heel_angle(0.0), foot_dif_rot_angle(0.0), use_toe_joint(false)
      {
        rdtg.set_dt(_dt);
        sdtg.set_dt(_dt);
        if (foot_ratio_interpolator == NULL) foot_ratio_interpolator = new interpolator(1, __dt);
        //if (foot_ratio_interpolator == NULL) foot_ratio_interpolator = new interpolator(1, __dt, interpolator::LINEAR);
        if (toe_heel_interpolator == NULL) toe_heel_interpolator = new interpolator(1, __dt);
      };
      ~leg_coords_generator()
      {
        if (foot_ratio_interpolator != NULL) {
            delete foot_ratio_interpolator;
            foot_ratio_interpolator = NULL;
        }
        if (toe_heel_interpolator != NULL) {
            delete toe_heel_interpolator;
            toe_heel_interpolator = NULL;
        }
        thp_ptr = NULL;
      };
      void set_default_step_height (const double _tmp) { default_step_height = _tmp; };
      void set_default_top_ratio (const double _tmp) { default_top_ratio = _tmp; };
      void set_default_orbit_type (const orbit_type _tmp) { default_orbit_type = _tmp; };
      void set_swing_trajectory_delay_time_offset (const double _time_offset)
      {
        rdtg.set_swing_trajectory_delay_time_offset(_time_offset);
        sdtg.set_swing_trajectory_delay_time_offset(_time_offset);
      };
      void set_swing_trajectory_final_distance_weight (const double _final_distance_weight)
      {
        rdtg.set_swing_trajectory_final_distance_weight(_final_distance_weight);
        sdtg.set_swing_trajectory_final_distance_weight(_final_distance_weight);
      };
      void set_stair_trajectory_way_point_offset (const hrp::Vector3 _offset) { sdtg.set_stair_trajectory_way_point_offset(_offset); };
      void set_toe_pos_offset_x (const double _offx) { toe_pos_offset_x = _offx; };
      void set_heel_pos_offset_x (const double _offx) { heel_pos_offset_x = _offx; };
      void set_toe_angle (const double _angle) { toe_angle = _angle; };
      void set_heel_angle (const double _angle) { heel_angle = _angle; };
      void set_use_toe_joint (const bool ut) { use_toe_joint = ut; };
      void reset(const size_t one_step_len,
                 const coordinates& _swing_leg_dst_coords,
                 const coordinates& _swing_leg_src_coords,
                 const coordinates& _support_leg_coords,
                 const double default_double_support_ratio)
      {
        swing_leg_dst_coords = _swing_leg_dst_coords;
        swing_leg_src_coords = _swing_leg_src_coords;
        support_leg_coords = _support_leg_coords;
        total_count = gp_count = one_step_len;
        thp_ptr->set_total_count(total_count);
        gp_index = 0;
        current_step_height = 0.0;
        rdtg.reset(one_step_len, default_double_support_ratio);
        sdtg.reset(one_step_len, default_double_support_ratio);
        reset_foot_ratio_interpolator(one_step_len);
      };
      void reset_foot_ratio_interpolator (const size_t one_step_len)
      {
        double tmp_ratio = 0.0;
        foot_ratio_interpolator->clear();
        foot_ratio_interpolator->set(&tmp_ratio);
        tmp_ratio = 1.0;
        foot_ratio_interpolator->go(&tmp_ratio, _dt*one_step_len, true);
      };
      void update_leg_coords (const std::vector<step_node>& fnl, const double default_double_support_ratio, const size_t one_step_len, const bool force_height_zero);
      size_t get_gp_index() const { return gp_index; };
      size_t get_gp_count() const { return gp_count; };
      double get_current_swing_time(const size_t idx) const { return current_swing_time[idx]; };
      const coordinates& get_swing_leg_coords() const { return swing_leg_coords; };
      const coordinates& get_support_leg_coords() const { return support_leg_coords; };
      const coordinates& get_swing_leg_src_coords() const { return swing_leg_src_coords; };
      const coordinates& get_swing_leg_dst_coords() const { return swing_leg_dst_coords; };
      leg_type get_support_leg() const { return support_leg;};
      leg_type get_swing_leg() const { return support_leg == RLEG ? LLEG : RLEG;};
      double get_default_step_height () const { return default_step_height;};
      void get_swing_support_mid_coords(coordinates& ret) const
      {
        coordinates tmp;
	mid_coords(tmp, rot_ratio, swing_leg_src_coords, swing_leg_dst_coords);
        mid_coords(ret, 0.5, tmp, support_leg_coords);
      };
      leg_type get_current_support_state () const
      {
	if ( current_step_height > 0.0 ) {
	  if ( 0.0 < swing_ratio && swing_ratio < 1.0 ) {
	    if ( get_support_leg() == RLEG ) return RLEG;
	    else return LLEG;
	  } else {
	    return BOTH;
	  }
	} else {
	  return BOTH;
	}
      };
      orbit_type get_default_orbit_type () const { return default_orbit_type; };
      double get_swing_trajectory_delay_time_offset () { return rdtg.get_swing_trajectory_delay_time_offset(); };
      double get_swing_trajectory_final_distance_weight () { return rdtg.get_swing_trajectory_final_distance_weight(); };
      hrp::Vector3 get_stair_trajectory_way_point_offset () { return sdtg.get_stair_trajectory_way_point_offset(); };
      double get_toe_pos_offset_x () { return toe_pos_offset_x; };
      double get_heel_pos_offset_x () { return heel_pos_offset_x; };
      double get_toe_angle () { return toe_angle; };
      double get_heel_angle () { return heel_angle; };
      double get_foot_dif_rot_angle () { return foot_dif_rot_angle; };
      bool get_use_toe_joint () { return use_toe_joint; };
    };

    enum velocity_mode_flag { VEL_IDLING, VEL_DOING, VEL_ENDING };
    enum emergency_flag { IDLING, EMERGENCY_STOP, STOPPING };

    /* member variables for gait_generator */
    std::vector<step_node> footstep_node_list;
    toe_heel_phase_counter thp;
    refzmp_generator rg;
    leg_coords_generator lcg;
    footstep_parameter footstep_param;
    velocity_mode_parameter vel_param, offset_vel_param;
    hrp::Vector3 cog, refzmp, prev_que_rzmp, swing_foot_zmp_offset, prev_que_sfzo; /* cog by calculating proc_one_tick */
    double dt; /* control loop [s] */
    double default_step_time;
    double default_double_support_ratio, default_double_support_static_ratio;
    double gravitational_acceleration;
    size_t one_step_len, finalize_count;
    velocity_mode_flag velocity_mode_flg;
    emergency_flag emergency_flg;
    bool use_inside_step_limitation;

    /* preview controller parameters */
    //preview_dynamics_filter<preview_control>* preview_controller_ptr;
    preview_dynamics_filter<extended_preview_control>* preview_controller_ptr;

    void solve_angle_vector (const leg_type support_leg, const coordinates& support_leg_coords,
                             const coordinates& swing_leg_coords, const hrp::Vector3& cog);
    void append_go_pos_step_node (const coordinates& _foot_midcoords,
                                  const leg_type _l_r)
    {
      step_node sn(_l_r, _foot_midcoords, lcg.get_default_step_height());
      sn.worldcoords.pos += sn.worldcoords.rot * footstep_param.leg_default_translate_pos[_l_r];
      footstep_node_list.push_back(sn);
    };
    void overwrite_refzmp_queue(const std::vector<coordinates>& cv);
    void calc_foot_midcoords_trans_vector_velocity_mode (coordinates& foot_midcoords, hrp::Vector3& trans, double& dth, const step_node& sn);
    void calc_next_coords_velocity_mode (std::vector<coordinates>& ret, const size_t idx);
    void append_footstep_list_velocity_mode ();

#ifndef HAVE_MAIN
    /* inhibit copy constructor and copy insertion not by implementing */
    gait_generator (const gait_generator& _p);
    gait_generator &operator=(const gait_generator &_p);
  public:
#endif
    gait_generator (double _dt,
                    /* arguments for footstep_parameter */
                    const std::vector<hrp::Vector3>& _leg_pos,
                    const double _stride_fwd_x, const double _stride_y, const double _stride_theta, const double _stride_bwd_x)
      : footstep_node_list(), thp(), rg(&thp), lcg(_dt, &thp),
        footstep_param(_leg_pos, _stride_fwd_x, _stride_y, _stride_theta, _stride_bwd_x),
        vel_param(), offset_vel_param(), cog(hrp::Vector3::Zero()), refzmp(hrp::Vector3::Zero()), prev_que_rzmp(hrp::Vector3::Zero()),
        swing_foot_zmp_offset(hrp::Vector3::Zero()), prev_que_sfzo(hrp::Vector3::Zero()),
        dt(_dt), default_step_time(1.0), default_double_support_ratio(0.2), default_double_support_static_ratio(0.0), gravitational_acceleration(DEFAULT_GRAVITATIONAL_ACCELERATION),
        one_step_len(default_step_time / dt), finalize_count(0),
        velocity_mode_flg(VEL_IDLING), emergency_flg(IDLING),
        use_inside_step_limitation(true),
        preview_controller_ptr(NULL) {};
    ~gait_generator () {
      if ( preview_controller_ptr != NULL ) {
        delete preview_controller_ptr;
        preview_controller_ptr = NULL;
      }
    };
    void initialize_gait_parameter (const hrp::Vector3& cog,
                                    const coordinates& initial_support_leg_coords,
                                    const coordinates& initial_swing_leg_dst_coords,
                                    const double delay = 1.6);
    bool proc_one_tick ();
    void append_footstep_node (const std::string& _leg, const coordinates& _fs)
    {
        footstep_node_list.push_back(step_node(_leg, _fs, lcg.get_default_step_height()));
    };
    void append_footstep_node (const std::string& _leg, const coordinates& _fs, const double _step_height)
    {
        footstep_node_list.push_back(step_node(_leg, _fs, _step_height));
    };
    void clear_footstep_node_list () { footstep_node_list.clear(); };
    void go_pos_param_2_footstep_list (const double goal_x, const double goal_y, const double goal_theta, /* [mm] [mm] [deg] */
                                       const coordinates& _foot_midcoords) {
      go_pos_param_2_footstep_list(goal_x, goal_y, goal_theta,
                                   _foot_midcoords, (goal_y > 0.0 ? RLEG : LLEG));
    }
    void go_pos_param_2_footstep_list (const double goal_x, const double goal_y, const double goal_theta, /* [mm] [mm] [deg] */
                                       const coordinates& _foot_midcoords, const leg_type start_leg);
    void go_single_step_param_2_footstep_list (const double goal_x, const double goal_y, const double goal_z, const double goal_theta, /* [mm] [mm] [mm] [deg] */
                                               const std::string& tmp_swing_leg,
                                               const coordinates& _support_leg_coords);
    void initialize_velocity_mode (const coordinates& _foot_midcoords,
				   const double vel_x, const double vel_y, const double vel_theta); /* [mm/s] [mm/s] [deg/s] */
    void finalize_velocity_mode ();
    void append_finalize_footstep ()
    {
      footstep_node_list.push_back(footstep_node_list[footstep_node_list.size()-2]);
    };
    void emergency_stop ()
    {
      if (!footstep_node_list.empty()) {
        velocity_mode_flg = VEL_IDLING;
        emergency_flg = EMERGENCY_STOP;
      }
    };
    /* parameter setting */
    void set_default_step_time (const double _default_step_time) { default_step_time = _default_step_time; };
    void set_default_double_support_ratio (const double _default_double_support_ratio) { default_double_support_ratio = _default_double_support_ratio; };
    void set_default_double_support_static_ratio (const double _default_double_support_static_ratio) { default_double_support_static_ratio = _default_double_support_static_ratio; };
    void set_default_zmp_offsets(const std::vector<hrp::Vector3>& tmp) { rg.set_default_zmp_offsets(tmp); };
    void set_toe_zmp_offset_x (const double _off) { rg.set_toe_zmp_offset_x(_off); };
    void set_heel_zmp_offset_x (const double _off) { rg.set_heel_zmp_offset_x(_off); };
    void set_use_toe_heel_transition (const double _u) { rg.set_use_toe_heel_transition(_u); };
    void set_default_step_height(const double _tmp) { lcg.set_default_step_height(_tmp); };
    void set_default_top_ratio(const double _tmp) { lcg.set_default_top_ratio(_tmp); };
    void set_velocity_param (const double vel_x, const double vel_y, const double vel_theta) /* [mm/s] [mm/s] [deg/s] */
    {
      vel_param.set(vel_x, vel_y, vel_theta);
    };
    void set_offset_velocity_param (const double vel_x, const double vel_y, const double vel_theta) /* [mm/s] [mm/s] [deg/s] */
    {
      offset_vel_param.set(vel_x, vel_y, vel_theta);
    };
    void set_stride_parameters (const double _stride_fwd_x, const double _stride_y, const double _stride_theta, const double _stride_bwd_x)
    {
      footstep_param.stride_fwd_x = _stride_fwd_x;
      footstep_param.stride_y = _stride_y;
      footstep_param.stride_theta = _stride_theta;
      footstep_param.stride_bwd_x = _stride_bwd_x;
    };
    void set_use_inside_step_limitation(const bool uu) { use_inside_step_limitation = uu; };
    void set_default_orbit_type (const orbit_type type) { lcg.set_default_orbit_type(type); };
    void set_swing_trajectory_delay_time_offset (const double _time_offset) { lcg.set_swing_trajectory_delay_time_offset(_time_offset); };
    void set_swing_trajectory_final_distance_weight (const double _final_distance_weight) { lcg.set_swing_trajectory_final_distance_weight(_final_distance_weight); };
    void set_stair_trajectory_way_point_offset (const hrp::Vector3 _offset) { lcg.set_stair_trajectory_way_point_offset(_offset); };
    void set_gravitational_acceleration (const double ga) { gravitational_acceleration = ga; };
    void set_toe_pos_offset_x (const double _offx) { lcg.set_toe_pos_offset_x(_offx); };
    void set_heel_pos_offset_x (const double _offx) { lcg.set_heel_pos_offset_x(_offx); };
    void set_toe_angle (const double _angle) { lcg.set_toe_angle(_angle); };
    void set_heel_angle (const double _angle) { lcg.set_heel_angle(_angle); };
    void set_toe_heel_phase_ratio (const double* ratio) { thp.set_toe_heel_phase_ratio(ratio); };
    void set_use_toe_joint (const bool ut) { lcg.set_use_toe_joint(ut); };
    void print_footstep_list () const
    {
      for (size_t i = 0; i < footstep_node_list.size(); i++)
        std::cerr << footstep_node_list[i] << std::endl;
    };
    /* parameter getting */
    const hrp::Vector3& get_cog () { return cog; };
    const hrp::Vector3& get_refzmp () { return refzmp;};
    hrp::Vector3 get_cart_zmp ()
    {
        double czmp[3];
        preview_controller_ptr->get_cart_zmp(czmp);
        return hrp::Vector3(czmp[0], czmp[1], czmp[2]);
    };
    const hrp::Vector3& get_swing_foot_zmp_offset () { return swing_foot_zmp_offset;};
    const hrp::Vector3& get_support_foot_zmp_offset () { return rg.get_default_zmp_offset(lcg.get_support_leg());};
    double get_toe_zmp_offset_x () const { return rg.get_toe_zmp_offset_x(); };
    double get_heel_zmp_offset_x () const { return rg.get_heel_zmp_offset_x(); };
    bool get_use_toe_heel_transition () const { return rg.get_use_toe_heel_transition(); };
    const std::string get_footstep_front_leg () const { return footstep_node_list[0].l_r == RLEG ? "rleg" : "lleg"; };
    const std::string get_footstep_back_leg () const { return footstep_node_list.back().l_r == RLEG ? "rleg" : "lleg"; };
    const std::string get_support_leg() const { return lcg.get_support_leg() == RLEG ? "rleg" : "lleg";};
    const std::string get_swing_leg() const { return lcg.get_swing_leg() == RLEG ? "rleg" : "lleg";};
    const coordinates& get_swing_leg_coords() const { return lcg.get_swing_leg_coords(); };
    const coordinates& get_support_leg_coords() const { return lcg.get_support_leg_coords(); };
    const coordinates& get_swing_leg_src_coords() const { return lcg.get_swing_leg_src_coords(); };
    const coordinates& get_swing_leg_dst_coords() const { return lcg.get_swing_leg_dst_coords(); };
    const coordinates get_dst_foot_midcoords() const /* get foot_midcoords calculated from swing_leg_dst_coords */
    {
      coordinates tmp(lcg.get_swing_leg_dst_coords());
      tmp.pos += tmp.rot * hrp::Vector3(-1*footstep_param.leg_default_translate_pos[lcg.get_swing_leg()]);
      return tmp;
    };
    void get_swing_support_mid_coords(coordinates& ret) const { lcg.get_swing_support_mid_coords(ret); };
    void get_stride_parameters (double& _stride_fwd_x, double& _stride_y, double& _stride_theta, double& _stride_bwd_x)
    {
      _stride_fwd_x = footstep_param.stride_fwd_x;
      _stride_y = footstep_param.stride_y;
      _stride_theta = footstep_param.stride_theta;
      _stride_bwd_x = footstep_param.stride_bwd_x;
    };
    size_t get_gp_index() const { return lcg.get_gp_index(); };
    size_t get_gp_count() const { return lcg.get_gp_count(); };
    double get_current_swing_time(const size_t idx) const { return lcg.get_current_swing_time(idx); };
    size_t get_current_support_state() const { return lcg.get_current_support_state();};
    double get_default_step_time () const { return default_step_time; };
    double get_default_step_height () const { return lcg.get_default_step_height(); };
    double get_default_double_support_ratio () const { return default_double_support_ratio; };
    double get_default_double_support_static_ratio () const { return default_double_support_static_ratio; };
    /* return whether _leg is swinging leg or not
     * swinging leg -> swing_leg and not double support phase
     *                 landing_offset_ratio is mergin from double support period
     */
    bool is_swinging_leg (const std::string& _leg, const double landing_offset_ratio = 0.08) const
    {
      if ( _leg == get_swing_leg() &&
	   lcg.get_gp_count() <= static_cast<size_t>( ( 1.0 - default_double_support_ratio - landing_offset_ratio) * one_step_len) &&
	   lcg.get_gp_count() >= static_cast<size_t>( (default_double_support_ratio + landing_offset_ratio) * one_step_len) )
	return true;
      else return false;
    };
    orbit_type get_default_orbit_type () const { return lcg.get_default_orbit_type(); };
    double get_swing_trajectory_delay_time_offset () { return lcg.get_swing_trajectory_delay_time_offset(); };
    double get_swing_trajectory_final_distance_weight () { return lcg.get_swing_trajectory_final_distance_weight(); };
    hrp::Vector3 get_stair_trajectory_way_point_offset () { return lcg.get_stair_trajectory_way_point_offset(); };
    double get_gravitational_acceleration () { return gravitational_acceleration; } ;
    double get_toe_pos_offset_x () { return lcg.get_toe_pos_offset_x(); };
    double get_heel_pos_offset_x () { return lcg.get_heel_pos_offset_x(); };
    double get_toe_angle () { return lcg.get_toe_angle(); };
    double get_heel_angle () { return lcg.get_heel_angle(); };
    double get_foot_dif_rot_angle () { return lcg.get_foot_dif_rot_angle(); };
    void get_toe_heel_phase_ratio (double* ratio) { thp.get_toe_heel_phase_ratio(ratio); };
    int get_NUM_TH_PHASES () { return thp.get_NUM_TH_PHASES(); };
    bool get_use_toe_joint () { return lcg.get_use_toe_joint(); };
    void print_param (const std::string& print_str = "")
    {
        double stride_fwd_x, stride_y, stride_th, stride_bwd_x;
        get_stride_parameters(stride_fwd_x, stride_y, stride_th, stride_bwd_x);
        std::cerr << "[" << print_str << "]   stride_parameter = " << stride_fwd_x << "[m], " << stride_y << "[m], " << stride_th << "[deg], " << stride_bwd_x << "[m]" << std::endl;
        std::cerr << "[" << print_str << "]   default_step_time = " << get_default_step_time() << "[s]" << std::endl;
        std::cerr << "[" << print_str << "]   default_step_height = " << get_default_step_height() << "[m]" << std::endl;
        std::cerr << "[" << print_str << "]   default_double_support_ratio = " << get_default_double_support_ratio() << ", default_double_support_static_ratio = " << get_default_double_support_static_ratio() << std::endl;
        std::cerr << "[" << print_str << "]   default_orbit_type = ";
        if (get_default_orbit_type() == gait_generator::SHUFFLING) {
            std::cerr << "SHUFFLING" << std::endl;
        } else if (get_default_orbit_type() == gait_generator::CYCLOID) {
            std::cerr << "CYCLOID" << std::endl;
        } else if (get_default_orbit_type() == gait_generator::RECTANGLE) {
            std::cerr << "RECTANGLE" << std::endl;
        } else if (get_default_orbit_type() == gait_generator::STAIR) {
            std::cerr << "STAIR" << std::endl;
        }
        std::cerr << "[" << print_str << "]   swing_trajectory_delay_time_offset = " << get_swing_trajectory_delay_time_offset() << "[s], swing_trajectory_final_distance_weight = " << get_swing_trajectory_final_distance_weight() << std::endl;
        hrp::Vector3 tmpv;
        tmpv = get_stair_trajectory_way_point_offset();
        std::cerr << "[" << print_str << "]   stair_trajectory_way_point_offset = " << tmpv.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[m]" << std::endl;
        std::cerr << "[" << print_str << "]   gravitational_acceleration = " << get_gravitational_acceleration() << "[m/s^2]" << std::endl;
        std::cerr << "[" << print_str << "]   toe_pos_offset_x = " << get_toe_pos_offset_x() << "[mm], heel_pos_offset_x = " << get_heel_pos_offset_x() << "[mm]" << std::endl;
        std::cerr << "[" << print_str << "]   toe_zmp_offset_x = " << get_toe_zmp_offset_x() << "[mm], heel_zmp_offset_x = " << get_heel_zmp_offset_x() << "[mm]" << std::endl;
        std::cerr << "[" << print_str << "]   toe_angle = " << get_toe_angle() << "[deg]" << std::endl;
        std::cerr << "[" << print_str << "]   heel_angle = " << get_heel_angle() << "[deg]" << std::endl;
        std::cerr << "[" << print_str << "]   use_toe_joint = " << (get_use_toe_joint()?"true":"false") << ", use_toe_heel_transition = " << (get_use_toe_heel_transition()?"true":"false") << std::endl;
    };
  };
}
#endif /* GAITGENERATOR_H */

