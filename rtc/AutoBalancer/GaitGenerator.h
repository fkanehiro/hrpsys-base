/* -*- mode:c++ -*- */
#ifndef GAITGENERATOR_H
#define GAITGENERATOR_H
#include "PreviewController.h"
#include "../ImpedanceController/RatsMatrix.h"
#include <vector>
#include <queue>

namespace rats
{
  class gait_generator
  {

  public:
    enum orbit_type {SHUFFLING, CYCLOID, RECTANGLE, STAIR};

#ifndef HAVE_MAIN
  private:
#endif
    enum leg_type {WC_LLEG, WC_RLEG};

    struct step_node
    {
      leg_type l_r;
      coordinates worldcoords;
      step_node (const leg_type _l_r, const coordinates& _worldcoords)
        : l_r(_l_r), worldcoords(_worldcoords) {};
      step_node (const std::string& _l_r, const coordinates& _worldcoords)
        : l_r((_l_r == "rleg") ? WC_RLEG : WC_LLEG), worldcoords(_worldcoords) {};
    };
    friend std::ostream &operator<<(std::ostream &os, const step_node &sn)
    {
      os << "footstep" << std::endl;
      os << "  name = [" << ((sn.l_r==WC_LLEG)?std::string("lleg"):std::string("rleg")) << "]" << std::endl;
      os << "  pos =" << std::endl;
      os << (sn.worldcoords.pos).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << std::endl;
      os << "  rot =" << std::endl;
      os << (sn.worldcoords.rot).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "    [", "]")) << std::endl;
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

    /* refzmp_generator to generate current refzmp from footstep_node_list */
    class refzmp_generator
    {
#ifdef HAVE_MAIN
    public:
#endif
      std::vector<hrp::Vector3> refzmp_cur_list;
      std::vector<hrp::Vector3> default_zmp_offsets; /* (list rleg lleg) */
      size_t fs_index, refzmp_index, refzmp_count;
      void calc_current_refzmp (hrp::Vector3& ret, const double default_double_support_ratio, const size_t one_step_len) const;
#ifndef HAVE_MAIN
    public:
#endif
      refzmp_generator()
        : refzmp_cur_list(), default_zmp_offsets(),
          fs_index(0), refzmp_index(0), refzmp_count(0) {};
      ~refzmp_generator() {};
      /*  */
      void remove_refzmp_cur_list_over_length (const size_t len)
      {
        while ( refzmp_cur_list.size() > len) refzmp_cur_list.pop_back();
      };
      void set_indices (const size_t idx) { fs_index = refzmp_index = idx; };
      void set_refzmp_count(const size_t _refzmp_count) { refzmp_count = _refzmp_count; };
      void set_default_zmp_offsets(const std::vector<hrp::Vector3>& tmp) { default_zmp_offsets = tmp; };
      void reset (const size_t _refzmp_count)
      {
        set_indices(0);
        set_refzmp_count(_refzmp_count);
        refzmp_cur_list.clear();
      };
      void push_refzmp_from_footstep_list_for_dual (const std::vector<step_node>& fnl,
                                                    const coordinates& _support_leg_coords,
                                                    const coordinates& _swing_leg_coords);
      void push_refzmp_from_footstep_list_for_single (const std::vector<step_node>& fnl);
      void update_refzmp (const std::vector<step_node>& fnl, const size_t one_step_len);
      bool get_current_refzmp (hrp::Vector3& rzmp, const double default_double_support_ratio, const size_t one_step_len) const
      {
        if (refzmp_cur_list.size() > refzmp_index ) calc_current_refzmp(rzmp, default_double_support_ratio, one_step_len);
        return refzmp_cur_list.size() > refzmp_index;
      };
      const hrp::Vector3& get_refzmp_cur () { return refzmp_cur_list.front(); };
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
      double total_time, time_offset, current_time, double_support_time_half; // [s]
      virtual hrp::Vector3 interpolate_antecedent_path (const hrp::Vector3& start, const hrp::Vector3& goal, const double height, const double tmp_ratio) = 0;
    public:
      delay_hoffarbib_trajectory_generator () : total_time(0), time_offset(0.2), current_time(0) {};
      ~delay_hoffarbib_trajectory_generator() { };
      void set_dt (const double __dt) { _dt = __dt; };
      void set_swing_trajectory_delay_time_offset (const double _time_offset) { time_offset = _time_offset; };
      void reset (const size_t _one_step_len, const double default_double_support_ratio)
      {
        total_time = _one_step_len * _dt;
        current_time = 0;
        double_support_time_half = default_double_support_ratio*total_time/2.0;
      };
      void get_trajectory_point (hrp::Vector3& ret, const hrp::Vector3& start, const hrp::Vector3& goal, const double height)
      {
        if ( double_support_time_half <= current_time && current_time <= total_time - double_support_time_half ) { // swing phase
          double swing_remain_time = total_time - current_time - double_support_time_half;
          double swing_total_time = total_time - double_support_time_half*2;
          if (swing_remain_time > time_offset) { // antecedent path is still interpolating
            hoffarbib_interpolation (time_offset, interpolate_antecedent_path(start, goal, height, ((swing_total_time - swing_remain_time) / (swing_total_time - time_offset))));
          } else if (swing_remain_time > 1e-5) { // antecedent path already reached to goal
            hoffarbib_interpolation (swing_remain_time, goal);
          } else {
            pos = goal;
          }
        } else if ( current_time < double_support_time_half ) { // first double support phase
          pos = start;
          vel = hrp::Vector3::Zero();
          acc = hrp::Vector3::Zero();
        } else { // last double support phase
          pos = goal;
          vel = hrp::Vector3::Zero();
          acc = hrp::Vector3::Zero();
        }
        ret = pos;
        current_time += _dt;
      };
      double get_swing_trajectory_delay_time_offset () { return time_offset; };
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
        if (height > 20 * 1e-3) {
          path.push_back(hrp::Vector3(goal(0), goal(1), 20*1e-3+goal(2)));
        }
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
      size_t gp_index, gp_count;
      leg_type support_leg;
      orbit_type default_orbit_type;
      rectangle_delay_hoffarbib_trajectory_generator rdtg;
      stair_delay_hoffarbib_trajectory_generator sdtg;
      void calc_current_swing_leg_coords (coordinates& ret,
                                          const double ratio, const double step_height);
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
      leg_coords_generator(const double __dt)
        : swing_leg_dst_coords(), support_leg_coords(), swing_leg_coords(), swing_leg_src_coords(),
          default_step_height(0.05), default_top_ratio(0.5), current_step_height(0.0), swing_ratio(0), rot_ratio(0), _dt(__dt), gp_index(0), gp_count(0), support_leg(WC_RLEG), default_orbit_type(CYCLOID)
      {
        rdtg.set_dt(_dt);
        sdtg.set_dt(_dt);
      };
      ~leg_coords_generator() {};
      void set_default_step_height (const double _tmp) { default_step_height = _tmp; };
      void set_default_top_ratio (const double _tmp) { default_top_ratio = _tmp; };
      void set_default_orbit_type (const orbit_type _tmp) { default_orbit_type = _tmp; };
      void set_swing_trajectory_delay_time_offset (const double _time_offset)
      {
        rdtg.set_swing_trajectory_delay_time_offset(_time_offset);
        sdtg.set_swing_trajectory_delay_time_offset(_time_offset);
      };
      void set_stair_trajectory_way_point_offset (const hrp::Vector3 _offset) { sdtg.set_stair_trajectory_way_point_offset(_offset); };
      void reset(const size_t one_step_len,
                 const coordinates& _swing_leg_dst_coords,
                 const coordinates& _swing_leg_src_coords,
                 const coordinates& _support_leg_coords,
                 const double default_double_support_ratio)
      {
        swing_leg_dst_coords = _swing_leg_dst_coords;
        swing_leg_src_coords = _swing_leg_src_coords;
        support_leg_coords = _support_leg_coords;
        gp_count = one_step_len;
        gp_index = 0;
        current_step_height = 0.0;
        rdtg.reset(one_step_len, default_double_support_ratio);
        sdtg.reset(one_step_len, default_double_support_ratio);
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
      double get_default_step_height () const { return default_step_height;};
      void get_swing_support_mid_coords(coordinates& ret) const
      {
        coordinates tmp;
	mid_coords(tmp, rot_ratio, swing_leg_src_coords, swing_leg_dst_coords);
        mid_coords(ret, 0.5, tmp, support_leg_coords);
      };
      size_t get_current_support_state () const
      {
	if ( current_step_height > 0.0 ) {
	  if ( 0.0 < swing_ratio && swing_ratio < 1.0 ) {
	    if ( get_support_leg() == WC_RLEG ) return 1; // rleg
	    else return 2;
	  } else {
	    return 0;
	  }
	} else {
	  return 0;
	}
      };
      orbit_type get_default_orbit_type () const { return default_orbit_type; };
      double get_swing_trajectory_delay_time_offset () { return rdtg.get_swing_trajectory_delay_time_offset(); };
      hrp::Vector3 get_stair_trajectory_way_point_offset () { return sdtg.get_stair_trajectory_way_point_offset(); };
    };

    enum velocity_mode_flag { VEL_IDLING, VEL_DOING, VEL_ENDING };
    enum emergency_flag { IDLING, EMERGENCY_STOP, STOPPING };

    /* member variables for gait_generator */
    std::vector<step_node> footstep_node_list;
    refzmp_generator rg;
    leg_coords_generator lcg;
    footstep_parameter footstep_param;
    velocity_mode_parameter vel_param, offset_vel_param;
    hrp::Vector3 cog, refzmp, prev_que_rzmp; /* cog by calculating proc_one_tick */
    double dt; /* control loop [s] */
    double default_step_time;
    double default_double_support_ratio;
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
      step_node sn(_l_r, _foot_midcoords);
      sn.worldcoords.pos += sn.worldcoords.rot * footstep_param.leg_default_translate_pos[(_l_r == WC_RLEG) ? 0 : 1];
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
      : footstep_node_list(), rg(), lcg(_dt),
        footstep_param(_leg_pos, _stride_fwd_x, _stride_y, _stride_theta, _stride_bwd_x),
        vel_param(), offset_vel_param(), cog(hrp::Vector3::Zero()), refzmp(hrp::Vector3::Zero()), prev_que_rzmp(hrp::Vector3::Zero()),
        dt(_dt), default_step_time(1.0), default_double_support_ratio(0.2),
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
      footstep_node_list.push_back(step_node((_leg == "rleg") ? WC_RLEG : WC_LLEG, _fs));
    };
    void clear_footstep_node_list () { footstep_node_list.clear(); };
    void go_pos_param_2_footstep_list (const double goal_x, const double goal_y, const double goal_theta, /* [mm] [mm] [deg] */
                                       const coordinates& _foot_midcoords) {
      go_pos_param_2_footstep_list(goal_x, goal_y, goal_theta,
                                   _foot_midcoords, (goal_y > 0.0 ? WC_RLEG : WC_LLEG));
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
    void set_default_zmp_offsets(const std::vector<hrp::Vector3>& tmp) { rg.set_default_zmp_offsets(tmp); };
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
    void set_stair_trajectory_way_point_offset (const hrp::Vector3 _offset) { lcg.set_stair_trajectory_way_point_offset(_offset); };
    void print_footstep_list () const
    {
      for (size_t i = 0; i < footstep_node_list.size(); i++)
        std::cerr << footstep_node_list[i] << std::endl;
    };
    /* parameter getting */
    const hrp::Vector3& get_cog () { return cog; };
    const hrp::Vector3& get_refzmp () { return refzmp;};
    const std::string get_footstep_front_leg () const { return footstep_node_list[0].l_r == WC_RLEG ? "rleg" : "lleg"; };
    const std::string get_footstep_back_leg () const { return footstep_node_list.back().l_r == WC_RLEG ? "rleg" : "lleg"; };
    const std::string get_support_leg() const { return lcg.get_support_leg() == WC_RLEG ? "rleg" : "lleg";};
    const std::string get_swing_leg() const { return lcg.get_support_leg() == WC_RLEG ? "lleg" : "rleg";};
    const coordinates& get_swing_leg_coords() const { return lcg.get_swing_leg_coords(); };
    const coordinates& get_support_leg_coords() const { return lcg.get_support_leg_coords(); };
    const coordinates& get_swing_leg_src_coords() const { return lcg.get_swing_leg_src_coords(); };
    const coordinates& get_swing_leg_dst_coords() const { return lcg.get_swing_leg_dst_coords(); };
    const coordinates get_dst_foot_midcoords() const /* get foot_midcoords calculated from swing_leg_dst_coords */
    {
      coordinates tmp(lcg.get_swing_leg_dst_coords());
      tmp.pos += tmp.rot * hrp::Vector3(-1*footstep_param.leg_default_translate_pos[(lcg.get_support_leg() == WC_RLEG) ? 1 : 0]);
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
    hrp::Vector3 get_stair_trajectory_way_point_offset () { return lcg.get_stair_trajectory_way_point_offset(); };
  };
}
#endif /* GAITGENERATOR_H */

