/* -*- coding:utf-8-unix; mode:c++; -*- */

#include "GaitGenerator.h"
/*!
 * @file  testGaitGenerator.cpp
 * @brief Test of gait generator
 * @date  $Date$
 *
 * $Id$
 * TODO
 *   Support test of quad walking
 *   Support test of foot_dif_angle
 *   Support footstep modification using capture point feedback
 *   Support TODO in check_end_values
 *   Supprot other robots and dt
 */


using namespace rats;
#include <cstdio>
#include <coil/stringutil.h>

#define eps_eq(a,b,epsilon) (std::fabs((a)-(b)) < (epsilon))
#ifndef rad2deg
#define rad2deg(rad) (rad * 180 / M_PI)
#endif
#ifndef deg2rad
#define deg2rad(deg) (deg * M_PI / 180)
#endif

// Difference checker to observe too large discontinuous values
// isSmallDiff is true, calculation is correct and continuous.
template<class T> class ValueDifferenceChecker
{
    T prev_value;
    double diff_thre, max_value_diff;
    bool is_small_diff, is_initialized;
    double calcDiff (T& value) const { return 0; };
public:
    ValueDifferenceChecker (double _diff_thre) : diff_thre (_diff_thre), max_value_diff(0), is_small_diff(true), is_initialized(false)
    {
    };
    ~ValueDifferenceChecker () {};
    void checkValueDiff (T& value)
    {
        // Initialize prev value
        if (!is_initialized) {
            prev_value = value;
            is_initialized = true;
        }
        // Calc diff and update
        double diff = calcDiff(value);
        if (diff > max_value_diff) max_value_diff = diff;
        is_small_diff = (diff < diff_thre) && is_small_diff;
        prev_value = value;
    };
    double getMaxValue () const { return max_value_diff; };
    double getDiffThre () const { return diff_thre; };
    bool isSmallDiff () const { return is_small_diff; };
};

template<> double ValueDifferenceChecker<hrp::Vector3>::calcDiff (hrp::Vector3& value) const
{
    return (value - prev_value).norm();
};

template<> double ValueDifferenceChecker< std::vector<hrp::Vector3> >::calcDiff (std::vector<hrp::Vector3>& value) const
{
    double tmp = 0;
    for (size_t i = 0; i < value.size(); i++) {
        tmp += (value[i] - prev_value[i]).norm();
    }
    return tmp;
};

// Error checker between two input values
// isSmallError is true, error is correct.
class ValueErrorChecker
{
    double error_thre, max_value_error;
    bool is_small_error;
public:
    ValueErrorChecker (double _thre) : error_thre (_thre), max_value_error(0), is_small_error(true)
    {
    };
    ~ValueErrorChecker () {};
    void checkValueError (const hrp::Vector3& p0, const hrp::Vector3& p1, std::vector<size_t> neglect_index = std::vector<size_t>())
    {
        hrp::Vector3 errorv(p0-p1);
        for (size_t i = 0; i < neglect_index.size(); i++) errorv(neglect_index[i]) = 0.0;
        double error = errorv.norm();
        if (error > max_value_error) max_value_error = error;
        is_small_error = (error < error_thre) && is_small_error;
    };
    double getMaxValue () const { return max_value_error; };
    double getErrorThre () const { return error_thre; };
    bool isSmallError () const { return is_small_error; };
};

class testGaitGenerator
{
protected:
    double dt; /* [s] */
    std::vector<hrp::Vector3> leg_pos; /* default footstep transformations are necessary */
    std::vector<std::string> all_limbs;
    hrp::Vector3 cog;
    gait_generator* gg;
    bool use_gnuplot, use_graph_append;
    // previous values for walk pattern calculation
    hrp::Vector3 prev_rfoot_pos, prev_lfoot_pos, prev_rfoot_rpy, prev_lfoot_rpy;
    hrp::Vector3 min_rfoot_pos, min_lfoot_pos, max_rfoot_pos, max_lfoot_pos;
    hrp::Vector3 prev_refzmp;
    coordinates prev_ssmc;
    std::vector<bool> prev_contact_states;
    std::vector<double> prev_swing_support_time;
    double min_toe_heel_dif_angle, max_toe_heel_dif_angle, min_zmp_offset_x, max_zmp_offset_x;
    // Value checker
    bool is_contact_states_swing_support_time_validity;
    //   Check difference of value
    ValueDifferenceChecker< hrp::Vector3 > refzmp_diff_checker, cartzmp_diff_checker, cog_diff_checker, ssmcpos_diff_checker, ssmcrot_diff_checker, ssmcposvel_diff_checker, ssmcrotvel_diff_checker;
    ValueDifferenceChecker< std::vector<hrp::Vector3> > footpos_diff_checker, footrot_diff_checker, footposvel_diff_checker, footrotvel_diff_checker, zmpoffset_diff_checker;
    //   Check errors between two values
    ValueErrorChecker zmp_error_checker, cogzmp_error_checker;
    //   Results of list of step time, toe/heel angle, and zmp offset
    std::vector<double> step_time_list, min_toe_heel_dif_angle_list, max_toe_heel_dif_angle_list, min_zmp_offset_x_list, max_zmp_offset_x_list;
    bool is_step_time_valid, is_toe_heel_dif_angle_valid, is_toe_heel_zmp_offset_x_valid;
    // For plot
    std::string test_doc_string;
    std::string fname_cogzmp;
    FILE* fp_cogzmp;
    std::string fname_fpos;
    FILE* fp_fpos;
    std::string fname_frot;
    FILE* fp_frot;
    std::string fname_zoff;
    FILE* fp_zoff;
    std::string fname_fposvel;
    FILE* fp_fposvel;
    std::string fname_frotvel;
    FILE* fp_frotvel;
    std::string fname_thpos;
    FILE* fp_thpos;
    std::string fname_sstime;
    FILE* fp_sstime;
    std::string fname_ssmc;
    FILE* fp_ssmc;
    std::string fname_ssmcvel;
    FILE* fp_ssmcvel;
private:
    ////
    // plot and pattern generation
    ////

    // Plot gnuplot graph and and save graphs to eps files
    void plot_and_save (FILE* gp, const std::string graph_fname, const std::string plot_str)
    {
        fprintf(gp, "%s\n unset multiplot\n", plot_str.c_str());
        fprintf(gp, "set terminal postscript eps color\nset output '/tmp/%s.eps'\n", graph_fname.c_str());
        fprintf(gp, "%s\n unset multiplot\n", plot_str.c_str());
        fflush(gp);
    };

    // Dump generated motion by proc_one_tick function.
    // Calculate error checking and difference values
    void proc_one_walking_motion (size_t i)
    {
            //std::cerr << gg->lcg.gp_count << std::endl;
            // if ( gg->lcg.gp_index == 4 && gg->lcg.gp_count == 100) {
            //   //std::cerr << gg->lcg.gp_index << std::endl;
            //   gg->update_refzmp_queue(coordinates(hrp::Vector3(150, 105, 0)), coordinates(hrp::Vector3(150, -105, 0)));
            // }

            // COG and ZMP
            fprintf(fp_cogzmp, "%f ", i * dt);
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp_cogzmp, "%f ", gg->get_refzmp()(ii));
            }
            hrp::Vector3 czmp = gg->get_cart_zmp();
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp_cogzmp, "%f ", czmp(ii));
            }
            for (size_t ii = 0; ii < 3; ii++) {
                double cogpos;
                if (ii==2) {
                    coordinates tmpc;
                    gg->get_swing_support_mid_coords(tmpc);
                    cogpos = tmpc.pos(2)+gg->get_cog()(2);
                } else {
                    cogpos = gg->get_cog()(ii);
                }
                fprintf(fp_cogzmp, "%f ", cogpos);
            }
            fprintf(fp_cogzmp, "\n");
            fflush(fp_cogzmp);

#define VEC1(s) std::vector<std::string> (1, s)

            // Foot pos
            fprintf(fp_fpos, "%f ", i * dt);
            hrp::Vector3 rfoot_pos = (gg->get_support_leg_names() == VEC1 ("rleg")) ? gg->get_support_leg_steps().front().worldcoords.pos : gg->get_swing_leg_steps().front().worldcoords.pos;
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp_fpos, "%f ", rfoot_pos(ii));
                min_rfoot_pos(ii) = std::min(min_rfoot_pos(ii), rfoot_pos(ii));
                max_rfoot_pos(ii) = std::max(max_rfoot_pos(ii), rfoot_pos(ii));
            }
            hrp::Vector3 lfoot_pos = (gg->get_support_leg_names() == VEC1("lleg")) ? gg->get_support_leg_steps().front().worldcoords.pos : gg->get_swing_leg_steps().front().worldcoords.pos;
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp_fpos, "%f ", lfoot_pos(ii));
                min_lfoot_pos(ii) = std::min(min_lfoot_pos(ii), lfoot_pos(ii));
                max_lfoot_pos(ii) = std::max(max_lfoot_pos(ii), lfoot_pos(ii));
            }
            fprintf(fp_fpos, "\n");
            fflush(fp_fpos);

            // Foot rot
            fprintf(fp_frot, "%f ", i * dt);
    hrp::Matrix33 rfoot_rot = (gg->get_support_leg_names() == VEC1("rleg")) ? gg->get_support_leg_steps().front().worldcoords.rot : gg->get_swing_leg_steps().front().worldcoords.rot;
            hrp::Vector3 rfoot_rpy = hrp::rpyFromRot(rfoot_rot);
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp_frot, "%f ", rad2deg(rfoot_rpy(ii)));
            }
    hrp::Matrix33 lfoot_rot = (gg->get_support_leg_names() == VEC1("lleg")) ? gg->get_support_leg_steps().front().worldcoords.rot : gg->get_swing_leg_steps().front().worldcoords.rot;
            hrp::Vector3 lfoot_rpy = hrp::rpyFromRot(lfoot_rot);
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp_frot, "%f ", rad2deg(lfoot_rpy(ii)));
            }
            fprintf(fp_frot, "\n");
            fflush(fp_frot);

            // ZMP offsets
            fprintf(fp_zoff, "%f ", i * dt);
    hrp::Vector3 rfoot_zmp_offset = (gg->get_support_leg_names() == VEC1("rleg")) ? gg->get_support_foot_zmp_offsets().front() : gg->get_swing_foot_zmp_offsets().front();
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp_zoff, "%f ", rfoot_zmp_offset(ii));
            }
    hrp::Vector3 lfoot_zmp_offset = (gg->get_support_leg_names() == VEC1("lleg")) ? gg->get_support_foot_zmp_offsets().front() : gg->get_swing_foot_zmp_offsets().front();
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp_zoff, "%f ", lfoot_zmp_offset(ii));
            }
            fprintf(fp_zoff, "\n");
            fflush(fp_zoff);
            double tmpzoff;
            if (gg->get_support_leg_names()[0] == "lleg") tmpzoff = rfoot_zmp_offset(0);
            else tmpzoff = lfoot_zmp_offset(0);
            min_zmp_offset_x = std::min(min_zmp_offset_x, tmpzoff);
            max_zmp_offset_x = std::max(max_zmp_offset_x, tmpzoff);

#undef VEC1

            // Foot pos vel
            fprintf(fp_fposvel, "%f ", i * dt);
            if ( i == 0 ) prev_rfoot_pos = rfoot_pos;
            hrp::Vector3 rfootpos_vel = (rfoot_pos - prev_rfoot_pos)/dt;
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp_fposvel, "%f ", rfootpos_vel(ii));
            }
            prev_rfoot_pos = rfoot_pos;
            if ( i == 0 ) prev_lfoot_pos = lfoot_pos;
            hrp::Vector3 lfootpos_vel = (lfoot_pos - prev_lfoot_pos)/dt;
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp_fposvel, "%f ", lfootpos_vel(ii));
            }
            prev_lfoot_pos = lfoot_pos;
            fprintf(fp_fposvel, "\n");
            fflush(fp_fposvel);

            // Foot rot vel
            fprintf(fp_frotvel, "%f ", i * dt);
            if ( i == 0 ) prev_rfoot_rpy = rfoot_rpy;
            hrp::Vector3 rfootrot_vel = (rfoot_rpy - prev_rfoot_rpy)/dt;
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp_frotvel, "%f ", rfootrot_vel(ii));
            }
            prev_rfoot_rpy = rfoot_rpy;
            if ( i == 0 ) prev_lfoot_rpy = lfoot_rpy;
            hrp::Vector3 lfootrot_vel = (lfoot_rpy - prev_lfoot_rpy)/dt;
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp_frotvel, "%f ", lfootrot_vel(ii));
            }
            prev_lfoot_rpy = lfoot_rpy;
            fprintf(fp_frotvel, "\n");
            fflush(fp_frotvel);

            // Toe heel pos
            fprintf(fp_thpos, "%f ", i * dt);
            hrp::Vector3 tmppos;
            tmppos = rfoot_pos+rfoot_rot*hrp::Vector3(gg->get_toe_pos_offset_x(), 0, 0);
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp_thpos, "%f ", tmppos(ii));
                min_rfoot_pos(ii) = std::min(min_rfoot_pos(ii), tmppos(ii));
                max_rfoot_pos(ii) = std::max(max_rfoot_pos(ii), tmppos(ii));
            }
            tmppos = lfoot_pos+lfoot_rot*hrp::Vector3(gg->get_toe_pos_offset_x(), 0, 0);
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp_thpos, "%f ", tmppos(ii));
                min_lfoot_pos(ii) = std::min(min_lfoot_pos(ii), tmppos(ii));
                max_lfoot_pos(ii) = std::max(max_lfoot_pos(ii), tmppos(ii));
            }
            tmppos = rfoot_pos+rfoot_rot*hrp::Vector3(gg->get_heel_pos_offset_x(), 0, 0);
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp_thpos, "%f ", tmppos(ii));
                min_rfoot_pos(ii) = std::min(min_rfoot_pos(ii), tmppos(ii));
                max_rfoot_pos(ii) = std::max(max_rfoot_pos(ii), tmppos(ii));
            }
            tmppos = lfoot_pos+lfoot_rot*hrp::Vector3(gg->get_heel_pos_offset_x(), 0, 0);
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp_thpos, "%f ", tmppos(ii));
                min_lfoot_pos(ii) = std::min(min_lfoot_pos(ii), tmppos(ii));
                max_lfoot_pos(ii) = std::max(max_lfoot_pos(ii), tmppos(ii));
            }
            fprintf(fp_thpos, "\n");
            fflush(fp_thpos);

            // Swing time
            fprintf(fp_sstime, "%f ", i * dt);
            fprintf(fp_sstime, "%f %f ",
                    gg->get_current_swing_time(RLEG),
                    gg->get_current_swing_time(LLEG));
            // Contact States
            std::vector<leg_type> tmp_current_support_states = gg->get_current_support_states();
            bool rleg_contact_states = std::find_if(tmp_current_support_states.begin(), tmp_current_support_states.end(), boost::lambda::_1 == RLEG) != tmp_current_support_states.end();
            bool lleg_contact_states = std::find_if(tmp_current_support_states.begin(), tmp_current_support_states.end(), boost::lambda::_1 == LLEG) != tmp_current_support_states.end();
            fprintf(fp_sstime, "%d %d %f",
                    (rleg_contact_states ? 1 : 0), (lleg_contact_states ? 1 : 0),
                    0.8*gg->get_current_toe_heel_ratio()+0.1); // scale+translation just for visualization
            fprintf(fp_sstime, "\n");
            fflush(fp_sstime);

            // swing support mid coords
            fprintf(fp_ssmc, "%f ", i * dt);
            coordinates tmp_ssmc;
            gg->get_swing_support_mid_coords(tmp_ssmc);
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp_ssmc, "%f ", tmp_ssmc.pos(ii));
            }
            hrp::Vector3 tmp_ssmcr = hrp::rpyFromRot(tmp_ssmc.rot);
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp_ssmc, "%f ", rad2deg(tmp_ssmcr(ii)));
            }
            fprintf(fp_ssmc, "\n");
            fflush(fp_ssmc);

            // swing support mid coords vel
            fprintf(fp_ssmcvel, "%f ", i * dt);
            if ( i == 0 ) prev_ssmc = tmp_ssmc;
            hrp::Vector3 tmp_ssmcpos_vel = (tmp_ssmc.pos - prev_ssmc.pos)/dt;
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp_ssmcvel, "%f ", tmp_ssmcpos_vel(ii));
            }
            hrp::Vector3 prev_ssmcr = hrp::rpyFromRot(prev_ssmc.rot);
            hrp::Vector3 tmp_ssmcrot_vel = (tmp_ssmcr - prev_ssmcr)/dt;
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp_ssmcvel, "%f ", tmp_ssmcrot_vel(ii));
            }
            fprintf(fp_ssmcvel, "\n");
            fflush(fp_ssmcvel);
            prev_ssmc = tmp_ssmc;

            // Toe heel angle
            double tmp_toe_heel_dif_angle = gg->get_toe_heel_dif_angle();
            min_toe_heel_dif_angle = std::min(min_toe_heel_dif_angle, tmp_toe_heel_dif_angle);
            max_toe_heel_dif_angle = std::max(max_toe_heel_dif_angle, tmp_toe_heel_dif_angle);

            // footstep_node change
            if (gg->get_lcg_count() == 0) {
                step_time_list.push_back(gg->get_one_step_count()*dt);
                min_toe_heel_dif_angle_list.push_back(min_toe_heel_dif_angle);
                max_toe_heel_dif_angle_list.push_back(max_toe_heel_dif_angle);
                min_toe_heel_dif_angle = 1e10;
                max_toe_heel_dif_angle = -1e10;
                if ( !eps_eq(min_zmp_offset_x, gg->get_default_zmp_offsets()[0](0), 1e-5) ) min_zmp_offset_x_list.push_back(min_zmp_offset_x);
                else min_zmp_offset_x_list.push_back(0.0);
                if ( !eps_eq(max_zmp_offset_x, gg->get_default_zmp_offsets()[0](0), 1e-5) ) max_zmp_offset_x_list.push_back(max_zmp_offset_x);
                else max_zmp_offset_x_list.push_back(0.0);
                min_zmp_offset_x = 1e10;
                max_zmp_offset_x = -1e10;
            }

            // Error checking
            {
                // Check error between RefZMP and CartZMP. If too large error, PreviewControl tracking is not enough.
                zmp_error_checker.checkValueError(gg->get_cart_zmp(), gg->get_refzmp());
                // Check too large differences (discontinuity)
                //   COG and ZMP
                hrp::Vector3 tmp(gg->get_refzmp());
                refzmp_diff_checker.checkValueDiff(tmp);
                tmp = gg->get_cart_zmp();
                cartzmp_diff_checker.checkValueDiff(tmp);
                tmp = gg->get_cog();
                cog_diff_checker.checkValueDiff(tmp);
                //   Foot pos and rot
                std::vector<hrp::Vector3> tmpvec = boost::assign::list_of(rfoot_pos)(lfoot_pos);
                footpos_diff_checker.checkValueDiff(tmpvec);
                tmpvec = boost::assign::list_of(rfoot_rpy)(lfoot_rpy).convert_to_container < std::vector<hrp::Vector3> > ();
                footrot_diff_checker.checkValueDiff(tmpvec);
                tmpvec = boost::assign::list_of(rfootpos_vel)(lfootpos_vel).convert_to_container < std::vector<hrp::Vector3> > ();
                footposvel_diff_checker.checkValueDiff(tmpvec);
                tmpvec = boost::assign::list_of(rfootrot_vel)(lfootrot_vel).convert_to_container < std::vector<hrp::Vector3> > ();
                footrotvel_diff_checker.checkValueDiff(tmpvec);
                //   Swing support mid coorsd
                ssmcpos_diff_checker.checkValueDiff(tmp_ssmc.pos);
                ssmcrot_diff_checker.checkValueDiff(tmp_ssmcr);
                ssmcposvel_diff_checker.checkValueDiff(tmp_ssmcpos_vel);
                ssmcrotvel_diff_checker.checkValueDiff(tmp_ssmcrot_vel);
                //   ZMP offset
                tmpvec = boost::assign::list_of(rfoot_zmp_offset)(lfoot_zmp_offset).convert_to_container < std::vector<hrp::Vector3> > ();
                zmpoffset_diff_checker.checkValueDiff(tmpvec);
            }
            //   If contact states are not change, prev_swing_support_time is not dt, otherwise prev_swing_support_time is dt.
            is_contact_states_swing_support_time_validity = is_contact_states_swing_support_time_validity &&
                ((prev_contact_states[0] == rleg_contact_states) ? !eps_eq(prev_swing_support_time[0],dt,1e-5) : eps_eq(prev_swing_support_time[0],dt,1e-5)) &&
                ((prev_contact_states[1] == lleg_contact_states) ? !eps_eq(prev_swing_support_time[1],dt,1e-5) : eps_eq(prev_swing_support_time[1],dt,1e-5));
            prev_refzmp = gg->get_refzmp();
            prev_contact_states[0] = rleg_contact_states;
            prev_contact_states[1] = lleg_contact_states;
            prev_swing_support_time[0] = gg->get_current_swing_time(RLEG);
            prev_swing_support_time[1] = gg->get_current_swing_time(LLEG);
    };

    // Plot state values and print error check
    void plot_and_print_errorcheck ()
    {
        /* plot */
        if (use_gnuplot) {
            size_t gpsize = 12;
            FILE* gps[gpsize];
            for (size_t ii = 0; ii < gpsize;ii++) {
                gps[ii] = popen("gnuplot", "w");
            }
            {
                std::ostringstream oss("");
                std::string gtitle("COG_and_ZMP");
                size_t tmp_start = 2;
                oss << "set multiplot layout 3, 1 title '" << gtitle << "'" << std::endl;
                std::string titles[3] = {"X", "Y", "Z"};
                for (size_t ii = 0; ii < 3; ii++) {
                    oss << "set xlabel 'Time [s]'" << std::endl;
                    oss << "set ylabel '" << titles[ii] << "[m]'" << std::endl;
                    oss << "plot "
                        << "'" << fname_cogzmp << "' using 1:" << (tmp_start+ii) << " with lines title 'REFZMP',"
                        << "'" << fname_cogzmp << "' using 1:" << (tmp_start+3+ii) << " with lines title 'CARTZMP',"
                        << "'" << fname_cogzmp << "' using 1:" << (tmp_start+6+ii) << " with lines title 'COG'"
                        << std::endl;
                }
                plot_and_save(gps[0], gtitle, oss.str());
            }
            {
                std::ostringstream oss("");
                std::string gtitle("Swing_support_pos");
                size_t tmp_start = 2;
                oss << "set multiplot layout 3, 1 title '" << gtitle << "'" << std::endl;
                std::string titles[3] = {"X", "Y", "Z"};
                for (size_t ii = 0; ii < 3; ii++) {
                    oss << "set xlabel 'Time [s]'" << std::endl;
                    oss << "set ylabel '" << titles[ii] << "[m]'" << std::endl;
                    oss << "plot "
                        << "'" << fname_fpos << "' using 1:" << (tmp_start+ii) << " with lines title 'rleg',"
                        << "'" << fname_fpos << "' using 1:" << (tmp_start+3+ii) << " with lines title 'lleg'"
                        << std::endl;
                }
                plot_and_save(gps[1], gtitle, oss.str());
            }
            {
                std::ostringstream oss("");
                std::string gtitle("Swing_support_rot");
                size_t tmp_start = 2;
                oss << "set multiplot layout 3, 1 title '" << gtitle << "'" << std::endl;
                std::string titles[3] = {"Roll", "Pitch", "Yaw"};
                for (size_t ii = 0; ii < 3; ii++) {
                    oss << "set xlabel 'Time [s]'" << std::endl;
                    oss << "set ylabel '" << titles[ii] << "[deg]'" << std::endl;
                    oss << "plot "
                        << "'" << fname_frot << "' using 1:" << (tmp_start+ii) << " with lines title 'rleg',"
                        << "'" << fname_frot << "' using 1:" << (tmp_start+3+ii) << " with lines title 'lleg'"
                        << std::endl;
                }
                plot_and_save(gps[2], gtitle, oss.str());
            }
            {
                std::ostringstream oss("");
                std::string gtitle("Swing_support_zmp_offset");
                size_t tmp_start = 2;
                oss << "set multiplot layout 3, 1 title '" << gtitle << "'" << std::endl;
                std::string titles[3] = {"X", "Y", "Z"};
                for (size_t ii = 0; ii < 3; ii++) {
                    oss << "set xlabel 'Time [s]'" << std::endl;
                    oss << "set ylabel '" << titles[ii] << "[m]'" << std::endl;
                    oss << "plot "
                        << "'" << fname_zoff << "' using 1:" << (tmp_start+ii) << " with lines title 'rleg',"
                        << "'" << fname_zoff << "' using 1:" << (tmp_start+3+ii) << " with lines title 'lleg'"
                        << std::endl;
                }
                plot_and_save(gps[3], gtitle, oss.str());
            }
            {
                std::ostringstream oss("");
                std::string gtitle("Swing_support_pos_vel");
                size_t tmp_start = 2;
                oss << "set multiplot layout 3, 1 title '" << gtitle << "'" << std::endl;
                std::string titles[3] = {"X", "Y", "Z"};
                for (size_t ii = 0; ii < 3; ii++) {
                    oss << "set xlabel 'Time [s]'" << std::endl;
                    oss << "set ylabel '" << titles[ii] << "[m/s]'" << std::endl;
                    oss << "plot "
                        << "'" << fname_fposvel << "' using 1:" << (tmp_start+ii) << " with lines title 'rleg',"
                        << "'" << fname_fposvel << "' using 1:" << (tmp_start+3+ii) << " with lines title 'lleg'"
                        << std::endl;
                }
                plot_and_save(gps[4], gtitle, oss.str());
            }
            {
                std::ostringstream oss("");
                std::string gtitle("Swing_support_rot_vel");
                size_t tmp_start = 2;
                oss << "set multiplot layout 3, 1 title '" << gtitle << "'" << std::endl;
                std::string titles[3] = {"Roll", "Pitch", "Yaw"};
                for (size_t ii = 0; ii < 3; ii++) {
                    oss << "set xlabel 'Time [s]'" << std::endl;
                    oss << "set ylabel '" << titles[ii] << "[deg/s]'" << std::endl;
                    oss << "plot "
                        << "'" << fname_frotvel << "' using 1:" << (tmp_start+ii) << " with lines title 'rleg',"
                        << "'" << fname_frotvel << "' using 1:" << (tmp_start+3+ii) << " with lines title 'lleg'"
                        << std::endl;
                }
                plot_and_save(gps[5], gtitle, oss.str());
            }
            {
                std::ostringstream oss("");
                std::string gtitle("Swing_support_pos_trajectory");
                double min_v[3], max_v[3], range[3];
                size_t tmp_thpos_start = 2;
                for (size_t ii = 0; ii < 3; ii++) {
                    min_v[ii] = std::min(min_rfoot_pos(ii), min_lfoot_pos(ii));
                    max_v[ii] = std::max(max_rfoot_pos(ii), max_lfoot_pos(ii));
                    if (min_v[ii] == 0.0 && max_v[ii] == 0.0) {
                        min_v[ii] = -0.1;
                        max_v[ii] = 0.1;
                    }
                    range[ii] = max_v[ii] - min_v[ii];
                    double mid = (max_v[ii]+min_v[ii])/2.0;
                    min_v[ii] = mid + range[ii] * 1.05 * -0.5;
                    max_v[ii] = mid + range[ii] * 1.05 * 0.5;
                }
                oss << "set multiplot layout 2, 1 title '" << gtitle << "'" << std::endl;
                //oss << "set title 'X-Z'" << std::endl;
                oss << "set size ratio " << range[2]/range[0] << std::endl;
                oss << "set xlabel 'X [m]'" << std::endl;            
                oss << "set ylabel 'Z [m]'" << std::endl;            
                oss << "plot "
                    << "[" << min_v[0]<< ":" << max_v[0] << "]"
                    << "[" << min_v[2] << ":" << max_v[2] << "]"
                    << "'" << fname_fpos << "' using " << (2) << ":" << (2+2)  << " with lines title 'rleg ee',"
                    << "'" << fname_fpos << "' using " << (2+3) << ":" << (2+3+2) << " with lines title 'lleg ee',"
                    << "'" << fname_thpos << "' using " << (tmp_thpos_start) << ":" << (tmp_thpos_start+2)  << " with lines title 'rleg toe',"
                    << "'" << fname_thpos << "' using " << (tmp_thpos_start+3) << ":" << (tmp_thpos_start+3+2)  << " with lines title 'lleg toe',"
                    << "'" << fname_thpos << "' using " << (tmp_thpos_start+3+3) << ":" << (tmp_thpos_start+3+3+2)  << " with lines title 'rleg heel',"
                    << "'" << fname_thpos << "' using " << (tmp_thpos_start+3+3+3) << ":" << (tmp_thpos_start+3+3+3+2)  << " with lines title 'lleg heel'"
                    << std::endl;
                //oss << "set title 'Y-Z'" << std::endl;
                oss << "set size ratio " << range[2]/range[1] << std::endl;
                oss << "set xlabel 'Y [m]'" << std::endl;            
                oss << "set ylabel 'Z [m]'" << std::endl;            
                oss << "plot "
                    << "[" << min_v[1]<< ":" << max_v[1] << "]"
                    << "[" << min_v[2] << ":" << max_v[2] << "]"
                    << "'" << fname_fpos << "' using " << (2+1) << ":" << (2+2)  << " with lines title 'rleg ee',"
                    << "'" << fname_fpos << "' using " << (2+3+1) << ":" << (2+3+2) << " with lines title 'lleg ee',"
                    << "'" << fname_thpos << "' using " << (tmp_thpos_start+1) << ":" << (tmp_thpos_start+2)  << " with lines title 'rleg toe',"
                    << "'" << fname_thpos << "' using " << (tmp_thpos_start+3+1) << ":" << (tmp_thpos_start+3+2)  << " with lines title 'lleg toe',"
                    << "'" << fname_thpos << "' using " << (tmp_thpos_start+3+3+1) << ":" << (tmp_thpos_start+3+3+2)  << " with lines title 'rleg heel',"
                    << "'" << fname_thpos << "' using " << (tmp_thpos_start+3+3+3+1) << ":" << (tmp_thpos_start+3+3+3+2)  << " with lines title 'lleg heel'"
                    << std::endl;
                plot_and_save(gps[6], gtitle, oss.str());
            }
            {
                std::ostringstream oss("");
                std::string gtitle("Swing_support_remain_time");
                oss << "set multiplot layout 1, 1 title '" << gtitle << "'" << std::endl;
                oss << "set title 'Remain Time'" << std::endl;
                oss << "set xlabel 'Time [s]'" << std::endl;
                oss << "set ylabel 'Time [s]'" << std::endl;
                oss << "plot "
                    << "'" << fname_sstime << "' using 1:" << 2 << " with lines title 'rleg remain time',"
                    << "'" << fname_sstime << "' using 1:" << 3 << " with lines title 'lleg remain time',"
                    << "'" << fname_sstime << "' using 1:" << 4 << " with lines title 'rleg contact states',"
                    << "'" << fname_sstime << "' using 1:" << 5 << " with lines title 'lleg contact states',"
                    << "'" << fname_sstime << "' using 1:" << 6 << " with lines title 'toe_heel_ratio*0.8+0.1'"
                    << std::endl;
                plot_and_save(gps[7], gtitle, oss.str());
            }
            {
                std::ostringstream oss("");
                std::string gtitle("Swing_support_mid_coords_pos");
                size_t tmp_start = 2;
                oss << "set multiplot layout 3, 1 title '" << gtitle << "'" << std::endl;
                std::string titles[3] = {"X", "Y", "Z"};
                for (size_t ii = 0; ii < 3; ii++) {
                    oss << "set xlabel 'Time [s]'" << std::endl;
                    oss << "set ylabel 'Pos " << titles[ii] << "[m]'" << std::endl;
                    oss << "plot "
                        << "'" << fname_ssmc << "' using 1:" << (tmp_start+ii) << " with lines title ''"
                        << std::endl;
                }
                plot_and_save(gps[8], gtitle, oss.str());
            }
            {
                std::ostringstream oss("");
                std::string gtitle("Swing_support_mid_coords_rot");
                size_t tmp_start = 2;
                oss << "set multiplot layout 3, 1 title '" << gtitle << "'" << std::endl;
                std::string titles[3] = {"Roll", "Pitch", "Yaw"};
                for (size_t ii = 0; ii < 3; ii++) {
                    oss << "set xlabel 'Time [s]'" << std::endl;
                    oss << "set ylabel 'Rot " << titles[ii] << "[deg]'" << std::endl;
                    oss << "plot "
                        << "'" << fname_ssmc << "' using 1:" << (tmp_start+ii+3) << " with lines title ''"
                        << std::endl;
                }
                plot_and_save(gps[9], gtitle, oss.str());
            }
            {
                std::ostringstream oss("");
                std::string gtitle("Swing_support_mid_coords_pos_vel");
                size_t tmp_start = 2;
                oss << "set multiplot layout 3, 1 title '" << gtitle << "'" << std::endl;
                std::string titles[3] = {"X", "Y", "Z"};
                for (size_t ii = 0; ii < 3; ii++) {
                    oss << "set xlabel 'Time [s]'" << std::endl;
                    oss << "set ylabel 'PosVel " << titles[ii] << "[m/s]'" << std::endl;
                    oss << "plot "
                        << "'" << fname_ssmcvel << "' using 1:" << (tmp_start+ii) << " with lines title ''"
                        << std::endl;
                }
                plot_and_save(gps[10], gtitle, oss.str());
            }
            {
                std::ostringstream oss("");
                std::string gtitle("Swing_support_mid_coords_rot_vel");
                size_t tmp_start = 2;
                oss << "set multiplot layout 3, 1 title '" << gtitle << "'" << std::endl;
                std::string titles[3] = {"Roll", "Pitch", "Yaw"};
                for (size_t ii = 0; ii < 3; ii++) {
                    oss << "set xlabel 'Time [s]'" << std::endl;
                    oss << "set ylabel 'RotVel " << titles[ii] << "[deg/s]'" << std::endl;
                    oss << "plot "
                        << "'" << fname_ssmcvel << "' using 1:" << (tmp_start+ii+3) << " with lines title ''"
                        << std::endl;
                }
                plot_and_save(gps[11], gtitle, oss.str());
            }
            //
            if (use_graph_append) {
                int ret = 0;
                usleep(500000); // Wait for gnuplot plot finishing (0.5[s])
                ret = system("bash -c 'for f in /tmp/*.eps; do convert -density 250x250 $f ${f//eps}jpg; done'");
                ret = system("(cd /tmp; convert +append Swing_support_mid_coords_pos.jpg Swing_support_mid_coords_pos_vel.jpg Swing_support_mid_coords_rot.jpg Swing_support_mid_coords_rot_vel.jpg img1.jpg)");
                ret = system("(cd /tmp/; convert +append Swing_support_pos.jpg Swing_support_pos_vel.jpg Swing_support_rot.jpg Swing_support_rot_vel.jpg img2.jpg)");
                ret = system("(cd /tmp/; convert +append Swing_support_zmp_offset.jpg Swing_support_remain_time.jpg COG_and_ZMP.jpg Swing_support_pos_trajectory.jpg img3.jpg)");
                std::string tmpstr = test_doc_string.substr(0, test_doc_string.find(":"));
                for(size_t c = tmpstr.find_first_of(" "); c != std::string::npos; c = c = tmpstr.find_first_of(" ")){
                    tmpstr.erase(c,1);
                }
                ret = system(std::string("(cd /tmp/; convert -append img1.jpg img2.jpg img3.jpg testGaitGeneratorResults_"+tmpstr+".jpg; rm -f /tmp/img[123].jpg /tmp/COG_and_ZMP.jpg /tmp/Swing_support*.jpg)").c_str());
            } else {
                double tmp;
                std::cin >> tmp;
            }
            for (size_t ii = 0; ii < gpsize; ii++) {
                fprintf(gps[ii], "exit\n");
                fflush(gps[ii]);
                pclose(gps[ii]);
            }
        }
        std::string tmpstr = test_doc_string.substr(0, test_doc_string.find(":"));
        for(size_t c = tmpstr.find_first_of(" "); c != std::string::npos; c = c = tmpstr.find_first_of(" ")){
            tmpstr.erase(c,1);
        }
        std::cerr << "Checking of " << tmpstr << " : all results = " << (check_all_results()?"true":"false") << std::endl;
        std::cerr << "  ZMP error : " << (zmp_error_checker.isSmallError()?"true":"false") << ", max_error : " << zmp_error_checker.getMaxValue()*1e3 << "[mm], thre : " << zmp_error_checker.getErrorThre()*1e3 << "[mm]" << std::endl;
        std::cerr << "  COGZMP error : " << (cogzmp_error_checker.isSmallError()?"true":"false") << ", max_error : " << cogzmp_error_checker.getMaxValue()*1e3 << "[mm], thre : " << cogzmp_error_checker.getErrorThre()*1e3 << "[mm]" << std::endl;
        std::cerr << "  RefZMP diff : " << (refzmp_diff_checker.isSmallDiff()?"true":"false") << ", max_diff : " << refzmp_diff_checker.getMaxValue()*1e3 << "[mm], thre : " << refzmp_diff_checker.getDiffThre()*1e3 << "[mm]" << std::endl;
        std::cerr << "  CartZMP diff : " << (cartzmp_diff_checker.isSmallDiff()?"true":"false") << ", max_diff : " << cartzmp_diff_checker.getMaxValue()*1e3 << "[mm], thre : " << cartzmp_diff_checker.getDiffThre()*1e3 << "[mm]" << std::endl;
        std::cerr << "  COG diff : " << (cog_diff_checker.isSmallDiff()?"true":"false") << ", max_diff : " << cog_diff_checker.getMaxValue()*1e3 << "[mm], thre : " << cog_diff_checker.getDiffThre()*1e3 << "[mm]" <<std::endl;
        std::cerr << "  FootPos diff : " << (footpos_diff_checker.isSmallDiff()?"true":"false") << ", max_diff : " << footpos_diff_checker.getMaxValue()*1e3 << "[mm], thre : " << footpos_diff_checker.getDiffThre()*1e3 << "[mm]" <<std::endl;
        std::cerr << "  FootRot diff : " << (footrot_diff_checker.isSmallDiff()?"true":"false") << ", max_diff : " << rad2deg(footrot_diff_checker.getMaxValue()) << "[deg], thre : " << rad2deg(footrot_diff_checker.getDiffThre()) << "[deg]" <<std::endl;
        std::cerr << "  FootPosVel diff : " << (footposvel_diff_checker.isSmallDiff()?"true":"false") << ", max_diff : " << footposvel_diff_checker.getMaxValue()*1e3 << "[mm/s], thre : " << footposvel_diff_checker.getDiffThre()*1e3 << "[mm/s]" <<std::endl;
        std::cerr << "  FootRotVel diff : " << (footrotvel_diff_checker.isSmallDiff()?"true":"false") << ", max_diff : " << rad2deg(footrotvel_diff_checker.getMaxValue()) << "[deg/s], thre : " << rad2deg(footrotvel_diff_checker.getDiffThre()) << "[deg/s]" <<std::endl;
        std::cerr << "  SwingSupportFootMidPos diff : " << (ssmcpos_diff_checker.isSmallDiff()?"true":"false") << ", max_diff : " << ssmcpos_diff_checker.getMaxValue()*1e3 << "[mm], thre : " << ssmcpos_diff_checker.getDiffThre()*1e3 << "[mm]" <<std::endl;
        std::cerr << "  SwingSupportFootMidRot diff : " << (ssmcrot_diff_checker.isSmallDiff()?"true":"false") << ", max_diff : " << rad2deg(ssmcrot_diff_checker.getMaxValue()) << "[deg], thre : " << rad2deg(ssmcrot_diff_checker.getDiffThre()) << "[deg]" <<std::endl;
        std::cerr << "  SwingSupportFootMidPosVel diff : " << (ssmcposvel_diff_checker.isSmallDiff()?"true":"false") << ", max_diff : " << ssmcposvel_diff_checker.getMaxValue()*1e3 << "[mm/s], thre : " << ssmcposvel_diff_checker.getDiffThre()*1e3 << "[mm/s]" <<std::endl;
        std::cerr << "  SwingSupportFootMidRotVel diff : " << (ssmcrotvel_diff_checker.isSmallDiff()?"true":"false") << ", max_diff : " << rad2deg(ssmcrotvel_diff_checker.getMaxValue()) << "[deg/s], thre : " << rad2deg(ssmcrotvel_diff_checker.getDiffThre()) << "[deg/s]" <<std::endl;
        std::cerr << "  ZMPOffset diff : " << (zmpoffset_diff_checker.isSmallDiff()?"true":"false") << ", max_diff : " << zmpoffset_diff_checker.getMaxValue()*1e3 << "[mm/s], thre : " << zmpoffset_diff_checker.getDiffThre()*1e3 << "[mm/s]" <<std::endl;
        std::cerr << "  Contact states & swing support time validity : " << (is_contact_states_swing_support_time_validity?"true":"false") << std::endl;
        std::cerr << "  Step time validity : " << (is_step_time_valid?"true":"false") << std::endl;
        std::cerr << "  ToeHeel angle validity : " << (is_toe_heel_dif_angle_valid?"true":"false") << std::endl;
        std::cerr << "  ToeHeel+ZMPoffset validity : " << (is_toe_heel_zmp_offset_x_valid?"true":"false") << std::endl;
    };

    void check_start_values ()
    {
        // Check if start/end COG(xy) is sufficiently close to REFZMP(xy).
        std::vector<size_t> neglect_index = boost::assign::list_of(2);
        cogzmp_error_checker.checkValueError(gg->get_cog(), gg->get_refzmp(), neglect_index);
    };

    void check_end_values ()
    {
        // Check if start/end COG(xy) is sufficiently close to REFZMP(xy).
        std::vector<size_t> neglect_index = boost::assign::list_of(2);
        cogzmp_error_checker.checkValueError(gg->get_cog(), gg->get_refzmp(), neglect_index);
        // Check step times by comparing calculated step time vs input footsteps step times
        std::vector< std::vector<step_node> > fsl;
        gg->get_footstep_nodes_list(fsl);
        is_step_time_valid = step_time_list.size() == fsl.size();
        if (is_step_time_valid) {
            for (size_t i = 0; i < step_time_list.size(); i++) {
                is_step_time_valid = eps_eq(step_time_list[i], fsl[i][0].step_time, 1e-5) && is_step_time_valid;
            }
        }
        // Check toe heel angle by comparing calculated toe heel angle (min/max = heel/toe) vs input footsteps toe heel angles
        is_toe_heel_dif_angle_valid = (min_toe_heel_dif_angle_list.size() == fsl.size()) && (max_toe_heel_dif_angle_list.size() == fsl.size());
        if (is_toe_heel_dif_angle_valid) {
            if (gg->get_use_toe_heel_auto_set()) {
                // TODO : not implemented yet
            } else {
                //for (size_t i = 0; i < min_toe_heel_dif_angle_list.size(); i++) {
                for (size_t i = 1; i < min_toe_heel_dif_angle_list.size(); i++) {
                    // std::cerr << "[" << min_toe_heel_dif_angle_list[i] << " " << max_toe_heel_dif_angle_list[i] << " "
                    //           << -fsl[i][0].heel_angle << " " << fsl[i][0].toe_angle << "]" << std::endl;
                    is_toe_heel_dif_angle_valid = eps_eq(min_toe_heel_dif_angle_list[i], (-fsl[i][0].heel_angle), 1e-5)
                        && eps_eq(max_toe_heel_dif_angle_list[i], fsl[i][0].toe_angle, 1e-5)
                        && is_toe_heel_dif_angle_valid;
                }
            }
        }
        // Check validity of toe heel + zmp offset
        //   If use_toe_heel_transition is true, use/not-use of toe/heel angle corresponds to use/not-use zmp transition of toe/heel.
        is_toe_heel_zmp_offset_x_valid = (min_zmp_offset_x_list.size() == fsl.size()) && (max_zmp_offset_x_list.size() == fsl.size());
        if (gg->get_use_toe_heel_transition()) {
            for (size_t i = 0; i < min_zmp_offset_x_list.size(); i++) {
                // std::cerr << "[" << min_zmp_offset_x_list[i] << " " << max_zmp_offset_x_list[i] << " " << gg->get_toe_zmp_offset_x() << " " << gg->get_heel_zmp_offset_x() << "]" << std::endl;
                bool heel_valid = true;
                if ( !eps_eq(min_toe_heel_dif_angle_list[i], 0.0, 1e-5) ) { // If use heel, zmp offset should be same as heel zmp offset
                    heel_valid = eps_eq(min_zmp_offset_x_list[i], gg->get_heel_zmp_offset_x(), 1e-5);
                } else { // If not use heel, zmp offset should not be same as heel zmp offset
                    heel_valid = !eps_eq(min_zmp_offset_x_list[i], gg->get_heel_zmp_offset_x(), 1e-5);
                }
                bool toe_valid = true;
                if ( !eps_eq(max_toe_heel_dif_angle_list[i], 0.0, 1e-5) ) { // If use toe, zmp offset should be same as toe zmp offset
                    toe_valid = eps_eq(max_zmp_offset_x_list[i], gg->get_toe_zmp_offset_x(), 1e-5);
                } else { // If not use toe, zmp offset should not be same as toe zmp offset
                    toe_valid = !eps_eq(max_zmp_offset_x_list[i], gg->get_toe_zmp_offset_x(), 1e-5);
                }
                // std::cerr << heel_valid << " " << toe_valid << std::endl;
                is_toe_heel_zmp_offset_x_valid = heel_valid && toe_valid && is_toe_heel_zmp_offset_x_valid;
            }
        } else {
            // TODO : not implemented yet
        }
    };

    // Generate and plot walk pattern
    void gen_and_plot_walk_pattern(const step_node& initial_support_leg_step, const step_node& initial_swing_leg_dst_step)
    {
        parse_params(false);
        gg->print_param();
        gg->initialize_gait_parameter(cog, boost::assign::list_of(initial_support_leg_step), boost::assign::list_of(initial_swing_leg_dst_step));
        while ( !gg->proc_one_tick() );
        //gg->print_footstep_list();
        /* make step and dump */
        check_start_values();
        size_t i = 0;
        while ( gg->proc_one_tick() ) {
            proc_one_walking_motion(i);
            i++;
        }
        check_end_values();
        plot_and_print_errorcheck ();
    };

    void gen_and_plot_walk_pattern()
    {
        std::vector<std::string> tmp_string_vector = boost::assign::list_of("rleg");
        if (gg->get_footstep_front_leg_names() == tmp_string_vector) {
            step_node initial_support_leg_step = step_node(LLEG, coordinates(leg_pos[1]), 0, 0, 0, 0);
            step_node initial_swing_leg_dst_step = step_node(RLEG, coordinates(leg_pos[0]), 0, 0, 0, 0);
            gen_and_plot_walk_pattern(initial_support_leg_step, initial_swing_leg_dst_step);
        } else {
            step_node initial_support_leg_step = step_node(RLEG, coordinates(leg_pos[0]), 0, 0, 0, 0);
            step_node initial_swing_leg_dst_step = step_node(LLEG, coordinates(leg_pos[1]), 0, 0, 0, 0);
            gen_and_plot_walk_pattern(initial_support_leg_step, initial_swing_leg_dst_step);
        }
    }

public:
    std::vector<std::string> arg_strs;
    testGaitGenerator(double _dt) : dt(_dt), use_gnuplot(true), use_graph_append(false), is_contact_states_swing_support_time_validity(true),
                                    refzmp_diff_checker(20.0*1e-3), cartzmp_diff_checker(20.0*1e-3), cog_diff_checker(10.0*1e-3), // [mm]
                                    ssmcpos_diff_checker(10.0*1e-3), ssmcrot_diff_checker(deg2rad(0.1)),
                                    ssmcposvel_diff_checker(10.0*1e-3), ssmcrotvel_diff_checker(deg2rad(1)),
                                    footpos_diff_checker(10.0*1e-3), footrot_diff_checker(deg2rad(1)),
                                    footposvel_diff_checker(40*1e-3), footrotvel_diff_checker(deg2rad(15)),
                                    zmpoffset_diff_checker(20.0*1e-3),
                                    zmp_error_checker(50*1e-3), cogzmp_error_checker(1.5*1e-3),
                                    min_rfoot_pos(1e10,1e10,1e10), min_lfoot_pos(1e10,1e10,1e10), max_rfoot_pos(-1e10,-1e10,-1e10), max_lfoot_pos(-1e10,-1e10,-1e10),
                                    min_toe_heel_dif_angle(1e10), max_toe_heel_dif_angle(-1e10), min_zmp_offset_x(1e10), max_zmp_offset_x(-1e10),
                                    prev_contact_states(2, true), // RLEG, LLEG
                                    prev_swing_support_time(2, 1e2), // RLEG, LLEG
                                    fname_cogzmp("/tmp/plot-cogzmp.dat"), fp_cogzmp(fopen(fname_cogzmp.c_str(), "w")),
                                    fname_fpos("/tmp/plot-fpos.dat"), fp_fpos(fopen(fname_fpos.c_str(), "w")),
                                    fname_frot("/tmp/plot-frot.dat"), fp_frot(fopen(fname_frot.c_str(), "w")),
                                    fname_zoff("/tmp/plot-zoff.dat"), fp_zoff(fopen(fname_zoff.c_str(), "w")),
                                    fname_fposvel("/tmp/plot-fposvel.dat"), fp_fposvel(fopen(fname_fposvel.c_str(), "w")),
                                    fname_frotvel("/tmp/plot-frotvel.dat"), fp_frotvel(fopen(fname_frotvel.c_str(), "w")),
                                    fname_thpos("/tmp/plot-thpos.dat"), fp_thpos(fopen(fname_thpos.c_str(), "w")),
                                    fname_sstime("/tmp/plot-sstime.dat"), fp_sstime(fopen(fname_sstime.c_str(), "w")),
                                    fname_ssmc("/tmp/plot-ssmc.dat"), fp_ssmc(fopen(fname_ssmc.c_str(), "w")),
                                    fname_ssmcvel("/tmp/plot-ssmcvel.dat"), fp_ssmcvel(fopen(fname_ssmcvel.c_str(), "w"))
    {};

    virtual ~testGaitGenerator()
    {
        if (gg != NULL) {
            delete gg;
            gg = NULL;
        }
        fclose(fp_cogzmp);
        fclose(fp_fpos);
        fclose(fp_frot);
        fclose(fp_zoff);
        fclose(fp_fposvel);
        fclose(fp_frotvel);
        fclose(fp_thpos);
        fclose(fp_sstime);
        fclose(fp_ssmc);
        fclose(fp_ssmcvel);
    };

    void test0 ()
    {
        test_doc_string = "test0 : Set foot steps";
        /* initialize sample footstep_list */
        parse_params();
        std::vector< std::vector<step_node> > fnsl;
        fnsl.push_back(boost::assign::list_of(step_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(0, 0, 0)+leg_pos[0])), gg->get_default_step_height(), gg->get_default_step_time(), gg->get_toe_angle(), gg->get_heel_angle())));
        fnsl.push_back(boost::assign::list_of(step_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(0, 0, 0)+leg_pos[1])), gg->get_default_step_height(), gg->get_default_step_time(), gg->get_toe_angle(), gg->get_heel_angle())));
        fnsl.push_back(boost::assign::list_of(step_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(100*1e-3, 0, 0)+leg_pos[0])), gg->get_default_step_height(), gg->get_default_step_time(), gg->get_toe_angle(), gg->get_heel_angle())));
        fnsl.push_back(boost::assign::list_of(step_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(200*1e-3, 0, 0)+leg_pos[1])), gg->get_default_step_height(), gg->get_default_step_time(), gg->get_toe_angle(), gg->get_heel_angle())));
        fnsl.push_back(boost::assign::list_of(step_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(200*1e-3, 0, 0)+leg_pos[0])), gg->get_default_step_height(), gg->get_default_step_time(), gg->get_toe_angle(), gg->get_heel_angle())));
        gg->set_foot_steps_list(fnsl);
        gen_and_plot_walk_pattern();
    };

    void test1 ()
    {
        test_doc_string = "test1 : Go pos x,y,th combination";
        /* initialize sample footstep_list */
        parse_params();
        gg->clear_footstep_nodes_list();
        coordinates start_ref_coords;
        mid_coords(start_ref_coords, 0.5, coordinates(leg_pos[0]), coordinates(leg_pos[1]));
        gg->go_pos_param_2_footstep_nodes_list(200*1e-3, 100*1e-3, 20, boost::assign::list_of(coordinates(leg_pos[0])), start_ref_coords, boost::assign::list_of(RLEG));
        gen_and_plot_walk_pattern();
    };

    void test2 ()
    {
        test_doc_string = "test2 : Go pos x";
        /* initialize sample footstep_list */
        parse_params();
        gg->clear_footstep_nodes_list();
        coordinates start_ref_coords;
        mid_coords(start_ref_coords, 0.5, coordinates(leg_pos[1]), coordinates(leg_pos[0]));
        gg->go_pos_param_2_footstep_nodes_list(300*1e-3, 0, 0, boost::assign::list_of(coordinates(leg_pos[1])), start_ref_coords, boost::assign::list_of(LLEG));
        gen_and_plot_walk_pattern();
    };

    void test3 ()
    {
        test_doc_string = "test3 : Go pos y";
        /* initialize sample footstep_list */
        parse_params();
        gg->clear_footstep_nodes_list();
        coordinates start_ref_coords;
        mid_coords(start_ref_coords, 0.5, coordinates(leg_pos[0]), coordinates(leg_pos[1]));
        gg->go_pos_param_2_footstep_nodes_list(0, 150*1e-3, 0, boost::assign::list_of(coordinates(leg_pos[0])), start_ref_coords, boost::assign::list_of(RLEG));
        gen_and_plot_walk_pattern();
    };

    void test4 ()
    {
        test_doc_string = "test4 : Go pos th";
        /* initialize sample footstep_list */
        parse_params();
        gg->clear_footstep_nodes_list();
        coordinates start_ref_coords;
        mid_coords(start_ref_coords, 0.5, coordinates(leg_pos[1]), coordinates(leg_pos[0]));
        gg->go_pos_param_2_footstep_nodes_list(0, 0, 30, boost::assign::list_of(coordinates(leg_pos[1])), start_ref_coords, boost::assign::list_of(LLEG));
        gen_and_plot_walk_pattern();
    };

    void test5 ()
    {
        test_doc_string = "test5 : Set foot steps with Z change";
        /* initialize sample footstep_list */
        parse_params();
        gg->set_default_orbit_type(CYCLOIDDELAY);
        std::vector< std::vector<step_node> > fnsl;
        fnsl.push_back(boost::assign::list_of(step_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(0, 0, 0)+leg_pos[0])), gg->get_default_step_height(), gg->get_default_step_time(), gg->get_toe_angle(), gg->get_heel_angle())));
        fnsl.push_back(boost::assign::list_of(step_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(0, 0, 0)+leg_pos[1])), gg->get_default_step_height(), gg->get_default_step_time(), gg->get_toe_angle(), gg->get_heel_angle())));
        fnsl.push_back(boost::assign::list_of(step_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(100*1e-3, 0, 100*1e-3)+leg_pos[0])), gg->get_default_step_height(), gg->get_default_step_time(), gg->get_toe_angle(), gg->get_heel_angle())));
        fnsl.push_back(boost::assign::list_of(step_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(200*1e-3, 0, 200*1e-3)+leg_pos[1])), gg->get_default_step_height(), gg->get_default_step_time(), gg->get_toe_angle(), gg->get_heel_angle())));
        fnsl.push_back(boost::assign::list_of(step_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(300*1e-3, 0, 300*1e-3)+leg_pos[0])), gg->get_default_step_height(), gg->get_default_step_time(), gg->get_toe_angle(), gg->get_heel_angle())));
        fnsl.push_back(boost::assign::list_of(step_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(300*1e-3, 0, 300*1e-3)+leg_pos[1])), gg->get_default_step_height(), gg->get_default_step_time(), gg->get_toe_angle(), gg->get_heel_angle())));
        gg->set_foot_steps_list(fnsl);
        gen_and_plot_walk_pattern();
    };

    void test6 ()
    {
        test_doc_string = "test6 : Go single step";
        parse_params();
        gg->clear_footstep_nodes_list();
        std::vector< std::vector<step_node> > fnsl;
        fnsl.push_back(boost::assign::list_of(step_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(0, 0, 0)+leg_pos[0])), gg->get_default_step_height(), gg->get_default_step_time(), gg->get_toe_angle(), gg->get_heel_angle())));
        fnsl.push_back(boost::assign::list_of(step_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(100*1e-3, 75*1e-3, 0)+leg_pos[1]), hrp::rotFromRpy(hrp::Vector3(deg2rad(5), deg2rad(-20), deg2rad(10)))),
                                                        gg->get_default_step_height(), gg->get_default_step_time(), gg->get_toe_angle(), gg->get_heel_angle())));
        gg->set_foot_steps_list(fnsl);
        gen_and_plot_walk_pattern();
    };

    void test7 ()
    {
        test_doc_string = "test7 : Toe heel walk";
        /* initialize sample footstep_list */
        parse_params();
        std::vector<hrp::Vector3> dzo;
        dzo.push_back(hrp::Vector3(20*1e-3,-30*1e-3,0));
        dzo.push_back(hrp::Vector3(20*1e-3,30*1e-3,0));
        gg->set_default_zmp_offsets(dzo);
        gg->set_toe_zmp_offset_x(137*1e-3);
        gg->set_heel_zmp_offset_x(-105*1e-3);
        gg->set_toe_pos_offset_x(137*1e-3);
        gg->set_heel_pos_offset_x(-105*1e-3);
        gg->set_stride_parameters(0.2,0.1,20,0.2,0.1*0.5,20*0.5);
        gg->set_use_toe_heel_auto_set(true);
        gg->set_toe_angle(30);
        gg->set_heel_angle(10);
        // gg->set_use_toe_heel_transition(false);
        gg->set_use_toe_heel_transition(true);
        gg->clear_footstep_nodes_list();
        coordinates start_ref_coords;
        mid_coords(start_ref_coords, 0.5, coordinates(leg_pos[1]), coordinates(leg_pos[0]));
        gg->go_pos_param_2_footstep_nodes_list(400*1e-3, 0, 0, boost::assign::list_of(coordinates(leg_pos[1])), start_ref_coords, boost::assign::list_of(LLEG));
        gen_and_plot_walk_pattern();
    };

    void test8 ()
    {
        test_doc_string = "test8 : Toe heel walk on slope";
        /* initialize sample footstep_list */
        parse_params();
        std::vector<hrp::Vector3> dzo;
        dzo.push_back(hrp::Vector3(20*1e-3,-30*1e-3,0));
        dzo.push_back(hrp::Vector3(20*1e-3,30*1e-3,0));
        gg->set_default_zmp_offsets(dzo);
        gg->set_toe_zmp_offset_x(137*1e-3);
        gg->set_heel_zmp_offset_x(-105*1e-3);
        gg->set_toe_pos_offset_x(137*1e-3);
        gg->set_heel_pos_offset_x(-105*1e-3);
        gg->set_toe_angle(30);
        gg->set_heel_angle(10);
        // gg->set_use_toe_heel_transition(false);
        gg->set_use_toe_heel_transition(true);
        gg->clear_footstep_nodes_list();
        hrp::Matrix33 initial_foot_mid_rot = Eigen::AngleAxis<double>(M_PI/2, hrp::Vector3::UnitZ()).toRotationMatrix();
        //hrp::Matrix33 initial_foot_mid_rot = Eigen::AngleAxis<double>(M_PI, hrp::Vector3::UnitZ()).toRotationMatrix();
        coordinates start_ref_coords;
        mid_coords(start_ref_coords, 0.5, coordinates(initial_foot_mid_rot*leg_pos[1], initial_foot_mid_rot), coordinates(initial_foot_mid_rot*leg_pos[0], initial_foot_mid_rot));
        gg->go_pos_param_2_footstep_nodes_list(100*1e-3, 0, 0, boost::assign::list_of(coordinates(initial_foot_mid_rot*leg_pos[1], initial_foot_mid_rot)), start_ref_coords, boost::assign::list_of(LLEG));

        std::vector<std::string> tmp_string_vector = boost::assign::list_of("rleg");
        if (gg->get_footstep_front_leg_names() == tmp_string_vector) {
            step_node initial_support_leg_step = step_node(LLEG, coordinates(hrp::Vector3(initial_foot_mid_rot * leg_pos[1]), initial_foot_mid_rot), 0, 0, 0, 0);
            step_node initial_swing_leg_dst_step = step_node(RLEG, coordinates(hrp::Vector3(initial_foot_mid_rot * leg_pos[0]), initial_foot_mid_rot), 0, 0, 0, 0);
            gen_and_plot_walk_pattern(initial_support_leg_step, initial_swing_leg_dst_step);
        } else {
            step_node initial_support_leg_step = step_node(RLEG, coordinates(hrp::Vector3(initial_foot_mid_rot * leg_pos[0]), initial_foot_mid_rot), 0, 0, 0, 0);
            step_node initial_swing_leg_dst_step = step_node(LLEG, coordinates(hrp::Vector3(initial_foot_mid_rot * leg_pos[1]), initial_foot_mid_rot), 0, 0, 0, 0);
            gen_and_plot_walk_pattern(initial_support_leg_step, initial_swing_leg_dst_step);
        }
    };

    void test9 ()
    {
        test_doc_string = "test9 : Stair walk";
        /* initialize sample footstep_list */
        parse_params();
        gg->clear_footstep_nodes_list();
        gg->set_default_orbit_type(STAIR);
        gg->set_swing_trajectory_delay_time_offset (0.2);
        std::vector< std::vector<step_node> > fnsl;
        fnsl.push_back(boost::assign::list_of(step_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(0, 0, 0)+leg_pos[0])), gg->get_default_step_height(), gg->get_default_step_time(), gg->get_toe_angle(), gg->get_heel_angle())));
        fnsl.push_back(boost::assign::list_of(step_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(0, 0, 0)+leg_pos[1])), gg->get_default_step_height(), gg->get_default_step_time(), gg->get_toe_angle(), gg->get_heel_angle())));
        fnsl.push_back(boost::assign::list_of(step_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(250*1e-3, 0, 200*1e-3)+leg_pos[0])), gg->get_default_step_height(), gg->get_default_step_time(), gg->get_toe_angle(), gg->get_heel_angle())));
        fnsl.push_back(boost::assign::list_of(step_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(250*1e-3, 0, 200*1e-3)+leg_pos[1])), gg->get_default_step_height(), gg->get_default_step_time(), gg->get_toe_angle(), gg->get_heel_angle())));
        fnsl.push_back(boost::assign::list_of(step_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(500*1e-3, 0, 400*1e-3)+leg_pos[0])), gg->get_default_step_height(), gg->get_default_step_time(), gg->get_toe_angle(), gg->get_heel_angle())));
        fnsl.push_back(boost::assign::list_of(step_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(500*1e-3, 0, 400*1e-3)+leg_pos[1])), gg->get_default_step_height(), gg->get_default_step_time(), gg->get_toe_angle(), gg->get_heel_angle())));
        fnsl.push_back(boost::assign::list_of(step_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(750*1e-3, 0, 600*1e-3)+leg_pos[0])), gg->get_default_step_height(), gg->get_default_step_time(), gg->get_toe_angle(), gg->get_heel_angle())));
        fnsl.push_back(boost::assign::list_of(step_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(750*1e-3, 0, 600*1e-3)+leg_pos[1])), gg->get_default_step_height(), gg->get_default_step_time(), gg->get_toe_angle(), gg->get_heel_angle())));
        gg->set_foot_steps_list(fnsl);
        gen_and_plot_walk_pattern();
    };

    void test10 ()
    {
        test_doc_string = "test10 : Stair walk + toe heel contact";
        /* initialize sample footstep_list */
        parse_params();
        gg->clear_footstep_nodes_list();
        gg->set_default_orbit_type(STAIR);
        gg->set_swing_trajectory_delay_time_offset (0.2);
        gg->set_toe_zmp_offset_x(137*1e-3);
        gg->set_heel_zmp_offset_x(-105*1e-3);
        gg->set_toe_pos_offset_x(137*1e-3);
        gg->set_heel_pos_offset_x(-105*1e-3);
        gg->set_toe_angle(20);
        gg->set_heel_angle(5);
        gg->set_default_step_time(1.5);
        gg->set_default_double_support_ratio_before(0.1);
        gg->set_default_double_support_ratio_after(0.1);
        double ratio[7] = {0.02, 0.28, 0.2, 0.0, 0.2, 0.25, 0.05};
        std::vector<double> ratio2(ratio, ratio+gg->get_NUM_TH_PHASES());
        gg->set_toe_heel_phase_ratio(ratio2);
        std::vector< std::vector<step_node> > fnsl;
        fnsl.push_back(boost::assign::list_of(step_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(0, 0, 0)+leg_pos[0])), gg->get_default_step_height(), gg->get_default_step_time(), gg->get_toe_angle(), gg->get_heel_angle())));
        fnsl.push_back(boost::assign::list_of(step_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(0, 0, 0)+leg_pos[1])), gg->get_default_step_height(), gg->get_default_step_time(), gg->get_toe_angle(), gg->get_heel_angle())));
        fnsl.push_back(boost::assign::list_of(step_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(250*1e-3, 0, 200*1e-3)+leg_pos[0])), gg->get_default_step_height(), gg->get_default_step_time(), gg->get_toe_angle(), gg->get_heel_angle())));
        fnsl.push_back(boost::assign::list_of(step_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(250*1e-3, 0, 200*1e-3)+leg_pos[1])), gg->get_default_step_height(), gg->get_default_step_time(), gg->get_toe_angle(), gg->get_heel_angle())));
        fnsl.push_back(boost::assign::list_of(step_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(500*1e-3, 0, 400*1e-3)+leg_pos[0])), gg->get_default_step_height(), gg->get_default_step_time(), gg->get_toe_angle(), gg->get_heel_angle())));
        fnsl.push_back(boost::assign::list_of(step_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(500*1e-3, 0, 400*1e-3)+leg_pos[1])), gg->get_default_step_height(), gg->get_default_step_time(), gg->get_toe_angle(), gg->get_heel_angle())));
        fnsl.push_back(boost::assign::list_of(step_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(750*1e-3, 0, 600*1e-3)+leg_pos[0])), gg->get_default_step_height(), gg->get_default_step_time(), gg->get_toe_angle(), gg->get_heel_angle())));
        fnsl.push_back(boost::assign::list_of(step_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(750*1e-3, 0, 600*1e-3)+leg_pos[1])), gg->get_default_step_height(), gg->get_default_step_time(), gg->get_toe_angle(), gg->get_heel_angle())));
        gg->set_foot_steps_list(fnsl);
        gen_and_plot_walk_pattern();
    };

    void test11 ()
    {
        test_doc_string = "test11 : Foot rot change";
        /* initialize sample footstep_list */
        parse_params();
        hrp::Matrix33 tmpr;
        std::vector< std::vector<step_node> > fnsl;
        fnsl.push_back(boost::assign::list_of(step_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(0, 0, 0)+leg_pos[1])), gg->get_default_step_height(), gg->get_default_step_time(), gg->get_toe_angle(), gg->get_heel_angle())));
        tmpr = hrp::rotFromRpy(5*M_PI/180.0, 15*M_PI/180.0, 0);
        fnsl.push_back(boost::assign::list_of(step_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(250*1e-3, 0, 0*1e-3)+leg_pos[0]), tmpr), gg->get_default_step_height(), gg->get_default_step_time(), gg->get_toe_angle(), gg->get_heel_angle())));
        tmpr = hrp::rotFromRpy(-5*M_PI/180.0, -15*M_PI/180.0, 0);
        fnsl.push_back(boost::assign::list_of(step_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(250*1e-3, 0, 0*1e-3)+leg_pos[1]), tmpr), gg->get_default_step_height(), gg->get_default_step_time(), gg->get_toe_angle(), gg->get_heel_angle())));
        gg->set_foot_steps_list(fnsl);
        gen_and_plot_walk_pattern();
    };

    void test12 ()
    {
        test_doc_string = "test12 : Change step param in set foot steps";
        /* initialize sample footstep_list */
        parse_params();
        std::vector< std::vector<step_node> > fnsl;
        gg->set_default_step_time(4.0); // dummy
        fnsl.push_back(boost::assign::list_of(step_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(0, 0, 0)+leg_pos[0])), gg->get_default_step_height(), 1.0, 0, 0)));
        fnsl.push_back(boost::assign::list_of(step_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(0, 0, 0)+leg_pos[1])), gg->get_default_step_height(), 2.0, 0, 0)));
        fnsl.push_back(boost::assign::list_of(step_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(100*1e-3, 0, 0)+leg_pos[0])), gg->get_default_step_height()*2, 1.5, 0, 0)));
        fnsl.push_back(boost::assign::list_of(step_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(200*1e-3, 0, 0)+leg_pos[1])), gg->get_default_step_height(), 2.5, 0, 0)));
        fnsl.push_back(boost::assign::list_of(step_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(300*1e-3, 0, 0)+leg_pos[0])), gg->get_default_step_height(), 1.0, 20, 5)));
        fnsl.push_back(boost::assign::list_of(step_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(300*1e-3, 0, 0)+leg_pos[1])), gg->get_default_step_height(), 2.0, 0, 0)));
        gg->set_foot_steps_list(fnsl);
        gen_and_plot_walk_pattern();
    };

    void test13 ()
    {
        test_doc_string = "test13 : Arbitrary leg switching";
        /* initialize sample footstep_list */
        parse_params();
        std::vector< std::vector<step_node> > fnsl;
        fnsl.push_back(boost::assign::list_of(step_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(0, 0, 0)+leg_pos[0])), gg->get_default_step_height(), gg->get_default_step_time(), 0, 0)));
        fnsl.push_back(boost::assign::list_of(step_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(0, 0, 0)+leg_pos[1])), gg->get_default_step_height(), gg->get_default_step_time(), 0, 0)));
        fnsl.push_back(boost::assign::list_of(step_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(100*1e-3, 0, 0)+leg_pos[1])), gg->get_default_step_height(), gg->get_default_step_time(), 0, 0)));
        fnsl.push_back(boost::assign::list_of(step_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(100*1e-3, 0, 0)+leg_pos[0])), gg->get_default_step_height(), gg->get_default_step_time(), 0, 0)));
        fnsl.push_back(boost::assign::list_of(step_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(200*1e-3, 0, 0)+leg_pos[0])), gg->get_default_step_height(), gg->get_default_step_time(), 0, 0)));
        fnsl.push_back(boost::assign::list_of(step_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(200*1e-3, 0, 0)+leg_pos[1])), gg->get_default_step_height(), gg->get_default_step_time(), 0, 0)));
        gg->set_foot_steps_list(fnsl);
        gen_and_plot_walk_pattern();
    };

        void test14 ()
    {
        test_doc_string = "test14 : kick walk";
        /* initialize sample footstep_list */
        parse_params();
        gg->clear_footstep_nodes_list();
        gg->set_default_orbit_type(CYCLOIDDELAYKICK);
        coordinates start_ref_coords;
        mid_coords(start_ref_coords, 0.5, coordinates(leg_pos[1]), coordinates(leg_pos[0]));
        gg->go_pos_param_2_footstep_nodes_list(300*1e-3, 0, 0, boost::assign::list_of(coordinates(leg_pos[1])), start_ref_coords, boost::assign::list_of(LLEG));
        gen_and_plot_walk_pattern();
    };

    void test15 ()
    {
        test_doc_string = "test15 : Stair walk down";
        /* initialize sample footstep_list */
        parse_params();
        gg->clear_footstep_nodes_list();
        gg->set_default_orbit_type(STAIR);
        gg->set_swing_trajectory_delay_time_offset (0.2);
        std::vector< std::vector<step_node> > fnsl;
        fnsl.push_back(boost::assign::list_of(step_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(0, 0, 0)+leg_pos[0])), gg->get_default_step_height(), gg->get_default_step_time(), gg->get_toe_angle(), gg->get_heel_angle())));
        fnsl.push_back(boost::assign::list_of(step_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(0, 0, 0)+leg_pos[1])), gg->get_default_step_height(), gg->get_default_step_time(), gg->get_toe_angle(), gg->get_heel_angle())));
        fnsl.push_back(boost::assign::list_of(step_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(250*1e-3, 0, -200*1e-3)+leg_pos[0])), gg->get_default_step_height(), gg->get_default_step_time(), gg->get_toe_angle(), gg->get_heel_angle())));
        fnsl.push_back(boost::assign::list_of(step_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(250*1e-3, 0, -200*1e-3)+leg_pos[1])), gg->get_default_step_height(), gg->get_default_step_time(), gg->get_toe_angle(), gg->get_heel_angle())));
        fnsl.push_back(boost::assign::list_of(step_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(500*1e-3, 0, -400*1e-3)+leg_pos[0])), gg->get_default_step_height(), gg->get_default_step_time(), gg->get_toe_angle(), gg->get_heel_angle())));
        fnsl.push_back(boost::assign::list_of(step_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(500*1e-3, 0, -400*1e-3)+leg_pos[1])), gg->get_default_step_height(), gg->get_default_step_time(), gg->get_toe_angle(), gg->get_heel_angle())));
        fnsl.push_back(boost::assign::list_of(step_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(750*1e-3, 0, -600*1e-3)+leg_pos[0])), gg->get_default_step_height(), gg->get_default_step_time(), gg->get_toe_angle(), gg->get_heel_angle())));
        fnsl.push_back(boost::assign::list_of(step_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(750*1e-3, 0, -600*1e-3)+leg_pos[1])), gg->get_default_step_height(), gg->get_default_step_time(), gg->get_toe_angle(), gg->get_heel_angle())));
        gg->set_foot_steps_list(fnsl);
        gen_and_plot_walk_pattern();
    };

    void test16 ()
    {
        test_doc_string = "test16 : Set foot steps with param (toe heel contact)";
        /* initialize sample footstep_list */
        parse_params();
        gg->clear_footstep_nodes_list();
        std::vector<hrp::Vector3> dzo;
        dzo.push_back(hrp::Vector3(20*1e-3,-30*1e-3,0));
        dzo.push_back(hrp::Vector3(20*1e-3,30*1e-3,0));
        gg->set_default_zmp_offsets(dzo);
        gg->set_toe_zmp_offset_x(137*1e-3);
        gg->set_heel_zmp_offset_x(-105*1e-3);
        gg->set_toe_pos_offset_x(137*1e-3);
        gg->set_heel_pos_offset_x(-105*1e-3);
        gg->set_toe_angle(20);
        gg->set_heel_angle(5);
        gg->set_default_step_time(2);
        gg->set_default_double_support_ratio_before(0.1);
        gg->set_default_double_support_ratio_after(0.1);
        gg->set_use_toe_heel_auto_set(true);
        gg->set_use_toe_heel_transition(true);
        //double ratio[7] = {0.02, 0.28, 0.2, 0.0, 0.2, 0.25, 0.05};
        double ratio[7] = {0.07, 0.20, 0.2, 0.0, 0.2, 0.25, 0.08};
        std::vector<double> ratio2(ratio, ratio+gg->get_NUM_TH_PHASES());
        gg->set_toe_heel_phase_ratio(ratio2);
        std::vector< std::vector<step_node> > fnsl;
        fnsl.push_back(boost::assign::list_of(step_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(0, 0, 0)+leg_pos[0])), gg->get_default_step_height()*0.5, gg->get_default_step_time()*0.5, gg->get_toe_angle()*0.5, gg->get_heel_angle()*0.5)));
        fnsl.push_back(boost::assign::list_of(step_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(100*1e-3, 0, 0)+leg_pos[1])), gg->get_default_step_height()*2.0, gg->get_default_step_time()*2.0, gg->get_toe_angle()*2.0, gg->get_heel_angle()*2.0)));
        fnsl.push_back(boost::assign::list_of(step_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(400*1e-3, 0, 0)+leg_pos[0])), gg->get_default_step_height()*0.5, gg->get_default_step_time()*0.5, gg->get_toe_angle()*0.5, gg->get_heel_angle()*0.5)));
        fnsl.push_back(boost::assign::list_of(step_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(400*1e-3, 0, 0)+leg_pos[1])), gg->get_default_step_height()*2.0, gg->get_default_step_time()*2.0, gg->get_toe_angle()*2.0, gg->get_heel_angle()*2.0)));
        gg->set_foot_steps_list(fnsl);
        gen_and_plot_walk_pattern();
    };

    void test17 ()
    {
        test_doc_string = "test17 : Test goVelocity (dx = 0.1, dy = 0.05, dth = 10.0)";
        /* initialize sample footstep_list */
        parse_params();
        gg->clear_footstep_nodes_list();
        gg->print_param();
        step_node initial_support_leg_step = step_node(LLEG, coordinates(leg_pos[1]), 0, 0, 0, 0);
        step_node initial_swing_leg_dst_step = step_node(RLEG, coordinates(leg_pos[0]), 0, 0, 0, 0);
        coordinates fm_coords;
        mid_coords(fm_coords, 0.5, initial_swing_leg_dst_step.worldcoords, initial_support_leg_step.worldcoords);
        gg->initialize_velocity_mode(fm_coords, 0.1, 0.05, 10.0, boost::assign::list_of(RLEG));
        gg->initialize_gait_parameter(cog, boost::assign::list_of(initial_support_leg_step), boost::assign::list_of(initial_swing_leg_dst_step));
        while ( !gg->proc_one_tick() );
        /* make step and dump */
        check_start_values();
        size_t i = 0;
        while ( gg->proc_one_tick() ) {
            proc_one_walking_motion(i);
            i++;
            if ( i > static_cast<size_t>(gg->get_default_step_time()/dt) && gg->get_overwrite_check_timing() ) {
                gg->finalize_velocity_mode();
            }
        }
        check_end_values();
        plot_and_print_errorcheck ();
    };

    void test18 ()
    {
        test_doc_string = "test18 : Test goVelocity with changing velocity (translation and rotation)";
        /* initialize sample footstep_list */
        parse_params();
        gg->clear_footstep_nodes_list();
        gg->set_overwritable_footstep_index_offset(0);
        gg->set_default_orbit_type(CYCLOIDDELAY);
        gg->print_param();
        step_node initial_support_leg_step = step_node(LLEG, coordinates(leg_pos[1]), 0, 0, 0, 0);
        step_node initial_swing_leg_dst_step = step_node(RLEG, coordinates(leg_pos[0]), 0, 0, 0, 0);
        coordinates fm_coords;
        mid_coords(fm_coords, 0.5, initial_swing_leg_dst_step.worldcoords, initial_support_leg_step.worldcoords);
        gg->initialize_velocity_mode(fm_coords, 0, 0, 0, boost::assign::list_of(RLEG));
        gg->initialize_gait_parameter(cog, boost::assign::list_of(initial_support_leg_step), boost::assign::list_of(initial_swing_leg_dst_step));
        while ( !gg->proc_one_tick() );
        /* make step and dump */
        check_start_values();
        size_t i = 0;
        while ( gg->proc_one_tick() ) {
            proc_one_walking_motion(i);
            i++;
            if ( i > static_cast<size_t>(gg->get_default_step_time()/dt)*5 && gg->get_overwrite_check_timing() ) {
                gg->finalize_velocity_mode();
            } else if ( i > static_cast<size_t>(gg->get_default_step_time()/dt)*4 && gg->get_overwrite_check_timing() ) {
                gg->set_velocity_param(0, 0, 0);
            } else if ( i > static_cast<size_t>(gg->get_default_step_time()/dt)*3 && gg->get_overwrite_check_timing() ) {
                gg->set_velocity_param(0, 0, 10);
            } else if ( i > static_cast<size_t>(gg->get_default_step_time()/dt)*2 && gg->get_overwrite_check_timing() ) {
                gg->set_velocity_param(0, 0, 0);
            } else if ( i > static_cast<size_t>(gg->get_default_step_time()/dt) && gg->get_overwrite_check_timing() ) {
                gg->set_velocity_param(0.1, 0.05, 0);
            }
        }
        check_end_values();
        plot_and_print_errorcheck ();
    };

    void test19 ()
    {
        test_doc_string = "test19 : Change stride parameter (translate)";
        /* initialize sample footstep_list */
        parse_params();
        std::vector<hrp::Vector3> dzo;
        dzo.push_back(hrp::Vector3(20*1e-3,-30*1e-3,0));
        dzo.push_back(hrp::Vector3(20*1e-3,30*1e-3,0));
        gg->set_default_zmp_offsets(dzo);
        gg->set_stride_parameters(0.25,0.15,25,0.25,0.1,10);
        gg->clear_footstep_nodes_list();
        coordinates start_ref_coords;
        mid_coords(start_ref_coords, 0.5, coordinates(leg_pos[1]), coordinates(leg_pos[0]));
        gg->go_pos_param_2_footstep_nodes_list(600*1e-3, -300*1e-3, 0, boost::assign::list_of(coordinates(leg_pos[1])), start_ref_coords, boost::assign::list_of(LLEG));
        gen_and_plot_walk_pattern();
    };

    void test20 ()
    {
        test_doc_string = "test20 : Change stride parameter (translate+rotate)";
        /* initialize sample footstep_list */
        parse_params();
        std::vector<hrp::Vector3> dzo;
        dzo.push_back(hrp::Vector3(20*1e-3,-30*1e-3,0));
        dzo.push_back(hrp::Vector3(20*1e-3,30*1e-3,0));
        gg->set_default_zmp_offsets(dzo);
        gg->set_stride_parameters(0.25,0.15,25,0.25,0.1,10);
        gg->clear_footstep_nodes_list();
        coordinates start_ref_coords;
        mid_coords(start_ref_coords, 0.5, coordinates(leg_pos[1]), coordinates(leg_pos[0]));
        gg->go_pos_param_2_footstep_nodes_list(400*1e-3, -200*1e-3, -55, boost::assign::list_of(coordinates(leg_pos[1])), start_ref_coords, boost::assign::list_of(LLEG));
        gen_and_plot_walk_pattern();
    };

    void parse_params (bool is_print_doc_setring = true)
    {
      if (is_print_doc_setring) std::cerr << test_doc_string << std::endl;
      for (unsigned int i = 0; i < arg_strs.size(); ++ i) {
          if ( arg_strs[i]== "--default-step-time" ) {
              if (++i < arg_strs.size()) gg->set_default_step_time(atof(arg_strs[i].c_str()));
          } else if ( arg_strs[i]== "--default-step-height" ) {
              if (++i < arg_strs.size()) gg->set_default_step_height(atof(arg_strs[i].c_str()));
          } else if ( arg_strs[i]== "--default-double-support-ratio-before" ) {
              if (++i < arg_strs.size()) gg->set_default_double_support_ratio_before(atof(arg_strs[i].c_str()));
          } else if ( arg_strs[i]== "--default-double-support-ratio-after" ) {
              if (++i < arg_strs.size()) gg->set_default_double_support_ratio_after(atof(arg_strs[i].c_str()));
          } else if ( arg_strs[i]== "--default-orbit-type" ) {
              if (++i < arg_strs.size()) {
                  if (arg_strs[i] == "SHUFFLING") {
                      gg->set_default_orbit_type(SHUFFLING);
                  } else if (arg_strs[i] == "CYCLOID") {
                      gg->set_default_orbit_type(CYCLOID);
                  } else if (arg_strs[i] == "RECTANGLE") {
                      gg->set_default_orbit_type(RECTANGLE);
                  } else if (arg_strs[i] == "STAIR") {
                      gg->set_default_orbit_type(STAIR);
                  } else if (arg_strs[i] == "CYCLOIDDELAY") {
                      gg->set_default_orbit_type(CYCLOIDDELAY);
                  } else if (arg_strs[i] == "CYCLOIDDELAYKICK") {
                      gg->set_default_orbit_type(CYCLOIDDELAYKICK);
                  } else if (arg_strs[i] == "CROSS") {
                      gg->set_default_orbit_type(CROSS);
                  } else {
                      std::cerr << "No such default-orbit-type " << arg_strs[i] << std::endl;
                  }
              }
          } else if ( arg_strs[i]== "--default-double-support-static-ratio-before" ) {
              if (++i < arg_strs.size()) gg->set_default_double_support_static_ratio_before(atof(arg_strs[i].c_str()));
          } else if ( arg_strs[i]== "--default-double-support-static-ratio-after" ) {
              if (++i < arg_strs.size()) gg->set_default_double_support_static_ratio_after(atof(arg_strs[i].c_str()));
          } else if ( arg_strs[i]== "--default-double-support-ratio-swing-before" ) {
              if (++i < arg_strs.size()) gg->set_default_double_support_ratio_swing_before(atof(arg_strs[i].c_str()));
          } else if ( arg_strs[i]== "--default-double-support-ratio-swing-after" ) {
              if (++i < arg_strs.size()) gg->set_default_double_support_ratio_swing_after(atof(arg_strs[i].c_str()));
          } else if ( arg_strs[i]== "--swing-trajectory-delay-time-offset" ) {
              if (++i < arg_strs.size()) gg->set_swing_trajectory_delay_time_offset(atof(arg_strs[i].c_str()));
          } else if ( arg_strs[i]== "--swing-trajectory-final-distance-weight" ) {
              if (++i < arg_strs.size()) gg->set_swing_trajectory_final_distance_weight(atof(arg_strs[i].c_str()));
          } else if ( arg_strs[i]== "--swing-trajectory-time-offset-xy2z" ) {
              if (++i < arg_strs.size()) gg->set_swing_trajectory_time_offset_xy2z(atof(arg_strs[i].c_str()));
          } else if ( arg_strs[i]== "--stair-trajectory-way-point-offset" ) {
              if (++i < arg_strs.size()) {
                  coil::vstring strs = coil::split(std::string(arg_strs[i].c_str()), ",");
                  gg->set_stair_trajectory_way_point_offset(hrp::Vector3(atof(strs[0].c_str()), atof(strs[1].c_str()), atof(strs[2].c_str())));
              }
          } else if ( arg_strs[i]== "--cycloid-delay-kick-point-offset" ) {
              if (++i < arg_strs.size()) {
                  coil::vstring strs = coil::split(std::string(arg_strs[i].c_str()), ",");
                  gg->set_cycloid_delay_kick_point_offset(hrp::Vector3(atof(strs[0].c_str()), atof(strs[1].c_str()), atof(strs[2].c_str())));
              }
          } else if ( arg_strs[i]== "--toe-angle" ) {
              if (++i < arg_strs.size()) gg->set_toe_angle(atof(arg_strs[i].c_str()));
          } else if ( arg_strs[i]== "--heel-angle" ) {
              if (++i < arg_strs.size()) gg->set_heel_angle(atof(arg_strs[i].c_str()));
          } else if ( arg_strs[i]== "--toe-heel-phase-ratio" ) {
              if (++i < arg_strs.size()) {
                  coil::vstring strs = coil::split(std::string(arg_strs[i].c_str()), ",");
                  std::vector<double> ratio;
                  for (size_t i_th = 0; i_th < strs.size(); i_th++) {
                      ratio.push_back(atof(strs[i_th].c_str()));
                  }
                  std::cerr << "[]   "; // for set_toe_heel_phase_ratio
                  gg->set_toe_heel_phase_ratio(ratio);
              }
          } else if ( arg_strs[i]== "--optional-go-pos-finalize-footstep-num" ) {
              if (++i < arg_strs.size()) gg->set_optional_go_pos_finalize_footstep_num(atoi(arg_strs[i].c_str()));
          } else if ( arg_strs[i]== "--use-gnuplot" ) {
              if (++i < arg_strs.size()) use_gnuplot = (arg_strs[i]=="true");
          } else if ( arg_strs[i]== "--use-graph-append" ) {
              if (++i < arg_strs.size()) use_graph_append = (arg_strs[i]=="true");
          }
      }   
    };

    bool check_all_results () const
    {
        return refzmp_diff_checker.isSmallDiff() && cartzmp_diff_checker.isSmallDiff() && cog_diff_checker.isSmallDiff()
            && ssmcpos_diff_checker.isSmallDiff() && ssmcrot_diff_checker.isSmallDiff()
            && ssmcposvel_diff_checker.isSmallDiff() && ssmcrotvel_diff_checker.isSmallDiff()
            && footpos_diff_checker.isSmallDiff() && footrot_diff_checker.isSmallDiff()
            && zmpoffset_diff_checker.isSmallDiff()
            && footposvel_diff_checker.isSmallDiff() && footrotvel_diff_checker.isSmallDiff()
            && zmp_error_checker.isSmallError() && cogzmp_error_checker.isSmallError()
            && is_step_time_valid && is_toe_heel_dif_angle_valid && is_toe_heel_zmp_offset_x_valid
            && is_contact_states_swing_support_time_validity;
    };
};

class testGaitGeneratorHRP2JSK : public testGaitGenerator
{
 public:
    testGaitGeneratorHRP2JSK () : testGaitGenerator(0.004)
        {
            cog = 1e-3*hrp::Vector3(6.785, 1.54359, 806.831);
            leg_pos.push_back(hrp::Vector3(0,1e-3*-105,0)); /* rleg */
            leg_pos.push_back(hrp::Vector3(0,1e-3* 105,0)); /* lleg */
            all_limbs.push_back("rleg");
            all_limbs.push_back("lleg");
            gg = new gait_generator(dt, leg_pos, all_limbs, 1e-3*150, 1e-3*50, 10, 1e-3*50, 1e-3*50*0.5, 10*0.5);
        };
};

void print_usage ()
{
    std::cerr << "Usage : testGaitGenerator [test-name] [option]" << std::endl;
    std::cerr << " [test-name] should be:" << std::endl;
    std::cerr << "  --test0 : Set foot steps" << std::endl;
    std::cerr << "  --test1 : Go pos x,y,th combination" << std::endl;
    std::cerr << "  --test2 : Go pos x" << std::endl;
    std::cerr << "  --test3 : Go pos y" << std::endl;
    std::cerr << "  --test4 : Go pos th" << std::endl;
    std::cerr << "  --test5 : Set foot steps with Z change" << std::endl;
    std::cerr << "  --test6 : Go single step" << std::endl;
    std::cerr << "  --test7 : Toe heel walk" << std::endl;
    std::cerr << "  --test8 : Toe heel walk on slope" << std::endl;
    std::cerr << "  --test9 : Stair walk" << std::endl;
    std::cerr << "  --test10 : Stair walk + toe heel contact" << std::endl;
    std::cerr << "  --test11 : Foot rot change" << std::endl;
    std::cerr << "  --test12 : Change step param in set foot steps" << std::endl;
    std::cerr << "  --test13 : Arbitrary leg switching" << std::endl;
    std::cerr << "  --test14 : kick walk" << std::endl;
    std::cerr << "  --test15 : Stair walk down" << std::endl;
    std::cerr << "  --test16 : Set foot steps with param (toe heel contact)" << std::endl;
    std::cerr << "  --test17 : Test goVelocity (dx = 0.1, dy = 0.05, dth = 10.0)" << std::endl;
    std::cerr << "  --test18 : Test goVelocity with changing velocity (translation and rotation)" << std::endl;
    std::cerr << "  --test19 : Change stride parameter (translate)" << std::endl;
    std::cerr << "  --test20 : Change stride parameter (translate+rotate)" << std::endl;
    std::cerr << " [option] should be:" << std::endl;
    std::cerr << "  --use-gnuplot : Use gnuplot and dump eps file to /tmp. (true/false, true by default)" << std::endl;
    std::cerr << "  --use-graph-append : Append generated graph to /tmp/testGaitGenerator.jpg. (true/false, false by default)" << std::endl;
    std::cerr << "  other options : for GaitGenerator" << std::endl;
};

int main(int argc, char* argv[])
{
  int ret = 0;
  if (argc >= 2) {
      testGaitGeneratorHRP2JSK tgg;
      for (int i = 1; i < argc; ++ i) {
          tgg.arg_strs.push_back(std::string(argv[i]));
      }
      if (std::string(argv[1]) == "--test0") {
          tgg.test0();
      } else if (std::string(argv[1]) == "--test1") {
          tgg.test1();
      } else if (std::string(argv[1]) == "--test2") {
          tgg.test2();
      } else if (std::string(argv[1]) == "--test3") {
          tgg.test3();
      } else if (std::string(argv[1]) == "--test4") {
          tgg.test4();
      } else if (std::string(argv[1]) == "--test5") {
          tgg.test5();
      } else if (std::string(argv[1]) == "--test6") {
          tgg.test6();
      } else if (std::string(argv[1]) == "--test7") {
          tgg.test7();
      } else if (std::string(argv[1]) == "--test8") {
          tgg.test8();
      } else if (std::string(argv[1]) == "--test9") {
          tgg.test9();
      } else if (std::string(argv[1]) == "--test10") {
          tgg.test10();
      } else if (std::string(argv[1]) == "--test11") {
          tgg.test11();
      } else if (std::string(argv[1]) == "--test12") {
          tgg.test12();
      } else if (std::string(argv[1]) == "--test13") {
          tgg.test13();
      } else if (std::string(argv[1]) == "--test14") {
          tgg.test14();
      } else if (std::string(argv[1]) == "--test15") {
          tgg.test15();
      } else if (std::string(argv[1]) == "--test16") {
          tgg.test16();
      } else if (std::string(argv[1]) == "--test17") {
          tgg.test17();
      } else if (std::string(argv[1]) == "--test18") {
          tgg.test18();
      } else if (std::string(argv[1]) == "--test19") {
          tgg.test19();
      } else if (std::string(argv[1]) == "--test20") {
          tgg.test20();
      } else {
          print_usage();
          ret = 1;
      }
      ret = (tgg.check_all_results()?0:2);
  } else {
      print_usage();
      ret = 1;
  }
  return ret;
}

