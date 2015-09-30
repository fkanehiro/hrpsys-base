/* -*- coding:utf-8-unix; mode:c++; -*- */

#include "GaitGenerator.h"
/* samples */
using namespace rats;
#include <cstdio>
#include <coil/stringutil.h>

#define eps_eq(a,b,epsilon) (std::fabs((a)-(b)) < (epsilon))

class testGaitGenerator
{
protected:
    double dt; /* [s] */
    std::vector<hrp::Vector3> leg_pos; /* default footstep transformations are necessary */
    std::vector<std::string> all_limbs;
    hrp::Vector3 cog;
    gait_generator* gg;
    bool use_gnuplot, is_small_zmp_error, is_small_zmp_diff, is_contact_states_swing_support_time_validity;
private:
    // error check
    bool check_zmp_error (const hrp::Vector3& czmp, const hrp::Vector3& refzmp)
    {
        return (czmp-refzmp).norm() < 50.0*1e-3; // [mm]
    }
    bool check_zmp_diff (const hrp::Vector3& prev_zmp, const hrp::Vector3& zmp)
    {
        return (prev_zmp - zmp).norm() < 10.0*1e-3; // [mm]
    }
    // plot and pattern generation
    void plot_and_save (FILE* gp, const std::string graph_fname, const std::string plot_str)
    {
        fprintf(gp, "%s\n unset multiplot\n", plot_str.c_str());
        fprintf(gp, "set terminal postscript eps color\nset output '/tmp/%s.eps'\n", graph_fname.c_str());
        fprintf(gp, "%s\n unset multiplot\n", plot_str.c_str());
        fflush(gp);
    };
    void plot_walk_pattern ()
    {
        /* make step and dump */
        size_t i = 0;
        std::string fname("/tmp/plot.dat");
        FILE* fp = fopen(fname.c_str(), "w");
        std::string fname_sstime("/tmp/plot-sstime.dat");
        FILE* fp_sstime = fopen(fname_sstime.c_str(), "w");
        hrp::Vector3 prev_rfoot_pos, prev_lfoot_pos;
        hrp::Vector3 min_rfoot_pos(1e10,1e10,1e10), min_lfoot_pos(1e10,1e10,1e10), max_rfoot_pos(-1e10,-1e10,-1e10), max_lfoot_pos(-1e10,-1e10,-1e10);
        //
        hrp::Vector3 prev_refzmp;
        std::vector<std::string> tmp_string_vector;
        std::vector<bool> prev_contact_states(2, true); // RLEG, LLEG
        std::vector<double> prev_swing_support_time(2, 1e2); // RLEG, LLEG
        while ( gg->proc_one_tick() ) {
            //std::cerr << gg->lcg.gp_count << std::endl;
            // if ( gg->lcg.gp_index == 4 && gg->lcg.gp_count == 100) {
            //   //std::cerr << gg->lcg.gp_index << std::endl;
            //   gg->update_refzmp_queue(coordinates(hrp::Vector3(150, 105, 0)), coordinates(hrp::Vector3(150, -105, 0)));
            // }
            fprintf(fp, "%f ", i * dt);
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp, "%f ", gg->get_refzmp()(ii));
            }
            hrp::Vector3 czmp = gg->get_cart_zmp();
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp, "%f ", czmp(ii));
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
                fprintf(fp, "%f ", cogpos);
            }
            // Foot pos
            tmp_string_vector = boost::assign::list_of("rleg");
            hrp::Vector3 rfoot_pos = (gg->get_support_leg_names() == tmp_string_vector) ? gg->get_support_leg_steps().front().worldcoords.pos : gg->get_swing_leg_steps().front().worldcoords.pos;
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp, "%f ", rfoot_pos(ii));
                min_rfoot_pos(ii) = std::min(min_rfoot_pos(ii), rfoot_pos(ii));
                max_rfoot_pos(ii) = std::max(max_rfoot_pos(ii), rfoot_pos(ii));
            }
            tmp_string_vector = boost::assign::list_of("lleg");
            hrp::Vector3 lfoot_pos = (gg->get_support_leg_names() == tmp_string_vector) ? gg->get_support_leg_steps().front().worldcoords.pos : gg->get_swing_leg_steps().front().worldcoords.pos;
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp, "%f ", lfoot_pos(ii));
                min_lfoot_pos(ii) = std::min(min_lfoot_pos(ii), lfoot_pos(ii));
                max_lfoot_pos(ii) = std::max(max_lfoot_pos(ii), lfoot_pos(ii));
            }
            // Foot rot
            hrp::Vector3 rpy;
            tmp_string_vector = boost::assign::list_of("rleg");
            hrp::Matrix33 rfoot_rot = (gg->get_support_leg_names() == tmp_string_vector) ? gg->get_support_leg_steps().front().worldcoords.rot : gg->get_swing_leg_steps().front().worldcoords.rot;
            rpy = hrp::rpyFromRot(rfoot_rot);
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp, "%f ", 180.0*rpy(ii)/M_PI);
            }
            tmp_string_vector = boost::assign::list_of("lleg");
            hrp::Matrix33 lfoot_rot = (gg->get_support_leg_names() == tmp_string_vector) ? gg->get_support_leg_steps().front().worldcoords.rot : gg->get_swing_leg_steps().front().worldcoords.rot;
            rpy = hrp::rpyFromRot(lfoot_rot);
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp, "%f ", 180.0*rpy(ii)/M_PI);
            }
            // ZMP offsets
            tmp_string_vector = boost::assign::list_of("rleg");
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp, "%f ", (gg->get_support_leg_names() == tmp_string_vector) ? gg->get_support_foot_zmp_offsets().front()(ii) : gg->get_swing_foot_zmp_offsets().front()(ii));
            }
            tmp_string_vector = boost::assign::list_of("lleg");
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp, "%f ", (gg->get_support_leg_names() == tmp_string_vector) ? gg->get_support_foot_zmp_offsets().front()(ii) : gg->get_swing_foot_zmp_offsets().front()(ii));
            }
            // Foot vel
            hrp::Vector3 tmpv;
            if ( i == 0 ) prev_rfoot_pos = rfoot_pos;
            tmpv = (rfoot_pos - prev_rfoot_pos)/dt;
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp, "%f ", tmpv(ii));
            }
            prev_rfoot_pos = rfoot_pos;
            if ( i == 0 ) prev_lfoot_pos = lfoot_pos;
            tmpv = (lfoot_pos - prev_lfoot_pos)/dt;
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp, "%f ", tmpv(ii));
            }
            prev_lfoot_pos = lfoot_pos;
            // Toe heel pos
            hrp::Vector3 tmppos;
            tmppos = rfoot_pos+rfoot_rot*hrp::Vector3(gg->get_toe_pos_offset_x(), 0, 0);
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp, "%f ", tmppos(ii));
                min_rfoot_pos(ii) = std::min(min_rfoot_pos(ii), tmppos(ii));
                max_rfoot_pos(ii) = std::max(max_rfoot_pos(ii), tmppos(ii));
            }
            tmppos = lfoot_pos+lfoot_rot*hrp::Vector3(gg->get_toe_pos_offset_x(), 0, 0);
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp, "%f ", tmppos(ii));
                min_lfoot_pos(ii) = std::min(min_lfoot_pos(ii), tmppos(ii));
                max_lfoot_pos(ii) = std::max(max_lfoot_pos(ii), tmppos(ii));
            }
            tmppos = rfoot_pos+rfoot_rot*hrp::Vector3(gg->get_heel_pos_offset_x(), 0, 0);
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp, "%f ", tmppos(ii));
                min_rfoot_pos(ii) = std::min(min_rfoot_pos(ii), tmppos(ii));
                max_rfoot_pos(ii) = std::max(max_rfoot_pos(ii), tmppos(ii));
            }
            tmppos = lfoot_pos+lfoot_rot*hrp::Vector3(gg->get_heel_pos_offset_x(), 0, 0);
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp, "%f ", tmppos(ii));
                min_lfoot_pos(ii) = std::min(min_lfoot_pos(ii), tmppos(ii));
                max_lfoot_pos(ii) = std::max(max_lfoot_pos(ii), tmppos(ii));
            }
            fprintf(fp, "\n");
            // Swing time
            fprintf(fp_sstime, "%f ", i * dt);
            fprintf(fp_sstime, "%f %f ",
                    gg->get_current_swing_time(RLEG),
                    gg->get_current_swing_time(LLEG));
            std::vector<leg_type> tmp_current_support_states = gg->get_current_support_states();
            bool rleg_contact_states = std::find_if(tmp_current_support_states.begin(), tmp_current_support_states.end(), boost::lambda::_1 == RLEG) != tmp_current_support_states.end();
            bool lleg_contact_states = std::find_if(tmp_current_support_states.begin(), tmp_current_support_states.end(), boost::lambda::_1 == LLEG) != tmp_current_support_states.end();
            fprintf(fp_sstime, "%d %d ", (rleg_contact_states ? 1 : 0), (lleg_contact_states ? 1 : 0));
            fprintf(fp_sstime, "\n");
            // Error checking
            is_small_zmp_error = check_zmp_error(gg->get_cart_zmp(), gg->get_refzmp()) && is_small_zmp_error;
            if (i>0) {
                is_small_zmp_diff = check_zmp_diff(prev_refzmp, gg->get_refzmp()) && is_small_zmp_diff;
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
            i++;
        }
        fclose(fp);
        fclose(fp_sstime);

        /* plot */
        if (use_gnuplot) {
            size_t gpsize = 7;
            size_t tmp_start = 2;
            FILE* gps[gpsize];
            for (size_t ii = 0; ii < gpsize;ii++) {
                gps[ii] = popen("gnuplot", "w");
            }
            {
                std::ostringstream oss("");
                std::string gtitle("COG_and_ZMP");
                oss << "set multiplot layout 3, 1 title '" << gtitle << "'" << std::endl;
                std::string titles[3] = {"X", "Y", "Z"};
                for (size_t ii = 0; ii < 3; ii++) {
                    oss << "set xlabel 'Time [s]'" << std::endl;
                    oss << "set ylabel '" << titles[ii] << "[m]'" << std::endl;
                    oss << "plot "
                        << "'" << fname << "' using 1:" << (tmp_start+ii) << " with lines title 'REFZMP',"
                        << "'" << fname << "' using 1:" << (tmp_start+3+ii) << " with lines title 'CARTZMP',"
                        << "'" << fname << "' using 1:" << (tmp_start+6+ii) << " with lines title 'COG'"
                        << std::endl;
                }
                plot_and_save(gps[0], gtitle, oss.str());
                tmp_start += 9;
            }
            {
                std::ostringstream oss("");
                std::string gtitle("Swing_support_pos");
                oss << "set multiplot layout 3, 1 title '" << gtitle << "'" << std::endl;
                std::string titles[3] = {"X", "Y", "Z"};
                for (size_t ii = 0; ii < 3; ii++) {
                    oss << "set xlabel 'Time [s]'" << std::endl;
                    oss << "set ylabel '" << titles[ii] << "[m]'" << std::endl;
                    oss << "plot "
                        << "'" << fname << "' using 1:" << (tmp_start+ii) << " with lines title 'rleg',"
                        << "'" << fname << "' using 1:" << (tmp_start+3+ii) << " with lines title 'lleg'"
                        << std::endl;
                }
                plot_and_save(gps[1], gtitle, oss.str());
                tmp_start += 6;
            }
            {
                std::ostringstream oss("");
                std::string gtitle("Swing_support_rot");
                oss << "set multiplot layout 3, 1 title '" << gtitle << "'" << std::endl;
                std::string titles[3] = {"Roll", "Pitch", "Yaw"};
                for (size_t ii = 0; ii < 3; ii++) {
                    oss << "set xlabel 'Time [s]'" << std::endl;
                    oss << "set ylabel '" << titles[ii] << "[deg]'" << std::endl;
                    oss << "plot "
                        << "'" << fname << "' using 1:" << (tmp_start+ii) << " with lines title 'rleg',"
                        << "'" << fname << "' using 1:" << (tmp_start+3+ii) << " with lines title 'lleg'"
                        << std::endl;
                }
                plot_and_save(gps[2], gtitle, oss.str());
                tmp_start += 6;
            }
            {
                std::ostringstream oss("");
                std::string gtitle("Swing_support_zmp_offset");
                oss << "set multiplot layout 3, 1 title '" << gtitle << "'" << std::endl;
                std::string titles[3] = {"X", "Y", "Z"};
                for (size_t ii = 0; ii < 3; ii++) {
                    oss << "set xlabel 'Time [s]'" << std::endl;
                    oss << "set ylabel '" << titles[ii] << "[m]'" << std::endl;
                    oss << "plot "
                        << "'" << fname << "' using 1:" << (tmp_start+ii) << " with lines title 'rleg',"
                        << "'" << fname << "' using 1:" << (tmp_start+3+ii) << " with lines title 'lleg'"
                        << std::endl;
                }
                plot_and_save(gps[3], gtitle, oss.str());
                tmp_start += 6;
            }
            {
                std::ostringstream oss("");
                std::string gtitle("Swing_support_vel");
                oss << "set multiplot layout 3, 1 title '" << gtitle << "'" << std::endl;
                std::string titles[3] = {"X", "Y", "Z"};
                for (size_t ii = 0; ii < 3; ii++) {
                    oss << "set xlabel 'Time [s]'" << std::endl;
                    oss << "set ylabel '" << titles[ii] << "[m]'" << std::endl;
                    oss << "plot "
                        << "'" << fname << "' using 1:" << (tmp_start+ii) << " with lines title 'rleg',"
                        << "'" << fname << "' using 1:" << (tmp_start+3+ii) << " with lines title 'lleg'"
                        << std::endl;
                }
                plot_and_save(gps[5], gtitle, oss.str());
                tmp_start += 6;
            }
            {
                std::ostringstream oss("");
                std::string gtitle("Swing_support_pos_trajectory");
                double min_v[3], max_v[3], range[3];
                for (size_t ii = 0; ii < 3; ii++) {
                    min_v[ii] = std::min(min_rfoot_pos(ii), min_lfoot_pos(ii));
                    max_v[ii] = std::max(max_rfoot_pos(ii), max_lfoot_pos(ii));
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
                    << "'" << fname << "' using " << (2+3+3+3+0) << ":" << (2+3+3+3+2)  << " with lines title 'rleg ee',"
                    << "'" << fname << "' using " << (2+3+3+3+3+0) << ":" << (2+3+3+3+3+2) << " with lines title 'lleg ee',"
                    << "'" << fname << "' using " << (tmp_start) << ":" << (tmp_start+2)  << " with lines title 'rleg toe',"
                    << "'" << fname << "' using " << (tmp_start+3) << ":" << (tmp_start+3+2)  << " with lines title 'lleg toe',"
                    << "'" << fname << "' using " << (tmp_start+3+3) << ":" << (tmp_start+3+3+2)  << " with lines title 'rleg heel',"
                    << "'" << fname << "' using " << (tmp_start+3+3+3) << ":" << (tmp_start+3+3+3+2)  << " with lines title 'lleg heel'"
                    << std::endl;
                //oss << "set title 'Y-Z'" << std::endl;
                oss << "set size ratio " << range[2]/range[1] << std::endl;
                oss << "set xlabel 'Y [m]'" << std::endl;            
                oss << "set ylabel 'Z [m]'" << std::endl;            
                oss << "plot "
                    << "[" << min_v[1]<< ":" << max_v[1] << "]"
                    << "[" << min_v[2] << ":" << max_v[2] << "]"
                    << "'" << fname << "' using " << (2+3+3+3+1) << ":" << (2+3+3+3+2)  << " with lines title 'rleg ee',"
                    << "'" << fname << "' using " << (2+3+3+3+3+1) << ":" << (2+3+3+3+3+2) << " with lines title 'lleg ee',"
                    << "'" << fname << "' using " << (tmp_start+1) << ":" << (tmp_start+2)  << " with lines title 'rleg toe',"
                    << "'" << fname << "' using " << (tmp_start+3+1) << ":" << (tmp_start+3+2)  << " with lines title 'lleg toe',"
                    << "'" << fname << "' using " << (tmp_start+3+3+1) << ":" << (tmp_start+3+3+2)  << " with lines title 'rleg heel',"
                    << "'" << fname << "' using " << (tmp_start+3+3+3+1) << ":" << (tmp_start+3+3+3+2)  << " with lines title 'lleg heel'"
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
                    << "'" << fname_sstime << "' using 1:" << 5 << " with lines title 'lleg contact states'"
                    << std::endl;
                plot_and_save(gps[4], gtitle, oss.str());
            }
            double tmp;
            std::cin >> tmp;
            for (size_t ii = 0; ii < gpsize; ii++) {
                fprintf(gps[ii], "exit\n");
                fflush(gps[ii]);
                pclose(gps[ii]);
            }
        }
        std::cerr << "Checking" << std::endl;
        std::cerr << "  ZMP error : " << is_small_zmp_error << std::endl;
        std::cerr << "  ZMP diff : " << is_small_zmp_diff << std::endl;
        std::cerr << "  Contact states & swing support time validity : " << is_contact_states_swing_support_time_validity << std::endl;
    };

    void gen_and_plot_walk_pattern(const step_node& initial_support_leg_step, const step_node& initial_swing_leg_dst_step)
    {
        parse_params();
        gg->print_param();
        gg->initialize_gait_parameter(cog, boost::assign::list_of(initial_support_leg_step), boost::assign::list_of(initial_swing_leg_dst_step));
        while ( !gg->proc_one_tick() );
        //gg->print_footstep_list();
        plot_walk_pattern();
    }

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
    testGaitGenerator() : use_gnuplot(true), is_small_zmp_error(true), is_small_zmp_diff(true), is_contact_states_swing_support_time_validity(true) {};
    virtual ~testGaitGenerator()
    {
        if (gg != NULL) {
            delete gg;
            gg = NULL;
        }
    };

    void test0 ()
    {
        std::cerr << "test0 : Set foot steps" << std::endl;
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
        std::cerr << "test1 : Go pos x,y,th combination" << std::endl;
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
        std::cerr << "test2 : Go pos x" << std::endl;
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
        std::cerr << "test3 : Go pos y" << std::endl;
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
        std::cerr << "test4 : Go pos th" << std::endl;
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
        std::cerr << "test5 : Set foot steps with Z change" << std::endl;
        /* initialize sample footstep_list */
        parse_params();
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
        std::cerr << "test6 : Go single step" << std::endl;
        parse_params();
        gg->clear_footstep_nodes_list();
        gg->go_single_step_param_2_footstep_nodes_list(100*1e-3, 0, 0, 0, "rleg", coordinates(leg_pos[0]));
        gen_and_plot_walk_pattern();
    };

    void test7 ()
    {
        std::cerr << "test7 : Toe heel walk" << std::endl;
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
        coordinates start_ref_coords;
        mid_coords(start_ref_coords, 0.5, coordinates(leg_pos[1]), coordinates(leg_pos[0]));
        gg->go_pos_param_2_footstep_nodes_list(100*1e-3, 0, 0, boost::assign::list_of(coordinates(leg_pos[1])), start_ref_coords, boost::assign::list_of(LLEG));
        gen_and_plot_walk_pattern();
    };

    void test8 ()
    {
        std::cerr << "test8 : Toe heel walk on slope" << std::endl;
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
        std::cerr << "test9 : Stair walk" << std::endl;
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
        std::cerr << "test10 : Stair walk + toe heel contact" << std::endl;
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
        gg->set_default_double_support_ratio(0.2);
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
        std::cerr << "test11 : Foot rot change" << std::endl;
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
        std::cerr << "test12 : Change step param in set foot steps" << std::endl;
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
        std::cerr << "test13 : Arbitrary leg switching" << std::endl;
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
        std::cerr << "test14 : kick walk" << std::endl;
        /* initialize sample footstep_list */
        parse_params();
        gg->clear_footstep_nodes_list();
        gg->set_default_orbit_type(CYCLOIDDELAYKICK);
        coordinates start_ref_coords;
        mid_coords(start_ref_coords, 0.5, coordinates(leg_pos[1]), coordinates(leg_pos[0]));
        gg->go_pos_param_2_footstep_nodes_list(300*1e-3, 0, 0, boost::assign::list_of(coordinates(leg_pos[1])), start_ref_coords, boost::assign::list_of(LLEG));
        gen_and_plot_walk_pattern();
    };


    void parse_params ()
    {
      for (int i = 0; i < arg_strs.size(); ++ i) {
          if ( arg_strs[i]== "--default-step-time" ) {
              if (++i < arg_strs.size()) gg->set_default_step_time(atof(arg_strs[i].c_str()));
          } else if ( arg_strs[i]== "--default-step-height" ) {
              if (++i < arg_strs.size()) gg->set_default_step_height(atof(arg_strs[i].c_str()));
          } else if ( arg_strs[i]== "--default-double-support-ratio" ) {
              if (++i < arg_strs.size()) gg->set_default_double_support_ratio(atof(arg_strs[i].c_str()));
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
          } else if ( arg_strs[i]== "--default-double-support-static-ratio" ) {
              if (++i < arg_strs.size()) gg->set_default_double_support_static_ratio(atof(arg_strs[i].c_str()));
          } else if ( arg_strs[i]== "--swing-trajectory-delay-time-offset" ) {
              if (++i < arg_strs.size()) gg->set_swing_trajectory_delay_time_offset(atof(arg_strs[i].c_str()));
          } else if ( arg_strs[i]== "--swing-trajectory-final-distance-weight" ) {
              if (++i < arg_strs.size()) gg->set_swing_trajectory_final_distance_weight(atof(arg_strs[i].c_str()));
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
          }
      }   
    };

    bool check_all_results ()
    {
        return is_small_zmp_error && is_small_zmp_diff && is_contact_states_swing_support_time_validity;
    };
};

class testGaitGeneratorHRP2JSK : public testGaitGenerator
{
 public:
    testGaitGeneratorHRP2JSK ()
        {
            dt = 0.004;
            cog = 1e-3*hrp::Vector3(6.785, 1.54359, 806.831);
            leg_pos.push_back(hrp::Vector3(0,1e-3*-105,0)); /* rleg */
            leg_pos.push_back(hrp::Vector3(0,1e-3* 105,0)); /* lleg */
            all_limbs.push_back("rleg");
            all_limbs.push_back("lleg");
            gg = new gait_generator(dt, leg_pos, all_limbs, 1e-3*150, 1e-3*50, 10, 1e-3*50);
        };
};

void print_usage ()
{
    std::cerr << "Usage : testGaitGenerator [option]" << std::endl;
    std::cerr << " [option] should be:" << std::endl;
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
    std::cerr << "  --test10 : Stair walk + toe heel contact" << std::endl;
    std::cerr << "  --test11 : Foot rot change" << std::endl;
    std::cerr << "  --test12 : Change step param in set foot steps" << std::endl;
    std::cerr << "  --test13 : Arbitrary leg switching" << std::endl;
    std::cerr << "  --test14 : kick walk" << std::endl;
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

