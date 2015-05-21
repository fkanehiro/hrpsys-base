/* -*- coding:utf-8-unix; mode:c++; -*- */

#include "GaitGenerator.h"
/* samples */
using namespace rats;
#include <cstdio>
#include <coil/stringutil.h>

class testGaitGenerator
{
protected:
    double dt; /* [s] */
    std::vector<hrp::Vector3> leg_pos; /* default footstep transformations are necessary */
    hrp::Vector3 cog;
    gait_generator* gg;
private:
    void plot_and_save (FILE* gp, const std::string graph_fname, const std::string plot_str)
    {
        gp = popen("gnuplot", "w");
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
        hrp::Vector3 prev_rfoot_pos, prev_lfoot_pos;
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
            hrp::Vector3 rfoot_pos = (gg->get_support_leg() == "rleg") ? gg->get_support_leg_coords().pos : gg->get_swing_leg_coords().pos;
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp, "%f ", rfoot_pos(ii));
            }
            hrp::Vector3 lfoot_pos = (gg->get_support_leg() == "lleg") ? gg->get_support_leg_coords().pos : gg->get_swing_leg_coords().pos;
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp, "%f ", lfoot_pos(ii));
            }
            // Foot rot
            hrp::Vector3 rpy;
            rpy = hrp::rpyFromRot((gg->get_support_leg() == "rleg") ? gg->get_support_leg_coords().rot : gg->get_swing_leg_coords().rot);
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp, "%f ", 180.0*rpy(ii)/M_PI);
            }
            rpy = hrp::rpyFromRot((gg->get_support_leg() == "lleg") ? gg->get_support_leg_coords().rot : gg->get_swing_leg_coords().rot);
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp, "%f ", 180.0*rpy(ii)/M_PI);
            }
            // ZMP offsets
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp, "%f ", (gg->get_support_leg() == "rleg") ? gg->get_support_foot_zmp_offset()(ii) : gg->get_swing_foot_zmp_offset()(ii));
            }
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp, "%f ", (gg->get_support_leg() == "lleg") ? gg->get_support_foot_zmp_offset()(ii) : gg->get_swing_foot_zmp_offset()(ii));
            }
            // Swing time
            fprintf(fp, "%f %f ",
                    gg->get_current_swing_time(gait_generator::RLEG),
                    gg->get_current_swing_time(gait_generator::LLEG));
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
            fprintf(fp, "\n");
            i++;
        }
        fclose(fp);

        /* plot */
        size_t gpsize = 7;
        size_t tmp_start = 2;
        FILE* gps[gpsize];
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
            std::string gtitle("Swing_support_remain_time");
            oss << "set multiplot layout 1, 1 title '" << gtitle << "'" << std::endl;
            oss << "set title 'Remain Time'" << std::endl;
            oss << "set xlabel 'Time [s]'" << std::endl;
            oss << "set ylabel 'Time [s]'" << std::endl;
            oss << "plot "
                << "'" << fname << "' using 1:" << (tmp_start+0) << " with lines title 'rleg',"
                << "'" << fname << "' using 1:" << (tmp_start+1) << " with lines title 'lleg'"
                << std::endl;
            plot_and_save(gps[4], gtitle, oss.str());
            tmp_start += 2;
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
        }
        {
            std::ostringstream oss("");
            std::string gtitle("Swing_support_pos_trajectory");
            oss << "set multiplot layout 2, 1 title '" << gtitle << "'" << std::endl;
            oss << "set title 'X-Z'" << std::endl;
            oss << "set xlabel 'X [m]'" << std::endl;            
            oss << "set ylabel 'Z [m]'" << std::endl;            
            oss << "plot "
                << "'" << fname << "' using " << (2+3+3+0) << ":" << (2+3+3+2)  << " with lines title 'rleg',"
                << "'" << fname << "' using " << (2+3+3+3+0) << ":" << (2+3+3+3+2) << " with lines title 'lleg'"
                << std::endl;
            oss << "set title 'Y-Z'" << std::endl;
            oss << "set xlabel 'Y [m]'" << std::endl;            
            oss << "set ylabel 'Z [m]'" << std::endl;            
            oss << "plot "
                << "'" << fname << "' using " << (2+3+3+1) << ":" << (2+3+3+2)  << " with lines title 'rleg',"
                << "'" << fname << "' using " << (2+3+3+3+1) << ":" << (2+3+3+3+2) << " with lines title 'lleg'"
                << std::endl;
            plot_and_save(gps[6], gtitle, oss.str());
        }
        double tmp;
        std::cin >> tmp;
        for (size_t ii = 0; ii < gpsize; ii++) {
            pclose(gps[ii]);
        }
    };

    void gen_and_plot_walk_pattern(const coordinates& initial_support_leg_coords, const coordinates& initial_swing_leg_dst_coords)
    {
        parse_params();
        gg->print_param();
        gg->initialize_gait_parameter(cog, initial_support_leg_coords, initial_swing_leg_dst_coords);
        while ( !gg->proc_one_tick() );
        //gg->print_footstep_list();
        plot_walk_pattern();
    }

    void gen_and_plot_walk_pattern()
    {
        coordinates initial_support_leg_coords(gg->get_footstep_front_leg()=="rleg"?leg_pos[1]:leg_pos[0]);
        coordinates initial_swing_leg_dst_coords(gg->get_footstep_front_leg()!="rleg"?leg_pos[1]:leg_pos[0]);
        gen_and_plot_walk_pattern(initial_support_leg_coords, initial_swing_leg_dst_coords);
    }

public:
    std::vector<std::string> arg_strs;
    testGaitGenerator() {};
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
        gg->clear_footstep_node_list();
        gg->append_footstep_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(0, 0, 0)+leg_pos[0])));
        gg->append_footstep_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(0, 0, 0)+leg_pos[1])));
        gg->append_footstep_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(100*1e-3, 0, 0)+leg_pos[0])));
        gg->append_footstep_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(200*1e-3, 0, 0)+leg_pos[1])));
        gg->append_footstep_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(200*1e-3, 0, 0)+leg_pos[0])));
        gg->append_finalize_footstep();
        gen_and_plot_walk_pattern();
    };

    void test1 ()
    {
        std::cerr << "test1 : Go pos x,y,th combination" << std::endl;
        /* initialize sample footstep_list */
        gg->clear_footstep_node_list();
        gg->go_pos_param_2_footstep_list(200*1e-3, 100*1e-3, 20, coordinates());
        gen_and_plot_walk_pattern();
    };

    void test2 ()
    {
        std::cerr << "test2 : Go pos x" << std::endl;
        /* initialize sample footstep_list */
        gg->clear_footstep_node_list();
        gg->go_pos_param_2_footstep_list(300*1e-3, 0, 0, coordinates());
        gen_and_plot_walk_pattern();
    };

    void test3 ()
    {
        std::cerr << "test3 : Go pos y" << std::endl;
        /* initialize sample footstep_list */
        gg->clear_footstep_node_list();
        gg->go_pos_param_2_footstep_list(0, 150*1e-3, 0, coordinates());
        gen_and_plot_walk_pattern();
    };

    void test4 ()
    {
        std::cerr << "test4 : Go pos th" << std::endl;
        /* initialize sample footstep_list */
        gg->clear_footstep_node_list();
        gg->go_pos_param_2_footstep_list(0, 0, 30, coordinates());
        gen_and_plot_walk_pattern();
    };

    void test5 ()
    {
        std::cerr << "test5 : Set foot steps with Z change" << std::endl;
        /* initialize sample footstep_list */
        gg->clear_footstep_node_list();
        gg->append_footstep_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(0, 0, 0)+leg_pos[0])));
        gg->append_footstep_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(0, 0, 0)+leg_pos[1])));
        gg->append_footstep_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(100*1e-3, 0, 100*1e-3)+leg_pos[0])));
        gg->append_footstep_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(200*1e-3, 0, 200*1e-3)+leg_pos[1])));
        gg->append_footstep_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(300*1e-3, 0, 300*1e-3)+leg_pos[0])));
        gg->append_footstep_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(300*1e-3, 0, 300*1e-3)+leg_pos[1])));
        gg->append_finalize_footstep();
        gen_and_plot_walk_pattern();
    };

    void test6 ()
    {
        std::cerr << "test6 : Go single step" << std::endl;
        gg->clear_footstep_node_list();
        gg->go_single_step_param_2_footstep_list(100*1e-3, 0, 0, 0, "rleg", coordinates(leg_pos[0]));
        gen_and_plot_walk_pattern();
    };

    void test7 ()
    {
        std::cerr << "test7 : Toe heel walk" << std::endl;
        /* initialize sample footstep_list */
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
        gg->clear_footstep_node_list();
        gg->go_pos_param_2_footstep_list(100*1e-3, 0, 0, coordinates());
        gen_and_plot_walk_pattern();
    };

    void test8 ()
    {
        std::cerr << "test8 : Toe heel walk on slope" << std::endl;
        /* initialize sample footstep_list */
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
        gg->clear_footstep_node_list();
        hrp::Matrix33 initial_foot_mid_rot = Eigen::AngleAxis<double>(M_PI/2, hrp::Vector3::UnitZ()).toRotationMatrix();
        //hrp::Matrix33 initial_foot_mid_rot = Eigen::AngleAxis<double>(M_PI, hrp::Vector3::UnitZ()).toRotationMatrix();
        gg->go_pos_param_2_footstep_list(100*1e-3, 0, 0, coordinates(initial_foot_mid_rot));
        coordinates initial_support_leg_coords(hrp::Vector3(initial_foot_mid_rot * (gg->get_footstep_front_leg()=="rleg"?leg_pos[1]:leg_pos[0])), initial_foot_mid_rot);
        coordinates initial_swing_leg_dst_coords(hrp::Vector3(initial_foot_mid_rot * (gg->get_footstep_front_leg()!="rleg"?leg_pos[1]:leg_pos[0])), initial_foot_mid_rot);
        gen_and_plot_walk_pattern(initial_support_leg_coords, initial_swing_leg_dst_coords);

    };

    void test9 ()
    {
        std::cerr << "test9 : Stair walk" << std::endl;
        /* initialize sample footstep_list */
        gg->clear_footstep_node_list();
        gg->set_default_orbit_type(gait_generator::STAIR);
        gg->set_swing_trajectory_delay_time_offset (0.15);
        gg->append_footstep_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(0, 0, 0)+leg_pos[0])));
        gg->append_footstep_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(0, 0, 0)+leg_pos[1])));
        gg->append_footstep_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(250*1e-3, 0, 200*1e-3)+leg_pos[0])));
        gg->append_footstep_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(250*1e-3, 0, 200*1e-3)+leg_pos[1])));
        gg->append_footstep_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(500*1e-3, 0, 400*1e-3)+leg_pos[0])));
        gg->append_footstep_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(500*1e-3, 0, 400*1e-3)+leg_pos[1])));
        gg->append_footstep_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(750*1e-3, 0, 600*1e-3)+leg_pos[0])));
        gg->append_footstep_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(750*1e-3, 0, 600*1e-3)+leg_pos[1])));
        gg->append_finalize_footstep();
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
          } else if ( arg_strs[i]== "--default-double-support-static-ratio" ) {
              if (++i < arg_strs.size()) gg->set_default_double_support_static_ratio(atof(arg_strs[i].c_str()));
          } else if ( arg_strs[i]== "--swing-trajectory-delay-time-offset" ) {
              if (++i < arg_strs.size()) gg->set_swing_trajectory_delay_time_offset(atof(arg_strs[i].c_str()));
          } else if ( arg_strs[i]== "--stair-trajectory-way-point-offset" ) {
              if (++i < arg_strs.size()) {
                  coil::vstring strs = coil::split(std::string(arg_strs[i].c_str()), ",");
                  gg->set_stair_trajectory_way_point_offset(hrp::Vector3(atof(strs[0].c_str()), atof(strs[1].c_str()), atof(strs[2].c_str())));
              }
          }
      }   
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
            gg = new gait_generator(dt, leg_pos, 1e-3*150, 1e-3*50, 10, 1e-3*50);
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
};

int main(int argc, char* argv[])
{
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
      } else {
          print_usage();
      }
  } else {
      print_usage();
  }
  return 0;
}

