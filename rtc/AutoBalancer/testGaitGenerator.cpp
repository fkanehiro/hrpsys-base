/* -*- coding:utf-8-unix; mode:c++; -*- */

#include "GaitGenerator.h"
/* samples */
using namespace rats;
#include <cstdio>

class testGaitGenerator
{
protected:
    double dt; /* [s] */
    std::vector<hrp::Vector3> leg_pos; /* default footstep transformations are necessary */
    hrp::Vector3 cog;
private:
    void plot_walk_pattern (gait_generator& gg, const double dt)
    {
        /* make step and dump */
        size_t i = 0;
        std::string fname("/tmp/plot.dat");
        FILE* fp = fopen(fname.c_str(), "w");
        while ( gg.proc_one_tick() ) {
            //std::cerr << gg.lcg.gp_count << std::endl;
            // if ( gg.lcg.gp_index == 4 && gg.lcg.gp_count == 100) {
            //   //std::cerr << gg.lcg.gp_index << std::endl;
            //   gg.update_refzmp_queue(coordinates(hrp::Vector3(150, 105, 0)), coordinates(hrp::Vector3(150, -105, 0)));
            // }
            fprintf(fp, "%f ", i * dt);
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp, "%f ", gg.get_refzmp()(ii));
            }
            for (size_t ii = 0; ii < 3; ii++) {
                double cogpos;
                if (ii==2) {
                    coordinates tmpc;
                    gg.get_swing_support_mid_coords(tmpc);
                    cogpos = tmpc.pos(2)+gg.get_cog()(2);
                } else {
                    cogpos = gg.get_cog()(ii);
                }
                fprintf(fp, "%f ", cogpos);
            }
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp, "%f ", (gg.get_support_leg() == "rleg") ? gg.get_support_leg_coords().pos(ii) : gg.get_swing_leg_coords().pos(ii));
            }
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp, "%f ", (gg.get_support_leg() == "lleg") ? gg.get_support_leg_coords().pos(ii) : gg.get_swing_leg_coords().pos(ii));
            }
            hrp::Vector3 rpy;
            rpy = hrp::rpyFromRot((gg.get_support_leg() == "rleg") ? gg.get_support_leg_coords().rot : gg.get_swing_leg_coords().rot);
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp, "%f ", 180.0*rpy(ii)/M_PI);
            }
            rpy = hrp::rpyFromRot((gg.get_support_leg() == "lleg") ? gg.get_support_leg_coords().rot : gg.get_swing_leg_coords().rot);
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp, "%f ", 180.0*rpy(ii)/M_PI);
            }
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp, "%f ", (gg.get_support_leg() == "rleg") ? gg.get_support_foot_zmp_offset()(ii) : gg.get_swing_foot_zmp_offset()(ii));
            }
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(fp, "%f ", (gg.get_support_leg() == "lleg") ? gg.get_support_foot_zmp_offset()(ii) : gg.get_swing_foot_zmp_offset()(ii));
            }
            fprintf(fp, "%f %f ",
                    gg.get_current_swing_time(gait_generator::RLEG),
                    gg.get_current_swing_time(gait_generator::LLEG));
            fprintf(fp, "\n");
            i++;
        }
        fclose(fp);

        /* plot */
        size_t gpsize = 6;
        size_t tmp_start = 2;
        FILE* gps[gpsize];
        {
            FILE* gp = gps[0];
            gp = popen("gnuplot", "w");
            fprintf(gp, "set multiplot layout 3, 1 title 'COG and ZMP'\n");
            std::string titles[3] = {"X", "Y", "Z"};
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(gp, "set xlabel 'Time [s]'\n");
                fprintf(gp, "set ylabel '%s [m]'\n", titles[ii].c_str());
                fprintf(gp, "plot '%s' using 1:%d with lines title 'refzmp', '%s' using 1:%d with lines title 'cog'\n",
                        fname.c_str(), (tmp_start+ii), fname.c_str(), (tmp_start+3+ii));
            }
            fflush(gp);
            tmp_start += 6;
        }
        {
            FILE* gp = gps[1];
            gp = popen("gnuplot", "w");
            fprintf(gp, "set multiplot layout 3, 1 title 'Swing support pos'\n");
            std::string titles[3] = {"X", "Y", "Z"};
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(gp, "set xlabel 'Time [s]'\n");
                fprintf(gp, "set ylabel '%s [m]'\n", titles[ii].c_str());
                fprintf(gp, "plot '%s' using 1:%d with lines title 'rleg', '%s' using 1:%d with lines title 'lleg'\n",
                        fname.c_str(), (tmp_start+ii), fname.c_str(), (tmp_start+3+ii));
            }
            fflush(gp);
            tmp_start += 6;
        }
        {
            FILE* gp = gps[2];
            gp = popen("gnuplot", "w");
            fprintf(gp, "set multiplot layout 3, 1 title 'Swing support rot'\n");
            std::string titles[3] = {"Roll", "Pitch", "Yaw"};
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(gp, "set xlabel 'Time [s]'\n");
                fprintf(gp, "set ylabel '%s [deg]'\n", titles[ii].c_str());
                fprintf(gp, "plot '%s' using 1:%d with lines title 'rleg', '%s' using 1:%d with lines title 'lleg'\n",
                        fname.c_str(), (tmp_start+ii), fname.c_str(), (tmp_start+3+ii));
            }
            fflush(gp);
            tmp_start += 6;
        }
        {
            FILE* gp = gps[3];
            gp = popen("gnuplot", "w");
            fprintf(gp, "set multiplot layout 3, 1 title 'Swing support zmp offset'\n");
            std::string titles[3] = {"X", "Y", "Z"};
            for (size_t ii = 0; ii < 3; ii++) {
                fprintf(gp, "set xlabel 'Time [s]'\n");
                fprintf(gp, "set ylabel '%s [m]'\n", titles[ii].c_str());
                fprintf(gp, "plot '%s' using 1:%d with lines title 'rleg zmpoff', '%s' using 1:%d with lines title 'lleg zmpoff'\n",
                        fname.c_str(), (tmp_start+ii), fname.c_str(), (tmp_start+3+ii));
            }
            fflush(gp);
            tmp_start += 6;
        }
        {
            FILE* gp = gps[4];
            gp = popen("gnuplot", "w");
            fprintf(gp, "set multiplot layout 1,1 title 'Swing support remain time'\n");
            fprintf(gp, "set title 'Remain Time'\n");
            fprintf(gp, "set xlabel 'Time [s]'\n");
            fprintf(gp, "set ylabel 'Tims [s]'\n");
            fprintf(gp, "plot '%s' using 1:%d with lines title 'rleg', '%s' using 1:%d with lines title 'lleg'\n",
                    fname.c_str(), (tmp_start+0), fname.c_str(), (tmp_start+1));
            fflush(gp);
        }
        {
            FILE* gp = gps[5];
            gp = popen("gnuplot", "w");
            fprintf(gp, "set multiplot layout 2,1 title 'Swing support pos trajectory'\n");
            fprintf(gp, "set title 'X-Z'\n");
            fprintf(gp, "set xlabel 'X [m]'\n");
            fprintf(gp, "set ylabel 'Z [m]'\n");
            fprintf(gp, "plot '%s' using %d:%d with lines title 'rleg', '%s' using %d:%d with lines title 'lleg'\n",
                    fname.c_str(), (2+3+3+0), (2+3+3+2), fname.c_str(), (2+3+3+3+0), (2+3+3+3+2));
            fprintf(gp, "set title 'Y-Z'\n");
            fprintf(gp, "set xlabel 'Y [m]'\n");
            fprintf(gp, "set ylabel 'Z [m]'\n");
            fprintf(gp, "plot '%s' using %d:%d with lines title 'rleg', '%s' using %d:%d with lines title 'lleg'\n",
                    fname.c_str(), (2+3+3+1), (2+3+3+2), fname.c_str(), (2+3+3+3+1), (2+3+3+3+2));
            fflush(gp);
        }
        double tmp;
        std::cin >> tmp;
        for (size_t ii = 0; ii < gpsize; ii++) {
            pclose(gps[ii]);
        }
    };

    void gen_and_plot_walk_pattern(gait_generator& gg, const coordinates& initial_support_leg_coords, const coordinates& initial_swing_leg_dst_coords)
    {
        gg.initialize_gait_parameter(cog, initial_support_leg_coords, initial_swing_leg_dst_coords);
        while ( !gg.proc_one_tick() );
        //gg.print_footstep_list();
        plot_walk_pattern(gg, dt);
    }

    void gen_and_plot_walk_pattern(gait_generator& gg)
    {
        coordinates initial_support_leg_coords(gg.get_footstep_front_leg()=="rleg"?leg_pos[1]:leg_pos[0]);
        coordinates initial_swing_leg_dst_coords(gg.get_footstep_front_leg()!="rleg"?leg_pos[1]:leg_pos[0]);
        gen_and_plot_walk_pattern(gg, initial_support_leg_coords, initial_swing_leg_dst_coords);
    }

public:
    testGaitGenerator() {};
    virtual ~testGaitGenerator() {};

    void test0 ()
    {
        std::cerr << "test0 : Set foot steps" << std::endl;
        gait_generator gg(dt, leg_pos, 1e-3*150, 1e-3*50, 10, 1e-3*50);
        /* initialize sample footstep_list */
        gg.clear_footstep_node_list();
        gg.append_footstep_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(0, 0, 0)+leg_pos[0])));
        gg.append_footstep_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(0, 0, 0)+leg_pos[1])));
        gg.append_footstep_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(100*1e-3, 0, 0)+leg_pos[0])));
        gg.append_footstep_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(200*1e-3, 0, 0)+leg_pos[1])));
        gg.append_footstep_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(200*1e-3, 0, 0)+leg_pos[0])));
        gen_and_plot_walk_pattern(gg);
    };

    void test1 ()
    {
        std::cerr << "test1 : Go pos x,y,th combination" << std::endl;
        gait_generator gg(dt, leg_pos, 1e-3*150, 1e-3*50, 10, 1e-3*50);
        /* initialize sample footstep_list */
        gg.clear_footstep_node_list();
        gg.go_pos_param_2_footstep_list(200*1e-3, 100*1e-3, 20, coordinates());
        gen_and_plot_walk_pattern(gg);
    };

    void test2 ()
    {
        std::cerr << "test2 : Go pos x" << std::endl;
        gait_generator gg(dt, leg_pos, 1e-3*150, 1e-3*50, 10, 1e-3*50);
        /* initialize sample footstep_list */
        gg.clear_footstep_node_list();
        gg.go_pos_param_2_footstep_list(300*1e-3, 0, 0, coordinates());
        gen_and_plot_walk_pattern(gg);
    };

    void test3 ()
    {
        std::cerr << "test3 : Go pos y" << std::endl;
        gait_generator gg(dt, leg_pos, 1e-3*150, 1e-3*50, 10, 1e-3*50);
        /* initialize sample footstep_list */
        gg.clear_footstep_node_list();
        gg.go_pos_param_2_footstep_list(0, 150*1e-3, 0, coordinates());
        gen_and_plot_walk_pattern(gg);
    };

    void test4 ()
    {
        std::cerr << "test4 : Go pos th" << std::endl;
        gait_generator gg(dt, leg_pos, 1e-3*150, 1e-3*50, 10, 1e-3*50);
        /* initialize sample footstep_list */
        gg.clear_footstep_node_list();
        gg.go_pos_param_2_footstep_list(0, 0, 30, coordinates());
        gen_and_plot_walk_pattern(gg);
    };

    void test5 ()
    {
        std::cerr << "test5 : Set foot steps with Z change" << std::endl;
        gait_generator gg(dt, leg_pos, 1e-3*150, 1e-3*50, 10, 1e-3*50);
        /* initialize sample footstep_list */
        gg.clear_footstep_node_list();
        gg.append_footstep_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(0, 0, 0)+leg_pos[0])));
        gg.append_footstep_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(0, 0, 0)+leg_pos[1])));
        gg.append_footstep_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(100*1e-3, 0, 100*1e-3)+leg_pos[0])));
        gg.append_footstep_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(200*1e-3, 0, 200*1e-3)+leg_pos[1])));
        gg.append_footstep_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(300*1e-3, 0, 300*1e-3)+leg_pos[0])));
        gg.append_footstep_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(300*1e-3, 0, 300*1e-3)+leg_pos[1])));
        gen_and_plot_walk_pattern(gg);
    };

    void test6 ()
    {
        std::cerr << "test6 : Go single step" << std::endl;
        gait_generator gg(dt, leg_pos, 1e-3*150, 1e-3*50, 10, 1e-3*50);
        gg.clear_footstep_node_list();
        gg.go_single_step_param_2_footstep_list(100*1e-3, 0, 0, 0, "rleg", coordinates(leg_pos[0]));
        gen_and_plot_walk_pattern(gg);
    };

    void test7 ()
    {
        std::cerr << "test7 : Toe heel walk" << std::endl;
        gait_generator gg(dt, leg_pos, 1e-3*150, 1e-3*50, 10, 1e-3*50);
        /* initialize sample footstep_list */
        std::vector<hrp::Vector3> dzo;
        dzo.push_back(hrp::Vector3(20*1e-3,-30*1e-3,0));
        dzo.push_back(hrp::Vector3(20*1e-3,30*1e-3,0));
        gg.set_default_zmp_offsets(dzo);
        gg.set_toe_zmp_offset_x(137*1e-3);
        gg.set_heel_zmp_offset_x(-105*1e-3);
        gg.set_toe_pos_offset_x(137*1e-3);
        gg.set_heel_pos_offset_x(-105*1e-3);
        gg.set_toe_angle(30);
        gg.set_heel_angle(10);
        // gg.set_use_toe_heel_transition(false);
        gg.set_use_toe_heel_transition(true);
        gg.clear_footstep_node_list();
        gg.go_pos_param_2_footstep_list(100*1e-3, 0, 0, coordinates());
        gen_and_plot_walk_pattern(gg);
    };

    void test8 ()
    {
        std::cerr << "test8 : Toe heel walk on slope" << std::endl;
        gait_generator gg(dt, leg_pos, 1e-3*150, 1e-3*50, 10, 1e-3*50);
        /* initialize sample footstep_list */
        std::vector<hrp::Vector3> dzo;
        dzo.push_back(hrp::Vector3(20*1e-3,-30*1e-3,0));
        dzo.push_back(hrp::Vector3(20*1e-3,30*1e-3,0));
        gg.set_default_zmp_offsets(dzo);
        gg.set_toe_zmp_offset_x(137*1e-3);
        gg.set_heel_zmp_offset_x(-105*1e-3);
        gg.set_toe_pos_offset_x(137*1e-3);
        gg.set_heel_pos_offset_x(-105*1e-3);
        gg.set_toe_angle(30);
        gg.set_heel_angle(10);
        // gg.set_use_toe_heel_transition(false);
        gg.set_use_toe_heel_transition(true);
        gg.clear_footstep_node_list();
        hrp::Matrix33 initial_foot_mid_rot = Eigen::AngleAxis<double>(M_PI/2, hrp::Vector3::UnitZ()).toRotationMatrix();
        //hrp::Matrix33 initial_foot_mid_rot = Eigen::AngleAxis<double>(M_PI, hrp::Vector3::UnitZ()).toRotationMatrix();
        gg.go_pos_param_2_footstep_list(100*1e-3, 0, 0, coordinates(initial_foot_mid_rot));
        coordinates initial_support_leg_coords(hrp::Vector3(initial_foot_mid_rot * (gg.get_footstep_front_leg()=="rleg"?leg_pos[1]:leg_pos[0])), initial_foot_mid_rot);
        coordinates initial_swing_leg_dst_coords(hrp::Vector3(initial_foot_mid_rot * (gg.get_footstep_front_leg()!="rleg"?leg_pos[1]:leg_pos[0])), initial_foot_mid_rot);
        gen_and_plot_walk_pattern(gg, initial_support_leg_coords, initial_swing_leg_dst_coords);

    };

    void test9 ()
    {
        std::cerr << "test9 : Stair walk" << std::endl;
        gait_generator gg(dt, leg_pos, 1e-3*150, 1e-3*50, 10, 1e-3*50);
        /* initialize sample footstep_list */
        gg.clear_footstep_node_list();
        gg.set_default_orbit_type(gait_generator::STAIR);
        gg.set_swing_trajectory_delay_time_offset (0.15);
        gg.append_footstep_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(0, 0, 0)+leg_pos[0])));
        gg.append_footstep_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(0, 0, 0)+leg_pos[1])));
        gg.append_footstep_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(100*1e-3, 0, 100*1e-3)+leg_pos[0])));
        gg.append_footstep_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(200*1e-3, 0, 200*1e-3)+leg_pos[1])));
        gg.append_footstep_node("rleg", coordinates(hrp::Vector3(hrp::Vector3(300*1e-3, 0, 100*1e-3)+leg_pos[0])));
        gg.append_footstep_node("lleg", coordinates(hrp::Vector3(hrp::Vector3(300*1e-3, 0, 100*1e-3)+leg_pos[1])));
        gen_and_plot_walk_pattern(gg);
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
  if (argc == 2) {
      if (std::string(argv[1]) == "--test0") {
          testGaitGeneratorHRP2JSK().test0();
      } else if (std::string(argv[1]) == "--test1") {
          testGaitGeneratorHRP2JSK().test1();
      } else if (std::string(argv[1]) == "--test2") {
          testGaitGeneratorHRP2JSK().test2();
      } else if (std::string(argv[1]) == "--test3") {
          testGaitGeneratorHRP2JSK().test3();
      } else if (std::string(argv[1]) == "--test4") {
          testGaitGeneratorHRP2JSK().test4();
      } else if (std::string(argv[1]) == "--test5") {
          testGaitGeneratorHRP2JSK().test5();
      } else if (std::string(argv[1]) == "--test6") {
          testGaitGeneratorHRP2JSK().test6();
      } else if (std::string(argv[1]) == "--test7") {
          testGaitGeneratorHRP2JSK().test7();
      } else if (std::string(argv[1]) == "--test8") {
          testGaitGeneratorHRP2JSK().test8();
      } else if (std::string(argv[1]) == "--test9") {
          testGaitGeneratorHRP2JSK().test9();
      } else {
          print_usage();
      }
  } else {
      print_usage();
  }
  return 0;
}

