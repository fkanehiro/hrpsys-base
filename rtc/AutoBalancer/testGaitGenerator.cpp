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
                double cogpos;
                if (ii==2) {
                    coordinates tmpc;
                    gg.get_swing_support_mid_coords(tmpc);
                    cogpos = tmpc.pos(2)+gg.get_cog()(2);
                } else {
                    cogpos = gg.get_cog()(ii);
                }
                fprintf(fp, "%f %f %f %f %f %f ",
                        gg.get_refzmp()(ii),
                        cogpos,
                        gg.get_support_leg_coords().pos(ii),
                        gg.get_swing_leg_coords().pos(ii),
                        gg.get_support_foot_zmp_offset()(ii),
                        gg.get_swing_foot_zmp_offset()(ii)
                        );
            }
            fprintf(fp, "\n");
            i++;
        }
        fclose(fp);

        /* plot */
        FILE* gp = popen("gnuplot", "w");
        fprintf(gp, "set multiplot layout 3, 1\n");
        std::string titles[3] = {"X", "Y", "Z"};
        int data_size = 6;
        for (size_t ii = 0; ii < 3; ii++) {
            fprintf(gp, "set title \"%s\"\n", titles[ii].c_str());
            fprintf(gp, "set xlabel \"Time [s]\"\n");
            fprintf(gp, "set ylabel \"[m]\"\n");
            fprintf(gp, "plot \"%s\" using 1:%d with lines title \"refzmp\", \"%s\" using 1:%d with lines title \"cog\"",
                    fname.c_str(), ( ii * data_size + 2), fname.c_str(), ( ii * data_size + 3));
            // fprintf(gp, ",\"%s\" using 1:%d with lines title \"support\", \"%s\" using 1:%d with lines title \"swing\"",
            //         fname.c_str(), ( ii * data_size + 4), fname.c_str(), ( ii * data_size + 5));
            // fprintf(gp, ",\"%s\" using 1:%d with lines title \"support zmpoff\", \"%s\" using 1:%d with lines title \"swing zmpoff\" ",
            //         fname.c_str(), ( ii * data_size + 6), fname.c_str(), ( ii * data_size + 7));
            fprintf(gp, "\n");
        }
        fflush(gp);
        double tmp;
        std::cin >> tmp;
        pclose(gp);
    };

    void gen_and_plot_walk_pattern(gait_generator& gg)
    {
        coordinates initial_support_leg_coords(gg.get_footstep_front_leg()=="rleg"?leg_pos[1]:leg_pos[0]);
        coordinates initial_swing_leg_dst_coords(gg.get_footstep_front_leg()!="rleg"?leg_pos[1]:leg_pos[0]);
        gg.initialize_gait_parameter(cog, initial_support_leg_coords, initial_swing_leg_dst_coords);
        while ( !gg.proc_one_tick() );
        gg.print_footstep_list();
        plot_walk_pattern(gg, dt);
    }

public:
    testGaitGenerator() {};
    virtual ~testGaitGenerator() {};

    void test0 ()
    {
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
        gait_generator gg(dt, leg_pos, 1e-3*150, 1e-3*50, 10, 1e-3*50);
        /* initialize sample footstep_list */
        gg.clear_footstep_node_list();
        gg.go_pos_param_2_footstep_list(200*1e-3, 100*1e-3, 20, coordinates());
        gen_and_plot_walk_pattern(gg);
    };

    void test2 ()
    {
        gait_generator gg(dt, leg_pos, 1e-3*150, 1e-3*50, 10, 1e-3*50);
        /* initialize sample footstep_list */
        gg.clear_footstep_node_list();
        gg.go_pos_param_2_footstep_list(300*1e-3, 0, 0, coordinates());
        gen_and_plot_walk_pattern(gg);
    };

    void test3 ()
    {
        gait_generator gg(dt, leg_pos, 1e-3*150, 1e-3*50, 10, 1e-3*50);
        /* initialize sample footstep_list */
        gg.clear_footstep_node_list();
        gg.go_pos_param_2_footstep_list(0, 150*1e-3, 0, coordinates());
        gen_and_plot_walk_pattern(gg);
    };

    void test4 ()
    {
        gait_generator gg(dt, leg_pos, 1e-3*150, 1e-3*50, 10, 1e-3*50);
        /* initialize sample footstep_list */
        gg.clear_footstep_node_list();
        gg.go_pos_param_2_footstep_list(0, 0, 30, coordinates());
        gen_and_plot_walk_pattern(gg);
    };

    void test5 ()
    {
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
        gait_generator gg(dt, leg_pos, 1e-3*150, 1e-3*50, 10, 1e-3*50);
        /* initialize sample footstep_list */
        std::vector<hrp::Vector3> dzo;
        dzo.push_back(hrp::Vector3(20*1e-3,30*1e-3,0));
        dzo.push_back(hrp::Vector3(20*1e-3,-30*1e-3,0));
        gg.set_default_zmp_offsets(dzo);
        gg.clear_footstep_node_list();
        gg.go_pos_param_2_footstep_list(100*1e-3, 0, 0, coordinates());
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
      }
  }
  return 0;
}

