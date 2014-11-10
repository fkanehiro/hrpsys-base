/* -*- coding:utf-8-unix; mode:c++; -*- */

#include "GaitGenerator.h"
/* samples */
using namespace rats;
#include <cstdio>

void test0 ()
{
  double dt = 0.005; /* [s] */
  std::vector<hrp::Vector3> leg_pos; /* default footstep transformations are necessary */
  leg_pos.push_back(hrp::Vector3(0,1e-3*-105,0)); /* rleg */
  leg_pos.push_back(hrp::Vector3(0,1e-3* 105,0)); /* lleg */
  gait_generator gg(dt, leg_pos, 1e-3*150, 1e-3*50, 10, 1e-3*50);

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
  //   gg.append_footstep_node("rleg", initial_swing_leg_dst_coords);
  //   gg.append_footstep_node("lleg", initial_support_leg_coords);
  //   gg.append_footstep_node("rleg", coordinates(hrp::Vector3(50, -105, 0)));
  //   gg.append_footstep_node("lleg", coordinates(hrp::Vector3(100, 105, 0)));
  //   gg.append_footstep_node("rleg", coordinates(hrp::Vector3(150, -105, 0)));
  //   gg.append_footstep_node("lleg", coordinates(hrp::Vector3(200, 105, 0)));
  //   gg.append_footstep_node("rleg", coordinates(hrp::Vector3(250, -105, 0)));
  //   gg.append_footstep_node("lleg", coordinates(hrp::Vector3(250, 105, 0)));
  //   gg.initialize_gait_parameter(cog, initial_support_leg_coords, initial_swing_leg_dst_coords);
  //   while ( !gg.proc_one_tick() );
  //   gg.print_footstep_list();

  //   /* make step and dump */
  //   size_t i = 0;
  //   std::string fname("/tmp/plot.dat");
  //   FILE* fp = fopen(fname.c_str(), "w");
  //   while ( gg.proc_one_tick() ) {
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
    gg.append_footstep_node("rleg", initial_swing_leg_dst_coords);
    gg.append_footstep_node("lleg", initial_support_leg_coords);
    gg.append_footstep_node("rleg", coordinates(hrp::Vector3(50*1e-3, -105*1e-3, 0)));
    gg.append_footstep_node("lleg", coordinates(hrp::Vector3(100*1e-3, 105*1e-3, 0)));
    gg.append_footstep_node("rleg", coordinates(hrp::Vector3(150*1e-3, -105*1e-3, 0)));
    gg.append_footstep_node("lleg", coordinates(hrp::Vector3(200*1e-3, 105*1e-3, 0)));
    gg.append_footstep_node("rleg", coordinates(hrp::Vector3(250*1e-3, -105*1e-3, 0)));
    gg.append_footstep_node("lleg", coordinates(hrp::Vector3(250*1e-3, 105*1e-3, 0)));
    gg.initialize_gait_parameter(cog, initial_support_leg_coords, initial_swing_leg_dst_coords);
    while ( !gg.proc_one_tick() );
    gg.print_footstep_list();

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
//   leg_pos.push_back(rb->get_end_coords("rleg").pos); /* rleg */
//   leg_pos.push_back(rb->get_end_coords("lleg").pos); /* lleg */
//   gait_generator gg(dt, leg_pos, stride_x, stride_y, stride_th); // for HRP2
//   if ( start_leg == "" ) {
//     gg.go_pos_param_2_footstep_list(xx, yy, th, coordinates());
//   } else {
//     gg.go_pos_param_2_footstep_list(xx, yy, th, coordinates(), (start_leg == "rleg" ? gait_generator::WC_RLEG : gait_generator::WC_LLEG));
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
//   leg_pos.push_back(rb->get_end_coords("rleg").pos); /* rleg */
//   leg_pos.push_back(rb->get_end_coords("lleg").pos); /* lleg */
//   gait_generator gg(dt, leg_pos, 150, 50, 10); // for HRP2
//   coordinates spc, swc;
//   std::string leg = "rleg";
//   double goal_x = 0.15, goal_y = 0, goal_z = 0, goal_theta = 0;
//   rb->get_end_coords(spc, (leg == "rleg") ? "lleg" : "rleg");
//   rb->get_end_coords(swc, (leg == "rleg") ? "rleg" : "lleg");
//   gg.clear_footstep_node_list();
//   gg.go_single_step_param_2_footstep_list(goal_x * 1000.0, goal_y * 1000.0, goal_z * 1000.0, goal_theta,
//                                           leg, spc);
//   std::vector<hrp::Vector3> default_zmp_offsets;
//   default_zmp_offsets.push_back(hrp::Vector3(0));
//   default_zmp_offsets.push_back(hrp::Vector3(0));
//   gg.set_default_zmp_offsets(default_zmp_offsets);
//   gg.initialize_gait_parameter(rb->calc_com(), swc, spc);
//   std::vector<hrp::Vector3> cog_v, spc_v, swc_v;
//   while ( !gg.proc_one_tick() );
//   while ( gg.proc_one_tick() ) {
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

