/* -*- coding:utf-8-unix; mode:c++; -*- */

#include "ZMPDistributor.h"
/* samples */
#include <cstdio>
#include <iostream>

int main(int argc, char* argv[])
{
    std::vector<std::vector<Eigen::Vector2d> > fs;
    std::vector<Eigen::Vector2d> tmpfs;
    tmpfs.push_back(Eigen::Vector2d(0.137525, 0.070104));
    tmpfs.push_back(Eigen::Vector2d(0.137525, -0.070104));
    tmpfs.push_back(Eigen::Vector2d(-0.106925, -0.070104));
    tmpfs.push_back(Eigen::Vector2d(-0.106925, 0.070104));
    fs.push_back(tmpfs);
    fs.push_back(tmpfs);
    SimpleZMPDistributor szd(fs);
    //
    hrp::Vector3 ref_foot_force[2], ref_foot_moment[2];
    std::vector<hrp::Vector3> ee_pos;
    std::vector<hrp::Vector3> cop_pos;
    std::vector<hrp::Matrix33> ee_rot;
    ee_pos.push_back(hrp::Vector3(0,-0.105,0));
    ee_pos.push_back(hrp::Vector3(0,0.105,0));
    cop_pos.push_back(hrp::Vector3(0,-0.105,0));
    cop_pos.push_back(hrp::Vector3(0,0.105,0));
    ee_rot.push_back(hrp::Matrix33::Identity());
    ee_rot.push_back(hrp::Matrix33::Identity());
    szd.print_params(std::string(""));
    szd.distributeZMPToForceMoments(ref_foot_force, ref_foot_moment,
                                    ee_pos, cop_pos, ee_rot,
                                    hrp::Vector3(0,0.05,0),
                                    hrp::Vector3(0,0.05,0),
                                    56*9.8066);
    std::cerr << "  rf =" << std::endl;
    std::cerr << ref_foot_force[0].format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << std::endl;
    std::cerr << "  lf =" << std::endl;
    std::cerr << ref_foot_force[1].format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << std::endl;
    std::cerr << "  rn =" << std::endl;
    std::cerr << ref_foot_moment[0].format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << std::endl;
    std::cerr << "  ln =" << std::endl;
    std::cerr << ref_foot_moment[1].format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << std::endl;
    return 0;
}

