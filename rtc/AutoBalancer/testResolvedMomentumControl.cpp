/* -*- coding:utf-8-unix; mode:c++; -*- */
#include "ResolvedMomentumControl.h"
#include <rtm/Manager.h>
#include <rtm/CorbaNaming.h>
#include <hrpModel/Config.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <cmath>
#include <fstream>
#include <iostream>
// #include <boost/filesystem.hpp>

inline double deg2rad (double deg) { return deg * M_PI / 180.0; }

class testResolvedMomentumControl
{
protected:
    double dt; /* [s] */
    rats::RMController* rmc;
    hrp::BodyPtr m_robot;
    std::map<std::string, hrp::dvector6> xi_ref;
    std::map<std::string, std::string> end_effectors;
    bool use_gnuplot;
    virtual void setResetPose() {};

    void loadModel(const char *file_path)
    {
        RTC::Manager& rtcManager = RTC::Manager::instance();
        std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
        int comPos = nameServer.find(",");
        if (comPos < 0){
            comPos = nameServer.length();
        }
        nameServer = nameServer.substr(0, comPos);
        RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
        if (!loadBodyFromModelLoader(m_robot, file_path,
                                     CosNaming::NamingContext::_duplicate(naming.getRootContext()))) {
            std::cerr << "Failed to load model"  << std::endl;
            std::cerr << "Please check if openhrp-model-loader is running"  << std::endl;
            exit(1);
        }
    }

private:
    class calcReferenceParameter
    {
    public:
        virtual hrp::Vector3 calcPref(const hrp::BodyPtr m_robot, const double tm, const double max_tm) {};
        virtual hrp::Vector3 calcLref(const hrp::BodyPtr m_robot, const double tm, const double max_tm) {};
        virtual hrp::dvector6 calcXiref(const hrp::BodyPtr m_robot, const std::string &constraint, const double tm, const double max_tm) {};
    };

    void plot_and_save(FILE* gp, const std::string graph_fname, const std::string plot_str)
    {
        fprintf(gp, "%s\n unset multiplot\n", plot_str.c_str());
        fprintf(gp, "set terminal postscript eps color\nset output '/tmp/%s.eps'\n", graph_fname.c_str());
        fflush(gp);
    }

    void executeAndPlot(const double max_tm, calcReferenceParameter* calcRef)
    {
        std::string fname("/tmp/plot.dat");
        std::ofstream fp(fname.c_str());
        hrp::Vector3 Pref, Lref, cur_basePos, ref_basePos, P, L, CM;
        hrp::Matrix33 cur_baseRot, ref_baseRot;
        hrp::dvector cur_q(m_robot->numJoints());
        std::map<std::string, hrp::Vector3> p_constraint;
        std::map<std::string, hrp::Matrix33> R_constraint;
        std::map<std::string, hrp::dvector6> xi;

        for (double tm = 0; tm < max_tm; tm += dt) {
            Pref = calcRef->calcPref(m_robot, tm, max_tm);
            Lref = calcRef->calcLref(m_robot, tm, max_tm);

            for (std::map<std::string, hrp::dvector6>::iterator it = xi_ref.begin(); it != xi_ref.end(); ++it) {
                (*it).second = calcRef->calcXiref(m_robot, (*it).first, tm, max_tm);
            }
            for (size_t i = 0; i < m_robot->numJoints(); ++i) {
                cur_q(i) = m_robot->joint(i)->q;
            }
            for (std::map<std::string, hrp::dvector6>::iterator it = xi_ref.begin(); it != xi_ref.end(); ++it) {
                p_constraint[(*it).first] = m_robot->link((*it).first)->p;
                R_constraint[(*it).first] = m_robot->link((*it).first)->R;
            }
            cur_basePos = m_robot->rootLink()->p;
            cur_baseRot = m_robot->rootLink()->R;
            ref_basePos = cur_basePos + Pref / m_robot->totalMass() * dt;
            ref_baseRot = cur_baseRot;

            // execute RMC
            rmc->rmControl(m_robot, Pref, Lref, xi_ref, ref_basePos, ref_baseRot, dt);

            m_robot->rootLink()->v = (m_robot->rootLink()->p - cur_basePos) / dt;
            hrp::Matrix33 dR = cur_baseRot.transpose() * m_robot->rootLink()->R;
            m_robot->rootLink()->w = hrp::omegaFromRot(dR) / dt;

            hrp::dvector tmpPL(6);
            tmpPL << Pref, Lref;
            tmpPL = tmpPL.cwiseProduct(rmc->getSelectionVector());
            Pref = tmpPL.segment(0, 3);
            Lref = tmpPL.segment(3, 3);
            m_robot->calcForwardKinematics(true);
            CM = m_robot->calcCM();
            m_robot->calcTotalMomentumFromJacobian(P, L);

            for (size_t i = 0; i < m_robot->numJoints(); ++i) {
                m_robot->joint(i)->dq = (m_robot->joint(i)->q - cur_q(i)) / dt;
            }
            for (std::map<std::string, hrp::dvector6>::iterator it = xi_ref.begin(); it != xi_ref.end(); ++it) {
                xi[(*it).first].segment(0, 3) = (m_robot->link((*it).first)->p - p_constraint[(*it).first]) / dt;
                xi[(*it).first].segment(3, 3) = hrp::omegaFromRot(R_constraint[(*it).first].transpose() * m_robot->link((*it).first)->R) / dt;
            }

            fp << std::fixed;
            fp << tm
               << " " << Pref(0)
               << " " << P(0)
               << " " << Pref(1)
               << " " << P(1)
               << " " << Pref(2)
               << " " << P(2)
               << " " << Lref(0)
               << " " << L(0)
               << " " << Lref(1)
               << " " << L(1)
               << " " << Lref(2)
               << " " << L(2);
            // for (size_t i = 0; i < m_robot->numJoints(); ++i) {
            //     fp << " " << m_robot->joint(i)->dq;
            // }
            for (std::map<std::string, hrp::dvector6>::iterator it = xi_ref.begin(); it != xi_ref.end(); ++it) {
                for (size_t i = 0; i < 6; ++i) {
                    fp << " " << (*it).second(i)
                       << " " << xi[(*it).first](i);
                }
            }
            fp << "\n";
        }

        size_t gpsize = 2 + xi_ref.size() * 2; // P, L, xi_p, xi_R
        size_t gp_count = 0;
        size_t start = 2;
        FILE* gps[gpsize];
        for (size_t ii = 0; ii < gpsize; ii++) {
            gps[ii] = popen("gnuplot", "w");
        }
        // P
        {
            std::ostringstream oss("");
            std::string gtitle("P");
            oss << "set multiplot layout 3, 1 title '" << gtitle << "'" << std::endl;
            std::string titles[3] = {"X", "Y", "Z"};
            for (size_t ii = 0; ii < 3; ii++) {
                oss << "set xlabel 'Time [s]'" << std::endl;
                oss << "set ylabel '" << titles[ii] << "[Ns]'" << std::endl;
                oss << "plot "
                    << "'" << fname << "' using 1:" << (start + ii * 2) << " with points ps 0.2 title 'Pref',"
                    << "'" << fname << "' using 1:" << (start + ii * 2 + 1) << " with points ps 0.2 title 'P'"
                    << std::endl;
            }
            plot_and_save(gps[gp_count++], gtitle, oss.str());
            start += 6;
        }
        // L
        {
            std::ostringstream oss("");
            std::string gtitle("L");
            oss << "set multiplot layout 3, 1 title '" << gtitle << "'" << std::endl;
            std::string titles[3] = {"X", "Y", "Z"};
            for (size_t ii = 0; ii < 3; ii++) {
                oss << "set xlabel 'Time [s]'" << std::endl;
                oss << "set ylabel '" << titles[ii] << "[mNs]'" << std::endl;
                oss << "plot "
                    << "'" << fname << "' using 1:" << (start + ii * 2) << " with points ps 0.2 title 'Lref',"
                    << "'" << fname << "' using 1:" << (start + ii * 2 + 1) << " with points ps 0.2 title 'L'"
                    << std::endl;
            }
            plot_and_save(gps[gp_count++], gtitle, oss.str());
            start += 6;
        }
        // // CoM
        // {
        //     std::ostringstream oss("");
        //     std::string gtitle("CoM");
        //     oss << "set multiplot layout 1, 1 title '" << gtitle << "'" << std::endl;
        //     oss << "set xlabel 'Time [s]'" << std::endl;
        //     oss << "set ylabel '" << "Position [m]'" << std::endl;
        //     oss << "plot "
        //         << "'" << fname << "' using 1:" << (start) << " with lines title 'X',"
        //         << "'" << fname << "' using 1:" << (start + 1) << " with lines title 'Y',"
        //         << "'" << fname << "' using 1:" << (start + 2) << " with lines title 'Z'"
        //         << std::endl;
        //     plot_and_save(gps[2], gtitle, oss.str());
        // }

        // dq
        // {
        //     std::ostringstream oss("");
        //     std::string gtitle("dq");
        //     oss << "set multiplot layout 1, 1 title '" << gtitle << "'" << std::endl;
        //     oss << "set xlabel 'Time [s]'" << std::endl;
        //     oss << "set ylabel '" << "[rad / s]'" << std::endl;
        //     oss << "plot ";
        //     size_t i;
        //     for (i = 0; i < m_robot->numJoints() - 1; ++i) {
        //         oss << "'" << fname << "' using 1:" << (start + i) << " with lines title '" << m_robot->joint(i)->name << "',";
        //     }
        //     oss << "'" << fname << "' using 1:" << (start + i) << " with lines title '" << m_robot->joint(i)->name << "'";
        //     oss << std::endl;
        //     plot_and_save(gps[2], gtitle, oss.str());
        // }

        // xi
        for (std::map<std::string, hrp::dvector6>::iterator it = xi_ref.begin(); it != xi_ref.end(); ++it) {
            // p
            std::ostringstream oss("");
            std::string gtitle("xi_p " + (*it).first);
            oss << "set multiplot layout 3, 1 title '" << gtitle << "'" << std::endl;
            std::string titles[3] = {"X", "Y", "Z"};
            for (size_t ii = 0; ii < 3; ii++) {
                oss << "set xlabel 'Time [s]'" << std::endl;
                oss << "set ylabel '" << titles[ii] << "[m/s]'" << std::endl;
                oss << "plot "
                    << "'" << fname << "' using 1:" << (start + ii * 2) << " with points ps 0.2 title 'xi_p ref',"
                    << "'" << fname << "' using 1:" << (start + ii * 2 + 1) << " with points ps 0.2 title 'xi_p'"
                    << std::endl;
            }
            plot_and_save(gps[gp_count++], gtitle, oss.str());
            start += 6;

            // R
            oss.str("");
            oss.clear();
            gtitle = "xi_R " + (*it).first;
            oss << "set multiplot layout 3, 1 title '" << gtitle << "'" << std::endl;
            // std::string titles[3] = {"X", "Y", "Z"};
            for (size_t ii = 0; ii < 3; ii++) {
                oss << "set xlabel 'Time [s]'" << std::endl;
                oss << "set ylabel '" << titles[ii] << "[rad/s]'" << std::endl;
                oss << "plot "
                    << "'" << fname << "' using 1:" << (start + ii * 2) << " with points ps 0.2 title 'xi_R ref',"
                    << "'" << fname << "' using 1:" << (start + ii * 2 + 1) << " with points ps 0.2 title 'xi_R'"
                    << std::endl;
            }
            plot_and_save(gps[gp_count++], gtitle, oss.str());
            start += 6;
        }

        for (std::map<std::string, hrp::dvector6>::iterator it = xi_ref.begin(); it != xi_ref.end(); ++it) {
            rmc->removeConstraintLink(m_robot, (*it).first, xi_ref);
        }
        delete calcRef;

        double tmp;
        std::cin >> tmp;
        for (size_t ii = 0; ii < gpsize; ii++) {
            fprintf(gps[ii], "exit\n");
            fflush(gps[ii]);
            pclose(gps[ii]);
        }
    }

    void setEndeffectorConstraint(std::string &limb)
    {
        rmc->addConstraintLink(m_robot, end_effectors[limb]);
    }

public:
    std::vector<std::string> arg_strs;
    testResolvedMomentumControl() : use_gnuplot(true)
    {
        m_robot = hrp::BodyPtr(new hrp::Body());
    }

    virtual ~testResolvedMomentumControl()
    {
        if (rmc != NULL) {
            delete rmc;
            rmc = NULL;
        }
    }

    void test0()
    {
        std::cerr << "test0 : Control All Momentum with legs constraints" << std::endl;
        setResetPose();

        double max_tm = 4.0;
        hrp::dvector6 Svec;
        Svec << 1, 1, 1, 1, 1, 1;
        rmc->setSelectionMatrix(Svec);
        xi_ref[end_effectors["rleg"]] = hrp::dvector6::Zero(6, 1);
        xi_ref[end_effectors["lleg"]] = hrp::dvector6::Zero(6, 1);

        for (std::map<std::string, hrp::dvector6>::iterator it = xi_ref.begin(); it != xi_ref.end(); ++it) {
            rmc->addConstraintLink(m_robot, (*it).first);
        }

        class calcRefParamTest0 : public calcReferenceParameter
        {
        public:
            hrp::Vector3 calcPref(const hrp::BodyPtr m_robot, const double tm, const double max_tm)
            {
                hrp::Vector3 Pref;
                Pref(0) = 0.01 * sin(tm * M_PI / max_tm * 4);
                Pref(1) = 0.01 * sin(tm * M_PI / max_tm * 4);
                Pref(2) = 0.02 * sin(tm * M_PI / max_tm * 4);
                Pref *= m_robot->totalMass();
                return Pref;
            }

            hrp::Vector3 calcLref(const hrp::BodyPtr m_robot, const double tm, const double max_tm)
            {
                hrp::Vector3 Lref;
                Lref(0) = 0.1 * sin(tm * M_PI / max_tm * 4);
                Lref(1) = 0.1 * sin(tm * M_PI / max_tm * 4);
                Lref(2) = 0.2 * sin(tm * M_PI / max_tm * 4);
                return Lref;
            }

            hrp::dvector6 calcXiref(const hrp::BodyPtr m_robot, const std::string &constraint, const double tm, const double max_tm)
            {
                hrp::dvector6 xi_ref = hrp::dvector6::Zero();
                // xi_ref(2) = 0.00 * sin(tm * M_PI / max_tm * 4);
                return xi_ref;
            }
        };

        calcRefParamTest0* calcRef = new calcRefParamTest0();
        executeAndPlot(max_tm, calcRef);
    }

    void test1()
    {
        std::cerr << "test1 : Control all momentum with lleg and rarm constraint" << std::endl;
        setResetPose();

        double max_tm = 4.0;
        hrp::dvector6 Svec;
        Svec << 1, 1, 1, 1, 1, 1;
        rmc->setSelectionMatrix(Svec);
        xi_ref[end_effectors["lleg"]] = hrp::dvector6::Zero(6, 1);
        xi_ref[end_effectors["rarm"]] = hrp::dvector6::Zero(6, 1);

        for (std::map<std::string, hrp::dvector6>::iterator it = xi_ref.begin(); it != xi_ref.end(); ++it) {
            rmc->addConstraintLink(m_robot, (*it).first);
        }

        class calcRefParamTest1 : public calcReferenceParameter
        {
        public:
            hrp::Vector3 calcPref(const hrp::BodyPtr m_robot, const double tm, const double max_tm)
            {
                hrp::Vector3 Pref;
                Pref(0) = 0.01 * cos(tm * M_PI / max_tm * 4);
                Pref(1) = 0.01 * sin(tm * M_PI / max_tm * 4);
                Pref(2) = 0.02 * cos(tm * M_PI / max_tm * 4);
                Pref *= m_robot->totalMass();
                return Pref;
            }

            hrp::Vector3 calcLref(const hrp::BodyPtr m_robot, const double tm, const double max_tm)
            {
                hrp::Vector3 Lref;
                Lref(0) = 0.1 * cos(tm * M_PI / max_tm * 4);
                Lref(1) = 0.1 * sin(tm * M_PI / max_tm * 4);
                Lref(2) = 0.2 * cos(tm * M_PI / max_tm * 4);
                return Lref;
            }

            hrp::dvector6 calcXiref(const hrp::BodyPtr m_robot, const std::string &constraint, const double tm, const double max_tm)
            {
                hrp::dvector6 xi_ref = hrp::dvector6::Zero();
                return xi_ref;
            }
        };

        calcRefParamTest1* calcRef = new calcRefParamTest1();
        executeAndPlot(max_tm, calcRef);
    }

    void test2()
    {
        std::cerr << "test2 : Control all momentum with arms constraints" << std::endl;
        setResetPose();

        double max_tm = 4.0;
        hrp::dvector6 Svec;
        Svec << 1, 1, 1, 1, 1, 1;
        rmc->setSelectionMatrix(Svec);
        xi_ref[end_effectors["rarm"]] = hrp::dvector6::Zero(6, 1);
        xi_ref[end_effectors["larm"]] = hrp::dvector6::Zero(6, 1);

        for (std::map<std::string, hrp::dvector6>::iterator it = xi_ref.begin(); it != xi_ref.end(); ++it) {
            rmc->addConstraintLink(m_robot, (*it).first);
        }

        class calcRefParamTest2 : public calcReferenceParameter
        {
        public:
            hrp::Vector3 calcPref(const hrp::BodyPtr m_robot, const double tm, const double max_tm)
            {
                hrp::Vector3 Pref;
                Pref(0) = 0.01 * cos(tm * M_PI / max_tm * 4);
                Pref(1) = 0.01 * sin(tm * M_PI / max_tm * 4);
                Pref(2) = 0.02 * cos(tm * M_PI / max_tm * 4);
                Pref *= m_robot->totalMass();
                return Pref;
            }

            hrp::Vector3 calcLref(const hrp::BodyPtr m_robot, const double tm, const double max_tm)
            {
                hrp::Vector3 Lref;
                Lref(0) = 0.1 * cos(tm * M_PI / max_tm * 4);
                Lref(1) = 0.1 * sin(tm * M_PI / max_tm * 4);
                Lref(2) = 0.2 * cos(tm * M_PI / max_tm * 4);
                return Lref;
            }

            hrp::dvector6 calcXiref(const hrp::BodyPtr m_robot, const std::string &constraint, const double tm, const double max_tm)
            {
                hrp::dvector6 xi_ref = hrp::dvector6::Zero();
                return xi_ref;
            }
        };

        calcRefParamTest2* calcRef = new calcRefParamTest2();
        executeAndPlot(max_tm, calcRef);
    }

    void test3()
    {
        std::cerr << "test3 : Control pitch and yaw axis angular momentum with leg constraints" << std::endl;
        setResetPose();

        double max_tm = 4.0;
        hrp::dvector6 Svec;
        Svec << 0, 0, 0, 1, 0, 1;
        rmc->setSelectionMatrix(Svec);
        xi_ref[end_effectors["rleg"]] = hrp::dvector6::Zero(6, 1);
        xi_ref[end_effectors["lleg"]] = hrp::dvector6::Zero(6, 1);

        for (std::map<std::string, hrp::dvector6>::iterator it = xi_ref.begin(); it != xi_ref.end(); ++it) {
            rmc->addConstraintLink(m_robot, (*it).first);
        }

        class calcRefParamTest3 : public calcReferenceParameter
        {
        public:
            hrp::Vector3 calcPref(const hrp::BodyPtr m_robot, const double tm, const double max_tm)
            {
                hrp::Vector3 Pref;
                Pref(0) = 0.01 * cos(tm * M_PI / max_tm * 4);
                Pref(1) = 0.01 * sin(tm * M_PI / max_tm * 4);
                Pref(2) = 0.02 * cos(tm * M_PI / max_tm * 4);
                Pref *= m_robot->totalMass();
                return Pref;
            }

            hrp::Vector3 calcLref(const hrp::BodyPtr m_robot, const double tm, const double max_tm)
            {
                hrp::Vector3 Lref;
                Lref(0) = 0.1 * cos(tm * M_PI / max_tm * 4);
                Lref(1) = 0.1 * sin(tm * M_PI / max_tm * 4);
                Lref(2) = 0.2 * cos(tm * M_PI / max_tm * 4);
                return Lref;
            }

            hrp::dvector6 calcXiref(const hrp::BodyPtr m_robot, const std::string &constraint, const double tm, const double max_tm)
            {
                hrp::dvector6 xi_ref = hrp::dvector6::Zero();
                return xi_ref;
            }
        };

        calcRefParamTest3* calcRef = new calcRefParamTest3();
        executeAndPlot(max_tm, calcRef);
    }

    void parseParams()
    {
        for (int i = 0; i < arg_strs.size(); ++ i) {
            if ( arg_strs[i]== "--use-gnuplot" ) {
                if (++i < arg_strs.size()) use_gnuplot = (arg_strs[i]=="true");
            }
        }
    }

    bool check_all_results ()
    {
        // return is_small_zmp_error && is_small_zmp_diff && is_contact_states_swing_support_time_validity;
        return true;
    }
};

class testResolvedMomentumControlSampleRobot : public testResolvedMomentumControl
{
protected:
    void setResetPose()
    {
        // Reset Pose
        double reset_pose_angles[] = {-0.004457, -21.692900, -0.012020, 47.672300, -25.930000, 0.014025,
                                      17.835600, -9.137590, -6.611880, -36.456000, 0.000000, 0.000000, 0.000000,
                                      -0.004457, -21.692900, -0.012020, 47.672300, -25.930000, 0.014025,
                                      17.835600, 9.137590, 6.611880, -36.456000, 0.000000, 0.000000, 0.000000,
                                      0.000000, 0.000000, 0.000000};
        for (size_t i = 0; i < m_robot->numJoints(); ++i) {
            m_robot->joint(i)->q = deg2rad(reset_pose_angles[i]);
        }
        m_robot->calcForwardKinematics();
    }
public:
    testResolvedMomentumControlSampleRobot ()
    {
        dt = 0.004;
        end_effectors["rleg"] = "RLEG_ANKLE_R";
        end_effectors["lleg"] = "LLEG_ANKLE_R";
        end_effectors["rarm"] = "RARM_WRIST_P";
        end_effectors["larm"] = "LARM_WRIST_P";
#ifdef OPENHRP_DIR
        std::string model_path = "file:///" + std::string(OPENHRP_DIR) + "/share/openhrp3/share/OpenHRP-3.1/sample/model/sample1.wrl";
        loadModel(model_path.c_str());
        rmc = new rats::RMController(m_robot);
#else
        std::cerr << "OPENHRP_DIR not found"  << std::endl;
        exit(1);
#endif
    }
};

void print_usage ()
{
    std::cerr << "Usage : testResolvedMomentumControl [option]" << std::endl;
    std::cerr << " [option] should be:" << std::endl;
    std::cerr << "  --test0 : Control All Momentum with legs constraint" << std::endl;
    std::cerr << "  --test1 : Control All Momentum with lleg and rarm constraint" << std::endl;
    std::cerr << "  --test2 : Control All Momentum with arms constraints" << std::endl;
    std::cerr << "  --test3 : Control pitch and yaw axis angular momentum with leg constraints" << std::endl;
}

int main(int argc, char* argv[])
{
  int ret = 0;
  if (argc >= 2) {
      testResolvedMomentumControlSampleRobot trmc;
      for (int i = 1; i < argc; ++ i) {
          trmc.arg_strs.push_back(std::string(argv[i]));
      }
      if (std::string(argv[1]) == "--test0") {
          trmc.test0();
      } else if (std::string(argv[1]) == "--test1") {
          trmc.test1();
      } else if (std::string(argv[1]) == "--test2") {
          trmc.test2();
      } else if (std::string(argv[1]) == "--test3") {
          trmc.test3();
      } else {
          print_usage();
          ret = 1;
      }
      ret = (trmc.check_all_results() ? 0 : 2);
  } else {
      print_usage();
      ret = 1;
  }
  return ret;
}
