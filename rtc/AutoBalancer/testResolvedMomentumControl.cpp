/* -*- coding:utf-8-unix; mode:c++; -*- */
#include "ResolvedMomentumControl.h"
#include <rtm/Manager.h>
#include <rtm/CorbaNaming.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <cmath>

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
    void plot_and_save(FILE* gp, const std::string graph_fname, const std::string plot_str)
    {
        fprintf(gp, "%s\n unset multiplot\n", plot_str.c_str());
        fprintf(gp, "set terminal postscript eps color\nset output '/tmp/%s.eps'\n", graph_fname.c_str());
        fprintf(gp, "%s\n unset multiplot\n", plot_str.c_str());
        fflush(gp);
    }

    template<class Functor1, class Functor2>
    void executeAndPlot(const double max_tm, Functor1 calcP, Functor2 calcL)
    {
        std::string fname("/tmp/plot.dat");
        FILE* fp = fopen(fname.c_str(), "w");
        hrp::Vector3 Pref, Lref, ref_basePos, P, L, CM_old, CM, CMref;
        hrp::Matrix33 ref_baseRot;

        for (double tm = 0; tm < max_tm; tm += dt) {
            Pref = calcP(m_robot, tm, max_tm);
            Lref = calcL(m_robot, tm, max_tm);
            CM_old = m_robot->calcCM();

            if (tm < 0.2) std::cout << "root_p: " << (m_robot->rootLink()->p).transpose() << std::endl << "root_R: " << std::endl << (m_robot->rootLink()->R) << std::endl;
            std::cout << "CM_old: " << CM_old.transpose() << std::endl;

            hrp::dvector dq(m_robot->numJoints());
            for (size_t i = 0; i < m_robot->numJoints(); ++i) {
                dq(i) = -m_robot->joint(i)->q;
            }

            ref_basePos = m_robot->rootLink()->p + Pref / m_robot->totalMass();
            ref_baseRot = m_robot->rootLink()->R;

            rmc->rmControl(m_robot, Pref, Lref, xi_ref, ref_basePos, ref_baseRot, dt);

            hrp::Vector3 omega;
            hrp::dmatrix dRRT = ((ref_baseRot - m_robot->rootLink()->R) / dt) * (m_robot->rootLink()->R).transpose();
            omega(0) = (-dRRT(1, 2) + dRRT(2, 1)) / 2.0;
            omega(1) = (-dRRT(2, 0) + dRRT(0, 2)) / 2.0;
            omega(2) = (-dRRT(0, 1) + dRRT(1, 0)) / 2.0;

            std::cerr << "dif root_p: " << (ref_basePos - m_robot->rootLink()->p).transpose() << std::endl;

            m_robot->rootLink()->p = ref_basePos;
            m_robot->rootLink()->R = ref_baseRot;
            m_robot->calcForwardKinematics();
            // P
            CM = m_robot->calcCM();
            P = m_robot->totalMass() * (CM - CM_old) / dt;
            // L
            hrp::dmatrix Jl;
            m_robot->calcAngularMomentumJacobian(NULL, Jl); // Eq.1 lower [H 0 I]
            for (size_t i = 0; i < m_robot->numJoints(); ++i) {
                dq(i) += m_robot->joint(i)->q;
                dq(i) /= dt;
            }
            hrp::dvector v_vec(6 + m_robot->numJoints());
            v_vec << dq, hrp::Vector3::Zero(), omega;
            L = Jl * v_vec;

            fprintf(fp, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
                    tm,
                    Pref(0),
                    P(0),
                    Pref(1),
                    P(1),
                    Pref(2),
                    P(2),
                    Lref(0),
                    L(0),
                    Lref(1),
                    L(1),
                    Lref(2),
                    L(2),
                    CM(0),
                    CM(1),
                    CM(2)
                    );
        }

        for (std::map<std::string, hrp::dvector6>::iterator it = xi_ref.begin(); it != xi_ref.end(); ++it) {
            rmc->removeConstraintLink((*it).first, xi_ref);
        }
        fclose(fp);

        size_t gpsize = 3;
        size_t start = 2;
        FILE* gps[gpsize];
        for (size_t ii = 0; ii < gpsize;ii++) {
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
                oss << "set ylabel '" << titles[ii] << "[kg m/s]'" << std::endl;
                oss << "plot "
                    << "'" << fname << "' using 1:" << (start + ii * 2) << " with lines title 'Pref',"
                    << "'" << fname << "' using 1:" << (start + ii * 2 + 1) << " with lines title 'P'"
                    << std::endl;
            }
            plot_and_save(gps[0], gtitle, oss.str());
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
                    << "'" << fname << "' using 1:" << (start + ii * 2) << " with lines title 'Lref',"
                    << "'" << fname << "' using 1:" << (start + ii * 2 + 1) << " with lines title 'L'"
                    << std::endl;
            }
            plot_and_save(gps[1], gtitle, oss.str());
            start += 6;
        }
        // CoM
        {
            std::ostringstream oss("");
            std::string gtitle("CoM");
            oss << "set multiplot layout 1, 1 title '" << gtitle << "'" << std::endl;
            oss << "set xlabel 'Time [s]'" << std::endl;
            oss << "set ylabel '" << "Position [m]'" << std::endl;
            oss << "plot "
                << "'" << fname << "' using 1:" << (start) << " with lines title 'X',"
                << "'" << fname << "' using 1:" << (start + 1) << " with lines title 'Y',"
                << "'" << fname << "' using 1:" << (start + 2) << " with lines title 'Z'"
                << std::endl;
            plot_and_save(gps[2], gtitle, oss.str());
        }

        double tmp;
        std::cin >> tmp;
        for (size_t ii = 0; ii < gpsize; ii++) {
            fprintf(gps[ii], "exit\n");
            fflush(gps[ii]);
            pclose(gps[ii]);
        }
        // std::cerr << "Checking" << std::endl;
        // std::cerr << "  ZMP error : " << is_small_zmp_error << std::endl;
        // std::cerr << "  ZMP diff : " << is_small_zmp_diff << std::endl;
        // std::cerr << "  Contact states & swing support time validity : " << is_contact_states_swing_support_time_validity << std::endl;
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
        std::cerr << "test0 : Control All Momentum" << std::endl;
        setResetPose();

        double max_tm = 8.0;
        hrp::dvector6 Svec;
        Svec << 1, 1, 1, 1, 1, 1;
        rmc->setSelectionMatrix(Svec);
        xi_ref[end_effectors["rleg"]] = hrp::dvector6::Zero(6, 1);
        xi_ref[end_effectors["lleg"]] = hrp::dvector6::Zero(6, 1);

        for (std::map<std::string, hrp::dvector6>::iterator it = xi_ref.begin(); it != xi_ref.end(); ++it) {
            rmc->addConstraintLink(m_robot, (*it).first);
        }

        struct P {
            static hrp::Vector3 calcP(const hrp::BodyPtr m_robot, const double tm, const double max_tm)
            {
                hrp::Vector3 Pref;
                Pref(0) = 0.001 * cos(tm * M_PI / max_tm * 4);
                Pref(1) = 0.001 * sin(tm * M_PI / max_tm * 4);
                Pref(2) = 0.002 * cos(tm * M_PI / max_tm * 4);
                Pref *= m_robot->totalMass();
                return Pref;
            }
        };


        struct L {
            static hrp::Vector3 calcL(const hrp::BodyPtr m_robot, const double tm, const double max_tm)
            {
                hrp::Vector3 Lref;
                Lref(0) = 0.01 * cos(tm * M_PI / max_tm * 4);
                Lref(1) = 0.01 * sin(tm * M_PI / max_tm * 4);
                Lref(2) = 0.02 * cos(tm * M_PI / max_tm * 4);
                return Lref;
            }
        };
        executeAndPlot(max_tm, P::calcP, L::calcL);
    }

    void test1()
    {
        std::cerr << "test1 : Move only P_z" << std::endl;
        setResetPose();

        double max_tm = 8.0;
        hrp::dvector6 Svec;
        Svec << 1, 1, 1, 1, 1, 1;
        rmc->setSelectionMatrix(Svec);
        xi_ref[end_effectors["rleg"]] = hrp::dvector6::Zero(6, 1);
        xi_ref[end_effectors["lleg"]] = hrp::dvector6::Zero(6, 1);

        for (std::map<std::string, hrp::dvector6>::iterator it = xi_ref.begin(); it != xi_ref.end(); ++it) {
            rmc->addConstraintLink(m_robot, (*it).first);
        }

        struct P {
            static hrp::Vector3 calcP(const hrp::BodyPtr m_robot, const double tm, const double max_tm)
            {
                hrp::Vector3 Pref;
                Pref(0) = 0;
                Pref(1) = 0;
                Pref(2) = 0.002 * sin(tm * M_PI / max_tm * 4);
                Pref *= m_robot->totalMass();
                return Pref;
            }
        };


        struct L {
            static hrp::Vector3 calcL(const hrp::BodyPtr m_robot, const double tm, const double max_tm)
            {
                hrp::Vector3 Lref;
                Lref(0) = 0;//.2 * cos(tm * M_PI / max_tm * 4);
                Lref(1) = 0;
                Lref(2) = 0;
                return Lref;
            }
        };

        executeAndPlot(max_tm, P::calcP, L::calcL);
    }

    void test2()
    {
        // std::cerr << "test2 : Control All Momentum" << std::endl;
        // double max_tm = 8.0;
        // hrp::dvector6 Svec;
        // Svec << 1, 1, 1, 1, 1, 1;
        // rmc->setSelectionMatrix(Svec);
        // xi_ref[end_effectors["rleg"]] = hrp::dvector6::Zero(6, 1);
        // xi_ref[end_effectors["lleg"]] = hrp::dvector6::Zero(6, 1);

        // for (std::map<std::string, hrp::dvector6>::iterator it = xi_ref.begin(); it != xi_ref.end(); ++it) {
        //     rmc->addConstraintLink(m_robot, (*it).first);
        // }

        // struct P {
        //     hrp::Vector3 operator() (const double tm, const double max_tm)
        //     {
        //         hrp::Vector3 Pref;
        //         Pref(0) = 0.001 * cos(tm * M_PI / max_tm * 4);
        //         Pref(1) = 0.001 * sin(tm * M_PI / max_tm * 4);
        //         Pref(2) = 0.002 * cos(tm * M_PI / max_tm * 4);
        //         Pref *= m_robot->totalMass();
        //         return Pref;
        //     }
        // };

        // struct L {
        //     hrp::Vector3 operator() (const double tm, const double max_tm)
        //     {
        //         hrp::Vector3 Lref;
        //         Lref(0) = 0.01 * cos(tm * M_PI / max_tm * 4);
        //         Lref(1) = 0.01 * sin(tm * M_PI / max_tm * 4);
        //         Lref(2) = 0.02 * cos(tm * M_PI / max_tm * 4);
        //         return Lref;
        //     }
        // };

        // executeAndPlot(max_tm, P(), L());
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
        double reset_pose_joints[] = {-0.004457, -21.692900, -0.012020, 47.672300, -25.930000, 0.014025,
                                      17.835600, -9.137590, -6.611880, -36.456000, 0.000000, 0.000000, 0.000000,
                                      -0.004457, -21.692900, -0.012020, 47.672300, -25.930000, 0.014025,
                                      17.835600, 9.137590, 6.611880, -36.456000, 0.000000, 0.000000, 0.000000,
                                      0.000000, 0.000000, 0.000000};
        for (size_t i = 0; i < m_robot->numJoints(); ++i) {
            m_robot->joint(i)->q += deg2rad(reset_pose_joints[i]);
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
        loadModel("file://../share/openhrp3/share/OpenHRP-3.1/sample/model/sample1.wrl");
        rmc = new rats::RMController(m_robot);
    }
};

class testResolvedMomentumControlHRP2JSK : public testResolvedMomentumControl
{
public:
    testResolvedMomentumControlHRP2JSK ()
    {
        dt = 0.004;
        end_effectors["rleg"] = "RLEG_JOINT5";
        end_effectors["lleg"] = "LLEG_JOINT5";
        end_effectors["rarm"] = "RARM_JOINT6";
        end_effectors["larm"] = "LARM_JOINT6";
        loadModel("file://../share/openhrp3/share/OpenHRP-3.1/sample/model/sample1.wrl");
        rmc = new rats::RMController(m_robot);
    };
};

void print_usage ()
{
    std::cerr << "Usage : testResolvedMomentumControl [option]" << std::endl;
    std::cerr << " [option] should be:" << std::endl;
    std::cerr << "  --test0 : Control All Momentum" << std::endl;
    std::cerr << "  --test1 : Control All Momentum" << std::endl;
    std::cerr << "  --test2 : Control All Momentum" << std::endl;
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
