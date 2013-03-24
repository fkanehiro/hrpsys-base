#undef NDEBUG
#include <assert.h>
#include <gtest/gtest.h>

#include <iostream>
#include <string>
#include <hrpModel/ModelLoaderUtil.h>

#define private public
#include "rtc/RobotHardware/robot.h"



int _argc;
char** _argv;

class test_RobotHardware : public ::testing::Test {
public:
    std::string pdgains_file_name;

    virtual void SetUp() {
        // check loadGain()
        m_robot = boost::shared_ptr<robot>(new robot());
        std::string model_file = std::string(OPENHRP_DIR)+"/share/OpenHRP-3.1/sample/model/PA10/pa10.main.wrl";
        //std::string model_file = "/home/k-okada/Downloads/HIRONX_110822/main.wrl";
        pdgains_file_name = std::string(PROJECT_SOURCE_DIR)+"/test/pdgains.sav";

        ASSERT_TRUE(loadBodyFromModelLoader(m_robot, model_file.c_str(), _argc, _argv));
        EXPECT_TRUE(m_robot->init());
    }

    virtual void TearDown() {
    }

protected:
    boost::shared_ptr<robot> m_robot;
};

TEST_F(test_RobotHardware, init) {
    // init
    EXPECT_TRUE(m_robot->init());
}

TEST_F(test_RobotHardware, numJoints) {
    // check robot joint size
    EXPECT_EQ(m_robot->numJoints(), 9);
}

TEST_F(test_RobotHardware, setProperty_default_pdgains) {
    m_robot->init();
    // check defualt gain
    EXPECT_EQ(m_robot->pgain[0],0);
    EXPECT_EQ(m_robot->old_pgain[0],0);
    EXPECT_EQ(m_robot->default_pgain[0],0);
    EXPECT_EQ(m_robot->dgain[0],0);
    EXPECT_EQ(m_robot->old_dgain[0],0);
    EXPECT_EQ(m_robot->default_dgain[0],0);
}

TEST_F(test_RobotHardware, setProperty_loaded_pdgains) {
    // check loaded gain
    m_robot->setProperty("pdgains.file_name", pdgains_file_name.c_str());
    m_robot->init();

    EXPECT_EQ(m_robot->pgain[0],10);
    EXPECT_EQ(m_robot->old_pgain[0],0);
    EXPECT_EQ(m_robot->default_pgain[0],10);
    EXPECT_EQ(m_robot->dgain[0],100);
    EXPECT_EQ(m_robot->old_dgain[0],0);
    EXPECT_EQ(m_robot->default_dgain[0],100);
}

TEST_F(test_RobotHardware, gain_control) {
    // check loaded gain
    m_robot->setProperty("pdgains.file_name", pdgains_file_name.c_str());
    m_robot->init();

    // copy from gain_control
    int GAIN_COUNT = 5*200 ;
    m_robot->gain_counter[0] = 0;
    double new_pgain=0,new_dgain=0;
    double from_pgain=0,from_dgain=0,to_pgain=0,to_dgain=0;

    from_pgain = 0;
    from_dgain = 0;
    m_robot->servo(0, true);
    to_pgain = m_robot->pgain[0];
    to_dgain = m_robot->dgain[0];
    std::cout << "pgain: " << from_pgain << " to " << to_pgain << std::endl;
    std::cout << "dgain: " << from_dgain << " to " << to_dgain << std::endl;
    while (m_robot->gain_counter[0] < GAIN_COUNT){
        m_robot->gain_counter[0]++;
        new_pgain = (m_robot->pgain[0]-m_robot->old_pgain[0])*m_robot->gain_counter[0]/GAIN_COUNT + m_robot->old_pgain[0];
        new_dgain = (m_robot->dgain[0]-m_robot->old_dgain[0])*m_robot->gain_counter[0]/GAIN_COUNT + m_robot->old_dgain[0];
        const double p = (to_pgain-from_pgain)*m_robot->gain_counter[0]/GAIN_COUNT + from_pgain;
        const double d = (to_dgain-from_dgain)*m_robot->gain_counter[0]/GAIN_COUNT + from_dgain;
        EXPECT_EQ(p, new_pgain);
        EXPECT_EQ(d, new_dgain);
    }

    from_pgain = m_robot->pgain[0];
    from_dgain = m_robot->dgain[0];
    m_robot->setServoGainPercentage("ALL", 50);
    to_pgain = m_robot->pgain[0];
    to_dgain = m_robot->dgain[0];
    std::cout << "pgain: " << from_pgain << " to " << to_pgain << std::endl;
    std::cout << "dgain: " << from_dgain << " to " << to_dgain << std::endl;
    while (m_robot->gain_counter[0] < GAIN_COUNT){
        m_robot->gain_counter[0]++;
        new_pgain = (m_robot->pgain[0]-m_robot->old_pgain[0])*m_robot->gain_counter[0]/GAIN_COUNT + m_robot->old_pgain[0];
        new_dgain = (m_robot->dgain[0]-m_robot->old_dgain[0])*m_robot->gain_counter[0]/GAIN_COUNT + m_robot->old_dgain[0];
        const double p = (to_pgain-from_pgain)*m_robot->gain_counter[0]/GAIN_COUNT + from_pgain;
        const double d = (to_dgain-from_dgain)*m_robot->gain_counter[0]/GAIN_COUNT + from_dgain;
        EXPECT_EQ(p, new_pgain);
        EXPECT_EQ(d, new_dgain);
    }

    m_robot->servo(0, false);
    from_pgain = 0;
    from_dgain = 0;
    m_robot->servo(0, true);
    to_pgain = m_robot->pgain[0];
    to_dgain = m_robot->dgain[0];
    std::cout << "pgain: " << from_pgain << " to " << to_pgain << std::endl;
    std::cout << "dgain: " << from_dgain << " to " << to_dgain << std::endl;
    while (m_robot->gain_counter[0] < GAIN_COUNT){
        m_robot->gain_counter[0]++;
        new_pgain = (m_robot->pgain[0]-m_robot->old_pgain[0])*m_robot->gain_counter[0]/GAIN_COUNT + m_robot->old_pgain[0];
        new_dgain = (m_robot->dgain[0]-m_robot->old_dgain[0])*m_robot->gain_counter[0]/GAIN_COUNT + m_robot->old_dgain[0];
        const double p = (to_pgain-from_pgain)*m_robot->gain_counter[0]/GAIN_COUNT + from_pgain;
        const double d = (to_dgain-from_dgain)*m_robot->gain_counter[0]/GAIN_COUNT + from_dgain;
        EXPECT_EQ(p, new_pgain);
        EXPECT_EQ(d, new_dgain);
    }
}

int main (int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    _argc = argc;
    _argv = argv;
    return RUN_ALL_TESTS();

}
