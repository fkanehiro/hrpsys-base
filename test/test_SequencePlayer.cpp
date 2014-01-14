#undef NDEBUG
#include <assert.h>
#include <gtest/gtest.h>

#include <iostream>
#include <string>
#include <hrpModel/ModelLoaderUtil.h>
#include <boost/random.hpp>

#define private public
#include "rtc/SequencePlayer/seqplay.h"



int _argc;
char** _argv;

class test_SequencePlayer : public ::testing::Test {
public:
    std::string pdgains_file_name;
    boost::variate_generator< boost::minstd_rand&, boost::uniform_real<> > *rand;

    virtual void SetUp() {
        m_dt = 0.005;
        m_robot = hrp::BodyPtr(new Body());
        std::string model_file = std::string(OPENHRP_DIR)+"/share/OpenHRP-3.1/sample/model/PA10/pa10.main.wrl";

        // set up random
        // 「線形合同法」( Seed=42 ) で
        // 「一様乱数」(0.0以上1.0未満) を生成
        boost::minstd_rand    gen( 42 );
        boost::uniform_real<> dst( -100, 100 );
        rand = new boost::variate_generator< boost::minstd_rand&, boost::uniform_real<> >(gen, dst);

        // load model
        ASSERT_TRUE(loadBodyFromModelLoader(m_robot, model_file.c_str(), _argc, _argv));

        // check model
        ASSERT_EQ(m_robot->numJoints(), 9);

        // initialize seqplay
        ASSERT_TRUE(m_seq = new seqplay(m_robot->numJoints(), m_dt));
    }

    virtual void TearDown() {
    }

    void testSetAngles(double *av) {
        double q[9], zmp[3], acc[3], pos[3], rpy[3], tq[9];

        while (!m_seq->isEmpty()) {
            m_seq->get(q, zmp, acc, pos, rpy, tq);
            for (int i = 0; i < 9; i++ ) {
                // check if no over shoot
                //std::cerr << i << " " << q[i] << " " << av[i] << std::endl;
                ASSERT_PRED_FORMAT2(testing::DoubleLE, 0, q[i]);
                ASSERT_PRED_FORMAT2(testing::DoubleLE, q[i], av[i]);
            }
        }
        for (int i = 0; i < 9; i++) {
            ASSERT_NEAR(av[i],q[i],1.0e-6);
        }
    }

protected:
    hrp::BodyPtr m_robot;
    seqplay *m_seq;
    double m_dt;
};

TEST_F(test_SequencePlayer, setJointAngles) {
    // init
    double av[9] = {0,1,2,3,4,5,6,7,8};

    m_seq->setJointAngles(av, 0.5);

    // test
    testSetAngles(av);
}

TEST_F(test_SequencePlayer, setJointAnglesContiniously) {
    // init
    int loop = 10;
    double av[9] = {0,1,2,3,4,5,6,7,8}, av_old[9] = {0,0,0,0,0,0,0,0,0};
    double q[9], q_old[9], tq[9];
    double zmp[3], acc[3], pos[3], rpy[3];

    m_seq->setJointAngles(av, 1);
    while (!m_seq->isEmpty()) {
        if ( loop > 0 &&
             m_seq->interpolators[seqplay::Q]->remain_t <
             m_seq->interpolators[seqplay::Q]->target_t/2) {
            for (int i = 0; i < 9; i ++ ) {
                av_old[i] = q[i];
                av[i] = (*rand)();
            }
            m_seq->setJointAngles(av, 1);
            loop--;
        }
        for (int i = 0; i < 9; i++ ) { q_old[i] = q[i]; }
        m_seq->get(q, zmp, acc, pos, rpy, tq);
        for (int i = 0; i < 9; i++ ) {
            // check velocity
            //std::cerr << i << " " << av_old[i] << " " << q[i] << " " << av[i] << " " << (q[i] - q_old[i])/m_dt << std::endl;
            double vel = fabs(q[i] - q_old[i])/m_dt;
            ASSERT_PRED_FORMAT2(testing::DoubleLE, 0, vel);
            ASSERT_PRED_FORMAT2(testing::DoubleLE, vel, 800); // 0.5 +- 200  = 800 
        }
    }
    for (int i = 0; i < 9; i++) {
        ASSERT_NEAR(av[i],q[i],1.0e-6);
    }
}

TEST_F(test_SequencePlayer, playPattern) {
    // init
    int loop = 100;
    std::vector<const double *> avs;
    std::vector<const double *> rpy;
    std::vector<const double *> zmp;
    std::vector<double> tm;
    double q[9] = {}, q_old[9] = {}, qInit[9] = {}, tq[9] = {};

    for ( int l = 0; l < loop; l++ ) {
        double* av = new double[9];
        for ( int i = 0; i < 9; i++ ) {
            av[i] = (*rand)();
        }
        avs.push_back((const double*)av);
        tm.push_back(1);
    }

    m_seq->playPattern(avs, rpy, zmp, tm, qInit, 9);
    while (!m_seq->isEmpty()) {
        for (int i = 0; i < 9; i++ ) { q_old[i] = q[i]; }
        double zmp[3], acc[3], pos[3], rpy[3];
        m_seq->get(q, zmp, acc, pos, rpy, tq);
        for (int i = 0; i < 9; i++ ) {
            // check velocity
            double vel = fabs(q[i] - q_old[i])/m_dt;
            ASSERT_PRED_FORMAT2(testing::DoubleLE, 0, vel);
            ASSERT_PRED_FORMAT2(testing::DoubleLE, vel, 800); // 0.5 +- 200  = 800 
        }
    }
    for (int i = 0; i < 9; i++) {
        ASSERT_NEAR(avs[loop-1][i],q[i],1.0e-6);
    }
}

class test_SequencePlayerOfGroup : public test_SequencePlayer,
                                   public ::testing::WithParamInterface<vector<int> > {};

TEST_P(test_SequencePlayerOfGroup, setJointAngles) {
    // init
    double av[9] = {0,1,2,3,4,5,6,7,8};

    std::vector<int> indices = GetParam();
    std::cout << "indices";
    for(int i = 0; i < indices.size(); i++ ) {
        std::cout  << " " << indices[i];
    }
    std::cout << std::endl;

    double avg[indices.size()];

    m_seq->addJointGroup("test", indices);
    for (int i = 0;  i < indices.size(); i++) {
        avg[i] = av[indices[i]]; // set target
        av[indices[i]] /= 2; //set dummy target
    }
    m_seq->setJointAngles(av, 0.5);
    // set group
    m_seq->setJointAnglesOfGroup("test", avg, 0.5);
    // back av to original value for assertion
    for (int i = 0;  i < indices.size(); i++) {
        av[indices[i]] = avg[i];
    }

    // test
    testSetAngles(av);
}

TEST_P(test_SequencePlayerOfGroup, setJointAnglesContiniously) {
    // init
    int loop = 10;
    double av[9] = {0,1,2,3,4,5,6,7,8}, av_old[9] = {0,0,0,0,0,0,0,0,0};
    double q[9], q_old[9], tq[9];
    double zmp[3], acc[3], pos[3], rpy[3];

    std::vector<int> indices = GetParam();
    std::cout << "indices";
    for(int i = 0; i < indices.size(); i++ ) {
        std::cout  << " " << indices[i];
    }
    std::cout << std::endl;
    double avg[indices.size()];
    m_seq->addJointGroup("test", indices);

    for (int i = 0;  i < indices.size(); i++) {
        avg[i] = av[indices[i]]; // set target
        av[indices[i]] /= 2; //set dummy target
    }
    m_seq->setJointAngles(av, 1);
    m_seq->setJointAnglesOfGroup("test", avg, 1);

    while (!m_seq->isEmpty()) {
        if ( loop > 0 &&
             m_seq->interpolators[seqplay::Q]->remain_t <
             m_seq->interpolators[seqplay::Q]->target_t/2) {
            for (int i = 0; i < 9; i ++ ) {
                av_old[i] = q[i];
                av[i] = (*rand)();
            }
            m_seq->setJointAngles(av, 2);
            m_seq->setJointAnglesOfGroup("test", avg, 1);
            loop--;
        }
        for (int i = 0; i < 9; i++ ) { q_old[i] = q[i]; }
        m_seq->get(q, zmp, acc, pos, rpy, tq);
        for (int i = 0; i < 9; i++ ) {
            // check velocity
            //std::cerr << i << " " << av_old[i] << " " << q[i] << " " << av[i] << " " << (q[i] - q_old[i])/m_dt << std::endl;
            double vel = fabs(q[i] - q_old[i])/m_dt;
            ASSERT_PRED_FORMAT2(testing::DoubleLE, 0, vel);
            ASSERT_PRED_FORMAT2(testing::DoubleLE, vel, 800); // 0.5 +- 200  = 800 
        }
    }

    // back av to original value for assertion
    for (int i = 0;  i < indices.size(); i++) {
        av[indices[i]] = avg[i];
    }
    for (int i = 0; i < 9; i++) {
        ASSERT_NEAR(av[i],q[i],1.0e-6);
    }
}

TEST_P(test_SequencePlayerOfGroup, playPattern) {
    // init
    int loop = 100;
    std::vector<const double *> avs;
    std::vector<const double *> rpy;
    std::vector<const double *> zmp;
    std::vector<double> tm;
    double q[9] = {}, q_old[9] = {}, qInit[9] = {}, tq[9] = {};

    // indices
    std::vector<int> indices = GetParam();
    std::cout << "indices";
    for(int i = 0; i < indices.size(); i++ ) {
        std::cout  << " " << indices[i];
    }
    std::cout << std::endl;
    // set joint group to sequence
    std::vector<const double *> avgs;
    m_seq->addJointGroup("test", indices);

    // initialize av sequence
    for ( int l = 0; l < loop; l++ ) {
        double* av = new double[9];
        for ( int i = 0; i < 9; i++ ) {
            av[i] = (*rand)();
        }
        avs.push_back((const double*)av);

        // av goal for joint group
        double* avg = new double[indices.size()];
        for (int j = 0;  j < indices.size(); j++) {
            avg[j] = av[indices[j]]/2; // set target
        }
        avgs.push_back((const double*)avg);

        // time sequence
        tm.push_back(1);
    }

    m_seq->playPattern(avs, rpy, zmp, tm, qInit, 9);
    m_seq->playPatternOfGroup("test", avgs, tm, qInit, indices.size());
    //m_seq->playPatternOfGroup("test", avs, tm, qInit, 9);
    while (!m_seq->isEmpty()) {
        for (int i = 0; i < 9; i++ ) { q_old[i] = q[i]; }
        double zmp[3], acc[3], pos[3], rpy[3];
        m_seq->get(q, zmp, acc, pos, rpy, tq);
        for (int i = 0; i < 9; i++ ) {
            // check velocity
            double vel = fabs(q[i] - q_old[i])/m_dt;
            ASSERT_PRED_FORMAT2(testing::DoubleLE, 0, vel);
            ASSERT_PRED_FORMAT2(testing::DoubleLE, vel, 800); // 0.5 +- 200  = 800 
        }
    }
    double av[9];
    // back av to original value for assertion
    for (int i = 0; i < 9; i++) { av[i] = avs[loop-1][i]; }
    for (int i = 0; i < indices.size(); i++) { av[indices[i]] = avgs[loop-1][i]; }
    for (int i = 0; i < 9; i++) {
        ASSERT_NEAR(av[i],q[i],1.0e-6);
    }
}

std::vector<std::vector<int> > test_indices;
INSTANTIATE_TEST_CASE_P(Test,
                        test_SequencePlayerOfGroup,
                        ::testing::ValuesIn(test_indices)
                        );


int main (int argc, char **argv) {
    {
        int indices[] = {0,4};
        test_indices.push_back(std::vector<int>(indices,indices+sizeof(indices)/sizeof(int)));
    }
    {
        int indices[] = {2,4,8};
        test_indices.push_back(std::vector<int>(indices,indices+sizeof(indices)/sizeof(int)));
    }

    _argc = argc;
    _argv = argv;
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();

}
