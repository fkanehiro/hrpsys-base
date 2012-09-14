#include <hrpModel/Body.h>
#include <hrpModel/Link.h>
#include <boost/intrusive_ptr.hpp>

// robot model copy from RobotHardware
class robot : public hrp::Body
{
public:
    /**
       \brief constructor
     */
    robot();

    /**
       \brief destructor
    */
    ~robot();

    /**
       \brief
     */
    bool init();

    /**
       \brief set servo error limit value for specific joint or joint group
       \param i_jname joint name or joint group name
       \param i_limit new limit value[rad]
       \return true if set successfully, false otherwise 
     */
    bool setServoErrorLimit(const char *i_jname, double i_limit);

    //boost::interprocess::interprocess_semaphore wait_sem;

    std::vector<double> m_servoErrorLimit;
};
