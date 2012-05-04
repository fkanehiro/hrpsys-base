#include <boost/python.hpp>
#include "util/BodyRTC.h"

class PyLink;

class PyBody : public BodyRTC
{
public:
    PyBody(RTC::Manager* manager = &RTC::Manager::instance());
    PyBody(){};
    virtual ~PyBody();
    //std::vector<double> getPosition();
    PyObject *getPosition();
    void setPosition(PyObject *v);
    PyObject *getOrientation();
    void setOrientation(PyObject *v);
    PyObject *getPosture();
    void setPosture(PyObject *v);
    std::string getName();
    void setName(std::string name);
    void calcForwardKinematics();
    PyLink *rootLink();
    PyLink *link(std::string name);
    PyObject *links();
    PyLink *joint(int i);
    PyObject *joints();
    static void moduleInit(RTC::Manager*);
private:
    static const char* pybody_spec[];
};

