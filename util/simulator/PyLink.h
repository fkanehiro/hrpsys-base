#ifndef __PYLINK_H__
#define __PYLINK_H__

#include <boost/python.hpp>
#include <hrpModel/Link.h>

class PyLink : public hrp::Link
{
public:
    void setPosition(PyObject *v);
    void setOrientation(PyObject *v);
    void setPosture(double);
    PyObject *getPosition();
    PyObject *getOrientation();
    double getPosture();
private:
};

#endif
