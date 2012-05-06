#ifndef __PYLINK_H__
#define __PYLINK_H__

#include <boost/python.hpp>
#include <hrpModel/Link.h>

class PyLink : public hrp::Link
{
public:
    ~PyLink();
    PyObject *getPosition();
    void setPosition(PyObject *v);
    PyObject *getRotation();
    void setRotation(PyObject *v);
    PyObject *getRelPosition();
    void setRelPosition(PyObject *v);
    PyObject *getRelRotation();
    void setRelRotation(PyObject *v);
    double getPosture();
    void setPosture(double);
    PyObject *getCoM();
    void setCoM(PyObject *v);
    PyObject *getInertia();
    void setInertia(PyObject *v);
    PyObject *getLinVel();
    void setLinVel(PyObject *v);
    PyObject *getAngVel();
    void setAngVel(PyObject *v);
    PyLink *createLink();
    PyLink *getParent();
    PyObject *getChildren();
    void setJointType(std::string type);
    std::string getJointType();
private:
    void notifyChanged();
};

#endif
