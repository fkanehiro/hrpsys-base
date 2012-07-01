#ifndef __PYLINK_H__
#define __PYLINK_H__

#include <boost/python.hpp>
#include "util/GLlink.h"

class PyShape;

class PyLink : public GLlink
{
public:
    PyLink();
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
    PyObject *getRotationAxis();
    void setRotationAxis(PyObject *v);
    PyObject *getTranslationAxis();
    void setTranslationAxis(PyObject *v);
    PyLink *addChildLink(std::string name);
    PyLink *getParent();
    void addShapeFromFile(std::string url);
    PyShape *addCube(double x, double y, double z); 
    PyObject *getChildren();
    void setJointType(std::string type);
    std::string getJointType();
    PyObject *shapes();
    int getJointId();
    void setJointId(int id);
private:
    void notifyChanged();
};

#endif
