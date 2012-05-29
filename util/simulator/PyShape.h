#ifndef __PYSHAPE_H__
#define __PYSHAPE_H__

#include <boost/python.hpp>
#include "util/GLshape.h"

class PyShape : public GLshape
{
public:
    PyObject *getRelPosition();
    void setRelPosition(PyObject *v);
    PyObject *getRelRotation();
    void setRelRotation(PyObject *v);
    PyObject *getDiffuseColor();
    void setDiffuseColor(PyObject *v);
private:
};

GLshape *createPyShape();

#endif
