#ifndef __PYUTIL_H__
#define __PYUTIL_H__

template<class T>
void VectorToPyList(const T& v, boost::python::list &l)
{
    for (int i=0; i<v.size(); i++){
        l.append(boost::python::object(v[i]));
    }
}

static void Matrix33ToPyList(const hrp::Matrix33& M, boost::python::list &l)
{
    for (int i=0; i<3; i++){
        for (int j=0; j<3; j++){
            l.append(boost::python::object(M(i,j)));
        }
    }
}

template<class T>
void PyListToVector(PyObject *pyo, T& v)
{
    for (int i=0; i<PySequence_Size(pyo); i++) {
        v[i] = boost::python::extract<double>(PySequence_GetItem(pyo, i));
    }
}

static void PyListToMatrix33(PyObject *pyo, hrp::Matrix33& M)
{
    for (int i=0; i<9; i++) {
        M(i/3, i%3) = boost::python::extract<double>(PySequence_GetItem(pyo, i));
    }
}

#endif
