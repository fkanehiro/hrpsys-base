#ifndef __PYSIMULATOR_H__
#define __PYSIMULATOR_H__

#include "util/SDLUtil.h"
#include "Simulator.h"
#include "GLscene.h"

class PyBody;

class PySimulator : public Simulator
{
public:
    PySimulator();
    PySimulator(PyObject *pyo);
    ~PySimulator();
    void initRTCmanager();
    void initRTCmanager(PyObject *pyo);
    void initRTCmanager(int argc, char **argv);
    void initViewer();
    PyBody* loadBody(std::string name, std::string url);
    PyBody* createBody(std::string name);
    void loadProject(std::string fname);
    void simulate();
    void simulate(double time);
    void start(double time);
    void endless(bool flag);
    void clear();
    void play();
    void pause();
    void notifyChanged();
    void addCollisionCheckPair(PyBody *b1, PyBody *b2);
    void capture(std::string);
    unsigned int logLength();
    PyObject *bodies();
    PyBody *getBody(std::string name);
    bool showSensors();
    void setShowSensors(bool flag);
    void reset();
    void setUseBBox(bool flag);
private:  
    LogManager<SceneState> log;
    GLscene scene;
    SDLwindow window;
    RTC::Manager* manager;
    bool useBBox;
};

#endif
