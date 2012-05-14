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
    void realTime(bool flag);
    void endless(bool flag);
    void clear();
    void play();
    void pause();
    void notifyChanged();
    void addCollisionCheckPair(PyBody *b1, PyBody *b2);
    PyObject *bodies();
private:  
    LogManager<SceneState> log;
    GLscene scene;
    SDLwindow window;
    RTC::Manager* manager;
};

#endif
