#ifndef __PYSIMULATOR_H__
#define __PYSIMULATOR_H__

#include "util/SDLUtil.h"
#include "Simulator.h"
#include "GLscene.h"

class PyBody;

class PySimulator
{
public:
    PySimulator();
    ~PySimulator();
    PyBody* loadBody(std::string name, std::string url);
    PyBody* createBody(std::string name);
    void loadProject(std::string fname);
    void simulate();
    void simulate(double time);
    void start(double time);
    void stop();
    void wait();
    void realTime(bool flag);
    void endless(bool flag);
    void setTimeStep(double t);
    double getTimeStep();
    void clear();
    void play();
    void pause();
    void notifyChanged();
    PyObject *bodies();
    void addCollisionCheckPair(PyBody *b1, PyBody *b2);
    bool oneStep();
    double time();
    void initialize();
private:  
    LogManager<SceneState> log;
    GLscene scene;
    Simulator simulator;
    SDLwindow window;
    RTC::Manager* manager;
};

#endif
