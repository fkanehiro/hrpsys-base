#include "ThreadedObject.h"

class GLsceneBase;
class ThreadedObject;
class LogManagerBase;

class SDLwindow : public ThreadedObject
{
public:
    SDLwindow(GLsceneBase *i_scene, LogManagerBase *i_lm, 
              ThreadedObject *i_throbj=NULL);
    ~SDLwindow();
    bool init(int w=0, int h=0, bool resizable=true);
    bool processEvents();
    void draw();
    void swapBuffers();
    bool oneStep();
    void setView(double T[16]);
private:
    double sliderRatio(double x);
    GLsceneBase *scene;
    LogManagerBase *log;
    ThreadedObject *throbj;
    int width, height;
    double pan, tilt, radius;
    bool isShiftPressed, isControlPressed;
    double xCenter, yCenter, zCenter;
    bool showingHelp, buttonPressedInSliderArea;
    std::vector<std::string> helpcommand, instructions;
    bool initialized; 
};

