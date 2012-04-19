class GLscene;
class ThreadedObject;

class SDLwindow
{
public:
    SDLwindow(GLscene *i_scene, ThreadedObject *i_throbj=NULL);
    bool init();
    bool processEvents();
    void draw();
    void swapBuffers();
private:
    double sliderRatio(double x);
    GLscene *scene;
    ThreadedObject *throbj;
    int width, height;
    double aspect;
    double pan, tilt, radius;
    bool isShiftPressed, isControlPressed;
    double xCenter, yCenter, zCenter;
    bool showingHelp, buttonPressedInSliderArea;
    std::vector<std::string> helpcommand, instructions;
};

