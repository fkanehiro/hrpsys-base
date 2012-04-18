class GLscene;

class SDLwindow
{
public:
    SDLwindow(GLscene *i_scene);
    bool init();
    bool processEvents();
    void draw();
    void swapBuffers();
private:
    double sliderRatio(double x);
    GLscene *scene;
    int width, height;
    double aspect;
    double pan, tilt, radius;
    bool isShiftPressed, isControlPressed;
    double xCenter, yCenter, zCenter;
    bool showingHelp;
    std::vector<std::string> helpcommand, instructions;
};

