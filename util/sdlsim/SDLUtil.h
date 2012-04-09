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
    GLscene *scene;
    int width, height;
    double aspect;
    double pan, tilt, radius;
    int button;
    bool isShiftPressed, isControlPressed;
    double xCenter, yCenter, zCenter;
};

