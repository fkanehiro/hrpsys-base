#include <vector>
#ifdef __APPLE__
#include "ysjoyreader.h"
#endif

class joystick
{
public:
    typedef enum {EVENT_AXIS, EVENT_BUTTON, EVENT_NONE} event;
    joystick(const char *dev);
    ~joystick();
    bool readEvent();
    bool getButtonState(int i_index) const { return m_buttons[i_index]; }
    float getAxisState(int i_index) const { return m_axes[i_index]; }
    bool is_open() const { return m_fd >= 0; }
    unsigned int nButtons() const { return m_buttons.size(); }
    unsigned int nAxes() const { return m_axes.size(); }
private:
    int m_fd;
    std::vector<float> m_axes;
    std::vector<bool> m_buttons;
#ifdef __APPLE__
#define maxNumJoystick 4
    YsJoyReader m_dev[maxNumJoystick];
#endif
};
