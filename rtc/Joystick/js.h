#include <vector>

class joystick
{
public:
    typedef enum {EVENT_AXIS, EVENT_BUTTON, EVENT_NONE} event;
    joystick(const char *dev);
    ~joystick();
    bool readEvent();
    void getLastEvent(event &o_event, int &o_index) const;
    bool getButtonState(int i_index) const;
    float getAxisState(int i_index) const;
    bool is_open() const;
    unsigned int nButtons() const;
    unsigned int nAxes() const;
private:
    int m_fd, m_lastEventIndex;
    event m_lastEvent;
    std::vector<float> m_axes;
    std::vector<bool> m_buttons;
};
