#include <iostream>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>
#include <unistd.h>
#include <errno.h>
#include <cstdio>
#include "js.h"

joystick::joystick(const char *dev) : m_fd(-1)
{
    if ((m_fd = open(dev, O_RDONLY|O_NONBLOCK)) < 0){
        perror("open");
        return;
    }

    char number_of_axes;
    ioctl(m_fd, JSIOCGAXES, &number_of_axes );
    std::cout << "number_of_axes = " << (int)number_of_axes << std::endl;
    m_axes.resize(number_of_axes);

    char number_of_buttons;
    ioctl(m_fd, JSIOCGBUTTONS, &number_of_buttons );
    std::cout << "number_of_buttons = " << (int)number_of_buttons << std::endl;
    m_buttons.resize(number_of_buttons);

    // read initial state
    for (int i=0; i<number_of_axes+number_of_buttons; i++){
        readEvent();
    }
}

joystick::~joystick()
{
    if (m_fd >= 0){
        close(m_fd);
    }
}

bool joystick::readEvent()
{
    js_event e;
    float	fval;
    const float MAX_VALUE_16BIT = 32767.0f;
    
    // read data of joystick 
    int rdlen = read(m_fd, &e, sizeof(js_event) );
    
    // error
    if( rdlen <= 0 ) {
        if( errno == EAGAIN ){
            
        }		
        //std::cout<< "event queue is empty"<<std::endl;
        return false;
    } else if( rdlen < (int)sizeof(js_event) ) {
        std::cout<<"ERROR: read"<<std::endl;
        return false;
    }else{ 
        if( e.type & JS_EVENT_AXIS ) {
            // axis
            // normalize value (-1.0〜1.0)
            fval = (float)e.value/MAX_VALUE_16BIT;
            //printf("%d:%x\n", e.number, e.value);
            m_axes[e.number] = fval;
        }else{
            // button 
            m_buttons[e.number] = e.value==0 ? false : true;
        }
    }
    return true;
}
