#include <iostream>
#include "js.h"

joystick::joystick(const char *dev)
{
  m_fd = atoi(dev);
  int numJoystick;
  
  YsJoyReaderSetUpJoystick(numJoystick,m_dev,maxNumJoystick);
  std::cout << "numJoystick:" << numJoystick << std::endl;
  if (m_fd >= numJoystick) {
    m_fd = -1;
    return;
  }
  YsJoyReaderLoadJoystickCalibrationInfo(numJoystick,m_dev);
  
  int j, nbuttons=0, naxes=0; 
  for(j=0; j<YsJoyReaderMaxNumAxis; j++){
    if(m_dev[m_fd].axis[j].exist!=0){
      naxes++;
    }
  }
  for(j=0; j<YsJoyReaderMaxNumButton; j++){
    if(m_dev[m_fd].button[j].exist!=0){
      nbuttons++;
    }
  }
  std::cout << "axes:" << naxes << ", buttons:" << nbuttons << std::endl;
  m_buttons.resize(nbuttons);
  m_axes.resize(naxes);
}

joystick::~joystick()
{
}

bool joystick::readEvent()
{
  m_dev[m_fd].Read();
  int j;
  for(j=0; j<YsJoyReaderMaxNumAxis; j++){
    if(m_dev[m_fd].axis[j].exist!=0){
      m_axes[j] = m_dev[m_fd].axis[j].GetCalibratedValue();
    }
  }
  for(j=0; j<YsJoyReaderMaxNumButton; j++){
    if(m_dev[m_fd].button[j].exist!=0){
      m_buttons[j] = m_dev[m_fd].button[j].value;
    }
  }
  return false;
}
