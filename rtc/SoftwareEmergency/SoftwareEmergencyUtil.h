#include <iostream>
#include "hrpsys/idl/SoftwareEmergencyService.hh"

std::ostream& operator<<(std::ostream& o, ::OpenHRP::SoftwareEmergencyService::mode m)
{
  switch(m){
  case ::OpenHRP::SoftwareEmergencyService::THROUGH:
    o << "THROUGH"; break; 
  case ::OpenHRP::SoftwareEmergencyService::FREEZE:
    o << "FREEZE"; break; 
  case ::OpenHRP::SoftwareEmergencyService::RELAX:
    o << "RELAX"; break; 
  }
  return o;
}

