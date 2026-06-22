#include <rtm/RTObject.h>
#include <signal.h>
#include <unistd.h>
#include "hrpsys/idl/SoftwareEmergencyService.hh"
#include "SoftwareEmergencyUtil.h"
extern "C"{
#include "usbio.h"
}

#define USBIO_EMERGENCY_ID 7
#define USBIO_BUTTON_ID 6

RTC::RTObject_var findRTC(const std::string &rtcName,
                          CosNaming::NamingContext_var namingContext)
{
    CosNaming::Name name;
    name.length(1);
    name[0].id = CORBA::string_dup(rtcName.c_str());
    name[0].kind = CORBA::string_dup("rtc");
    try{
        CORBA::Object_ptr obj = namingContext->resolve(name);
        return RTC::RTObject::_narrow(obj);
    }catch(...){
        return NULL;
    }
}

bool getServiceIOR(RTC::RTObject_var rtc, 
                   const char *sname, std::string &ret)
{
    const char *ior = NULL;

    RTC::PortServiceList ports;
    ports = *(rtc->get_ports());

    RTC::ComponentProfile* cprof;
    cprof = rtc->get_component_profile();
    std::string portname = std::string(cprof->instance_name) + "." + sname;

    for(unsigned int i=0; i < ports.length(); i++)
        {
            RTC::PortService_var port = ports[i];
            RTC::PortProfile* prof = port->get_port_profile();
            if(std::string(prof->name) == portname)
                {
                    RTC::ConnectorProfile connProfile;
                    connProfile.name = "noname";
                    connProfile.connector_id = "";
                    connProfile.ports.length(1);
                    connProfile.ports[0] = port;
                    port->connect(connProfile);

                    connProfile.properties[0].value >>= ior;
                    ret = std::string(ior);
                    
                    port->disconnect(connProfile.connector_id);
                    
                    return true;
                }
        }

    return false;
}

OpenHRP::SoftwareEmergencyService_var getSoftwareEmergencyService(CORBA::ORB_var orb, CosNaming::NamingContext_var namingContext)
{
    RTC::RTObject_var rtc = findRTC("se", namingContext);
    if (!rtc){
        std::cerr << "can't find SoftwareEmergency RTC(se)" << std::endl;
        return NULL;
    }
    std::string ior_str;
    if (!getServiceIOR(rtc, "SoftwareEmergencyService", ior_str)){
        std::cerr << "can't find SoftwareEmergencyService" << std::endl;
        return NULL;
    }
    return OpenHRP::SoftwareEmergencyService::_narrow(orb->string_to_object(ior_str.c_str()));
}

static bool loop = true;

void handler(int signum){
    loop = false;
}

int main(int argc, char *argv[])
{
    int dev_num = 0;
    usbio_t hd[MAX_USBIO2_NUM];
    bool ret = usbio_init(hd, &dev_num, false);
    if (!ret || (dev_num == 0)){
        std::cerr << "failed to open Emergency Button" << std::endl;
        return 1;
    }
    std::cout << "number of button = " << dev_num << std::endl;
    
    std::vector<int> usb_id(dev_num, -1);
    std::vector<int> is_on[2];
    is_on[0].resize(dev_num);
    is_on[1].resize(dev_num);
    std::vector<int> led_on(dev_num, 0);
    std::vector<int> led_off(dev_num, 1);
    uint8_t recv;
    for (int i = 0 ; i < dev_num ; i++ ){
      usbio_read(hd[i], &recv);
      int id = (recv & 0x0e) >> 1;
      if( id == USBIO_EMERGENCY_ID ){
        usb_id[0] = i;
        is_on[0][0] = 0;
        is_on[1][0] = 1;
        led_on[0] = 1;
        led_off[0] = 0;
        std::cout << "detect emergency button id=" << id << std::endl;
      }
      else if( id == USBIO_BUTTON_ID ){
        usb_id[1] = i;
        is_on[0][1] = 1;
        is_on[1][1] = 0;
        led_on[1] = 0;
        led_off[1] = 1;
        std::cout << "detect user customized button id=" << id << std::endl;
      }
    }
    
    signal(SIGINT, handler);
  
    CORBA::ORB_var orb;
    CosNaming::NamingContext_var namingContext;
 
    try {
        orb = CORBA::ORB_init(argc, argv);

        CORBA::Object_var obj;
        obj = orb->resolve_initial_references("RootPOA");
        PortableServer::POA_var poa = PortableServer::POA::_narrow(obj);
        if(CORBA::is_nil(poa)){
            throw std::string("error: failed to narrow root POA.");
        }
        
        PortableServer::POAManager_var poaManager = poa->the_POAManager();
        if(CORBA::is_nil(poaManager)){
            throw std::string("error: failed to narrow root POA manager.");
        }
        
        obj = orb->resolve_initial_references("NameService");
        namingContext = CosNaming::NamingContext::_narrow(obj);
        if(CORBA::is_nil(namingContext)){
            throw std::string("error: failed to narrow naming context.");
        }
        
        poaManager->activate();
    }catch (CORBA::SystemException& ex) {
        std::cerr << ex._rep_id() << std::endl;
    }catch (const std::string& error){
        std::cerr << error << std::endl;
    }

    OpenHRP::SoftwareEmergencyService_var svc;
    
    std::vector<bool> button_prev(dev_num, false);
    int c=0;
    while(loop){
        try{
            uint8_t recv;
            std::vector<bool> button(dev_num, false);
            for (int i = 0 ; i < dev_num ; i++ ){
              usbio_read(hd[usb_id[i]], &recv);
              button[i] = is_on[recv & 0x01][i];
            }
            if (c%10==0){
                if (CORBA::is_nil(svc)){
                    svc = getSoftwareEmergencyService(orb, namingContext);

                    if (!CORBA::is_nil(svc) && svc->_non_existent()){
                        std::cout << "svc doesn't exist" << std::endl;
                        svc = NULL;
                    }
                }
                if (!CORBA::is_nil(svc)){
                    ::OpenHRP::SoftwareEmergencyService::mode m = svc->getMode();
                    std::cout << "mode = " << m << std::endl;
                    for (int i = 0 ; i < dev_num ; i++ ){
                      if (button[i]) {
                        usbio_write(hd[usb_id[i]], led_on[i], 0x01); // button is pushed
                      }else{
                        usbio_write(hd[usb_id[i]], (c%20==0) ? led_on[i] : led_off[i], 0x01); // button is not pushed, function is ready
                      }
                    }
                }else{
                    for (int i = 0 ; i < dev_num ; i++ )
                      usbio_write(hd[usb_id[i]], led_off[i], 0x01); // function is not ready
                }
            }

            if (!CORBA::is_nil(svc)){
                if (!button_prev[0] && button[0]){
                    svc->switchModeTo(::OpenHRP::SoftwareEmergencyService::RELAX);
                }else if(button_prev[0] && !button[0]){
                    svc->switchModeTo(::OpenHRP::SoftwareEmergencyService::THROUGH);
                }
              
                if(dev_num > 1){
                    if (!button_prev[1] && button[1]){
                        svc->setButtonStatus(0, true);
                    }else if(button_prev[1] && !button[1]){
                        svc->setButtonStatus(0, false);
                    }
                }
            }
            button_prev = button;

        }catch(...){
            svc = NULL;
            std::cout << "no connection to SoftwareEmergencyService" << std::endl;
        }
        usleep(100*1000);
        c++;
    }
    
    for (int i = 0 ; i < dev_num ; i++ )
      usbio_cleanup(hd[usb_id[i]], true);
    
    return 0;
}
