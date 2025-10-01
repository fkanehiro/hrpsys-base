import rtm
import OpenHRP

rtm.nshost="hrp5p01c"
rtm.nsport=2809
rtm.initCORBA()

se = rtm.findRTC("se")
if not se:
    print "can't find SoftwareEmergency RTC"

se_svc = rtm.narrow(se.service("service0"), "SoftwareEmergencyService") 

mode = input("Which mode?(0: THROUGH, 1: FREEZE, 2: RELAX) ")

if mode == 0:    
    se_svc.switchModeTo(OpenHRP.SoftwareEmergencyService.THROUGH)
elif mode == 1:
    se_svc.switchModeTo(OpenHRP.SoftwareEmergencyService.FREEZE)
elif mode == 2:
    se_svc.switchModeTo(OpenHRP.SoftwareEmergencyService.RELAX)






