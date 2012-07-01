#
# test script for VirtualCamera component
# 
# start VirtualCameraComp before running this script
#
import rtm
import commands

openhrp_dir = commands.getoutput("pkg-config --variable=prefix openhrp3.1")
project = "file://"+openhrp_dir+"/share/OpenHRP-3.1/sample/project/SampleRobot_inHouse.xml"
vc = rtm.findRTC("VirtualCamera0")
print "vc:",vc
vc.setProperty("project", project)
vc.setProperty("camera", "Robot:VISION_SENSOR1")
vc.start()
