#
# test script for Simulator component
# 
# start hrpsys-viewer and SimulatorComp before running this script
#
import rtm
import commands

openhrp_dir = commands.getoutput("pkg-config --variable=prefix openhrp3.1")
project = "file://"+openhrp_dir+"/share/OpenHRP-3.1/sample/project/Sample.xml"
sim = rtm.findRTC("Simulator0")
print "sim:",sim
sim.setProperty("project", project)
sim.setProperty("useOLV", "1")
sim.start()
