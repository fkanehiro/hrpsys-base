#
# test script for Simulator and Viewer components
# 
# start SimulatorComp and ViewerComp before running this script
#
import rtm
import commands

openhrp_dir = commands.getoutput("pkg-config --variable=prefix openhrp3.1")
project = "file://"+openhrp_dir+"/share/OpenHRP-3.1/sample/project/SamplePD.xml"
sim = rtm.findRTC("Simulator0")
print "sim:",sim
sim.setProperty("project", project)
vwr = rtm.findRTC("Viewer0")
print "vwr:",vwr
vwr.setProperty("project", project)
rtm.connectPorts(sim.port("state"), vwr.port("state"))
vwr.start()
sim.start()
