import rtm
import commands

mgr = rtm.findRTCmanager()
mgr.load("VirtualCamera")
mgr.load("OccupancyGridMap3D")
mgr.load("OGMap3DViewer")

vc     = mgr.create("VirtualCamera")
ogmap  = mgr.create("OccupancyGridMap3D")
viewer = mgr.create("OGMap3DViewer")

openhrp_dir = commands.getoutput("pkg-config --variable=prefix openhrp3.1")
project = "file://"+openhrp_dir+"/share/OpenHRP-3.1/sample/project/SampleRobot_inHouse.xml"
vc.setProperty("project", project)
vc.setProperty("camera", "Robot:VISION_SENSOR1")
vc.setProperty("generateRange", "0")
vc.setProperty("generatePointCloud", "1")
vc.setProperty("generatePointCloudStep", "5")

ogmap.setProperty("resolution", "0.05")

rtm.connectPorts(vc.port("cloud"), ogmap.port("cloud"))
rtm.connectPorts(vc.port("poseSensor"), ogmap.port("pose"))
rtm.connectPorts(ogmap.port("OGMap3DService"), viewer.port("OGMap3DService"))

rtm.serializeComponents([vc,ogmap])
vc.start()
ogmap.start()
viewer.start()


