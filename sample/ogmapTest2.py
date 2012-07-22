import rtm
import commands

mgr = rtm.findRTCmanager()
mgr.load("VirtualCamera")
mgr.load("Range2PointCloud")
mgr.load("OccupancyGridMap3D")
mgr.load("OGMap3DViewer")

vc     = mgr.create("VirtualCamera")
r2pc   = mgr.create("Range2PointCloud")
ogmap  = mgr.create("OccupancyGridMap3D")
viewer = mgr.create("OGMap3DViewer")

openhrp_dir = commands.getoutput("pkg-config --variable=prefix openhrp3.1")
project = "file://"+openhrp_dir+"/share/OpenHRP-3.1/sample/project/SampleRobot_inHouse.xml"
vc.setProperty("project", project)
vc.setProperty("camera", "Robot:VISION_SENSOR1")
vc.setProperty("generateRange", "1")
vc.setProperty("rangerMaxAngle", "0.4")
vc.setProperty("rangerMinAngle", "-0.4")

ogmap.setProperty("resolution", "0.05")

viewer.setProperty("xSize", "5")
viewer.setProperty("ySize", "5")
viewer.setProperty("zSize", "2.5")
viewer.setProperty("xOrigin", "0")
viewer.setProperty("yOrigin", "-2.5")
viewer.setProperty("zOrigin", "-0.5")

rtm.connectPorts(vc.port("range"), r2pc.port("range"))
rtm.connectPorts(r2pc.port("cloud"), ogmap.port("cloud"))
rtm.connectPorts(vc.port("poseSensor"), ogmap.port("pose"))
rtm.connectPorts(ogmap.port("OGMap3DService"), viewer.port("OGMap3DService"))

vc.start()
r2pc.start()
ogmap.start()
viewer.start()


