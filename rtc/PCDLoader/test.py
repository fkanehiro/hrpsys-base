import rtm

rtm.nsport=2809
rtm.initCORBA()

mgr = rtm.findRTCmanager()

mgr.load("PCDLoader")
mgr.load("PointCloudViewer")

pcl = mgr.create("PCDLoader")
pcv = mgr.create("PointCloudViewer")

rtm.connectPorts(pcl.port("cloud"), pcv.port("cloud"))

pcl.setProperty("fields", "XYZRGB")
pcl.start()
pcv.start()

