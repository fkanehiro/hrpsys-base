import rtm

rtm.nshost="localhost"
rtm.nsport=2809
rtm.initCORBA()

mgr = rtm.findRTCmanager()

mgr.load("CameraImageLoader")
loader = mgr.create("CameraImageLoader")

mgr.load("ColorExtractor")
extractor = mgr.create("ColorExtractor")
extractor.setProperty("minPixels","10")
extractor.setProperty("rgbRegion","100,0,0,255,100,100")

mgr.load("CameraImageViewer")
viewer = mgr.create("CameraImageViewer")

#rtm.serializeComponents([loader, extractor, viewer])

rtm.connectPorts(loader.port("image"), extractor.port("original"))
rtm.connectPorts(extractor.port("result"), viewer.port("imageIn"))

viewer.start()
extractor.start()
loader.start()



