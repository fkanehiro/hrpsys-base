import time
import rtm
rtm.nshost = "localhost"
rtm.nsport = 2809
rtm.initCORBA()

from Img import *

def capture():
    global vc, civ, ccs
    mgr = rtm.findRTCmanager()
    mgr.load("VideoCapture")
    mgr.load("CameraImageViewer")

    vc  = mgr.create("VideoCapture")
    civ = mgr.create("CameraImageViewer")

    ccs = rtm.narrow(vc.service("service0"), "CameraCaptureService", "Img")

    rtm.connectPorts(vc.port("CameraImage"), civ.port("imageIn"))
    rtm.serializeComponents([vc, civ])
    vc.start()
    civ.start()

def rgb2gray():
    mgr = rtm.findRTCmanager()
    mgr.load("VideoCapture")
    mgr.load("RGB2Gray")
    mgr.load("CameraImageViewer")

    vc  = mgr.create("VideoCapture")
    r2g = mgr.create("RGB2Gray")
    civ = mgr.create("CameraImageViewer")

    rtm.connectPorts(vc.port("CameraImage"), r2g.port("rgb"))
    rtm.connectPorts(r2g.port("gray"), civ.port("imageIn"))
    rtm.serializeComponents([vc, r2g, civ])
    vc.start()
    r2g.start()
    civ.start()
    
def jpeg():
    global vc,je,jd,civ
    mgr = rtm.findRTCmanager()
    mgr.load("VideoCapture")
    mgr.load("JpegEncoder")
    mgr.load("JpegDecoder")
    mgr.load("CameraImageViewer")

    vc  = mgr.create("VideoCapture")
    je  = mgr.create("JpegEncoder")
    jd  = mgr.create("JpegDecoder")
    civ = mgr.create("CameraImageViewer")

    rtm.connectPorts(vc.port("CameraImage"), je.port("decoded"))
    rtm.connectPorts(je.port("encoded"), jd.port("encoded"))
    rtm.connectPorts(jd.port("decoded"), civ.port("imageIn"))
    rtm.serializeComponents([vc, je, jd, civ])
    vc.start()
    je.start()
    jd.start()
    civ.start()
    time.sleep(3)
    print "jpeg quality 95 -> 30"
    je.setProperty("quality", "30")

    
def resize():
    global vc, ri, civ
    mgr = rtm.findRTCmanager()
    mgr.load("VideoCapture")
    mgr.load("ResizeImage")
    mgr.load("CameraImageViewer")

    vc  = mgr.create("VideoCapture")
    ri  = mgr.create("ResizeImage")
    civ = mgr.create("CameraImageViewer")

    ri.setProperty("scale", "0.5")

    rtm.connectPorts(vc.port("CameraImage"), ri.port("original"))
    rtm.connectPorts(ri.port("resized"), civ.port("imageIn"))
    rtm.serializeComponents([vc, ri, civ])
    vc.start()
    ri.start()
    civ.start()

def total():
    global vc, rg2, ri, je, jd, civ
    mgr = rtm.findRTCmanager()
    mgr.load("VideoCapture")
    mgr.load("RGB2Gray")
    mgr.load("ResizeImage")
    mgr.load("JpegEncoder")
    mgr.load("JpegDecoder")
    mgr.load("CameraImageViewer")

    vc  = mgr.create("VideoCapture")
    r2g = mgr.create("RGB2Gray")
    ri  = mgr.create("ResizeImage")
    je  = mgr.create("JpegEncoder")
    jd  = mgr.create("JpegDecoder")
    civ = mgr.create("CameraImageViewer")

    ri.setProperty("scale", "0.5")

    rtm.connectPorts(vc.port("CameraImage"), r2g.port("rgb"))
    rtm.connectPorts(r2g.port("gray"), ri.port("original"))
    rtm.connectPorts(ri.port("resized"), je.port("decoded"))
    rtm.connectPorts(je.port("encoded"), jd.port("encoded"))
    rtm.connectPorts(jd.port("decoded"), civ.port("imageIn"))
    rtm.serializeComponents([vc, r2g, ri, je, jd, civ])
    vc.start()
    r2g.start()
    ri.start()
    je.start()
    jd.start()
    civ.start()

    
#total()
