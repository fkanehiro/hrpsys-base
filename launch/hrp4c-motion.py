#!/usr/bin/env python

"""
 this is example file for HRP4-C robot

 $ roslaunch hrpsys hrp4c.launch
 $ rosrun    hrpsys hrp4c-motion.py

If you does not have hrp4c robot model, download the data first
 $ rosrun    hrpsys hrp4c_model_download.sh

"""
import imp, sys, os

sys.path.append(imp.find_module('hrpsys')[1]) # set path to hrpsys
sys.path.append(os.path.dirname(os.path.abspath(__file__))+'/../share/hrpsys/samples/HRP4C/') # set path to PA10

#os.environ['ORBInitRef'] = 'NameService=corbaloc:iiop:{0}:{1}/NameService'.format('localhost','2809')

import HRP4C
HRP4C.rtm.initCORBA()
HRP4C.init()
HRP4C.loadPattern(os.path.dirname(os.path.abspath(__file__))+'/../share/hrpsys/samples/HRP4C/data/hear')
HRP4C.loadPattern(os.path.dirname(os.path.abspath(__file__))+'/../share/hrpsys/samples/HRP4C/data/bow')
HRP4C.loadPattern(os.path.dirname(os.path.abspath(__file__))+'/../share/hrpsys/samples/HRP4C/data/roate_waist')
HRP4C.loadPattern(os.path.dirname(os.path.abspath(__file__))+'/../share/hrpsys/samples/HRP4C/data/warmup_r')
HRP4C.loadPattern(os.path.dirname(os.path.abspath(__file__))+'/../share/hrpsys/samples/HRP4C/data/warmup_y')
HRP4C.loadPattern(os.path.dirname(os.path.abspath(__file__))+'/../share/hrpsys/samples/HRP4C/data/walk2m')







