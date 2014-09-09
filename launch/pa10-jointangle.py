#!/usr/bin/env python

"""
 this is example file for PA10 robot

 $ roslaunch hrpsys pa10.launch
 $ rosrun    hrpsys pa10-jointangle.py
"""

import imp, sys, os


# set path to hrpsys to get import rpy, see <hrpsys>/test/test-jointangle.py for using HrpsysConfigurator
try:
    imp.find_module('hrpsys') # catkin installed
    sys.path.append(imp.find_module('hrpsys')[1])
except: # rosbuild installed
    import rospkg
    rp = rospkg.RosPack()
    sys.path.append(rp.get_path('hrpsys')+'/lib/python2.7/dist-packages/hrpsys')
    sys.path.append(rp.get_path('openrtm_aist_python')+'/lib/python2.7/dist-packages')

sys.path.append(os.path.dirname(os.path.abspath(__file__))+'/../share/hrpsys/samples/PA10/') # set path to PA10

#os.environ['ORBInitRef'] = 'NameService=corbaloc:iiop:{0}:{1}/NameService'.format('localhost','2809')

import PA10
PA10.rtm.initCORBA()
PA10.demo()

## IGNORE ME: this code used for rostest
if [s for s in sys.argv if "--gtest_output=xml:" in s] :
    import unittest, rostest
    rostest.run('hrpsys', 'pa10_sample', unittest.TestCase, sys.argv)



