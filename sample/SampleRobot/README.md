========================================================================================================================
Examples
========================================================================================================================

This page shows basic exsample for hrpsys controller using [SampleRobot](https://github.com/fkanehiro/hrpsys-base/blob/master/sample/SampleRobot).  


To learn more about API, please refer to [API DOC in hrpsys-base](http://fkanehiro.github.io/hrpsys-base/).

# samplerobot-walk.py
1. Launch hrpsys-simulator

 ```
rtmlaunch hrpsys samplerobot.launch
 ```
2. python example

 ```
rosrun hrpsys samplerobot-walk.py
 ```
  This example imports [hrpsys-base samplerobot_walk.py](https://github.com/fkanehiro/hrpsys-base/blob/master/sample/SampleRobot/samplerobot_walk.py) for mor info.

# samplerobot-data-logger.py
1. Launch hrpsys-simulator

 ```
rtmlaunch hrpsys samplerobot.launch
 ```
2. python example

 ```
rosrun hrpsys samplerobot_data_logger.py
 ```
 See 
 [hrpsys-base samplerobot_data_logger.py](https://github.com/fkanehiro/hrpsys-base/blob/master/sample/SampleRobot/samplerobot_data_logger.py) for more info.  
3. RTC explanation 
 - DataLogger  
 DataLogger is hrpsys-base RTC for data logging of hrpsys-base RTCs. 
 Please see 
 [Overview](http://fkanehiro.github.io/hrpsys-base/d4/d46/DataLogger.html) 
 and [IDL API](http://fkanehiro.github.io/hrpsys-base/d7/dbb/interfaceOpenHRP_1_1DataLoggerService.html).  
 - Save log  
 By using ``save()`` function, DataLogger saves data ports values in a ring buffer into several files. If data ports are connected by ``connectLoggerPort`` in [hrpsys-base hrpsys_config.py](https://github.com/fkanehiro/hrpsys-base/blob/master/python/hrpsys_config.py), file extensions are determined by RTC names and data port names.  
 In the above example, files are saved as ``/tmp/test-samplerobot-log.**``. 
 The log file for StateHolder RTC's ``qOut`` data port is written as ``/tmp/test-samplerobot-log.sh_qOut``. 
 ``sh`` is the comonent name for StateHolder, which is specified in
 [hrpsys-base hrpsys_config.py's getRTCList function](https://github.com/fkanehiro/hrpsys-base/blob/master/python/hrpsys_config.py). 


# samplerobot_remove_force_offset.py
1. Launch hrpsys-simulator

 ```
rtmlaunch hrpsys samplerobot.launch
 ```
2. python example

 ```
rosrun hrpsys samplerobot_remove_force_offset.py
 ```
 See
 [hrpsys-base samplerobot_remove_force_offset.py](https://github.com/fkanehiro/hrpsys-base/blob/master/sample/SampleRobot/samplerobot_remove_force_offset.py) for more info.  
3. RTC explanation  
 - RemoveForceSensorLinkOffset  
 RemoveForceSensorLinkOffset is hrpsys-base RTC to remove hands or feet from force sensor values. 
 Please see 
 [Overview](http://fkanehiro.github.io/hrpsys-base/dc/d76/RemoveForceSensorLinkOffset.html) 
 and [IDL API](http://fkanehiro.github.io/hrpsys-base/d6/d02/interfaceOpenHRP_1_1RemoveForceSensorLinkOffsetService.html).  
 - Offsetting  
 In the above example, the initial wrench values are larger than zero:

    ```
    # 1. force and moment are large because of link offsets
    fm=numpy.linalg.norm(rtm.readDataPort(hcf.rmfo.port("off_rhsensor")).data)
    print "no-offset-removed force moment (rhsensor) ", fm, "=> ", fm > 1e-2
    fm=numpy.linalg.norm(rtm.readDataPort(hcf.rmfo.port("off_lhsensor")).data)
    print "no-offset-removed force moment (lhsensor) ", fm, "=> ", fm > 1e-2
    ```
 This is because these values include hand link weight force and weight moment.  
 After offsetting by ``setForceMomentOffsetParam()`` function, 
 wrench values become almost zero:

    ```
    # 3. force and moment are reduced
    fm=numpy.linalg.norm(rtm.readDataPort(hcf.rmfo.port("off_rhsensor")).data)
    print "no-offset-removed force moment (rhsensor) ", fm, "=> ", fm < 1e-2
    fm=numpy.linalg.norm(rtm.readDataPort(hcf.rmfo.port("off_lhsensor")).data)
    print "no-offset-removed force moment (lhsensor) ", fm, "=> ", fm < 1e-2
    ```

# samplerobot_impedance_controller.py
1. Launch hrpsys-simulator

 ```
rtmlaunch hrpsys samplerobot.launch
 ```
2. python example

 ```
rosrun hrpsys samplerobot_impedance_controller.py
 ```
 See 
 [hrpsys-base samplerobot_impedance_controller.py](https://github.com/fkanehiro/hrpsys-base/blob/master/sample/SampleRobot/samplerobot_impedance_controller.py) for more info.  
3. RTC explanation  
 - ImpedanceController  
 ImpedanceController is hrpsys-base RTC for cartesian impedance control. 
 Please see 
 [Overview](http://fkanehiro.github.io/hrpsys-base/d2/d9f/ImpedanceController.html) 
 and [IDL API](http://fkanehiro.github.io/hrpsys-base/d9/d8b/interfaceOpenHRP_1_1ImpedanceControllerService.html).  
 
# samplerobot_auto_balancer.py
1. Launch hrpsys-simulator

 ```
rtmlaunch hrpsys samplerobot.launch
 ```
2. python example

 ```
rosrun hrpsys samplerobot_auto_balancer.py
 ```
 <div align="center"><p><img src="http://wiki.ros.org/rtmros_common/Tutorials/WorkingWithEusLisp?action=AttachFile&do=get&target=abc.png" alt="AutoBalancer" title="AutoBalancer" width=300/></p></div>  
 See
 [hrpsys-base samplerobot_auto_balancer.py](https://github.com/fkanehiro/hrpsys-base/blob/master/sample/SampleRobot/samplerobot_auto_balancer.py) for more info.  
3. RTC explanation  
 - AutoBalancer  
 AutoBalancer is hrpsys-base RTC to generate walking pattern and control Center Of Gravity for legged robots. 
 Please see 
 [Overview](http://fkanehiro.github.io/hrpsys-base/d1/d15/AutoBalancer.html) 
 and [IDL API](http://fkanehiro.github.io/hrpsys-base/d4/d5b/interfaceOpenHRP_1_1AutoBalancerService.html).  


# samplerobot_stabilizer.py
1. Launch hrpsys-simulator

 ```
rtmlaunch hrpsys samplerobot.launch
 ```
2. python example

 ```
rosrun hrpsys samplerobot_stabilizer.py
 ```
 See
 [hrpsys-base samplerobot_stabilizer.py](https://github.com/fkanehiro/hrpsys-base/blob/master/sample/SampleRobot/samplerobot_stabilizer.py) for more info.  
3. RTC explanation  
 - Stabilizer  
 Stabilizer is hrpsys-base RTC to maintain full-body balance based on sensor feedback. 
 Please see 
 [Overview](http://fkanehiro.github.io/hrpsys-base/d6/d76/Stabilizer.html) 
 and [IDL API](http://fkanehiro.github.io/hrpsys-base/d5/dc8/interfaceOpenHRP_1_1StabilizerService.html).  

# samplerobot_terrain_walk.py
0. These examples are related with AutoBalancer RTC.  
1. Example for slope walking  
 <div align="center"><p><img src="http://wiki.ros.org/rtmros_common/Tutorials/WorkingWithEusLisp?action=AttachFile&do=get&target=slope.png" alt="Slope" title="Slope" width=300/></p></div>  
 1-1. Launch hrpsys-simulator
 ```
rtmlaunch hrpsys samplerobot-terrain-walk.launch MODEL:=SlopeUpDown
 ```
 1-2. python example
 ```
rosrun hrpsys samplerobot_terrain_walk.py --SlopeUpDown
 ```
 This Example imports
 [hrpsys-base samplerobot_terrain_walk.py](https://github.com/fkanehiro/hrpsys-base/blob/master/sample/SampleRobot/samplerobot_terrain_walk.py).

2. Example for stair climbing-up
 <div align="center"><p><img src="http://wiki.ros.org/rtmros_common/Tutorials/WorkingWithEusLisp?action=AttachFile&do=get&target=stairup.png" alt="StairUp" title="StairUp" width=300/></p></div>  
 2-1. Launch hrpsys-simulator
 ```
rtmlaunch hrpsys samplerobot-terrain-walk.launch MODEL:=StairUp
 ```
 2-2. python example
 ```
rosrun hrpsys samplerobot-terrain-walk.py --StairUp
 ```
 This example imports
 [hrpsys-base samplerobot_terrain_walk.py](https://github.com/fkanehiro/hrpsys-base/blob/master/sample/SampleRobot/samplerobot_terrain_walk.py).

3. Example for stair climbing-down  
 <div align="center"><p><img src="http://wiki.ros.org/rtmros_common/Tutorials/WorkingWithEusLisp?action=AttachFile&do=get&target=stairdown.png" alt="StairDown" title="StairDown" width=300/></p></div>  
 3-1. Launch hrpsys-simulator
 ```
rtmlaunch hrpsys samplerobot-terrain-walk.launch MODEL:=StairDown
 ```
 2-2. python example
 ```
rosrun hrpsys samplerobot-terrain-walk.py --StairDown
 ```
 This example imports
 [hrpsys-base samplerobot_terrain_walk.py](https://github.com/fkanehiro/hrpsys-base/blob/master/sample/SampleRobot/samplerobot_terrain_walk.py).

# samplerobot-drc-testbed.launch
1. Launch hrpsys-simulator

 ```
rtmlaunch hrpsys samplerobot-drc-testbed.launch
 ```
2. python example

 ```
ipython -i `rospack find hrpsys`/samples/SampleRobot/samplerobot_terrain_walk.py 
 ```

3. Programming
 <div align="center"><p><img src="http://wiki.ros.org/rtmros_common/Tutorials/WorkingWithEusLisp?action=AttachFile&do=get&target=samplerobot-drc-testbed.png" alt="DRC Testbed" title="DRC Testbed" height=300/></p></div>  
 - Python Interface
 From python iterface, we can use `hcf` as interface object to the robot, for example
`hcf.setJointAngle('LARM_SHOULDER_P', -90, 1)`
 moves shoulder pitch joint of left arm to 90 degree, with 1 seconds.
 Please see [Pyton API](http://fkanehiro.github.io/hrpsys-base/df/d98/classpython_1_1hrpsys__config_1_1HrpsysConfigurator.html) for the avilable method functions, and [SampleRobot Model](http://www.openrtp.jp/openhrp3/en/sample_model.html) for the list of joint names.

