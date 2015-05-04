^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hrpsys
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

Stable RTCs
===========

* RobotHardware (lib/io/iob.cpp) API Updated

  * [rtm/RobotHardware/robot.cpp] fix for source program with ROBOT_IOB_VERSION < 2
  * [RobotHardwareService.idl, lib/io/iob.cpp] adds a field, temperature to RobotState2
  * [RobotHardwareService.idl] changes interface to get battery status
  * [lib/io/iob{cpp,h},rtc/RobotHardware/{robot.cpp,robot.h,RobotHardwareService_impl.cpp,CMakeList.txt] add ROBOT_IOB_VERSION for backword compatibility
  * [RobotHardware, iob, BodyRTC] fix to compatible with old API
  * [lib/io/iob.cpp] adds the third argument to read_power of iob.h
  * [idl/RobotHardwareService.idl] adds a field, battery to RobotState
  * [RobotHrdware.cpp, robot.cpp] adds checkJointCommands() to check joint commands before setting

* KalmanFilter

  * [KalmanFilter.cpp] output the body frame relative to a world reference frame as baseRpyCurrent
  * [KalmanFilter.cpp] use yaw of sensor->link->R for update of yaw instead of 0.0
  * [python/hrpsys_config.py, KalmanFilter.cpp] connect port from rh.q to kf.qCurrent
  * [KalmanFilter.h] use openhrp3/hrplib/hrpUtil for rotation conversion

* hrpsys_config.py

  * [hrpsys_config.py] add startImpedance and stopImpedance
  * [hrpsys_config.py] add MaxLength option for setupLogger
  * [hrpsys_config.py] `#567 <https://github.com/fkanehiro/hrpsys-base/issues/567>`_ is not correct PROJECT_DIR is
  (OpenHRP3  installed directory)/share/OpenHRP-3.1/sample/project
  * [hrpsys_config.py] `#567 <https://github.com/fkanehiro/hrpsys-base/issues/567>`_ is wrong, do not need to decode
  * [hrpsys_config.py] fix code to work on python2.5
  * [hrpsys_config.py] Import check_output near the line which check_output is used
  * [hrpsys_config.py] add verbose option to findCOmps()
 * [hrpsys_config.py] add max_timeout_count to findComps()
  * [hrpsys_config.py] fix print message
  * [hrpsys_config.py] add verbose option to getRTCInstanceList()

* rtm.py

  * fix typo time->tm in stop()
  * adds timeout to start() and stop()

* test

  * [test-samplerobot-impedance] add test code for impedance controller API
  * [test-drc-testbet.test] add test code for drc samplerobot
  * [test-robot-hardware.test] add test to check robot-hardware service call
  * [test-colcheck.test] load modelfiles

* sample

  * [SampleRobot/samplerobot_auto_balancer.py] Update auto balancer python sample. Add toe heel samples.
  * [PA10/PA10.py] add from hrpsys import OpenHRP, something has changed during openhrp3 3.1.7 and 3.1.8, https://github.com/fkanehiro/openhrp3/commits/master
  * [sample/SampleRobot] fix for new PROJECT_DIR location
  * [sample/RampleRobot,sample/environment] install pyhton scripts under deval
  * [samplerobot_kalman_filter.py] fix typo
  * [samplerobot_kalman_filter.py] update kf test program for baseRpyCurrent
  * install SampleRobot.DRCTestbed.xml under devel
  * remove ROS examples, but enable to call Hrpsys examples directly

* .travis

  * [.travis.sh, .travis.yml] compile with -DROBOT_IOB_VERSION=0
  * [.travis.sh] any diff between 315.1.9 and current is not permitted, since we use cproto without any -DROBOT_IOB_VERSION, so this should output header file compatible with stable version
  * [.travis.sh] hot fix for https://github.com/start-jsk/rtmros_hironx/pull/358
  * [.travis.yml, .travis.sh] enable hrpsys with 315.1.10
  * [.travis.sh] RULE CHANGED adding new function to iob.h is ok
  * [.travis.sh] do not install test/share/samples/src of old hrpsys, use sample/test/launch of latest hrpsys
  * [.travis.sh] display test results when failure

* [util/monitor]

  * [Monitor.cpp] show velocity and acceleration (hold maxmum value for 5 sec)
  * [Monitor.cpp] add -nogui mode
  * [main.cpp] add --host, --port, --interval option
  * [GLscene.cpp, Monitor.cpp, main.cpp] add many error check codes

* [lib/util]

  * [bodyRTC.cpp] get sensor data through getStatus
  * [BodyRTC.cpp] fix bugs in `#200 <https://github.com/fkanehiro/hrpsys-base/issues/200>`_

* [CMakeLists.txt] touch rospack_nosubdirs for not search by roslaunch, onlyfor ROS users
* [python/hrpsyspy] rewrite hrpsyspy, now you can just call hrpsyspy to create hcf instance

Unstable RTCs
=============

* OccupancyGridMap3D

  * [OccupancyGridMap3D.cpp] outputs an update signal when onActivate() is called
  * [OccupancyGridMap3D.cpp] uses mutex lock to prevent crash

* ImpedanceController (API updated)

  * [JointPathEx.cpp] Enable to change weight caluclation. If use_inside_joint_weight_retrieval=true (true by default),
    inward joint weight retrievs to 1.0 (original). Otherwise, always weight is calculated from joint limit to solve
    https://github.com/fkanehiro/hrpsys-base/issues/516
  * Disable use_inside_joint_weight_retrieval by default in ImpedanceController, AutoBalancer, Stabilizer to reduce oscillation (https://github.com/fkanehiro/hrpsys-base/issues/516)
  * [idl/ImpedanceControllerService.idl] Add optional weight vector for impedance control IK. Currently, this is used for toe joint supression.
  * [ImpedanceController.cpp] Fix target_link calculation for IC. Support sensor->link is not same as ee target link (such as toe joint).

* AutoBalancer (API updated)

  * [GaitGenerator.cpp] Fix discontinuous autobalancer parameter using interpolator
  * [GaitGenerator.h] Fix foot trajectory discontinuous
  * [testGaitGenerator.cpp] Add plotting for several properties of GaitGenerator
  * [AutoBalancer.cpp] Enable to configure toe heel zmp transition

* Stabilizer (API updated)

  * [ZMPDistributor.h] Fix foot vertices and update evaluation codes
  * [Stabilizer.cpp] Update Stabilizer debug messages and calculation
  * [rtc/Stabilizer/testZMPDistributor.cpp] add stdio for qnx
  * [CMakeLists.txt] Add qpoases code. Disabled by default
  * [Stabilizer.cpp] Update for sole vertices
  * [ZMPDistributor.h] Add testing code for force distribution to use fz diff control ()
  * [ZMPDistributor.h] Add plotting of force moment
  * [Stabilizer.cpp] Separate ZMP distribution codes
  * [Stabilizer.cpp, idl/Stabilizer.idl] Enable to set Stabilizer gravitational acceleration (9.8 by default)
  * [GaitGenerator.cpp] Add comments and some functions are renamed
  * [GaitGenerator.cpp] Use negative value for heel_pos_offset_x
  * [idl/AutoBalancerService.idl] Fix unit system in documenatation
  * [AutoBalancer.cpp] Enable to set toe heel zmp
  * [GaitGenerator.cpp] Remove refzmp vel junping
  * [GaitGenerator.cpp] Add transition ZMP among toe, heel, and ee pos. Update tests.
  * [GaitGenerator.cpp] Rename supprot_leg_list -> swing_leg_list
  * [GaitGenerator.cpp] Update refzmp interpolation codes
  * [GaitGenerator.cpp] Use toe_heel_phase_counter pointer in lcg
  * [GaitGenerator.cpp] Add toe_heel_phase_counter. This should be change behaviour.
  * [AutoBalancer.cpp] Get default zmp offset and use ZMP offset in abc
  * [GaitGenerator.cpp] Push swing_foot_zmp_offset to PreviewControl qdata and update sample
  * [PreviewController.cpp] Add qdata for preview control
  * [GaitGenerator.cpp] Enable to get swing foot zmp offset
  * [testGaitGenerator.cpp] Update testGaitGenerator plotting
  * [AutoBalancer.cpp] Add check for toe heel ratio summation
  * [GaitGenerator.cpp] Use leg_type for array index
  * [AutoBalancer.cpp] Update leg_type. RLEG = 0, LLEG = 1. Add both and use it in Abc.
  * [/GaitGenerator.cpp] Rename WC_RLEG => RLEG, WC_LLEG => LLEG
  * [idl/AutoBalancerService.idl] Add StepParam and enable to set step height
  * [GaitGenerator.h, testGaitGenerator.cpp] Update testGaitGenerator sample
  * [idl/AutoBalancerService.idl] Add argument for toe and heel zmp
  * [idl/AutoBalancerService.idl, AutoBalancer.cpp, GaitGenerator.cpp, GaitGenerator.h] Add parameter for use toe joint or not
  * [AutoBalancer.cpp, GaitGenerator.cpp, GaitGenerator.h] Use toe joint in heel toe contact. (Disabled by default)
  * [GaintGenerator.cpp] Update interpolation from toe to heel

* Contributors: Eisoku Kuroiwa, Fumio KANEHIRO, Kei Okada, Shunichi Nozawa, YoheiKakiuchi, Chi Wun Au, Eisoku Kuroiwa, Masaki Mmurooka

315.3.2 (2015-04-13)
--------------------
* hrpsys_config.py

  * python 2.5 - 3.0 supports

    * [hrpsys_config.py,rtm.py] #559 is not what expected, we do not want to change output string style
    * [rtm.py, hrpsys_config.py] fix exception to work on all python version. see http://stackoverflow.com/questions/11285313/try-except-as-error-in-python-2-5-python-3-x
    * [rtm.py, hrpsys_config.py] use print() for support python2.5-python3.4

  * Behavior change on waitInputConfrim() https://github.com/fkanehiro/hrpsys-base/issues/480, https://github.com/fkanehiro/hrpsys-base/pull/565#issuecomment-92078185

    * [hrpsys_config.py] remove waitInputConfirmWithDisplayCheck(), since waitInputConfirm() in waitInput.py now support non-X environment
    * [hrpsys_config.py] previous commit changes waitInputConfirm() behavior, it returns True/False for Ok/Cance, not raise error
    * [waitInput.py] if window is not available, ask via screen, this changes behavior of waitInputConfirm() which previously raise error on cancel input, but not it returns False

  * [hrpsys_config.py] support $(OPENHRP_DIR) and $(PROJECT_DIR) in getBodyInfo and loadPattern
  * Add calibrateInertiaSensor function and add calibration function with dialog
  * [hrpsys_config.py] add setMaxLogLength()
  * [hrpsys_config.py] add dq to setupLogger()
  * Enable to get impedance controller mode

* lib/util/BodyRTC

  * [lib/util/BodyRTC.{cpp,h}] Set all servo off if robot has no joints, which means single rigid body
  * fixes a bug in checkEmergency()

* rtc/RobotHardware

  * send emergency signal by hardware servo alarm https://github.com/fkanehiro/hrpsys-base/pull/556

* sample/SampleRobot

  * [sample/SampleRobot] add README.md
  * [launch/samplerobot-drc-testbed.launch, sample/SampleRobot/SampleRobot.DRCTestbed.xml, sample/SampleRobot/environments/DRCTestbed*.wrl] add DRC Testbed models
  * [DRCTestbedTerrainUSBlock.wrl] fix floor color
  * [launch/samplerobot-terrain-walk.{launch,py} add sample program for terrain walk
  * [launch/samplerobot-walk.py] use check_output, instead of abspath(__file__) ../sample...
  * [launch/samplerobot*.py] remove python files just to run from ROS environment, now you can call everything by rosrun hrpsys samplerobot_auto_balacner.py
  * [sample/SampleRobot] use ''  to specify model location

* .travis.sh

  * [.travis.sh] remove hrpsys deb file for both downstream and 315.1.9 test
  * [.travis.sh] https://github.com/start-jsk/rtmros_hironx/pull/318.diff has been merged
  * [.travis.sh] remove hrpsys deb file for both downstream and 315.1.9 test
  * [.travis.h] https://github.com/start-jsk/rtmros_hironx/pull/318.diff has been merged
  * [.travis.yml] check python code work on python2.5 to python3.4
  * [.travis.sh] add --no-check-certificate

* Contributors: Fumio KANEHIRO, Kei Okada, Shunichi Nozawa, Yohei Kakiuchi

315.3.1 (2015-04-07)
--------------------

Stable RTCs
===========

* python/hrpsys_config.py

  * [hrpsy_config.py] paralell running of log is not available if ExtTrigExecutationContet In simulator, log is not using own ecs so it does not write log on emergency signal, if you wan to activate this se hcf.log_use_own_ec as samplerobot_data_logger, see https://github.com/fkanehiro/hrpsys-base/issues/490 for discussion
  * fixes a bug in disconnectPorts() and it returns True/False
  * (hrpsys python) Correct relative rotation. (https://github.com/fkanehiro/hrpsys-base/pull/543 )
  * (hrpsys_config.py) Better referencing to local getCurrentPose method.  (https://github.com/fkanehiro/hrpsys-base/pull/543 )
  * [hrpsys_conifg.py] set version for findComp()
  * [hrpsys_config.py] remove None from getRTCInstanceList() (https://github.com/fkanehiro/hrpsys-base/pull/534)
  * [hrpsys_config.py] add version check for some plugins when connectPorts() (https://github.com/fkanehiro/hrpsys-base/pull/533)
  * [hrpsys_config.py] add variables for servo controller instances (fixes https://github.com/fkanehiro/hrpsys-base/pull/529)
  * [hrpsys_config.py] add delteComps(), deleteComp(), deactiveComps() (https://github.com/fkanehiro/hrpsys-base/pull/512)
  * [hrpsys_conig.py] try to import OpenHRP3 for old version https://github.com/start-jsk/rtmros_hironx/issues/331
  * (Doc) minor clarification in hrpsys_config.py (https://github.com/fkanehiro/hrpsys-base/pull/505)

* python/rtm.py

  * [rtm.py] add more debug message when serializeComponents failed (https://github.com/fkanehiro/hrpsys-base/pull/536)
  * [rtm.py] add more debug message when failed connect port  (https://github.com/fkanehiro/hrpsys-base/pull/535)
  * [rtm.py] use rtc.ref.exit after delete_components https://github.com/fkanehiro/hrpsys-base/pull/512#issuecomment-80430387
  * [rtm.py] add delete() (https://github.com/fkanehiro/hrpsys-base/pull/512)

* hrpsys_simulator

  * [lib/util] enable robot simulation on RobotHardwareService (#187)
  * [hrpsys_config.py] fix for connecting to old RobotHardware which does not have service ports
  * [BodyRTC,PortHander] fix memory leak
  * Use basePose in kinemtics only simulation ( https://github.com/fkanehiro/hrpsys-base/pull/521 )

    * Connect RTCs' basePose outports to simulator basePose inport
    * Add basePose port to AutoBalancer like StateHolder
    * Add basePose inport for kinematics simultion (integrate == false)
    * Enable to parse ABS_TRANSFORM inport from link name according to outport ABS_TRANSFORM setting

  * fixes an invalid access to log

* SequencePlayer

  * (fix problem when calling setTargetPose in first time) https://github.com/fkanehiro/hrpsys-base/pull/519

    * [SequencePlayer.cpp] forget to call setInitialState in setTargetPose , update m_Ref.data using m_qInit.data
    * [SequencePlayer.cpp] Print info in setTargetPose

* CollisionDetector

  * add lin breank in debug message (https://github.com/fkanehiro/hrpsys-base/pull/528)
  * [CollisionDetector] set m_have_safe_posture to false in initial time, so that we can check if the system did not
    know any safe posture (https://github.com/fkanehiro/hrpsys-base/pull/525)

* DataLogger

  * [hrpsys_config.py] running log process for receiving emergency signal (https://github.com/fkanehiro/hrpsys-base/pull/544)
* lib/io

  * adds an option not to install libhrpIo.so

* lib/util

  * [SDLUtil.cpp] SDLwindow::~SDLwindow, call SDL_Quit only when this object is initialized (https://github.com/fkanehiro/hrpsys-base/pull/524)


Unstable RTCs
=============


* AutoBalancer

  * Add toe-off and heelo-contact (https://github.com/fkanehiro/hrpsys-base/pull/553 )

    * Use interpolator for rot_ratio
    * Add toe-off and heel-contact parameters.
    * Enable to set phase time ratio
    * Enable to interpolate smoothly if SOLE1 phase does not exist

  * Update st abc kf param (https://github.com/fkanehiro/hrpsys-base/pull/549)

    * Update print message for parameters
    * Update default parameters for ST, KF, ABC

  * Fix for limbcop and toe joint usage in ST (https://github.com/fkanehiro/hrpsys-base/pull/539)

    * Use ee_pos for foot edge points and alpha calculation
    * Fix variable name ee_trans => STIKParam and add comment
    * Calculate params and IK using reference COP instead of ee
    * Overwrite toe joints' joint angles based on StateHolder output
    * Fix getting ee name and sensor name in ST

  * Add ports for offsets of limbCOP (https://github.com/fkanehiro/hrpsys-base/pull/538)

  * Add user-defined weight for IK and use it for toe joint (https://github.com/fkanehiro/hrpsys-base/pull/515)

    * Set toe joint weight 0 by default
    * Enable to set user-defined weight vector for Inverse Kinematics

* TorqueController

  * Check boost version in TorqueController. sign.hpp is added at 1.35 (https://github.com/fkanehiro/hrpsys-base/pull/541)

* NullComponent

  * adds debug messages in constructor/destructor

* ImpedanceController

  * [rtc/ImpedanceController] return true/false for {start,stop}ImpedanceController https://github.com/fkanehiro/hrpsys-base/pull/513

* sample/SampleRobot

  * add yaw test code (samplerobot_kalman_filter.py : https://github.com/fkanehiro/hrpsys-base/pull/550)
  * Reset virtual force sensor conf setting (https://github.com/fkanehiro/hrpsys-base/pull/545)
  * [sample/SampleRobot/rtc/CMakeLists.txt] set OPENHRP_DIR (https://github.com/fkanehiro/hrpsys-base/pull/520)
  * [sample/SampleRobot/rtc] update for solo compilation (https://github.com/fkanehiro/hrpsys-base/pull/520)
  * [sample/SampleRbot/rtc] add sample rtc program for samplerobot(https://github.com/fkanehiro/hrpsys-base/pull/520)

* sample/environments/
  
  * Add Stair model and update Terrain model with ground https://github.com/fkanehiro/hrpsys-base/pull/540
  * Fix indent of vrml models
  * Add DRC testbed terrain VRMLs (JP block version and US block version) https://github.com/fkanehiro/hrpsys-base/pull/523

* doc

  * Add PD controller doc (https://github.com/fkanehiro/hrpsys-base/pull/517)
* travis.sh

  * [.travis.sh] use infinite loop to compile old hrpsys
  * [.travis.sh] do not touch installed, to rerun catkin_make twice
  * [.travis.sh] add CollisionDetector, which is disabled by USE_HRPSYSUTIL=0
  * [.travis.sh] run catkin_make_isolated twice for old hrpsys compile due to g++ memory error
  * [.travis.sh] disabling all SDL does not work
  * [.travis.sh] display time for each tests
  * [.travis.sh] use -j2 and disable SDL
  * [.travis.sh] do not compile hrpsysext (python binding)
  * [.travis.sh] less verbose when install code
  * [.travis.sh] disable roslisp generation for speedup
  * [.travis.sh] disable PCL/OCTMAP/IRRLIGHT for test
  * [.travis.sh] use -j2 for old hrpsys compile
  * [.travis.sh] 'check rtmros_common compiled on newer version of hrpsys works with deb version of hrpsys' means hrpsys running in OLD machine and roslaunch from NEW machine so we assume latest python codes using for this setup
  * [.travis.sh] use git repository, instead of svn due to googlecode shoutdown
  * [.travis.sh] apply https://github.com/start-jsk/rtmros_hironx/pull/318.diff
  * [.travis.sh] disable pcl/octmap/irrlight
  * [.travis.sh] do not show status for basic packages of  apt-get install
  * [.travis.sh] hrpsys_config requires openhrp3

* Contributors: Eisoku Kuroiwa, Fumio KANEHIRO, Isaac IY Saito, Kei Okada, Shunichi Nozawa, YoheiKakiuchi, eisoku9618

315.3.0 (2015-03-07)
--------------------

Stable RTCs
===========

* RobotHardware

  * Use average of force sensor values for force sensor offsetting and use it in removeforcesensorlinkoffset (https://github.com/fkanehiro/hrpsys-base/pull/455 )

* SequencePlayer

  * Write more message for Seq remove and add group functions (https://github.com/fkanehiro/hrpsys-base/pull/477)
  * Use Wrech In Sequence Player (https://github.com/fkanehiro/hrpsys-base/pull/434 )

* CollisionDetector

  * Add beep sound for CollisionDetector (https://github.com/fkanehiro/hrpsys-base/pull/476 )
  * Check Servo On/Off status in CollisionDetector (https://github.com/fkanehiro/hrpsys-base/pull/442 )
  * Use delete because m_interpolator is not array (https://github.com/fkanehiro/hrpsys-base/pull/430 )
  * Update collision debug message. Include m_loop_for_check and print time as [s] (https://github.com/fkanehiro/hrpsys-base/pull/430 )
  * Support CollisionDetector build without hrpsysUtil (https://github.com/fkanehiro/hrpsys-base/pull/428 )
  * Use onInitialize and onFinalize for CollisionDetector member init+finalizing, becaus onActivate (start) and onDeactivated (stop) may be used as servoOn/Off syncing, according to https://github.com/fkanehiro/hrpsys-base/issues/215#issuecomment-52633295)

* SoftErrorLimitter

  * Reduce debug print message from SoftErrorLimiter (once per 0.02[s]) (https://github.com/fkanehiro/hrpsys-base/pull/492 )
  * Add joint velocity limitation in SoftErrorLimiter (https://github.com/fkanehiro/hrpsys-base/pull/459 )
  * Swap order of error limit check and position limit check: check position limit check , then check error limit (https://github.com/fkanehiro/hrpsys-base/pull/457 )

* DataLogger

  * enables each instance has different max length (https://github.com/fkanehiro/hrpsys-base/commit/f9caa7a193a2909fd82cb85c52bdadf4783491f9 )

* [python/hrpsys_config.py]

  * Add 'some' argument for isServoOn and update documentation (https://github.com/fkanehiro/hrpsys-base/pull/484)
  * Add utility functions for Unstable RTCs (https://github.com/fkanehiro/hrpsys-base/pull/479)
  * connect rh->ic evenif they do not have rmfo (https://github.com/fkanehiro/hrpsys-base/pull/462 )
  * (hrpsys_config) : Reduce service port accessing and remove unnecessary sleep (https://github.com/fkanehiro/hrpsys-base/pull/424 )

* QNX

  * [Hrpsys.h] add sin/cos for QNX
  * [Hrpsys.h, testKalmanFilterEstimation.cpp] fix for qnx compile, see #470
  * Include Hrpsys.h for QNX compile according to https://github.com/fkanehiro/hrpsys-base/pull/476#issuecomment-73882215

* Documentation

  * configure doxygen to output xml data that can be imported from other document system (https://github.com/fkanehiro/hrpsys-base/pull/469)
  * Update seqencePlayer IDL documentation (https://github.com/fkanehiro/hrpsys-base/pull/448 )
  * (doc, hrpsys_config) Elaborate what wait means. (https://github.com/fkanehiro/hrpsys-base/pull/445 )
  * Add documentation of CollisionDetector (https://github.com/fkanehiro/hrpsys-base/pull/426 )

Unstable RTCs
=============

* New feature

  * adds a new component, RotateImage

* AutoBalancer (walking plugin)

  * Check leg name alternation in setFootSteps (https://github.com/fkanehiro/hrpsys-base/pull/496 )
  * Add graspless manip mode, in which foot steps are modified based on hand modification (https://github.com/fkanehiro/hrpsys-base/pull/487)
  * smooth velocity interpolation during sync mode (https://github.com/fkanehiro/hrpsys-base/pull/465 )
  * Fix initialize f value and this will solve divergence of rare pattern generation  (https://github.com/fkanehiro/hrpsys-base/pull/464 )
  * Update calculation of fix coords (https://github.com/fkanehiro/hrpsys-base/pull/449 )
  * Reduce duplicated ee vfs codes (https://github.com/fkanehiro/hrpsys-base/pull/438 )

    * Use virtual force sensor param in abc balancing
    * Rename m_sensor->m_vfs in ic
    * Move VirtualForceSensorParam to common file and use it in ABC and IC
    * Use localPos, link, localR in VirtualForceSensor
    * Rename to localPos and localR
  * Enable to set Gravitational_Acceleration (https://github.com/fkanehiro/hrpsys-base/pull/432 )

* AverageFilter

  * fixes a bug in conversion from point cloud to height field (https://github.com/fkanehiro/hrpsys-base/commit/b332f4476e8e9ee3da195329c7a761224e7ec5a1 )

* GraspController

  * Update GraspController's control law. Remove unnecessary limitation and gain parameter (https://github.com/fkanehiro/hrpsys-base/pull/453 )

* ImpedanceController

  * use manipulator name such as "larm" for configuration (https://github.com/fkanehiro/hrpsys-base/pull/467, related discussion -> https://github.com/start-jsk/rtmros_common/issues/649)
  * Fix Impedance Controlelr start + stop functions (https://github.com/fkanehiro/hrpsys-base/pull/461, Issue https://github.com/fkanehiro/hrpsys-baase/issues/232)
  * Fix velocity calculation. Convert ee frame objective vel -> link frame objective vel (https://github.com/fkanehiro/hrpsys-base/pull/441 )
  * Load end_effectors and use ee in ik and ref wrench, Support virtual force ref force (https://github.com/fkanehiro/hrpsys-base/pull/437 )
  * Add joint limit table and currently use el (https://github.com/fkanehiro/hrpsys-base/pull/433 )

* KalmenFilter

  * Use RPY-fixed linear KF by default (https://github.com/fkanehiro/hrpsys-base/pull/489 )
  * fix test cord for KF plugins(https://github.com/fkanehiro/hrpsys-base/pull/473, https://github.com/fkanehiro/hrpsys-base/pull/470)
  * Clean up Kalman filter codes and add test code made by @fkanehiro (https://github.com/fkanehiro/hrpsys-base/pull/469 )
  * Add SampleRobot KF example (https://github.com/fkanehiro/hrpsys-base/pull/463 )
  * Add ABS_TRANSFORM port and connect it to logger (https://github.com/fkanehiro/hrpsys-base/pull/463 )
  * Enable to change KalmanFilter Algorithm from idl (https://github.com/fkanehiro/hrpsys-base/pull/452 )
  * Update KF samle (https://github.com/fkanehiro/hrpsys-base/pull/450 )

* OccupancyGridMap3D

  * adds an inport to receive sensor position
  * adds an inport to receive range data
  * changes initial pose of sensor and replaces a deprecated function

* RemoveForceSnsorLinkOffset

  * Fix typo in RMFO. force_offset -> moment_offset (https://github.com/fkanehiro/hrpsys-base/pull/454 )

* Stabilizer

  * "fix" mode (fix hand position relateive to world coordinates) in stabilizer  (https://github.com/fkanehiro/hrpsys-base/pull/499 )
  * Add actual contact states port based on Force sensor measurement (https://github.com/fkanehiro/hrpsys-base/pull/474 )
  * Keep fix pose bofore solving IK (https://github.com/fkanehiro/hrpsys-base/pull/466)
  * Add reference wrench from state holder to stabilizer and new zmp distributor (https://github.com/fkanehiro/hrpsys-base/pull/422)
  * Control force xy during double support phase to avoid too large internal force (https://github.com/fkanehiro/hrpsys-base/pull/418)
  * Add alpha blending parameter for Fz distribution (https://github.com/fkanehiro/hrpsys-base/pull/417)

* ThermoLimitter

  * Reduce ThermoLimiter debug print (once per 0.1[s]) (https://github.com/fkanehiro/hrpsys-base/pull/491 )
  * Print in current value and max value in ThermoLimiter alart (https://github.com/fkanehiro/hrpsys-base/pull/447 )

* VideCapture

  * adds a test program
  * enables to change video capture format
  * fixes a memory leak
  * makes start_capturing return boolean result

* Other Fixes

  * [lib/util/GLutil.cpp] adds a missing bracket in output
  * [CMakeLists.txt, idl/CMakeLists.txt] fix for deb building
  * [CMakeLists.txt] RelWithDebInfo is not supported on hrpsys-base
  * [CMakeLists.txt] fix for shre/hrpsys/share link
  * [CMakeLists.txt] super ugry hack for catkin build (https://github.com/fkanehiro/hrpsys-base/pull/497)
  * create symlink for share directory for backword compatibility (https://github.com/fkanehiro/hrpsys-base/pull/471)
  * Autobalancer/ImpedanceController : Use lvlimit/uvlimit in dq limitation of JointPathEx. This commit changes costructor of JointPathEx (add control cycle argument) (https://github.com/fkanehiro/hrpsys-base/pull/460 )
  * [CMakeLists.txt] improve openrtm major version detection (https://github.com/fkanehiro/hrpsys-base/pull/446 )
  * fix problem when environment variable "_" not set (https://github.com/fkanehiro/hrpsys-base/pull/444 )
  * [sample/Sample6dofRobot] Fix rpy of sample6dofrobot (https://github.com/fkanehiro/hrpsys-base/pull/451 )
  * [sample/Sample6dofRobot] Use samplerobot as 6dofarm robot. Fix end effector link to ARM_WRIST_P which has force sensors (https://github.com/fkanehiro/hrpsys-base/pull/439 )
  * [sample/Sample6dofRobot] Update samplerobot examples (https://github.com/fkanehiro/hrpsys-base/pull/436 )

    * Add auto-balancer example to balance against reference hand forces
    * Add SoftErrorLimiter example for SampleRobot. Currently joint limit table is tested.
  * [sample/SampleRobot] Update seq sample without self collision (https://github.com/fkanehiro/hrpsys-base/pull/429 )
  * [sample/SampleRobot] Update collision detector examples (https://github.com/fkanehiro/hrpsys-base/pull/427 )

    * Add samplerobot_collision_detector example
    * Update autoablancer and terrain walk example without self collision
  * [sample/SampleRobot] Add example for SampleRobot SequencePlayer (https://github.com/fkanehiro/hrpsys-base/pull/419 )

* Contributors: Shunichi Nozawa, Yosuke Matsusaka, Fumio Kanehiro, Iori Kumaga, Kunio Kojima, Isaac IY Saito, Kei Okada

315.2.8 (2014-12-16)
--------------------

Stable RTCs
===========

* hrpEC

  * update message of timeover on hrpEC (https://github.com/fkanehiro/hrpsys-base/pull/411 )

* SequencePlayer

  * Update seq documentation to add loadPattern file format  (https://github.com/fkanehiro/hrpsys-base/pull/385 )

* SoftErorLImitter

  * Update beep sound frequency to avoid from making confusing alert sound. This commit is discussed in https://github.com/fkanehiro/hrpsys-base/issues/220, (https://github.com/fkanehiro/hrpsys-base/pull/403 )

* Fixes

  * add missing EXPORT headers (https://github.com/fkanehiro/hrpsys-base/pull/408 )
  * [.gitigure] adds a line for emacs backup files
  * [iob.h] updates comments ( https://github.com/fkanehiro/hrpsys-base/commit/c91ff93ba2459adcf810e1f8f8c8697b93a1d530 )
  * Use OpenHRP3 codes (https://github.com/fkanehiro/hrpsys-base/pull/370 )

    * (ImpedanceController,RatsMatrix) : Remove rotation_matrix and use Eigen toRotationMatrix
    * (RatsMatrix, Stabilizer) : Remove outer_product_matrix and matrix_exponent, use calcRodrigues, and comment out unused function
    * (RatsMatrix,AutoBalancer,GaitGenerator) : Remove local copied function. Remove translate, difference_position

* [python/rtm.py]

  * adds an option parameter pushpolicy to connectPorts() (https://github.com/fkanehiro/hrpsys-base/commit/1e7aaa7fea8e770fbdfe9ceb7a844fa370399df5)
  * clean up styles (https://github.com/fkanehiro/hrpsys-base/pull/358)

* [CatkinLists.txt] https://github.com/fkanehiro/hrpsys-base/pull/371

  * set Release if CMAKE_BUILD_TYPE is set to None
  * fix for ros buildfirm, check CATKIN_BUILD_BINARY_PACKAGE
  * Disable Compile java stuff

* add package.xml

Unstable RTCs
=============

* AutoBalancer

  * Fix zmp z to add zmp to preview queue (https://github.com/fkanehiro/hrpsys-base/pull/401 )
  * Add backward stride limitation, Remove deprecated samplerobot conf setting for abc (https://github.com/fkanehiro/hrpsys-base/pull/400 )
  * Fix double support phase for Rectangle and Stair orbit (https://github.com/fkanehiro/hrpsys-base/pull/399 )
  * Update default values for abc idl parameters (https://github.com/fkanehiro/hrpsys-base/pull/398 )
  * Fix servoOff transition of AutoBalancer.  (https://github.com/fkanehiro/hrpsys-base/pull/397 )
  * (GaitGenerator, PreviewController) : Push finailize reference zmp until que is full (https://github.com/fkanehiro/hrpsys-base/pull/396 )
  * (hrpsys_config, AutoBalancer) : Add debug port for COG (https://github.com/fkanehiro/hrpsys-base/pull/396 )
  * (AutoBalancer, Stabilizer) : Add interface to get current controller mode (https://github.com/fkanehiro/hrpsys-base/pull/387 )
  * Use optionalData from SH in ABC. (https://github.com/fkanehiro/hrpsys-base/pull/384 )
  * Enable to Passthrough the Reference  (https://github.com/fkanehiro/hrpsys-base/pull/382 )

    * (Stabilizer, AutoBalancer) : Use ref_zmp in base frame according to specification of Seq and Sh
    * (AutoBalancer) : Use MODE_SYNC_TO_IDLE instead of MODE_IDLE.
    * (AutoBalancer) : Use transition_interpolator for joint angle synchronization.
    * (samplerobot_auto_balancer.py) : Add example to check reference pass through by starting and stopping of abc mode.
    * (AutoBalancer) : Enable to pass through references.
    * (hrpsys_config.py) Revert qCurrent for st calculation.
  * Add rtc instance name in debug message (https://github.com/fkanehiro/hrpsys-base/pull/381 )
  * Input StateHolder Reference to ABC (https://github.com/fkanehiro/hrpsys-base/pull/379 )

    * (hrpsys_config, AutoBalancer) : Refine data ports. Remove unused qCurrent and rename in-out data ports.
    * (hrpsys_config, AutoBalancer) : Input references from StateHolder for AutoBalancer
    * (RatsMatrix) : Add mid_rot to interpolate matrix33
  * Update Many Coords... (https://github.com/fkanehiro/hrpsys-base/pull/378 )

    * check walking and add print message for setter function
    * get current value in onExecute and use target coords
    * remove unused gg setting and fix time
    * remove unused is_legged_robot checking
    * fix guard
    * remove unused resetting in startABC
    * set cog and end-coords in onExecute thread
    * remove unused param and fix initialize
  * Add new trajectory any time (https://github.com/fkanehiro/hrpsys-base/pull/369 )

    * (samplerobot_terrain_walk) : Support stair swing orbit type example
    * (AutoBalancer, GaitGenerator) : Add STAIR swing orbit type and gaitgeneratorParam for swing trajectory
    * (samplerobot_terrain_walk) : Do not specify step height.
    * (GaitGenerator) : Update rectangle trajectory to use common linear path function
  * Update control swing support time (https://github.com/fkanehiro/hrpsys-base/pull/364 )

    * (hrpsys_config) : Logging controlSwingSupportTime
    * (AutoBalancer, GaitGenerator, Stabilizer) : Define controlSwingSupportTime as TimedDoubleSeq for both feet

* CaptureController

  * enables to control frame rate (https://github.com/fkanehiro/hrpsys-base/commit/aa6a20ee241601243d580da813f5899f21d17545)

* ImpedanceController

  * Remove name from impedanceParam fucntion **** This fill change IDL ****. (https://github.com/fkanehiro/hrpsys-base/pull/388 )
  * Add rtc instance name in debug message (https://github.com/fkanehiro/hrpsys-base/pull/380 )
  * remove unused include (boost/interprocess/sync/interprocess_semaphore.hpp) (https://github.com/fkanehiro/hrpsys-base/pull/365 )

* ProjectGenerator

  * Connect ports of first input object. (https://github.com/fkanehiro/hrpsys-base/pull/402 )
  * Support PD Controller for project generation (https://github.com/fkanehiro/hrpsys-base/pull/373 )

    * (utilities.h) : Add documentation for use-highgain-mode and for default values
    * (ProjectGenerator) : Support PDcontrollre in ProjectGenerator.
  * Add documentation for ProjectGenerator (https://github.com/fkanehiro/hrpsys-base/pull/372 )

* PDController

  * Add instance name to PDcontroller debug message and add file open debug message  (https://github.com/fkanehiro/hrpsys-base/pull/486 )

* RemoveForceSensorLinkOffset

  * Add feture to dump offset file (https://github.com/fkanehiro/hrpsys-base/pull/415 )

    * Add example to use load and dump offset file for rmfo
    * Check string reading in rmfo loading
    * Update documentation for RMFO parameter file
    * Add service to load and dump rmfo offset files
  * Add component name to debug message (https://github.com/fkanehiro/hrpsys-base/pull/377 )

* Stabilzer

  * Represent st foot moment in ee frame (https://github.com/fkanehiro/hrpsys-base/pull/413 )
  * Add calcAlpha function to distribute force and moment. (https://github.com/fkanehiro/hrpsys-base/pull/412 )
  * Use foot edges in calculation of alpha (https://github.com/fkanehiro/hrpsys-base/pull/394 )
  * Fix Alpha Calculation (https://github.com/fkanehiro/hrpsys-base/pull/392 )

    * Calculate ee local moment and do not use OrientRotationMatrix
    * Consider foot orientation in calculating alpha
    * Add comments to specify frame
  * Move order of Stabilizer codes and remove unused Stabilizer codes. (https://github.com/fkanehiro/hrpsys-base/pull/391 )
  * Calculate LPF parameter based on controller time step dt and enable to set cut-off frequency from IDL. (https://github.com/fkanehiro/hrpsys-base/pull/389 )
  * Add component name to debug message (https://github.com/fkanehiro/hrpsys-base/pull/377 )
  * Enable to Change ST Algorithms (https://github.com/fkanehiro/hrpsys-base/pull/376 )

    * (StabilizerService, Stabilizer) : Add new body attitude control parameter not to share same parameter among different ST alg.
    * (Stabilizer) : Remove unnecessary ST algorithm checking.
    * (Stabilizer) : Just fix indent.
    * (StabilizerService, Stabilizer) : Enable to change st algorithm (TPCC or EEFM).
  * modify spell miss in comment (https://github.com/fkanehiro/hrpsys-base/pull/375 )
  * Update ST Transition (https://github.com/fkanehiro/hrpsys-base/pull/366 )

    * (Stabilizer) : Add eefm_pos_margin_time for gain scheduling
    * (Stabilizer, hrpsys_config) : Add data port for debugging data for Stabilizer

* [util/monitor, utils/simulator] (https://github.com/fkanehiro/hrpsys-base/pull/407 )

  * add help message for viewer as well
  * add small setting to make emacs people happy convert tab character to spaces
  * improve user interface

* New Features

  * adds a new component, AccelerationChecker
  * adds a new component MLSFilter

* [samples/SampleRobot]

  * (samplerobot_terrain_walk) : Update parameter setting for terrain walk example and add start up down demo
  * (sample6dofrobot_kalman_filter) : Update to use get and set  KalmanFilterParam. (https://github.com/fkanehiro/hrpsys-base/pull/390 )
  * (SampleRobot.conf.in) : Fix order of end effector according to ForceSensor order. (https://github.com/fkanehiro/hrpsys-base/pull/383 )

315.2.7 (2014-10-15)
--------------------

* New feature

 * (AutoBalancer, ImpedanceController) : Enable onDeactivated function for ImpedanceController and AutoBalancer
 * Added getControllerParams method and modified type of Param (struct -> class)
 * Added type check before use conf params
* Fixation

 * fix for old pcl
* Improvement

 * (ImpedanceController, AutoBalancer, Stabilizer) : Move to idling mode if stop() and start() are called. This is discussed in https://github.com/fkanehiro/hrpsys-base/issues/215
 * print exception when plugin is not found
 * add document to change timestep
 * (AutoBalancer) : Add time stamp to abc walking data ports and initialize contactStates
 * (GaitGenerator, Ratsmatrix) : Remove unused print functions and update print functions to use Eigen IOFormat
 * (AutoBalancer, ImpedanceController, RemoveForceSensorLinkOffset, Stabilizer) : Update unstable RTCs documentation
 * (hrpsys_config, DataLogger) : Enable to log contactStates as TimedBooleanSeq
 * set compile flag -ffloat-store to 32bit system add message for setting collision_loop
 * removes input dataport "sensorPose" and uses pose in RangeData
 * Connect q and qRef to ThermoEstimator to estimate joint torque from error
 * modify OpenHRP-3.1 path due to ROS-fhs layout, see `#128 <https://github.com/start-jsk/hrpsys/issues/128>`_
* Contributors: Kei Okada, Shunichi Nozawa, Isaac Saito

315.2.6 (2014-09-30)
--------------------
* New Feature

 * Adds PointCloudLogViewer, encords the number of points in point cloud. compiles PointCloudLogViewer only with PCL >= 1.7.
 * (Stabilizer) : Add leg inside margin to IDL
 * (KalmanFilter) : Add KalmanFilterParam as struct
 * Better error handling
* Fix
 
 * fixes linker error on 12.04amd
 * Partially reverted to handle Python version < 2.6
* hrpsys_config

 * Add DataLogger logging for servoState port.
 * Add playPattern* methods.
 * (hrpsys_config.py, SequencePlayer, StateHolder) : Add optional data for seq in https://github.com/fkanehiro/hrpsys-base/issues/190
 * Add and connect logger ports for offset force moment and ref force moment
* I/F improvement 

 * Added old style parameter functions to TwoDofController for Stabilizer
 * Modified controller arguments from double valiables to struct parameter.
* Document update

 * Apply doxygen style, somehow needs exclamation mark (http://stackoverflow.com/questions/7690220/how-to-document-python-function-parameter-types)
 * Doc improved for many components: AverageFilter, ExtractCameraImage, SequencePlayer, HGcontroller, CaptureController, VideoCapture, PCDLoader, SORFilter, RangeNoiseMixer, CaptureController, JpegEncoder, RGB2Gray, PlaneRemover, AverageFilter. AutoBalancer, Stabilizer, KalmanFilter, RemoveForceSensorLinkOffset, ImpedanceController
* Contributors: Kei Okada, Shunichi Nozawa, Isaac IY Saito

315.2.5 (2014-09-02)
--------------------
* Stabilizer:

 * Use force difference control
 * Add data port for Stabilizer root pos and rot debugging
 * (Stabilizer, hrpsys_config.py) Add debug port for Stabilizer compensation
 * Add both foot contact checker and update force z control
* AutoBalancer:

 * Add data port for swing and support period remain times and connect it between abc and st
 * Fix end effector name, e.g., :rarm => rarm. This change is based on JointGroup name discussed in https://github.com/fkanehiro/hrpsys-base/issues/232
* KalmanFilter:

 * Inhibit debug  print in KalmanFilter.h
 * Add DEBUG to control printing of KalmanFilter
 * KF -> EKF and RPY -> Quaternion

* sample6dofrobot*:

 * Add wrapper for sample6dofrobot examples added in https://github.com/fkanehiro/hrpsys-base/pull/281
 * Add set ref force and moment example for impedance controller
 * (terrain-walk) : Add wrapper of example in hrpsys-base samplerobot_terrain_walk.py

* (test/test-hostname.py) catch exit with exception(SystemExit)
* (readme) Clarify tasks in generating and merging changelog.
* (create_changelog) : Fix bug of hydro Changelog.rst path reported in https://github.com/start-jsk/hrpsys/pull/96/files#r16879095
* travis:

 * add graphbiz to install
 * add automatic push to gh-pages
* hrpsys_config.py:

 * Enable to use RMFO on robots without imu. Connect RPY port only if it exists.
 * move api doc for some methods from downstream.
 * enable to set reference frame in get{Reference,Current}{Pose,Position,Rotation,RPY}, see #297
 * use CPython as default python and add hrpsys_config.py
* adds a new component, AverageFilter
* Contributors: Kei Okada, Shunichi Nozawa, Isaac IY Saito, Yutaka Kondo

315.2.4 (2014-08-10)
--------------------
* AutoBalancer:

 * Add data port for acceleration reference which can be used in KalmanFilter.cpp
 * Use function and variable names. Use TargetParameter and CurrentParmeter
 * Remove duplicate codes for transition_smooth_gain
 * Remove unused codes and use is_legged_robot flag
* hrpsys_config.py:

 * Connect accRef from abc instead of seq. Note that connection from seq at previous r
 * Use contactStates in Stabilizer to specify single support ph
 * Add out data ports for Stabilizer debug
* (KalmanFilter.cpp) : Use accRef compensation
* (PDcontroller,...) : Add PD controller and examples
* samplerobot:

 * Add print message and comments to samples, remove direct writing of getRTCList, and 
 * Use .in file to specify openhrp3 directory for sample1.wrl model
 * Add conf_file setting to samplerobot.launch by copying hrpsys_tools/hrpsys/hrpsys.launch setting
 * Add impedancecontroller example
* Stabilizer:

 * Fix transition between MODE_AIR, MODE_IDLE, and MODE_ST. Set MODE_AIR if startStabilizer 
 * Fix USE_IMU_STATEFEEDBACK to USE_EEFM_STABILIZER for switching stabilizer algorithm and f
 * Add LPF for ground contact checking
 * Fix transition between st ON mode and st OFF mode
 * Rotate robot around COG in rpy control
 * Support rotational walking by fixing ref force and ref moment coordinates
 * Update calculation of actual and reference values for Stabilizer
 * Check legged robot or not
 * Add getActualParameters and update to use it
 * Update member variables (rename and remove)
 * Fix idl to specify zmp delay time constant and auxiliary zmp inp
* (Sample6dofRobot) : Add sample6dofrobot VRML which has 3 slide joints and 3 rotate joints. Add example f

* rtc/DataLogger/DataLogger.cpp rtc/DataLogger/DataLogger.h: remove needless variable tm from member metho
* (catkin.cmake, CMakeLists, samples/samplerobot*) : Move samplerobot examples to hrpsys-base https://github.com/fkanehiro/hrpsys-base/pull/252
* Contributors: Shunichi Nozawa, Kunio Kojima, Isaac IY Saito

315.2.3 (2014-07-28)
--------------------
* Adjusted to OpenRTM 1.1.1
* use OCTOMAP_LIBRARY_DIRS instead of OCTOMAP_DIR, Fix #258
* Use boost library for copysign because copysign in cmath only can be used in C++11 later
* samplerobot:

  * Add example for impedancecontroller rtc. 
  * Add examples for samplerobot by copying from start-jsk/hrpsys/samples discussed in https://github.com/fkanehiro/hrpsys-base/issues/240. 
  * Add setFootSteps examples. 
  * Add samples for DataLogger and Stabilizer.
  * Add example for impedancecontroller rtc
* (JointPathEx.*, AutoBalancer, Stabilizer, ImpedanceController) : Remove solveLimbIK and use calcInverseKinematics2Loop
* (samplerobot_auto_balancer.py, AutoBalancer.cpp) Fix overwriting of target foot coords, add example to check non-default stride stopping, and check RECTANGLE swing orbit
* JointPathEx.*:

  * Move nullspace codes to reduce difference between calcInverseKinematics2Loop and solveLimbIK. 
  * Remove unnecessary transition_count and resetting of nullspace vector. 
  * Move nullspace codes to reduce difference between calcInverseKinematics2Loop and solveLimbIK.
* hrpsys_config.py:

  * Add readDigitalOutput.
  * Add connection for st qCurrent. 
  * Add comment upon setTargetPose IK failure. 
  * Add logger connection for walking RTCs. 
  * Use Group to find eef name. PEP8 improvement.
* Stabilizer.*:

  * Add new stabilizer control law (currently not enabled). 
  * Use :end_effector instead of link origin in IK and fix mode transition.
  * Add getParameter function for stabilizer parameter
* create_changelog.sh: Add script for changelog from subdirectory information (discussed in `jsk-ros-pkg/jsk_roseus#134 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/134>`_)
* GaitGenerator.*:

  * Fix bug of swing foot calculation and add reset orbit
  * Support rectangle foot swing orbit
* (AutoBalancerService.idl, AutoBalancer.*, GaitGenerator.*, testGaitGenerator) : Enable to configure swing orbit type
* (TorqueController) Added TwoDofControllerDynamicsModel option to initialize process. Use dynamic model based on equation of motion.
* Fixed default tauMax from model. climit -> climit*gearRatio*torqueConst
* Modified m_loop type int -> long long
* Contributors: Kei Okada, Shunichi Nozawa

315.2.2 (2014-06-17)
--------------------
* (catkin.cmake) add code to check if hrpsys is installed correctly
* manifest.xml/package.xml: depends on cv_bridge instad of opencv (https://github.com/ros/rosdistro/pull/4763)
* add patch to use opencv2.pc for last resort
* (catkin.cmake) install src directory for custom iob
* fix for hrp4c.launch
* update to hrpsys version 315.2.2
* (catkin.cmake) install src directory for custom iob, see https://github.com/start-jsk/rtmros_gazebo/issues/35 for discussion
* (hrp4c_model_download.sh) set rw permissions to all users for hrp4c model
* (catkin.cmkae) use sed to fis install dir
* sample/samplerobot-remove-force-offset.py : add sample code for RMFO rtc
* (catkin.cmake) add disable ssl
*
* update in fkanehiro/hrpsys-base repository
* 74d07f9 (lib/util/CMakeLists.txt) forget to install Hrpsys.h (24c6139826)
* 0303d15 (rtc/PlaneRemover) adds a configuration variable pointNumThd to specify the minimum number of points to define a plane#226 from orikuma/refactoring-thermo-limiter
* f34f28b (python/rtm.py) adds return value of setConfiguration() and setProperty()
* 85afa1c (rtc/ThermoLimiter) Removed TwoDofController, which is not used in ThermoLimiter now
* 63f3ae7 (python/hrpsys_config.py) add getRTCList for unstable RTCs
* 9eb3a12 (rtc/SORFilter) fixes typos(again)
* 233a31a (rtc/PlaneRemover) adds a new component, PlaneRemover
* 26f2f09 (rtc/SORFilter) fixes typos
* c5a8ee5 (rtc/TorqueFilter) Modified debug message position for tf params
* 9c13ee2 (rtc/TorqueFilter) Added timestamp to tf.rtc:tauOut and modified method to deal with input error3e Modified and supressed error messages for TorqueFilter
* de0b63e (rtc/TorqueFilter) Modified and supressed error messages for TorqueFilter
* 6ebcb7b (rtc/TorqueController) Supress error message by debugLevel and output qRefIn to qRefOut when torque controller does not work due to some fault of input.
* d3a7750 (rtc/PCDLoader) removes backup files
* eafe5f5 (rtc/PCDLoader) adds a new component, PCDLoader

* Contributors: Kei Okada, Shunichi Nozawa

315.2.1 (2014-05-12)
--------------------
* Merge pull request `#83 <https://github.com/start-jsk/hrpsys/issues/83>`_ from k-okada/add_git
  add build_depend to git
* Contributors: Kei Okada

315.2.0 (2014-05-11)
--------------------
* update in fkanehiro/hrpsys-base repository
* 53de9aa (hrpsys_config.py) fix getRTCList only for stable RTC
* 69b153e (KalmanFilter, Stabilizer) adds options to disable building KalmanFilter and Stabilizer
* 1c6a1dd (hrpsys_config.py) add DataLogger clear in setupLogger to start log data with same starting time
* ad5401f (rtm.py) use % operator instead of format ;; format cannot be used in python < 2.6
* 7eec546 (KalmanFilter) avoid devision by zero
* d6db569 (CMakeLists.txt) add Boost patch (remove -mt suffix)
* 5dc9883 (ImpedanceController) add time stamp to output port, which are copied from m_q input time stamp
* 917c8f1 (AutoBalancer) add time stamp to output ports, which are copied from m_q input time stamp
* 9f09a3e (AutoBalancer) add baseTform to output transformation of base link
* eaf85c2 (VideoCapture) enters ERROR state when a video devices doesn't exist
* 8034945 (VideoCapture) opens video devices at onActivate()
* b3e253b (SORFilter) adds a new component, SORFilter(PCL is required)
* ec32ed0 (VideoCapture) enables to specify camera device ids by using a configuration variable, devIds
* d651827 (AutoBalancer) fix first foot steps ;; this update is discussed in https://github.com/jsk-ros-pkg/jsk_control/issues/1
* e889719 (RemoveForceSensorLinkOffset) remove unused files commited at previous commit
* 430aa95 rename rtc ;; AbsoluteForceSensor -> RemoveForceSensorLinkOffset
* 72fff04 (AutoBalancerService.idl, AutoBalancer) update start and stop function for AutoBalancer mode ;; use string sequence instead of deprecated type's sequence ;; rename function
* 811c573 (AutoBalancerService.idl) update comments for AutoBalancer idl
* fb155c6 (hrpsys_config.py, SequencePlayer) adds an input data port, zmpRefInit to SequencePlayer(by notheworld)
* 47677b7 (util/PortHandler.cpp) updates an error message
* 9417846 (315.1.10:sample/HRP4C/HRP4C.py) fix HRP4C.py: use `__main__` to call demo() and it also call initCORBA, see Issue 195
* d30a9f6 (315.1.10:sample/PA10/PA10.py) log is already started in activateComps()
* d09f1b9 (315.1.10:rtm.py) print error message when roonc is not defined in findRTCmanager and findObject, it also set hostname from set.gethostname if not defined in findRTCmanager(), see Issue #173
* d196165 (315.1.10:sample/PA10/PA10.py) use `__main__` to call demo() and it also call initCORBA, see Issue 195
* ed59880 (AutoBalancer) set current footstep pos and rot even if not ABC mode
* 6b84d09 (Range2PointCloud) supports unsymmetric scan angles
* 12ff024 (lib/util/PortHandler.cpp) sets RangerConfig
* 25df3dd (python/waitInput.py) executes waitInputMenuMain() in a thread
* 76f5762 (rtm.py) fixes a typo
* c0d8a92 (rtm.py) adds the second argument to load()
* d7b2646 (ImageData2CameraImage) initialize error_code
* b54cb47 (RangeDataViewer) adds a new component, RangeDataViewer
* 1e6360e (315.1.10:ProjectGenerator) do not pass non-openrtm arg to Manager::init(), see Issue #193
* de4b353 (415.1.10:ProjectGenerator) clean up debug message see Issue #193
* 03ec80d (lib/util/VectorConvert.h) adds operator>> for hrp::dvector and hrp::Vector3
* 77af006 (SequencePlayer/interpolator.cpp) enable user to change DEFAULT_AVG_VEL, see Issue 189 (interpolators[WRENCHES])
* 1859064 (SequencePlayerService.idl) add setWrenches, interpolate wrench in seq, see Issue 153
* 848bbfc (hrpsys_config.py) add function documents, many thanks to isaac
* e203012 (hrpsys_config.py) add to call setSelfGroups in init()
* 73f80e2 (hrpsys_config.py) move common code for real robots, see issue https://github.com/start-jsk/rtmros_common/issues/289
* 2182a35 (TorqueController) show error message every 100 loops
* 90a8bfc (hrpsys_config.py) do not raise error when component is not found in findComp
* 9fd098e (hrpsys_config.py) add findComps, see https://github.com/start-jsk/rtmros_common/issues/340
* ccf60e3 (hrpsys_config.py) fix wrong commit on r976/Issue #179
* bd4e92f (CMakeLists.txt) add more message when library is not found
* f966a06 (CMakeLists.txt) add message when library is not found
* 3feb6b3 (SequencePlayer) adds a misc. change
* 5741b9f (SequencePlayer) revert rpy loading according to discussion in https://code.google.com/p/hrpsys-base/source/detail?r=896 ;; load RPY from .hip file and load pos and RPY from .waist file
* 0a1ee15 (CaptureController) add a new component CaptureController
* 67b6b7d (hrpsys-base.pc.in) add idldir to hrpsys-base.pc.in
* 24bd8fa (FindOpenHRP.cmake) use OPENHRP_IDL_DIR for openhrp3 idl file location
* 87e91e5 (hrpsys_config.py) support  setTargetPose(self, gname, pos, rpy, tm, frame_name=None), fixed Issue 184
* 2936ce6 (ImpedanceController) more user friendly error message
* a386425 (rtm.py) fixes a problem in readDataPort() and adds an option, disconnect to writeDataPort
* 576a969 (rpy.py) More human friendly error message upon connection error, see Issue 183
* 6539ee3 (Range2PointCloud) supports multiple lines
* a585b54 (VideoCapture) fixes a bug in oneshot mode
* 9d6517f (rtm.py) add more user friendly error message
* a66c478 (CMakeLists.txt, rtc) set tag version to compoent profile version, see Issue 181
* 1a284f7 (Range2PointCloud) adds a port for sensor pose input
* 08a2dc1 (lib/util/PortHandler.cpp) sets angularRes in RangeData
* bff42b8 (ExtractCameraImage) add a new component, ExtractCameraImage
* 26dc4e4 (ImageData2CameraImage) add a component ImageData2TimedcameraImage
* f1f90d8 (sample/visionTest.py) installs visionTest.py
* d5c79c2 (VideoCapture) fixes a problem in oneshot mode
* 1446d24 (hrpsys_config.py) fix confusing variable names pos->angles, see Issue 179
* d6c56f8 (sample/visionTest.py)adds a sample script to use vision related RTCs
* 099bd22 (JpegDecoder) supports grayscale images
* d5e5096 (Img.idl) adds new image formats
* 520a3d4 (VideoCapture) added a service port for CameraCaptureService to VideoCapture component
* 2219c36 (ResizeImage) add a component ResizeImage(not tested yet)
* 58fe438 (RGB2Gray) added a component RGB2Gray
* 556d65c (JpegEncoder) added a component JpegEncoder
* c39d7a3 (VideoCapture) changes data type of outport depending on the number of cameras
* 7f9d2f5 (CameraImageViewer) corrects description
* 
* update to hrpsys version 315.2.0, remove patches
* use hrpsys_config.py according to https://github.com/start-jsk/hrpsys/pull/79 discussion ;; support latest autoablancer idl
* import imp package and roslib
* pass EXTRA RTC setting by string
* fix Makefile.hrpsys-base, git checkout $(GIT_REVISION) after git reset --hard
* use hrpsys_config.py for creating RTCs, connecting of ports, and activation
* (package.xml) Add version semantics clarification.
* use http://github.com/fkanehiro/hrpsys-base
* remove installed file if openhrp3_FOUND is not found
  Add auto balancer samples
* add sample code for auto balancer
* add AutoBalancer parameter to SampleRobot.conf.in
* add conf setting for StateHolder and AutoBalancer
* Merge pull request `#63 <https://github.com/start-jsk/hrpsys/issues/63>`_ from k-okada/315_1_10
  update to 315.1.10
  - ProjectGenerator : clean up debug message  (https://code.google.com/p/hrpsys-base/issues/detail?id=193)
  - PA10.py : call initCORBA() in `__main__`,log is already started in activateComps() so comment out setupLogger()  (https://code.google.com/p/hrpsys-base/issues/detail?id=195)
  - rtm.py : add debug messages if function called without initCORBA ()https://code.google.com/p/hrpsys-base/issues/detail?id=173
* qhull.patch only requres for arch package
* samples/{pa10,hrp4c,samplerobot}.launch: add sample programs
* test-pkg-config.py: add test code to check if file exists, test-joint-angle.py: add more test on setJointAngle
* move to 315.1.10
* Update README.md
* (test-hostname.py) add more debug message when test failed
* start_omninames.sh: fix typo
* add rosbash : temporarily until openrtm_aist_core provides rosbash
* `test-*.py`: use imp.find_module to check if we need to use roslib.load_manifest()
* (test-hostname.py): add more debug message when test failed
* add start_omninames.sh start omniNames for test code
* add Isaac to maintainer
* add python-tk to run_depend
* (CMakeLists.txt) fix conf file path for deb/rosbuild environment
* fix rosbuild compile option for working both deb/source
* add PKG_CONFIG_PATH for rosbuild environment
* (.travis.yml) add rosbuild/deb test
* Contributors: Isaac IY Saito, Kei Okada, Ryohei Ueda, Shunichi Nozawa

315.1.9 (2014-03-15)
--------------------
* "315.1.9"
* prepare for release 315.1.9
* Merge pull request `#53 <https://github.com/start-jsk/hrpsys/issues/53>`_ from k-okada/failed_to_compile_using_rosbuild_52
  - add test codes
  - merge `#39 <https://github.com/start-jsk/hrpsys/issues/39>`_
  - fix PKG_CONFIG_PATH before rostest
  - use load_manifest for rosbuild
* use load_manifest for rosbuild
* set PKG_CONFIG_PATH before rosmake test to find openhrp3.1.pc and hrpsys-base.pc
* use := instead of ?= because ?= does not work if PKG_CONFIG_PATH exists and openrtm.pc or openhrp3.pc are not included in PKG_CONFIG_PATH ;; I does not work groovy+rosbuild environment
* add test codes
* add rosbuild/roslang to depend
* rename manifest.xml for rosdep, see https://github.com/jsk-ros-pkg/jsk_common/issues/301
* add retry for test, see https://code.google.com/p/hrpsys-base/issues/detail?id=192 for the problem
* add groovy/catkin/deb
* fix openhrp3 path for deb environment
* (manifeset.xml) add restest to rosdep
* check rosdep until it succeeded
* fix print LastTest.log
* Add python patch for Arch
* Add Boost patch (remove -mt suffix).
* Fix qhull paths.
* (manifeset.xml) add restest to rosdep
* check rosdep until it succeeded
* check rosbuild/catkin deb/source with travis
* clean up test code for hrpsys (use findComps(), add DataLogger, test hrpsys_config.py, cleanup test name)
* start using 315.1.9, do not release until 315.1.9 is finally fixed
* added -l option as well as -j
* compile hrpsys in parallel, but it's up to 12 parallel jobs
* (hrpsys_config.py) wait (at most 10sec) if findComp found target component, check if  RobotHardware is active, see Issue #191
* (hrpsys_config.py) add max_timeout_count to findComps, if findComp could not find RTC  (for 10 seconds), successor RTC only check for 1 time
* Contributors: Benjamin Chrétien, Kei Okada, Ryohei Ueda, Shunichi Nozawa

315.1.8 (2014-03-06)
--------------------
* Do not pollute src directory, https://github.com/start-jsk/hrpsys/issues/3
* Utilize .travis.yml
* Initial commit of CHANGELOG.rst
* Contributors: Kei Okada, Atsushi Tsuda, Isaac Isao Saito, chen.jsk, Ryohei Ueda, Iori Kumagai, Manabu Saito, Takuya Nakaoka, Shunichi Nozawa, Yohei Kakiuchi
