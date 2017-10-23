^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hrpsys
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

315.15.0 (2017-10-22)
---------------------

Stable RTCs
=============
* SequencePlayer
  * Fix bug in CUBICSPLINE mode (`#1201 <https://github.com/fkanehiro/hrpsys-base/issues/1201>`_)
    * fix bug in CUBICSOLINE mode a[5] -> a5[i]
  * Remove offset while playing patterns (`#1191 <https://github.com/fkanehiro/hrpsys-base/issues/1191>`_)
    * enable to generate continuous motion from motion patterns
    * enable to fix a link while playing patterns
* RobotHardware/DataLogger
  * Add data port for TimedRobotState2 `#1195 <https://github.com/fkanehiro/hrpsys-base/issues/1195>`_ from fkanehiro/add_rstate2_port
    * add an output data port for TimedRobotState2
* RobotHardware
  * Add servo on delay (`#1210 <https://github.com/fkanehiro/hrpsys-base/issues/1210>`_)
    * [RobotHardware] add document on servoOnDelay
    * [RobotHardware] add servoOnDelay configuration variable
  * Fix bug in getStatus() (`#1197 <https://github.com/fkanehiro/hrpsys-base/issues/1197>`_)
    * [RobotHardware] fix a bug in getStatus()
* DataLogger
  * Add logSplitter (`#1198 <https://github.com/fkanehiro/hrpsys-base/issues/1198>`_)
    * add logSplitter
* rtm.py (`#1199 <https://github.com/fkanehiro/hrpsys-base/issues/1199>`_)
  * [rtm.py] add RTcomponent.getProperties()
* hrpEC
  * Add missing initialization (`#1209 <https://github.com/fkanehiro/hrpsys-base/issues/1209>`_)
    * Add missing initialization
  * Detect use-after-free of hrpExecutionContext (`#1208 <https://github.com/fkanehiro/hrpsys-base/issues/1208>`_)
    * Remove C++11 dependency from previous commit
    * Detect use-after-free of hrpExecutionContext
* Travis (travis.sh, travis.yaml)
  * Update travis to run hydro/indigo/kinetic (`#1194 <https://github.com/fkanehiro/hrpsys-base/issues/1194>`_)
    * .travis.sh: remove -j1 -l1 to speed up compile
    * Update .travis.yml
    * run docker without -it
    * disable DEBIAN_FRONTEND, install tzdata before other package https://askubuntu.com/questions/909277/avoiding-user-interaction-with-tzdata-when-installing-certbot-in-a-docker-contai
    * .travis.sh: old git does not support -b with tag name
    * export DISTRO
    * skip hydro specific patches
    * use DISTRO
    * use ROS_DISTRO instead of hydro
    * show lsb_release
    * .travis.yml: add hydro/indigo/kinetic: test on .travis.yml
    * apt-get install python2.7
    * install apt-get software-properties-common
    * mkdir tet_results
    * install git,wget,sudo,sed
    * use docker to run test

Unstable RTCs
=============

* ReferenceForceUpdater
  * Add is_hold_value flag writing (`#1213 <https://github.com/fkanehiro/hrpsys-base/issues/1213>`_)
    * [rtc/ReferenceForceUpdater/ReferenceForceUpdater.cpp] Add missing writing of is_hold_value flag
  * Update rfu functions (`#1212 <https://github.com/fkanehiro/hrpsys-base/issues/1212>`_)
    * [idl/ReferenceForceUpdaterService.idl, rtc/ReferenceForceUpdater] Enable to set transition time for RFU
    * [idl/ReferenceForceUpdaterService.idl, rtc/ReferenceForceUpdater] Add no-wait version functions
* AccelerationChecker (`#1205 <https://github.com/fkanehiro/hrpsys-base/issues/1205>`_)
  * remove a newline
  * check joint command acceleration only when servo is on
* AutoBalancer (`#1203 <https://github.com/fkanehiro/hrpsys-base/issues/1203>`_)
  * [rtc/AutoBalancer/AutoBalancer.cpp] Update for MODE_REF_FORCE_RFU_EXT_MOMENT. Separate MODE_REF_FORCE_RFU_EXT_MOMENT from other UseForceMode.
* CameraImageSaver : Add new component (`#1200 <https://github.com/fkanehiro/hrpsys-base/issues/1200>`_)
  * add bindParameter()
  * add a new component, CameraImageSaver
* CameraImageViewer : Support RTC::CameraImage (`#1196 <https://github.com/fkanehiro/hrpsys-base/issues/1196>`_)
  * [CameraImageViewer] support RTC::CameraImage
* Stabilizer
  * Update calculation of foot origin ext moment (`#1203 <https://github.com/fkanehiro/hrpsys-base/issues/1203>`_)
    * [rtc/Stabilizer/Stabilizer.*] Update calculation of diff_foot_origin_ext_moment not to use ZMP and to use foot moment
    * [rtc/Stabilizer/Stabilizer.cpp] Move calculation of actual foot origin coords. This is expected not to change the program behavior.
  * Update travis to run hydro/indigo/kinetic (`#1194 <https://github.com/fkanehiro/hrpsys-base/issues/1194>`_)
    * sample/SampleRobot/samplerobot_stabilizer.py: on hrpsys < 315.5.0 this outputs huge error log message

* Contributors: Fumio KANEHIRO, Jun Inoue, Kei Okada, Shunichi Nozawa, Yasuhiro Ishiguro

315.14.0 (2017-08-04)
---------------------

Stable RTCs
=============

* .gitignore (`#1186 <https://github.com/fkanehiro/hrpsys-base/issues/1186>`_)
  * [.gitignore] add qpoases directory

* CMakeLists
  * Update for compile definitions (`#1179 <https://github.com/fkanehiro/hrpsys-base/issues/1179>`_)
    * CMakeLists.txt : add " for string REGREX whoen OPENHRP_DEFINITIONS is not set
  * Update compile flags (`#1177 <https://github.com/fkanehiro/hrpsys-base/issues/1177>`_)
    * CMakeLists.txt : remove -O2 from OPENHRP_DEFINITIONS

* CollisionDetector
  * Update debug message (`#1178 <https://github.com/fkanehiro/hrpsys-base/issues/1178>`_)
    * CollisionDetector.cpp: setupVclipModel() : display size of vclip model verts
  * Add service (`#1174 <https://github.com/fkanehiro/hrpsys-base/issues/1174>`_)
    * add service for collision loop

* rtm.py
  * Fix (`#1183 <https://github.com/fkanehiro/hrpsys-base/issues/1183>`_)
    * fix classFromString()
  * Update behavior (`#1181 <https://github.com/fkanehiro/hrpsys-base/issues/1181>`_)
    * 関数名を変更
    * readDataPort、writeDataPort関数で毎回コネクタを生成しないようにする

* SequencePlayer
  * Update sample checking (`#1143 <https://github.com/fkanehiro/hrpsys-base/issues/1143>`_)
    * [sample/SampleRobot/samplerobot_sequence_player.py] Check return value of checkJointAngles of SequencePlayer sample.
  * Update for clearOfGroups (`#1141 <https://github.com/fkanehiro/hrpsys-base/issues/1141>`_)
    * SequencePlayer.cpp: (setTargetPose) send resetJointGroup to m_seq before call playPatternOfGroup
    * add test code to check clearOfGroups work when setTargetPose is called
  * Fix bug of return value (`#1139 <https://github.com/fkanehiro/hrpsys-base/issues/1139>`_)
    * [rtc/SequencePlayer/interpolator.h] Fix bug of return value type of interpolator dimension.

* hrpsys_config.py
  * Update debug message (`#1153 <https://github.com/fkanehiro/hrpsys-base/issues/1153>`_)
    * hrpsys_config.py: display findComps -> (rtcname)
  * Update documentation (`#1135 <https://github.com/fkanehiro/hrpsys-base/issues/1135>`_)
    * [py][doc] Clarify setTargetPoseRelative's wait behavior (addresses `#1121 <https://github.com/fkanehiro/hrpsys-base/issues/1121>`_).
      `wait` argument is to regulate the commands previously run before `setTargetPoseRelative`, but there are users who thought it stops subsequent commands.
      This change hopefully clarifies the difference.

Unstable RTCs
=============
* PointCloudViewer
  * Update checking (`#1145 <https://github.com/fkanehiro/hrpsys-base/issues/1145>`_)
    * [PointCloudViewer] Validity check seems not to be needed when is_dense param is correctly set
    * [PointCloudViewer] Use union rgb member in PointXYZRGB
    * [PointCloudViewer] Fix point validity check, isnan should take floating value
    * [PointCloudViewer] Fix validity check of color points
  * Support coloring (`#1144 <https://github.com/fkanehiro/hrpsys-base/issues/1144>`_)
    * [PointCloudViewer] Support color pointcloud view

* Beeper (`#1143 <https://github.com/fkanehiro/hrpsys-base/issues/1143>`_)
  * [rtc/Beeper/Beeper.cpp] Update sleep time of beep. Do not access beep device during previous beep command is executed.

* GraspController (`#1160 <https://github.com/fkanehiro/hrpsys-base/issues/1160>`_)
  * [rtc/GraspController] Add [] for grasp controller instance name and add more information about parsing of joint group setting

* For fullbody manipulation : AutoBalancer, Stabilizer, ObjectContactTurnaroundDetector, ReferenceForceUpdater
  * Fix bug of initialization (`#1160 <https://github.com/fkanehiro/hrpsys-base/issues/1160>`_)
    * [rtc/ReferenceForceUpdater] Update to use interpolated results.
    * [rtc/ReferenceForceUpdater] Output default value for refFootOriginExtMoment if is_active is false.
    * [rtc/Stabilizer] Do not calculate actual foot_origin_ext_moment if in the air, because of invalid act_zmp.
  * Update for fullbody manip (`#1192 <https://github.com/fkanehiro/hrpsys-base/issues/1192>`_)
    * [idl/ObjectContactTurnaroundDetectorService.idl, rtc/ObjectContactTurnaroundDetector] Add TOTAL_MOMENT2 to use end-effector moments
    * [idl/ObjectContactTurnaroundDetectorService.idl, rtc/ObjectContactTurnaroundDetector, sample/SampleRobot/samplerobot_carry_object.py] Add friction coefficient wrench. Update samples to print it.
    * [rtc/AutoBalancer/AutoBalancer.cpp] Set 1.0 as default value for swingsupport time
    * [rtc/Stabilizer/Stabilizer.cpp] Update increment of support_time. In some PC, previous version does not work because of optimized out.
  * Update for single-leg support (`#1180 <https://github.com/fkanehiro/hrpsys-base/issues/1180>`_)
    * [rtc/AutoBalancer] Support foot contact manipulation in static balance point calculation
    * [python/hrpsys_config.py, rtc/ObjectContactTurnaroundDetector] Use reference contact states in object contact turnaround detector foot origin coords calculation
    * [rtc/Stabilizer/ZMPDistributor.h] Add check for ee size. Currently, two feet contact is supported.
    * [rtc/AutoBalancer] Check leg_names length for biped force distribution
    * [rtc/ObjectContactTurnaroundDetector] Use footorigincoords calculation in OCTD
  * Support fullbody manipulation (`#1151 <https://github.com/fkanehiro/hrpsys-base/issues/1151>`_)
    * [rtc/ReferenceForceUpdater] Add arm name printing to add more information in debug print
    * [sample/SampleRobot/samplerobot_carry_object.py] Add Joint Groups setting for startDefaultUnstableControllers
    * [sample/SampleRobot/samplerobot_reference_force_updater.py] Update rfu sample (argument update checking and FootOriginExtMoment)
    * [python/hrpsys_config.py] Connect FootOriginExtMoment ports for fullbody contact manipulation
    * [rtc/AutoBalancer] Add distibution of zmp to ref force/moment.
    * [rtc/AutoBalancer] Input ext moment around the foot origin pos and update static balance point calculation for foot force and ext moment.
    * [idl/AutoBalancerService.idl,rtc/AutoBalancer] Add MODE_REF_FORCE_RFU_EXT_MOMENT and MODE_REF_FORCE_WITH_FOOT of UseForceMode for fullbody contact manipulation.
    * [rtc/ReferenceForceUpdater] Separate functions for update ref forces / ext moment.
    * [rtc/ReferenceForceUpdater] Add update for ext moment around the foot origin pos.
    * [idl/ReferenceForceUpdaterService.idl, rtc/ReferenceForceUpdater] Add is_hold_value to choose holding current value or not. If is_hold_value is true, do not update value, otherwise, update value.
    * [rtc/ReferenceForceUpdater] Add sensor name for eet param.
    * [rtc/ReferenceForceUpdater] Enable to set some RFU parameters while active. Separate parameters which can be changed or not while active.
    * [rtc/Stabilizer] Add calculation of ext moment around the foot origin position for fullbody manipulation.
    * [rt/Stabilizer] Update for st target values considering foot origin coords

* ReferenceForceUpdater : Fix bug of get/set value (`#1173 <https://github.com/fkanehiro/hrpsys-base/issues/1173>`_)
  * [rtc/ReferenceForceUpdater/ReferenceForceUpdater.cpp] Fix bug in get value for is_hold_value flag
  * [sample/SampleRobot/samplerobot_reference_force_updater.py] Add check for value setting and add comments

* Stabilizer
  * Support n-sided polygon (`#1189 <https://github.com/fkanehiro/hrpsys-base/issues/1189>`_)
    * [idl/StabilizerService.idl] add comment about the order of eefm_support_polygon_vertices_sequence
    * [Stabilizer.cpp, Stabilizer.cpp] add vertices with margin for fall detection
    * [ZMPDistributor.h] remove unused function
    * [ZMPDistributor.h] fix bug on calcProjectedPoint
    * [Stabilizer.*, ZMPDistributor.h] calculate convex hull from n-sided polygon's vertices
  * Fix bug of initialization (`#1190 <https://github.com/fkanehiro/hrpsys-base/issues/1190>`_)
    * [rtc/Stabilizer/Stabilizer.cpp] Fix bug of support_time. Initialize support_time and limit max support_time (too large support_time). Add debug print.
  * Support ZMP truncate (`#1158 <https://github.com/fkanehiro/hrpsys-base/issues/1158>`_)
    * [idl/StabilizerService.idl, rtc/Stabilizer/Stabilizer.cpp,Stabilizer.h,ZMPDistributor.h] add option of zmp truncation
    * [rtc/Stabilizer/ZMPDistributor.h] Add func to calculate nearest point of foot support polygon

* AutoBalancer
  * Update ref force balancing (`#1188 <https://github.com/fkanehiro/hrpsys-base/issues/1188>`_)
    * [idl/AutoBalancerService.idl,python/hrpsys_config.py,rtc/AutoBalancer,rtc/ReferenceForceUpdater] Support walking while MODE_REF_FORCE_RFU_EXT_MOMENT by using is_hold_value flag and move base offset
    * [idl/AutoBalancerService.idl, rtc/AutoBalancer] Add parameters to specify link name and offset point for additional force, such as torso.
  * Update footstep limitation (`#1185 <https://github.com/fkanehiro/hrpsys-base/issues/1185>`_)
    * [rtc/AutoBalancer/GaitGenerator.h, idl/AutoBalancerService.idl] Update documentations. Add new explanations and fix incorrect explanations.
    * [rtc/AutoBalancer/testGaitGenerator.cpp] Add sample functions for stride limitation
    * [rtc/AutoBalancer/GaitGenerator.cpp] Use inside limitation in footstep calculation.
    * [idl/AutoBalancerService.idl, rtc/AutoBalancer] Enable to set stride_inside_y and stride_inside_th as stride_parameter
    * [rtc/AutoBalancer/AutoBalancer.cpp, Gaitgenerator] Rename stride_y and stride_theta -> stride_outside_y and stride_outside_theta
  * Fix bug of limiting (`#1162 <https://github.com/fkanehiro/hrpsys-base/issues/1162>`_)
    * [rtc/AutoBalancer/GaitGenerator, sample/SampleRobot/samplerobot_auto_balancer.py] Fix foot step limitation in case of offset vel is specified. Fix testing code.
  * Fix bug of initialization (`#1157 <https://github.com/fkanehiro/hrpsys-base/issues/1157>`_)
    * [rtc/AutoBalancer/GaitGenerator.h] set state at first double support phase and last double support phase to modify bug of no double support walk without reducing swing count.
  * Fix bug of GaitGenerator memory access (`#1159 <https://github.com/fkanehiro/hrpsys-base/issues/1159>`_)
    * [rtc/AutoBalancer/GaitGenerator] Fix invalid access for step_count_list (https://github.com/fkanehiro/hrpsys-base/issues/1154)
    * [rtc/AutoBalancer/GaitGenerator] Remove unused argument in update_refzmp.
  * Separate simplefullbodyiksolver (`#1150 <https://github.com/fkanehiro/hrpsys-base/issues/1150>`_)
    * move operator<< overload into .cpp
    * separate into SimpleFullbodyInverseKinematicsSolver.h
  * Support footstep modification in goVelocity (`#1147 <https://github.com/fkanehiro/hrpsys-base/issues/1147>`_)
    * [GaitGenerator.*] support goVelocity in modifying footsteps
    * [GaitGenerator.cpp] remove unused variable
  * Update preview queue in GaitGenerator (`#1140 <https://github.com/fkanehiro/hrpsys-base/issues/1140>`_)
    * [GaitGenerator.cpp] overwrite refzmp queue only when is_emergency_walking
    * [GaitGenerator.cpp, PreviewController.h] overwrite preview queue without clearing queue
  * Update testing and plotting of GaitGenerator (`#1143 <https://github.com/fkanehiro/hrpsys-base/issues/1143>`_)
    * [rtc/AutoBalancer/testGaitGenerator.cpp] Update comments and print message. Add TODO memo.
    * [rtc/AutoBalancer/testGaitGenerator.cpp, GaitGenerator.*] Add check for toe heel angle and zmp offset transigion. Use eps_eq instead of calling fabs directly.
    * [rtc/AutoBalancer/CMakeLists.txt, GaitGenerator.h, testGaitGenerator.cpp] Add check for step times.
    * [rtc/AutoBalancer/testGaitGenerator.cpp] Add error checker class and add start/end error check between COGxy and REFZMPxy
    * [rtc/AutoBalancer/CMakeLists.txt] Enable test6 checking in make test
    * [rtc/AutoBalancer/testGaitGenerator.cpp] Update comment and use fflush to confirm file writing.
    * [rtc/AutoBalancer/testGaitGenerator.cpp] Add value discontinuous checking for state values.
    * [rtc/AutoBalancer/testGaitGenerator.cpp] Update test for single step and z change (test6, test5) to remove discontinuous results.
    * [rtc/AutoBalancer/testGaitGenerator.cpp] Fix empty range of gnuplotting (force set range).
    * [rtc/AutoBalancer/testGaitGenerator.cpp] Add test_doc_string, add test name to file name, and use sleep for use-graph-append.
  * Fix bug of discontinuous trajectory (`#1139 <https://github.com/fkanehiro/hrpsys-base/issues/1139>`_)
    * [rtc/AutoBalancer/testGaitGenerator.cpp] Update comments and documentation. Add appepnding of graphs.
    * [rtc/AutoBalancer/CMakeLists.txt] Call tests for test17 and test18 in 'make test' (govelocity tests)
    * [rtc/AutoBalancer/GaitGenerator.*] Update rot eps for mid_coords functions.
    * [rtc/ImpedanceController/RatsMatrix.*] Add eps argument for mid_rot and mid_coords
    * [rtc/AutoBalancer/testGaitGenerator.cpp] Increase velocity plotting
    * [rtc/AutoBalancer/GaitGenerator.*] Enable to update the target value of foot midcoords (for example, foot step modification for updown and rotation). This bug is originally reported by @kyawawa.
    * [rtc/AutoBalancer/GaitGenerator.cpp] Interpolate difference from src to dst in swing_foot_rot_interpolator
    * [rtc/AutoBalancer/GaitGenerator.h] Calculate dst and src swing coordinates midcoords for all limbs at first.
    * [rtc/AutoBalancer/testGaitGenerator.cpp] Add plot for swing support mid coords
    * [rtc/AutoBalancer/testGaitGenerator.cpp] Separate plot files. This will not change behavior.
    * [rtc/AutoBalancer/testGaitGenerator.cpp] Add argument for default-double-support-ratio-swing-[after/before]
    * [rtc/AutoBalancer/testGaitGenerator.cpp] Add rotation velocity change for test18 govelocity test
    * [rtc/AutoBalancer/GaitGenerator.*] Enable to update the target value of swing foot rot (for example, foot step modification for rotation)
    * [rtc/AutoBalancer/testGaitGenerator.cpp] Add test for changing govelocity velocities
    * [rtc/AutoBalancer/testGaitGenerator.cpp] Add test for goVelocity mode.
    * [rtc/AutoBalancer/testGaitGenerator.cpp] Separate plot and walk pattern generation parts as functions. This will not change the program behavior.


* Contributors: Fumio KANEHIRO, Iori Kumagai, Isaac I.Y. Saito, Juntaro Tamura, Kei Okada, Nobuhiko Miyamoto, Noriaki Takasugi, Shunichi Nozawa, Yuta Kojio, Yasuhiro Ishiguro, Yisha

315.13.0 (2017-05-19)
---------------------

Stable RTCs
=============

* Update for beeping of SoftErrorLimiter and CollisionDetector (`#1124 <https://github.com/fkanehiro/hrpsys-base/issues/1124>`_)
  * [rtc/SoftErrorLimiter/SoftErrorLimiter.cpp,rtc/CollisionDetector/CollisionDetector.cpp] Call quit_beep when is_beep_port_connected is true, because beeping is not required in these RTCs and deregated to Beeper RTC in this case.

* Update documentations of python interfaces for stable rtcs (`#1120 <https://github.com/fkanehiro/hrpsys-base/issues/1120>`_)
  * [doc][py] Explain setWrenches method (thanks to https://github.com/fkanehiro/hrpsys-base/issues/1131).
    Similar info should be added to IDL level (this Python file is only the end-user API so the number of audience can be minimum).
  * [py][doc] Add how to set ref\_{force, moment} for 315.2.0 <= version.
  * [doc][py] More elaboration on startImpedance.
  * [py] Elaborate the "tm" arg.
   AFAI understood from looking at [SequencePlayer/SequencePlayer.cpp](https://github.com/fkanehiro/hrpsys-base/blob/350a3bcdeafd39d82a44c2dbde6a279adb6d88eb/rtc/SequencePlayer/SequencePlayer.cpp#L547)'s implementation, time taken for planning and other processes are not considered for "`tm`" argument.
  * [doc][py] Elaborate playPattern* methods.

* hrpsys-simulator
  * set default background color of hrpsys-viewer

* Update CMake for VTK (`#1123 <https://github.com/fkanehiro/hrpsys-base/issues/1123>`_)
  * Precise the version of VTK
    There seems to be an incompatibility between the version 5.8 and the version 7

Unstable RTCs
=============
* Update Unstable RTC sample, readme, and images (`#1126 <https://github.com/fkanehiro/hrpsys-base/issues/1126>`_, `#1127 <https://github.com/fkanehiro/hrpsys-base/issues/1127>`_)
  * [sample/SampleRobot/README.md] Fix path of images of README
  * [sample/SampleRobot/README.md, img] Add image and update link for hrpsys sample README
  * [sample/SampleRobot/samplerobot_terrain_walk.py] Update terrain walk samples
  * [sample/SampleRobot/SampleRobot.DRCTestbed.xml] Update DRC testbed xml to add floor.
  * [sample/README.md, SampleRobot/README.md] Update README to fix indent and alignment.

* Update Stabilizer (`#1125 <https://github.com/fkanehiro/hrpsys-base/issues/1125>`_, `#1128 <https://github.com/fkanehiro/hrpsys-base/issues/1128>`_, `#1137 <https://github.com/fkanehiro/hrpsys-base/issues/1137>`_)
  * Update Stabilizer-related codes and samples (`#1125 <https://github.com/fkanehiro/hrpsys-base/issues/1125>`_)
    * [sample/SampleRobot/samplerobot_stabilizer.py] Add new sample for st for root rot change and mimic rough terrain.
    * [rtc/PDcontroller/PDcontroller.cpp] Print PD gain when loading
    * [rtc/Stabilizer/Stabilizer.*] Remove unused parameters.
  * Update damping control in horizontal direction (`#1128 <https://github.com/fkanehiro/hrpsys-base/issues/1128>`_)
    * [Stabilizer] Enable to change horizontal swing damping gain when detect large force/moment
  * IK loop and base rot control (`#1137 <https://github.com/fkanehiro/hrpsys-base/issues/1137>`_)
    * [rtc/Stabilizer/Stabilizer.* idl/StabilizerService.idl] Enable to set ik_loop_count in st
    * [rtc/Stabilizer/ZMPDistributor.h, Stabilizer.cpp] Reduce lines of printing of stabilizer.
    * [idl/StabilizerService.idl, rtc/Stabilizer] Enable to set Limit of compensation for difference between ref-act root rot [rad].

* Add new functionality for walking (`#1129 <https://github.com/fkanehiro/hrpsys-base/issues/1129>`_, `#1134 <https://github.com/fkanehiro/hrpsys-base/issues/1134>`_)
  * Modify footsteps to keep balance (`#1129 <https://github.com/fkanehiro/hrpsys-base/issues/1129>`_)
    * [AutoBalancerService.idl, StabilizerService.idl, hrpsys_config.py, AutoBalancer.*, AutoBalancer.*, Stabilizer.*] modify footsteps based on CP
    * [AutoBalancerService.idl, AutoBalancer.cpp, GaitGenerator.*, sample/SampleRobot/samplerobot_auto_balancer.py] use different parameter when generating footsteps with circle type
  * Change base height to avoid from leg stretching (`#1134 <https://github.com/fkanehiro/hrpsys-base/issues/1134>`_)
    * [AutoBalancerService.idl, AutoBalancer.*] avoid limb stretch by changing root height
    * [Stabilizer.cpp] smoothly change root height when switching use_limb_stretch_avoidance

* Update python function to start all default controllers (`#1130 <https://github.com/fkanehiro/hrpsys-base/issues/1130>`_)
  * [python/hrpsys_config.py] Support non-legged robot and leg-only robot in startDefaultUnstableControllers and stopDefaultUnstableControllers

* Add new functionality to remove force sensor offset (`#1132 <https://github.com/fkanehiro/hrpsys-base/issues/1132>`_, `#1133 <https://github.com/fkanehiro/hrpsys-base/issues/1133>`_)
  * [idl/RemoveForceSensorLinkOffsetService.idl, rtc/RemoveForceSensorLinkOffset, python/hrpsys_config.py, sample/SampleRobot/samplerobot_remove_force_offset.py] Add argument for duration of calibration. Set 8.0[s] as default value in python code
  * [python/hrpsys_config.py,sample/SampleRobot/samplerobot_remove_force_offset.py] Add sample and python interface for removing force sensor offset.
  * [idl/RemoveForceSensorLinkOffsetService, rtc/RemoveForceSensorLinkOffset] Add function to remove force sensor offset in RMFO RTC.
  * [rtc/RemoveForceSensorLinkOffset/RemoveForceSensorLinkOffset.txt] Add documentation abount two types of offsets.

* Contributors: Fumio KANEHIRO, Isaac I.Y. Saito, Mehdi Benallegue, Shunichi Nozawa, Tatsuya Ishikawa, YutaKojio

315.12.1 (2017-04-04)
---------------------

Stable RTCs
=============

* Fix bug of https://github.com/fkanehiro/hrpsys-base/pull/1103 (`#1114 <https://github.com/fkanehiro/hrpsys-base/issues/1114>`_)
  * [CMakeLists.txt] use separate_arguments to set semicolon to variable.

Unstable RTCs
=============

* Update for 3rdparty (`#1115 <https://github.com/fkanehiro/hrpsys-base/issues/1115>`_)
  * [3rdparty/qpOASES/CMakeLists.txt] trust server of QPOASES

* Add ApproximateVoxelGridFilter (`#1117 <https://github.com/fkanehiro/hrpsys-base/issues/1117>`_)
  * replace VoxelGridFilter with ApproximateVoxelGridFilter
  * add a new RT component, ApproximateVoxelGridFilter

* Update for ApproximateVoxelGridFilter
  * copy timestamp from input cloud to output cloud
  * add debug mode and pass through mode

* Add PointCloudViewer (`#1116 <https://github.com/fkanehiro/hrpsys-base/issues/1116>`_)
  * add a new RT component, PointCloudViewer

* Update for VoxelGridFilter
  * copy is_dense attribute from input point cloud
  * add debugLevel property and support pass-through mode

* Update Stabilizer in-the-air behavior (`#1090 <https://github.com/fkanehiro/hrpsys-base/issues/1090>`_)
  * [Stabilizer] Set default detection time whether robot is in air to zero
  * [Stabilizer] Set detection time whether robot is in air
  * [Stabilizer] Rename variables

* Rename and separate functions (`#1111 <https://github.com/fkanehiro/hrpsys-base/issues/1111>`_)
  * Stabilizer
    * [rtc/Stabilizer/Stabilizer.*] Rename contact_states -> ref_contact_states. Rename isContact -> act_contact_states
  * AutoBalancer
    * [rtc/AutoBalancer/AutoBalancer.*] Separate solving IK codes as class. Currently, transition and SBP calculation is not separated. (This is expected not to change behavior).
    * [rtc/AutoBalancer/AutoBalancer.cpp] Output and set OutPort only for legged robots.
    * [rtc/AutoBalancer/AutoBalancer.*] Make startWalking without ABC deprecated and fix position of calling of stopWalking and zmp_weight_mzp interpolation.
    * [rtc/AutoBalancer/AutoBalancer.*] Remove unnecessary member vaiable (target_end_coords)
    * [rtc/AutoBalancer/AutoBalancer*] Separate codes from IK parts. (This commit is expected not to change the behavior)
    * [rtc/AutoBalancer/AutoBalancer.*] Include target EE coords calculation in separated functions. (This commit is expected not to change the behavior).
    * [rtc/AutoBalancer/AutoBalancer.*] Separate procedures in getTargetParameters as fucntions. (This commit is expected not to change the behavior).
    * [rtc/AutoBalancer/AutoBalancer.*] Fix order of transition code. Separate functions to calculate output from ABC. (This commit is expected not to change the behavior).
    * [rtc/AutoBalancer/AutoBalancer.cpp] Fix parameter setting section in non-GaitGenerator parts. Fix index of default_zmp_offsets usage.
    * [rtc/AutoBalancer/AutoBalancer, GaitGenerator] Separate and move codes from AutoBalancer to GaitGenrator, in order to get GaitGenerator results in AutoBalancer. (This commit is expected not to change the behavior)
    * [rtc/AutoBalancer/AutoBalancer.cpp] Use target EE pos and rot instead of target link origin pos and rot from SequencePlayer target. (This commit is expected not to change the behavior)
  * ImpedanceController
    * [rtc/ImpedanceController/ImpedanceController.*] Separate functions, add comments, and fix order and calculation of calcForceMoment. (This commit is expected not to change the behavior)
  * Update samples for unstable RTCs
    * [launch/sample4legrobot.launch,sample6dofrobot.launch,samplespecialjointrobot.launch] Update conf_file setting for launch samples
    * [sample/SampleRobot/samplerobot_impedance_controller.py] Separate tests as functions and improve execution time and result checking.

* AutoBalancer update by adding inverse dynamics function (`#1110 <https://github.com/fkanehiro/hrpsys-base/issues/1110>`_)
  * refactor IIRFilter::setParameterAsBiquadButterworth -> IIRFilter::setParameterAsBiquad
  * sqrt -> std::sqrt one more
  * sqrt -> std::sqrt
  * seperate InverseDynamics calculation sequence
  * tan -> std::tan
  * Revert "remove unused dependency from AutoBalancer/CMakeLists.txt"
    This reverts commit 933edb0a1d05b7104e22eccc96f5639cedd607b4.
  * remove unused dependency from AutoBalancer/CMakeLists.txt
  * Revert "move InverseDynamics from ABC to IC/JointPathEx"
    This reverts commit 9b2a76ea3d6e0f871b4460002ded668649711a19.
  * move InverseDynamics from ABC to IC/JointPathEx
  * move InverseDynamics from ABC to IC/JointPathEx
  * merge from master
  * Add inverse dynamics function

* Contributors: Fumio KANEHIRO, Masaki Murooka, Ryo KOYAMA, Shunichi Nozawa, Tatsuya Ishikawa, Yasuhiro Ishiguro

315.12.0 (2017-03-08)
---------------------

Stable RTCs
=============

* Do not store history of gh-pages
  * Do not store history of gh-pages (`#1107 <https://github.com/fkanehiro/hrpsys-base/issues/1107>`_)
    * .travis.yml : do not keep history in gh-pages
  * Fix git push for gh-pages (`#1108 <https://github.com/fkanehiro/hrpsys-base/issues/1108>`_)
    * fix `#1107 <https://github.com/fkanehiro/hrpsys-base/issues/1107>`_: need to push with -f

* Fix bug of cmake for latest catkin (`#1103 <https://github.com/fkanehiro/hrpsys-base/issues/1103>`_)
  * fix https://github.com/start-jsk/rtmros_common/pull/998, for latest catkin

* Update maintainer and docs (`#1100 <https://github.com/fkanehiro/hrpsys-base/issues/1100>`_)
  * Update maintainer
  * [doc] Clarify workspace for get*Pose methods. [doc] Add missing rtype for setJoint* methods.


Unstable RTCs
=============

* CameraImageViewer OpenNIGrabber
  * Update for depth image(`#1106 <https://github.com/fkanehiro/hrpsys-base/issues/1106>`_)
    * resolve conflicts
    * enable output both of image and point cloud
    * update document
    * add a configuration variable depthBits to CameraImageViewer
    * support depth image (not tested yet)
    * support CF_DEPTH (not tested yet)
    * add CF_RGB to ColorFormat
  * Update for OpenNIGrabber (`#1096 <https://github.com/fkanehiro/hrpsys-base/issues/1096>`_)
    * flip Y and Z coordinates
    * enable to change execution period
    * set timestamps to output data
    * set true to is_dense

* Stabilizer
  * Bug fix of transition among stop, start, ground, and air mode (`#1104 <https://github.com/fkanehiro/hrpsys-base/issues/1104>`_)
    * [rtc/Stabilizer/Stabilizer.cpp] Fix bug of st transition related with air and ground (https://github.com/fkanehiro/hrpsys-base/issues/1098, https://github.com/fkanehiro/hrpsys-base/pull/1102)
    * [sample/SampleRobot/samplerobot_stabilizer.py] Add test code to check ST transition problem (https://github.com/fkanehiro/hrpsys-base/issues/1098, https://github.com/fkanehiro/hrpsys-base/pull/1102).
  * Bug fix of stop/start transition (`#1102 <https://github.com/fkanehiro/hrpsys-base/issues/1102>`_)
    * [rtc/Stabilizer/Stabilizer.cpp] Reset prev_ref_cog for transition (MODE_IDLE=>MODE_ST) because the coordinates for ref_cog differs among st algorithms.
    * [rtc/Stabilizer/Stabilizer.cpp] Wait for MODE transition regardless of whether the client program is mode changer or not (https://github.com/fkanehiro/hrpsys-base/issues/1098)
  * Update for st control low (`#1099 <https://github.com/fkanehiro/hrpsys-base/issues/1099>`_)
    * [rtc/Stabilizer/Stabilizer.*] Enable to update is_feedback_control_enable flag while MODE_ST
    * [rtc/Stabilizer/ZMPDistributor.h] Use 6dof total wrench in ZMPdistribution in EEFMQPCOP2

* ObjectContactTurnaroundDetector (Add new RTC separated from ImpedanceController) (`#1101 <https://github.com/fkanehiro/hrpsys-base/issues/1101>`_)
  * [idl/ImpedanceControllerService.idl,rtc/ImpedanceController] Remove ObjectContactTurnaroundDetector from ImpedanceController.
  * [sample/SampleRobot/samplerobot_carry_object.py, samplerobot_impedancecontroller.py] Use OCTD RTC instead of ImpedanceController
  * [python/hrpsys_config.py,launch/samplerobot.launch,doc] Update hrpsyspy, launch, and doc for ObjectContactTurnaroundDetector RTC
  * [rtc/CMakeLists.txt,idl/CMakeLists.txt,rtc/ObjectContactTurnaroundDetector,idl/ObjectContactTurnaroundDetectorService.idl] Add ObjectContactTurnaroundDetector RTC for object manipulation separated from ImpedanceController

* TorqueFilter/IIRFilter (`#1097 <https://github.com/fkanehiro/hrpsys-base/issues/1097>`_)
  * [rtc/TorqueFilter] Add comments for IIR Filter implementation
  * [rtc/TorqueFilter] Fix reset function to output initial_value

* ImpedanceController (`#1099 <https://github.com/fkanehiro/hrpsys-base/issues/1099>`_)
  * [rtc/ImpedanceController/ImpedanceController.cpp,h,ObjectTurnaroundDetector.h] Add port for otd data (mode, wrench values)
  * [rtc/ImpedanceController/ImpedanceController.cpp] Add calcForwardKinematics for state calculation.

* AutoBalancer (`#1099 <https://github.com/fkanehiro/hrpsys-base/issues/1099>`_)
  * [rtc/AutoBalancer/AutoBalancer.cpp] Transition m_limbCOPOffset in ABC (in MODE_IDLE, set to 0).
  * [rtc/AutoBalancer/AutoBalancer.cpp] Use setGoal instead of go for autobalancer transition.

* KalmanFilter (`#1099 <https://github.com/fkanehiro/hrpsys-base/issues/1099>`_)
  * [rtc/KalmanFilter/CMakeLists.txt, testKFilter.cpp] Add test code for KFilter class.
  * [rtc/KalmanFilter/KalmanFilter.cpp] Consider sensor offset for gyro value.

* Contributors: Fumio KANEHIRO, Isaac I.Y. Saito, Kei Okada, Shunichi Nozawa, Iori Kumagai

315.11.0 (2017-02-17)
---------------------

Stable RTCs
=============

* supports OpenRTM-aist 1.2.0 (`#1076 <https://github.com/fkanehiro/hrpsys-base/issues/1076>`_ from fkanehiro/openrtm_1.2.0)

* add ctype and time to Hrpsys.h for QNX (`#1085 <https://github.com/fkanehiro/hrpsys-base/issues/1085>`_)

  * add include time.h in lib/util/Hrpsys.h for QNX
  * include ctype.h in lib/util/Hrpsys.h for QNX

* Documentation

  * idl/SequencePlayerService.idl: add suppored version in @brief (`#1092 <https://github.com/fkanehiro/hrpsys-base/issues/1092>`_)
  * idl/SequencePlayerService.idl: Fix documentation of setJointAngleSequenceFull (fix typoes and unit systems). (`#1067 <https://github.com/fkanehiro/hrpsys-base/issues/1067>`_)

* DataLogger

  * rtc/DataLogger/DataLogger.cpp: add log_precision (configuration variable)  (`#1081 <https://github.com/fkanehiro/hrpsys-base/issues/1081>`_)

* RobotHardware

  * [rtc/RobotHardware/robot.cpp] Fix bugs in setServoGainPercentage condition (`#1074 <https://github.com/fkanehiro/hrpsys-base/issues/1074>`_)

* Add beep trylock, print PDgain, print time in EC debug (`#1060 <https://github.com/fkanehiro/hrpsys-base/issues/1060>`_)

  * [rtc/RobotHardware/robot.cpp] Print loaded PD gain.
  * [ec/hrpEC/hrpEC-common.cpp] Print gettimeofday time in ENABLE_DEBUG_PRINT
  * [rtc/Beeper/Beeper.cpp] Use mutex trylock not to block realtime thread

* lib/util/PortHandler.cpp : add VisionSensorPortHandler intrinsic paramter to TimedCamera (`#1091 <https://github.com/fkanehiro/hrpsys-base/issues/1091>`_ from k-okada/add_camera_info)

* [rtm.py] try to re-connecting to name server, when failed (`#1083 <https://github.com/fkanehiro/hrpsys-base/issues/1083>`_)
* [hrpsys_config.py] Add clearJointAngles* methods. (`#1064 <https://github.com/fkanehiro/hrpsys-base/issues/1064>`_)
* [hrpsys_config.py] set HrpsysConfigurator as object class (`#1048 <https://github.com/fkanehiro/hrpsys-base/issues/1048>`_)

Unstable RTCs
=============

* fix time stamp of warning messages ( `#1089 <https://github.com/fkanehiro/hrpsys-base/issues/1089>`_)

  * [Stabilizer] reduce number of print messages
  * [AutoBalancer, CollisionDetector, EmergencyStopper, SoftErrorLimitter, Stabilizer, ReferenceForceUpdater] fix time stamp of warning messages
* [AutoBalancer, CollisionDetector, EmergencyStopper, SoftErrorLimitter, Stabilizer] add time stamp to warning messages (`#1087 <https://github.com/fkanehiro/hrpsys-base/issues/1087>`_)

* PCDLoader

  * rtc/PCDLoader/PCDLoader.cpp: support XYZRGB in PCDLoader (`#1093 <https://github.com/fkanehiro/hrpsys-base/issues/1093>`_)

* Stabilizer

  * Fix st damping compensation bug. Enable yaw rotation. (`#1082 <https://github.com/fkanehiro/hrpsys-base/issues/1082>`_)

    * [rtc/Stabilizer/Stabilizer.cpp] Fix st damping compensation bug. Enable yaw rotation.

  * Update st debug messages ( `#1075 <https://github.com/fkanehiro/hrpsys-base/issues/1075>`_)

    * [sample/SampleRobot/samplerobot_stabilizer.py] Add printing of actual base pos during st testing.
    * [rtc/Stabilizer/ZMPDistributor.h, Stabilizer.cpp] Reduce lines of st debug message in setParameter (total information is same).

  * Change root link height depending on limb length (`#1069 <https://github.com/fkanehiro/hrpsys-base/issues/1069>`_)

    * [StabilizerService.idl, Stabilizer.*] rename variable and add function for avoiding limb stretch
    * [Stabilizer.cpp] change root link height on world frame not root link frame
    * [StabilizerService.idl, Stabilizer.*] add idl for changing root link height
    * [Stabilizer.*] change root link height depending on limb length

  * do not distribute moment to swing leg (`#1068 <https://github.com/fkanehiro/hrpsys-base/issues/1068>`_)

    * [Stabilizer.cpp, ZMPDistributor.h] do not distribute moment to swing leg

  * [AutoBalancer, Stabilizer] add warning message (`#1066 <https://github.com/fkanehiro/hrpsys-base/issues/1066>`_)
  * Update st swing modif (`#1062 <https://github.com/fkanehiro/hrpsys-base/issues/1062>`_)

    * [rtc/Stabilizer/CMakeLists.txt] Fix build dependency not to depends on Stabilizer codes.
    * [rtc/Stabilizer/Stabilizer.cpp] Use Eigen cwise functions for swing ee modif calculation.
    * [rtc/Stabilizer/Stabilizer.cpp,h] Move vectors for swing ee modif to stikp.
    * [rtc/Stabilizer/Stabilizer.cpp] Enable retrieving of swing ee modif during support phase.
    * [rtc/Stabilizer/Stabilizer.cpp,h] Separate swing foot modif function and re-organize calcEEForceMomentControl
    * [rtc/Stabilizer/Stabilizer.cpp,h] Update swing ee modification to use current state.

  * Initialize st eefm_ee_forcemoment_distribution_weight parameter. (`#1059 <https://github.com/fkanehiro/hrpsys-base/issues/1059>`_)

    * [rtc/Stabilizer/Stabilizer.cpp] Initialize st eefm_ee_forcemoment_distribution_weight parameter.

  * Use damping control in swing phase (`#1058 <https://github.com/fkanehiro/hrpsys-base/issues/1058>`_)

    * [StabilizerService.idl, Stabilizer.*] use swing damping only when exceeding threthold
    * [StabilizerService.idl, Stabilizer.*] use damping control in swing phase

* AutoBalancer

  * Add and update test for sync sh baseTform (`#1065 <https://github.com/fkanehiro/hrpsys-base/issues/1065>`_)

    * [rtc/AutoBalancer/AutoBalancer.cpp] Set fix_leg_coords according to basePos and rpy from StateHolder during MODE_IDLE.
    * [sample/SampleRobot/samplerobot_auto_balancer.py] Add and update test for sync sh baseTform

  * Enable auto toeheel (`#1056 <https://github.com/fkanehiro/hrpsys-base/issues/1056>`_)

    * [idl/AutoBalancerService.idl, rtc/AutoBalancer/AutoBalancer.cpp] Enable to set use_toe_heel_auto_set, toe,heel_check_thre from idl.
    * [rtc/AutoBalancer/*GaitGenerator.*] Fix typo (double -> bool) and add flag whether use auto toe heel set or not. Update test to use use_toe_heel_auto_set flag.
    * [rtc/AutoBalancer/GaitGenerator.h] Add toe_heel_type_checker to determin autonomusly whether toe or heel is used.
    * [rtc/AutoBalancer/GaitGenerator.*] Separate calc swing support leg steps as functions
    * [rtc/AutoBalancer/GaitGenerator.cpp] Use first_xx and second_xx instead of toe_xx and heel_xx. Renaming and fix transition.

  * Update toeheel and st (`#1054 <https://github.com/fkanehiro/hrpsys-base/issues/1054>`_)

    * [rtc/Stabilizer/testZMPDistributor.cpp, CMakeLists.txt] Update testZMPDistributor to switch distribute algorithm and add cmake test for them.
    * [sample/SampleRobot/samplerobot_stabilizer.py] Add test for st+toe heel usage
    * [python/hrpsys_config.py, rtc/Autobalancer, rtc/Stabilizer] Add and connect ports to send toe heel ratio between AutoBalancer and Stabilizer. Add weighting matrix update based on toe heel ratio.
    * [rtc/AutoBalancer/testGaitGenerator.cpp, GaitGenerator.h] Add functions to obtain toe heel ratio and add test for it. Update comment in codes.

  * Fix toe heel phase count of refzmp_generator (`#1053 <https://github.com/fkanehiro/hrpsys-base/issues/1053>`_)

    * [rtc/AutoBalancer/CMakeLists.txt] Enable cmake test for testGaitGenerator test16
    * [rtc/AutoBalancer/GaitGenerator.*] Fix bug of toe heel phase. Make independent toe heel phase for leg_coords_generator and refzmp_generator. Use thp instead of thp_ptr->.
    * [rtc/AutoBalancer/testGaitGenerator.cpp] Update zmp diff check thre (10mm->20mm). Update test16. Update step time. Add default zmp offsets.
    * [rtc/AutoBalancer/GaitGenerator.h] Use NUM_TH_PHASES directly.

  * Update abc posik ggtest (`#1052 <https://github.com/fkanehiro/hrpsys-base/issues/1052>`_)

    * [rtc/AutoBalancer/testGaitGenerator.cpp, CMakeLists.txt] Add test for setFootStepWithParam related functions. CMake test is disabled currently.
    * [rtc/AutoBalancer/AutoBalancer.cpp] Update default value for pos_ik_thre according to hrp2 reset manip pose. 0.1[mm] -> 0.5 [mm].

* sample

  * Update SampleRobot xml project file (`#1086 <https://github.com/fkanehiro/hrpsys-base/issues/1086>`_)

    * [sample/SampleRobot/samplerobot_auto_balancer.py] Omit demoStandingPosResetting because we use floor5 and we do not need to worry about falling down from the floor.
    * [sample/SampleRobot/samplerobot_auto_balancer.py] Add print message for all autobalancer sample time.
    * [sample/SampleRobot/README.md,SampleRobot.*.xml.in] Use floor5 instead of longfloor
    * [sample/SampleRobot/SampleRobot*.xml.in] Update to sample's xml.in files using latest openhrp-project-generator

  * [sample/SampleRobot/README.md] Add readme to generate sample's xml.in files
  * Update st tests (`#1061 <https://github.com/fkanehiro/hrpsys-base/issues/1061>`_)

    * [sample/SampleRobot/README.md] Add command documentation to generate xml
    * [rtc/AutoBalancer/GaitGenerator.h] Add toe usage for down-stair walking.
    * [sample/SampleRobot/samplerobot_stabilizer.py] Update st tests. Add stair test. Re-organize all tests. Use swing damping mode.
    * [sample/SampleRobot/SampleRobot.torque.xml.in] Add box for stair walking tests.

* IIRFilter.cpp

  * [rtc/TorqueFilter/IIRFilter.cpp] Fix problem of resetting coefficient value in setParameter (`#1077 <https://github.com/fkanehiro/hrpsys-base/issues/1077>`_)
  * [TorqueFilter, IIRFilter.h] add getParameter and reset method (`#1050 <https://github.com/fkanehiro/hrpsys-base/issues/1050>`_)

* AccelerationFilter

  * add AccelerationFilter (`#1051 <https://github.com/fkanehiro/hrpsys-base/issues/1051>`_)
  * [AccelerationFilter] fix compile AccelerationFilterService.idl
  * [AccelerationFilter] add AccelerationFilter to hrpsys_config.py
  * [AccelerationFilter] add Acceleration Filter

* util/SelfCollisionChecker/main.cpp

  * enables to visualize motion with -olv option (`#1055 <https://github.com/fkanehiro/hrpsys-base/issues/1055>`_)

* JpegDetector/JpegEncoder/RGB2Gray/VideoCapture

  * Add opencv missing header and namespace for OpenCV3 (`#1049 <https://github.com/fkanehiro/hrpsys-base/issues/1049>`_)

* Contributors: Fumio KANEHIRO, Isaac Saito, Kei Okada, Shunichi Nozawa, Tatsuya Ishikawa, Yohei Kakiuchi, Yuta Kojio, Iori Yanokura, Yasuhiro Ishiguro

315.10.1 (2016-10-08)
---------------------

Stable RTCs
=============

* [python/hrpsys_config.py, sample/SampleRobot/*.py] Fix bug of hrpsys version checking. Use StrictVersion for version checking. (`#1044 <https://github.com/fkanehiro/hrpsys-base/issues/1044>`_)

  * Closes https://github.com/tork-a/rtmros_nextage/issues/260

Unstable RTCs
=============

* IIRFilter

  * Add setParameter, passFilter methods (`#1046 <https://github.com/fkanehiro/hrpsys-base/issues/1046>`_)
   * [IIRFilter] remove warning message
   * [IIRFilter] add test code for IIRFilter to testIIRFilter
   * [IIRFilter] fix indent on IIRFilter.h IIRFilter.cpp
   * [IIRFilter] add new method for using IIRFilter

* AutoBalancer

  * [rtc/AutoBalancer/GaitGenerator.cpp] fix bug of emergency stop (`#1045 <https://github.com/fkanehiro/hrpsys-base/issues/1045>`_)

  * Add GaitGenerator Sample codes (`#1043 <https://github.com/fkanehiro/hrpsys-base/issues/1043>`_)

    * [rtc/AutoBalancer/testGaitGenerator.cpp] Add test for stairdown
    * [rtc/AutoBalancer/GaitGenerator.h] Add print message for swing_trajectory_time_offset_xy2z

  * Update swing trajectory xy and z convergence (`#1042 <https://github.com/fkanehiro/hrpsys-base/issues/1042>`_)

    * [rtc/AutoBalancer/testGaitGenerator.cpp] Add --swing-trajectory-time-offset-xy2z option for testGaitGenerator
    * [idl/AutoBalancerService.idl, rtc/AutoBalancer/AutoBalancer.cpp,GaitGenerator.h] Add swing_trajectory_time_offset_xy2z for time between Z convergence time and XY convergence time. 0 by default, which does not change default behavior.
    * [rtc/AutoBalancer/GaitGenerator.h] Separate xy interpolation and z interpolation
    * [rtc/AutoBalancer/GaitGenerator.h] Use int for if check
    * [rtc/AutoBalancer/GaitGenerator.h] Return final distance antecedent path ratio
    * [rtc/AutoBalancer/GaitGenerator.h] Separate calc function and interpolate function for antecedent path
    * [rtc/AutoBalancer/GaitGenerator.h] Define foot hoffarbib_interpolation as double type interpolation
   * [rtc/AutoBalancer/GaitGenerator.h] Define interpolate_antecedent_path functions as const member functions.

* OpenNIGrabber

  * Changes point type for depth_and_color (`#1041 <https://github.com/fkanehiro/hrpsys-base/issues/1041>`_)

* Contributors: Fumio Kanehiro, Shunichi Nozawa, Tatsuya Ishikawa, Yohei Kakiuchi

315.10.0 (2016-09-13)
---------------------

Stable RTCs
=============

* SequencePlayer

  * [rtc/SequencePlayer/timeUtil.cpp] get_tick(): add support for ARM_ARCH_7A and AARCH64EL (`#1018 <https://github.com/fkanehiro/hrpsys-base/issues/1018>`_ )

* RobotHardware

  * [rtc/RobotHardware/robot.*] Check read_power_state flags to servo off. If power OFF is detected while servo ON, servo OFF all and write EMG_POWER_OFF. Do not used by default. (`#1036 <https://github.com/fkanehiro/hrpsys-base/issues/1036>`_)

  * [rtc/RobotHardware/RobotHardware.cpp] checks if model path is given before trying to load(`#1001 <https://github.com/fkanehiro/hrpsys-base/issues/1001>`_)

    * doesn't return RTC_ERROR to pass tests
    * returns RTC_ERROR is the model path is not given instead of calling abort()
    * checks if model path is given before trying to load

  * Use robothardware with simulation(`#995 <https://github.com/fkanehiro/hrpsys-base/issues/995>`_)

    * [RobotHardware] add accessor to m_robot
    * [RobotHardware] base time is taken by a virtual method

* CollisionDetector

  * [CollisionDetector] fix warning message while servo off (`#1023 <https://github.com/fkanehiro/hrpsys-base/issues/1023>`_)

  * get out from initial collision state (`#1015 <https://github.com/fkanehiro/hrpsys-base/issues/1015>`_)

    * [CollisionDetector] update test code for collision detector
    * [CollisionDetector] fix, get out from initial collision state
    * add test for CollisionDetector on initial collision state

  * [CollisionDetector] Fix problem when set collision_loop (`#993 <https://github.com/fkanehiro/hrpsys-base/issues/993>`_, `#990 <https://github.com/fkanehiro/hrpsys-base/issues/990>`_)

    * add version check to test-collision-loop
    * [CollisionDetector] fix behavior when using collision_loop
    * add version check to samplerobot_collision_detector.py
    * add test for collision_loop on CollisionDetector

* fixes warnings detected by -Wsign-compare (`#1039 <https://github.com/fkanehiro/hrpsys-base/issues/1039>`_)
* fixes warnings detected by -Wreorder (`#1038 <https://github.com/fkanehiro/hrpsys-base/issues/1038>`_)
* Support clang (`#1037 <https://github.com/fkanehiro/hrpsys-base/issues/1037>`_)
* [ec/hrpEC/hrpEC-common.cpp] supports trunk version of OpenRTM-aist (https://github.com/fkanehiro/hrpsys-base/commit/efce7d47dc3723c868b66bf6205f93bab99b1537)
* [lib/util, rtc/CollisionDetector, rtc/ServoController, util/monitor] fixes some of compile warnings (`#1007 <https://github.com/fkanehiro/hrpsys-base/issues/1007>`)
* [hrpsys-base.pc] fix version in hrpsys-base.pc(`#1004 <https://github.com/fkanehiro/hrpsys-base/issues/1004>`_)
* [CMakeLists.txt] enables to use RelWithDebInfo (`#997 <https://github.com/fkanehiro/hrpsys-base/issues/997>`_)
* [CMakeLists.txt] check version in CMakeLists.txt against package.xml (`#991 <https://github.com/fkanehiro/hrpsys-base/issues/991>`_)
* [CMakeLists.txt] supports ubuntu16.04 (`#989 <https://github.com/fkanehiro/hrpsys-base/issues/989>`_)

* python

  * [python/rtm.py] checks if component is already activated/deactivated (`#1024 <https://github.com/fkanehiro/hrpsys-base/issues/1024>`_)
  * [python/rtm.py] checks return values of activate_component() and deactivate_component() (`#1022 <https://github.com/fkanehiro/hrpsys-base/issues/1022>`_)
  * [python/hrpsys_config.py] add method for setInterpolationMode (`#1012 <https://github.com/fkanehiro/hrpsys-base/issues/1012>`_)
  * [python/hrpsys_config.py] save ReferenceForceUpdater output with DataLogger (`#1008 <https://github.com/fkanehiro/hrpsys-base/issues/1008>`_)
  * [python/rtm.py] improves efficiency of readDataPort() by returning value of dataport.data_value property if available (`#1000 <https://github.com/fkanehiro/hrpsys-base/issues/1000>`_)
  * [python/rtm.py] enables to specify interface_type (`#998 <https://github.com/fkanehiro/hrpsys-base/issues/998>`_)
  * [python/hrpsys_config.py] Fixed mistake of waitForRTCManagerAndRoboHardware's argument (`#988 <https://github.com/fkanehiro/hrpsys-base/issues/988>`_)
  * [pyrhon/hrpsys_config.py] Renamed waitForRTCManagerAndRoboHardware to waitForRTCManagerAndRobotHardware `#980 <https://github.com/fkanehiro/hrpsys-base/issues/980>`_ (`#984 <https://github.com/fkanehiro/hrpsys-base/issues/984>`_)

* [doc] hrpsys_config.py comment update (`#1016 <https://github.com/fkanehiro/hrpsys-base/issues/1016>`_)

  * [doc for setJointAnglesSequence*] Wrong param type. Better description.
  * [doc] Add in-code comment for the addition `#1012 <https://github.com/fkanehiro/hrpsys-base/issues/1012>`_.

Unstable RTCs
=============

* AutoBalancer

  * [rtc/AutoBalancer/AutoBalancer.cpp,rtc/Stabilizer/Stabilizer.cpp] Modify ref force output from ABC to ST (`#1035 <https://github.com/fkanehiro/hrpsys-base/issues/1035>`_)

  * add new stride limitation type when goPos and goVelocity (`#1031 <https://github.com/fkanehiro/hrpsys-base/issues/1031>`_)

    * [AutoBalancerService.idl, hrpsys_config.py, AutoBalancer.*, GaitGenerator.h, Stabilizer.*] add idl for leg_margin
    * [samplerobot_auto_balancer.py] add test to check goPos when changing stride limitation type to circle
    * [AutoBalancerService.idl, AutoBalancer.cpp, GaitGenerator.*] add new stride limitation type when goPos and goVelocity
    * [GaitGenerator.*] fix region of stride limitation

  * add a function to limit stride (`#1029 <https://github.com/fkanehiro/hrpsys-base/issues/1029>`_)

    * [AutoBalancerService.idl, AutoBalancer.cpp, GaitGenerator.*] limit stride when use_stride_limitation is true
    * [AutoBalancerService.idl, AutoBalancer.cpp, GaitGenerator.*] add function to limit stride
    * [hrpsys_config.py, AutoBalancer.*, GaitGenerator.h, Stabilizer.*] get leg_margin from st

  * [AutoBalancer/PreviewController.h] get preview_control_gain f (`#1027 <https://github.com/fkanehiro/hrpsys-base/issues/1027>`_)

  * [rtc/AutoBalancer/AutoBalancer.{cpp,h}] print limb neame in the case of  too large IK error. (`#1017 <https://github.com/fkanehiro/hrpsys-base/issues/1017>`_)

* Stabilizer

  *  Add new st distribution and add interpolator printing. (`#1013 <https://github.com/fkanehiro/hrpsys-base/issues/1013>`_)

    * [idl/StabilizerService.idl, rtc/Stabilizer/] Add new st_algorithm to check multi contact distribution.
    * [rtc/Stabilizer/Stabilizer.cpp] Update checking of st_algorithm (TPCC or not). This commit should not change behavior.
    * [rtc/SequencePlayer/interpolator.cpp] Add print message for MIN_INTERPOLATION_TIME in interpolator.

  * Update stabilizer sbp_cog_offset and add RobotHardware comment (`#987 <https://github.com/fkanehiro/hrpsys-base/issues/987>`_)

    * [rtc/RobotHardware/robot.cpp] Add print message for removeForceSensorOffset and calibrateInertiaSensor
    * [rtc/Stabilizer/Stabilizer.cpp] Fix orientation of sbp_cog_offset (reference world frame -> foot origin frame).

* ReferenceForceUpdater

  * [rtc/ReferenceForceUpdater/,idl/ReferenceForceUpdaterService.idl] add frame parameter to set move_dir in world coordinates (`#1033 <https://github.com/fkanehiro/hrpsys-base/issues/1033>`_)

  * [rtc/ReferenceForceUpdater] Arrange Reference Force Updater(`#1010 <https://github.com/fkanehiro/hrpsys-base/issues/1010>`_)

    * [ReferenceForceUpdater/ReferenceForceUpdater.cpp] remove unused local variable, base_name_map.
    * [ReferenceForceUpdater/ReferenceForceUpdater.cpp] remove unncessary lines: setting eet.sensor_name.
    * [ReferenceForceUpdater/ReferenceForceUpdater.h] remove unused variable, m_data.
    * [ReferenceForceUpdater/ReferenceForceUpdater.cpp] remove debug print in constructor and deconstructor because oth    er rtc does not have debug print.
    * [ReferenceForceUpdater/ReferenceForceUpdater.cpp] fix indent.

  * [rtc/ReferenceForceUpdater] set time for ReferenceForceUpdater output variable (`#1009 <https://github.com/fkanehiro/hrpsys-base/issues/1009>`_ )

  * support both arm in ReferenceForceUpdater (`#1005 <https://github.com/fkanehiro/hrpsys-base/issues/1005>`_)

    * [rtc/ReferenceForceUpdater/ReferenceForceUpdater.cpp] fix indent
    * [sample/SampleRobot/samplerobot_reference_force_updater.py] update rfu sample to check data port
    * [idl/ReferenceForceUpdaterService.idl] remove arm parameter from ReferenceForceUpdaterParam and add arm arg to interfaces of rfu
      [rtc/ReferenceForceUpdater/ReferenceForceUpdaterService_impl{.h,.cpp}] remove arm parameter from ReferenceForceUpdaterParam
      [rtc/ReferenceForceUpdater/ReferenceForceUpdater.h] add ReferenceForceUpdaterParam structure
      [rtc/ReferenceForceUpdater/ReferenceForceUpdater.cpp] add Initialization for use_sh_base_pos_rpy
      [rtc/ReferenceForceUpdater/ReferenceForceUpdater.cpp] enable to set both arms parameters independently in rfu

  * add sample and test for ReferenceForceUpdater(`#1003 <https://github.com/fkanehiro/hrpsys-base/issues/1003>`_)

    * [test] add test for ReferenceForceUpdater.
    * [sample/SampleRobot] add sample for ReferenceForceUpdater.

* TorqueFilter

  * [rtc/TorqueFilter/IIRFilter.h] Fix type of getCurrentValue (`#1032 <https://github.com/fkanehiro/hrpsys-base/issues/1032>`_)

* Beeper

  * [rtc/Beeper] Update mutex lock and use buffer for communication between beep thread and real-time thread. (`#1030 <https://github.com/fkanehiro/hrpsys-base/issues/1030>`_)

* SelfCollisionChecker

  * adds a tool, hrpsys-self-collision-checker (`#1026 <https://github.com/fkanehiro/hrpsys-base/issues/1026>`_)

* OpenNIGrabber  (`#1021 <https://github.com/fkanehiro/hrpsys-base/issues/1021>`_)

  * checks if OpenNI2 is installed
  * makes error message from OpenNIGrabber more informative
  * adds a configuration variable, mode
  * improves error handling
  * adds a new component, OpenNIGrabber

* Contributors: Fumio Kanehiro, Isaac I.Y. Saito, Jun Inoue, Ryo Koyama, Kei Okada, Masaki Murooka, Noriaki Takasugi, Shunichi Nozawa, Yohei Kakiuchi, Yuta Kojio, Iori Yanokura

315.9.0 (2016-04-19)
--------------------

Stable RTCs
=============

* SequencePlayer

  * fix bug of setJointAnglesSequenceFull function in SequencePlayer (updated by snozawa) (`#908 <https://github.com/fkanehiro/hrpsys-base/issues/908>`_)

    * [sample/SampleRobot/samplerobot_sequence_player.py] Add optionalData check and comment out clear joint angle check currently not working.
    * [rtc/SequencePlayer/seqplay.cpp] Fix typo : RPY->ACC.
    * rtc/SequencePlayer/SequencePlayer.cpp: fix bug of setJointAnglesSequenceFull function in SequencePlayer.
    * rtc/SequencePlayer/SequencePlayerService_impl.cpp: fix the expected length of acc array in setJointAnglesSequenceFull.
    * sample/SampleRobot/samplerobot_sequence_player.py: add test of setJointAnglesSequenceFull in samplerobot_sequence_player.py

  * sample/SampleRobot/samplerobot_sequence_player.py: remove joint group when finishing test. enable clear test of setJointAnglesSequenceFull. (`#914 <https://github.com/fkanehiro/hrpsys-base/issues/914>`_)

* DataLogger

  * Make more explicit the dependence of PointCloudViewer on VTK and on IO package of PCL  (`#65 <https://github.com/fkanehiro/hrpsys-base/issues/968>`_)

* fix include directory for iob.h/idl/util (`#842 <https://github.com/fkanehiro/hrpsys-base/issues/842>`_)
  (io/iob.h -> hrpsys/io/iob.h,
  xxx.hh -> hrpsys/idl/xxx.hh,
  util/xxx.h -> hrpsys/util/xxx.h)

* Update semaphore and EcexutionContext (`#970 <https://github.com/fkanehiro/hrpsys-base/issues/970>`_)

  * [ec/hrpEC/hrpEC-common.cpp, hrpEC.h] Add ordinaly debug message for processing time if ENABLE_DEBUG_PRINT is true. Update print message for processing time. Move QNX ifdef for fprintf to header file.
  * [rtc/RobotHardware/robot.[cpp,h] ,rtc/SequencePlayer/SequencePlayer.[cpp,h], rtc/StateHolder/StateHolder.[cpp,h]] Use semaphore.h instead of interprocess_semaphore because we do not use interprocess_semaphore specific functionality. On old OS, interprocess_semaphore cannot be used (boost version <= 1.35.0). This commit is related with the discussion : https://github.com/fkanehiro/hrpsys-base/issues/969

* rtm.py

  * rtm.py : fix wrong commit on #634 (isConnected() and False: ) (`#978 <https://github.com/fkanehiro/hrpsys-base/issues/978>`_)
  * test/test-hrpsysconf.py: add check do not connect again if already connected for https://github.com/fkanehiro/hrpsys-base/issues/979

* PDController

  * adds a function to interpolate reference angles (`#954 <https://github.com/fkanehiro/hrpsys-base/issues/954>`_)

    * updates description of ref_dt
    * adds a function to interpolate reference angles

* lib/util/VectorConvert
   * fixes a parse problem (`#954 <https://github.com/fkanehiro/hrpsys-base/issues/954>`_)

* Update docs (`#975 <https://github.com/fkanehiro/hrpsys-base/issues/975>`_)

  * [idl/CollisionDetectorService.idl, RobotHardwareService.idl] Update documentation of idl
  * [doc/Doxyfile.in, doc/package.h] Add beeper RTC documentation links.
  * [README.md] Add documentation for directories, papers, and ros wiki

* Fix include dir for QNX build (`#971 <https://github.com/fkanehiro/hrpsys-base/issues/971>`_)

  * [rtc/ImpedanceController/ObjectTurnaroundDetector.h] Add including of Hrpsys.h to pass QNX build.
  * [CMakeLists.txt] Specify include_directories as higher priority to pass QNX build.

* [sample/SampleRobot/samplerobot_emergency_stopper.py,samplerobot_remove_force_offset.py] Use DataLogger instead of readDataPort for sample. (`#950 <https://github.com/fkanehiro/hrpsys-base/issues/950>`_)

* sample/SampleRobot/samplerobot_sequence_player.py: fix checkArrayBetween function (`#919 <https://github.com/fkanehiro/hrpsys-base/issues/919>`_)

  * Add loadpatternst sample (`#907 <https://github.com/fkanehiro/hrpsys-base/issues/907>`_)

    * [sample/SampleRobot/samplerobot_stabilizer.py] Add example for loadPattern + Stabilizer.
    * [sample/SampleRobot/data, CMakeLists.txt] Add generated walking pattern file for SampleRobot. Add installation of data.

  * Update carry sample  (`#909 <https://github.com/fkanehiro/hrpsys-base/issues/909>`_)

   * [.travis.sh] Download and overwrite deb installed tests for downstream hrpsys-ros-bridge
   * [sample/SampleRobot/samplerobot_carry_object.py] Define object turnaround detection time threshold and use hand fix mode during pushing manipulation.
   * [sample/SampleRobot/README.md] Add conf file setting for el sample README.

  * Update st, abc, el, and travis.sh to pass travis tests (`#903 <https://github.com/fkanehiro/hrpsys-base/issues/903>`_)

    * [sample/SampleRobot/samplerobot_soft_error_limiter.py] Check soft error limit checking after version '315.5.0'
    * [.travis.sh] Add inputting of N for mongodb configuration during deb install reported in https://github.com/fkanehiro/hrpsys-base/pull/900#issuecomment-162392884
    * [sample/SampleRobot/samplerobot_soft_error_limiter.py] Use getActualState().command instead of rtm.readDataPort of el joint angle output to keep thread safety.
    * [rtc/AutoBalancer/AutoBalancer.cpp] Revert AutoBalancer 7a8bc6781608d4251b6c268123d99781ea4d405b change which does not pass samplerobot_auto_balancer.py test.
    * [sample/SampleRobot/samplerobot_auto_balancer.py] Use abs for Base RPY error checking and check base RPY error between reference and actual.
    * [rtc/Stabilizer/Stabilizer.*] Update Stabilizer doc including paper names and equation numbers.

  * Update samplerobot python unittest (`#912 <https://github.com/fkanehiro/hrpsys-base/issues/912>`_)

   * [sample/SampleRobot/samplerobot_sequence_player.py] Use StateHolder's getCommand to get seq results.
   * [test/test-samplerobot-*.py] Use Unittest for samplerobot example testing to enable test results output and respawn of rostest.
   * [.travis.sh] Print rosunit-*.xml if rostest fails

* Fix pkg-config file, includedir should be the include directory, not the compiler flag. (`#947 <https://github.com/fkanehiro/hrpsys-base/issues/947>`_)

* [.travis.sh] Use --purge option for mongodb apt-get remove in order to remove configuration file. (reported in https://github.com/fkanehiro/hrpsys-base/pull/900#issuecomment-162392884) (`#906 <https://github.com/fkanehiro/hrpsys-base/issues/906>`_)



Unstable RTCs
=============

* ImpedanceController

  * Fix bug of virtualforce (`#976 <https://github.com/fkanehiro/hrpsys-base/issues/976>`_)

    * Modified Stabilizer to fix bug of virtual force
    * Modified ImpedanceController for enabling VirtualForce

  * [rtc/ImpedanceController/ObjectTurnaroundDetector.h,sample/SampleRobot/samplerobot_impedance_controller.py] Fix to  use round to convert double time parameter to size_t time count. (`#964 <https://github.com/fkanehiro/hrpsys-base/issues/964>`_)

  * Add FFI for JointPathEx (`#938 <https://github.com/fkanehiro/hrpsys-base/issues/938>`_)

    * [sample/euslisp/eus-joint-path-ex.l] Add FFI example using euslisp. Keeping hrpsys-base working without euslisp existence.
    * [rtc/ImpedanceController/CMakeLists.txt, JointPathExC.cpp] Add JointPathEx example externed into C used for FFI.

  * [rtc/ImpedanceController/JointPathEx.cpp] Add debug print for nan from Inverse Kinematics calculations (`#925 <https://github.com/fkanehiro/hrpsys-base/issues/925>`_)

* ReferenceForceUpdater Add reference force updater (`#974 <https://github.com/fkanehiro/hrpsys-base/issues/974>`_)

  * [doc/, rtc/ReferenceForceUpdater/ReferenceForceUpdater.txt] add document for ReferenceForceUpdater
  * [python/hrpsys_config.py, launch/samplerobot.launch] enable to use rfu in robots
  * [idl/, rtc/CMakeLists, rtc/ReferenceForceUpdater/] add new RTC named ReferenceForceUpdater(rfu)

* SoftErrorLimitter

  * [rtc/SoftErrorLimiter/robot.cpp] Add print message for setServoErrorLimit (`#967 <https://github.com/fkanehiro/hrpsys-base/issues/967>`_)
  * [rtc/SoftErrorLimiter/SoftErrorLimiter.cpp, sample/SampleRobot/samplerobot_soft_error_limiter.py] Fix reference joint angle used to calculate error. Use joint angle which consider position limit and velocity limit.  (`#966 <https://github.com/fkanehiro/hrpsys-base/issues/966>`_)

* Beeper

  * Add commenttoconnect and build beeper (`#964 <https://github.com/fkanehiro/hrpsys-base/issues/964>`_)

    * [python/rtm.py] Add print message for dataflow_type, subscription_type, and so on.
    * [rtc/CMakeLists.txt] Build Beeper RTC.

  * Add Beeper RTC (`#963 <https://github.com/fkanehiro/hrpsys-base/issues/963>`_)

    * [python/hrpsys_config.py] Add Beeper to getUnstableRTC. Change order of el and tl to make tl higher priority for beeping.
    * [rtc/SoftErrorLimiter/SoftErrorLimiter.*, rtc/CollisionDetector/CollisionDetector.*] Support both BeepClient to use BeeperRTC and start_beep for stable beeping RTCs because stable RTC does not support BeeperRTC.
    * [rtc/ThermoLimiter/ThermoLimiter*, rtc/EmergencyStopper/EmergencyStopper.*] Use BeepClient to use BeeperRTC instead of start_beep for unstable beeping RTCs.
    * [rtc/SoftErrorLimiter/beep.h] Add BeepClient class to use BeepRTC
    * [rtc/Beeper] Add RTC to beep which takes input from several RTCs.

* Kalman Filter

  * [EKFilter.h] fix typo : fussy -> fuzzy  (`#958 <https://github.com/fkanehiro/hrpsys-base/issues/958>`_)

  * Fussy tuned kalman filter (`#957 <https://github.com/fkanehiro/hrpsys-base/issues/957>`_)

    * [KalmanFilter/EKFilter.h] use fuzzy logic to tune R matrix
    * [samplerobot_kalman_filter.py] run test programs both with RPYKalmanFilter and with QuaternionExtendedKalmanFilter
    * [samplerobot_kalman_filter.py] start auto balancer at the beginning to avoid slip
    * [samplerobot_kalman_filter.py] compare kf_baseRpyCurrent with SampleRobot(Robot)0_WAIST not kf_rpy

  * Add quaternion kf test  (`#956 <https://github.com/fkanehiro/hrpsys-base/issues/956>`_)

    * [sample6dofrobot_kalman_filter.py.in] optimize label location
    * [sample6dofrobot_kalman_filter.py.in] add quaternion estimator test
    * [EKFilter.h, KalmanFilter.cpp] implement resetKalmanFilterState in EKFFilter
    * [sample6dofrobot_kalman_filter.py.in] use actual rpy from simualtor
    * [sample/Sample6dofRobot] rotate initial pose

  * Update quaternion ekf (`#955 <https://github.com/fkanehiro/hrpsys-base/issues/955>`_)

    * [KalmanFilter/EKFilter.h] update coding styles for readability
    * [KalmanFilter/EKFilter.h] refectering
    * [KalmanFilter/EKFilter.h] use reference instead of returning value
    * [KalmanFilter/EKFilter.h] clean up redundant codes
    * [KalmanFilter/EKFilter.h] use rotation quaternion to rotate coordinate instead of rotation matrix
    * [KalmanFilter/EKFilter.h] use hrpUtil to get Euler Angles from Rotation Matrix
    * [KalmanFilter/EKFilter.h] use const reference parameters
    * [KalmanFilter/EKFilter.h] do not pass a member variable to member functions
    * [KalmanFilter/EKFilter.h] update calcF for readability
    * [KalmanFilter/EKFilter.h] use const member functions
    * [KalmanFilter/EKFilter.h] remove unused old comments
    * [KalmanFilter/EKFilter.h] add a magic comment to use a 2 space indentation
    * [KalmanFilter/EKFilter.h] use initializer list at EKFilter
    * [KalmanFilter/EKFilter.h] Q should be gyro noise covariance in order to make it easy to tune parameters
    * [KalmanFilter/EKFilter.h] normalize rotation quaternion as soon as possible
    * [KalmanFilter/EKFilter.h] acceleration reference is to handle in KalmanFilter.cpp
    * [KalmanFilter/EKFilter.h] fix bug : we should normalize only rotation quaternion

* TorqueController

  * Add getter method to torque controller (`#933 <https://github.com/fkanehiro/hrpsys-base/issues/933>`_)

    * [TorqueController] Fix transition time expression bag
    * [TorqueController] Rename paramter argument name in torque controller to corersponding rtm-ros-robot-interface: t_param -> i_param
    * [TorqueController] Add get parameter methods for torque controller

  * Fix torque controller pass qref mode  (`#926 <https://github.com/fkanehiro/hrpsys-base/issues/926>`_)

    * [TorqueController] Fix merge miss in timestamp
    * [TorqueController] Supress dq from torque controller by min/max_dq
    * [TorqueController] Pass qRefIn without checking range of motion when motor torque contorller is disabled
    * [MotorTorqueController] tauMax should be not zero when tau is zero
    * [TorqueController] Check size of qRef to prevent accessing qRefIn when qRefIn size is not same as joint_num

* AutoBalancer

  * [rtc/AutoBalancer/GaitGenerator.h] Add boundary conditions of velocity and acceleration to GaitGenerator (`#981 <https://github.com/fkanehiro/hrpsys-base/issues/981>`_)
  * [rtc/AutoBalancer/GaitGenerator.h] Fix zmp weight interpolation and use setGoal instead of go. (`#973 <https://github.com/fkanehiro/hrpsys-base/issues/973>`_)

  * Get foosteps (`#939 <https://github.com/fkanehiro/hrpsys-base/issues/939>`_)

    * [AutoBalancer.cpp, GaitGenerator.cpp, GaitGenerator.h] use const member function in getGoPosFootstepsSequence
    * [GaitGenerator.cpp, GaitGenerator.h] pass vel_param for argument in go_pos_param_2_footstep_nodes_list
    * [AutoBalancerService.idl, AutoBalancer.cpp, AutoBalancer.h, AutoBalancerService_impl.cpp, AutoBalancerService_impl.h] add getFootstepsSequence function
    * [GaitGenerator.cpp, GaitGenerator.h] overload go_pos_param_2_footstep_nodes_list to get new_footstep_nodes_list
    * [AutoBalancer.cpp, AutoBalancer.h] move initial_support_legs calculation method from inside goPos to a new method

  * [rtc/AutoBalancer/AutoBalancer.cpp] Initialize gait_type as BIPED.  (`#937 <https://github.com/fkanehiro/hrpsys-base/issues/937>`_)

  * Update JointPathEx IK (`#942 <https://github.com/fkanehiro/hrpsys-base/issues/942>`_)

    * [idl/AutoBalancerService.idl,idl/StabilizerService.idl,rtc/AutoBalancer/AutoBalancer.*,rtc/Stabilizer/Stabilizer.cpp] Enable to set IK weight vector for STtabilizer and Autobalancer like ImpedanceController.
    * [rtc/ImpedanceController/ImpedanceController.cpp,rtc/ImpedanceController/JointPathEx.*,rtc/Stabilizer/Stabilizer.cpp] Move end-effector version inverse kinematics to JointPathEx and use it in IC and ST.
    * [rtc/ImpedanceController/ImpedanceController.cpp,JointPathEx.*,rtc/Stabilizer/Stabilizer.cpp,rtc/AutoBalancer/AutoBalancer.cpp] Add calcInverseKinematics2Loop function to take target pos and Rot and use it in ic, abc, and st. Currently omegaFromRot is under checking and tempolarily use old matrix_log function, so program behaviour does not change.
    * [idl/AutoBalancerService.idl,rtc/AutoBalancer/AutoBalancer.*] Remove deprecated footstep information lleg_coords and rleg_coords. Remove unused current\_* parameter from ABCIKparam.

  * fix bug when overwriting footstep (`#940 <https://github.com/fkanehiro/hrpsys-base/issues/940>`_)

    * [rtc/AutoBalancer/GaitGenerator.cpp, rtc/AutoBalancer/PreviewController.h] fix bug when overwriting footstep
    * [sample/SampleRobot/samplerobot_auto_balancer.py] Add checking for discontinuity of COG trajectory during footstep overwriting by checking COG too large acc.

  * Update gaitgenerator and fix bugs  (`#918 <https://github.com/fkanehiro/hrpsys-base/issues/918>`_)

    * [rtc/AutoBalancer/GaitGenerator.cpp,h] Add get_overwrite_check_timing
    * [.travis.sh] Print if rosunit_xml_result_files exists
    * [rtc/AutoBalancer/GaitGenerator.cpp] Enable emergencyStop for walking anytime. Previously, emergency flag is checked at half of step time.
    * [rtc/AutoBalancer/GaitGenerator.cpp] Set toe heel time count based on each footstep step count

  * Update gopos (`#877 <https://github.com/fkanehiro/hrpsys-base/issues/877>`_)

    * [sample/SampleRobot/samplerobot_impedance_controller.py] Check hrpsys_version for samplerobot impedance test
    * [rtc/ImpedanceController/JointPathEx.*, AutoBalancer, ImpedanceController, SequencePlayer, Stabilizer] Reduce limit over print message frequence in JointPathEx and add more information for it.
    * [sample/SampleRobot/samplerobot_impedance_controller.py, test/test-samplerobot-impedance.py] Test samplerobot_impedance_controller python example
    * [rtc/*] Update print message from RTCs like [el]
    * [rtc/AutoBalancer/GaitGenerator.*] Update appending of footstep function. Define both const and non-const member function.
    * [rtc/AutoBalancer/GaitGenerator.*, AutoBalancer.cpp] Enable to overwrite goPos target goal.
    * [sample/SampleRobot/samplerobot_auto_balancer.py] Add check test for goPos final dst_foot_midcoords and add example for goPos overwrite.
    * [rtc/AutoBalancer/GaitGenerator.h] Set is_initialize for gopos true by default to pass tests with default argument.
    * [rtc/AutoBalancer/GaitGenerator.*] Use const member function for getter and printing functions.


  * Overwrite current footstep  (`#916 <https://github.com/fkanehiro/hrpsys-base/issues/916>`_)

    * [rtc/AutoBalancer/GaitGenerator.cpp] Fix for future velocity footsteps. Integrate future steps.
    * [sample/SampleRobot/samplerobot_auto_balancer.py] Add example for current footstep overwrite
    * [rtc/AutoBalancer/GaitGenerator.cpp,rtc/AutoBalancer/PreviewController.h] Enable to set overwritable_footstep_index_offset = 0.
    * [idl/AutoBalancerService.idl, rtc/AutoBalancer/AutoBalancer.cpp,GaitGenerator.h] Enable to set overwritable_footstep_index_offset.
    * [rtc/AutoBalancer/GaitGenerator.cpp] Fix order of overwrite zmp processing and add comments. This should not change behaviour.
    * [rtc/AutoBalancer/GaitGenerator.cpp,h] Enable to set future_step_num and use get_overwritable_index.

* Stabilizer

  * Add refforce weight to eefmqp  (`#977 <https://github.com/fkanehiro/hrpsys-base/issues/977>`_)

    * [rtc/Stabilizer/Stabilizer.cpp,ZMPDistributor] Add ref force weight to eefmqp
    * [rtc/AutoBalancer/AutoBalancer.cpp] Modify ref force output

  * [rtc/Stabilizer/ZMPDistributor.h] do not distribute ForceMoment to swing foot (`#972 <https://github.com/fkanehiro/hrpsys-base/issues/972>`_)

  * Add fall direction  (`#948 <https://github.com/fkanehiro/hrpsys-base/issues/948>`_)

    * merge origin/master by hand
    * [AutoBalancer.cpp] stop walking if emergency signal is set
    * [Stabilizer.cpp] check single support phase only in wailking for recovery
    * [StabilizerService.idl, Stabilizer.cpp, Stabilizer.h] add tilt_margin parameter for single support phase and double support phase
    * [StabilizerService.idl, Stabilizer.cpp] add TILT emergency mode
    * [Stabilizer.cpp, Stabilizer.h] add fall direction caulculator


  * Fix abc st segfo (`#951 <https://github.com/fkanehiro/hrpsys-base/issues/951>`_)

    * [rtc/AutoBalancer/AutoBalancer.cpp] Fix initialization of target_p0 and target_r0
    * [rtc/Stabilizer/ZMPDistributor.h] Check size of ee params to avoid segfo.

  * [Stabilizer.cpp, Stabilizer.h] fix swing leg modification rule (`#949 <https://github.com/fkanehiro/hrpsys-base/issues/949>`_)

  * [StabilizerService.idl, Stabilizer.cpp, Stabilizer.h] add eefm_swing_pos_time_const/eefm_swing_rot_time_const parameter (`#949 <https://github.com/fkanehiro/hrpsys-base/issues/949>`_)

  * Add argument check st abc (`#945 <https://github.com/fkanehiro/hrpsys-base/issues/945>`_)

    * [sample/SampleRobot/samplerobot\_*.py] Use DataLogger log for check robot's state for testing
    * [test/test-samplerobot.test] Set order for samplerobot test execution. For example, DataLogger, SequencePlayer, ...
    * [rtc/AutoBalancer/AutoBalancer.cpp,rtc/Stabilizer/Stabilizer.cpp] Fix location of set Ik parameter and add comments and message
    * [rtc/AutoBalancer/AutoBalancer.cpp,rtc/Stabilizer/Stabilizer.cpp] Add argument length check for IK parameter for AutoBalancer and Stabilizer

  * [idl/AutoBalancerService.idl,idl/StabilizerService.idl,rtc/AutoBalancer/AutoBalancer.cpp,rtc/Stabilizer/Stabilizer.cpp] Use IKLimbParameters instead of each sequence paraemters for IK of AutoBalancer and Stabilizer.  (`#944 <https://github.com/fkanehiro/hrpsys-base/issues/944>`_)

  * [idl/AutoBalancerService.idl,idl/StabilizerService.idl,rtc/AutoBalancer/AutoBalancer.*,rtc/Stabilizer/Stabilizer.*] Add IK parameter interface for AutoBalancer and Stabilizer. (`#943 <https://github.com/fkanehiro/hrpsys-base/issues/943>`_)

  * Add moment limit and test for turnwalk (`#936 <https://github.com/fkanehiro/hrpsys-base/issues/936>`_)

    * [sample/SampleRobot/samplerobot_stabilizer.py] Add test for turn walk abount 180[deg] yaw rotation.
    * [idl/StabilizerService.idl, rtc/Stabilizer/Stabilizer.*] Add limitation for end-effector frame local reference moment to avoid hardware break.

  + [Stabilizer.cpp] match f_diff frame to ref_f_diff one (`#935 <https://github.com/fkanehiro/hrpsys-base/issues/935>`_)

  * Fix fixed coords again  (`#917 <https://github.com/fkanehiro/hrpsys-base/issues/917>`_)

    * [AutoBalancer.cpp] resize leg_pos to end-effector size
    * [AutoBalancer.cpp] move leg_pos initialization position
    * [Stabilizer.cpp] match f_diff frame to ref_f_diff one
    * [sample4legrobot_auto_balancer.py] add goVelocity in trot sample
    * [AutoBalancer.cpp, GaitGenerator.cpp, GaitGenerator.h] support multileg in go velocity
    * [GaitGenerator.cpp, GaitGenerator.h] support multi legs for overwirte
    * [sample4legrobot_auto_balancer.py] add goPos in trot and in pace samples
    * [sample4legrobot_auto_balancer.py] move all end-pos +50mm in z axis to get manipulability
    * [AutoBalancerService.idl, AutoBalancer.cpp, AutoBalancer.h] add gait_type to AutoBalancer param to realize multiple gait in goPos
    * [sample4legrobot_auto_balancer.py] add crawl mode test
    * [AutoBalancer.cpp, GaitGenerator.h] fix fixed coordinates in multiple legs : only use legs / re-revert and update https://github.com/fkanehiro/hrpsys-base/commit/ad4eb10d05f98aca9f243bb72a81ffba4b51dd77

  * Modify swing leg end coords  (`#934 <https://github.com/fkanehiro/hrpsys-base/issues/934>`_)

    * [Stabilizer.cpp] add a new modification law as a comment
    * [Stabilizer.cpp] modify swing leg coods only in actual and reference swing time
    * [Stabilizer.cpp] calulate difference rpy in new_swg_R coordinates at swing leg modification
    * [Stabilizer.cpp] print d_rpy_swing / d_pos_swing for DEBUG
    * [Stabilizer.cpp] support multiple legs for swing leg modification
    * [Stabilizer.cpp] separte rotation scope and position one for readability
    * [Stabilizer.cpp, Stabilizer.h] rename delta_pos / delta_rpy to d_rpy_swing / d_pos_swing and keep these variables as member variables for extensibility
    * [Stabilizer.cpp] use eefm_swing_rot_spring_gain / eefm_swing_rot_spring_gain param
    * [StabilizerService.idl, Stabilizer.cpp, Stabilizer.h] add eefm_swing_rot_spring_gain / eefm_swing_rot_spring_gain as st param
    * [Stabilizer.cpp, Stabilizer.h] modify swing leg end-coords to follow target one in world coordinates

  * Return total force or moment from getObjectForcesMoments and consider moment_center as foot mid frame. (`#932 <https://github.com/fkanehiro/hrpsys-base/issues/932>`_)

    * [rtc/ImpedanceController/ImpedanceController.cpp] Check for legged robot.
    * [rtc/ImpedanceController/ObjectTurnaroundDetector.h] Reset current filtered param when detect mode switched.
    * [idl/ImpedanceControllerService.idl, rtc/ImpedanceController/*] Return total force or moment from getObjectForcesMoments and consider moment_center as foot mid frame.

  * Enable total moment detection by object turnaround detection. (`#930 <https://github.com/fkanehiro/hrpsys-base/issues/930>`_)

    * [idl/ImpedanceControllerService.idl,rtc/ImpedanceController/ImpedanceController.cpp,rtc/ImpedanceController/ImpedanceController.h,rtc/ImpedanceController/ObjectTurnaroundDetector.h] Enable total moment detection by object turnaround detection.

  * [Stabilizer.cpp] enable to change compensation limit : omission of https://github.com/fkanehiro/hrpsys-base/pull/852 (`#929 <https://github.com/fkanehiro/hrpsys-base/issues/929>`_)

  * [Stabilizer.cpp] fix com height of LIPM in Capture Point calculation (`#924 <https://github.com/fkanehiro/hrpsys-base/issues/924>`_)

  * [hrpsys_config.py] start stabilizer after auto-balancer in startDefaultUnstableControllers (`#928 <https://github.com/fkanehiro/hrpsys-base/issues/928>`_)

  * [Stabilizer.cpp] fix typo of https://github.com/fkanehiro/hrpsys-base/pull/895 (`#922 <https://github.com/fkanehiro/hrpsys-base/issues/922>`_)

  * Update graspless manip mode (`#921 <https://github.com/fkanehiro/hrpsys-base/issues/921>`_)

    * [sample/SampleRobot/samplerobot_auto_balancer.py] Add example for dual-arm graspless manip walking.
    * [rtc/AutoBalancer/AutoBalancer.cpp] Support dual-arm graspless mode while walking.

  * Add rostest for stabilizer.  (`#910 <https://github.com/fkanehiro/hrpsys-base/issues/910>`_)

    * [sample/SampleRobot/samplerobot_stabilizer.py] Check existence of sample1_bush.wrl because openhrp3 <= 3.1.8 does not have it.
    * [test/test-samplerobot-st.test, .travis.yml] Add test for samplerobot st with torque + pdcontrol + bush. Add travis job for testing st test.


* RangeDataViewer

  * rtc/RangeDataViewer/RangeDataViewer.cpp: suppresses debug messages and ignores inf

* get all q log (`#915 <https://github.com/fkanehiro/hrpsys-base/issues/915>`_)

  * [CollisionDetector.cpp] set timestamp for out port
  * [TorqueController.cpp] use upstream timestamp instead of current timestamp


* Contributors: Benjamin Chrétien, Eisoku Kuroiwa, Fumio Kanehiro, Iori Kumagai, Kei Okada, Kohei Kimura, Masaki Murooka, Mehdi Benallegue, Ryo Koyama, Shunichi Nozawa, Takasugi Noriaki, Yohei Kakiuchi, Yuta Kojio, Iori Yanokura

315.8.0 (2015-11-29)
--------------------

Stable RTCs
=============

* add rtc xml (https://github.com/fkanehiro/hrpsys-base/pull/880)
*  [rtc/SequencePlayer/interpolator.*,seqplay.cpp,rtc/AutoBalancer/AutoBalancer.cpp,rtc/AutoBalancer/GaitGenerator.h,rtc/CollisionDetector/CollisionDetector.cpp,rtc/EmergencyStopper/EmergencyStopper.cpp]
  Add name for interpolator and set name for RTCs using interpolator (`#848 <https://github.com/fkanehiro/hrpsys-base/issues/848>`_ )
* [README.md, sample/README.md] Add link for samples README.md
* [lib/util/BodyRTC.cpp] Do not use servo off emulation in HighGain mode.
* [python] Clarify arguments for setTargetPoseRelative
* [python/hrpsys_config.py] Modify hrpsys_config.py for connection
* [ec/hrpEC/hrpEC-common.cpp, hrpEC.h] Get RTC names when rtcs size change for https://github.com/fkanehiro/hrpsys-base/issues/806
* [ec/hrpEC/hrpEC-common.cpp] shows instance names when time over is detected
* [.travis.yml] Exec USE_SRC_OPENHRP3=true tests in faster orders to make debug of these tests easy.
* [.travis.{sh,yml}] add test code for openhrp3 source
* [.travis.yml] add slack notification  https://jsk-robotics.slack.com/messages/travis/details/

* Change include file path settings in hrpsys-base.pc file

  * [test/test-pkgconfig.py] fixes a include path ("io/iob.h" -> "hrpsys/io/iob.h")
  * [hrpsys-base.pc.in] changes includedir in pc file

* RobotHardware

  * [rtc/RobotHardware/robot.cpp] Add a compile option to add default implementation whenever readDigitalInput and lenghtDigitalInput are not available
  * Add a port and methods to read command torques, as well as actual torques. (They differ when the robot has torque sensing capabilities)
  * [rtc/RobotHardware/robot.cpp] Add print message for setServoGainPercentage

* SequencePlayer

  * [rtc/EmergencyStopper/EmergencyStopper.cpp,rtc/SequencePlayer/SequencePlayer.cpp] fill time stamp on reference of angles
  * Protect pop() and pop_back() operations with a mutex(`#839 <https://github.com/fkanehiro/hrpsys-base/issues/839>`_ )
  * [rtc/SequencePlayer/interpolator.{cpp,h}] Switch to using coil::Guard instead of boost
  * [rtc/SequencePlayer/interpolator.{cpp,h}] Include locks.hpp instead of lock_guard, for backwards compatibility
  * [rtc/SequencePlayer/interpolator.{cpp,h}] Protect pop() and pop_back() operations with a mutex to avoid popping twice the same element


* DataLogger

  * [rtc/DataLogger/DataLogger.cpp] Add message printing to DataLogger functions

* rtm.py

  * [python/rtm.py] narrow to RTObject produce error on some environments (`#858  <https://github.com/fkanehiro/hrpsys-base/issues/858>`_ )
  * [python/rtm.py] Add try&except for import CORBA failing on old python environment.

* hrpsys_config.py

  * [python/hrpsys_config.py] Support latest startAutoBalancer in startDefaultUnstableControllers.
  * [python/hrpsys_config.py] Use 4limbs in startAutoBalancer when Groups has rarm and larm.
  * [python/hrpsys_config.py, rtc/AutoBalancer/AutoBalancer.*, rtc/Stabilizer/Stabilizer.*, rtc/Stabilizer/ZMPDistributor.h] shift a support polygon when set-ref-force
  * fix typo: tmp_contollers -> tmp_controllers
  * [python/hrpsys_config.py, rtc/Stabilizer, rtc/AutoBalancer] add walkingStates port from abc to st
  * [python/hrpsys_config.py] Add el log for final reference joint angles output for both Stable RTC users and Unstable RTC Users


Unstable RTCs
=============

* Samples

  * [sample/SampleSpecialJointRobot/SampleSpecialJointRobot.conf.in] Add interlocking joint setting
  * [sample/environments/Dumbbell.wrl, sample/SampleRobot/samplerobot_carry_object.py] Update Dumbbell handle and add auto detection sample
  * [README.md, sample/SampleRobot/README.md] Add new sample explanation to SampleRobot README and add link to top page
  * [sample/environments/Dumbbell.wrl, sample/SampleRobot/samplerobot_carry_object.py] Update Dumbbell handle and add auto  detection sample
  * [sample/SampleRobot/ForceSensorOffset_SampleRobot.txt, samplerobot_carry_object.py, CMakeLists.txt] Add force sensor offset for rmfo and update controller initialization in carry sample
  * [sample/SampleRobot/*carry*, sample/environments/PushBox.wrl] Add push box and push manipulation demo
  * [sample/SampleRobot/SampleRobot.carryobject.xml.in,samplerobot_carry_object.py] Add ABS_TRANSFORM for each object and  add walking example
  * sample/environments/Dumbbell.wrl, sample/SampleRobot/*] Add Dumbbell model and add carry up example.
  * [sample/SampleSpecialJointRobot/*, rtc/AutoBalancer/*] Enable toe joint example and support robots witch leg joints >= 7
  * [launch/samplespecialjointrobot.launch, sample/SampleSpecialJointRobot/, sample/CMakeLists.txt] Add files for SampleSpecialJointRobot
  * [sample/Sample4LegRobot] Add kinematics simulation xml and call set parameter func in demo program
  * [sample/Sample4LegRobot/sample4legrobot_stabilizer.py] Update st params
  * [sample/Sample4LegRobot/Sample4LegRobot.xml.in] Use non-bush model for non-torquecontrol simulation
  * [sample/Sample*Robot/Sample*Robot.conf.in] Add optionalData setting
  * [sample/Sample*Robot/Sample*Robot.conf.in] Add parameters for ThermoLimter and CollisionDetector and hide print messages on simulation
  * [sample/Sample4LegRobot/sample4legrobot_stabilizer.py] Add st and abc setting
  * [launch/sample4legrobot.launch, sample/Sample4LegRobot, sample/CMakeLists.txt] Add files for Sample4LegRobot
  * [sample/SampleRobot/SampleRobot.conf.in, rtc/PDcontroller/PDcontroller.cpp] Enable to set gain file from bindParameter (https://github.com/fkanehiro/hrpsys-base/pull/789) and rename pdgains_sim.file_name => pdgains_sim_file_name
  * [sample/SampleRobot/samplerobot_auto_balancer.py] add a sample program of setFootSteps with arms
  * [sample/SampleRobot/samplerobot_auto_balancer.py] add a sample program of four leg auto-balancer
  * [sample/SampleRobot/samplerobot_auto_balancer.py] add four legs mode pose
  * [sample/SampleRobot/samplerobot_auto_balancer.py] apply numpy.allclose to list of list
  * [sample/SampleRobot/samplerobot_auto_balancer.py] set acceptable error between reference  and actual default_zmp_offsets
  * [sample/SampleRobot/samplerobot_auto_balancer.py] add debug message to demoAutoBalancerSetParam
  * [sample/SampleRobot/samplerobot_stabilizer.py] Tune stabilizer eefm parameter using rubber bush and torque control mode
  * [sample/SampleRobot/samplerobot_soft_error_limiter.py] Remove unnecessary mdlldr and fix newline
  * [sample/SampleRobot/samplerobot_soft_error_limiter.py] Update limit table check and add error and vel limit check
  * [launch/samplerobot.launch,sample/SampleRobot/SampleRobot.PDgain.dat,SampleRobot.torque.xml.in] Update torquecontrol to use sample1_bush
  * [sample/SampleRobot/samplerobot_auto_balancer.py] add assert to check success of setting default_zmp_offsets
  * [sample/SampleRobot/samplerobot_stabilizer.py] Fix samplerobot st sample parameter

* AutoBalancer (support 4 legs)

  * [rtc/AutoBalancer/AutoBalancer.cpp] do not change autobalancer mode when leg_names are unchanged
  * [rtc/AutoBalancer/AutoBalancer.cpp] Set is_hand_fix_mode false by default same as startautobalancer in [rleg, lleg].
  * [sample/Sample4LegRobot/sample4legrobot_stabilizer.py,sample/SampleRobot/samplerobot_auto_balancer.py,sample/SampleSpecialJointRobot/samplespecialjointrobot_auto_balancer.py]
Update samples for startAutoBalancer update.
  * [sample/Sample4LegRobot/sample4legrobot_auto_balancer.py] Add Rectangle and Cycloiddelay orbit 4leg walking samples.
  * [rtc/AutoBalancer/GaitGenerator.[cpp,h]] Support rectangle and cycloiddelay for multi leg walking. Currently other orbits are not supported because we need to update a method to parameter setting and getting.
  * [AutoBalancer/AutoBalancer.cpp] fix fixed coordinates in multiple legs : only use legs
  * [AutoBalancer/AutoBalancer.cpp, Stabilizer/Stabilizer.cpp] do not change end-effector parameters except during MODE_IDLE
  * [rtc/AutoBalancer/AutoBalancer.cpp] add end_effector_list to set/getAutoBalancerParam
  * [idl/AutoBalancerService.idl] add end_effector_list to AutoBalancerParam
  * [sample/SampleRobot] set all limbs to limbs arguments in trot walking
  * [sample/Sample4LegRobot] add a trot walking demo program
  * [AutoBalancer/GaitGenerator.cpp] modify toe heel angle in only biped or crawl
  * [rtc/AutoBalancer/AutoBalancer.cpp] Disable to change new zmp parameter and Modify for old zmp parameter
  * [rtc/AutoBalancer/AutoBalancer.cpp] Enable to Change zmp parameters
  * [rtc/AutoBalancer/AutoBalancer.cpp] Add Zmp parameter (default double support ratio before and after)
  * [rtc/AutoBalancer/AutoBalancer.cpp] Add Zmp Parameter(default double support static ratio before and after)
  * [rtc/AutoBalancer/AutoBalancer.cpp] Fix rotation of hand fix coords offset
  * [sample/SampleRobot/samplerobot_auto_balancer.py] Fix order of samples and update for zmp transition and fix hands
  * [rtc/AutoBalancer/AutoBalancer.txt] Update fix hand mode according to cog vel and update documentation.
  * [sample/SampleRobot/samplerobot_auto_balancer.py] Add sample for hand fix walking.
  * [idl/AutoBalancerService.idl, rtc/AutoBalancer/AutoBalancer.[cpp,h]] Add hand fix mode. By default, no fix mode.
  * [rtc/AutoBalancer/GaitGenerator.cpp] Check difference projected on start coords to avoid problems reported in https://github.com/fkanehiro/hrpsys-base/issues/845
  * [idl/AutoBalancerService.idl, rtc/AutoBalancer/AutoBalancer.cpp] add use_force_mode to AutoBalancerParam

* AutoBalancer (support external forces)

  * [idl/AutoBalancerService.idl, rtc/AutoBalancer/AutoBalancer.cpp] add use_force_mode to AutoBalancerParam
  * add leg orbit type for cross step
  * [rtc/Autobalancer/GaitGenerator.cpp] Modify leg coords generator for changing double support time after swing
　* [rtc/Autobalancer/Autobalancer.cpp] Disable to change double support time for swing leg
　* [rtc/AutoBalancer/AutoBalancer.cpp] Add double support time before and after swing to AutoBalancer
  * [rtc/AutoBalancer/GaitGenerator.cpp] Do not reuse vector for swing foot zmp offsets.
  * [rtc/AutoBalancer/GaitGenerator.h] Fix printing of footsteps.
  * [rtc/AutoBalancer/AutoBalancer.cpp] Substitute ref_forces calculated from ZMP for ref_force's outport at ABC
  * [rtc/AutoBalancer/AutoBalancer.cpp] Add Outport of ref_forces to AutoBalancer
  * [rtc/AutoBalancer/AutoBalancer.cpp] Set Contact States for ee not included in leg_names to false
  * [rtc/AutoBalancer/AutoBalancer.*] Enable to output contact and swing support time
  * [AutoBalancer.*] add leg_names_interpolator in order to change leg_names during MODE_ABC
  * [AutoBalancer.cpp] add Guard at the top of setAutoBalancerParam
  * [rtc/AutoBalancer/testGaitGenerator.cpp,GaitGenerator.cpp] Fix double support phase count and contact state change.
  * [rtc/AutoBalancer/GaitGenerator.*] Add is_swing_phase member
  * [rtc/AutoBalancer/testGaitGenerator.cpp] Display contact states on swing support time plotting
  * [AutoBalancer.cpp, GaitGenerator.*] extend contactStates, controlSwingSupportTime and limbCOPOffset for arms
  * [AutoBalancer.cpp] fix typo of index
  * [rtc/AutoBalancer/AutoBalancer.*] Reduce debug pring for ik error
  * [GaitGenerator.cpp] fix the order of passing arguments
  * [AutoBalancer.cpp] use target_p0/r0 instead of target_link->p/R to calculate ref_cog in order to avoid discontinuity of ref_cog
  * [AutoBalancer.cpp, GaitGenerator.h] add zmp_weight_interpolator
  * [AutoBalancer.*] rename zmp_interpolator to zmp_offset_interpolator for zmp_weight_interpolator
  * [idl/AutoBalancerService.idl, AutoBalancer.cpp, AutoBalancerService_impl.cpp] set the number of default_zmp_offsets according to the number of end-effectors
  * [AutoBalancer/AutoBalancer.cpp] fix typo : get_default_step_height -> get_toe_angle / get_heel_angle
  * [AutoBalancer.cpp] move some code blocks in onInitialize to use end-effector information
  * [rtc/AutoBalancer/AutoBalancer.cpp] Fix abc ik error bug. Calculate difference from current->target and update threshold
  * [testGaitGenerator.cpp] cannot use comparison operator between const std::vector<std::string> and boost::assign::list_of(std::string) in HRP2 inside PC
  * [idl/AutoBalancerService.idl, AutoBalancer.cpp, AutoBalancerService_impl.cpp] add zmp_weight_map to GaitGeneratorParams
  * [AutoBalancer.cpp, GaitGenerator.*] add zmp weight map which is used in target zmp calculation
  * [rtc/AutoBalancer/AutoBalancer.cpp] Do not check ik error during start and stop auto balancer
  * [idl/AutoBalancerService.idl, AutoBalancer.*, AutoBalancerService_impl.*, GaitGenerator.h] add setFootStepNodes for multiple legs
  * [GaitGenerator.h] use weight factor in get_swing_support_mid_coords for crawl walking
  * [GaitGenerator.h] print index of foot steps
  * [AutoBalancer.cpp] do not print unless DEBUG mode
  * [GaitGenerator.h] add default constructor of step_node
  * add outport for ref-capture-point
  * [idl/AutoBalancerService.idl, rtc/AutoBalancer/AutoBalancer.*] Enable to check ik error.

* Stabilizer (capture points)

  * [rtc/Stabilizer/Stabilizer.cpp] Fix bug of st compensation frame.
  * [rtc/Stabilizer/Stabilizer.cpp] fix calculation of cp for visualization
  * [idl/StabilizerService.idl, rtc/Stabilizer/Stabilizer.*] check whether capture point is inside support polygon
  * [rtc/Stabilizer/ZMPDistributor.h] add function to check whether point is inside support polygon
  * [rtc/Stabilizer/ZMPDistributor.h] add function to calculate ConvexHull
  * [idl/StabilizerService.idl, rtc/Stabilizer/Stabilizer.*] disable emergency stop while walking by default
  * [idl/StabilizerService.idl, rtc/Stabilizer/Stabilizer.cpp] add end_effector_list to set/getParameter
  * [Stabilizer/Stabilizer.cpp] add mutex guards
  * [Stabilizer/Stabilizer.cpp] add tm info to out ports
  * [rtc/Stabilizer/Stabilizer.cpp] fix bug about checking cp error
  * [rtc/Stabilizer/Stabilizer.cpp] fix typo : Reduce frequency of cp error print message
  * [Stabilizer/Stabilizer.cpp] set contact states for all the limbs
  * [idl/AutoBalancerService.idl] Change idl's description
  * [rtc/Stabilizer/Stabilizer.cpp, rtc/Stabilizer/ZMPDistributor.h] change detection of falling with cp
  * [idl/StabilizerService.idl,  rtc/Stabilizer/Stabilizer.*] enable to set compensation limit
  * [rtc/Stabilizer/Stabilizer.cpp] Enable to set emergency_check_mode always.
  * [rtc/Stabilizer/Stabilizer.*] Reduce frequency of cp error print message
  * [rtc/Stabilizer/Stabilizer.cpp] Use inport ref-force moment for initial values.
  * [rtc/Stabilizer/Stabilizer.cpp] Fix wait for stop stabilizer.
  * [rtc/Stabilizer/ZMPDistributor.h, Stabilizer.cpp] Use pinv version for multileg debug and add print messages
  * [rtc/Stabilizer/ZMPDistributor.h] Use limb_gain for feedforward force calculation
  * [rtc/Stabilizer/] Use limb gain for swing support transition
  * [rtc/Stabilizer/ZMPDistributor.h] Add non inequality distribution
  * [idl/StabilizerService.idl] convert CapturePoint from foot-origin relative to root-link relative
  * [rtc/Stabilizer/Stabilizer.cpp] Fix st sensor name check for robots with toe joints
  * [rtc/Stabilizer/Stabilizer.*] Enable swing->support gain transition
  * [rtc/Stabilizer/Stabilizer.cpp, rtc/AutoBalancer/GaitGenerator.*] Print swing support time and consider swing phase for swing suport time calculation
  * [rtc/Stabilizer/Stabilizer.*] Calc swing support gain from remain time
  * [rtc/Stabilizer/Stabilizer.cpp, ZMPDistributor.h] Use cop distance and add d_foot_pos print message
  * [rtc/Stabilizer/Stabilizer.cpp] Add independent limb ik
  * [rtc/Stabilizer/Stabilizer.cpp] Reduce redundant calculation of pos_ctrl
  * [idl/StabilizerService.idl, rtc/Stabilizer/Stabilizer.cpp] Remove deprecated parameters for old st mode
  * [idl/StabilizerService.idl, rtc/Stabilizer/Stabilizer.*] Add argument to select force difference control mode
  * [python/hrpsys_config.py, rtc/Stabilizer/Stabilizer.*] Update st debug reference and compensation port for multi legged robots
  * [idl/StabilizerService.idl, rtc/Stabilizer/Stabilizer.cpp, sample/Sample*/*_stabilizer.py] Enable to set all vertices of support polygon
  * [rtc/Stabilizer/testZMPDistributor.cpp] Initialize ref force moment for test
  * [idl/StabilizerService.idl, rtc/Stabilizer/Stabilizer.*] Enable to set eefmqpcop algorithm
  * [rtc/Stabilizer/ZMPDistributor.h] Update for multi leg force moment distribution
  * [rtc/Stabilizer/Stabilizer.*] Rename ref force moment variable
  * [rtc/Stabilizer/*] Enable to set limb ref force and moment
  * [rtc/Stabilizer/Stabilizer.*] Fix for prev act force z
  * [rtc/Stabilizer/Stabilizer.cpp] Use zmp calc and feedback checking
  * [rtc/Stabilizer/ZMPDistributor.h] Fix for compile not USE_QPOASES
  * [idl/StabilizerService.idl, rtc/Stabilizer/Stabilizer.*] Add parameter for end effector feedback and zmp calc params
  * [rtc/Stabilizer/ZMPDistributor.h, rtc/Stabilizer/testZMPDistributor.cpp] Add force moment distribution by cop distance
  * [rtc/Stabilizer/testZMPDistributor.cpp] Fix plotting of test zmp distributor
  * [rtc/Stabilizer/Stabilizer.*, rtc/EmergencyStopper/EmergencyStopper.cpp] Reset emergency flag when st mode is moved to idle or air.
  * [rtc/Stabilizer/Stabilizer.cpp, rtc/AutoBalancer/AutoBalancer.cpp, rtc/ImpedanceController/ImpedanceController.cpp, JointPathEx.*] Enable interlocking joints setting for AutoBalancer, ImpedanceController, Stabilizer
  * [idl/StabilizerService.idl] Update comments of types
  * [rtc/Stabilizer/Stabilizer.cpp] Update print message and add setter check
  * [idl/StabilizerService.idl, rtc/Stabilizer/Stabilizer.*] Enable to set all end effector damping param.
  * [rtc/Stabilizer/Stabilizer.*] Add d_foot_xx to st ik param
  * [rtc/Stabilizer/Stabilizer.*] Use LPF for target ee diff p
  * [rtc/Stabilizer/Stabilizer.*] Enable to use body attitude control for both tpcc and eefm
  * [rtc/Stabilizer/*] Use LPF in IIRFilter.h
  * [rtc/Stabilizer/ZMPDistributor.h] Fix argument for USE_QPOASES OFF
  * [rtc/Stabilizer/Stabilizer.*] Remove deprecated leg variables and force sensor checking in every loop
  * [python/hrpsys_config.py, rtc/Stabilizer/Stabilizer.*] Connect all force sensors and do not check whether leg or not in python and connection phase
  * [rtc/Stabilizer/testZMPDistributor.cpp, Stabilizer.cpp, ZMPDistributor.h] Fix immediate value for rleg lleg index.
  * [rtc/Stabilizer/ZMPDistributor.h] Fix const addition
  * [rtc/Stabilizer/ZMPDistributor.h] Fix rleg and lleg usage
  * [rtc/Stabilizer/ZMPDistributor.h,rtc/Stabilizer/testZMPDistributor.cpp] Update test moment plot range and extract calc alpha function

* ImpedanceController (estimated force and external objects)

  * [idl/ImpedanceControllerService.idl,rtc/ImpedanceController/ImpedanceController.cpp,rtc/ImpedanceController/ObjectTurnaroundDetector.h] Add tuning parameter for time count after object turnaround detection.
  * [idl/StabilizerService.idl, rtc/Stabilizer/ZMPDistributor.h, rtc/Stabilizer/Stabilizer.*] change variable type of cp_check_margin
  * [rtc/ImpedanceController/JointPathEx.cpp] Fix bug of interlocking joint. Initialize matrix by zero setting.
  * [rtc/ImpedanceController/ImpedanceController.cpp] Print impedance control parameter when DEBUGP controlled by debugLevel.
  * [idl/ImpedanceControllerService.idl,rtc/ImpedanceController/ImpedanceController*, ObjectTurnaroundDetector.h] Return object turnaround detector mode while checking.
  * [idl/ImpedanceControllerService.idl, rtc/ImpedanceController/Impedance*] Add get function for estimated force and moment
  * [idl/ImpedanceControllerService.idl, rtc/ImpedanceController/*] Add idl service functions for object turnaround detector.
  * [rtc/ImpedanceController/ObjectTurnaroundDetector.h] Add axis and update params
  * [rtc/ImpedanceController/ObjectTurnaroundDetector.h] Add counter and fix checking
  * [rtc/ImpedanceController/*] Add ObjectTurnaroundDetector and tests
  * [idl/ImpedanceControllerService.idl, rtc/ImpedanceController/Impedance*] Add get function for estimated force and moment
  * [idl/ImpedanceControllerService.idl, rtc/ImpedanceController/*] Add idl service functions for object turnaround detector.
  * [rtc/ImpedanceController/ObjectTurnaroundDetector.h] Add axis and update params
  * [rtc/ImpedanceController/ObjectTurnaroundDetector.h] Add counter and fix checking
  * [rtc/ImpedanceController/*] Add ObjectTurnaroundDetector and tests
  * [rtc/ImpedanceController/JointPathEx.*] Add interlocking joint usage. Add interlocking joint component to jacobian and workspace velocity.

* EmergencyStopper

  * add test for emergency stop of wrench in samplerobot_emergency_stopper.py
  * connect data ports of wrenches for EmergencyStopper.
  * interpolate wrenches according to emergency_mode.
  * connect servoState from rh to es.
  * add input/output dataport for reference force sensors in EmergencyStopper

* GaitGenerator

  * [rtc/GaitGenerator.h] Add get function for cog vel and cog acc

* ThermoLimitService

  * [idl/ThermoLimiterService.idl, rtc/ThermoLimiter/ThermoLimiter.*, rtc/ThermoLimiter/ThermoLimiterService_impl.*] enable to set and get ThermoLimiter parameters
  * [ThermoLimiter/ThermoLimiter.cpp] decrease debug messages

* PDController

  * [rtc/PDcontroller/PDcontroller.cpp] Add check for m_robot in PDcontroller (https://github.com/fkanehiro/hrpsys-base/issues/796)
  * [rtc/PDcontroller/PDcontroller.*, sample/SampleRobot/SampleRobot.conf..in] Add torque limit ratio for PDcontroller simulation.
  * [PDcontroller] read gain file at onActivated
  * [rtc/PDcontroller/PDcontroller.*] Remove unused joint reading and add debugLevel and debug print
  * [rtc/PDcontroller/PDcontroller.*] Initialize pdgain and joint angles in onExecute to use bindParameter
  * [rtc/PDcontroller/CMakeLists.txt, PDcontroller.*] Add tlimit based on ModelLoader climit.
  * [PDcontroller] initialize reference angle with current angle at onActevated()

* GraspController

  * [rtc/GraspController/GraspController.cpp] Move to idle mode when servo on/off deactivation

* KalmanFilter

  * [KalmanFilter] add time stamp to output of Kalmanfilter

* SoftErrorLimiter

  * [rtc/SoftErrorLimiter/SoftErrorLimiter.cpp] Limit joint angles in one for loop
  * [rtc/SoftErrorLimiter/SoftErrorLimiter.cpp] Move comments for joint/link
  * [rtc/SoftErrorLimiter/SoftErrorLimiter.cpp] Remove unused variable
  * [rtc/SoftErrorLimiter/SoftErrorLimiter.cpp] Update limitation considering vel, pos, err at once
  * [rtc/SoftErrorLimiter/SoftErrorLimiter.cpp] Limitation by llimit and ulimit to approach valid joint range when (llimit > m_qRef.data[i] && prev_angle[i] <= m_qRef.data[i]) or ( ulimit < m_qRef.data[i] && prev_angle[i] >= m_qRef.data[i] )
  * [rtc/SoftErrorLimiter/SoftErrorLimiter.cpp] Store total last output as prev_angle

* TorqueFilter

  * [TorqueFilter/testIIRFilter.cpp] fix header file to pass qnx
  * [rtc/TorqueFilter/CMakeLists.txt] Add cmake test for testIIRFilter
  * [rtc/TorqueFilter/testIIRFilter.cpp] Enable test for hrp::Vector3
  * [rtc/TorqueFilter/*IIRFilter*, rtc/TorqueFilter/Stabilizer.cpp, ZMPDistributor.h] Initialize value in constructor
  * [rtc/TorqueFilter/testIIRFilter.cpp, CMakeLists] Add test for IIR filter

* ServoController

  * use 0x... format instead of binary format 0b... Fixes (`#868 <https://github.com/fkanehiro/hrpsys-base/issues/868>`_ )
  * [rtc/ServoController/ServoSerial.h, CMakeLists.txt] Check gcc version >= 4.3 for binary format integer constant. (For forl old ubuntu `#854 <https://github.com/fkanehiro/hrpsys-base/issues/854>`_ )

* Contributors: Eisoku Kuroiwa, Fumio KANEHIRO, Hervé Audren, Isaac IY Saito, Kei Okada, Shunichi Nozawa, Takasugi Noriaki, Yohei Kakiuchi, Yosuke Matsusaka, Yuta Kojio, Masaki Murooka, jenkinshrg

315.7.0 (2015-08-19)
--------------------

Stable RTCs
===========

* [doc] Remove old info from downstream pkg
* [CMakeLists.txt] Build 3rdparty directory
* [3rdparty] Add 3rdparty directory based on https://github.com/fkanehiro/hrpsys-base/pull/683 discussion (currently for qpOASES)
* [test/test-samplerobot-el.test] Increase rostest execution time
* [.travis.sh] Check make test
* [.travis] add more information on test matrix see https://github.com/fkanehiro/hrpsys-base/pull/363#issuecomment-122634139

* SequencePlayer

  * [sample/SampleRobot/samplerobot\_*.py] Direct printing message to stderr to visualize in rostest results.
  * [sample/SampleRobot/samplerobot_sequence_player.py] Fix invalid length of joint angle function of groups
  * [sample/SampleRobot/samplerobot_sequence_player.py] Direct printing message to stderr to visualize in rostest results.
  * [rtc/SequencePlayer/seqplay.cpp] Fix typo in print message
  * [rtc/SequencePlayer/SequencePlayer*, rtc/SequencePlayer/seqplay*] Add checking of length of argument joint angles for setJointAnglesOfGroups and setJointAnglesSequenceOfGroup and update related function arguments.
  * [sample/SampleRobot/samplerobot_stabilizer.py, samplerobot_remove_force_offset.py, samplerobot_kalman_filter.py, samplerobot_auto_balancer.py] Check hrpsys version for unstable rtc testing

* CollisionDetector

  * [test/test-samplerobot-collision.py,test-samplerobot-datalogger.py] Add unittest for collision and datalogger
  * [sample/SampleRobot/samplerobot_collision_detector.py, samplerobot_data_logger.py] Use functions in hrpsys_config.py instead of idl functions
  * [sample/SampleRobot/samplerobot_collision_detector.py] Add assert for unittesting of collision check. (Comment out collision mask sample because it requires conf change).

* SoftErrorLimitter

  * [rtc/SoftErrorLimiter/SoftErrorLimiter.cpp] Fix bug of Velocity limitation in https://github.com/fkanehiro/hrpsys-base/pull/726
  * [sample/SampleRobot/samplerobot_soft_error_limiter.py] Check hrpsys version
  * [rtc/SoftErrorLimiter/JointLimitTable., rtc/ImpedanceController/JointPathEx*, rtc/SoftErrorLimiter/CMakeLists.txt, rtc/SoftErrorLimiter/SoftErrorLimiter.h] Move limit table codes to separated file.
  * [sample/SampleRobot/samplerobot_soft_error_limiter.py] Add position limit testing
  * [test/test-samplerobot-el.*, sample/SampleRobot/samplerobot_soft_error_limiter.py] Add rostest for soft error limiter
  * do not check position/limit error when lower limit and upper limit is same


Unstable RTCs
=============

* [python/hrpsys_config.py] Enable thermolimiter and thermoestimator (in Unstable RTC)
* [test/test-samplerobot.test, test-samplerobot-*.py] Add rostests for unstable rtcs.
* [test/test-samplerobot.test] Add data logger and collision detector tests to samplerobot rostest.
* [CMakeLists.txt, rtc/[AutoBalancer, ImpedanceController, Stabilizer]/CMakeLists.txt] Add enable_testing to toplevel cmake and add add_test for impedance, autobalnacer, and stabilizer examples

* AutoBalancer (Support 4/multi leg  mode)

  * [GaitGenerator.cpp] fix bug: keep align the order of names and coordinates of foostep_nodes_list.front()
  * [AutoBalancer.cpp, GaitGenerator.*] add multi_mid_coords function to calculate a midcoords of multi coordinates in fixLegToCoords, get_swing_support_mid_coords and stopWalking
  * [AutoBalancer.cpp] use leg_names instead of "rleg" or "lleg"
  * [GaitGenerator.*] use leg_type_map in order to convert between leg_type and name
  * [AutoBalancer.cpp, GaitGenerator.h] move leg_type_map to gait_generator
  * [GaitGenerator.*] rename get_support_leg_types_from_footstep_nodes to calc_counter_leg_types_from_footstep_nodes
  * [GaitGenerator.*] move get_support_leg_types_from_footstep_nodes to gait_generator
  * [AutoBalancer.cpp, GaitGenerator.h] fix return type of get_dst_foot_midcoords from std::vector<coordinates> to coordinates because we need the reference coordinates
  * [GaitGenetarot.*] keep swing_legs at update_leg_steps
  * [AutoBalancer.cpp] consider some variable life times
  * [AutoBalancer.cpp, GaitGenerator.*, testGaitGenerator.cpp] use step_node instead of coords because we need to align the oder of names of legs and coords of legs
  * [AutoBalancer.cpp, GaitGenerator.*, testGaitGenerator.cpp] fix variable names of legs and corresponding method names
  * [GaitGenerator.h] add hints for the second template argument of boost::assign::list_of at a constructor initialization phase
  * [GaitGenerator.*] use std::count_if instead of boost::count_if since HRP2 inside PC does not support boost::count_if
  * [GaitGenerator.*] use vector class functions to get errors : http://qiita.com/ota42y/items/f2067f6b81dd15bca95a
  * [AutoBalancer.cpp] improve startWalking for multiple legs
  * [AutoBalancer.cpp, GaitGenerator.*, testGaitGenerator.cpp] improve go_pos_param_2_footstep_nodes_list for multiple legs
  * [AutoBalancer.cpp, GaitGenerator.*, testGaitGenerator.cpp] remove an unused argument of go_pos_param_2_footstep_nodes_list
  * [AutoBalancer.cpp, GaitGenerator.h] fix indent
  * [AutoBalancer.cpp, GaitGenerator.*, testGaitGenerator.cpp] add an argument of go_pos_param_2_footstep_nodes_list to set start_ref_coords no matter which gait we choose
  * [GaitGenerator.*] improve get_swing_legs of leg_coords_generator for multiple legs
  * add outport for capture point
  * [GaitGenerator.cpp/.h] extend append_go_pos_step_nodes to get an argument of multiple legs
  * [AutoBalancer.cpp, GaitGenerator.cpp/h, testGaitGenerator.cpp] add a variable named all_limbs which stands for candidates of contact legs
  * [GaitGenerator.cpp, .h] fix a function name to follow the naming rule
  * [GaitGenerator.cpp] use count_if for multiple legs
  * [GaitGenerator.cpp] use min max functions for the stride limits
  * [sample/SampleRobot/samplerobot_auto_balancer.py] Add attitude check to auto balancer test
  * do not use boost::remove_erase_if() because it is too new for old systems
  * remove undefined function
  * I will squash this commit: Update variable names following the naming rule
  * extend "std::vector<step_node> footstep_node_list" to "std::vector< std::vector<step_node> > footstep_node_list_list" for N leg walk
  * replace hard-cording value "2" to leg_names.size() or leg_pos.size()
  * fix return type
  * [rtc/AutoBalancer/testPreviewController.cpp] Add use_gnuplot argument for testPreviewController
  * [rtc/AutoBalancer/testGaitGenerator.cpp] Pass check results to return code
  * [rtc/AutoBalancer/testGaitGenerator.cpp] Fix indent for testGaitGenerator
  * [rtc/AutoBalancer/testGaitGenerator.cpp] Add value checking for testGaitGenerator. Currently zmp error and zmp diff are checked
  * [rtc/AutoBalancer/hrpsys_AutoBalancer_GaitGenerator_memo.pptx] Update memo slide to add footstep overwriting
  * [rtc/AutoBalancer/AutoBalancer.txt] Add url linking to AutoBalancer GaitGenerator memo slide
  * [sample/SampleRobot/samplerobot_auto_balancer.py] Update demoGaitGeneratorOverwriteFootsteps
  * [python/hrpsys_config.py] Add setfootsteps wrapper funcs to hrpsys_config.py
  * [rtc/AutoBalancer/AutoBalancer.cpp, rtc/AutoBalancer/GaitGenerator.*] Enable to consider overwrite footstep index in footstep overwriting
  * [rtc/AutoBalancer/AutoBalancer.cpp] Remove unused variable and return current footstep index
  * [rtc/AutoBalancer/AutoBalancer.*] Add arguments for overwrite_fs_idx
  * [idl/AutoBalancerService.idl, rtc/AutoBalancer/AutoBalancerService_impl.*] Add overwrite footstep index to setFootSteps and getRemainingFootstepSequence
  * [idl/AutoBalancerService.idl,rtc/AutoBalancer/GaitGenerator.h] Do not return current support leg from getRemainingFootstepSequence
  * [rtc/AutoBalancer/hrpsys_AutoBalancer_GaitGenerator_memo.pptx] Add documentation and figures for explanation of AutoBalancer and GaitGenerator
  * [rtc/AutoBalancer/GaitGenertor.cpp] Use overwrite_footstep_node_list
  * [samples/SampleRobot/samplerobot_auto_balancer.py] Add sample for footstep overwriting
  * [rtc/AutoBalancer/AutoBalancer.cpp] Enable to pass footstep overwriting from outside of Autobalancer RTC.
  * [rtc/AutoBalancer/GaitGenerator.*] Enable to overwrite current footsteps
  * [AutoBalancer.cpp] remove unused if else
  * [idl/AutoBalancerService.idl, AutoBalancer.cpp, GaitGenerator.h] add a parameter "leg_names" to AutoBalancerParam

* Stabilizer

  * [rtc/Stabilizer/CMakeLists.txt] Add subdirectory for qpoases linking
  * [rtc/Stabilizer/CMakeLists.txt] Use qpOASES installed by 3rdparty directory.
  * [sample/SampleRobot/samplerobot_stabilizer.py, SampleRobot.torque.xml.in] Add check for attitude to st test
  * [idl/StabilizerService.idl, rtc/Stabilizer/Stabilizer.*] Use vector3 for eefm_pos_time_const_support
  * [idl/StabilizerService.idl] enable to set contact decision threshold
  * [sample/SampleRobot/samplerobot_stabilizer.py] Add assert check for stabilizer
  * [rtc/Stabilizer/testZMPDistributor.cpp] Add use_gnuplot argument for testZMPDistributor

* VoxelGridFilter

  * removes an unused file
  * adds a new component,
  * fix name : foot_midcoords -> ref_coords

* EmergencyStopper

  * add mutex lock when writing is_emergency_mode variable
  * add test-samplerobot-emergency.py and test EmergencyStopper in test-samplerobot.test
  * add test code of EmergencyStopper function
  * go to release_mode when deactivated in EmergencyStopper
  * fix format specifier of size_t
  * support multiple zmp offsets input to PreviewController
  * use switch instead of if to judge emergency_check_mode
  * beep on emergency mode

* TorqueController

  * [TorqueController] Fix typo, emergencyController -> normalController, in disable error message
  * [TorqueController] Add enable/disable methods to MotorTorqueController
  * [TorqueController] Add enable flag to MotorTorqueController to manage activity of both normal and emergency controller
  * [TorqueController] Add instance name to error prefix
  * [TwoDofController] Move error_prefix to TwoDofControllerInterface
  * [TwoDofController] Add instnace_name to error message
  * [TorqueFilter] Add instance name to error message of TorqueFilter

* GraspContrller

  * [rtc/GraspController/GraspController.cpp] Add debug message to grasp controller start/stop grasp and add instance
    name for print message

* ImpedanceController

  * [sample/SampleRobot/samplerobot_impedance_controller.py] Fix typo in print message.
  * [rtc/ImpedanceController/testImpedanceOutputGenerator.cpp] Add arguments for plotting and printing usage.

* KalmanFilter

  * [sample/SampleRobot/samplerobot_kalman_filter.py] Add exception if no plot is available.
  * [sample/SampleRobot/samplerobot_kalman_filter.py] Add check and assertion for sample kalmanfilter

* RemoveForcesSensorLinkOffset

  * [rtc/RemoveForceSensorLinkOffset/RemoveForceSensorLinkOffset.cpp] Return false for invalid argument
  * [sample/SampleRobot/samplerobot_remove_force_offset.py] Add value check for RMFO

* DataLogger

  * [sample/SampleRobot/samplerobot_data_logger.py] Add assert for unittesting of data logger.
  * [sample/SampleRobot/samplerobot_data_logger.py, samplerobot_soft_error_limiter.py] Define examples as demo functions

* Contributors: Fumio KANEHIRO, Isaac IY Saito, Kei Okada, Masaki Murooka, Shunichi Nozawa, Yuta Kojio, Eisoku Kuroiwa, Iori Kumagai

315.6.0 (2015-07-10)
--------------------

Stable RTCs
===========

* SequencePlayer

  * Rename arguments and local variables remain_t, x, v, and a because these are same name as member variables
  * Add comments to interpolator
  * [SequencePlayer/seqplay.cpp] clearJointAnglesOfGroup use online = true to clear remain_t
  * Connect seq port to monitor seq interpolation

* python/hrpsys_config.py

  * Add HardEmergencyStopper RTC to stop almost all rtc motion
  * Add check for rmfo-st connection
  * Use rmfo off sensor values in st
  * Remove seq data logging which can replaced by StateHolder data (reported in https://github.com/fkanehiro/hrpsys-base/issues/594)

* test

  * [test/test-samplerobot.test b/test/test-samplerobot.test] wrenches is available from 315.2.0
  * [test/test-samplerobot.test b/test/test-samplerobot.test] update timelimit to 120

* sample

  * [sample/SampleRobot/samplerobot_sequence_player.py] check  seq rtc version for executing tests
  * [sample/SampleRobot/samplerobot_sequence_player.py, test/test-samplerobot-sequence.py, test-test-samplerobot.test] add samplerobot_sequence_player to test case
  * [sample/SampleRobot/samplerobot_sequence_player.py] add demoSetJointAnglesSequence() demoSetJointAnglesSequenceOfGroup()
  * [sample/SampleRobot/samplerobot_sequence_player.py] add test code for override and clear function to demoSetJointAngles() demoSetJointAnglesOfGroup(), demoSetJointAnglesOfGroup()
  * [sample/SampleRobot/samplerobot_sequence_player.py] add setSetJointAnglesOfGroup() and check results
  * [sample/SampleRobot/SampleRobot.torque.xml.in] Use RUNGE_KUTTA for torque simulation
  * [sample/SampleRobot/samplerobot_stabilizer.py] Add tpcc eefm st sample
  * [sample/SampleRobot/SampleRobot.PDgain.dat] Fix SampleRobot PD gain
  * Fix stabilizer sample
  * Update sample for stepparam change
  * Use functions defined in hrpsys_config.py
  * Divide samples into small sample functions
  * Add emergency stop and remain fot step sample

* lib/util/Hrpsys.h

  * [lib/util/Hrpsys.h] add atoi
  * [lib/util/Hrpsys.h] add header file for QNX compile

* [doc] Elaborate package overview

Unstable RTCs
=============

* AutoBalancer

  * add kick-test to testGaitGenerator.cpp
  * Update AutoBalancer.cpp enable to stop with one lne
  * check capture point to detect falling down
  * Do not set is_stop_mode for testing
  * Add emergency stop mode and release mode for AutoBalancer
  * Add emergency stop port for autoBalancer to stop walking
  * enable to step with one leg
  * add height check to cycloid_delay_kick_hoffarbib_trajectory_generator and changed initerpolation point
  * modify cycloid_delay_kick_trajectory_generator by adding start_rot
  * modify orbit : enabled to modify kick_offset by function
  * 1 control loop by default for default_retrieve_time
  * Add retrieving after emergency stop
  * add swing leg orbit type :CYCLOIDDELAYKICK
  * Update single footstep support coords
  * Add support and swing leg coords to lcg
  * Add test13 to argument
  * Add test for arbitrary leg switching
  * Revert previous estop commit
  * Fix paren and indent
  * Update rmfo documentation. off_xx equal to xx
  * Update footstep calculation. Push refzmp list immediately.
  * Fix calculation of current remain time and update sample
  * Use footstep_node_list step_time in refzmp_generator
  * Rename leg_coords_generator _dt => dt
  * Remove one_step_len and use foot step time in footstep_node_list
  * Use step time from footstep node list in leg_coords_generator
  * Use total step count from footstep_node_list
  * Set step parameters for foot step node list
  * Set height, toe_angle, heel_angle to 0 at initial and final foot step
  * Add test for changing step param
  * Add set foot steps function
  * Add step_time for each step parameter. Currently interface are provided and step_time is not used in GaitGenerator
  * Fix go pos 000 discontinuous last foot.
  * Update overwrite refzmp
  * Remove unused function is_swinging_leg
  * Rename variables for lcg and add comments
  * Separate gait generator type, class, functions from gait_generator class
  * Add function to get remaining foot steps
  * Add emergency stop interface for walking. Currently, velocity mode is supported.

* Stabilizer

  * add cp_check_margin to avoid hard coding
  * Add emregency check mode for st
  * Fix st cop check to strong constraint
  * Use is_emerency for emergency signal checking
  * Separate state calculation function for emergency signal
  * Add add_subdirectory for qpOASES
  * Fix calculation of stop queue and current seq state resetting
  * Svn co and build qpOases. Disabled by default
  * Add foot rot test
  * Add jaxonred zmp sample and parse args
  * Add test class for ZMPDistributor
  * plot alpha in ZMPdistributor check
  * Set outside margin
  * Add outside margin
  * Enable to set cop check margin
  * Add debug message and check both cop on ground
  * Add check cop outside
  * Add COPInfo including total moment x, y, and total force at each end effectors
  * Add documentation for test samples

* ProjectGenerator

  * add a note in read to use a new program instead of this one

* EmergencyStopper

  * add out port for emergency_mode
  * Fix m_stop_posture setting to be able to change retrieve time
  * Add EmergencyStopper Param and add getter and setter
  * Set interpolator and add message
  * Add emergency signal port to ES and ST and connect them (currently signal writing is comment-outed).
  * implement EmergencyStopper and add sample script
  * add source files of EmergencyStopper rtc

* PDcontroller

  * Add warning for too short pdgain

* Contributors: Eisoku Kuroiwa, Isaac IY Saito, Kei Okada, Masaki Murooka, Shunichi Nozawa, Takasugi Noriaki, Yuta Kojio

315.5.0 (2015-06-10)
--------------------

Stable RTCs
===========

* rtc/SequencePlayer

  * [idl/SequencePlayerService.idl, SequencePlayer.{h,cpp}, SequencePlayerService_impl.{h,cpp}, seqplay.{h,cpp}] add clearJointAngles and clearJointAnglesOfGroup()
  * [seqplay.cpp] push current data to the queue
  * [idl/SequencePlayerService.idl, SequencePlayer.{h,cpp}, SequencePlayerService_impl{h,cpp}, seqplay.{h,cpp}] add setJointAnglesSequenceFull()
  * [interpolator.h] add dimension() returns dim
  * [interpolator.{cpp,h}] add setGoal(double *, double, bool = true)
  * [python/hrpsys_config.py, SequencePlayer.{h,cpp}, SequencePlayerService_impl.cpp, seqplay.{h,cpp}] add setJointAnglesSequenceOfGroup
  * [SequencePlayer.cpp] use setJointAnglesSequence for setJointAngles
  * [idl/SequencePlayerService.idl, python/hrpsys_config.py, SequencePlayer.{h,cpp}, SequencePlayerService_impl.{h,cpp}, seqplay.{h,cpp}] add setJointAnglesSequence wcich takes Sequence of JointAngles and overwrite current motion

* rtc/StateHolder

  * [StateHolder.cpp] Reset StateHolder wrench in goActual. Currently zero is assumed.

* rtc/RobotHardware

  * [RobotHardware.cpp,robot.{h,cpp}] adds check of joint command acceleration
  * [robot.{h,cpp}] use imu coordinate for reference gravity
  * [robot.{h,cpp}] memorize the previous joint commands
  * [RobotHardware.cpp,robot.{h,cpp}] modifies checkJointCommands() to check joint command velocities
  * [RobotHardware.cpp,robot.{h,cpp}] Revert "changes checkJointCommands() to check joint command velocities"
  * [RobotHardware.cpp,robot.{h,cpp}]changes checkJointCommands() to check joint command velocities

* python

  * [rtm.py] fixes a mistake in a debug message
  * [hrpsys_config.py] add more features to logger
  * [hrpsys_config.py] Add function to start and stop default unstable controllers (st, abc, ic)
  * [hrpsys_config.py] Add kinematics_only_mode flag to hcf
  * [rtm.py] add more error messages on activate and connnect components
  * [rpsyspy] chekc if RobotHadwareService has joint angle (to avoid confusion such as longfloor)
  * [hrpsys_config.py] import waitInputConfirm from waitInput.py in hrpsys_config.py to resolve function name

* [package.xml] add deped to graphviz for dot program fix #629

Unstable RTCs
=============

* sample

  * [environments/DRCFinalStair.wrl] Add DRC final stair with sloped ground
  * [environments/DRCTestfieldStair.wrl] Add drc testfield stair
  * [environments/DRCTestbedTerrainJPBlock.wrl,environments/DRCTestbedTerrainUSBlock.wrl] Update location of each block of terrain models
  * [environments/DRCTestfieldTerrain.wrl] Add testfield drc terrain vrml file
  * [SampleRobot/samplerobot_impedance_controller.py] Add print message for impedance controller sample
  * [SampleRobot/samplerobot_impedance_controller.py] Add tracking check sample to impedance controller sample
  * [SampleRobot/CMakeLists.txt, ampleRobot/SampleRobot.kinematicsonly.xml.in] Add kinematics only mode Project file for sample robot

* rtc/AverageFilter

  * [AverageFilter.{h,cpp}] adds a configuration parameter, dilation

* rtc/CameraImageLoader

  * adds a new component, CameraImageLoader

* rtc/UndistortImage

  * [UndistortImage.cpp] fixes a bug in onExecute()
  * [UndistortImage.cpp] checks if the calibration file exists
  * adds a new component UndistortImage

* rtc/SoftErrorLimiter

  * [SoftErrorLimit.cpp] display limit violation message for 0.2  period
  * [SoftErrorLimit.cpp] check velocity limit using limit - 0.01 deg, if we use limit = limit, then it will fail at RobotHardware
  * [SoftErrorLimit.cpp] display error message in the first time, see #498

* rtc/ImpedanceController

  * [ImpedanceController.cpp] write debug message
  * [ImpedanceController.cpp] Stop impedance controller which is active in onDeactivated
  * [idl/ImpedanceControllerService.idl, ImpedanceControllerService_impl.{cpp,h}, ImpedanceController.h] Add start and stop impedance without waiting
  * [ImpedanceOutputGenerator.h] Use new version impedance output generation by default which considerstarget acceleration
  * [ImpedanceOutputGenerator.h] Add comment for ImpedanceOutputGenerator
  * [ImpedanceOutputGenerator.h] Fix subtraction of current and target rotation
  * [ImpedanceController/testImpedanceOutputGenerator.cpp] Add plotting of rotation
  * [ImpedanceOutputGenerator.h] Add calcTargetVelocityNew.
  * Separate calculation of impedance control output. This commit will not change ImpedanceController behavior
  * [ImpedanceController.cpp,ImpedanceOutputGenerator.h] Update variables in ImpedanceOutputGenerator. Use output, target, and current
  * [idl/ImpedanceControllerService.idl,ImpedanceController.{h,cpp},sample/SampleRobot/samplerobot_impedance_controller.py] Enable to fix ref force frame (experimental codes)

* rtc/CollisionDetector

  * (Code Refactoring) [rtc/CollisionDetector] enable to work both collision_mask and use_collision_limb
  * [CollisionDetector.{h,cpp}] enable to work both collision_mask and use_collision_limb
  * [sample/SampleRobot/samplerobot_collision_detector.py] Add example for desired collision behavior
  * [CollisionDetector.{h,cpp}] add use_limb_collision mode (VERY EXPERIMENTAL)
  * [sample/SampleRobot/samplerobot_collision_detector.py]Add collision mask test added in  https://github.com/fkanehiro/hrpsys-base/pull/627
  * [CollisoinDetector.{h,cpp}] add m_collision_mask property

* rtc/OccupancyGridMap3D

  * [OccupancyGridMap3D.cpp] emits update signal at the end of clear()
  * [idl/OGMap3DService.idl,OccupancyGridMap3D.{h,cpp},OGMap3DService_impl.{h,cpp}] adds clear() to OGMap3DService

* rtc/KalmanFilter

  * [idl/KalmanFilterService.idl, KalmanFilter.{h,cpp}] Add kalman filter offset parameters

* rtc/Stabilizer

  * [Stabilizer.cpp] Fix invalid st index
  * [Stabilizer.cpp] Fix pos_ctrl frame in stabilzier
  * [environments/DRCTestfieldStair.wrl, Stabilizer.cpp] Use sequence for foot_origin_offset and fix printing
  * [idl/StabilizerService.idl, Stabilizer.cpp] Enable to set foot origin offset in ST
  * [idl/StabilizerService.idl, Stabilizer.cpp] Add ee error pos and rot compensation and enable to set is_ik_enable
  * [Stabilizer.cpp] Make slow stabilizer sync
  * [Stabilizer.cpp] Use moment at EE instead of COP
  * [Stabilizer.cpp] Do not use local y COP offset
  * [ZMPDistributor.h] Update Stabilizer QP foot moment force distribution to use alpha parameter
  * [idl/StabilizerService.idl] Add documentation for st param
  * [idl/StabilizerService.idl, Stabilizer.{h,cpp}] Enable to change transition time of ST
  * [ZMPDistributor.h] Do not use foot distribution coords in force moment distribution and limit alpha
  * [idl/StabilizerService.idl, Stabilizer.{h,cpp}, ZMPDistributor.h, testZMPDistributor.cpp] Add alpha force moment distribution lpf
  * [idl/StabilizerService.idl, Stabilizer.cpp, ZMPDistributor.h] Enable to change force moment distribution
  * [ZMPDistributor.h] Update force moment distribution based on reference zmp alpha parameter.

* rtc/AccelerationChecker

  * [AccelerationChecker.cpp] add stdio.h, hope this works on qnx
  * [AccelerationChecker.cpp] adds a configuration variable, print

* rtc/AutoBalancer

  * [idl/AutoBalancerService.idl, AutoBalancer.cpp, GaitGenerator.{h,cpp}] Enable to set toe heel angle during setFootSteps
  * [AutoBalancer.cpp] Fix org origin and target origin in adjust function
  * [idl/AutoBalancerService.idl, AutoBalancer.cpp] Add adjust footstep for walking
  * [idl/AutoBalancerService.idl, AutoBalancer.cpp, GaitGenerator.{h,cpp}] Enable to set go pos finalize footsteps num
  * [GaitGenerator.cpp] Remove unused debug message
  * [GaitGenerator.{h,cpp}] Use swing_rot_ratio for swing rot calculation and swing_ratio to get swing phase
  * [GaitGenerator.{h,cpp}] directly use swing_ratio
  * [GaitGenerator.{h,cpp}] Rename rot_ratio -> foot_midcoords_ratio
  * [idl/AutoBalancerService.idl, AutoBalancer.cpp, AutoBalancerService_impl.{h,cpp}, GaitGenerator.h] Add waitFootStepsEarly
  * [GaitGenerator.{h,cpp}] Use setGoal instead of go in gait generator interpolators
  * [idl/AutoBalancerService.idl, AutoBalancer.{h,cpp}] Enable to set leg default offset position
  * [testGaitGenerator.cpp] Add foot rotation testing
  * [GaitGenerator.{h,cpp}] Use interpolator for foot rotation calculation
  * [AutoBalancer.cpp, GaitGenerator.h, testGaitGenerator.cpp] Update toe_heel_phase_ratio enable to be set
  * [testGaitGenerator.cpp] Update for toe heel trajectory. Display toe heel trajectory and add test for it
  * [idl/AutoBalancerService.idl, AutoBalancer.cpp, GaitGenerator.{cpp,h}, testGaitGenerator.cpp] Add swing foot trajectry by combining cycloid and delay hoff arbib
  * [testGaitGenerator.cpp] Fix plot size of foot trajectory
  * [idl/AutoBalancerService.idl, AutoBalancer.cpp, GaitGenerator.h, testGaitGenerator.cpp] Add weighting parameter for final path of delay_hoffarbib_trajectory_generator
  * [GaitGenerator.{h,cpp}] Fix zmp transition in second and second_last phase
  * [GaitGenerator.h, testGaitGenerator.cpp] Fix gg param setting and plot cart zmp
  * [testGaitGenerator.cpp] Generate graph eps file, fix test9 foot steps, and arg setting
  * [testGaitGenerator.cpp] Add parsing of GaitGenerator params
  * [AutoBalancer.cpp, GaitGenerator.{h,cpp}] Add print_param for GaitGenerator
  * [testGaitGenerator.cpp] Add foot velocity trajectories in GaitGenerator test
  * [GaitGenerator.{h,cpp}] Use calculation of default_double_support_static_ratio
  * [testGaitGenerator.cpp] Update graph printing and add print messages
  * [idl/AutoBalancerService.idl, AutoBalancer.cpp, GaitGenerator.h] Add default_double_support_static_ratio to keep reference zmp static
  * [idl/AutoBalancerService.idl, AutoBalancer.{h,cpp}] Enable to set zmp transition time for abc
  * [idl/AutoBalancerService.idl, AutoBalancer.{h,cpp}] Enable to set transition time for abc
  * [sample/SampleRObot/samplerobot_auto_balancer.py] Add test for discontinuous sync
  * [AutoBalancer.cpp] clear interpolator before setting
  * [AutoBalancer.cpp] Fix discontinuous when stopping auto balancer after walking

* Contributors: Eisoku Kuroiwa, Fumio KANEHIRO, Kei Okada, Ryohei Ueda, Shunichi Nozawa

315.4.0 (2015-05-04)
--------------------

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
