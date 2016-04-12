/**
\mainpage

\section intro_hrpsys Overview

"hrpsys": Basic RT components and utilities to control robots using <a href=http://www.openrtm.org>%OpenRTM</a>. 
Some notable features:

<ul>
  <li>Real-time capable utilizing OS feature</li>
  <li>Rich interface for kinematic operation</li>
    <li>Manipulator operation by the pose of end-effector (EEF linear interpolation)</li>
    <li>Manipulation by passing the set of joint angles (joint angle interpolation)</li>
    <li>2 interpolation options; linear / minimum jerk</li>
    <li>Preemptive kinematic operation (canceleable and modifyable)</li>
    <li>Preemption can be disabled</li>
  <li>Plugins</li>
    <li>EEF compliance control plugin (using on force-torque sensor)</li>
    <li>EEF virtual force computation plugin (using joint torque values)</li>
    <li>Joint protection plugin</li>
    <li>Self interference detection plugin</li>
    <li>Servo module hand plugin</li>
  <li>Proven operational history</li>
    <li>Development has been active over years at institutes such as the original developer <a href = "http://aist.go.jp/">AIST</a> (National Institute of Advanced Industrial Science and Technology in Japan), <a href = "http://jsk.t.u-tokyo.ac.jp/">JSK Lab at U-Tokyo</a>.</li>
    <li>Dozens of thousands of operational time</li>
    <li>Number of robots are running atop</li>
      <li>Include 4 DRC contender robots</li>
      <li>See the incomplete list of the hrpsys-based robots <a href = "http://wiki.ros.org/hrpsys">here</a></li>
  <li>Active development status; 73 commits during month of June 2015 made by 5 authors</li>
</ul>

\section requirements System requirements

<a href=http://www.ubuntu.com>ubuntu</a> 12.04LTS and 14.04LTS are supported. <br>
<a href=http://www.openrtp.jp/openhrp3>%OpenHRP</a> version 3.1.5 or later is required.<br>
To use python scripts to create RT components, connect ports and get/set properties, <a href=http://www.openrtm.org>OpenRTM-aist-python</a> is also required.<br>

\section contents Contents of the package

<ul>
  <li>\ref hrpsys_config.py "hrpsys python helper library"<br>
  This library helps working on hrpsys manager and plugins.</li>
  <li>\ref rtm.py "rtm python helper library"<br>
  This library helps to create/delete and activate/deactivate RT components and connect/disconnect ports.</li>
  <li>Basic RT components</li>
  <ul>
    <li>\ref AccelerationChecker</li>
    <li>\ref AutoBalancer</li>
    <li>\ref AverageFilter</li>
    <li>\ref Beeper</li>
    <li>\ref CameraImageViewer</li>
    <li>\ref CaptureController</li>
    <li>\ref CollisionDetector</li>
    <li>\ref DataLogger</li>
    <li>\ref ExtractCameraImage</li>
    <li>\ref ForwardKinematics</li>
    <li>\ref GraspController</li>
    <li>\ref HGcontroller</li>
    <li>\ref ImageData2CameraImage</li>
    <li>\ref ImpedanceController</li>
    <li>\ref Joystick</li>
    <li>\ref Joystick2Velocity2D</li>
    <li>\ref Joystick2Velocity3D</li>
    <li>\ref Joystick2PanTiltAngles</li>
    <li>\ref JpegDecoder</li>
    <li>\ref JpegEncoder</li>
    <li>\ref KalmanFilter</li>
    <li>\ref MLSFilter</li>
    <li>\ref NullComponent</li>
    <li>\ref EmergencyStopper</li>
    <li>\ref OGMap3DViewer</li>
    <li>\ref OccupancyGridMap3D</li>
    <li>\ref PCDLoader</li>
    <li>\ref PDcontroller</li>
    <li>\ref PlaneRemover</li>
    <li>\ref RGB2Gray</li>
    <li>\ref Range2PointCloud</li>
    <li>\ref RangeDataViewer</li>
    <li>\ref RangeNoiseMixer</li>
    <li>\ref ReferenceForceUpdater</li>
    <li>\ref RemoveForceSensorLinkOffset</li>
    <li>\ref ResizeImage</li>
    <li>\ref RobotHardware</li>
    <li>\ref SORFilter</li>
    <li>\ref SequencePlayer</li>
    <li>\ref ServoController</li>
    <li>\ref Simulator</li>
    <li>\ref SoftErrorLimiter</li>
    <li>\ref Stabilizer</li>
    <li>\ref StateHolder</li>
    <li>\ref ThermoEstimator</li>
    <li>\ref ThermoLimiter</li>
    <li>\ref TorqueController</li>
    <li>\ref TorqueFilter</li>
    <li>\ref VideoCapture</li>
    <li>\ref Viewer</li>
    <li>\ref VirtualCamera</li>
    <li>\ref VirtualForceSensor</li>
    <li>\ref WavPlayer</li>
  </ul>
  <li>Services provided by basic RT components</li>
  <ul>
    <li>\ref OpenHRP::AutoBalancerService</li>
    <li>\ref OpenHRP::CollisionDetectorService</li>
    <li>\ref OpenHRP::DataLoggerService</li>
    <li>\ref OpenHRP::ExecutionProfileService</li>
    <li>\ref OpenHRP::ForwardKinematicsService</li>
    <li>\ref OpenHRP::GraspControllerService</li>
    <li>\ref Img::CameraCaptureService</li>
    <li>\ref OpenHRP::ImpedanceControllerService</li>
    <li>\ref OpenHRP::KalmanFilterService</li>
    <li>\ref OpenHRP::OGMap3DService</li>
    <li>\ref OpenHRP::RemoveForceSensorLinkOffsetService</li>
    <li>\ref OpenHRP::RobotHardwareService</li>
    <li>\ref OpenHRP::SequencePlayerService</li>
    <li>\ref OpenHRP::ServoControllerService</li>
    <li>\ref OpenHRP::SoftErrorLimiterService</li>
    <li>\ref OpenHRP::StabilizerService</li>
    <li>\ref OpenHRP::StateHolderService</li>
    <li>\ref OpenHRP::TimeKeeperService</li>
    <li>\ref OpenHRP::TorqueControllerService</li>
    <li>\ref OpenHRP::TorqueFilterService</li>
    <li>\ref OpenHRP::VirtualForceSensorService</li>
    <li>\ref OpenHRP::WavPlayerService</li>
  </ul>
  <li>Utilities</li>
  <ul>
    <li>\ref ProjectGenerator "ProjectGenerator"</li>
    <li>\ref hrpsys-simulator "hrpsys-simulator"</li>
    <li>\ref hrpsys-simulator-jython "hrpsys-simulator-jython"</li>
    <li>\ref hrpsys-simulator-python "hrpsys-simulator-python"</li>
    <li>\ref hrpsys-viewer "hrpsys-viewer"</li>
    <li>\ref hrpsys-monitor "hrpsys-monitor"</li>
    <li>\ref hrpsysjy "hrpsysjy"</li>
    <li>\ref hrpsyspy "hrpsyspy"</li>
    <li>\ref python_binding "Python bindings"
  </ul>
  <li>\ref iob.h "IO library"<br>
  IO library provides a standard access method to the robot hardware. A library in this package just provides its dummy implementation. To control your robot with this package, you need to implement the IO library for your robot.</li>
  <li>Execution Context<br>
  This execution context executes RT components in real-time cooperating with the IO library. <a href=http://www.dh.aist.go.jp/jp/research/humanoid/ART-Linux/>ART-Linux</a> or Preemptive Kernel of Linux can be used as a real-time OS.
  </li>
  <li>Eclipse plugin<br>
  This plugin extends GrxUI(Graphical User Interface of OpenHRP3) and enables to monitor the robot status on GrxUI communicating with RT components \ref RobotHardware and \ref StateHolder.</li>
</ul>

\section how_to_monitor How to monitor my robot?

In order to monitor your robot using GrxUI, you need to implement IO library for your robot. Please develop a library which has interface defined in \ref iob.h and replace pre-installed libhrpIo.so with it.

*/
