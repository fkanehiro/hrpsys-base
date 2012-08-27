/**
\mainpage

\section intro_hrpsys Overview

Basic RT components and utilities to control robots using <a href=http://www.openrtm.org>%OpenRTM</a>

\section requirements System requirements

<a href=http://www.ubuntu.com>ubuntu</a> 10.04LTS is supported. <br>
<a href=http://www.openrtp.jp/openhrp3>%OpenHRP</a> version 3.1.4 or later is required.<br>
To use jython scripts to create RT components, connect ports and get/set properties, <a href=http://www.jython.org>Jython</a> is also required.<br>
To use python scripts to create RT components, connect ports and get/set properties, <a href=http://www.openrtm.org>OpenRTM-aist-python</a> is also required.<br>

\section contents Contents of the package

<ul>
  <li>\ref rtm.py "Jython and CPython library"<br>
  This library helps to create/delete and activate/deactivate RT components and connect/disconnect ports.</li>
  <li>Basic RT components</li>
  <ul>
    <li>\ref CameraImageViewer</li>
    <li>\ref CollisionDetector</li>
    <li>\ref DataLogger</li>
    <li>\ref ForwardKinematics</li>
    <li>\ref HGcontroller</li>
    <li>\ref ImpedanceController</li>
    <li>\ref Joystick</li>
    <li>\ref Joystick2Velocity2D</li>
    <li>\ref Joystick2Velocity3D</li>
    <li>\ref Joystick2PanTiltAngles</li>
    <li>\ref JpegDecoder</li>
    <li>\ref NullComponent</li>
    <li>\ref OccupancyGridMap3D</li>
    <li>\ref OGMap3DViewer</li>
    <li>\ref Range2PointCloud</li>
    <li>\ref RobotHardware</li>
    <li>\ref SequencePlayer</li>
    <li>\ref Simulator</li>
    <li>\ref StateHolder</li>
    <li>\ref Viewer</li>
    <li>\ref VideoCapture</li>
    <li>\ref VirtualCamera</li>
    <li>\ref WavPlayer</li>
  </ul>
  <li>Services provided by basic RT components</li>
  <ul>
    <li>\ref OpenHRP::DataLoggerService</li>
    <li>\ref OpenHRP::ExecutionProfileService</li>
    <li>\ref OpenHRP::ForwardKinematicsService</li>
    <li>\ref OpenHRP::ImpedanceControllerService</li>
    <li>\ref OpenHRP::OGMap3DService</li>
    <li>\ref OpenHRP::RobotHardwareService</li>
    <li>\ref OpenHRP::SequencePlayerService</li>
    <li>\ref OpenHRP::StateHolderService</li>
    <li>\ref OpenHRP::TimeKeeperService</li>
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
