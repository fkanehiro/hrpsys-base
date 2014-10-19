/**

\page utilities

\section ProjectGenerator ProjectGenerator

generate a template project file 

ProjectGenerator [input files] [options]

<i>input files</i> shold be VRML or COLLADA files.<br>

--output [output file]<br>
&nbsp;&nbsp;Specify output file path (Required)<br>
&nbsp;&nbsp;For example, when output file is <i>test.xml</i>, ProjectGenerator generates <i>test.xml</i>, <i>test.conf</i>, and <i>test.RobotHardware.conf</i>.<br>
--integrate [true or false]<br>
&nbsp;&nbsp;Use forward dynamics mode or kinematics mode.<br>
--dt [dt]<br>
&nbsp;&nbsp;<i>dt</i> is controllers' time step[s].<br>
--timestep [timestep]<br>
&nbsp;&nbsp;<i>timestep</i> is simulator time step[s].<br>
--conf-file-option [conf file option]<br>
&nbsp;&nbsp;<i>conf file option</i> is added to controller's config file such as <i>test.conf</i>.<br>
--robothardware-conf-file-option [robothardware conf file option]<br>
&nbsp;&nbsp;<i>robothardware conf file option</i> is added to robothardware's config file such as <i>test.Robothardware.conf</i>.<br>
--joint-properties [joint properties]<br>
&nbsp;&nbsp;<i>joint properties</i> are properties for each joint. Specify property name and property value.<br>
&nbsp;&nbsp;For example, <i>--joint-properties RLEG_JOINT0.angle,0,RLEG_JOINT1.mode,Torque</i><br>

\section hrpsys-simulator hrpsys-simulator

read a project file and execute simulation

hrpsys-simulator [project file] [options]

-nodisplay execute simulation without display<br>
-realtime execute simulation in real-time<br>
-usebbox use bounding boxes instead of actual geometries<br>
-endless never finish simulation<br>
-showsensors show sensor output<br>
-size size specify initial window size<br>
-no-default-lights turn off default lights<br>
-max-edge-length length[m] divide large triangles which have longer edges than this value<br>
-max-log-length length[s] set length of ring buffer<br>
-exit-on-finish exit this program when the simulation finishes

Note:NameServer and openhrp-model-loader must be running

In order to change timestep (ex 0.002), you need to change SampleRobot.conf to

  dt: 0.002

and SampleRobot.xml to

  <property name="SampleRobot(Robot)0.period" value="0.002"/>
  <property name="timeStep" value="0.002"/>


dt of SampleRobot.conf and XX.period of SampleRobot.xml should be same. 
timeStep of SampleRobot.xml is not larger than these values.

\section hrpsys-simulator-jython hrpsys-simulator-jython

read a project file and execute simulation and a jython script

Note:NameServer and openhrp-model-loader must be running

hrpsys-simulator-jython [project file] [jython script] [options]

-nodisplay execute simulation without display<br>
-realtime execute simulation in real-time<br>
-usebbox use bounding boxes instead of actual geometries<br>
-endless never finish simulation<br>
-showsensors show sensor output<br>
-size size specify initial window size<br>
-bg r g b background color
-max-log-length length[s] set length of ring buffer<br>

\section hrpsys-simulator-python hrpsys-simulator-python

read a project file and execute simulation and a python script

hrpsys-simulator-python [project file] [python script] [options]

nodisplay execute simulation without display<br>
realtime execute simulation in real-time<br>
usebbox use bounding boxes instead of actual geometries<br>
endless never finish simulation<br>
showsensors show sensor output<br>
size [size] set window size<br>
bg [r] [g] [b] background color
max-log-length length[s] set length of ring buffer<br>

Note:NameServer and openhrp-model-loader must be running

\section hrpsys-viewer hrpsys-viewer

visualize a model file and work as a OnlineViewer server

hrpsys-viewer [model file] [-size size]

-size specify initial window size<br>
-no-default-lights turn off default lights<br>
-max-edge-length divide large triangles which have longer edges than this value<br>

Note:NameServer and openhrp-model-loader must be running

\section hrpsys-monitor hrpsys-monitor

monitor robot's status

hrpsys-monitor [project file] [-rh rtcName] [-sh rtcName] [-size size]

-rh name of %RTC which provides OpenHRP::RobotHardwareService (default:RobotHardware0)<br>
-sh name of %RTC which provides OpenHRP::StateHolderService (default:StateHolder0)<br>
-size initial window size<br>
-bg r g b background color

Note:NameServer and openhrp-model-loader must be running

\section hrpsysjy hrpsysjy

Jython interpreter with hrpsys library

hrpsysjy [jython arguments]

\section hrpsyspy hrpsyspy

Python interpreter with hrpsys library

hrpsyspy [python arguments]

\section python_binding Python bindings

A python module hrpsys.so provides python bindings to setup and execute simulations interactively.
*/
