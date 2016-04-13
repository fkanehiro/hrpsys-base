[![Build Status](https://travis-ci.org/fkanehiro/hrpsys-base.svg?branch=master)](https://travis-ci.org/fkanehiro/hrpsys-base)

# Install

For Install, See [INSTALL](https://raw.githubusercontent.com/fkanehiro/hrpsys-base/master/INSTALL)

# API

For API, See [API Doc](http://fkanehiro.github.io/hrpsys-base/)

# Samples

For Samples, See [Samples](https://github.com/fkanehiro/hrpsys-base/tree/master/sample)

For related informations:

ROS integration with hrpsys-base [ROS wiki hrpsys page](http://wiki.ros.org/rtmros_common/Tutorials/GettingStart)

Example case of system configurations is written in the following paper Fig.3:
Shunichi Nozawa, Eisoku Kuroiwa, Kunio Kojima, Ryohei Ueda, Masaki Murooka, Shintaro Noda,
Iori Kumagai, Yu Ohara, Yohei Kakiuchi, Kei Okada, Masayuki Inaba, 
"Multi-layered real-time controllers for humanoid's manipulation and locomotion tasks with emergency stop",
2015 IEEE-RAS 15th International Conference on Humanoid Robots (Humanoids), pp.381-388, 2015

# Directories
Main directories are here (for more information, please see API Doc):
## 3rdparty
3rd party software installation cmake and settings.
## ec
For execution context implementations for real-time linux used for hrpsys-base software.
## python/jython
python/jython files.
## lib
Library programs which can be used from outside of hrpsys-base.
For example, IO libraries.
## sample
Sample programs. See Samples.
## util
Utilities.
## doc
Doxygen files.
Substantial documentation files are in each directories such as rtc/XX/XX.txt.
## idl
IDL files used for each RTC.
## launch
ROS related programs.
## test
Test codes including python samples and rostest.
## rtc
RT-Component implementations.
Basically, one directory corresponds to one RT-Component.
