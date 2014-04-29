#!/bin/bash

set -x

function error {
    find ${HOME}/.ros/test_results -type f -exec echo "=== {} ===" \; -exec\
 cat {} \;
    for file in ${HOME}/.ros/log/rostest-*; do echo "=== $file ==="; cat \$\
file; done
    exit 1
}

trap error ERR

export CI_SOURCE_PATH=$(pwd)
export REPOSITORY_NAME=${PWD##*/}
echo "Testing branch $TRAVIS_BRANCH of $REPOSITORY_NAME"
sudo sh -c 'echo "deb http://packages.ros.org/ros-shadow-fixed/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update -qq

sudo apt-get install -qq -y freeglut3-dev python-tk jython doxygen libboost-all-dev libsdl1.2-dev libglew1.6-dev libqhull-dev libirrlicht-dev libxmu-dev libcv-dev libhighgui-dev libopencv-contrib-dev

# disable ssl
git config --global http.sslVerify false

case $TEST_PACKAGE in
    hrpsys)
        # COMPILE_ONLY
        sudo apt-get install -qq -y ros-hydro-openhrp3
        source /opt/ros/hydro/setup.bash
        mkdir -p ~/build
        cd ~/build && cmake ${CI_SOURCE_PATH} -DOPENRTM_DIR=`pkg-config openrtm-aist --variable=libdir`/openrtm_aist -DCOMPILE_JAVA_STUFF=OFF && make
        ;;

    *)
        # check rtmros_common
        pkg=$TEST_PACKAGE
        sudo apt-get install -qq -y python-wstool ros-hydro-catkin ros-hydro-mk ros-hydro-rostest ros-hydro-rtmbuild

        sudo apt-get install -qq -y ros-hydro-$pkg

        sudo apt-get install -qq -y ros-hydro-rqt-robot-dashboard

        # this is hotfix
        if [ -e /opt/ros/hydro/share/hrpsys_tools ] ; then
            sudo wget https://raw.githubusercontent.com/start-jsk/rtmros_common/master/hrpsys_tools/test/test-pa10.test -O /opt/ros/hydro/share/hrpsys_tools/test/test-pa10.test
        fi

        if [ -e /opt/ros/hydro/lib/python2.7/dist-packages/hrpsys_ros_bridge/ ] ; then
            sudo touch /opt/ros/hydro/lib/python2.7/dist-packages/hrpsys_ros_bridge/__init__.py;

            #
            sudo sed -i s@imu_floor@odom@g /opt/ros/hydro/share/hrpsys_ros_bridge/test/test-samplerobot.py

            #
            sudo patch -p0 /opt/ros/hydro/share/hrpsys_ros_bridge/scripts/sensor_ros_bridge_connect.py <<EOF
--- /opt/ros/hydro/share/hrpsys_ros_bridge/scripts/sensor_ros_bridge_connect.py 2014-04-17 17:28:42.000000000 +0900
+++ /opt/ros/hydro/share/hrpsys_ros_bridge/scripts/sensor_ros_bridge_connect.py 2014-04-28 00:30:27.250839313 +0900
@@ -21,7 +21,7 @@
             if rh.port(sen.name) != None: # check existence of sensor ;; currently original HRP4C.xml has different naming rule of gsensor and gyrometer
                 print program_name, "connect ", sen.name, rh.port(sen.name).get_port_profile().name, bridge.port(sen.name).get_port_profile().name
                 connectPorts(rh.port(sen.name), bridge.port(sen.name), "new")
-                if sen.type == 'Force':
+                if sen.type == 'Force' and afs != None:
                     print program_name, "connect ", sen.name, afs.port("off_" + sen.name).get_port_profile().name, bridge.port("off_" + sen.name).get_port_profile().name
                     connectPorts(afs.port("off_" + sen.name), bridge.port("off_" + sen.name), "new") # for abs forces
         else:

EOF

            sudo patch -p0 /opt/ros/hydro/share/hrpsys_ros_bridge/launch/hrpsys_ros_bridge.launch <<EOF
--- /opt/ros/hydro/share/hrpsys_ros_bridge/launch/hrpsys_ros_bridge.launch 2014-04-17 17:28:42.000000000 +0900
+++ /opt/ros/hydro/share/hrpsys_ros_bridge/launch/hrpsys_ros_bridge.launch 2014-04-28 00:30:27.250839313 +0900
@@ -10,9 +10,9 @@
   <!-- Set these values false when using HIRO -->
   <arg name="USE_COMMON" default="true" />
   <arg name="USE_ROBOTHARDWARE" default="false" />
-  <arg name="USE_WALKING" default="true" />
+  <arg name="USE_WALKING" default="false" />
   <arg name="USE_COLLISIONCHECK" default="true" />
-  <arg name="USE_IMPEDANCECONTROLLER" default="true" />
+  <arg name="USE_IMPEDANCECONTROLLER" default="false" />
   <arg name="USE_GRASPCONTROLLER" default="false" />
   <arg name="USE_TORQUECONTROLLER" default="false" />
   <arg name="USE_SOFTERRORLIMIT" default="true" />
EOF
        fi
        #
        source /opt/ros/hydro/setup.bash

        # set hrpsys (hrpsys wrapper for ROS, just for compile and test)
        mkdir -p ~/catkin_ws/src
        cd ~/catkin_ws/src
        wstool init
        wstool set hrpsys http://github.com/start-jsk/hrpsys --git -y
        wstool update
        # compile HEAD of hrpsys-base
        sed -i s/^GIT_REVISION/#GIT_REVISION/g hrpsys/Makefile.hrpsys-base
        sed -i s/^GIT_PATCH/#GIT_PATCH/g hrpsys/Makefile.hrpsys-base
        cd ~/catkin_ws

        # us hrpsys-base what the travis donwnloaded
        mkdir -p build/hrpsys/build
        ln -sf ${CI_SOURCE_PATH} build/hrpsys/build/hrpsys-base-source
        touch build/hrpsys/installed

        if [ "$TEST_TYPE" == "work_with_downstream" ]; then
            echo "
            #
            # check newer version of hrpsys works on current rtmros_common deb package
            # [hrpsys:new] <-> [rtmros_common:old] + [hrpsys:old]
            "
            sudo dpkg -r --force-depends ros-hydro-hrpsys
            catkin_make -j8 -l8
            catkin_make install -j8 -l8
            source install/setup.bash
        else
            echo "
            #
            # check rtmros_common compiled on newer version of hrpsys works with deb version of hrpsys
            # [hrpsys:old] <-> [rtmros_common:new] + [hrpsys:new]
            "
            # set up sorce code of downstream package
            cd src
            wstool set rtmros_common http://github.com/start-jsk/rtmros_common --git -y
            wstool set rtmros_hironx http://github.com/start-jsk/rtmros_hironx --git -y
            wstool set rtmros_nextage http://github.com/tork-a/rtmros_nextage --git -y
            wstool update

            ## HOTFIX: https://github.com/start-jsk/rtmros_common/pull/447
            wget https://github.com/start-jsk/rtmros_common/pull/447.diff
            (cd rtmros_common; patch -p1 < ../447.diff)
            cd ..
            # do not copile hrpsys because we wan to use them
            sed -i "1imacro(dummy_install)\nmessage(\"install(\${ARGN})\")\nendmacro()" src/hrpsys/CMakeLists.txt
            sed -i "s@install(@dummy_install(@g" src/hrpsys/CMakeLists.txt
            sed -i "s@install(@dummy_install(@g" src/hrpsys/catkin.cmake
            catkin_make -j8 -l8 --only-pkg-with-deps `echo $pkg | sed s/-/_/g`
            catkin_make install -j8 -l8
            rm -fr install/share/hrpsys ./install/lib/pkgconfig/hrpsys.pc
            source install/setup.bash

            # checkokut old hrpsys
            mkdir -p ~/hrpsys_ws/src
            cd ~/hrpsys_ws/src
            wstool init .
            wstool set hrpsys http://github.com/start-jsk/hrpsys -v 315.1.9 --git -y
            wstool update
            cd ~/hrpsys_ws

            catkin_make -j8 -l8
            catkin_make install -j8 -l8
            # HOTFIX: https://github.com/k-okada/hrpsys-base/commit/9ce00db.diff
            sed -i "s@\['vs@#\['vs@g" install/lib/python2.7/dist-packages/hrpsys/hrpsys_config.py
            sed -i "s@\['afs@#\['afs@g" install/lib/python2.7/dist-packages/hrpsys/hrpsys_config.py
            sed -i "s@\['abc@#\['abc@g" install/lib/python2.7/dist-packages/hrpsys/hrpsys_config.py
            sed -i "s@\['st@#\['st@g" install/lib/python2.7/dist-packages/hrpsys/hrpsys_config.py

            source install/setup.bash

            cd ~/catkin_ws
        fi

        rospack profile

        export EXIT_STATUS=0;
        pkg_path=`rospack find \`echo $pkg | sed s/-/_/g\``
        if [ "`find $pkg_path/test -iname '*.test'`" == "" ]; then
            echo "[$pkg] No tests ware found!!!"
        else
            find $pkg_path/test -iname "*.test" -print0 | xargs -0 -n1 rostest || export EXIT_STATUS=$?;
        fi

        [ $EXIT_STATUS == 0 ] || exit 1
        ;;
esac
