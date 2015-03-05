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

# MongoDB hack
dpkg -s mongodb || echo "ok"; export HAVE_MONGO_DB=$?
if [ $HAVE_MONGO_DB == 0 ]; then sudo apt-get remove -qq -y mongodb mongodb-10gen || echo "ok"; fi
if [ $HAVE_MONGO_DB == 0 ]; then sudo apt-get install -qq -y mongodb-clients mongodb-server -o Dpkg::Options::="--force-confdef" || echo "ok"; fi # default actions

export CI_SOURCE_PATH=$(pwd)
export REPOSITORY_NAME=${PWD##*/}
echo "Testing branch $TRAVIS_BRANCH of $REPOSITORY_NAME"
sudo sh -c 'echo "deb http://packages.ros.org/ros-shadow-fixed/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update -qq

# disable ssl
git config --global http.sslVerify false

case $TEST_PACKAGE in
    hrpsys)
        case $TEST_TYPE in
            iob)
                sudo apt-get install -qq -y cproto wget
                wget https://github.com/fkanehiro/hrpsys-base/raw/315.1.9/lib/io/iob.h -O iob.h.315.1.9
                echo -e "#define pid_t int\n#define size_t int\n#include \"lib/io/iob.h\""  | cproto -x - | sort > iob.h.current
                echo -e "#define pid_t int\n#define size_t int\n#include \"iob.h.315.1.9\"" | cproto -x - | sort > iob.h.stable
                cat iob.h.current
                cat iob.h.stable
                diff iob.h.current iob.h.stable || exit 1
                ;;
            stable_rtc)
                sudo apt-get install -qq -y omniidl diffstat wget ros-hydro-openrtm-aist
                source /opt/ros/hydro/setup.bash
                ## check stableRTCList
                sed -i 's@^from@#from@g' python/hrpsys_config.py
                sed -i 's@^import@#import@' python/hrpsys_config.py
                sed -i 's@^_EPS@#_EPS@' python/hrpsys_config.py
                sed -i 's@=nshost@=None@' python/hrpsys_config.py
                sed -i 's@initCORBA@#initCORBA@' python/hrpsys_config.py
                if [ "`python -c "import python.hrpsys_config; hcf=python.hrpsys_config.HrpsysConfigurator(); print [ x[1] for x in hcf.getRTCList()]" | tail -1`" != "['SequencePlayer', 'StateHolder', 'ForwardKinematics', 'CollisionDetector', 'SoftErrorLimiter', 'DataLogger']" ]; then
                    exit 1
                fi

                ## check idl
                mkdir stable_idl
                for idl_file in SequencePlayerService.idl StateHolderService.idl ForwardKinematicsService.idl CollisionDetectorService.idl SoftErrorLimiterService.idl DataLoggerService.idl   ExecutionProfileService.idl HRPDataTypes.idl RobotHardwareService.idl ; do
                    wget https://github.com/fkanehiro/hrpsys-base/raw/315.1.9/idl/${idl_file} -O stable_idl/${idl_file}
                    omniidl -bcxx -I/opt/ros/hydro/include/openrtm-1.1/rtm/idl/                      idl/${idl_file}
                    omniidl -bcxx -I/opt/ros/hydro/include/openrtm-1.1/rtm/idl/ -C stable_idl stable_idl/${idl_file}
                    sk_file=$(basename ${idl_file} .idl)SK.cc
                    cat ${sk_file}
                    cat stable_idl/${sk_file}

                    diff stable_idl/${sk_file} ${sk_file} | tee >(cat - 1>&2)  | diffstat | grep -c deletion && exit 1
                done
                echo "ok"
                ;;
            *)
                # COMPILE_ONLY
                sudo apt-get install -qq -y freeglut3-dev python-tk jython doxygen graphviz libboost-all-dev libsdl1.2-dev libglew1.6-dev libqhull-dev libirrlicht-dev libxmu-dev libcv-dev libhighgui-dev libopencv-contrib-dev

                sudo apt-get install -qq -y ros-hydro-openhrp3
                source /opt/ros/hydro/setup.bash
                mkdir -p ~/build
                cd ~/build && cmake ${CI_SOURCE_PATH} && make
                ;;
        esac
        ;;
    *)
        # COMPILE
        sudo apt-get install -qq -y freeglut3-dev python-tk jython doxygen libboost-all-dev libsdl1.2-dev libglew1.6-dev libqhull-dev libirrlicht-dev libxmu-dev libcv-dev libhighgui-dev libopencv-contrib-dev
        # check rtmros_common
        pkg=$TEST_PACKAGE
        sudo apt-get install -qq -y python-wstool ros-hydro-catkin ros-hydro-mk ros-hydro-rostest ros-hydro-rtmbuild ros-hydro-roslint

        sudo apt-get install -qq -y ros-hydro-$pkg

        source /opt/ros/hydro/setup.bash

        # set hrpsys in catkin workspace
        mkdir -p ~/catkin_ws/src
        cd ~/catkin_ws/src
        wstool init
        ln -sf ${CI_SOURCE_PATH} hrpsys
        sed -i "s@if(ENABLE_DOXYGEN)@if(0)@" hrpsys/CMakeLists.txt         # disable doc generation
        cd ~/catkin_ws

        if [ "$TEST_TYPE" == "work_with_downstream" ]; then
            echo "
            #
            # check newer version of hrpsys works on current rtmros_common deb package
            # [hrpsys:new] <-> [rtmros_common:old] + [hrpsys:old]
            "
            sudo dpkg -r --force-depends ros-hydro-hrpsys
            catkin_make_isolated -j1 -l1
            catkin_make_isolated --install -j2 -l2
            # you need to pretend this is catkin package since you only have hrpsys in catkin_ws
            export ROS_PACKAGE_PATH=`pwd`/install_isolated/share:`pwd`/install_isolated/stacks:$ROS_PACKAGE_PATH
            source install_isolated/setup.bash
        else
            echo "
            #
            # check rtmros_common compiled on newer version of hrpsys works with deb version of hrpsys
            # [hrpsys:old] <-> [rtmros_common:new] + [hrpsys:new]
            "
            #
            find /opt/ros/hydro/share/hrpsys
            sudo cp /opt/ros/hydro/bin/hrpsys-simulator /opt/ros/hydro/share/hrpsys/
            sudo cp /opt/ros/hydro/bin/ProjectGenerator /opt/ros/hydro/share/hrpsys/
            # set up sorce code of downstream package
            cd src
            wstool set rtmros_common http://github.com/start-jsk/rtmros_common --git -y
            wstool set rtmros_hironx http://github.com/start-jsk/rtmros_hironx --git -y
            wstool set rtmros_nextage http://github.com/tork-a/rtmros_nextage --git -y
            wstool update
            sudo apt-get install -qq -y ros-hydro-urdf

            cd ..
            # do not copile hrpsys because we wan to use them
            sed -i "1imacro(dummy_install)\nmessage(\"install(\${ARGN})\")\nendmacro()" src/hrpsys/CMakeLists.txt
            sed -i "s@install(@dummy_install(@g" src/hrpsys/CMakeLists.txt
            catkin_make_isolated -j2 -l2 --merge   --only-pkg-with-deps `echo $pkg | sed s/-/_/g`
            catkin_make_isolated -j2 -l2 --install --only-pkg-with-deps `echo $pkg | sed s/-/_/g`
            rm -fr ./install_isolated/hrpsys/share/hrpsys ./install_isolated/hrpsys/lib/pkgconfig/hrpsys.pc
            source install_isolated/setup.bash

            # checkokut old hrpsys
            mkdir -p ~/hrpsys_ws/src
            cd ~/hrpsys_ws/src
            wstool init .
            wstool set hrpsys http://github.com/start-jsk/hrpsys -v 315.1.9 --git -y
            wstool update
            sed -i "s@find_package(catkin REQUIRED COMPONENTS rostest mk openrtm_aist openhrp3)@find_package(catkin REQUIRED COMPONENTS rostest mk)\nset(openrtm_aist_PREFIX /opt/ros/hydro/)\nset(openhrp3_PREFIX /opt/ros/hydro/)@"  hrpsys/catkin.cmake
            sed -i "s@NUM_OF_CPUS = \$(shell grep -c '^processor' /proc/cpuinfo)@NUM_OF_CPUS = 2@" hrpsys/Makefile.hrpsys-base
            #git clone -b 315.1.9 http://github.com/fkanehiro/hrpsys-base hrpsys
            #cp ~/catkin_ws/src/hrpsys/CMakeLists.txt hrpsys/
            #cp ~/catkin_ws/src/hrpsys/package.xml hrpsys/
            cd ~/hrpsys_ws
            ls -al src
            ls -al src/hrpsys
            catkin_make_isolated -j1 -l1 --merge
            catkin_make_isolated -j2 -l2 --install
            # HOTFIX: https://github.com/k-okada/hrpsys-base/commit/9ce00db.diff
            sed -i "s@\['vs@#\['vs@g" install_isolated/lib/python2.7/dist-packages/hrpsys/hrpsys_config.py
            sed -i "s@\['afs@#\['afs@g" install_isolated/lib/python2.7/dist-packages/hrpsys/hrpsys_config.py
            sed -i "s@\['abc@#\['abc@g" install_isolated/lib/python2.7/dist-packages/hrpsys/hrpsys_config.py
            sed -i "s@\['st@#\['st@g" install_isolated/lib/python2.7/dist-packages/hrpsys/hrpsys_config.py

            #cp ~/catkin_ws/src/hrpsys/package.xml install_isolated/share/hrpsys/ # old hrpsys did not do this
            source install_isolated/setup.bash

            #echo $ROS_PACKAGE_PATH
            #export ROS_PACKAGE_PATH=`pwd`/install_isolated:$ROS_PACKAGE_PATH # for hrpsys (plane cmake)
            ls -al install_isolated/share/hrpsys
            rospack profile
            rospack find hrpsys

            cd ~/catkin_ws
        fi

        rospack profile


        # https://github.com/fkanehiro/openhrp3/issues/46
        ls -al /opt/ros/hydro/share/openhrp3/share/OpenHRP-3.1/sample/controller/SampleController || echo "ok"
        sudo mkdir -p /opt/ros/hydro/share/openhrp3/share/OpenHRP-3.1/sample/controller/SampleController/etc/ || echo "ok"
        sudo wget https://raw.githubusercontent.com/fkanehiro/openhrp3/master/sample/controller/SampleController/etc/Sample.pos -O /opt/ros/hydro/share/openhrp3/share/OpenHRP-3.1/sample/controller/SampleController/etc/Sample.pos || echo "ok"
        sudo wget https://raw.githubusercontent.com/fkanehiro/openhrp3/master/sample/controller/SampleController/etc/Sample.vel -O /opt/ros/hydro/share/openhrp3/share/OpenHRP-3.1/sample/controller/SampleController/etc/Sample.vel || echo "ok"
        # https://github.com/start-jsk/rtmros_hironx/issues/287
        if [ -e /opt/ros/hydro/share/hironx_ros_bridge/test/test_hironx_ros_bridge.py ]; then
            sudo sed -i "s@test_tf_and_controller@_test_tf_and_controller@" /opt/ros/hydro/share/hironx_ros_bridge/test/test_hironx_ros_bridge.py
        fi

        sudo /etc/init.d/omniorb4-nameserver stop || echo "stop omniserver just in case..."
        export EXIT_STATUS=0;
        pkg_path=`rospack find \`echo $pkg | sed s/-/_/g\``
        if [ "`find $pkg_path/test -iname '*.test'`" == "" ]; then
            echo "[$pkg] No tests ware found!!!"
        else
            find $pkg_path/test -iname "*.test" -print0 | xargs -0 -n1 rostest || export EXIT_STATUS=$?;
        fi

        # for debugging
        [ $TEST_PACKAGE == "hrpsys-ros-bridge" ] && rostest -t hrpsys_ros_bridge test-samplerobot.test
        [ $EXIT_STATUS == 0 ] || exit 1
        ;;
esac
