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
                cd ~/build && cmake ${CI_SOURCE_PATH} -DOPENRTM_DIR=`pkg-config openrtm-aist --variable=libdir`/openrtm_aist -DCOMPILE_JAVA_STUFF=OFF && make
                ;;
        esac
        ;;
    *)
        # COMPILE
        sudo apt-get install -qq -y freeglut3-dev python-tk jython doxygen libboost-all-dev libsdl1.2-dev libglew1.6-dev libqhull-dev libirrlicht-dev libxmu-dev libcv-dev libhighgui-dev libopencv-contrib-dev
        # check rtmros_common
        pkg=$TEST_PACKAGE
        sudo apt-get install -qq -y python-wstool ros-hydro-catkin ros-hydro-mk ros-hydro-rostest ros-hydro-rtmbuild

        sudo apt-get install -qq -y ros-hydro-$pkg

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
