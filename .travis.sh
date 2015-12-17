#!/bin/bash

set -x

function error {
    travis_time_end 31
    find ${HOME}/.ros/test_results -type f -exec echo "=== {} ===" \; -exec\
 cat {} \;
    for file in ${HOME}/.ros/log/rostest-*; do echo "=== $file ==="; cat \$\
file; done
    exit 1
}

trap error ERR

function travis_time_start {
    set +x
    TRAVIS_START_TIME=$(date +%s%N)
    TRAVIS_TIME_ID=$(cat /dev/urandom | tr -dc 'a-z0-9' | fold -w 8 | head -n 1)
    TRAVIS_FOLD_NAME=$1
    echo -e "\e[0Ktraivs_fold:start:$TRAVIS_FOLD_NAME"
    echo -e "\e[0Ktraivs_time:start:$TRAVIS_TIME_ID"
    set -x
}
function travis_time_end {
    set +x
    _COLOR=${1:-32}
    TRAVIS_END_TIME=$(date +%s%N)
    TIME_ELAPSED_SECONDS=$(( ($TRAVIS_END_TIME - $TRAVIS_START_TIME)/1000000000 ))
    echo -e "traivs_time:end:$TRAVIS_TIME_ID:start=$TRAVIS_START_TIME,finish=$TRAVIS_END_TIME,duration=$(($TRAVIS_END_TIME - $TRAVIS_START_TIME))\n\e[0K"
    echo -e "traivs_fold:end:$TRAVIS_FOLD_NAME"
    echo -e "\e[0K\e[${_COLOR}mFunction $TRAVIS_FOLD_NAME takes $(( $TIME_ELAPSED_SECONDS / 60 )) min $(( $TIME_ELAPSED_SECONDS % 60 )) sec\e[0m"
    set -x
}

travis_time_start mongo_hack

# MongoDB hack
dpkg -s mongodb || echo "ok"; export HAVE_MONGO_DB=$?
# Remove configuration file for mongodb using --purge option (reported in https://github.com/fkanehiro/hrpsys-base/pull/900#issuecomment-162392884)
if [ $HAVE_MONGO_DB == 0 ]; then sudo apt-get remove --purge -qq -y mongodb mongodb-10gen || echo "ok"; fi
if [ $HAVE_MONGO_DB == 0 ]; then sudo apt-get install -qq -y mongodb-clients mongodb-server -o Dpkg::Options::="--force-confdef" || echo "ok"; fi # default actions

travis_time_end
travis_time_start setup_ros

export CI_SOURCE_PATH=$(pwd)
export REPOSITORY_NAME=${PWD##*/}
echo "Testing branch $TRAVIS_BRANCH of $REPOSITORY_NAME"
sudo sh -c 'echo "deb http://packages.ros.org/ros-shadow-fixed/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update -qq

travis_time_end
travis_time_start disable_ssl

# disable ssl
git config --global http.sslVerify false

travis_time_end

case $TEST_PACKAGE in
    hrpsys)
        case $TEST_TYPE in
            python)
                travis_time_start  install_python

                #http://askubuntu.com/questions/204510/how-to-install-python-2-5-4
                sudo apt-add-repository -y ppa:fkrull/deadsnakes
                sudo apt-get update -qq
                sudo apt-get install -qq -y python2.5 python3.4

                travis_time_end
                travis_time_start  check_python

                for code in python/*.py; do
                    python2.5 -m py_compile $code
                    python2.7 -m py_compile $code
                    python3.4 -m py_compile $code
                done


                travis_time_end
                ;;
            iob)
                travis_time_start  install_wget

                sudo apt-get install -qq -y cproto wget diffstat

                travis_time_end
                travis_time_start  iob_test

                wget https://github.com/fkanehiro/hrpsys-base/raw/315.1.9/lib/io/iob.h -O iob.h.315.1.9
                echo -e "#define pid_t int\n#define size_t int\n#include \"lib/io/iob.h\""  | cproto -x - | sort > iob.h.current
                echo -e "#define pid_t int\n#define size_t int\n#include \"iob.h.315.1.9\"" | cproto -x - | sort > iob.h.stable
                cat iob.h.current
                cat iob.h.stable
                diff iob.h.stable iob.h.current || exit 1

                travis_time_end
                ;;
            stable_rtc)
                travis_time_start  install_openrtm

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

                travis_time_end
                travis_time_start  check_idl

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

                travis_time_end
                ;;
            *)
                travis_time_start  install_libs

                # COMPILE_ONLY
                sudo apt-get install -qq -y freeglut3-dev python-tk jython doxygen graphviz libboost-all-dev libsdl1.2-dev libglew1.6-dev libqhull-dev libirrlicht-dev libxmu-dev libcv-dev libhighgui-dev libopencv-contrib-dev

                travis_time_end
                travis_time_start  install_openhrp3

                sudo apt-get install -qq -y ros-hydro-openhrp3
                source /opt/ros/hydro/setup.bash
                if [ "$USE_SRC_OPENHRP3" == true ] ; then
                    sudo dpkg -r --force-depends ros-hydro-openhrp3
                    mkdir -p ~/build_openhrp3
                    cd ~/build_openhrp3
                    git clone http://github.com/fkanehiro/openhrp3
                    sed -i 's/COLLADA_DOM_FOUND/0/' openhrp3/sample/CMakeLists.txt
                    cd openhrp3 && cmake . ${COMPILE_OPTION} && make && sudo make install
                fi
                mkdir -p ~/build

                travis_time_end
                travis_time_start  compile_hrpsys

                cd ~/build && cmake ${CI_SOURCE_PATH} ${COMPILE_OPTION} && make
                # Check make test by passing PATH to bin directory
                PATH=$PATH:~/build/bin make test

                travis_time_end
                ;;
        esac
        ;;
    *)
        travis_time_start  install_libs

        # COMPILE
        sudo apt-get install -qq -y freeglut3-dev python-tk jython doxygen libboost-all-dev libsdl1.2-dev libglew1.6-dev libqhull-dev libirrlicht-dev libxmu-dev libcv-dev libhighgui-dev libopencv-contrib-dev
        # check rtmros_common

        if [ "$TEST_PACKAGE" == "hrpsys-base" ]; then
            TEST_PACKAGE="hrpsys"
        fi
        travis_time_end
        travis_time_start  install_$TEST_PACKAGE

        pkg=$TEST_PACKAGE
        sudo apt-get install -qq -y python-wstool ros-hydro-catkin ros-hydro-mk ros-hydro-rostest ros-hydro-rtmbuild ros-hydro-roslint > /dev/null

        sudo apt-get install -qq -y ros-hydro-pcl-ros ros-hydro-moveit-commander ros-hydro-rqt-robot-dashboard > /dev/null

        sudo apt-get install -qq -y ros-hydro-$pkg

        source /opt/ros/hydro/setup.bash

        travis_time_end
        travis_time_start  setup_catkin_ws

        # set hrpsys in catkin workspace
        mkdir -p ~/catkin_ws/src
        cd ~/catkin_ws/src
        wstool init
        ln -sf ${CI_SOURCE_PATH} hrpsys
        sed -i "s@if(ENABLE_DOXYGEN)@if(0)@" hrpsys/CMakeLists.txt         # disable doc generation
        find hrpsys -name CMakeLists.txt -exec sed -i "s@PCL_FOUND@0@" {} \; # disable PCL
        find hrpsys -name CMakeLists.txt -exec sed -i "s@OCTMAP_FOUND@0@" {} \; # disable OCTMAP
        find hrpsys -name CMakeLists.txt -exec sed -i "s@IRRLIGHT_FOUND@0@" {} \; # disable IRRLIGHT
        if [ "$USE_SRC_OPENHRP3" == true ] ; then
            git clone http://github.com/fkanehiro/openhrp3/
            sed -i 's/COLLADA_DOM_FOUND/0/' openhrp3/sample/CMakeLists.txt
        fi
        cd ~/catkin_ws

        travis_time_end

        if [ "$TEST_TYPE" == "work_with_downstream" ]; then
            echo "
            #
            # check newer version of hrpsys works on current rtmros_common deb package
            # [hrpsys:new] <-> [rtmros_common:old] + [hrpsys:old]
            "

            travis_time_start  compile_and_install_downstream

            sudo dpkg -r --force-depends ros-hydro-hrpsys

            catkin_make_isolated --install -j1 -l1
            # you need to pretend this is catkin package since you only have hrpsys in catkin_ws
            export ROS_PACKAGE_PATH=`pwd`/install_isolated/share:`pwd`/install_isolated/stacks:$ROS_PACKAGE_PATH
            source install_isolated/setup.bash

            travis_time_end
        else
            echo "
            #
            # check rtmros_common compiled on newer version of hrpsys works with deb version of hrpsys
            # [hrpsys:old] <-> [rtmros_common:new] + [hrpsys:new]
            "

            travis_time_start  setup_wstool_rtmros

            # set up sorce code of downstream package
            cd src
            wstool set rtmros_common http://github.com/start-jsk/rtmros_common --git -y
            wstool set rtmros_hironx http://github.com/start-jsk/rtmros_hironx --git -y
            wstool set rtmros_nextage http://github.com/tork-a/rtmros_nextage --git -y
            wstool update
            sudo apt-get install -qq -y ros-hydro-urdf
            sudo dpkg -r --force-depends ros-hydro-hrpsys
            export ROS_LANG_DISABLE=genlisp

            cd ..
            # do not compile unstable rtc
            sed -i -e 's/\(add_subdirectory(\(RobotHardware\|SequencePlayer\|StateHolder\|ForwardKinematics\|CollisionDetector\|SoftErrorLimiter\|DataLogger\))\)/#\1/' -e 's/add_subdirectory/#add_subdirectory/' -e 's/##add/add/' src/hrpsys/rtc/CMakeLists.txt
            cat  src/hrpsys/rtc/CMakeLists.txt
            # do not copile hrpsys because we wan to use them
            sed -i "1imacro(dummy_install)\nmessage(\"install(\${ARGN})\")\nendmacro()" src/hrpsys/CMakeLists.txt
            sed -i "s@install(@dummy_install(@g" src/hrpsys/CMakeLists.txt
            sed -i "\$iinstall(DIRECTORY test launch sample DESTINATION share/hrpsys USE_SOURCE_PERMISSIONS)" src/hrpsys/CMakeLists.txt
            sed -i "\$iinstall(FILES package.xml DESTINATION share/hrpsys/)" src/hrpsys/CMakeLists.txt
            sed -i "\$iinstall(CODE \"execute_process(COMMAND cmake -E make_directory share/hrpsys WORKING_DIRECTORY \$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/share/hrpsys)\")" src/hrpsys/CMakeLists.txt
            sed -i "\$iinstall(CODE \"execute_process(COMMAND cmake -E create_symlink ../../../hrpsys/idl     share/hrpsys/idl WORKING_DIRECTORY \$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/share/hrpsys)\")" src/hrpsys/CMakeLists.txt
            sed -i "\$iinstall(CODE \"execute_process(COMMAND cmake -E create_symlink ../../../hrpsys/samples share/hrpsys/samples WORKING_DIRECTORY \$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/share/hrpsys)\")" src/hrpsys/CMakeLists.txt
            cat src/hrpsys/CMakeLists.txt

            travis_time_end
            travis_time_start  compile_new_version

            catkin_make_isolated -j1 -l1 --install --only-pkg-with-deps `echo $pkg | sed s/-/_/g` | grep -v '^-- \(Up-to-date\|Installing\):' | grep -v 'Generating \(Python\|C++\) code from' | grep -v '^Compiling .*.py ...$' | uniq
            rm -fr ./install_isolated/hrpsys/share/hrpsys ./install_isolated/hrpsys/lib/pkgconfig/hrpsys.pc
            source install_isolated/setup.bash

            travis_time_end
            travis_time_start  setup_old_hrpsys

            # checkokut old hrpsys
            mkdir -p ~/hrpsys_ws/src
            cd ~/hrpsys_ws/src
            wstool init .
            wstool set hrpsys http://github.com/start-jsk/hrpsys -v 315.1.9 --git -y
            wstool update
            #
            sed -i "1imacro(dummy_macro)\nmessage(\"dummy(\${ARGN})\")\nendmacro()" hrpsys/catkin.cmake
            sed -i "s@install(DIRECTORY test share@dummy_macro(DIRECTORY test share@" hrpsys/catkin.cmake
            sed -i "s@find_package(catkin REQUIRED COMPONENTS rostest mk openrtm_aist openhrp3)@find_package(catkin REQUIRED COMPONENTS rostest mk)\nset(openrtm_aist_PREFIX /opt/ros/hydro/)\nset(openhrp3_PREFIX /opt/ros/hydro/)@"  hrpsys/catkin.cmake
            cat hrpsys/catkin.cmake
            sed -i "s@NUM_OF_CPUS = \$(shell grep -c '^processor' /proc/cpuinfo)@NUM_OF_CPUS = 2@" hrpsys/Makefile.hrpsys-base
            sed -i "s@touch installed@@" hrpsys/Makefile.hrpsys-base
            cat hrpsys/Makefile.hrpsys-base
            # use git repository, instead of svn due to googlecode shoutdown
            git clone http://github.com/fkanehiro/hrpsys-base --depth 1 -b 315.1.9 ../build_isolated/hrpsys/build/hrpsys-base-source
            # we use latest hrpsys_ocnfig.py for this case, so do not install them
            sed -i -e 's/\(add_subdirectory(python)\)/#\1/' ../build_isolated/hrpsys/build/hrpsys-base-source/CMakeLists.txt
            sed -i -e 's/\(add_subdirectory(test)\)/#\1/' ../build_isolated/hrpsys/build/hrpsys-base-source/CMakeLists.txt
            find ../build_isolated/hrpsys/build/hrpsys-base-source -name CMakeLists.txt -exec sed -i "s@PCL_FOUND@0@" {} \; # disable PCL
            find ../build_isolated/hrpsys/build/hrpsys-base-source -name CMakeLists.txt -exec sed -i "s@OCTOMAP_FOUND@0@" {} \; # disable OCTOMAP
            find ../build_isolated/hrpsys/build/hrpsys-base-source -name CMakeLists.txt -exec sed -i "s@IRRLIGHT_FOUND@0@" {} \; # disable IRRLIGHT
            sed -i "s@USE_HRPSYSUTIL@0@" ../build_isolated/hrpsys/build/hrpsys-base-source/rtc/CMakeLists.txt # disable HRPSYSUTIL APPs # this disables CollisionDetector
            sed -i "\$aadd_subdirectory(CollisionDetector)" ../build_isolated/hrpsys/build/hrpsys-base-source/rtc/CMakeLists.txt

            sed -i "s@NOT QNXNTO@1@" ../build_isolated/hrpsys/build/hrpsys-base-source/rtc/CMakeLists.txt # disable Joystick

            sed -i "1imacro(dummy_macro)\nmessage(\"dummy(\${ARGN})\")\nendmacro()" ../build_isolated/hrpsys/build/hrpsys-base-source/util/simulator/CMakeLists.txt
            sed -i "s@\(.*\)(hrpsysext@dummy_macro(hrpsysext@g" ../build_isolated/hrpsys/build/hrpsys-base-source/util/simulator/CMakeLists.txt
            sed -i "s@install(TARGETS hrpsysext@dummy_macro(TARGETS hrpsysext@g" ../build_isolated/hrpsys/build/hrpsys-base-source/util/simulator/CMakeLists.txt

            cat ../build_isolated/hrpsys/build/hrpsys-base-source/CMakeLists.txt
            cat ../build_isolated/hrpsys/build/hrpsys-base-source/rtc/CMakeLists.txt

            cd ~/hrpsys_ws
            ls -al src
            ls -al src/hrpsys

            travis_time_end
            travis_time_start  compile_old_hrpsys

            trap 0 ERR
            need_compile=1
            while [ $need_compile != 0 ]; do
                catkin_make_isolated -j1 -l1 --merge
                need_compile=$?
            done
            trap error ERR

            travis_time_end
            travis_time_start  install_old_hrpsys

            trap 0 ERR
            need_compile=1
            while [ $need_compile != 0 ]; do
                catkin_make_isolated -j1 -l1 --install | grep -v '^-- \(Up-to-date\|Installing\):'
                need_compile=${PIPESTATUS[0]}
            done
            trap error ERR

            #cp ~/catkin_ws/src/hrpsys/package.xml install_isolated/share/hrpsys/ # old hrpsys did not do this
            mkdir -p install_isolated/share/hrpsys/share/hrpsys/
            cp -r ~/catkin_ws/install_isolated/share/hrpsys/idl install_isolated/share/hrpsys/share/hrpsys/
            cp -r ~/catkin_ws/install_isolated/share/hrpsys/{test,launch,samples} install_isolated/share/hrpsys/ # cp latest script

            source install_isolated/setup.bash

            #echo $ROS_PACKAGE_PATH
            #export ROS_PACKAGE_PATH=`pwd`/install_isolated:$ROS_PACKAGE_PATH # for hrpsys (plane cmake)
            ls -al install_isolated/share/hrpsys
            rospack profile
            rospack find hrpsys

            cd ~/catkin_ws

            travis_time_end
        fi

        travis_time_start  hot_fixes

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
        #https://github.com/start-jsk/rtmros_hironx/pull/358
        if [ -e /opt/ros/hydro/lib/python2.7/dist-packages/hironx_ros_bridge/hironx_client.py ]; then
            sudo wget https://raw.githubusercontent.com/k-okada/rtmros_hironx/stop_unfinished_battle/hironx_ros_bridge/src/hironx_ros_bridge/hironx_client.py -O /opt/ros/hydro/lib/python2.7/dist-packages/hironx_ros_bridge/hironx_client.py
        fi
        #https://github.com/start-jsk/rtmros_common/commit/51ec26b899f09304705fe0528a068e57b061b9b7
        #https://github.com/start-jsk/rtmros_common/pull/880
        #https://github.com/start-jsk/rtmros_common/pull/879
        if [ -e /opt/ros/hydro/share/hrpsys_ros_bridge/test/test-samplerobot.test ]; then
            sudo wget https://raw.githubusercontent.com/start-jsk/rtmros_common/1.3.1/hrpsys_ros_bridge/test/test-samplerobot.test -O /opt/ros/hydro/share/hrpsys_ros_bridge/test/test-samplerobot.test
        fi
        travis_time_end

        # Check make test
        (cd ~/catkin_ws/build_isolated/hrpsys/install && make test)
        # Check rostest
        sudo /etc/init.d/omniorb4-nameserver stop || echo "stop omniserver just in case..."
        export EXIT_STATUS=0;
        pkg_path=`rospack find \`echo $pkg | sed s/-/_/g\``
        if [ "`find $pkg_path/test -iname '*.test'`" == "" ]; then
            echo "[$pkg] No tests ware found!!!"
        else
            for test_file in `find $pkg_path/test -iname "*.test" -print`; do
                travis_time_start $(echo $test_file | sed 's@.*/\([a-zA-Z0-9-]*\).test$@\1@' | sed 's@-@_@g')
                export TMP_EXIT_STATUS=0
                rostest $test_file && travis_time_end || export TMP_EXIT_STATUS=$?
                if [ "$TMP_EXIT_STATUS" != 0 ]; then
                    export EXIT_STATUS=$TMP_EXIT_STATUS
                    # Print results of rostest-*.xml files
                    find ~/.ros/test_results -type f -iname "*`basename $test_file .test`.xml" -print -exec echo "=== {} ===" \; -exec cat {} \;
                    # Print results of each rosunit-*.xml file
                    #   Get rosunit*.xml file path from rostest-*.xml file by usig awk and cut.
                    #   Files are assumed to include "xxx results are in [/home/xxx/rosunit-yy.xml]"
                    cat $(find ~/.ros/test_results -type f -iname "*`basename $test_file .test`.xml" -print -exec echo "=== {} ===" \; -exec cat {} \; | grep "results are in" | awk -F'results are in ' '{print $2}' | cut -d\[ -f2 | cut -d\] -f1)
                    travis_time_end 31
                fi
            done
        fi

        travis_time_start  end_tests

        # for debugging
        [ $TEST_PACKAGE == "hrpsys-ros-bridge" ] && rostest -t hrpsys_ros_bridge test-samplerobot.test
        [ $EXIT_STATUS == 0 ] || exit 1

        travis_time_end
        ;;
esac
