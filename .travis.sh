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

        # this is hotfix
        sudo wget https://raw.githubusercontent.com/start-jsk/rtmros_common/master/hrpsys_tools/test/test-pa10.test -O /opt/ros/hydro/share/hrpsys_tools/test/test-pa10.test
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
            sed -i "s@install\(@dummy_install\(@f" src/hrpsys/CMakeLists.txt
            sed -i "s@install\(@dummy_install\(@f" src/hrpsys/catkin.cmake
            catkin_make -j8 -l8 --only-pkg-with-deps `echo $pkg | sed s/-/_/g`
            catkin_make install -j8 -l8

            # checkokut old hrpsys
            mkdir -p ~/hrpsys_ws/src
            cd ~/hrpsys_ws/src
            wstool init .
            wstool set hrpsys http://github.com/start-jsk/hrpsys -v 315.1.9 --git -y
            wstool update
            cd ~/hrpsys_ws

            catkin_make -j8 -l8
            catkin_make install -j8 -l8
            source install/setup.bash
            cd ~/catkin_ws
        fi

        rospack profile
        source install/setup.bash

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
