#!/bin/bash

HRP4C_DOWNLOAD_FILE=$HOME/Downloads/HRP-4C.zip
HRP4C_MODEL_DIR=`rospack find hrpsys`/share/hrpsys/samples/
HRP4C_MODEL_PATH=${HRP4C_MODEL_DIR}/HRP4C/HRP4Cmain.wrl

if [ ! -f ${HRP4C_MODEL_PATH} ]; then
    if [ -f ${HRP4C_DOWNLOAD_FILE} ]; then
        command1="unzip -j ${HRP4C_DOWNLOAD_FILE}    HRP-4C/HRP4Cmain.wrl -d ${HRP4C_MODEL_DIR}/HRP4C"
        command2="unzip -j ${HRP4C_DOWNLOAD_FILE} -x HRP-4C/HRP4Cmain.wrl -d ${HRP4C_MODEL_DIR}/HRP4C/cover"
        $command1
        $command2
        if [ $? != 0 ]; then
            echo "[CAUTION] "
            echo "[CAUTION] If you failed download for some reason, pleasy try following command manually "
            echo " sudo $command1"
            echo " sudo $command2"
            echo " sudo chmod a+rw -R ${HRP4C_MODEL_DIR}"
        else
            echo " Congratulations! You have HRP-4C robot model under to ${HRP4C_MODEL_PATH}"
        fi
    else
        echo " This script assuming you have downloaded HRP4 model file to ${HRP4C_DOWNLOAD_FILE}   "
        echo "                                                      "
        echo " If you do not have one                               "
        echo "  1) read overview at https://unit.aist.go.jp/is/humanoid/hrp-4c/hrp-4c.html"
        echo "  2) read term and condition at https://unit.aist.go.jp/is/humanoid/hrp-4c/agreement.html"
        echo "     put your Name, Affliaciton and E-mail Address and push \"Submit\" button to obtain download link and user/pass"
        echo "                                                      "
        exit -1
    fi
else
        echo " You already have HRP-4C robot model under to ${HRP4C_MODEL_PATH}"
fi
