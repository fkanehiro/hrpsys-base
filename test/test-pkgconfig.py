#!/usr/bin/env python

PKG = 'hrpsys'
NAME = 'test-pkgconfig'

code = """
#include <sys/types.h> /* iob.h need this */
#include "io/iob.h"
int main (int argc, char** argv)
{
  open_iob();
  close_iob();
  return 0;
}
"""
import unittest, os, sys
from subprocess import call, check_output, Popen, PIPE, STDOUT

class TestHrpsysPkgconfig(unittest.TestCase):
    PKG_CONFIG_PATH = ''

    def setUp(self):
        # if rosbuild environment
        hrpsys_path = check_output(['rospack','find','hrpsys']).rstrip()
        openhrp3_path = check_output(['rospack','find','openhrp3']).rstrip()
        if os.path.exists(os.path.join(hrpsys_path, "bin")) :
            self.PKG_CONFIG_PATH='PKG_CONFIG_PATH=%s/lib/pkgconfig:%s/lib/pkgconfig:$PKG_CONFIG_PATH'%(hrpsys_path, openhrp3_path)

    def pkg_config_variable(self, var):
        return check_output("%s pkg-config hrpsys-base --variable=%s"%(self.PKG_CONFIG_PATH, var), shell=True).rstrip()

    def check_if_file_exists(self, var, fname):
        pkg_var = var
        pkg_dname = self.pkg_config_variable(pkg_var)
        pkg_path = os.path.join(pkg_dname, fname)
        pkg_ret = os.path.exists(pkg_path)
        self.assertTrue(pkg_ret, "pkg-config hrpsys --variable=%s`/%s (%s) returns %r"%(pkg_var, fname, pkg_path, pkg_ret))


    def check_if_file_exists_from_rospack(self, fname):
        pkg_dname = check_output(['rospack','find','hrpsys']).rstrip()
        pkg_path = os.path.join(pkg_dname, fname)
        pkg_ret = os.path.exists(pkg_path)
        self.assertTrue(pkg_ret, "`rospack find hrpsys`(%s) returns %r"%(pkg_path, pkg_ret))

    def test_files_for_hrpsys(self):
        # https://github.com/start-jsk/hrpsys/blob/master/test/test-pa10.test#L13
        self.check_if_file_exists_from_rospack("share/hrpsys/samples/PA10/")
        self.check_if_file_exists_from_rospack("share/hrpsys/samples/PA10/rtc.conf")
        self.check_if_file_exists_from_rospack("share/hrpsys/samples/PA10/RobotHardware.conf")
        self.check_if_file_exists_from_rospack("share/hrpsys/samples/PA10/PA10.conf")

    def test_files_for_hrpsys_ros_bridge(self):
        # https://github.com/start-jsk/rtmros_common/blob/master/hrpsys_ros_bridge/catkin.cmake#L50
        self.check_if_file_exists("idldir", "HRPDataTypes.idl")

    def test_compile_iob(self):
        global PID
        cmd = "%s pkg-config hrpsys-base --cflags --libs"%(self.PKG_CONFIG_PATH)
        print "`"+cmd+"` =",check_output(cmd, shell=True, stderr=STDOUT)
        ret = call("gcc -o hrpsys-sample-pkg-config /tmp/%d-hrpsys-sample.cpp `%s` -lhrpIo"%(PID,cmd), shell=True)
        self.assertTrue(ret==0)

    def test_idlfile(self):
        cmd = "%s pkg-config hrpsys-base --variable=idldir"%(self.PKG_CONFIG_PATH)
        print "`"+cmd+"`/RobotHardwareService.idl = ",os.path.join(check_output(cmd, shell=True, stderr=STDOUT).rstrip(), "RobotHardwareService.idl")
        self.assertTrue(os.path.exists(os.path.join(check_output(cmd, shell=True).rstrip(), "RobotHardwareService.idl")))

#unittest.main()
if __name__ == '__main__':
    import rostest
    global PID
    PID = os.getpid()
    f = open("/tmp/%d-hrpsys-sample.cpp"%(PID),'w')
    f.write(code)
    f.close()
    rostest.run(PKG, NAME, TestHrpsysPkgconfig, sys.argv)
