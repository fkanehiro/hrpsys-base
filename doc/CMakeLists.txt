set(input_files package.h utilities.h
		      ../python/hrpsys_config.py
		      ../python/rtm.py
		      ../rtc/CameraImageViewer/CameraImageViewer.txt
		      ../rtc/DataLogger/DataLogger.txt
		      ../rtc/ForwardKinematics/ForwardKinematics.txt
		      ../rtc/HGcontroller/HGcontroller.txt
		      ../rtc/Joystick/Joystick.txt
		      ../rtc/Joystick2Velocity2D/Joystick2Velocity2D.txt
		      ../rtc/Joystick2Velocity3D/Joystick2Velocity3D.txt
		      ../rtc/Joystick2PanTiltAngles/Joystick2PanTiltAngles.txt
		      ../rtc/JpegDecoder/JpegDecoder.txt
		      ../rtc/NullComponent/NullComponent.txt
		      ../rtc/OccupancyGridMap3D/OccupancyGridMap3D.txt
		      ../rtc/OGMap3DViewer/OGMap3DViewer.txt
		      ../rtc/Range2PointCloud/Range2PointCloud.txt
		      ../rtc/RobotHardware/RobotHardware.txt
		      ../rtc/SequencePlayer/SequencePlayer.txt
		      ../rtc/Simulator/Simulator.txt
		      ../rtc/SoftErrorLimiter/SoftErrorLimiter.txt
		      ../rtc/StateHolder/StateHolder.txt
		      ../rtc/VideoCapture/VideoCapture.txt
		      ../rtc/Viewer/Viewer.txt
		      ../rtc/VirtualCamera/VirtualCamera.txt
		      ../rtc/VirtualForceSensor/VirtualForceSensor.txt
		      ../rtc/WavPlayer/WavPlayer.txt
		      ../lib/io/iob.h
		      ../idl/CollisionDetectorService.idl
		      ../idl/DataLoggerService.idl
		      ../idl/ExecutionProfileService.idl
		      ../idl/ForwardKinematicsService.idl
		      ../idl/GraspControllerService.idl
		      ../idl/ImpedanceControllerService.idl
		      ../idl/OGMap3DService.idl
		      ../idl/RobotHardwareService.idl
		      ../idl/SequencePlayerService.idl
		      ../idl/StateHolderService.idl
		      ../idl/TimeKeeperService.idl
		      ../idl/WavPlayerService.idl
		      )
configure_file(Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
set(output_dir ${CMAKE_CURRENT_BINARY_DIR}/html)
file(MAKE_DIRECTORY ${output_dir})

add_custom_command(
  OUTPUT ${output_dir}/index.html
  COMMAND doxygen
  DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile ${input_files}
)

add_custom_target(doc ALL DEPENDS ${output_dir}/index.html)

install(DIRECTORY ${output_dir} DESTINATION share/doc/hrpsys/base)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/hrpsys_base.doxytag DESTINATION share/doc/hrpsys/base)
