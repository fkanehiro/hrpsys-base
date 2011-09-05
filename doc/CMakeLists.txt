set(input_files package.h
		      ../jython/rtm.py
		      ../rtc/DataLogger/DataLogger.txt
		      ../rtc/ForwardKinematics/ForwardKinematics.txt
		      ../rtc/Joystick/Joystick.txt
		      ../rtc/Joystick2Velocity2D/Joystick2Velocity2D.txt
		      ../rtc/Joystick2PanTiltAngles/Joystick2PanTiltAngles.txt
		      ../rtc/NullComponent/NullComponent.txt
		      ../rtc/OccupancyGridMap3D/OccupancyGridMap3D.txt
		      ../rtc/RobotHardware/RobotHardware.txt
		      ../rtc/SequencePlayer/SequencePlayer.txt
		      ../rtc/StateHolder/StateHolder.txt
		      ../rtc/WavPlayer/WavPlayer.txt
		      ../hrplib/hrpIo/iob.h
		      ../idl/DataLoggerService.idl
		      ../idl/ExecutionProfileService.idl
		      ../idl/ForwardKinematicsService.idl
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
