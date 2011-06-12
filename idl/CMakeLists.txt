set(idl_files 
  DataLoggerService.idl 
  StateHolderService.idl 
  RobotHardwareService.idl 
  WavPlayerService.idl 
  SequencePlayerService.idl 
  ForwardKinematicsService.idl
  ExecutionProfileService.idl
  )

install(FILES ${idl_files} DESTINATION share/hrpsys/idl)

