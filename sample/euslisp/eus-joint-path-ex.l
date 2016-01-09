(defvar *libjointpathexc* (load-foreign (format nil "~A/../../lib/libJointPathExC.so" (ros::resolve-ros-path "package://hrpsys"))))

(defforeign _initializeOpenHRPModel
  *libjointpathexc*
  "initializeOpenHRPModel"
  (:string) ;; _filename
  :integer)

(defforeign _initializeJointPathExInstance
  *libjointpathexc*
  "initializeJointPathExInstance"
  (:string  ;; root link
   :string) ;; target link
  :integer)

(defforeign _setJointAngles
  *libjointpathexc*
  "_setJointAngles"
  (:string) ;; ja
  :integer)

(defforeign _getJointAngles
  *libjointpathexc*
  "_getJointAngles"
  (:string) ;; ja
  :integer)

(defforeign _calcInverseKinematics2Loop
  *libjointpathexc*
  "_calcInverseKinematics2Loop"
  (:string :string) ;; vel_p, vel_r
  :integer)

#|
(defun test-joint-path-ex ()
  (_initializeOpenHRPModel (format nil "~A/../OpenHRP-3.1/sample/model/sample1.wrl" (ros::resolve-ros-path "package://openhrp3")))
  (_initializeJointPathExInstance "WAIST" "LLEG_ANKLE_R")
  (load "package://hrpsys_ros_bridge/models/samplerobot.l")
  (objects (list (setq *robot* (samplerobot))))
  (_setJointAngles (map float-vector #'deg2rad (send *robot* :angle-vector)))
  (setq av (instantiate float-vector (length (send *robot* :angle-vector))))
  (_getJointAngles av)
  (send *robot* :angle-vector (map float-vector #'rad2deg av))
  (do-until-key
   (_calcInverseKinematics2Loop #f(0 0 0.001) #F(0 0 0))
   (_getJointAngles av)
   (send *robot* :angle-vector (map float-vector #'rad2deg av))
   (send *irtviewer* :draw-objects))
  )
#|

