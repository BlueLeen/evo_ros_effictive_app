cmake_minimum_required(VERSION 2.8.3)
project(common_msgs)
include(${CMAKE_INSTALL_PREFIX}/../CMakeLists.txt)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

option(EVODEBUG "Use tutorial provided math implementation" OFF)
configure_file(${PROJECT_SOURCE_DIR}/include/config.h.in ${PROJECT_SOURCE_DIR}/include/config.h)

message(${EVODEBUG})
add_message_files(
  DIRECTORY msg
  FILES
  serdata.msg
  msgdata.msg
  single_status.msg
  power_detection_status.msg
  camera_status.msg
  camera_info.msg
  camera_info_list.msg
  infrared_info.msg
  infrared_info_list.msg
  infrared_threshold.msg
  ultra_info.msg
  ultra_info_list.msg
  ultra_work_status.msg
  ultra_charge_pile_feedback.msg
  ultra_charge_pile_status.msg
  ultra_air_quality_feedback.msg
  ultra_id_sn.msg
  upgrade_comm.msg
  dead_reckon_info.msg
  set_head_move.msg
  set_wing_move.msg  
  set_wheel_differential_motion.msg  
  set_wheel_fixed_motion.msg
  motion_result_feedback.msg
  motion_id_sn.msg
  head_feedback.msg
  wing_feedback.msg
  log.msg
  get_calibration_info.msg
  charge_lead_info.msg
  Point.msg
  Pose2D.msg
  TrajectoryInfo.msg
  TrajectorytoMotor.msg)

add_service_files(
  DIRECTORY srv
  FILES
  set_node_status.srv
  get_power_detection_status.srv
  infrared_init_data.srv
  get_charge_band_status.srv
  get_ultra_work_status.srv
  set_ultra_work_status.srv
  get_ultra_charge_pile_feedback.srv
  set_ultra_charge_pile_status.srv
  get_node_work_status.srv
  set_uint16_status.srv
  set_uint8_status.srv
  get_ultra_blind_feedback.srv
  get_ultra_idsn.srv
  get_motion_idsn.srv
  get_head_angle.srv
  get_wing_angle.srv
  set_dead_reckon_info.srv
  request_wheels_motion.srv
  requset_infrared.srv
  set_ultra_blind_status.srv
  get_ultra_calibration_feedback.srv
  reg_dbg_service.srv
  dbg_service.srv
  get_recalibration_info.srv
  get_charge_lead_data.srv
  set_infrared_work_status.srv
  get_driver_check_status.srv
  get_io_charge_status.srv
)

generate_messages(DEPENDENCIES std_msgs) 
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
