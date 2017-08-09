cmake_minimum_required(VERSION 2.8.3)
project (config)
#ADD_DEFINITIONS(-DANDROID)
include(${CMAKE_INSTALL_PREFIX}/../CMakeLists.txt)
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs roscpp)
#option(EVODEBUG "Use tutorial provided math implementation" OFF)

#add_subdirectory(xml)

#message(${EVODEBUG})
# Generate messages in the 'msg' folder
# add_message_files(
#   DIRECTORY cfms/msg
#   FILES
#   node_info.msg
#   node_generality.msg
#   wheels_modify.msg
#   codes_to_distance.msg
#   driver_config.msg
#   version_config.msg
#   local_config.msg
#   charge_config.msg
# )

# Generate services in the 'srv' folder
# add_service_files(
#   DIRECTORY cfms/srv
#   FILES
#   node_req.srv
#   time_info.srv
#   get_driver_config.srv
#   get_version_config.srv
#   get_local_config.srv
#   get_charge_config.srv
#   set_driver_config.srv
#   set_version_config.srv
#   set_local_config.srv
#   set_charge_config.srv
# )
 
# Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# ) 
# Declare our catkin package .
#catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
catkin_package()

# Specify locations of header files .
include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${PROJECT_SOURCE_DIR}/lib ${catkin_LIBRARIES} ${CMAKE_INSTALL_PREFIX}/lib/)

# Declare the executable , along with its sourcefiles . If
# there are multiple executables ,use multiple copies of
# this line .
add_executable (
	config
	#src/chargefile.cpp  
	#src/driboardfile.cpp
	src/config_main.cpp 
	#src/updatemanager.cpp
	#src/oldnodecfg.cpp
	#src/padmanager.cpp
	#src/configmanager.cpp
)
#add_dependencies(config config_generate_messages_cpp)
# Specify libraries against which to link. Again , this
# line should be copied for each distinct executable in
# the package .
#target_link_libraries(config ${catkin_LIBRARIES} fdp_serial_opt tiny_xml_proc)
#target_link_libraries(config ${catkin_LIBRARIES} ulog evolib endian_convert tiny_xml_proc)
target_link_libraries(config ${catkin_LIBRARIES} ulog)
install(TARGETS config
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

#install(DIRECTORY include/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h")
  
#install(FILES include/common.h
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
