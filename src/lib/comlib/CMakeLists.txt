cmake_minimum_required(VERSION 2.8.3)
project (comlib)
#include(${CMAKE_INSTALL_PREFIX}/../CMakeLists.txt)

option(ROSASYNCTIMERSWITCH "Run in android system" OFF)
option(ANDROIDSWITCH "Run in android system" OFF)
option(X86SWITCH "Run in X86_64-PC system" OFF)
if(ANDROIDSWITCH)
	add_definitions(-DANDROIDLOG)
	add_definitions(-DE01S)
endif(ANDROIDSWITCH)
if(X86SWITCH)
	add_definitions(-DX86SERIAL)
	add_definitions(-DE01S)
endif(X86SWITCH)

find_package(catkin REQUIRED COMPONENTS roscpp common_msgs)

# Specify locations of header files .
include_directories(include serial/include timer/include ${catkin_INCLUDE_DIRS})
link_directories(${PROJECT_SOURCE_DIR}/lib ${catkin_LIBRARIES} ${CMAKE_INSTALL_PREFIX}/lib/)

#message("keen1:#############################"${PROJECT_SOURCE_DIR})
#message("keen1:#############################"${catkin_LIBRARIES})

#add_message_files(
#  FILES
#  data.msg
#  msgdata.msg
#)

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

# Declare our catkin package .
catkin_package()

add_library(tiny_xml_proc  
   src/configer.cpp
   src/tinystr.cpp
   src/tinyxml.cpp
   src/tinyxmlerror.cpp 
   src/tinyxmlparser
)
     
add_library(endian_convert 
     src/endianconvert.c)          

add_library(ulog  
     src/ulog.cpp src/umutex.cpp)
     
add_library(order  
     src/orsbase.cpp src/orrbase.cpp)   
     
add_library(serial  
	 serial/src/serialinit.cpp
     serial/src/boostserial.cpp)
     
if(ROSASYNCTIMERSWITCH)	 
	add_library(asynctimer  
	 	timer/src/rosasynctimer.cpp)
else()	 	
	add_library(asynctimer  
	 	timer/src/asynctimer.cpp)
endif()	 	 

# Specify libraries against which to link. Again , this
# line should be copied for each distinct executable in
# the package .
if(ANDROIDSWITCH)
	target_link_libraries(ulog ${catkin_LIBRARIES} log)
elseif(X86SWITCH)
	target_link_libraries(ulog ${catkin_LIBRARIES})
else()
	target_link_libraries(ulog ${catkin_LIBRARIES})
	#target_link_libraries(hwrlib ${catkin_LIBRARIES} inputs_user outputs_user)
endif()
target_link_libraries(order ${catkin_LIBRARIES} ulog)
target_link_libraries(serial ${catkin_LIBRARIES} ulog order)
target_link_libraries(asynctimer ${catkin_LIBRARIES} ulog)

#install(DIRECTORY lib/
#   DESTINATION ${CATKIN_GLOBAL_LIB_DESTINATION}
#   FILES_MATCHING PATTERN "*.so")

install(FILES serial/include/serialinit.h
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
  
install(FILES timer/include/asynctimer.h
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}) 

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")

install(TARGETS ulog
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}) 
  
install(TARGETS endian_convert
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION})   
  
install(TARGETS tiny_xml_proc
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION})   
  
install(TARGETS order
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION})      
  
install(TARGETS serial
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION})  
  
  #message("keen1:#############################"${CATKIN_PACKAGE_LIB_DESTINATION})
