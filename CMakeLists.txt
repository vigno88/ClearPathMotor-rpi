cmake_minimum_required(VERSION 2.8.3)
project(teknic_motor_controller)

set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(Threads REQUIRED)
#find_package(CMAKE_DL_LIBS REQUIRED)
add_compile_options(-lpthread)

#find_package(
#  COMPONENTS
#  # message_generation
## sensor_msgs
#)

include_directories(
  include
  include/inc
  include/inc/inc-private
  include/inc/inc-private/linux
  include/inc/inc-private/sFound
  include/inc/inc-pub
  include/LibINI/inc
  include/LibLinuxOS/inc
  include/LibXML/inc
  include/LibMotors/inc
  ${Threads_INCLUDE_DIRS}
)

add_executable(Motor_Test
  src/motor_control/motor_test.cpp
)

add_executable(Homing_Test
  src/motor_control/homing_test.cpp	
)

#add_executable(Example-MultiThreaded
#  src/SDK_Examples/Example-MultiThreaded/Example-MultiThreaded.cpp
#  src/SDK_Examples/Example-MultiThreaded/Axis.cpp
#  src/SDK_Examples/Example-MultiThreaded/Supervisor.cpp
#)

add_library(LibINI
  src/LibINI/src/dictionary.cpp
  src/LibINI/src/iniparser.cpp
)

add_library(LibLinuxOS
  src/LibLinuxOS/src/tekEventsLinux.cpp
  src/LibLinuxOS/src/tekThreadsLinux.cpp

  src/LibLinuxOS/src/version.cpp
)

add_library(LibXML
  src/LibXML/src/ErrCodeStr.cpp
  src/LibXML/src/pugixml.cpp
)

add_library(sFoundation
  src/sFoundation/src/converterLib.cpp
  src/sFoundation/src/cpmAPI.cpp
  src/sFoundation/src/cpmClassImpl.cpp
  src/sFoundation/src/iscAPI.cpp
  src/sFoundation/src/lnkAccessCommon.cpp
  src/sFoundation/src/meridianNet.cpp
  src/sFoundation/src/netCmdAPI.cpp
  src/sFoundation/src/netCoreFmt.cpp
  src/sFoundation/src/SerialEx.cpp
  src/sFoundation/src/sysClassImpl.cpp
  src/sFoundation/src-linux/lnkAccessLinux.cpp
  src/sFoundation/src-linux/SerialLinux.cpp
)

add_library(LibMotors STATIC
  src/LibMotors/src/motors.cpp
  src/LibMotors/src/clearPathMotors.cpp
  src/LibINI/src/dictionary.cpp
  src/LibINI/src/iniparser.cpp
  src/LibLinuxOS/src/tekEventsLinux.cpp
  src/LibLinuxOS/src/tekThreadsLinux.cpp
  src/LibLinuxOS/src/version.cpp
  src/LibXML/src/ErrCodeStr.cpp
  src/LibXML/src/pugixml.cpp
  src/sFoundation/src/converterLib.cpp
  src/sFoundation/src/cpmAPI.cpp
  src/sFoundation/src/cpmClassImpl.cpp
  src/sFoundation/src/iscAPI.cpp
  src/sFoundation/src/lnkAccessCommon.cpp
  src/sFoundation/src/meridianNet.cpp
  src/sFoundation/src/netCmdAPI.cpp
  src/sFoundation/src/netCoreFmt.cpp
  src/sFoundation/src/SerialEx.cpp
  src/sFoundation/src/sysClassImpl.cpp
  src/sFoundation/src-linux/lnkAccessLinux.cpp
  src/sFoundation/src-linux/SerialLinux.cpp
)


target_link_libraries(sFoundation LibINI LibLinuxOS LibXML ${CMAKE_DL_LIBS} Threads::Threads)
#target_link_libraries(sFoundation LibINI LibLinuxOS LibXML Threads::Threads)
target_link_libraries(LibMotors sFoundation LibINI LibLinuxOS LibXML ${CMAKE_DL_LIBS} Threads::Threads)

#target_link_libraries(trash_bot_motor_controller sFoundation)
target_link_libraries(Motor_Test sFoundation)
target_link_libraries(Homing_Test sFoundation)
# target_link_libraries(Example-MultiThreaded sFoundation)
