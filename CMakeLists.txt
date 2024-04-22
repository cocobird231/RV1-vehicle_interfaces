cmake_minimum_required(VERSION 3.16)
project(vehicle_interfaces)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/Chassis.msg"
  "msg/Distance.msg"
  "msg/Environment.msg"
  "msg/GPS.msg"
  "msg/GroundDetect.msg"
  "msg/Header.msg"
  "msg/IDTable.msg"
  "msg/Image.msg"
  "msg/IMU.msg"
  "msg/MillitBrakeMotor.msg"
  "msg/MillitPowerMotor.msg"
  "msg/MotorAxle.msg"
  "msg/MotorSteering.msg"
  "msg/QosUpdate.msg"
  "msg/SteeringWheel.msg"
  "msg/UPS.msg"
  "msg/WheelState.msg"
)

set(msg_content_files
  "msg_content/ChassisInfo.msg"
  "msg_content/ControlChassis.msg"
  "msg_content/ControllerInfo.msg"
  "msg_content/ControlServerStatus.msg"
  "msg_content/ControlSteeringWheel.msg"
  "msg_content/DataServerStatus.msg"
  "msg_content/DevInfo.msg"
  "msg_content/InteractiveNode.msg"
  "msg_content/InteractiveNodeMasterPrivilege.msg"
  "msg_content/MappingData.msg"
  "msg_content/MotorValueRange.msg"
  "msg_content/QosProfile.msg"
  "msg_content/SurroundEmergency.msg"
  "msg_content/TopicDeviceInfo.msg"
)

set(msg_geo_files
  "msg_geo/Bbox2d.msg"
  "msg_geo/Point2d.msg"
  "msg_geo/Point2f.msg"
  "msg_geo/Size2f.msg"
)

set(srv_files
  "srv/ControlChassisReg.srv"
  "srv/ControlChassisReq.srv"
  "srv/ControllerInfoReg.srv"
  "srv/ControllerInfoReq.srv"
  "srv/ControlServer.srv"
  "srv/ControlSteeringWheelReg.srv"
  "srv/ControlSteeringWheelReq.srv"
  "srv/DataServer.srv"
  "srv/DevInfoReg.srv"
  "srv/DevInfoReq.srv"
  "srv/IDServer.srv"
  "srv/InteractiveNode.srv"
  "srv/InteractiveNodeReq.srv"
  "srv/QosReg.srv"
  "srv/QosReq.srv"
  "srv/SafetyReg.srv"
  "srv/SafetyReq.srv"
  "srv/TimeSync.srv"
  "srv/TopicDeviceInfoReg.srv"
  "srv/SlamDeviceInfo.srv"
  "srv/SlamFrameReq.srv"
)

SET(PYLIB False CACHE BOOL "Some user-specified option")
if( PYLIB ) 

find_package(Python COMPONENTS Interpreter Development)
find_package(pybind11 CONFIG REQUIRED)
find_package(${PROJECT_NAME} REQUIRED)

set(src_files
 "src/pybind11.cpp"
)

ament_python_install_package(${PROJECT_NAME})

add_library(cpplib MODULE ${src_files})
ament_target_dependencies(cpplib rclcpp vehicle_interfaces pybind11)
# pybind11_add_module(cpplib ${src_files})

set_target_properties(cpplib PROPERTIES PREFIX "")

if($ENV{ROS_DISTRO} STREQUAL "foxy")
  add_compile_options(-DROS_DISTRO=1)
  install(
    TARGETS cpplib
    DESTINATION lib/python3.8/site-packages/${PROJECT_NAME}
   )
elseif($ENV{ROS_DISTRO} STREQUAL "humble")
  add_compile_options(-DROS_DISTRO=2)
  install(
    TARGETS cpplib
    DESTINATION local/lib/python3.10/dist-packages/${PROJECT_NAME}
   )
else()# eloquent
  add_compile_options(-DROS_DISTRO=0)
  install(
  TARGETS cpplib
  DESTINATION lib/python3.6/site-packages/${PROJECT_NAME}
  )
endif()

else()

## Build
rosidl_generate_interfaces(${PROJECT_NAME} ${msg_files} ${msg_content_files} ${msg_geo_files} ${srv_files} DEPENDENCIES builtin_interfaces sensor_msgs)


## Install
ament_export_dependencies(rosidl_default_runtime)

if($ENV{ROS_DISTRO} STREQUAL "foxy")
  add_compile_options(-DROS_DISTRO=1)
  install(
    DIRECTORY include/
    DESTINATION include
  )
  install(
    DIRECTORY lib/
    DESTINATION lib/python3.8/site-packages/${PROJECT_NAME}
  )
  ament_export_include_directories(
    include
  )
elseif($ENV{ROS_DISTRO} STREQUAL "humble")
  add_compile_options(-DROS_DISTRO=2)
  install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
  )
  install(
    DIRECTORY lib/
    DESTINATION local/lib/python3.10/dist-packages/${PROJECT_NAME}
  )
  ament_export_include_directories(
    include
  )
else()# eloquent
  add_compile_options(-DROS_DISTRO=0)
  install(
    DIRECTORY include/
    DESTINATION include
  )
  install(
    DIRECTORY lib/
    DESTINATION lib/python3.6/site-packages/${PROJECT_NAME}
  )
  ament_export_include_directories(
    include
  )
endif()

ament_package()

endif()
