cmake_minimum_required(VERSION 3.16)
project(vehicle_interfaces)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/Distance.msg"
  "msg/Environment.msg"
  "msg/GPS.msg"
  "msg/Header.msg"
  "msg/IDTable.msg"
  "msg/IMU.msg"
  "msg/Image.msg"
  "msg/WheelState.msg"
  "msg/MotorAxle.msg"
  "msg/MotorSteering.msg"
  "msg/UPS.msg"
)

set(srv_files
  "srv/IDServer.srv"
  "srv/SafetyReg.srv"
  "srv/SafetyReq.srv"
  "srv/TimeSync.srv"
)

## Build
rosidl_generate_interfaces(${PROJECT_NAME} ${msg_files} ${srv_files} DEPENDENCIES builtin_interfaces)


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


#find_package(Python COMPONENTS Interpreter Development)
#find_package(pybind11 CONFIG REQUIRED)
#find_package(${PROJECT_NAME} REQUIRED)

#set(src_files
#  "src/pybind11.cpp"
#)

#ament_python_install_package(${PROJECT_NAME})

#add_library(utils MODULE ${src_files})
#ament_target_dependencies(utils rclcpp vehicle_interfaces pybind11)

#install(
#  TARGETS utils
#  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
#)