*`Established: 2024/04/26`* *`Updated: 2024/04/29`*

## About The Project
The ROS2 interface and useful service codes for the robot vehicle ver. 1 project.

### Files under `include`
- `control.h`: implement the `ControllerServer` and `ControllerClient` classes.
- `devinfo.h`: implement the `DevInfoNode` and `DevInfoServer` classes.
- `interactive_node.h` (not used):
- `interactive_publisher.h` (not used):
- `interactive_subscription.h` (not used):
- `interactive_topic.h` (not used):
- `msg_json.h`: four namespaces, `msg_to_json`, `json_to_msg`, `msg_to_msg` and `msg_show`, to convert the message to json, json to message, message to message, and show the message, respectively.
- `params.h`: implement the `GenericParams` class for `VehicleServiceNode` constructor to set the parameters.
- `qos.h`: implement the `QoSUpdateNode` and `QoSServer` classes to update the QoS profile.
- `qos2.h` (not used):
- `safety.h`: implement the `SafetyNode` and `SafetyServer` classes to manage the safety status.
- `timer.h`: implement the `LiteTimer`,  `TimerPool` and `Timer` classes.
- `timesync.h`: implement the `TimeSyncNode` and `TimeSyncServer` classes to synchronize the time.
- `utils.h`: implement the useful classes and functions, such as `unique_thread`, `HierarchicalPrint`, `SpinExecutor`, `split` , `replace_all`, etc..
- `vehicle_interfaces.h`: implement the `VehicleServiceNode` class derived from `DevInfoNode`, `QoSUpdateNode`, `SafetyNode`, `TimeSyncNode`, and `rclcpp::Node` classes.

### Files under `lib`
- `devinfo.py`: implement the `DevInfoNode` for python node.
- `node_adaptor.py`: implement the `NodeAdaptor` class to solve the `VehicleServiceNode` multiple inheritance problem.
- `params.py`: implement the `GenericParams` class for `VehicleServiceNode` constructor to set the parameters.
- `qos.py`: implement the `QoSUpdateNode` for python node.
- `safety.py`: implement the `SafetyNode` for python node.
- `timesync.py`: implement the `TimeSyncNode` for python node.
- `utils.py`: implement the useful classes and functions, such as `TopicNames` and `ConnToService`.
- `vehicle_interfaces.py`: implement the `VehicleServiceNode` for python node.

### Files under `msg`
- `Chassis.msg`: describe the chassis information.
- `Distance.msg`: describe the sensor distance information, e.g. ultrasound sensor.
- `Environment.msg`: describe the environment information, e.g. temperature, humidity, etc..
- `GPS.msg`: describe the GPS information.
- `GroundDetect.msg` (not used):
- `Header.msg`: the header for all messages under the `msg` directory.
- `IDTable.msg` (not used):
- `Image.msg`: describe the image information using uint8 array.
- `IMU.msg`: describe the 9-DoF IMU information.
- `MillitBrakeMotor.msg` (not used):
- `MillitPowerMotor.msg` (not used):
- `MotorAxle.msg`: describe the axle motor information.
- `MotorSteering.msg`: describe the steering motor information.
- `QosUpdate.msg`: describe the QoS update information.
- `SteeringWheel.msg`: describe the steering wheel information.
- `UPS.msg` (not used):
- `WheelState.msg` (not used):

### Files under `msg_content`
- `ChassisInfo.msg`: describe the chassis component information.
- `ControlChassis.msg`: the control signal between ControllerServer and ControllerClient.
- `ControllerInfo.msg`: describe the controller information.
- `ControlServerStatus.msg`: describe the control server status.
- `ControlSteeringWheel.msg`: the control signal between ControllerServer and ControllerClient.
- `DataServerStatus.msg`: describe the data server status.
- `DevInfo.msg`: describe the device information.
- `InteractiveNodeMasterPrivilege.msg` (not used):
- `InteractiveNode.msg` (not used):
- `MappingData.msg`: describe two vectors mapping data.
- `MotorValueRange.msg`: describe the motor value range.
- `QosProfile.msg`: describe the QoS profile.
- `SurroundEmergency.msg`: describe the surround emergency information.
- `TopicDeviceInfo.msg` (not used):

### Files under `msg_geo`
- `Bbox2d.msg` (not used): 
- `Point2d.msg`: 2d point in double.
- `Point2f.msg`: 2d point in float.
- `Size2f.msg`: 2d size in float.

### Files under `src`
- `pybind11.cpp`: bind `LiteTimer` to Python `Timer`.

### Files under `srv`
- `ControlChassisReg.srv` (not used):
- `ControlChassisReq.srv`: the control signal between ControllerServer and ControllerClient.
- `ControllerInfoReg.srv`: controller information registration.
- `ControllerInfoReq.srv`: controller information request.
- `ControlServer.srv`: get/set the control server status.
- `ControlSteeringWheelReg.srv` (not used):
- `ControlSteeringWheelReq.srv`: the control signal between ControllerServer and ControllerClient.
- `DataServer.srv`: get/set the data server status.
- `DevInfoReg.srv`: device information registration.
- `DevInfoReq.srv`: device information request.
- `IDServer.srv`(not used):
- `InteractiveNodeReq.srv` (not used): 
- `InteractiveNode.srv` (not used): 
- `QosReg.srv`: qos profile registration.
- `QosReq.srv`: qos profile request.
- `SafetyReg.srv`: safety status registration.
- `SafetyReq.srv`: safety status request.
- `SlamDeviceInfo.srv` (not used):
- `SlamFrameReq.srv` (not used): 
- `TimeSync.srv`: time synchronization.
- `TopicDeviceInfoReg.srv` (not used):



## Getting Started

### Prerequisites
- ROS2 `Foxy` or later (`Humble` recommended)
- `nlohmann-json3-dev`
- `iproute2`
- `pybind11-dev`

The required packages are listed in the `requirements_apt.txt` file.

### Installation
1. Make sure the ROS2 environment is properly installed. If not, please refer to the [ROS2 official website](https://docs.ros.org/en/humble/Installation.html) for installation, or simply run the following command:
    ```bash
    curl -fsSL ftp://61.220.23.239/scripts/install-ros2.sh | bash
    ```
    **NOTE:** The script only supports `Foxy` and `Humble` versions depending on the system.
    **NOTE:** The script will create a new workspace at `~/ros2_ws`.
    **NOTE:** The script will create an alias `humble` for `source /opt/ros/humble/setup.bash` or alias `foxy` for `source /opt/ros/foxy/setup.bash`.

2. Clone the repository under `~/ros2_ws/src` and rename it to `vehicle_interfaces`:
    ```bash
    git clone https://github.com/cocobird231/RV1-vehicle_interfaces.git vehicle_interfaces
    ```

3. Install the required packages:
    ```bash
    # Change the directory to the vehicle_interfaces package.
    cd ~/ros2_ws/src/vehicle_interfaces

    # Install the required packages.
    xargs sudo apt install -y < requirements_apt.txt
    ```

4. Build the interfaces:
    ```bash
    # Copy the vehicle_interfaces package to the workspace src directory.
    cp -rv vehicle_interfaces ~/ros2_ws/src

    # Source the ROS2 environment (humble)
    humble # source /opt/ros/humble/setup.bash

    # Build the interfaces
    cd ~/ros2_ws
    colcon build --symlink-install --packages-select vehicle_interfaces
    ```
    **NOTE:** vehicle_interfaces is installed in the local workspace.

5. Build the Python binding (optional):
    ```bash
    colcon build --packages-select vehicle_interfaces --cmake-args -DPYLIB=TRUE
    ```
    **NOTE:** The `step 4.` is required to build the Python binding.

### Usage
1. Source the local workspace:
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

2. Check the interfaces:
    ```bash
    # List the interfaces.
    ros2 interface list | grep vehicle_interfaces
    ```

3. Python binding (optional):
    ```python
    from vehicle_interfaces.cpplib import make_unique_timer, Timer

    # Create a unique timer.
    my_timer = make_unique_timer(period_ms, my_callback)
    my_timer.start()
    my_timer.stop()
    ```

### Known Issues
- For python nodes, the `ConnToService` function may cause the rclpy timer `wait_set.is_ready` issue at some environments.
    - The issue was discovered under `DevInfoNode` and `TimeSyncNode` while `ConnToService` function ends.
    - To temporarily solve the issue, disable the `DevInfoNode` and `TimeSyncNode` by passing empty string to both service name while constructing the node.
- The QoS profile update may not work properly in some scenarios.
    - Due to the QoS update mechanism is depending on each of the nodes, the update orders for publishers and subscriptions may cause the confliction of the QoS profile.