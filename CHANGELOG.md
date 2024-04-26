# Change Log
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).


---


## [v1.2.0] - 2024-04-26

- Release stable version 1.2.
- Add `README.md` for the `vehicle_interfaces` package.

### Added
- `README.md`


---


## [Unreleased] - 2024-04-25

- Update lock and thread mechanism for `devinfo.h` and `qos.h`.
- Add `msg_show` namespace under `msg_json.h`.
- Update `HierarchicalPrint` under `utils.h`.

### Added
- `msg_json.h` add `msg_show` namespace, support message to `Hierarchical` object.
    - Point2d
    - MappingData
    - MotorValueRange
    - ChassisInfo
    - ControllerInfo
    - ControlServerStatus
    - DataServerStatus

### Changed
- `HierarchicalPrint` support `std::ostream& operator<<`, ex:
    ```cpp
    auto pt = vehicle_interfaces::msg::Point2d();
    pt.x = 1.0;
    pt.y = 2.0;
    std::cout << "Point2d: " << vehicle_interfaces::msg_show::Point2d::hprint(pt) << std::endl;
    // Expected output: 
    // ---<Point2d>
    //    |--x: 1.0
    //    \__y: 2.0
    ```
- `HierarchicalPrint` support combination:
    - Concat two `Hierarchical` with `<<` operator:
        ```cpp
        HierarchicalPrint hp;
        hp.push(0, "root");
        hp.push(1, "child1");
        hp.push(2, "child1-1");
        hp.push(1, "child2");
        HierarchicalPrint hp2;
        hp2.push(1, "child3");
        hp2.push(2, "child3-1");
        hp2.push(1, "child4");
        hp << hp2;
        std::cout << hp << std::endl;
        // Expected output:
        // ---<root>
        //    |--child1
        //    |  \__child1-1
        //    |--child2
        //    |--child3
        //    |  \__child3-1
        //    \__child4
        ```
    - Concat `Hierarchical` with `append()` can specify the level:
        ```cpp
        hp.append(2, hp2);
        std::cout << hp << std::endl;
        // Expected output:
        // ---<root>
        //    |--child1
        //    |  \__child1-1
        //    \__child2
        //       |--child3
        //       |  \__child3-1
        //       \__child4
        ```


---


## [Unreleased] - 2024-04-22

- Rename `ControlServer.msg` to `ControlServerStatus.msg` to avoid confliction under Matlab ROS2 toolbox.
- Add `reason` field to `ControlServer.srv`.
- New `DataServerStatus.msg` and `DataServer.srv`.
- Fix `LiteTimer` stop issue.

### .msg
- Remove `ControlServer.msg`
- Add `ControlServerStatus.msg`
    - Same as `ControlServer.msg` with an additional `reason` field.
- Add `DataServerStatus.msg`
    - `server_action` to set the action of the data server.
    - `server_scan_timer_status` and `server_scan_period_ms` to modify the scan timer.
    - `server_sample_timer_status` and `server_sample_period_ms` to modify the sample timer.
    - `server_dump_timer_status` and `server_dump_period_ms` to modify the dump timer.
    - `server_countdown_timer_status` and `server_countdown_period_ms` to modify the countdown timer.

### .srv
- Modify `ControlServer.srv`
    - Add `reason` field to describe the response failure reason.
- Add `DataServer.srv`
    - request: `DataServerStatus`. Set the data server status.
    - response: `bool` determining if the request was successful.
    - reason: `string` describing the response failure reason.
    - status: `DataServerStatus`. Get current data server status.

### Fixed
- The `LiteTimer` can now be restarted after stopping.


---


## [Unreleased] - 2024-04-17

- New modify .msg and new .srv files.
- New library `vehicle_interfaces.cpplib` for python3.
    ```bash
    # The vehicle_interfaces need to be built first.
    colcon build --packages-select vehicle_interfaces --symlink-install

    # Build vehicle_interfaces with python3 support.
    colcon build --packages-select vehicle_interfaces --symlink-install --cmake-args -DPYLIB=TRUE
    ```

### .msg
- Modify `ControllerInfo.msg`
    - change `PUB_TYPE_XXX` names.
    - add `node_name` variable.

### .srv
- Add `SlamDeviceInfo.srv` for SLAM device information request.
- Add `SlamFrameReq.srv` for SLAM device data request.

### Added
- Python3 support new `LiteTimer` class and `make_unqiue_timer` function.
    ```python3
    from vehicle_interfaces.cpplib import make_unique_timer, Timer
    ```
- `timer.h`
    - New `LiteTimer` class.
    - New `TimerPool` class.
    - New `make_unique_timer` function.
    - New `make_shared_timer` function.
    - Modify `Timer` class.

### Changed

#### Cpp
- `srv/pybind11.cpp` suport new `LiteTimer` class.
- CMakeLists.txt add `pybind11` support.

- `control.h`
    - Change Timer* to shared_ptr<Timer>.
    - Rewrite `BaseControllerServer` structure and constructor arguments.
    - Rewrite `BaseControllerClient` structure and constructor arguments.
    - Rewrite `ControllerServer` structure and constructor arguments.
    - Rewrite `ControllerClient` structure and constructor arguments.
    - Add comments.
    - New node naming strategy.

- `qos.h` Update timer.
- `qos2.h` Update timer and safe thread.

- `timesync.h`
    - Update timer and safe thread.

- `utils.h`
    - Update timer and safe thread.
    - Move `Time` class to `timer.h`
    - Add `MsgDataSender` and `MsgDataReceiver` classes.
    - Modifies `SpinExecutor` and `SpinExecutor2` function arguments.
        - Use shared_ptr instead of raw pointer.

- `vehicle_interfaces.h`
    - Remove reference from `VehicleServiceNode` constructor argument.

#### Python3
- `timesync.py`
    - Remove `Time` class.
    - import `LiteTimer` class and `make_unique_timer` function.

- `__init__.py`
    - `from vehicle_interfaces.cpplib import *`

### Fixed
- The `TopicRecordSaveQueueNode` (formerly `SubSaveQueueNode`) will not always push the data into queue, instead it will push the data into queue only if callback function is enabled.