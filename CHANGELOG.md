# Change Log
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

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