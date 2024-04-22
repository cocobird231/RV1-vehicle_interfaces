# Change Log
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

## [Unreleased] - 2024-04-22

- Rename `ControlServer.msg` to `ControlServerStatus.msg` to avoid confliction under Matlab ROS2 toolbox.
- Add `reason` field to `ControlServer.srv`.
- New `DataServerStatus.msg` and `DataServer.srv`.

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