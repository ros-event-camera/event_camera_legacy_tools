# event\_camera\_legacy\_tools

This repository holds ROS1 and ROS2 tools for backwards compatibility of the
[event\_camera\_msgs](https://github.com/ros-event-camera/event_camera_msgs) format
with the older
[dvs\_msgs](https://github.com/ros-event-camera/dvs_msgs) and
[prophesee\_event\_msgs](https://github.com/ros-event-camera/prophesee_event_msgs)
formats.
Future use of dvs\_msgs and prophesee\_event\_msgs is discouraged as they
have large storage requirements, cause high CPU load under ROS2,
and perform poorly in high event rate scenarios (both under ROS1 and ROS2)

## Supported platforms

ROS2 is supported for ROS2 Humble and later distros.
ROS1 is EOL but the ROS1 code is left in this repository and may still compile. Pull requests for fixes against the ROS1 code will be accepted but no new features.

## How to build

Set the following shell variables:
```bash
repo=event_camera_legacy_tools
url=https://github.com/ros-event-camera/${repo}.git
```
and follow the [instructions here](https://github.com/ros-misc-utilities/.github/blob/master/docs/build_ros_repository.md)

## Republish conversion nodelet

The ``republish`` node converts event\_camera\_msgs to legacy formats
like dvs, prophesee, and decoded ("mono") event_camera messages. Note
that this node will consume a significant amount of CPU
resources and should not be run unnecessarily while recording data.
The following command will start a conversion node to republish
events from ``/event_camera/events`` to
``/event_camera/republished_events`` and
``/event_camera/republished_triggers``
(see launch file for remapping):

```bash
ros2 launch event_camera_legacy_tools republish_composable.launch.py camera:=event_camera message_type:=event_packet
```

## Tools

Convert bags with DVS or Prophesee messages to evt3 event\_camera\_msgs:

```bash
ros2 run event_camera_legacy_tools legacy_to_bag -b <input_bag_file> -o <output_bag_file> -t <topic1>[topic2 topic3 ...]
```


## License

This software is issued under the Apache License Version 2.0.
