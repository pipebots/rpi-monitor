# Raspberry Pi Monitor

A ROS 2 package that provides diagnostic info about the Raspberry Pi.

Tested on a Raspberry Pi 4B but should work on other models.  Tested on ROS Humble and Jazzy.

Pages for [design notes](design.md) and [testing](testing.md) have also been written.

## ROS 2 Messages

The following topics and messages are used by this driver.

| Type | Topic | Message |
|---|---|---|
| Publisher | `diagnostics` | [diagnostic_msgs/msg/DiagnosticStatus](https://github.com/ros2/common_interfaces/blob/rolling/diagnostic_msgs/msg/DiagnosticStatus.msg)|

## Build and run

NOTE: where `~/ws` is used, replace with your own workspace path.

Clone the repo:

```bash
cd ~/ws/src
git clone https://github.com/pipebots/rpi4-monitor.git
```

Install pre-requisites:

```bash
sudo apt install ros-jazzy-diagnostics
```

Then build:

```bash
cd ~/ws
colcon build
```

And finally launch:

```bash
cd ~/ws
. ./install.setup.bash
ros2 launch rpi_monitor monitor.launch.py
```

## Acknowledgments

This work is supported by the UK's Engineering and Physical Sciences Research Council (EPSRC) Programme Grant EP/S016813/1

© 2023,2025 University of Leeds.

The author, A. Blight, has asserted his moral rights.
