# Raspberry Pi Monitor

A ROS 2 package that provides diagnostic info about the Raspberry Pi.

Tested on a Raspberry Pi 4B but should work on other models.

## Build and run

```bash
cd ~/ws/src
git clone https://github.com/pipebots/rpi4-monitor.git
cd ~/ws
colcon build
```

and to launch:

```bash
. ./install.setup.bash
ros2 launch rpi_monitor monitor.launch.py
```

## Acknowledgments

This work is supported by the UK's Engineering and Physical Sciences Research Council (EPSRC) Programme Grant EP/S016813/1

© 2023, University of Leeds.

The author, A. Blight, has asserted his moral rights.
