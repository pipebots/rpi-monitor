# Design Notes

This package publishes a single diagnostics message every 5 seconds that contains the following data:

* CPU usage
* CPU temperature
* Free memory

## Research

I spent about an hours looking for previous work on the internet and found the following:

* <http://wiki.ros.org/pr2_computer_monitor>
* <http://wiki.ros.org/libsensors_monitor>
* <https://tier4.github.io/autoware.iv/tree/main/system/system_monitor/> Docs
* <https://github.com/autowarefoundation/autoware_ai_utilities/tree/master/system_monitor> Code for above.

The `autowarefoundation` code looks the best but is ROS 1 so I can use it as inspiration.

## Implementation

Inspired by the Autoware Foundation code, I decided to give the Diagnostic Update method of publishing a try.  I also decided to do away with the base class approach as I on need this to work for the Raspberry Pi.  I can always refactor later if I need to use it on another CPU.

I setup the new package and then checked that it built.

## Comparison between Diagnostic Update and the "do it yourself" method

TODO Compare Dynamixel diff drive diagnostics with the updater method.