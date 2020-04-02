# covent
A collection of components to help control an open source ventilator

This combines componets from several other projects to allow a simple ventilator control system to be built.

Initially using a Raspberry Pi and ROS for rapid development and deployment.

Once things get a little better defined ESP8266 or ESP32 systems could take over the whole control.

Initial components are PI4 , RoboClaw V5C motor controller https://www.pololu.com/product/3284 and ROS
The motor control has been copied from the JPL Opensource Rover project and modified to run just one controller.
Other components have been scavenged from different projects, some using MQTT, these will be turned into ROS components.

Why ROS ? http://wiki.ros.org

This may look like an overkill to get a "simple" project up and running but it does provide a great messaging infrastructure and a wealth of ready to run components for later integration.


Base Directories

notes - some initial notes / documentation

mqtt - a load of code examples to get peripherals running with MQTT used as a code base for teh ROS work

ROS - initial set of ROS components

esp8266 - start of a collection of controller components

esp32  - same for the 32 bit variant.

