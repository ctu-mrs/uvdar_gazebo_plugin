# UVDAR Gazebo plugin

![](.fig/thumbnail.jpg)

| Build status | [![Build Status](https://github.com/ctu-mrs/uvdar_gazebo_plugin/workflows/Melodic/badge.svg)](https://github.com/ctu-mrs/uvdar_gazebo_plugin/actions) | [![Build Status](https://github.com/ctu-mrs/uvdar_gazebo_plugin/workflows/Noetic/badge.svg)](https://github.com/ctu-mrs/uvdar_gazebo_plugin/actions) |
|--------------|-------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------|

## Description
This package contains plugins for generating synthetic outputs emulating the images produced by UV-sensitive cameras onboard of MAVs, observing blinking UV LEDs attached to targets
The output is used in simulations involving the [UVDAR System](https://github.com/ctu-mrs/uvdar_core) for mutual relative localization of MAVs

This package was developed for the [Gazebo robotic simulator](http://gazebosim.org/) and it has been tested with version 9.13.
Compatibility with other versions is currently not guaranteed.

## System requirements

#### Hardware:
Fairly powerful CPU. Ideally have at least one core/thread per each camera included in the simulation.
Additionally, the requirements increase further if you enable obstacle occlusions - this is necessary to test the effects of partial or complete occlusions on the precision of the estimation


#### Software
  * [ROS (Robot Operating System)](https://www.ros.org/) Melodic Morenia
  * [Gazebo robotic simulator](http://gazebosim.org/) - Gazebo simulator v. 9.13
  * [mrs_msgs](https://github.com/ctu-mrs/mrs_msgs) - ROS package with message types used by the MRS group
  * [uvdar_core](https://github.com/ctu-mrs/uvdar_core) - Processing of the UVDAR inputs

#### For testing
  * [mrs_uav_system](https://github.com/ctu-mrs/mrs_uav_system) Our ROS-based ecosystem for flying and testing multi-UAV systems. Includes simulation with examples of attaching this plugin to objects in the simulated world

## Installation
Install the dependencies.
Clone this repository into a ROS workspace as a package.
If you are using the `mrs_modules` meta package (currently only accessable internally to MRS staff, to be released at later date), this repository is already included.
Build the package using catkin tools (e.g. `catkin build uvdar_gazebo_plugin`)

## Testing
See in [uvdar_core](https://github.com/ctu-mrs/uvdar_core)

## Acknowledgements

### MRS group
This work would not be possible without the hard work, resources and support from the [Multi-Robot Systems (MRS)](http://mrs.felk.cvut.cz/) group from Czech Technical University.

### Included libraries
This package contains the following third-party libraries by the respective authors:
  * [OCamCalib](https://sites.google.com/site/scarabotix/ocamcalib-toolbox) calibration system by Davide Scaramuzza - only the C++ sources
