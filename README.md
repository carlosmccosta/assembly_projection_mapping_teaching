# assembly_projection_mapping_teaching

## Overview

This ROS package provides an immersive projection mapping system for interactively teaching assembly operations.

This project has the following associated paper:

[Modeling of video projectors in OpenGL for implementing a spatial augmented reality teaching system for assembly operations](https://www.researchgate.net/publication/332083524_Modeling_of_video_projectors_in_OpenGL_for_implementing_a_spatial_augmented_reality_teaching_system_for_assembly_operations)


Currently it has assembly instructions for a Mitsubishi M000T20873 starter motor, but it can be easily reconfigured to other tasks (you just need to add the content to the [media](media) folder and change the [yaml/assembly.yaml](yaml/assembly.yaml) file).


[![Assisted assembly of a starter motor](http://img.youtube.com/vi/KWiw9Gbkx2I/maxresdefault.jpg)](http://www.youtube.com/watch?v=KWiw9Gbkx2I)

Video 1: Assisted assembly of a starter motor


[![Immersive natural interaction for assisted assembly operations](http://img.youtube.com/vi/pYHGaGZzmJw/maxresdefault.jpg)](http://www.youtube.com/watch?v=pYHGaGZzmJw)

Video 2: Immersive natural interaction for assisted assembly operations


[![Object pose estimation for assisted assembly operations](http://img.youtube.com/vi/557vglPW6Ko/maxresdefault.jpg)](http://www.youtube.com/watch?v=557vglPW6Ko)

Video 3: Object pose estimation for assisted assembly operations


[![Projection mapping for assisted assembly operations](http://img.youtube.com/vi/vfYDPL8DXGY/maxresdefault.jpg)](http://www.youtube.com/watch?v=vfYDPL8DXGY)

Video 4: Projection mapping for assisted assembly operations


[![Disassembly of a starter motor](http://img.youtube.com/vi/USEo3qots5g/maxresdefault.jpg)](http://www.youtube.com/watch?v=USEo3qots5g)

Video 5: Disassembly of a starter motor



## Software installation

Quick overview of the main installation steps:

* Install [ROS](http://wiki.ros.org/melodic/Installation/Ubuntu)
* [Upgrade gazebo to at least version 9.12](http://gazebosim.org/tutorials?cat=install&tut=install_ubuntu&ver=9.0) for getting [this pull request](https://bitbucket.org/osrf/gazebo/pull-requests/2058/added-camera-intrinsics-fx-fy-cx-cy-s)
* Install the [catkin-tools build system](https://catkin-tools.readthedocs.io/en/latest/installing.html)
* Create a [catkin tools workspace](http://catkin-tools.readthedocs.io/en/latest/quick_start.html) using [this script](https://github.com/carlosmccosta/ros_development_tools/blob/master/catkin/create_catkin_tools_ws.bash) for building all the libraries and executables
* Comile my fork of gazebo_ros and gpm:
  * [gazebo_ros_pkgs](https://github.com/carlosmccosta/gazebo_ros_pkgs)
  * [gazebo_projection_mapping](https://github.com/inesc-tec-robotics/gazebo_projection_mapping)
  * Compile using the [catkin build command](http://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html)
* Install or compile the 3D perception system:
  * [dynamic_robot_localization](https://github.com/carlosmccosta/dynamic_robot_localization#installation)
* Compile the teaching system:
  * [occupancy_detection](https://github.com/carlosmccosta/occupancy_detection)
  * [assembly_projection_mapping_teaching](https://github.com/carlosmccosta/assembly_projection_mapping_teaching)


### Notes:

Before compiling packages, check if you have installed all the required dependencies (use [rosdep](http://wiki.ros.org/rosdep) to speedup this task):
```
cd ~/catkin_ws
rosdep check --from-paths src --ignore-src
```



## Hardware setup and calibration

This package was tested with a [Asus Xtion Pro Live](https://www.asus.com/3D-Sensor/Xtion_PRO_LIVE/) for object recognition and a [Kinect 2](http://www.xbox.com/en-US/xbox-one/accessories/kinect) for bare hand human machine interaction.

You will need to install the hardware drivers and calibrate them.

Sensors drivers:
* [Asus Xtion Pro Live](http://wiki.ros.org/openni2_launch)
* [Kinect 2](https://github.com/code-iai/iai_kinect2)

Sensors calibration:
* [Asus Xtion Pro Live](http://wiki.ros.org/openni_launch/Tutorials)
* [Kinect 2](https://github.com/code-iai/iai_kinect2/tree/master/kinect2_calibration)

You will also need to calibrate the projector (using for example [this tool](http://mesh.brown.edu/calibration/)) and update the intrinsics parameters of the rendering camera in [worlds/assembly.world](worlds/assembly.world).

Finally, you will need to compute the extrinsics (position and rotation in relation to the chessboard origin) of the sensors and projector (using the [charuco_detector](https://github.com/carlosmccosta/charuco_detector)) and update [launch/assembly_tfs.launch](launch/assembly_tfs.launch).



## Usage

The teaching system can be started with the following launch  file:

```
roslaunch assembly_projection_mapping_teaching assembly.launch
```

You can start several modules of the system individually (such as sensors, rendering, perception). Look into the [launch](launch) folder and [tests.txt](docs/tests.txt).

After the system is started, you can navigate between the textual / video instructions using the projected buttons and can also pause / play / seek the video. In the last step it is projected into the workspace the outline of the 3D model for visual inspection and assembly validation. Check the videos above for a demonstration of the system functionality.
