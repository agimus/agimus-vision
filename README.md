This ROS node will find the AprilTag 0 in the video stream and publish its pose in the TF tree


Dependencies:
* ROS Kinetic
* OpenCV3
* Visp3 and visp\_bridge (sudo apt-get install ros-kinetic-vision-visp)
* libxml2


Installation:
mkdir build && cd build && cmake .. && make install

Usage:
rosrun agimus-vision tracker\_box

There are some ros parameters used to configure the node:
* **/imageTopic**: Topic where the camera publish the RGB image (default: /camera/rgb/image\_rect\_color).
* **/cameraInfoTopic**: Topic where the camera publish the information about the camera settings (default: /camera/rgb/camera\_info).
* **/positionNode**: Name of the TF node corresponding to the box position (default: box).
* **/positionParentNode**: Name of its parent node in the TF tree (default: rgbd\_rbg\_optical\_frame).wn

### Generate DAE from AprilTag png files

Moved to gerard bauzil package.

### Debugging images on the robot

- We emulate a virtual display on the robot. On the robot, run the
  command in a terminal and leave it open.
```bash
Xvfb :1 -screen 0 800x600x16
```
- We start the tracker node within this display, by setting the environment
  variable DISPLAY to `:1` for the tracker node executable.
```bash
export DISPLAY=:1
```
- Once the node is launched, you can ask for the debug to be published. There
  are two parameters for this: *~debugDisplay* and *~publishDebugDisplay*.
  Something similar to `rosparam set /vision/tracker/debugDisplay true` followed
  by `rosparam set /vision/tracker/publishDebugDisplay true` should enable the
  publication.
```bash
rosparam set /vision/tracker/publishDebugDisplay true
rosparam set /vision/tracker/debugDisplay true
```
- On another terminal, possibly on another computer, run:
```bash
rosrun image_view image_view image:=/vision/tracker/debug _image_transport:=compressed
```
