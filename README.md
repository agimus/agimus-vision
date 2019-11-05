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

Run the command blender-2.78 --background --python scripts/april-tag-to-dae.py -- -h to get some help on how to do this.
