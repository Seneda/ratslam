ratslam
=======

Modifications to the OpenRatSLAM project at http://code.google.com/p/ratslam/


To Run This version of RatSLAM:

First install the dependencies as stated in the documentation for OpenRatSLAM : 
"Dependencies
OpenRatSLAM depends on ROS packages: opencv2 and topological_nav_msgs and also on 3D graphics library Irrlicht Irrlicht can be installed on Ubuntu with apt-get

sudo apt-get install libirrlicht-dev"

Then copy the contents of this repository into your Catkin workspace.

catkin_make

To run ratslam, open four terminals:

Launch RatSLAM
Terminal 1: roslaunch ratslam_ros test.launch

The odom topic needs to be renamed
Terminal 2: rosrun topic_tools relay /odom /test/odom

The Video feed needs to be renamed and the framerate reduced
Terminal 3: rosrun topic_tools drop /stereo/left/image_mono/compressed 3 4 /test/camera/image1/compressed

Run one of the bagfiles provided
Terminal 4: rosbag play FinalBagfiles/dynamic-control-2.bag 

