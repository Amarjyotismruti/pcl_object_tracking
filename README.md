# pcl_object_tracking
Color based segmentation and point cloud extraction.(Current configurations are for tracking orange coloured objects, the colour can be varied by changing the hsv constraints)

The built source files are located in Project_1 folder. Follow the following steps in order to run the executables.

i. Copy the folder Project_1 to the catkin workspace/src folder.

ii. Build the project by using catkin_make command.

iii. Run either FreeNect_Launch or OpenNI_Launch, in order to access the point cloud and rectified rgb image topics from Microsoft Kinect.

iv. Launch either of the above in a terminal.

v. Run the "test_image" program and note the specific HSV values required to detect the object, and close after visualizing effective results.

vi. Change the HSV values in the Project_1/src/Tracker_object2.cpp file.

v. Run "Tracker_object2". The point cloud of the object can be visualized in rviz by choosing the topic "pointcloud/object".
