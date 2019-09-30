color_cloud

This package can map colors from cameras onto pointclouds, giving you a colored
pointcloud!

To get started:

* edit the color_cloud.launch file to reflect your setup
* `roslaunch color_cloud color_cloud.launch`

Common problems:

* Colored cloud wont display
  * is your pointcloud within the camera field of view (FOV)?
  * are your transforms set up properly?

Known issues:

* strange rviz rgb transparancy
* currently publishes at a fixed rate
