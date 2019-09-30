# color_cloud

This package can map colors from cameras onto pointclouds, giving you a colored
pointcloud!

**To get started:**

* edit the color_cloud.launch file to reflect your setup
* `roslaunch color_cloud color_cloud.launch`

**Common problems:**

* Colored cloud wont display
  * is your pointcloud within the camera field of view (FOV)?
  * are your transforms set up properly?

* My colors dont overlay the pointcloud
  * check if your transforms have the correct dimensions
  * try tweaking the fov paramter in the launch file
  * make sure to use a rectified image for your image topic

**Known issues:**

* currently publishes at a fixed rate
