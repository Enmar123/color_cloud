# color_cloud

This package can map colors from cameras onto pointclouds, giving you a colored
pointcloud!

![color_cloud](https://github.com/Enmar123/color_cloud/blob/master/readme_images/color_cloud_rviz.png "rviz")

##To get started:##

* edit the color_cloud.launch file to reflect your setup
* `roslaunch color_cloud color_cloud.launch`

##Launch Parameters##

- **fov_width**: field of view width of your camera (0 < x < 180)
- **fov_height**: field of view height of your camera (0 < x < 180)
- **width_offset**: positive or negative pixel shift horizontally (+-#)
- **height_offset**: positive or negative pixel shift vertically (+-#)

##Common problems##

* Colored cloud wont display
  * is your pointcloud within the camera field of view (FOV)?
  * are your transforms set up properly?

* My colors dont overlay the pointcloud
  * check if your transforms have the correct dimensions
  * try tweaking the fov and offset parameters in the launch file
  * make sure to use a rectified image for your image topic

* My points are in the wong location
  * your camera frame must be setup following the camera standard
    * z-axis: into the screen
    * x-axis: pointing right
    * y-axis: pointing down

##Known issues##

* will produce an xyzrgb pointcloud only
  * values such as intensity do not transfer over
* error on startup (doesnt affect performance)

