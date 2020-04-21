# lidar_dewarping
Experiments with de-warping lidar scans

RPLidar A1 360 degree scanners (or clones of it / similar devices removed from Neato robots) are available for <$100.  

Unfortuantely, they update at a relatively slow rate of about 5-7Hz. In other words, it takes about 150ms for one scan of the lidar to complete.   During this time, the robot is of course moving, which results in the lidar scan "image" getting distorted.  

The SLAM-type solutions available with ROS seem to make the assumption that the lidar scan takes place instantaneously and that it contains zero distortions. THe algorithms tend to get confused when the lidar scan data contains distortion due to robot movement.

The purpose of the code here is to remove the distortions from the lidar scan.
