# lidar_dewarping
Experiments with de-warping lidar scans

RPLidar A1 360 degree scanners (or clones of it / similar devices removed from Neato robots) are available for <$100.  

Unfortuantely, they update at a relatively slow rate of about 5-7Hz. In other words, it takes about 150ms for one scan of the lidar to complete.   During this time, the robot is of course moving, which results in the lidar scan "image" getting distorted.  An analogy would be the motion blur that takes place in photography when the camera or the object is moving and the exposure time of the camera is too high.  In our case, the time it takes for a lidar scan to complete its full 360 degrees is the exposure time.  (The analogy is not perfect, because the effect we get with the lidar is not so much a "blurring" effect but a warping effect - see pictures below)

What's the point of this:   The SLAM-type solutions available with ROS seem to make the assumption that the lidar scan takes place instantaneously and that it contains zero distortions. Tee algorithms tend to get confused when the lidar scan data contains distortion due to robot movement.

The purpose of the code here is to remove the distortions from the lidar scan.

One could of course simply upgrade to a RPLidar A3 which spins quite a bit faster, but   a) what's the fun in that?    b) the A3 costs $600 instead of $100
 
When the robot (red arrow) is stationary in the corner of a hallway, this is what the lidar scan looks like. Very much as expected.

![Original data - without motion](https://github.com/nettercm/lidar_dewarping/raw/master/original_no-motion_1a.png)


Once the robot starts rotating in place, i.e. turning, the data looks like this however.  

![Original data - with motion](https://github.com/nettercm/lidar_dewarping/raw/master/original_with-motion_1a.png)

Granted, the above example of continuously turning in place is a bit of an extreme case, for the purpose of illustrating the problem and for the purpose of evaluating solutions, but the fact of the matter is that algorithms like hector slam and gmapping don't like the distorted scan data at all.  Rotational motion is in fact the biggest issue.  Translation (i.e. just going straight) does not cause nearly as much confusion in those algorithms.

How to de-warp?

The basic idea is to look at the robot's motion, i.e. pose as a function of time, and use this information to de-warp the warped "image".


Current status:

I did get this to work.  Below is a picture that shows the de-warped (white) and orignal (yellow) scan.   However, performance of gmapping did not improve as much as I hoped.  Maybe I just didn't tune gmapping well enough.....

![Dewarped and original data](https://github.com/nettercm/lidar_dewarping/blob/master/dewarped-with-motion-1a.png)
