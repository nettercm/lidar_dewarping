#!/usr/bin/env python2

print("starging dwarp_scan.py");

import time, signal, sys, threading, math, os, copy
from math import cos,sin,pi

import rospy
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point,Pose,Quaternion,Twist,Vector3,PointStamped
from std_msgs.msg import Int16
from sensor_msgs.msg import LaserScan,PointCloud2,PointCloud

import sensor_msgs.point_cloud2 as pc2
import laser_geometry.laser_geometry as lg



def rads(degrees):
    """
    convert from degrees to radians
    """
    return degrees * pi / 180.0



def degs(radians):
    """
    convert from radians to degrees
    """
    return radians * 180 / pi



def norm(theta):
    """
    normalize the angle theta so that it always falls between -pi and +pi
    """
    TWO_PI = 2.0 * pi
    normalized = theta % TWO_PI;
    normalized = (normalized + TWO_PI) % TWO_PI;
    if normalized > pi:
        normalized = normalized - TWO_PI;
    return normalized



def euler_angle_from_pose(pose):
    """
    extract the angle theta from the x,y,z,w quaternion that is part of pose structure
    return theta
    """
    quaternion = ( pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    th= euler[2]
    th= norm(th)
    return th




def project(msg):
    # convert the message of type LaserScan to a PointCloud2
    pc2_msg = lp.projectLaser(msg)

    # now we can do something with the PointCloud2 for example:
    # publish it
    projection_pub.publish(pc2_msg)
    
    # convert it to a generator of the individual points
    #point_generator = pc2.read_points(pc2_msg)
    

    # we can access a generator in a loop
    #for point in point_generator:
    #    continue

    # or a list of the individual points which is less efficient
    #point_list = pc2.read_points_list(pc2_msg)

    # we can access the point list with an index, each element is a namedtuple
    # we can access the elements by name, the generator does not yield namedtuples!
    # if we convert it to a list and back this possibility is lost
    #print(point_list[len(point_list)/2].x)
    return




class odom_history:

    def __init__(self,size=12):
        self.s=size
        self.h=[]
        self.i=0  #index i is the the next slot in the ringbuffer, so basically start w/ the oldest entry
        self.l=threading.RLock()     
        self.th_now=0.0
        self.th_last=0.0


    def append(self,item):
        self.lock()
        if len(self.h) < self.s:
            self.h.append(item)
        else:
            self.h[self.i] = item

        self.i = self.i + 1
        if self.i >= self.s:
            self.i = 0

        self.th_now = euler_angle_from_pose(item.pose.pose)
        #print("odom deg/sec=%6.2f" % (degs(self.th_now - self.th_last)*100))
        self.th_last = self.th_now
        self.release()


    def show(self):
        print(self.h)


    def lock(self):
        #self.l.acquire()
        return

    def release(self):
        #self.l.release()
        return 


    def len(self):
        return len(self.h)


    def size(self):
        return self.s


    def get_oldest_item(self):
        return self.h[self.i]


    def ready(self):
        return self.len() == self.size()


    def get_interpolated_pose(self,sample_time):
        """
        Input:
        1) self.h == odom history array
        2) time  (usually corresponding to the time stamp of one individual laser range sample point)
    
        Output:
        1) x, y, theta corresponding to the given time - usually interpolated because actual odom data is only available for specific discrete points in time
        """
        self.lock()

        ts1=time.time()

        #find the entry in the odom history ringbuffer that corresponds to sample_time
        n = self.i
        m = self.i+self.s
        for j in range(n,m):   
            if sample_time < self.h[j % self.s].header.stamp:
                break #ok we found the slot 

        ts2=time.time()

        #use j-1 because that corresponds to the slot that's just right
        t1 =    self.h[(j-1-0) % self.s].header.stamp
        pose1 = self.h[(j-1-0) % self.s].pose.pose
        th1 = euler_angle_from_pose(pose1)
        x1 = pose1.position.x
        y1 = pose1.position.y


        ts3=time.time()

        t2 =    self.h[(j-0) % self.s].header.stamp
        pose2 = self.h[(j-0) % self.s].pose.pose
        th2 = euler_angle_from_pose(pose2)
        x2 = pose2.position.x
        y2 = pose2.position.y

        ts4=time.time()

        t3 = sample_time
        x3 = x1  + ((t3-t1)/(t2-t1)) * (x2  - x1)
        y3 = y1  + ((t3-t1)/(t2-t1)) * (y2  - y1)
        th3= th1 + ((t3-t1)/(t2-t1)) * norm(th2 - th1)
        th3= norm(th3)

        ts5=time.time()

        self.release()

        #print("%6.2f, %6.2f, %6.2f, %6.2f, %6.2f" % ( (ts2-ts1)*1000, (ts3-ts2)*1000, (ts4-ts3)*1000, (ts5-ts4)*1000, (ts5-ts1)*1000) )


        #print("%7.1f , %7.3f , %7.3f , %6.3f ,   %7.1f , %7.3f , %7.3f , %6.3f ,   %7.1f , %7.3f , %7.3f , %6.3f ,    %6.3f" % (ms(t1),x1,y1,th1,ms(t2),x2,y2,th2,ms(t3),x3,y3,th3,norm(th3-th1)))

        #print(j % self.s)

        return (x3,y3,th3)
        

        

def ms(time):
    global start_time
    t = (time - start_time).to_nsec() / 100000
    t = int(t)
    t = float(t/10.0)
    return t



def odom_callback(msg):

    #if start_time == 0:
    #    start_time = msg.header.stamp

    o.append(msg)

    return	    



def process_scan(scan_msg):
    global o

    #if we have not collected a full ring buffer's worth of data yet, return
    if not o.ready():
        return

    #this is to ensure that the most up to date odom data is available
    rospy.sleep(0.01)

    p = PointCloud()

    ts1=time.time()
    o.lock()

    t = scan_msg.header.stamp
    #t = t - rospy.Duration(scan_msg.time_increment * len(scan_msg.ranges) * 1.0)
    (robot_x1,robot_y1,robot_th1) = o.get_interpolated_pose( t )
    robot_th1 = norm(robot_th1)
    
    for i in range(0,len(scan_msg.ranges)):
        t = t + rospy.Duration(scan_msg.time_increment)
        laser_d  = scan_msg.ranges[i]
        laser_th = scan_msg.angle_min + (i * scan_msg.angle_increment)
        laser_th = norm(laser_th)

        if laser_d == float("inf"):   #if its an invalid reading, don't bother
            continue

        (robot_x,robot_y,robot_th) = o.get_interpolated_pose( t )
        robot_dx = robot_x1 - robot_x
        robot_dy = robot_y1 - robot_y
        robot_dth= norm(robot_th1 - robot_th)

        #laser_th = laser_th + pi #to make things line up... should really use tf for this...
        x = robot_dx + laser_d * cos(robot_dth + laser_th)
        y = robot_dy + laser_d * sin(robot_dth + laser_th)

        p.points.append(Point(x,y,0))

    #in case the last sample was invalid, we need to do this
    (robot_x,robot_y,robot_th) = o.get_interpolated_pose( t )
    robot_dx = robot_x1 - robot_x
    robot_dy = robot_y1 - robot_y
    robot_dth= norm(robot_th1 - robot_th)


    o.release()

    ts2=time.time()

    #print("robot_dth=%6.2f   deg/sec=%6.2f" % (-degs(robot_dth) , -(degs(robot_dth) / scan_msg.scan_time)) )

    """
    now transform (rotate than translate) the point cloud so that the origin corresponds to robot pose
    associated with the last laser ray 
    robot_dx, robot_dy and robot_th already contain the right values
    """

    p.header.stamp = t   # t corresponds to the time of the last laser ray in the scan
    p.header.frame_id = 'laser'
    cloud_pub.publish(p)


    #for some reasone, the pointcloud is twisting in the same direction as robot is twisting

    #the retoation transform is supposed to fix this, but w/out any adjustment to the rotation angle, 
    #the resulting scan is moving in the opposite direction of robot turning by a quite a bit
    robot_dth2 = robot_dth / 4  

    ts3=time.time()

    for point in p.points:
        # rotation:
        x = point.x
        y = point.y
        xp= x*cos(robot_dth2) - y*sin(robot_dth2)
        yp= y*cos(robot_dth2) + x*sin(robot_dth2)

        # translation:
        # first we need to rotate the offset, then we can add it
        dx = robot_dx*cos(robot_dth2) - robot_dy*sin(robot_dth2)
        dy = robot_dy*cos(robot_dth2) + robot_dx*sin(robot_dth2)
        xpp = xp - dx
        ypp = yp - dy

        point.x = xpp
        point.y = ypp


    ts4=time.time()

    """
    now convert the point cloud into a LaserScan message and publish it

    the following python code is based on the c++ code from
    https://github.com/ros-perception/pointcloud_to_laserscan/blob/lunar-devel/src/pointcloud_to_laserscan_nodelet.cpp
    """

    l = LaserScan()
    l.header.stamp = t  # t corresponds to the time of the last laser ray in the scan
    l.header.frame_id = 'laser'
    # let the de-warped LaserScan msg have the same basic parameters as the original scan message
    l.angle_min = scan_msg.angle_min
    l.angle_max = scan_msg.angle_max
    l.angle_increment = scan_msg.angle_increment
    l.time_increment = 0.0 # since our de-warped scan has time taken into account...
    l.scan_time = 0.0 # let's pretend total scan time was 0, i.e. instantaneous
    l.range_min = 0.0  # scan_msg.range_min
    l.range_max = 20.0 # scan_msg.range_max

    #initialize the arrays
    #the following will go from 0....<360  we need to round up with +1.5 to make sure we get the full range
    for i in range(0,int((l.angle_max - l.angle_min) / l.angle_increment + 1.5)):
        l.ranges.append(float("inf"))
        l.intensities.append(0.0)

    for point in p.points:
        x = point.x
        y = point.y
        r = math.hypot(x,y)  # calculate the lenght of the vector
        a = math.atan2(y,x)  # caluclate the angle of the vector
        index = int( (a - l.angle_min) / l.angle_increment + 0.5 )  # now convert angle to array index
        if r < l.ranges[index]:  # update the array; we may override what's alrady there, but only if the new reading is smaller
            l.ranges[index] = r
            l.intensities[index] = 100.0 # original scan usually has 47.0

    ts5=time.time()

    # now we have a dewarped laser scan ...

    # what's left to do is to remove the artifact that appears in this dewarped scan
    # when the robot is twisting right, then the artifact is on the left  and vice a versa
    # root cause is unknown
    # as a workaround, just filter out (i.e. invalidate by setting to inf) readings in a certain angle range
    # input 

    #if abs(degs(robot_dth)) > 5:
    #    filter_scan(l,rads(135.0), rads(45.0) )
    #    print("robot_dth=%5.2fdeg" % degs(robot_dth))

    removed_items = filter_scan(l, robot_dth)

    ts6=time.time()

    scan_pub.publish(l)
    scan3_pub.publish(removed_items)

    ts7=time.time()

    #the first section of this function seems to take 99% of the time
    #print("%6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f" % ( (ts2-ts1)*1000, (ts3-ts2)*1000, (ts4-ts3)*1000, (ts5-ts4)*1000, (ts6-ts5)*1000, (ts7-ts6)*1000, (ts7-ts1)*1000 ) )

"""
standing still, facing wall straight on
rosbag play --loop --start=0 --duration=10 2020-02-16-11-52-35.bag

"""
filter_dth_last=0

def filter_scan(scan,robot_dth):
    """
    remove, i.e. invalidate, readings from input scan starting at start_angle for a range of angle_range
    """
    global filter_dth_last
    start_angle = rads(135.0)
    angle_range = rads(30.0)
    s = len(scan.ranges)
    removed_items = copy.deepcopy(scan)
    for i in range(0,s):
        removed_items.ranges[i] = float("inf")
        removed_items.intensities[i % s] = 0.0

    print("@t= %6.2f: robot_dth= %6.2f deg" % (ms(scan.header.stamp), degs(robot_dth)))

    rate_of_change = (degs(abs(robot_dth - filter_dth_last)))
    
    #if robot is not twisting fast, don't filter anything out
    if abs(degs(robot_dth)) < 5 and rate_of_change < 10:
        filter_dth_last = robot_dth
        return removed_items
        #start_angle = rads(180.0)
        #angle_range = rads(45.0)
        #start_angle = rads(157.5)
        #angle_range = rads(45)

        
    #180,45 is front left, 45degree cone
    #135,45 is front right, 45degree cone
    #157,45 is front center, 45degree cone
    #=>postive increase in starting angle means moving the cone left i.e. CCW
    if degs(robot_dth) < -2.0:
        start_angle = rads(135.0)
        angle_range = rads(90.0)
        
    if degs(robot_dth) < -25.0:
        #need to make this a function of twist speed; at slower speeds, need to filter closer to the front
        #for dth=-30, 155, 25 works well, but not for slower turns
        start_angle = rads(155.0)
        angle_range = rads(25.0)

    if degs(robot_dth) > 5.0:
        start_angle = rads(185.0)
        angle_range = rads(20.0)

    if rate_of_change > 10:
        print("rate of turn speed change too high")
        start_angle = rads(0)
        angle_range = rads(359)
        
    filter_dth_last = robot_dth

    start_index = int( (start_angle - scan.angle_min) / scan.angle_increment + 0.5 )
    count = int(angle_range / scan.angle_increment)
    for i in range(start_index, start_index+count):
        removed_items.ranges[i % s] = scan.ranges[i % s]
        removed_items.intensities[i % s] = scan.intensities[i % s]
        scan.ranges[i % s] = float("inf")
        scan.intensities[i % s] = 0.0

    return removed_items


def scan_callback(scan_msg):
    process_scan(scan_msg)
    #project(scan_msg)
    return




rospy.init_node('odometry_monitor')

odom_sub =  rospy.Subscriber("odom",     Odometry,     odom_callback)
scan_sub =  rospy.Subscriber("scan",     LaserScan,    scan_callback)
#enc_sub  = rospy.Subscriber("encoders", PointStamped, enc_callback)

scan_pub       = rospy.Publisher("scan_2",              LaserScan,   queue_size=10, tcp_nodelay=True) # dewarped scan
scan3_pub      = rospy.Publisher("scan_3",              LaserScan,   queue_size=10, tcp_nodelay=True) # for diagnostics
projection_pub = rospy.Publisher("projection_pub",      PointCloud2, queue_size=10, tcp_nodelay=True) # testing only 
cloud_pub      = rospy.Publisher("cloud_from_laser_pub",PointCloud,  queue_size=10, tcp_nodelay=True) # for diagnostic - intermediate representation


TWO_PI = 2.0 * pi

start_time = rospy.Duration(0)
time_offset = rospy.Time.now()

o = odom_history(20)

lp = lg.LaserProjection()



while not rospy.is_shutdown():
    rospy.spin()
    
    
