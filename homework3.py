#!/usr/bin/env python

import rospy
import math
import sys

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

x_speed = 0.0 #0.8 #0.3 #m/s
z_rotation = 120 * math.pi / 180 #0.0 #degrees/s
forward_distance = 5.0
mode = 'bot'
rotate = True
target_rotation = 360 * math.pi / 180
#front_range = 99999999.0
#initial_range = -1

def minIndex(arr):
    min_i = -1
    min_v = 9999999
    for i in range(len(arr)):
        if arr[i] < min_v:
            min_i = i
            min_v = arr[i]
    return min_i

class Status:
    def __init__(self):
        self.front_range = 5.0
        self.initial_range = -1
        self.pose_set = False
        self.final_pose = (0,0)
        self.initial_pose = (0,0)
         
        self.initial_yaw = 0
        self.final_yaw = 9999999

        self.initial_min_i = -1
        self.final_min_i = -1
        self.angle_increment = 0.0
        self.yaw_set = False
        self.full_circles = 0

    def set_pose(self,data):
        #self.final_pose = data.twist.twist
        #return
        #self.odom_dist = data.pose.pose.position.x
        #print(self.initial_pose)
        
        if not self.pose_set:
            self.initial_pose = (data.pose.pose.position.x, data.pose.pose.position.y)
            self.pose_set = True
        self.final_pose = (data.pose.pose.position.x, data.pose.pose.position.y)
        
        #print("%s, %s" % (self.final_yaw, self.initial_yaw))
        yaw = data.pose.pose.orientation.z * math.pi
        if abs(abs(self.final_yaw) - yaw) < 0.00001:
            return
        elif abs(self.final_yaw) < yaw:
            yaw = -yaw
        else:
            pass
        
        #if self.yaw_set and yaw > self.initial_yaw and self.final_yaw < self.initial_yaw:
        #    self.over2pi = True
        if self.final_yaw < 99999 and not self.yaw_set:
            self.initial_yaw = yaw
            self.yaw_set = True
        if self.final_yaw < 99999 and abs(self.final_yaw - yaw) > math.pi / 2:
            self.full_circles += 1

        self.final_yaw = yaw

        #print("%s, %s circles, %s raw diff, %s, %s" % (self.get_yaw_change(), self.full_circles, self.final_yaw - self.initial_yaw, self.final_yaw, yaw))
        #self.final_yaw = data.pose.pose.orientation.z * math.pi
        #print(self.final_yaw)
        #if not self.yaw_set:
        #    self.initial_yaw = data.pose.pose.orientation.z * math.pi
        #    self.yaw_set = True
        #elif mode == 'stage':
        #    self.final_pose = data.pose.pose.orientation.z
    def set_range(self, data):
        #rospy.loginfo(str(self.front_range))
        mid = int(math.floor(len(data.ranges)/2))
        self.front_range = data.ranges[mid]
        if self.initial_range == -1:
             self.initial_range = self.front_range
        
        #print(dir(data.ranges))
        self.angle_increment = data.angle_increment
        min_i = minIndex(data.ranges) 
        #print("%s, %s" % (self.final_min_i, self.initial_min_i))
        self.final_min_i = min_i
        if self.initial_min_i < 0:
            self.initial_min_i = min_i
            #self.yaw_set = True

    def get_distance(self):
        return - self.front_range + self.initial_range
    def get_odom_distance(self):
        #print("|%s to %s" % (self.initial_pose, self.final_pose))
        return math.sqrt(pow(self.final_pose[0] - self.initial_pose[0], 2) + pow(self.final_pose[1] - self.initial_pose[1], 2))
    def get_yaw_change(self):
        return -2 * math.pi * self.full_circles + self.final_yaw - self.initial_yaw
        #return self.final_yaw - self.initial_yaw
    def get_measured_yaw_change(self):
        di = self.final_min_i - self.initial_min_i
        return di * self.angle_increment

status = Status()

def callbackOdometry(data):
    #Log pose
    #rospy.loginfo(rospy.get_caller_id() + 'The pose is %s', data.twist)
    global status
    status.set_pose(data)
def callbackLaser(data):
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.ranges[342])
    global status
    status.set_range(data)

def primary():
    if len(sys.argv) > 1 and sys.argv[1] == 'stage':
        global mode
        mode = 'stage'

    #Create node
    rospy.init_node('move')

    # Publish to cmd_vel
    p = rospy.Publisher('cmd_vel', Twist)

    # Subscribe to Odometry
    if mode == 'bot':
        rospy.Subscriber('/pose', Odometry, callbackOdometry)
    elif mode == 'stage':
        rospy.Subscriber('/odom', Odometry, callbackOdometry)

    #Hokuyo Laser Subsrciber
    #/scan for ACTUAL ROBOT!!!
    if mode == 'bot':
        rospy.Subscriber('/scan', LaserScan, callbackLaser)
    elif mode == 'stage':
        rospy.Subscriber('/base_scan_1', LaserScan, callbackLaser)
    
    #Create Twist message
    twist = Twist()
    twist.linear.x = x_speed;                   # forward speed
    twist.angular.z = z_rotation;               # rotation

    # Log move
    rospy.loginfo("I am moving forward")

    #Change range to change time elapsed and thereby distance travelled
    #34 for 0.3 m/s, 1m
    #167 for 0.3 m/s, 5m
    #13 for 0.8 m/s, 1m 
    #63 for 0.8 m/s, 5m
    #3 for 30 degrees/s. 10 degrees?
    #54 for 30 degrees/s, 180 degrees?
    #for i in range(34):
    i = 0
    if not rotate:
      while True:
        #print(status.get_odom_distance())
        #if i > 120: break
        i = i + 1
        if status.pose_set and status.get_odom_distance() >= forward_distance: break 
        p.publish(twist)
        rospy.sleep(0.1) # 30*0.3 = 1.0  Change back to 30 for range
    else:
        while True:
            if status.yaw_set and abs(status.get_yaw_change()) >= target_rotation: break
            p.publish(twist)
            rospy.sleep(0.1)

    # Create Twist message (Defaults set to 0)
    twist.linear.x = 0.0
    twist.angular.z = 0.0

    #Log stop
    p.publish(twist)
    rospy.loginfo("I am stopping")
    #rospy.sleep(1.0)
    rospy.loginfo("Initial laser %s, final laser %s" % (status.initial_range, status.front_range))
    rospy.loginfo("Initial odom %s, final odom %s" % (status.initial_pose, status.final_pose))
    rospy.loginfo("Laser distance: " + str(status.get_distance()))
    rospy.loginfo("Odom distance: " + str(status.get_odom_distance()))
    rospy.loginfo("Odom Rotation: %s. (Initial: %s, Final: %s)" % (str(status.get_yaw_change()), status.initial_yaw, status.final_yaw))
    rospy.loginfo("Laser rotation: %s. (Initial %s, Final: %s)" % (status.get_measured_yaw_change(), status.initial_min_i * status.angle_increment, status.final_min_i * status.angle_increment))
    rospy.loginfo("Measured rotation: " + str(status.get_measured_yaw_change()))

if __name__=="__main__":
    primary()
