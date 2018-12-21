#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
import math
import numpy as np
from numpy import linalg as la
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import csv
import os

#This code is used to determine the car's next waypoint goal based on its localization. It does not consider any obstacles, that is handled by the local planner that subscribes to the topic that this node publishes the goal to. 

class next_waypoint:

    def __init__(self):

        self.LOOKAHEAD_DISTANCE = 1.5 # meters
        self.goal = 0
        self.read_waypoints()

	#Publisher for the goal point
	self.goal_pub = rospy.Publisher('/waypoint/goal', Point, queue_size=1)

        rospy.Subscriber('/pf/viz/inferred_pose', PoseStamped, self.callback, queue_size=1)

    # Import waypoints.csv into a list (path_points)
    def read_waypoints(self):

        dirname  = os.path.dirname(__file__)
        filename = os.path.join(dirname, '../waypoints/levine_waypoints.csv')

        with open(filename) as f:
            path_points = [tuple(line) for line in csv.reader(f)]

        # Turn path_points into a list of floats to eliminate the need for casts in the code below.
        self.path_points_x   = [float(point[0]) for point in path_points]
        self.path_points_y   = [float(point[1]) for point in path_points]
        self.path_points_w   = [float(point[2]) for point in path_points]

        self.dist_arr= np.zeros(len(self.path_points_y))

   
    # Computes the Euclidean distance between two 2D points p1 and p2.
    def dist(self, p1, p2):
        return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    # Input data is PoseStamped message from topic /pf/viz/inferred_pose.
    # Determines the next waypoint based on lookahead distance and sets a pose angle for this waypoint
    def callback(self,data):

        qx=data.pose.orientation.x
        qy=data.pose.orientation.y
        qz=data.pose.orientation.z
        qw=data.pose.orientation.w

        quaternion = (qx,qy,qz,qw)
        euler   = euler_from_quaternion(quaternion)
        yaw     = euler[2] 

        x = data.pose.position.x
        y = data.pose.position.y

        self.path_points_x = np.array(self.path_points_x)
        self.path_points_y = np.array(self.path_points_y)

        ## finding the distance of each waypoint from the current position 

        for i in range(len(self.path_points_x)):
            self.dist_arr[i] = self.dist((self.path_points_x[i],self.path_points_y[i]),(x,y))

        ##finding those points which are less than the look ahead distance (will be behind and ahead of the vehicle)

        goal_arr = np.where((self.dist_arr < self.LOOKAHEAD_DISTANCE+0.3)&(self.dist_arr > self.LOOKAHEAD_DISTANCE-0.3))[0]

        ##finding the goal point which is the last in the set of points less than the lookahead distance
        ##if the closest points array could not be formed, then the point which is closest to the current position is the goal. 
        
        for idx in goal_arr:
            v1 = [self.path_points_x[idx]-x , self.path_points_y[idx]-y]
            v2 = [np.cos(yaw), np.sin(yaw)]
            temp_angle = self.find_angle(v1,v2)
            if abs(temp_angle) < np.pi/2:
                self.goal = idx
                break

        ##finding the distance of the goal point from the vehicle coordinatesr

        L = self.dist_arr[self.goal]

        ##Transforming the goal point into the vehicle coordinate frame 

        gvcx = self.path_points_x[self.goal] - x
        gvcy = self.path_points_y[self.goal] - y 
        goal_x_veh_coord = gvcx*np.cos(yaw) + gvcy*np.sin(yaw)
        goal_y_veh_coord = gvcy*np.cos(yaw) - gvcx*np.sin(yaw)

        # math: find the curvature and the angle 
        alpha = self.path_points_w[self.goal] - (yaw)
        k = 2 * math.sin(alpha)/L
        angle_i = math.atan(k*0.4)

        angle = angle_i/2.05
        angle = np.clip(angle, -0.4189, 0.4189) # 0.4189 radians = 24 degrees because car can only turn 24 degrees max

	#publish the goal in the vehicle coordinates. This topic is read by the RRT local planner for creating a trajectory. 
	goalPoint = Point(float(goal_x_veh_coord),float(goal_y_veh_coord),self.path_points_w[self.goal]);
	self.goal_pub.publish(goalPoint)

       # print functions for DEBUGGING
       # print(self.path_points_x[self.goal],self.path_points_y[self.goal],self.path_points_w[self.goal])
       # print(x,y,180*yaw/math.pi)
       # print(goal_y_veh_coord,angle)
       # print("*******")


    # find the angle bewtween two vectors    
    def find_angle(self, v1, v2):
        cosang = np.dot(v1, v2)
        sinang = la.norm(np.cross(v1, v2))
        return np.arctan2(sinang, cosang)


if __name__ == '__main__':
    rospy.init_node('next_waypoint')
    C = next_waypoint()  
    r = rospy.Rate(40)

    while not rospy.is_shutdown():
        r.sleep()
