#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from scipy.spatial import KDTree 
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
import math

LOOKAHEAD_WPS = 50 
MAX_DECEL = 0.5

class WaypointUpdater(object):
    
    def __init__(self):
        rospy.init_node('waypoint_updater')        
        self.base_lane = None
        self.prev_state = self.now_state = -1
        self.pose = None
        self.stopline_wp_idx = -1
        self.waypoints_2d = None
        self.waypoint_tree = None
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb) #simulator updates /current_pose topic
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb) #read only once in callback because it never changes 
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb) #read light position information
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.loop()
    
    def loop(self):
        rate = rospy.Rate(30) 
        while not rospy.is_shutdown():
            if self.pose and self.base_lane:
                self.publish_waypoints() #invoked periodically 30 times each second
            rate.sleep()
            
    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x,y],1)[1] #get closest waypoint id
  
        #check if closest is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx] #get closest point coordinate
        prev_coord = self.waypoints_2d[closest_idx - 1] #get previous position coordinate

        #identify weather closest waypoint behind or ahead of the car
        #equation for hyperplane through closest_coords
        #cl_vect(closest waypoint) - prev_vect is a vector
        #pos_vect(car's position) - cl_vect is another vector        
        cl_vect = np.array(closest_coord) 
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x,y])         

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect) 
        #  if dot product is positive the closest waypoint is behind the car
        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)

        return closest_idx
    
    def publish_waypoints(self):
        final_lane = self.generate_lane()        
        self.final_waypoints_pub.publish(final_lane)
        
    def generate_lane (self):
        lane = Lane()        
        closest_idx = self.get_closest_waypoint_idx() #find closest waypoint ahead
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_lane.waypoints[closest_idx:farthest_idx]
        
        if self.stopline_wp_idx == -1 or self.stopline_wp_idx >= farthest_idx: #red light ahead in close distance (-1 is for green light) ?
            #no
            lane.waypoints = base_waypoints 
        else:
            #yes
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)
        return lane

    def decelerate_waypoints(self, waypoints, closest_idx):#slow down
        temp = [] 
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose 
            #calculate distance between closest waypoint and stopline           
            stop_idx = max(self.stopline_wp_idx - closest_idx - 3 , 0)  # -3 is to ensure nose of the car is behind the light
            dist = self.distance(waypoints, i, stop_idx)
            #calculate velocity - decellerate
            vel = math.sqrt(2 * MAX_DECEL * dist)
            if vel < 1.:
                vel = 0                
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)            
        return temp
    
    def pose_cb(self, msg):
        self.pose = msg #set current position x and y

    def waypoints_cb(self, waypoints):#get waypoints only one time 
        self.base_lane = waypoints
        if not self.waypoints_2d:#wre they received before ?
            #no - get them
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)
                                 
    def traffic_cb(self, msg):#get red light info
        self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
