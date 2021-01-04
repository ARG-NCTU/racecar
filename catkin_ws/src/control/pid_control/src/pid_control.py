#!/usr/bin/env python

import time
import rospy
from nav_msgs.msg import Odometry
import numpy as np
from std_msgs.msg import Int32
from robotx_msgs.msg import roboteq_drive
from robotx_msgs.srv import waypoint
from std_srvs.srv import *
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class WAMV_PID():
	def __init__(self, pos_P=0.8, pos_I=0.005, pos_D=0.3, vel_P=0.4, vel_I=0.0, vel_D=0.0):
		self.node_name = rospy.get_name()
		
		self.is_close_distance = 3
		self.pos_Kp = pos_P
		self.pos_Ki = pos_I
		self.pos_Kd = pos_D
		self.vel_Kp = vel_P
		self.vel_Ki = vel_I
		self.vel_Kd = vel_D

		self.dis4constV =  # Distance for constant velocity
		self.pos_ctrl_max = 
		self.pos_ctrl_min = 
		self.pos_station_max =
		self.pos_station_min = 
		self.cmd_ctrl_max = 
		self.cmd_ctrl_min = 
		self.station_keeping_dis = 3.5
		self.frame+id = "map"
		self.is_station_keeping = False
		self.final_goal = None
		self.goal = self.final_goal

		rospy.loginfo("[%s] Initializing " &(self.node_name))

		rospy.Subscriber("odometry/filtered", Odometry, cb_nav)
		self.pub_vel = rospy.Publisher("cmd_drive", roboteq_drive, queue_size = 20)
		# start_srv = rospy.Service("start_waypoint_nav", Trigger, start_waypoint_handler)
		# stop_srv = rospy.Service("pause_waypoint_nav", Trigger, pause_waypoint_handler)
		# station_keep_srv = rospy.Service("station_keep", Trigger, station_keep_handler)
		# station_keep_srv = rospy.Service("station_keep_unlock", Trigger, station_keep_unlock_handler)
		# clear_waypoints_srv = rospy.Service("clear_waypoints", Trigger, clear_waypoints_handler)
		# add_waypoint_srv = rospy.Service("add_waypoint", waypoint, add_waypoint_handler)
		# add_current_loc_srv = rospy.Service("add_current_loc", Trigger, add_current_loc_handler)	
		# self.pub_marker = rospy.Publisher("waypoint_marker", Marker, queue_size = 10)
		# self.pub_wp_nav_state = rospy.Publisher("wp_nav_state", Int32, queue_size = 10)
		print("waiting for start srv")

		def publish_goal(self, goal):
			marker = Marker()
			marker.header.frame_id = self.frame_id
			marker.header.stamp = rospy.Time.now()
			marker.ns = "pure_pursuit"
			marker.type = marker.SPHERE
			marker.action = marker.ADD
			marker.pose.orientation.w = 1
			marker.pose.position.x = goal[0]
			marker.pose.position.y = goal[1]
			marker.id = 0
			marker.scale.x = 0.6
			marker.scale.y = 0.6
			marker.scale.z = 0.6
			marker.color.a = 1.0
			marker.color.g = 1.0
			self.pub_goal.publish(marker)



def cb_nav(msg):
    return

def start_waypoint_handler(req):
    return

def pause_waypoint_handler(req):
    return

def station_keep_handler(req):
    return

def station_keep_unlock_handler(req):
    return

def clear_waypoints_handler(req):
    return

def add_waypoint_handler(req):
    return

def add_current_loc_handler(req):
    return

if __name__ == "__main__":
	rospy.init_node("pid_control_node")
	
	# while start_flag == 0 and waypoints is None:	
	# 	rospy.sleep(0.1)