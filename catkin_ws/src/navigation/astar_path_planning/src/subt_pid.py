#!/usr/bin/env python
import numpy as np
import cv2
import roslib
import rospy
import tf
import struct
import math
import time
from sensor_msgs.msg import Image, LaserScan
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Point, Twist
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
import rospkg
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from control.cfg import pos_PIDConfig, ang_PIDConfig
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import Bool
from PID import PID_control
from sensor_msgs.msg import Joy

class Robot_PID():
	def __init__(self):
		self.node_name = rospy.get_name()
		self.dis4constV = 3. # Distance for constant velocity
		self.pos_ctrl_max = 0.7
		self.pos_ctrl_min = 0
		self.ang_ctrl_max = 1.0
		self.ang_ctrl_min = -1.0
		self.turn_threshold = 20
		self.slow_speed = 0.1
		self.cmd_ctrl_max = 0.7
		self.cmd_ctrl_min = -0.7
		self.arrived_dis = 0.2 # meters
		#self.frame_id = 'map'
		self.frame_id = 'laser_frame'
		#self.frame_id = 'camera_link'
		self.emergency_stop = True
		self.final_goal = None # The final goal that you want to arrive
		self.goal = self.final_goal

		rospy.loginfo("[%s] Initializing " %(self.node_name))
		
		self.use_odom = rospy.get_param("use_odom")
		if not self.use_odom:
			rospy.Timer(rospy.Duration(1/30.), self.timer_cb)
			self.cmd = Twist()

		self.sub_goal = rospy.Subscriber("pursue_point", PoseStamped, self.goal_cb, queue_size=1)
		self.pub_arrive = rospy.Publisher("arrive", Bool, queue_size=1)

		if self.use_odom: rospy.Subscriber('odometry/ground_truth', Odometry, self.odom_cb, queue_size = 1, buff_size = 2**24)
		self.pub_cmd = rospy.Publisher("cmd_vel", Twist, queue_size = 1)
		self.pub_goal = rospy.Publisher("goal_point", Marker, queue_size = 1)
		self.pub_estop = rospy.Publisher("/racecar/stop_mode", Bool, queue_size = 1)
		self.emergency_stop_srv = rospy.Service("emergency_stop", SetBool, self.emergency_stop_cb)

		self.pos_control = PID_control("Position")
		self.ang_control = PID_control("Angular")

		self.pos_srv = Server(pos_PIDConfig, self.pos_pid_cb, "Position")
		self.ang_srv = Server(ang_PIDConfig, self.ang_pid_cb, "Angular")
		
		self.initialize_PID()

 		# sub joy to switch on or off
		self.auto = False
		self.sub_joy = rospy.Subscriber('joy', Joy, self.cb_joy, queue_size=1)

	def odom_cb(self, msg):
		self.frame_id = msg.header.frame_id
		robot_position = [msg.pose.pose.position.x, msg.pose.pose.position.y]
		quat = (msg.pose.pose.orientation.x,\
				msg.pose.pose.orientation.y,\
				msg.pose.pose.orientation.z,\
				msg.pose.pose.orientation.w)
		_, _, yaw = tf.transformations.euler_from_quaternion(quat)

		if self.goal is None: # if the robot haven't recieve any goal
			return

		#yaw = yaw + np.pi/2
		goal_distance = self.get_distance(robot_position, self.goal)
		goal_angle = self.get_goal_angle(yaw, robot_position, self.goal)
		if goal_distance < self.arrived_dis or self.emergency_stop:
			print "Goal: ", self.goal, ", robot at: ", robot_position
			print goal_distance
			rospy.loginfo("Stop in odom_cb!!!")
			self.pub_arrive.publish(Bool(True))
			print goal_distance, " ", self.emergency_stop, " "
			pos_output, ang_output = (0, 0)
                        self.goal = None
		else:
			pos_output, ang_output = self.control(goal_distance, goal_angle)
		
		cmd_msg = Twist()
		cmd_msg.linear.x = pos_output * 2
		cmd_msg.angular.z = ang_output
		if not self.emergency_stop:
			self.pub_cmd.publish(cmd_msg)
		if self.goal is not None:
			self.publish_goal(self.goal)

	def control(self, goal_distance, goal_angle):
		self.pos_control.update(goal_distance)
		self.ang_control.update(goal_angle)

		# pos_output will always be positive
		pos_output = self.pos_constrain(-self.pos_control.output/self.dis4constV)

		# -1 = -180/180 < output/180 < 180/180 = 1
		ang_output = self.ang_constrain(self.ang_control.output*3/180.)
		if abs(self.ang_control.output) > self.turn_threshold:
			if pos_output > 0.1:
				pos_output = self.slow_speed
		return pos_output, ang_output

	def goal_cb(self, p):
		print('goal_cb')
		self.final_goal = [p.pose.position.x, p.pose.position.y]
		self.goal = self.final_goal
		# New goal, publish arrive to false
		self.pub_arrive.publish(Bool(False))
		if not self.use_odom:
			robot_position = [0.0, 0.0]
			yaw = 0
			goal_distance = self.get_distance(robot_position, self.goal)
			goal_angle = self.get_goal_angle(yaw, robot_position, self.goal)
			estop = Bool()
			if goal_distance < self.arrived_dis:
				rospy.loginfo("Arrived!!!")
				self.pub_arrive.publish(Bool(True))
				print goal_distance
				pos_output , ang_output = (0, 0)
				estop.data = True
				self.pub_estop.publish(estop)
				self.goal = None
			elif self.emergency_stop:
				rospy.loginfo("JOYSTICK: Stop!!!")
				self.pub_arrive.publish(Bool(True))
				pos_output , ang_output = (0, 0)
				estop.data = True
				self.pub_estop.publish(estop)
				self.publish_goal(self.goal)
				return
			else:
				estop.data = False
				self.pub_estop.publish(estop)
				pos_output, ang_output = self.control(goal_distance, goal_angle)
			
				self.cmd = Twist()
				
				self.cmd.linear.x = pos_output * 2
				self.cmd.angular.z = ang_output
				if self.auto:
					self.pub_cmd.publish(self.cmd)
					print('Pub cmd')
			if self.goal is not None:
				self.publish_goal(self.goal)
		

	def emergency_stop_cb(self, req):
		if req.data == True:
			self.emergency_stop = True
			rospy.loginfo("Mode: Joystick")
		else:
			self.emergency_stop = False
			rospy.loginfo("Mode: Autonomous")
		res = SetBoolResponse()
		res.success = True
		res.message = "recieved"
		return res

	def cmd_constarin(self, input):
		if input > self.cmd_ctrl_max:
			return self.cmd_ctrl_max
		if input < self.cmd_ctrl_min:
			return self.cmd_ctrl_min
		return input

	def pos_constrain(self, input):
		if input > self.pos_ctrl_max:
			return self.pos_ctrl_max
		if input < self.pos_ctrl_min:
			return self.pos_ctrl_min
		return input

	def ang_constrain(self, input):
		if input > self.ang_ctrl_max:
			return self.ang_ctrl_max
		if input < self.ang_ctrl_min:
			return self.ang_ctrl_min
		return input

	def initialize_PID(self):
		self.pos_control.setSampleTime(1)
		self.ang_control.setSampleTime(1)

		self.pos_control.SetPoint = 0.0
		self.ang_control.SetPoint = 0.0

	def get_goal_angle(self, robot_yaw, robot, goal):
		robot_angle = np.degrees(robot_yaw)
		p1 = [robot[0], robot[1]]
		p2 = [robot[0], robot[1]+1.]
		p3 = goal
		angle = self.get_angle(p1, p2, p3)
		result = angle - robot_angle
		result = self.angle_range(-(result + 90.))
		return result

	def get_angle(self, p1, p2, p3):
		v0 = np.array(p2) - np.array(p1)
		v1 = np.array(p3) - np.array(p1)
		angle = np.math.atan2(np.linalg.det([v0,v1]),np.dot(v0,v1))
		return np.degrees(angle)

	def angle_range(self, angle): # limit the angle to the range of [-180, 180]
		if angle > 180:
			angle = angle - 360
			angle = self.angle_range(angle)
		elif angle < -180:
			angle = angle + 360
			angle = self.angle_range(angle)
		return angle

	def get_distance(self, p1, p2):
		return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

	def publish_goal(self, goal):
		marker = Marker()
		if self.use_odom: marker.header.frame_id = self.frame_id
		else: marker.header.frame_id = "laser_frame"
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
	def timer_cb(self, event):
		if not self.emergency_stop:
			self.pub_cmd.publish(self.cmd)

	def pos_pid_cb(self, config, level):
		print("Position: [Kp]: {Kp}   [Ki]: {Ki}   [Kd]: {Kd}\n".format(**config))
		Kp = float("{Kp}".format(**config))
		Ki = float("{Ki}".format(**config))
		Kd = float("{Kd}".format(**config))
		self.pos_control.setKp(Kp)
		self.pos_control.setKi(Ki)
		self.pos_control.setKd(Kd)
		return config

	def ang_pid_cb(self, config, level):
		print("Angular: [Kp]: {Kp}   [Ki]: {Ki}   [Kd]: {Kd}\n".format(**config))
		Kp = float("{Kp}".format(**config))
		Ki = float("{Ki}".format(**config))
		Kd = float("{Kd}".format(**config))
		self.ang_control.setKp(Kp)
		self.ang_control.setKi(Ki)
		self.ang_control.setKd(Kd)
		return config


    	def cb_joy(self, joy_msg):
        	# MODE D
        	start_button = 1
        	back_button = 3
        	# Start button
        	if joy_msg.buttons[start_button] == 1:
			self.auto = True
			rospy.loginfo('go auto')
        	elif joy_msg.buttons[back_button] == 1 :
            		self.auto = False
            		rospy.loginfo('go manual')

if __name__ == '__main__':
	rospy.init_node('PID_control')
	foo = Robot_PID()
	rospy.spin()
