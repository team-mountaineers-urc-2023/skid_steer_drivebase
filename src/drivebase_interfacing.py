#!/usr/bin/env python3

from time import time
from threading import Lock

import rospy
from geometry_msgs.msg import Twist

from std_msgs.msg import Float32MultiArray

### helpers ##################################################################

def clamp(value: float, lower: float, upper: float) -> float:
	return min(upper, max(value, lower))

### main #####################################################################

def main():
	DrivebaseInterfacer().loop()

class DrivebaseInterfacer:
	def __init__(self):

		rospy.init_node("drivebase_interfacer")

		### local variables ##################################################

		self.timeout = rospy.get_param("~timeout")

		self.min_linear_speed = rospy.get_param("~min_linear_speed")
		self.max_linear_speed = rospy.get_param("~max_linear_speed")
		self.min_angular_speed = rospy.get_param("~min_angular_speed")
		self.max_angular_speed = rospy.get_param("~max_angular_speed")

		self.last_tick = 0.0  # time of last received cmd_vel
		self.last_tick_lock = Lock()

		### connect to ROS ###################################################

		self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic")
		self.motor_cmd_topic = rospy.get_param("~motor_cmd_topic")
		
		self.cmd_vel_sub = rospy.Subscriber(self.cmd_vel_topic, Twist, self.cmd_vel_callback)
		self.motor_cmd_pub = rospy.Publisher(self.motor_cmd_topic, Float32MultiArray, queue_size=1)

		### end init #########################################################

	### callbacks ############################################################

	def cmd_vel_callback(self, cmd_vel: Twist):
		with self.last_tick_lock:
			self.last_tick = time()  # record time of most recent input

		linear, angular = cmd_vel.linear.x, cmd_vel.angular.z
		linear_scale = abs(self.max_linear_speed if linear >= 0 else self.min_linear_speed)
		angular_scale = abs(self.max_angular_speed if angular >= 0 else self.min_angular_speed)

		# unit scale motor "speeds"
		l_cmd = (linear / linear_scale) - (angular / angular_scale)
		r_cmd = (linear / linear_scale) + (angular / angular_scale)
		l_cmd = clamp(l_cmd, -1, 1)
		r_cmd = clamp(r_cmd, -1, 1)

		motor_cmd = Float32MultiArray()
		motor_cmd.data = [l_cmd, r_cmd]
		self.motor_cmd_pub.publish(motor_cmd)

	### loop #################################################################

	def loop(self):
		rate = rospy.Rate(5 // self.timeout)
		stop_motors = Float32MultiArray()
		stop_motors.data = [0, 0]

		while not rospy.is_shutdown():

			# tell robot to stay still if connection lost
			with self.last_tick_lock:
				timed_out = (time() - self.last_tick) > self.timeout
			if timed_out:
				self.motor_cmd_pub.publish(stop_motors)

			rate.sleep()

if __name__ == "__main__":
	main()
