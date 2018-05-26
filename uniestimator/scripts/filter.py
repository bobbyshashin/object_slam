#!/usr/bin/python
import roslib
import rospy
import tf

import numpy as np
from numpy.linalg import inv
import math

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

dt = 0.005
# x = [x, y, theta, v]
x_real = np.matrix(np.zeros((4,1)))
P_real = np.diag([1.0, 1.0, 1.0, 1.0])

Q = np.diag([0.5, 0.5, 0.5])**2
R = np.diag([1.0, 1.0, 1.0, 1.0])

counter = 0
imu_bias = 0
first = True

def motion_model(x, u):
	# x = [x, y, theta, v]
	# u = [omega, a]
	theta = math.radians(x[2, 0])

	F = np.matrix([[1.0, 0, 0, dt*math.cos(theta)],
								 [0, 1.0, 0, dt*math.sin(theta)],
								 [0, 0, 1.0, 0],
								 [0, 0, 0, 1.0]])

	B = np.matrix([[0, 0.5*dt*dt*math.cos(theta)],
								 [0, 0.5*dt*dt*math.sin(theta)],
								 [dt, 0],
								 [0, dt]])

	x = F * x + B * u

	return x

def jacobian(x, u):

	d = x[3, 0] * dt + 0.5 * u[1, 0] * dt * dt
	theta = math.radians(x[2, 0])

	G = np.matrix([[1.0, 0, -d*math.sin(theta), dt*math.cos(theta)],
								 [0, 1.0, d*math.cos(theta), dt*math.sin(theta)],
								 [0, 0, 1.0, 0],
								 [0, 0, 0, 1.0]])
	return G



def prediction(x, P, u):

	#u = np.matrix([[2.0], [3.0]])
	x_pred = motion_model(x, u)

	P_pred = jacobian(x, u) * P * jacobian(x, u).T + R

	return x_pred, P_pred


def measurement(x, P, z):
	H = np.matrix([[1.0, 0, 0, 0],
								 [0, 1.0, 0, 0],
								 [0, 0, 1.0, 0],
								 [0, 0, 0, 0]])
	C = np.matrix([[1.0, 0, 0, 0],
								 [0, 1.0, 0, 0],
								 [0, 0, 1.0, 0]])

	K = P * C.T * inv(C * P * C.T + Q)

	x_est = x + K * (z - C * x)

	P_est = (np.eye(len(x_est)) - K * C) * P

	return x_est, P_est



def odomCallback(msg):
	global counter
	counter = counter + 1
	if counter%100 != 0:
		return
	counter = 0
	x_pos = msg.pose.pose.position.x
	y_pos = msg.pose.pose.position.y

	quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)

	theta = euler[2]

	z = np.matrix([[x_pos], [y_pos], [theta]])

	global x_real, P_real
	x_real, P_real = measurement(x_real, P_real, z)

	print x_real
	publishOdom()

def imuCallback(msg):
	#t = msg.header.stamp.
	#global dt
	#dt = msg.header.stamp.secs - t_last
	global first, imu_bias
	if first == True:
		imu_bias = msg.linear_acceleration.z
		first = False
		return


	omega = msg.angular_velocity.y
	accel = msg.linear_acceleration.z - imu_bias
	#print accel

	u = np.matrix([[omega], [accel]])

	global x_real, P_real
	x_real, P_real = prediction(x_real, P_real, u)

	print x_real
	publishOdom()

def publishOdom():
  odom = Odometry()
  odom.header.stamp = rospy.Time.now()
  odom.header.frame_id = "odom_ekf"
  odom.pose.pose.position.x = x_real[0]
  odom.pose.pose.position.y = x_real[1]

  quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, x_real[2])
  odom.pose.pose.orientation.x = quaternion[0]
  odom.pose.pose.orientation.y = quaternion[1]
  odom.pose.pose.orientation.z = quaternion[2]
  odom.pose.pose.orientation.w = quaternion[3]

  ekf_odom_pub.publish(odom)


if __name__ == "__main__":
	rospy.init_node('ekf_estimator', anonymous=True)
	ekf_odom_pub = rospy.Publisher('odom_ekf', Odometry, queue_size=1)
	rospy.Subscriber('/encoder', Odometry, odomCallback)
	rospy.Subscriber('/imu/data', Imu, imuCallback)

	rospy.spin()


# class EKF:
# 	def __init__(self):
		
