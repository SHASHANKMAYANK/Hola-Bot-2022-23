#!/usr/bin/env python3
import rospy
# publishing to /cmd_vel with msg type: Twist
from geometry_msgs.msg import Twist
# subscribing to /odom with msg type: Odometry
from nav_msgs.msg import Odometry
# for finding sin() cos() 
import math
# Odometry is given as a quaternion, but for the controller we'll need to find the orientaion theta by converting to euler angle
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseArray
import math
global i
i=0
hola_x = 0
hola_y = 0
hola_theta = 0
x_d = 0
y_d = 0
theta_d = 0
Kp_x = 0
Kp_y = 0
Kp_theta = 0
x_goals = [1, -1, -1, 1, 0]
y_goals = [1, 1, -1, -1, 0]
theta_goals = [(math.pi) / 4, 3*(math.pi) / 4, -3*math.pi / 4, -math.pi / 4, 0]
def task1_goals_Cb(msg):
	global x_goals, y_goals, theta_goals

	x_goals.clear()
	y_goals.clear()
	theta_goals.clear()

	for waypoint_pose in msg.poses:
		x_goals.append(waypoint_pose.position.x)
		y_goals.append(waypoint_pose.position.y)

		orientation_q = waypoint_pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		theta_goal = euler_from_quaternion (orientation_list)[2]
		theta_goals.append(theta_goal)

def odometryCb(msg):

	global hola_x, hola_y, hola_theta
	hola_x = msg.pose.pose.position.x
	hola_y = msg.pose.pose.position.y
	orientation_q = msg.pose.pose.orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(roll, pitch, yaw) = euler_from_quaternion(orientation_list)
	hola_theta = yaw
	# Write your code to take the msg and update the three variables
	
def waypoint_navigation():
	rospy.loginfo("Shashank")
	global i
	if(y_d == y_goals[len(x_goals)-1] and x_d == x_goals[len(x_goals)-1] and theta_d == theta_goals[len(x_goals)-1]):
		pass
	elif(y_d == y_goals[i] and x_d == x_goals[i] and theta_d == theta_goals[i]):
		pass
	elif (y_d == y_goals[i+1] and x_d == x_goals[i+1] and theta_d == theta_goals[i+1]):
		i=i+1

def main():
	global x_d, y_d, theta_d
	rospy.init_node('controller')
	# Initialze Node
	# We'll leave this for you to figure out the syntax for 
	# initialising node named "controller"
	rospy.Subscriber('/odom', Odometry, odometryCb)
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	# declare that the node subscribes to task1_goals along with the other declarations of publishing and subscribing
	rospy.Subscriber('task1_goals', PoseArray, task1_goals_Cb)
	# Initialze Publisher and Subscriber
	# We'll leave this for you to figure out the syntax for
	# initialising publisher and subscriber of cmd_vel and odom respectively

	# Declare a Twist message
	vel = Twist()
	# Initialise the required variables to 0
	# <This is explained below>


	Kp_x = 1.5
	Kp_y = 1.5
	Kp_theta = 1.5
	x_d = x_goals[0]
	y_d = y_goals[0]
	theta_d = theta_goals[0]
	# For maintaining control loop rate.
	rate = rospy.Rate(100)

	# Initialise variables that may be needed for the control loop
	# For ex: x_d, y_d, theta_d (in **meters** and **radians**) for defining desired goal-pose.
	# and also Kp values for the P Controller

	#if (x_goals[1] - 0.1 <= hola_x <= x_goals[1] + 0.1 and y_goals[1] - 0.1 <= hola_y <= y_goals[1] + 0.1 and
			#theta_goals[1] - 0.5 <= hola_theta <= theta_goals[1] + 0.5):
		#rospy.loginfo(hola_x)
	#
	# 
	# Control Loop goes here
	#
	#
	while not rospy.is_shutdown():

				if  (y_d==y_goals[i] and x_d==x_goals[i] and theta_d==theta_goals[i] and y_goals[i] - 0.005 < hola_y < y_goals[i] + 0.005 and x_goals[i] - 0.005 < hola_x < x_goals[i] + 0.005):
						rospy.loginfo(i)
						x_d = x_goals[i+1]
						y_d = y_goals[i+1]
						theta_d = theta_goals[i+1]
						waypoint_navigation()
				else:
						pass
					
				# Find error (in x, y and theta) in global frame
				# the /odom topic is giving pose of the robot in global frame
				# the desired pose is declared above and defined by you in global frame
				# therefore calculate error in global frame

				x_error = hola_x - x_d
				y_error = hola_y - y_d
				theta_error = hola_theta - theta_d
				# (Calculate error in body frame)
				# But for Controller outputs robot velocity in robot_body frame,
				# i.e. velocity are define is in x, y of the robot frame,
				# Notice: the direction of z axis says the same in global and body frame
				# therefore the errors will have have to be calculated in body frame.
				#
				# This is probably the crux of Task 1, figure this out and rest should be fine.

				# Finally implement a P controller
				# to react to the error with velocities in x, y and theta.
				vel_x = Kp_x * x_error
				vel_y = Kp_y * y_error
				vel_z = Kp_theta * theta_error
				# Safety Check
				# make sure the velocities are within a range.
				# for now since we are in a simulator and we are not dealing with actual physical limits on the system
				# we may get away with skipping this step. But it will be very necessary in the long run.

				vel.linear.x = vel_x
				vel.linear.y = vel_y
				vel.angular.z = vel_z

				pub.publish(vel)

				rate.sleep()


				







if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass	
	
	

	
	


