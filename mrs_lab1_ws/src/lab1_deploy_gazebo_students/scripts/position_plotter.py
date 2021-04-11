#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
#from lab1_deploy_gazebo_students.srv import *
from lab1_deploy_gazebo_students.msg import *
import sys
from math import radians, copysign, sqrt, pow, pi, atan2
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use("TkAgg")
import tf
from matplotlib.animation import FuncAnimation

class Plotter():
	def __init__(self, n_robots):
    	# Class attributes 
		self.n_robots = n_robots
        # Positions of all the robots. To make things easier, we assume that the robot ids are consecutive 0..n-1 
		# and that here we index them in the array with 0..n-1 respectively.
		self.v_robot_x = np.zeros([n_robots])
		self.v_robot_y = np.zeros([n_robots])

		self.fig = plt.figure()

		self.gt_pos_robot_x = {}
		self.gt_pos_robot_y = {}
		for n in range(n_robots):
			self.gt_pos_robot_x[n] = []
			self.gt_pos_robot_y[n] = []
		
		
		self.tf_listener = tf.TransformListener()
		self.sub = rospy.Subscriber('topic_queue_position_plot', queue_position_plot, self.callback_position_received)
		rospy.sleep(0.1)# give some time to receive at least a set of robot positions.
		self.obtain_pos = rospy.Timer(rospy.Duration(0.1), self.query_positions_timer)
		ani = FuncAnimation(self.fig, self.act_drawing)
		plt.show(block=True)
		
	def act_drawing(self, event):
		plt.clf()
		v_x_text = self.v_robot_x * 1.01
		v_y_text = self.v_robot_y * 1.01
		for i in range(0,n_robots):
			plt.plot(self.v_robot_x[i],self.v_robot_y[i],'bo')
			plt.text(v_x_text[i], v_y_text[i], str(i))
			plt.plot(self.gt_pos_robot_x[i], self.gt_pos_robot_y[i], 'r*')
		plt.draw()
		plt.show(block=False)

	def callback_position_received(self, new_position):
		#print(rospy.get_caller_id() + "I heard:")
		#print('robot_id: ', new_position.robot_id)
		#print('x: ', new_position.x)
		#print('y: ', new_position.y)
		#self.v_robot_x[new_position.robot_id-1]=new_position.x
		#self.v_robot_y[new_position.robot_id-1]=new_position.y
		self.v_robot_x[new_position.robot_id]=new_position.x
		self.v_robot_y[new_position.robot_id]=new_position.y
		#print('Positions printer x : ', self.v_robot_x, 'Positions y: ', self.v_robot_y)

	def query_positions_timer(self, event):
		for n in range(self.n_robots):
			try:
				(trans, rot) = self.tf_listener.lookupTransform('tb3_{}/odom'.format(n), 'tb3_{}/base_footprint'.format(n), rospy.Time(0))
				self.gt_pos_robot_x[n].append(trans[0])
				self.gt_pos_robot_y[n].append(trans[1])
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				print("Looking for {} robot position".format(n))

if __name__ == '__main__':
	#Input arguments (robot id, x0, y0, Tlocal, neig1, neig2... neign)
	sysargv = rospy.myargv(argv=sys.argv) # to avoid problems with __name:= elements.
	num_args=len(sysargv)
	if (num_args >= 1):
		n_robots = int(sysargv[1])
	else:
		n_robots = 1
	try:
		rospy.init_node('position_plotter', anonymous=False)
		my_plotter=Plotter(n_robots)
	except rospy.ROSInterruptException:
		pass
