#!/usr/bin/env python
# license removed for brevity
import rospy
import actionlib
from std_msgs.msg import String
from lab1_deploy_gazebo_students.srv import gossip_update, gossip_updateResponse
from lab1_deploy_gazebo_students.msg import GoToGoal_goal, queue_position_plot
import sys
from math import radians, copysign, sqrt, pow, pi, atan2
import numpy as np
import tf
import random

class Robot():
	def __init__(self, robot_id, leader=False):
    	# Class attributes 
		self.robot_id = robot_id
		self.leader = leader
		self.t_local = 1/2.0
		self.x = 0.0
		self.y = 0.0
		self.neighbors = []
		self.available_neightbors = []
		self.inter_distance_x = 1
		self.inter_distance_y = 1

		# Obtain robot position
		self.connected = False
		self.tf_listener = tf.TransformListener()
		while(not self.connected and not rospy.is_shutdown()):
			try:
				(trans, rot) = self.tf_listener.lookupTransform('tb3_{}/odom'.format(self.robot_id), 'tb3_{}/base_footprint'.format(self.robot_id), rospy.Time(0))
				self.x = trans[0]
				self.y = trans[1]
				self.connected = True
				print("Robot {} located: {} {}".format(self.robot_id, self.x, self.y))
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				print("Looking for {} robot position".format(self.robot_id))		
			rospy.sleep(0.1)

		self.pub = rospy.Publisher('topic_queue_position_plot', queue_position_plot, queue_size=10)
		self.pub_goTogoal = rospy.Publisher('topic_GoToGoal_goal'+str(self.robot_id), GoToGoal_goal, queue_size=10)
		self.ser = rospy.Service('gossip_update'+str(self.robot_id), gossip_update, self.handle_gossip_update)
		
		# Read params. Currently: neightbors and t_local. robot_id is arg and x,y are obtained from tf
		self.read_parameters()
		print("Initialized robot {}\n - Position: {} {}\n - Neightbours {} (requestable: {})\n - Local rate: {}"
			.format(self.robot_id, self.x, self.y, self.neighbors, self.available_neightbors, self.t_local))
		
		if not self.leader:
			self.timer = rospy.Timer(rospy.Duration(self.t_local), self.request_gossip_update)

		rospy.sleep(0.5)
		self.update_pose()
		rospy.spin()
		
	def request_gossip_update(self, event):
		print("Callback triggered {} times: {}".format(self.robot_id, self.counter))

		if self.available_neightbors is not None and self.available_neightbors:
			target_id = random.choice(self.available_neightbors)
		else:
			return
		
		rospy.wait_for_service('gossip_update'+str(target_id)) # ask for service gossip_update
		try:
			print("{} request a service to {}".format(self.robot_id, target_id))
			service_gossip_update = rospy.ServiceProxy('gossip_update'+str(target_id), gossip_update)
			resp1=service_gossip_update(self.robot_id, self.x, self.y)

			self.x=resp1.avg_x
			self.y=resp1.avg_y
			self.update_pose()

		except rospy.ServiceException as e:
			print("Service call failed: %s"%e)

	def handle_gossip_update(self, req):
		# Gossip and use of bx_ij, by_ij for deploying on a line
		print("I am robot "+str(self.robot_id)+" and I received a gossip_update request: "+str(req.x)+","+str(req.y))
		myResponse=gossip_updateResponse()
		myResponse.avg_x=(req.x + self.x) / 2
		myResponse.avg_y=(req.y + self.y) / 2
		
		self.x = myResponse.avg_x
		self.y = myResponse.avg_y
		self.update_pose()
				
		return myResponse

	def update_pose(self):
		print("{} publish at topic_queue_position_plot".format(self.robot_id))
		my_pos_plot=queue_position_plot()
		my_pos_plot.robot_id=self.robot_id
		my_pos_plot.x=self.x + self.inter_distance_x * self.robot_id
		my_pos_plot.y=self.y + self.inter_distance_y * self.robot_id
		self.pub.publish(my_pos_plot)

		print("{} publish a navigation goal at topicGoToGoal_goal".format(self.robot_id)+str(self.robot_id))
		next_pos = GoToGoal_goal()
		next_pos.goal_coords=[self.x  + self.inter_distance_x * self.robot_id, self.y + self.inter_distance_y * self.robot_id]
		next_pos.goal_z=0.0 #currently, not used
		next_pos.speed=0.0 #currently, not used
		self.pub_goTogoal.publish(next_pos)
		#Up the here, endif

	def read_parameters(self):
		try:
			#self.robot_id = int(rospy.get_param("~robot_id"))

			# self.position = rospy.get_param("~position")
			# if type(self.position) is str:
			# 	self.position = [float(x)
			# 						for x in self.position.split()]
			
			# self.x = self.position[0]
			# self.y = self.position[1]

			self.inter_distance_x = rospy.get_param("/inter_distance_x")
			self.inter_distance_y = rospy.get_param("/inter_distance_y")

			self.neighbors = rospy.get_param("~neightbors")
			
			if type(self.neighbors) is str:
				self.neighbors = [int(x)
									for x in self.neighbors.split()]
				self.available_neightbors = []
				for neightbor in self.neighbors:
					if neightbor > self.robot_id:
						self.available_neightbors.append(neightbor)
			else:
				self.neighbors = [self.neighbors]
				if self.neighbors[0] > self.robot_id:
					self.available_neightbors = self.neighbors
				else:
					self.available_neightbors = None

			self.t_local = float(rospy.get_param("~t_local"))
			return True
		except rospy.ServiceException as e:
			print("Parameters not set: "+str(e))
			return False

if __name__ == '__main__':
	#Input arguments (robot id, x0, y0, Tlocal, neig1, neig2... neign)
	sysargv = rospy.myargv(argv=sys.argv) # to avoid problems with __name:= elements.
	num_args=len(sysargv)
	if (num_args >= 1):
		robot_id = int(sysargv[1])
	else:
		robot_id=0
	
	if (num_args > 2 and sysargv[2] == '--leader'):
		leader = True
	else:
		leader = False
	try:
		rospy.init_node('deploy_'+str(robot_id), anonymous=False)
		my_naive_robot=Robot(robot_id, leader)
		print('Finished!')

	except rospy.ROSInterruptException:
		pass
