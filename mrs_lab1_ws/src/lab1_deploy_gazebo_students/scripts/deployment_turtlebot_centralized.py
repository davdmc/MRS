#!/usr/bin/env python
# license removed for brevity
import ast
import rospy
from std_msgs.msg import String
from lab1_deploy_gazebo_students.srv import gossip_update, gossip_updateResponse
from lab1_deploy_gazebo_students.msg import GoToGoal_goal, queue_position_plot
import sys
from math import radians, copysign, sqrt, pow, pi, atan2
import numpy as np
import tf
from matplotlib import pyplot as plt

class LaplacianGraph:
	def __init__(self, n, links, is_undirected=True):
		self.n=4 # number of agents. Indices are 0, 1, ..., n-1
		self.graph_E=np.zeros((self.n,self.n))
		#links 
		self.list_links = links # include more links as desired
		self.is_undirected=is_undirected
		self.init_E()
		self.plot_graph()
		self.obtain_weights()
		# Read params. Currently: t_local and inter_distance_x/y.
		self.read_parameters()

		# Obtain robot positions
		self.tf_listener = tf.TransformListener()
		self.x = np.zeros((n,1))
		self.y = np.zeros((n,1))
		self.pub = rospy.Publisher('topic_queue_position_plot', queue_position_plot, queue_size=10)
		self.pub_goTogoal = []
		# Gather info from every robot
		rospy.sleep(0.5)
		for i in range(self.n):
			connected = False
			while(not connected and not rospy.is_shutdown()):
				try:
					(trans, rot) = self.tf_listener.lookupTransform('tb3_{}/odom'.format(i), 'tb3_{}/base_footprint'.format(i), rospy.Time(0))
					self.x[i,0] = trans[0]
					self.y[i,0] = trans[1]
					connected = True
					#print("Robot {} located: {} {}".format(i, self.x, self.y))
				except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
					print("Looking for {} robot position".format(i))

			self.pub_goTogoal.append(rospy.Publisher('topic_GoToGoal_goal'+str(i), GoToGoal_goal, queue_size=10))
			
		
		self.update_poses(self.x, self.y)
		self.laplacian_consensus()

	def init_E(self):
		# Implemented using iterators
		for link in self.list_links:
			print("link element: (", link[0], ",", link[1], ")")
			self.graph_E[link[0], link[1]]=1
			if self.is_undirected:
				self.graph_E[link[1], link[0]]=1

	def plot_graph(self):
		# Plot the topology. Nodes are placed in a circule. Lines represent links between them.
		v_angles = np.linspace(0, 2*np.pi, self.n, endpoint=False) # Get equidistant angles
		v_x = np.cos(v_angles) # Obtain X coord of V_i
		v_y = np.sin(v_angles) # Obtain Y coord of V_i
		v_x_text = v_x * 1.1
		v_y_text = v_y * 1.1
		
		plt.figure()
		plt.title("Graph: links") 
		plt.xlabel("x axis") 
		plt.ylabel("y axis") 

		# Plot nodes
		plt.plot(v_x,v_y, "or") ,#without lines, only the dots
		for i in range(self.n):
			plt.text(v_x_text[i], v_y_text[i], str(i))

		# Plot edges
		for i in range(self.n):
			for j in range(self.n):
				# If there exist a connection, draw it
				if self.graph_E[i,j]>0:
					x_ini = v_x[i]
					y_ini = v_y[i]
					# Obtain the direction between start and end node
					dx = v_x[j] - x_ini 
					dy = v_y[j] - y_ini
					plt.arrow(x_ini, y_ini, dx, dy, head_length=0.1,length_includes_head=True, head_width=0.05)
		plt.show(block=False)

	def obtain_weights(self, alpha=0.01, verbose=False):
		# Compute degree matrix:
		 # It is important that it axis=1 for leader-follower strategy!
		self.degree_matrix = np.diag(self.graph_E.sum(axis=1))
		if verbose:
			print("Degree matrix: \n" , self.degree_matrix)
			print("----------------------------------------")

		#Compute laplacian matrix:
		self.laplacian_matrix = self.degree_matrix - self.graph_E
		if verbose:
			print("Laplacian matrix: \n" , self.laplacian_matrix)
			print("----------------------------------------")

		# Compute algebraic connecivity
		eigen_vals = np.linalg.eigvals(self.laplacian_matrix)
		self.algebraic_conn = eigen_vals[1]
		if verbose:
			print("Laplacian eigen values: ", eigen_vals)
			print("Algebraic connectivity: \n" , self.algebraic_conn)
			print("----------------------------------------")

		# Compute Perron matrix and eigenvalues
		self.weight_matrix = np.identity(self.n) - alpha * self.laplacian_matrix
		if verbose:
			print("Weight matrix: \n" , self.weight_matrix)
			print("----------------------------------------")
		eigen_vals = np.linalg.eigvals(self.weight_matrix)
		if verbose:
			print("Weight eigen values: ", eigen_vals)
			print("----------------------------------------")
		return eigen_vals # For experimental purposes

	def laplacian_consensus(self):
		self.states_x_iter = np.zeros((self.n, 1))
		self.states_y_iter = np.zeros((self.n, 1))

		# Initialization
		self.states_x_iter[:,0] = np.ravel(self.x)
		self.states_y_iter[:,0] = np.ravel(self.y)

		# CONSENSUS ITERATIONS
		k = 0
		while not rospy.is_shutdown():
			self.x = self.states_x_iter[:,k]
			self.y = self.states_y_iter[:,k]
			next_x = np.dot(self.weight_matrix,self.x)[..., np.newaxis]# compute the next states
			next_y = np.dot(self.weight_matrix,self.y)[..., np.newaxis]# compute the next states
			# store values
			self.states_x_iter = np.hstack((self.states_x_iter, next_x))
			self.states_y_iter = np.hstack((self.states_y_iter, next_y))
			# Only update poses after 100 iterations
			self.update_poses(next_x, next_y)
			rospy.sleep(self.t_local)
			k+=1

		return

	def update_poses(self, next_x, next_y):
		for i in range(self.n):
			#print("{} publish at topic_queue_position_plot".format(i))
			my_pos_plot=queue_position_plot()
			my_pos_plot.robot_id=i
			my_pos_plot.x=next_x[i,0] + self.inter_distance_x * i
			my_pos_plot.y=next_y[i,0] + self.inter_distance_y * i
			self.pub.publish(my_pos_plot)

			#print("{} publish a navigation goal at topicGoToGoal_goal".format(i)+str(i))
			next_pos = GoToGoal_goal()
			next_pos.goal_coords=[next_x[i,0] + self.inter_distance_x * i, next_y[i,0] + self.inter_distance_y * i]
			next_pos.goal_z=0.0 #currently, not used
			next_pos.speed=0.0 #currently, not used
			self.pub_goTogoal[i].publish(next_pos)
			#Up the here, endif

	def read_parameters(self):
		try:

			self.inter_distance_x = rospy.get_param("/inter_distance_x")
			self.inter_distance_y = rospy.get_param("/inter_distance_y")

			self.t_local = float(rospy.get_param("~t_local"))
			return True
		except rospy.ServiceException as e:
			print("Parameters not set: "+str(e))
			return False

if __name__ == '__main__':
	#Input arguments (robot id, x0, y0, Tlocal, neig1, neig2... neign)
	sysargv = rospy.myargv(argv=sys.argv) # to avoid problems with __name:= elements.
	num_args=len(sysargv)
	print(sysargv)
	if (num_args >= 1):
		num_robots = int(sysargv[1])
	else:
		num_robots=4

	if (num_args > 2):
		links = ast.literal_eval(sysargv[2])		
		print(links)
	else:
		links=[[0,1],[0,2],[0,3],[1,2],[1,3],[2,3]]

	if (num_args > 3):
		is_undirected = bool(sysargv[3])
	else:
		is_undirected = True
	try:
		rospy.init_node('deploy_Laplacian', anonymous=False)
		my_naive_robot=LaplacianGraph(num_robots, links, is_undirected)
		print('Finished!')

	except rospy.ROSInterruptException:
		pass
