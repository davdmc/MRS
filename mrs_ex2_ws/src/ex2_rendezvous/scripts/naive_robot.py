#!/usr/bin/env python
import random

import rospy
from ex2_rendezvous.msg import queue_position_plot
from ex2_rendezvous.srv import gossip_update, gossip_updateResponse


class NaiveRobot:

    def __init__(self):
        rospy.init_node("robot")

        # Class attributes 
        self.robot_id = 0
        self.t_local = 1/2.0
        self.position = [0,0]
        self.neighbors = []
        self.available_neightbors = []

        if self.read_parameters():
            rospy.loginfo("Succesfully launched robot {} with position {} and neightbors {}".format(
                self.robot_id, self.position, self.neighbors))

        self.gossip_update_server = rospy.Service(
            'gossip_update_'+str(self.robot_id), gossip_update, self.handle_gossip_update)
        self.position_publisher = rospy.Publisher('queue_position_plot', queue_position_plot, queue_size=10)
        position_update = queue_position_plot()
        position_update.x = self.position[0]
        position_update.y = self.position[1]
        position_update.robot_id = self.robot_id
        rospy.sleep(0.5)
        self.position_publisher.publish(position_update)
        rospy.sleep(1.5)
        self.timer = rospy.Timer(rospy.Duration(
            self.t_local), self.request_gossip_update)

    def read_parameters(self):
        try:
            self.inter_distance_x = rospy.get_param("/inter_distance_x")
            self.inter_distance_y = rospy.get_param("/inter_distance_y")

            self.robot_id = int(rospy.get_param("~robot_id"))

            self.position = rospy.get_param("~position")
            if type(self.position) is str:
                self.position = [float(x)
                                 for x in self.position.split()]

            self.neighbors = rospy.get_param("~neightbors")
            
            if type(self.neighbors) is str:
                self.neighbors = [int(x)
                                  for x in self.neighbors.split()]
                self.available_neightbors = []
                for neightbor in self.neighbors:
                    if neightbor < self.robot_id:
                        self.available_neightbors.append(neightbor)
            else:
                self.neighbors = [self.neighbors]
                if self.neighbors[0] < self.robot_id:
                    self.available_neightbors = self.neighbors
                else:
                    self.available_neightbors = None

            self.t_local = float(rospy.get_param("~t_local"))
            return True
        except rospy.ServiceException as e:
            print("Parameters not set: "+str(e))
            return False

    def handle_gossip_update(self, req):
        res = gossip_updateResponse()
        res.x_resp = (req.x_req + self.position[0]) / 2
        res.y_resp = (req.y_req + self.position[1]) / 2

        self.position[0] = res.x_resp
        self.position[1] = res.y_resp
        rospy.loginfo("Position of robot {} updated: {}".format(
                self.robot_id, self.position))
        position_update = queue_position_plot()
        position_update.x = self.position[0] + self.inter_distance_x * self.robot_id
        position_update.y = self.position[1] + self.inter_distance_y * self.robot_id
        position_update.robot_id = self.robot_id
        self.position_publisher.publish(position_update)
        return res

    def request_gossip_update(self, event):
        if self.available_neightbors is not None:
            target_id = random.choice(self.available_neightbors)
        else:
            return

        rospy.wait_for_service('gossip_update_'+str(target_id))
        try:
            service_gossip_update = rospy.ServiceProxy(
                'gossip_update_'+str(target_id), gossip_update)
            res = service_gossip_update( self.robot_id,
                self.position[0], self.position[1])
            self.position[0] = res.x_resp 
            self.position[1] = res.y_resp
            rospy.loginfo("Position of robot {} updated: {}".format(
                self.robot_id, self.position))
            position_update = queue_position_plot()
            position_update.x = self.position[0] + self.inter_distance_x * self.robot_id
            position_update.y = self.position[1] + self.inter_distance_y * self.robot_id
            position_update.robot_id = self.robot_id
            self.position_publisher.publish(position_update)
        except rospy.ServiceException as e:
            print("Service call failed: "+str(e))


if __name__ == "__main__":
    robot_node = NaiveRobot()
    rospy.spin()
