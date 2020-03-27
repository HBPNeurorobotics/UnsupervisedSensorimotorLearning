#!/usr/bin/env python
""" """
from std_msgs.msg import Float64
from std_msgs.msg import Int64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3
from threading import Lock
import math
import numpy as np
from numpy.linalg import norm

__author__ = 'Annika Schmidt'

import rospy
from external_module_interface.external_module import ExternalModule


class setWeights_Module(ExternalModule):

    def __init__(self, module_name=None, steps=1):
        super(setWeights_Module, self).__init__(module_name, steps)

    def jointStateCallback(self, data):
        with self.joint_state_lock:
            self.joint_states = data

    def setWeightCallback(self, data):
        self.set_weights = data

    def initialize(self):
        self.weight1_oja =1.0
        self.weight2_oja =-1.5
        self.weight1_gd  =math.sqrt(2)
        self.weight2_gd =-math.sqrt(2)

        self.joint_state_lock = Lock()
        self.joint_states = None
        self.joint_state_sub = rospy.Subscriber("/joint_states", JointState, self.jointStateCallback, queue_size=1)
        self.set_weights = None
        self.set_weights_sub = rospy.Subscriber("/set_weight", Int64, self.setWeightCallback, queue_size=1)

        self.weight_pub = rospy.Publisher("/weights", Vector3, queue_size=1)

    def run_step(self):
        with self.joint_state_lock:
            if self.joint_states is not None:

                if self.set_weight.value == None:
                    weight_change = 2
                else:
                    weight_change = self.set_weight.value.data

                if np.isnan(self.weight1_gd.value) == True:
                    self.weight1_gd.value = math.sqrt(2)
                    self.weight2_gd.value = -math.sqrt(2)

                ## read out current positions of the joints from /joint_states
                phi1 = self.joint_states.value.position[1]
                phi2 = self.joint_states.value.position[2]
                q1 = phi1
                q2 = phi1+phi2

                #################### Parameters ###################
                ## set time from which recording of data (for CSV) should start

                if weight_change == 0:
                    w = [0, 0]
                elif weight_change == 1:
                    w = [-1, 1]
                elif weight_change == 2:
                    w =[self.weight1_oja.value, self.weight2_oja.value]
                elif weight_change == 3:
                    w =[self.weight1_gd.value,  self.weight2_gd.value]

                weights = Vector3(w[0], w[1], 0.0)
                self.weight_pub.send_message(weights)
                self.module_data = [0, w[0], w[1], 0.0]


                #################### Oja rule ###################
                ## set parameters
                dt = 0.02 #sec --> default NRP update rate (CLE)
                gamma = 0.5 #Hz

                ## calculate rate change
                dw1_oja = gamma*(self.weight1_oja.value*q1+self.weight2_oja.value*q2)*(q1-((self.weight1_oja.value*q1+self.weight2_oja.value*q2)*self.weight1_oja.value))
                dw2_oja = gamma*(self.weight1_oja.value*q1+self.weight2_oja.value*q2)*(q2-((self.weight1_oja.value*q1+self.weight2_oja.value*q2)*self.weight2_oja.value))
                self.weight1_oja.value = self.weight1_oja.value+(dw1_oja*dt)
                self.weight2_oja.value = self.weight2_oja.value+(dw2_oja*dt)

                #################### Gradient Descent ###################
                alpha = 0.2
                q = np.array([q1,q2])
                weights_gd = np.array([self.weight1_gd.value, self.weight2_gd.value])
                # estimated joint angles
                q_est = weights_gd.dot(q)*weights_gd
                # error function
                L = 1/2*(q_est-q)**2
                # gradient descent
                L_dw1 = q[0]*self.weight2_gd.value*(q_est[0]-q[0])
                L_dw2 = q[1]*self.weight1_gd.value*(q_est[1]-q[1])

                self.weight1_gd.value = self.weight1_gd.value - alpha*L_dw1
                self.weight2_gd.value = self.weight2_gd.value - alpha*L_dw2

    #def shutdown(self):
    #    pass

    #def share_module_data(self):
    #    self.module_data = [1, 2.50, -3.7]

if __name__ == "__main__":
    m = setWeights_Module(module_name='setWeights_Module', steps=1)
    rospy.spin()
