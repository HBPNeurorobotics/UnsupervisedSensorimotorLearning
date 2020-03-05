#!/usr/bin/env python
""" """
import rospy
from std_msgs.msg import Float64
from gazebo_msgs.srv import ApplyJointEffort, JointRequest
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Vector3
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
import math
from gazebo_msgs.srv import ApplyJointEffort, JointRequest
from geometry_msgs.msg import Wrench, Vector3
from std_msgs.msg import Float64
import rospy
import math
import numpy as np
from numpy.linalg import norm

__author__ = 'Omer Yilmaz'

import rospy
from external_module_interface.external_module import ExternalModule


class Module1(ExternalModule):
    
    def __init__(self, module_name=None, steps=1):
        super(Module1, self).__init__(module_name, steps)

    def jointStateCallback(self, data):
        self.joint_states = data

    def initialize(self):
        self.weight1_oja = 1.0
        self.weight_pub = rospy.Publisher("/weights", Vector3, queue_size=1)

        self.joint_states = None
        self.joint_state_sub = rospy.Subscriber("/joint_states", JointState, self.jointStateCallback, queue_size=1)
    
    def run_step(self):
        weight_change = 3

        if self.joint_states is not None:

            # [0]    zero weights - to surpress jumping
            # [1]    constant weights (jump straight)
            # [2]    angle dependent (RAL paper)
            # [3]    adjusting according to Oja rule
            # [4]    adjusting according to Gradient Descent

            #################### Initialization ###################
            ## functions you want to use in the function need to be initialized again

            if np.isnan(self.weight1_gd.value) == True:
                clientLogger.info("help")
                weight1_gd.value = math.sqrt(2)
                weight2_gd.value = -math.sqrt(2)

            #################### Definitions ###################
            ## read out current positions of the joints from /joint_states
            # [0, 1, 2, 3, 4, 5] = [gear1, link1, link2, link3, world_1, world_2] (check SDF for definintion)
            phi1 = self.joint_states.value.position[1]
            phi2 = joint_states.value.position[2]

            ## definitions according to matlab script/ RAL paper, defined solely for comparison
            q1 = phi1
            q2 = phi1 + phi2

            #################### Parameters ###################
            ## set time from which recording of data (for CSV) should start
            ## also time when weights start adjusting
            if t_start.value is None:
                record_time = 0.0
            else:
                record_time = t_start.value.data

            if weight_change == 0:
                w = [0, 0]
            elif weight_change == 1:
                w = [-1, 1]
            elif weight_change == 2:
                alpha = math.pi * 1.1
                w = [math.sin(alpha), math.cos(alpha)]
            elif weight_change == 3:
                w = [weight1_oja.value, weight2_oja.value]
            elif weight_change == 4:
                w = [weight1_gd.value, weight2_gd.value]

            weights = Vector3(w[0], w[1], 0.0)
            weight_pub.publish(weights)

            #################### Oja rule ###################
            ## set parameters
            dt = 0.02  # sec --> default NRP update rate (CLE)
            if t > record_time:
                gamma = 0.5  # Hz
            else:
                gamma = 0.0  # Hz

            ## calculate rate change
            dw1_oja = gamma * (weight1_oja.value * q1 + weight2_oja.value * q2) * (
                        q1 - ((weight1_oja.value * q1 + weight2_oja.value * q2) * weight1_oja.value))
            dw2_oja = gamma * (weight1_oja.value * q1 + weight2_oja.value * q2) * (
                        q2 - ((weight1_oja.value * q1 + weight2_oja.value * q2) * weight2_oja.value))

            ## calculate new weight
            weight1_oja.value = weight1_oja.value + (dw1_oja * dt)
            weight2_oja.value = weight2_oja.value + (dw2_oja * dt)

            #################### Gradient Descent ###################
            alpha = 0.2
            q = np.array([q1, q2])
            weights_gd = np.array([weight1_gd.value, weight2_gd.value])
            # estimated joint angles
            q_est = weights_gd.dot(q) * weights_gd
            # error function
            L = 1 / 2 * (q_est - q) ** 2
            # gradient descent
            L_dw1 = q[0] * weight2_gd.value * (q_est[0] - q[0])
            L_dw2 = q[1] * weight1_gd.value * (q_est[1] - q[1])

            weight1_gd.value = weight1_gd.value - alpha * L_dw1
            weight2_gd.value = weight2_gd.value - alpha * L_dw2

            #################### Recording ###################
            # record data to CSV file --> needs to correspond to headers defined in beginning
            if t > record_time:
                recorder_control.record_entry(t, w[0], w[1], weight1_oja.value, weight2_oja.value, weight1_gd.value,
                                              weight2_gd.value)

            #################### Debugging ###################
            ## check interesting values during simulation
            clientLogger.info("Oja:", weight1_oja.value, weight2_oja.value)
            clientLogger.info("G-D:", weight1_gd.value, weight2_gd.value)
            # clientLogger.info('q____:', q)
            # clientLogger.info('q____:', q)
            # clientLogger.info('q_est:', q_est)

    #def shutdown(self):
    #    pass

    #def share_module_data(self):
    #    self.module_data = [1, 2.50, -3.7]

if __name__ == "__main__":
    m = Module1(module_name='module1', steps=1)
    rospy.spin()
