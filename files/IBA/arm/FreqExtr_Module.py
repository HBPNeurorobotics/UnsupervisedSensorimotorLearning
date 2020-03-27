#!/usr/bin/env python
""" """
from std_msgs.msg import Float64
from gazebo_msgs.srv import ApplyJointEffort, JointRequest
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Wrench, Vector3
from threading import Lock
import math
import numpy as np
from numpy.linalg import norm

__author__ = 'Annika Schmidt'

import rospy
from external_module_interface.external_module import ExternalModule


class FreqExtr_Module(ExternalModule):

    def __init__(self, module_name=None, steps=1):
        super(FreqExtr_Module, self).__init__(module_name, steps)

    def jointStateCallback(self, data):
        with self.joint_state_lock:
            self.joint_states = data

    def initialize(self):
        rospy.wait_for_service("gazebo/apply_joint_effort")
        rospy.wait_for_service("gazebo/clear_joint_forces")
        self.applyEffortService = rospy.ServiceProxy("gazebo/apply_joint_effort", gazebo_msgs.srv.ApplyJointEffort)
        self.clearJointService = rospy.ServiceProxy("gazebo/clear_joint_effort", gazebo_msgs.srv.JointRequest)

        self.theta1 = 0.0
        self.theta2 = 0.0

        self.joint_state_lock = Lock()
        self.joint_states = None
        self.joint_state_sub = rospy.Subscriber("/joint_states", JointState, self.jointStateCallback, queue_size=1)
        
        self.weights = [0.0, 0.0, 0.0]

    def run_step(self):
        weight_change = 3

        if len(self.synced_data.m1) is 4:
            self.weights = self.synced_data.m1[1:]

        with self.joint_state_lock:
            if self.joint_states is not None:
                def clearJoints():
                    self.clearJointService.value('HumerusBone_shoulder')
                    self.clearJointService.value('RadiusBone_ellbow')

                #################### Definitions ###################
                ## read out current positions of the joints from /joint_states
                phi1 = self.joint_states.value.position[1]
                phi2 = self.joint_states.value.position[2]
                dq1  = self.joint_states.value.velocity[1]
                dq2  = self.joint_states.value.velocity[2]
                q1 = phi1
                q2 = phi1+phi2
                
                #################### Parameters ###################
                ## set threshold, spring stiffness and damping (k and d should correspond to values in SDF file)
                threshold = 1.0 # Nm
                k         = 10  # Nm/rad
                d         = 0.7 # Nm/rad
                w_default = [-1, 1]
                
                if self.weights.value is None:
                    w = w_default
                else:
                    w = [self.weights.value.x, self.weights.value.y]

                #################### Controller ###################
                ## set weights
                weights_tau = np.array([w[0], w[1]])
                weights_theta = np.array([w[0], w[1]])
                
                ## get joint torques at beginning of step
                tau_tot_1 = k*(self.theta1.value-q1)
                tau_tot_2 = k*(self.theta2.value-q2)
                tau_tot = np.array([tau_tot_1, tau_tot_2])
                
                ## reduce dimension: set threshold and choose control value theta_z according to threshold
                if norm(w)==0:
                    tau_z = 0.0
                    weights_theta = w_default
                else:
                    tau_z = weights_tau.dot(tau_tot)/norm(weights_tau)

                if tau_z > threshold:
                    theta_z = 0.3
                elif -threshold <= tau_z and tau_z <= threshold:
                    theta_z = 0.0
                elif tau_z < -threshold:
                    theta_z = -0.3

                ## go back to 2 dimension
                self.theta1.value = weights_theta[0]*theta_z/norm(weights_theta)
                self.theta2.value = weights_theta[1]*theta_z/norm(weights_theta)

                ## calculate motor signal
                tau1 = (self.theta1.value)*(k)
                tau2 = (self.theta2.value)*(k)

                clearJoints()
                if t>1: ## otherwise gazebo crashes
                    self.applyEffortService.value('myoarm::HumerusBone_shoulder', tau1, rospy.Time(), rospy.Duration(-1))
                    self.applyEffortService.value('myoarm::RadiusBone_ellbow', tau2, rospy.Time(), rospy.Duration(-1))
                    #if duration < 0 apply continuously

                self.module_data = [1, tau1, tau2]

    #def shutdown(self):
    #    pass

    #def share_module_data(self):
    #    self.module_data = [1, 2.50, -3.7]

if __name__ == "__main__":
    m = FreqExtr_Module(module_name='FreqExtr_Module', steps=1)
    rospy.spin()
