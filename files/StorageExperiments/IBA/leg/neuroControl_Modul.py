#!/usr/bin/env python
""" """

__author__ = 'Annika Schmidt'

from std_msgs.msg import Float64
from gazebo_ros_muscle_interface.msg import MuscleStates
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState
from hbp_nrp_cle.brainsim import simulator as sim
from gazebo_msgs.srv import ApplyJointEffort, JointRequest
from threading import Lock
from pyNN.nest import *
import rospy
import numpy
import scipy
import pylab
import os
import pdb
from external_module_interface.external_module import ExternalModule


class NeuroControl_Module(ExternalModule):

    def __init__(self, module_name=None, steps=1):
        super(NeuroControl_Module, self).__init__(module_name, steps)

    def jointStateCallback(self, data):
        with self.joint_state_lock:
            self.joint_states = data

    def create_brain(self):
        n_refl = 100         # amount of reflex neurons
        n_raphe = 100        # amount of reaphe neurons per raphe nucleus
        n_total = n_refl+2*n_raphe

        w = 1.0             # neuron weight

        # neuron and synapse parameter
        SENSORPARAMS = {'v_rest': -70.0,
                    'tau_m': 10.0,
                    'v_thresh': -50.0,
                    'tau_refrac': 5.0}

        setup(timestep=0.1,min_delay=0.1,max_delay=4.0)
        refl_class = create(IF_cond_alpha, cellparams=SENSORPARAMS)
        self.neurons = Population(size=n_total, cellclass=refl_class, label='neurons')
        initialize(neurons)

        self.knee_refl = create(SpikeSourcePoisson)
        self.hip_refl = create(SpikeSourcePoisson)
        self.knee_raphe = create(SpikeSourcePoisson)
        self.hip_raphe = create(SpikeSourcePoisson)
        
        self.rate_refl_neurons = create(IF_curr_exp)
        self.rate_raphe_j1 = create(IF_curr_exp, tau_m=10000.0)
        self.rate_raphe_j2 = create(IF_curr_exp, tau_m=10000.0)

        connect(self.knee_refl, self.neurons, weight=15.0)
        connect(self.hip_refl, self.neurons, weight=15.0)
        connect(self.knee_raphe, self.neurons, weight=50.0)
        connect(self.hip_raphe, self.neurons, weight=50.0)

        connect(self.neurons, self.rate_refl_neurons)
        connect(self.neurons, self.rate_raphe_j1, delay=40.0)
        connect(self.neurons, self.rate_raphe_j2, delay=40.0)

    def initialize(self):
        rospy.wait_for_service("gazebo/apply_joint_effort")
        rospy.wait_for_service("gazebo/clear_joint_forces")
        self.applyEffortService = rospy.ServiceProxy("gazebo/apply_joint_effort", gazebo_msgs.srv.ApplyJointEffort)
        self.clearJointService = rospy.ServiceProxy("gazebo/clear_joint_effort", gazebo_msgs.srv.JointRequest)

        self.circuit = self.create_brain()

        self.joint_state_lock = Lock()
        self.joint_states = None
        self.joint_state_sub = rospy.Subscriber("/joint_states", JointState, self.jointStateCallback, queue_size=1)

        self.create_brain()

    def run_step(self):
        with self.joint_state_lock:
            if self.joint_states is not None and self.applyEffortService is not None:
                def clearJoints():
                    self.clearJointService.value('link1')
                    self.clearJointService.value('link3')

                ## CONSTANTS
                m_sens = 500.0   # to tune reflex signal
                m_ser  = 200.0   # to tune rephe signal
                ser_c  = 10.0    # constant serotonin
                m_f    = 0.35    # to tune muscle force
                m_ser  = 0.005   # to tune serotonin weight

                hip_rad = self.joint_states.value.position[1]
                knee_rad = -self.joint_states.value.position[3]
                angles = [hip_rad, knee_rad]

                # set rate with which poisson fire [Hz]
                #raphe neurons
                if hip_rad > 0.:
                    self.hip_raphe.rate = ser_c
                else:
                    self.hip_raphe.rate = -m_ser*hip_rad
                if knee_rad < 0.:
                    self.knee_raphe.rate = ser_c
                else:
                    self.knee_raphe.rate = m_ser*knee_rad
                #reflex neurons
                if hip_rad > 0.0:
                    self.hip_refl.rate = 0.0
                else:
                    self.hip_refl.rate = -m_sens*hip_rad

                if knee_rad < 0.0:
                    self.knee_refl.rate = 0.0
                else:
                    self.knee_refl.rate = m_sens*knee_rad


                ## apply muscles
                rate_refl = self.rate_refl_neurons.voltage
                rate_raphe_hip = self.rate_raphe_j1.voltage
                rate_raphe_knee = self.rate_raphe_j1.voltage
                fire_rates = [rate_refl, rate_raphe_hip, rate_raphe_knee]

                weight_hip = rate_raphe_hip*m_ser
                weight_knee = rate_raphe_knee*(-m_ser)
                weights = [weight_hip, weight_knee]

                force = m_f*rate_refl
                
                force_hip=weights[0]*force/norm(weights)
                force_knee=weights[1]*force/norm(weights)

                if force_hip > 1:
                    signal_hip = 1.0
                else:
                    signal_hip = force_hip

                if force_knee > 1:
                    signal_knee = 1.0
                else:
                    signal_knee = force_knee

                tau1_appl = force_hip
                tau2_appl = -force_knee

                if t > 1:
                    clearJoints()
                    self.applyEffortService.value('robot::link1', tau1_appl, rospy.Time(), rospy.Duration(-1))
                    self.applyEffortService.value('robot::link3', tau2_appl, rospy.Time(), rospy.Duration(-1))

                self.module_data = [ 0, tau1_appl, tau2_appl ]

    #def shutdown(self):
    #    pass

    #def share_module_data(self):
    #    self.module_data = [1, 2.50, -3.7]

if __name__ == "__main__":
    m = NeuroControl_Module(module_name='NeuroControl_Module', steps=1)
    rospy.spin()
