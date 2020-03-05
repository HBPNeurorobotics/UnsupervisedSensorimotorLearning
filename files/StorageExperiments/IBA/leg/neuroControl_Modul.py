#!/usr/bin/env python
""" """
from std_msgs.msg import Float64
from gazebo_ros_muscle_interface.msg import MuscleStates
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState
from hbp_nrp_cle.brainsim import simulator as sim
from gazebo_msgs.srv import ApplyJointEffort, JointRequest

__author__ = 'Annika Schmidt'

import rospy
from external_module_interface.external_module import ExternalModule


class NeuroControl_Module(ExternalModule):

    def __init__(self, module_name=None, steps=1):
        super(NeuroControl_Module, self).__init__(module_name, steps)

    def jointStateCallback(self, data):
        self.joint_states = data

    ?????????????????????????
    def register_spike_source(self, populations, spike_generator_type, **params):
        ts = self.__notify_processes_register('source', populations, spike_generator_type, params)
        device = super(DistributedPyNNCommunicationAdapter, self). \
                       register_spike_source(populations, spike_generator_type, **params)
        device.timestep = ts
        # mark the device as MPI-aware, only used by Nest-specific devices
        if isinstance(device, PyNNNestDevice):
             setattr(device, 'mpi_aware', True)
        return device

    def register_spike_sink(self, populations, spike_detector_type, **params):
        ts = self.__notify_processes_register('sink', populations, spike_detector_type, params)
        device = super(DistributedPyNNCommunicationAdapter, self). \
                register_spike_sink(populations, spike_detector_type, **params)
        device.timestep = ts


    ???????????????????????????


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

        REFL_PARAMS = {'cm': 0.025,
                    'tau_m': 10.0,
                    'v_reset': -60.0,
                    'v_thresh': -55.0}

        RAPHENUCLEI_PARAMS = {'cm': 0.025,
                    'v_reset': -60.0,
                    'v_thresh': -55.0}

        SYNAPSE_PARAMS = {'weight': w,
                      'delay': 0.0,
                      'U': 1.0,
                      'tau_rec': 1.0,
                      'tau_facil': 1.0}

        refl_class = sim.IF_cond_alpha(**SENSORPARAMS)
        neurons = sim.Population(size=n_total, cellclass=refl_class, label='neurons')
        sim.initialize(neurons)
        return neurons


    def initialize(self):
        rospy.wait_for_service("gazebo/apply_joint_effort")
        rospy.wait_for_service("gazebo/clear_joint_forces")
        self.applyEffortService=None
        self.clearJointService=None

        self.circuit = self.create_brain()

        self.weight1_oja = 1.0
        self.weight_pub = rospy.Publisher("/weights", Vector3, queue_size=1)

        self.joint_states = None
        self.joint_state_sub = rospy.Subscriber("/joint_states", JointState, self.jointStateCallback, queue_size=1)
        self.knee_refl  = self.register_spike_source
        self.hip_refl   = None
        self.knee_raphe = None
        self.hip_raphe  = None
        @nrp.MapSpikeSource("knee_refl", nrp.brain.refl_neurons, nrp.poisson, weight=15.0)
        @nrp.MapSpikeSource("hip_refl", nrp.brain.refl_neurons, nrp.poisson, weight=15.0)
        @nrp.MapSpikeSource("hip_raphe", nrp.brain.raphe_j1, nrp.poisson, weight=50.)
        @nrp.MapSpikeSource("knee_raphe", nrp.brain.raphe_j2, nrp.poisson, weight=50.)

        self.rate_refl_neurons  = None
        self.rate_raphe_j1      = None
        self.rate_raphe_j2      = None
        @nrp.MapSpikeSink("rate_refl_neurons", nrp.brain.refl_neurons, nrp.leaky_integrator_exp)
        @nrp.MapSpikeSink("rate_raphe_j1", nrp.brain.raphe_j1, nrp.leaky_integrator_exp, delay=40.0, tau_m=10000.0)
        @nrp.MapSpikeSink("rate_raphe_j2", nrp.brain.raphe_j2, nrp.leaky_integrator_exp, delay=40.0, tau_m=10000.0)


    def run_step(self):
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

    #def shutdown(self):
    #    pass

    #def share_module_data(self):
    #    self.module_data = [1, 2.50, -3.7]

if __name__ == "__main__":
    m = NeuroControl_Module(module_name='NeuroControl_Module', steps=1)
    rospy.spin()
