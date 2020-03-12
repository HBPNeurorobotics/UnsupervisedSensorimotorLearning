import rospy
from std_msgs.msg import Float64
from gazebo_ros_muscle_interface.msg import MuscleStates
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState
from hbp_nrp_cle.brainsim import simulator as sim
from gazebo_msgs.srv import ApplyJointEffort, JointRequest

rospy.wait_for_service("gazebo/apply_joint_effort")
rospy.wait_for_service("gazebo/clear_joint_forces")
@nrp.MapVariable("applyEffortService", initial_value=None, scope=nrp.GLOBAL)
@nrp.MapVariable("clearJointService", initial_value=None, scope=nrp.GLOBAL)

# create poisson spikes as source for the brain
# poisson neurons from elbow and shoulder to reflex neurons
@nrp.MapSpikeSource("knee_refl", nrp.brain.refl_neurons, nrp.poisson, weight=15.0)
@nrp.MapSpikeSource("hip_refl", nrp.brain.refl_neurons, nrp.poisson, weight=15.0)

@nrp.MapSpikeSink("rate_refl_neurons", nrp.brain.refl_neurons, nrp.leaky_integrator_exp)
@nrp.MapSpikeSink("rate_raphe_j1", nrp.brain.raphe_j1, nrp.leaky_integrator_exp, delay=40.0, tau_m=10000.0)
@nrp.MapSpikeSink("rate_raphe_j2", nrp.brain.raphe_j2, nrp.leaky_integrator_exp, delay=40.0, tau_m=10000.0)

# poisson neurons from elbow and shoulder to raphe nuclei
@nrp.MapSpikeSource("hip_raphe", nrp.brain.raphe_j1, nrp.poisson, weight=50.)
@nrp.MapSpikeSource("knee_raphe", nrp.brain.raphe_j2, nrp.poisson, weight=50.)

@nrp.MapRobotSubscriber("joint_states", Topic("/joint_states", sensor_msgs.msg.JointState))

@nrp.Robot2Neuron()
def neuroControl (t, knee_refl, hip_refl, knee_raphe, hip_raphe, joint_states, rate_refl_neurons, rate_raphe_j1, rate_raphe_j2, applyEffortService, clearJointService):

    import rospy
    import numpy as np
    import math
    from numpy.linalg import norm
    from gazebo_msgs.srv import ApplyJointEffort, JointRequest

    if joint_states.value.position is None:
        return
    if applyEffortService.value is None:
        applyEffortService.value = rospy.ServiceProxy("/gazebo/apply_joint_effort", ApplyJointEffort)
    if clearJointService.value is None:
        clearJointService.value = rospy.ServiceProxy("/gazebo/clear_joint_forces", JointRequest)

    def clearJoints():
        clearJointService.value('link1')
        clearJointService.value('link3')


    ## CONSTANTS
    record_time = 0
    m_sens = 500.0 #295.0 # [Hz/rad] transform weight sensor input [Hz/rad]
    m_ser = 200.0 # [Hz/rad] weight mapping on serotonin trigger in paper = 1000
    ser_constant = 10.0

    hip_rad = joint_states.value.position[1]
    knee_rad = -joint_states.value.position[3]
    angles = [hip_rad, knee_rad]

    # set rate with which poisson fire [Hz]
    #raphe neurons
    if hip_rad > 0.: #or hip_rad < -0.4:
        hip_raphe.rate = ser_constant
    else:
        hip_raphe.rate = -m_ser*hip_rad
    if knee_rad < 0.: #or knee_rad > 0.4:
        knee_raphe.rate = ser_constant
    else:
        knee_raphe.rate = m_ser*knee_rad

    #reflex neurons
    if hip_rad > 0.0:
        hip_refl.rate = 0.0
    else:
        hip_refl.rate = -m_sens*hip_rad

    if knee_rad < 0.0:
        knee_refl.rate = 0.0
    else:
        knee_refl.rate = m_sens*knee_rad

    ## apply muscles
    # constant to tune muscle force
    m_f = 0.35 # 525 [microN m Hz^(-1)]
    # constant to tune serotonin weight
    m_ser = 0.005

    rate_refl = rate_refl_neurons.voltage
    rate_raphe_hip = rate_raphe_j1.voltage
    rate_raphe_knee = rate_raphe_j2.voltage
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

    if t > 1: ##otherwise gazebo crashes
        clearJoints()
        applyEffortService.value('robot::link1', tau1_appl, rospy.Time(), rospy.Duration(-1))
        applyEffortService.value('robot::link3', tau2_appl, rospy.Time(), rospy.Duration(-1))
    # if duration < 0 apply continuously









