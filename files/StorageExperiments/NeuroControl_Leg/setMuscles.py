##################################################################################
#################### ACTIVATE MUSCLES DEPENDING ON SPIKES ########################
##################################################################################

# import needed functions
import rospy
from std_msgs.msg import Float64
from gazebo_ros_muscle_interface.msg import MuscleStates
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import ApplyJointEffort, JointRequest

rospy.wait_for_service("gazebo/apply_joint_effort")
rospy.wait_for_service("gazebo/clear_joint_forces")
@nrp.MapVariable("applyEffortService", initial_value=None, scope=nrp.GLOBAL)
@nrp.MapVariable("clearJointService", initial_value=None, scope=nrp.GLOBAL)

@nrp.MapVariable("phase", initial_value='ground', scope=nrp.GLOBAL)

@nrp.MapSpikeSink("rate_refl_neurons", nrp.brain.refl_neurons, nrp.leaky_integrator_exp)
@nrp.MapSpikeSink("rate_raphe_j1", nrp.brain.raphe_j1, nrp.leaky_integrator_exp, delay=40.0, tau_m=10000.0)
@nrp.MapSpikeSink("rate_raphe_j2", nrp.brain.raphe_j2, nrp.leaky_integrator_exp, delay=40.0, tau_m=10000.0)

@nrp.MapCSVRecorder("recorder_firerates", filename="firerates.csv", headers=["t", "reflex_pop", "raphe_hip", "raphe_knee"])

@nrp.MapCSVRecorder("recorder_actuator", filename="actuator.csv", headers=["t","tau1","tau2", "w1", "w2", "force"])

@nrp.Robot2Neuron()

def setMuscles(t, rate_refl_neurons, rate_raphe_j1, rate_raphe_j2, recorder_firerates, applyEffortService, clearJointService, phase, recorder_actuator):

    import rospy
    from numpy.linalg import norm
    from gazebo_msgs.srv import ApplyJointEffort, JointRequest

    if applyEffortService.value is None:
        applyEffortService.value = rospy.ServiceProxy("/gazebo/apply_joint_effort", ApplyJointEffort)

    if clearJointService.value is None:
        clearJointService.value = rospy.ServiceProxy("/gazebo/clear_joint_forces", JointRequest)

    # joint forces applied cumulatively, therefore need clearing
    # clear joints helper function
    def clearJoints():
        clearJointService.value('link1')
        clearJointService.value('link3')

    # constants
    record_time = 0
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

    if t > record_time:
        recorder_firerates.record_entry(t, rate_refl, rate_raphe_hip, rate_raphe_knee)
        recorder_actuator.record_entry(t, signal_hip, signal_knee, weight_hip, weight_knee, force)

    clientLogger.info('n weight:', (weight_hip, weight_knee))

