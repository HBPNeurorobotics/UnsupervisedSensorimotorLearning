##################################################################################
########### SET FIRE RATE OF POISSON NEURONS DEPENDING ON JOINT STATE ############
##################################################################################

from std_msgs.msg import Float64
from gazebo_ros_muscle_interface.msg import MuscleStates
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState
from hbp_nrp_cle.brainsim import simulator as sim

# create poisson spikes as source for the brain
# poisson neurons from elbow and shoulder to reflex neurons
@nrp.MapSpikeSource("knee_refl", nrp.brain.refl_neurons, nrp.poisson, weight=15.0)
@nrp.MapSpikeSource("hip_refl", nrp.brain.refl_neurons, nrp.poisson, weight=15.0)

# poisson neurons from elbow and shoulder to raphe nuclei
@nrp.MapSpikeSource("hip_raphe", nrp.brain.raphe_j1, nrp.poisson, weight=50.)
@nrp.MapSpikeSource("knee_raphe", nrp.brain.raphe_j2, nrp.poisson, weight=50.)

@nrp.MapRobotSubscriber("link_states", Topic("/gazebo/link_states", gazebo_msgs.msg.LinkStates))
@nrp.MapRobotSubscriber("joint_states", Topic("/joint_states", sensor_msgs.msg.JointState))
@nrp.MapCSVRecorder("recorder_states", filename="states.csv", headers=["t", "phi1", "phi2", "q1","q2","dq1","dq2", "z"])

@nrp.Robot2Neuron()

def setNeurons(t, knee_refl, hip_refl, knee_raphe, hip_raphe, joint_states, link_states, recorder_states):
    import numpy as np
    import math
    import tf.transformations as trafo

    ## CONSTANTS
    record_time = 0
    m_sens = 500.0 #295.0 # [Hz/rad] transform weight sensor input [Hz/rad]
    m_ser = 200.0 # [Hz/rad] weight mapping on serotonin trigger in paper = 1000
    ser_constant = 10.0

    body_pos = link_states.value.pose[2]
    body_height = body_pos.position.z

    phi1 = joint_states.value.position[1]
    phi2 = -joint_states.value.position[3]
    dq1 = joint_states.value.velocity[1]
    dq2 = -joint_states.value.velocity[3]

    ## definitions according to matlab script/ RAL paper, defined solely for comparison
    offset_q1 = -40*math.pi/180
    offset_q2 = 40*math.pi/180
    q1 = phi1+offset_q1
    q2 = phi2+offset_q2

    hip_rad = phi1
    knee_rad = phi2
    hip_deg = hip_rad*180/math.pi
    knee_deg = knee_rad*180/math.pi
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


    if t > record_time:
        recorder_states.record_entry(t, phi1, phi2, q1, q2, dq1, dq2, body_height)



