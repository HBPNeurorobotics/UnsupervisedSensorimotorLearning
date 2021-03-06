#################### Initialization ###################
##set up environment of experiment by importing packages and suscribing to ros-topics
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

rospy.wait_for_service("gazebo/apply_joint_effort")
rospy.wait_for_service("gazebo/clear_joint_forces")

@nrp.MapVariable("applyEffortService", initial_value=None, scope=nrp.GLOBAL)
@nrp.MapVariable("clearJointService", initial_value=None, scope=nrp.GLOBAL)
@nrp.MapVariable("theta1", initial_value=0.0, scope=nrp.GLOBAL)
@nrp.MapVariable("theta2", initial_value=0.0, scope=nrp.GLOBAL)

@nrp.MapRobotSubscriber('link_states', Topic('/gazebo/link_states', LinkStates))
@nrp.MapRobotSubscriber("joint_states", Topic("/joint_states", JointState))
@nrp.MapRobotSubscriber("collision", Topic('/robot_bumper', gazebo_msgs.msg.ContactsState))
@nrp.MapRobotSubscriber('t_start', Topic('/record_time', Float64))
@nrp.MapRobotSubscriber('weights', Topic('/weights', Vector3))

@nrp.MapCSVRecorder("recorder_control", filename="Rflx_controller.csv", headers=["t", "threshold","q1", "q2", "dq1", "dq2", "theta1", "theta2","tau1","tau2", "z", "tau_z", "theta_z", "phi1", "phi2", "tau_tot1", "tau_tot2", "ground", "w1", "w2"])

## can be used to create new topics
#@nrp.MapRobotPublisher("hip", Topic("/angles/Reflex/q1", Float64))
#@nrp.MapRobotPublisher("knee", Topic("/angles/Reflex/q2", Float64))

#################### Function ###################
## This is where the function with its input starts
@nrp.Robot2Neuron()
def Reflex_Controller (t, applyEffortService, clearJointService, link_states, collision, recorder_control, joint_states, theta1, theta2, t_start, weights):

    #################### Initialization ###################
    ## functions you want to use in the function need to be initialized again
    from gazebo_msgs.srv import ApplyJointEffort, JointRequest
    from geometry_msgs.msg import Wrench, Vector3
    from std_msgs.msg import Float64
    import rospy
    import math
    import numpy as np
    from numpy.linalg import norm
    import time

    if link_states.value is None:
        return
    if joint_states.value.position is None:
        return
    if applyEffortService.value is None:
        applyEffortService.value = rospy.ServiceProxy("/gazebo/apply_joint_effort", ApplyJointEffort)
    if clearJointService.value is None:
        clearJointService.value = rospy.ServiceProxy("/gazebo/clear_joint_forces", JointRequest)

    # joint forces applied cumulatively, therefore need clearing (helper function)
    def clearJoints():
        clearJointService.value('link1')
        clearJointService.value('link3')

    #################### Definitions ###################
    ## read out current positions of the joints from /joint_states
    # [0, 1, 2, 3, 4, 5] = [gear1, link1, link2, link3, world_1, world_2] (check SDF for definintion)
    phi1 = joint_states.value.position[1]
    phi3 = -joint_states.value.position[3]
    dq1 = joint_states.value.velocity[1]
    dq3 = -joint_states.value.velocity[3]

    ## definitions according to matlab script/ RAL paper, defined solely for comparison
    offset_q1 = -40*math.pi/180
    offset_q2 = 40*math.pi/180
    q1 = phi1+offset_q1
    q3 = phi3+offset_q2

    ## read out position of base_link (body) to plot the jumping height (either throguh odom topic or link_state)
    body_pos = link_states.value.pose[2]
    body_height = body_pos.position.z

    ## show if model is in contact with ground --> help to debug
    contact = collision.value.states
    if contact == []:
        ground = 0
    else:
        ground = 1

    #################### Parameters ###################
    ## set time from which recording of data (for CSV) should start
    ## also time when weights start adjusting
    if t_start.value is None:
        record_time = 0.0
    else:
        record_time = t_start.value.data

    if weights.value is None:
        w = [-1, 1]
    else:
        w = [weights.value.x, weights.value.y]

    ## set threshold, spring stiffness and damping (k and d should correspond to values in SDF file)
    threshold = 0.3
    k = 1.46        # Nm/rad
    d = 0.0219      # Nm/rad

    #################### Controller ###################
    ## set weights
    weights_tau = np.array([w[0], w[1]])
    weights_theta = np.array([w[0], w[1]])

    ## get joint torques at beginning of step
    tau_tot_1 = k*(theta1.value-phi1)
    tau_tot_2 = k*(theta2.value-phi3)
    tau_tot = np.array([tau_tot_1, tau_tot_2])

    ## reduce dimension: set threshold and choose control value theta_z according to threshold
    if norm(w)==0:
        tau_z = 0.0
        weights_theta = [-1, 1]
    else:
        tau_z = weights_tau.dot(tau_tot)/norm(weights_tau)

    # changing the value of theta_z determines how much torque is applied to the joints
    if tau_z > threshold:
        theta_z = 0.45
    elif -threshold <= tau_z and tau_z <= threshold:
        theta_z = 0.0
    elif tau_z < -threshold:
        theta_z = -0.45

    ## go back to 2 dimension
    theta1.value = weights_theta[0]*theta_z/norm(weights_theta)
    theta2.value = weights_theta[1]*theta_z/norm(weights_theta)

    ## calculate motor signal --> acting on top of dynamic forces  defined in SDF (spring, damping, ...)
    tau1 = (theta1.value)*(k) #+0.14)
    tau2 = (theta2.value)*(k)

    ## apply to model according to needed direction
    tau1_appl = tau1
    tau2_appl = -tau2

    clearJoints()
    if t>1: ## otherwise gazebo crashes
        applyEffortService.value('robot::link1', tau1_appl, rospy.Time(), rospy.Duration(-1))
        applyEffortService.value('robot::link3', tau2_appl, rospy.Time(), rospy.Duration(-1))
        #if duration < 0 apply continuously

    '''
    #################### Publish ###################
    ## if necessary publish data from here to ros (e.g. to use in different function)
    hip.send_message(q1)
    knee.send_message(q2)
    '''

    #################### Recording ###################
    # record data to CSV file --> needs to correspond to headers defined in beginning
    if t > record_time:
        recorder_control.record_entry(t, threshold, q1, q3, dq1, dq3, theta1.value, theta2.value, tau1, tau2, body_height, tau_z, theta_z, phi1, phi3, tau_tot_1, tau_tot_2, ground, w[0], w[1])


