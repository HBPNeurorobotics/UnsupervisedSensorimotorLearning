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

@nrp.MapVariable("weight1", initial_value=1.0, scope=nrp.GLOBAL)
@nrp.MapVariable("weight2", initial_value=-1.5, scope=nrp.GLOBAL)

@nrp.MapCSVRecorder("recorder_control", filename="weights.csv", headers=["t", "w1", "w2", 'weight1', 'weight2'])

@nrp.MapRobotSubscriber('joint_states', Topic('/joint_states', JointState))
@nrp.MapRobotSubscriber('t_start', Topic('/record_time', Float64))

## can be used to create new topics
@nrp.MapRobotPublisher("weight_pub", Topic("/weights", Vector3))


#################### Function ###################
## This is where the function with its input starts
@nrp.Robot2Neuron()
def set_weight (t, recorder_control, joint_states, weight1, weight2, weight_pub, t_start):

    weight_change = 5

    # [0]    zero weights - to surpress jumping
    # [1]    constant weights (jump straight)
    # [2]    constant weights (jump forward)
    # [3]    constant weights (jump backward)
    # [4]    angle dependent (RAL paper)
    # [5]    adjusting according to Oja rule

    #################### Initialization ###################
    ## functions you want to use in the function need to be initialized again
    from gazebo_msgs.srv import ApplyJointEffort, JointRequest
    from geometry_msgs.msg import Wrench, Vector3
    from std_msgs.msg import Float64
    import rospy
    import math

    #################### Definitions ###################
    ## read out current positions of the joints from /joint_states
    # [0, 1, 2, 3, 4, 5] = [gear1, link1, link2, link3, world_1, world_2] (check SDF for definintion)
    phi1 = joint_states.value.position[1]
    phi2 = joint_states.value.position[2]
    phi3 = -joint_states.value.position[3]

    ## definitions according to matlab script/ RAL paper, defined solely for comparison
    offset_q1 = -40*math.pi/180
    offset_q2 = 40*math.pi/180
    q1 = phi1+offset_q1
    q2 = phi3+offset_q2

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
        w = [-0.5, 1]
    elif weight_change == 3:
        w = [-1, 0.5]
    elif weight_change == 4:
        alpha = math.pi * 1.1
        w = [math.sin(alpha), math.cos(alpha)]
    elif weight_change == 5:
        w =[weight1.value, weight2.value]

    weights = Vector3(w[0], w[1], 0.0)
    weight_pub.send_message(weights)


    #################### Oja rule ###################
    ## set parameters
    dt = 0.02 #sec --> default NRP update rate (CLE)
    if t > record_time:
        gamma = 0.5 #Hz
    else:
        gamma = 0.0 #Hz

    ## calculate rate change
    dw1 = gamma*(weight1.value*q1+weight2.value*q2)*(q1-((weight1.value*q1+weight2.value*q2)*weight1.value))
    dw2 = gamma*(weight1.value*q1+weight2.value*q2)*(q2-((weight1.value*q1+weight2.value*q2)*weight2.value))

    ## calculate new weight
    weight1.value = weight1.value+(dw1*dt)
    weight2.value = weight2.value+(dw2*dt)


    #################### Recording ###################
    # record data to CSV file --> needs to correspond to headers defined in beginning
    if t > record_time:
        recorder_control.record_entry(t, w[0], w[1], weight1.value, weight2.value)

    #################### Debugging ###################
    ## check interesting values during simulation
    #clientLogger.info('r weight', weight1.value, weight2.value)
