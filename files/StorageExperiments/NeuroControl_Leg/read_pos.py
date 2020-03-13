import rospy
from std_msgs.msg import Float64
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState

rospy.wait_for_service("gazebo/get_link_state")

@nrp.MapVariable("readLinkState", initial_value=None, scope=nrp.GLOBAL)

@nrp.MapRobotSubscriber('link_states', Topic('/gazebo/link_states', LinkStates))
@nrp.MapRobotSubscriber("joint_states", Topic("/joint_states", JointState))
@nrp.MapCSVRecorder("recorder_pos", filename="positions.csv", headers=["t","body_x", "body_z", "foot_z", "q1", "q2", "phi1", "phi2"])

@nrp.Robot2Neuron()
def read_pos(t, link_states, recorder_pos, joint_states, readLinkState):
#pub_foot_z, pub_body_z):

    ## Initialization
    from geometry_msgs.msg import Wrench, Vector3
    from std_msgs.msg import Float64
    import rospy
    import math
    from numpy.linalg import norm
    from gazebo_msgs.srv import GetLinkState
    from tf.transformations import euler_from_quaternion, quaternion_from_euler, compose_matrix, quaternion_matrix

    if readLinkState.value is None:
        readLinkState.value = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)

    ## set time from which recording of data (for CSV) should start
    record_time = 0.0

    # get current angle
    phi1 = joint_states.value.position[1]
    phi3 = -joint_states.value.position[3]

    ## definitions according to matlab script/ RAL paper, defined solely for comparison
    offset_q1 = -40*math.pi/180
    offset_q2 = 40*math.pi/180
    q1 = phi1+offset_q1
    q3 = phi3+offset_q2

    body_pos = readLinkState.value('base_link', 'link')
    body_z = body_pos.link_state.pose.position.z
    body_x = body_pos.link_state.pose.position.y

    lower_leg_pos = readLinkState.value('link2_link', 'link')
    (knee_x, knee_y, knee_z) = [lower_leg_pos.link_state.pose.position.x, lower_leg_pos.link_state.pose.position.y, lower_leg_pos.link_state.pose.position.z]
    orientation_q = lower_leg_pos.link_state.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    #matrix_k = quaternion_matrix(orientation_list)
    #matrix_k[:3,3] = [knee_x, knee_y, knee_z]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    matrix_knee = compose_matrix(scale=None, shear=None, angles=[roll, pitch, yaw], translate=[knee_x, knee_y, knee_z], perspective=None)
    matrix_kf = compose_matrix(scale=None, shear=None, angles=[0, 0, 0], translate=[0, 0.08, 0], perspective=None)
    matrix_foot = np.dot(matrix_knee, matrix_kf)

    foot_z = matrix_foot[2][3]#-0.0239

    if t > record_time:
        recorder_pos.record_entry(t, body_x, body_z, foot_z, q1, q3, phi1, phi3)

