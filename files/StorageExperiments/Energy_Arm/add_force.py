## Apply force to the body of the jumper

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

#################### Function ###################
## This is where the function with its input starts
@nrp.MapCSVRecorder("recorder_force", filename="force.csv", headers=["g_t", "t", "force", "torque"])

@nrp.MapVariable("dur_b", initial_value=60.0, scope=nrp.GLOBAL)
@nrp.MapVariable("dur_l2", initial_value=1.0, scope=nrp.GLOBAL)

@nrp.MapRobotSubscriber('t_start', Topic('/record_time', Float64))
@nrp.MapRobotSubscriber("gazebo_time", Topic('/clock', Clock))

@nrp.MapRobotPublisher("force_humerus", Topic("/force_humerus", geometry_msgs.msg.Wrench))
@nrp.MapRobotPublisher("force_radius", Topic("/force_radius", geometry_msgs.msg.Wrench))

def add_force(t, gazebo_time, force_humerus, force_radius, recorder_force, t_start, dur_b, dur_l2):

    #################### Initialization ###################
    ## functions you want to use in the function need to be initialized again
    from gazebo_msgs.srv import ApplyJointEffort, JointRequest
    from geometry_msgs.msg import Wrench, Vector3
    from std_msgs.msg import Float64
    import rospy
    import math
    import numpy as np

    #record_time = t_start.value.data
    dt = 0.02
    #dur_b.value = 60.0
    #dur_l2.value = 2.0

    start_body =  60*7.5 #record_time + 10.0
    start_link2 = 0 #record_time + 25.0

    ## add force to body
    if t > start_body and t < t+dur_b.value:
        #f_b = 1.0*9.81 # for initial jump
        f_b = -2.  # for added weight
        dur_b.value = dur_b.value - dt
        clientLogger.info("duck:")
    else:
        f_b = 0

    if t > start_link2 and t < t+dur_l2.value:
        t_l2 = -1.
        dur_l2.value = dur_l2.value - dt
    else:
        t_l2 = 0

    f1 = 0
    f2 = 0

    # additional weight
    force_l1  = Vector3(0.0, 0.0, f1) # hopper moves along y and z axis
    torque_l1 = Vector3(0.0, 0.0, 0.0)
    wrench_l1 = Wrench(force_l1, torque_l1)

    # stumble
    force_l2  = Vector3(0.0, 0.0, f2)
    torque_l2 = Vector3(0.0, 0.0, 0.0)
    wrench_l2 = Wrench(force_l2, torque_l2)

    force_humerus.send_message(wrench_l1)
    force_radius.send_message(wrench_l2)

    ## When the time step within gazebo is manually changed during the simulation, the times (NRP and gazebo) do not sync
    ## therefore the gazebo time can additionally be recorded
    gazebo_t = gazebo_time.value
    g_secs = gazebo_t.clock.secs
    g_nsecs = gazebo_t.clock.nsecs/1000000000.0
    g_t = g_secs+g_nsecs

    '''
    if t > record_time:
        recorder_force.record_entry(g_t, t, force_b, torque_b)
    '''
    clientLogger.info("force:", f_b, dur_b.value)
