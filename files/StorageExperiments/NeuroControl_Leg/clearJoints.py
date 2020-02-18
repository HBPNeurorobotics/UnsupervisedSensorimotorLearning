import rospy
from std_msgs.msg import Float64
from gazebo_msgs.srv import ApplyJointEffort, JointRequest

rospy.wait_for_service("gazebo/apply_joint_effort")
rospy.wait_for_service("gazebo/clear_joint_forces")

@nrp.MapVariable("clearJointService", initial_value=None, scope=nrp.GLOBAL)

@nrp.Robot2Neuron()
def clearJoints (t, clearJointService):
    import rospy
    from std_msgs.msg import Float64
    from gazebo_msgs.srv import ApplyJointEffort, JointRequest

    if clearJointService.value is None:
        clearJointService.value = rospy.ServiceProxy("/gazebo/clear_joint_forces", JointRequest)

    clearJointService.value('link1')
    clearJointService.value('link3')
