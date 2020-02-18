import rospy
from std_msgs.msg import Float64
from gazebo_msgs.srv import ApplyJointEffort, JointRequest
rospy.wait_for_service("gazebo/apply_joint_effort")
rospy.wait_for_service("gazebo/clear_joint_forces")

@nrp.MapVariable("applyEffortService", initial_value=None, scope=nrp.GLOBAL)
@nrp.MapVariable("clearJointService", initial_value=None, scope=nrp.GLOBAL)

@nrp.Robot2Neuron()
def applyEffort (t, applyEffortService, clearJointService):
    import rospy
    from std_msgs.msg import Float64
    from gazebo_msgs.srv import ApplyJointEffort, JointRequest

    if applyEffortService.value is None:
        applyEffortService.value = rospy.ServiceProxy("/gazebo/apply_joint_effort", ApplyJointEffort)

    if clearJointService.value is None:
        clearJointService.value = rospy.ServiceProxy("/gazebo/clear_joint_forces", JointRequest)

    def clearJoints():
        clearJointService.value('link1')
        clearJointService.value('link3')

    tau1 = 0.0
    tau2 = 0.0

    if t > 1: ##otherwise gazebo crashes
        clearJoints()

        applyEffortService.value('robot::link1', tau1, rospy.Time(), rospy.Duration(-1))
        applyEffortService.value('robot::link3', tau2, rospy.Time(), rospy.Duration(-1))
