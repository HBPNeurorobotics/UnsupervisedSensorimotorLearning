import rospy
from std_msgs.msg import Float64
from gazebo_msgs.srv import ApplyJointEffort, JointRequest
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState 
rospy.wait_for_service("gazebo/apply_joint_effort")
rospy.wait_for_service("gazebo/clear_joint_forces")

@nrp.MapVariable("applyEffortService", initial_value=None, scope=nrp.GLOBAL)
@nrp.MapVariable("clearJointService", initial_value=None, scope=nrp.GLOBAL)

@nrp.MapRobotSubscriber("joint_states", Topic("/joint_states", JointState))

@nrp.Robot2Neuron()
def applyEffort (t, applyEffortService, clearJointService, joint_states):
    import rospy
    import math
    from std_msgs.msg import Float64
    from gazebo_msgs.srv import ApplyJointEffort, JointRequest
    
    if applyEffortService.value is None:
        applyEffortService.value = rospy.ServiceProxy("/gazebo/apply_joint_effort", ApplyJointEffort)
                 
    if clearJointService.value is None:
        clearJointService.value = rospy.ServiceProxy("/gazebo/clear_joint_forces", JointRequest)
    
    def clearJoints(): 
        clearJointService.value('HumerusBone_shoulder')
        clearJointService.value('RadiusBone_ellbow')
                     
    k = 10
    d = 0.7
    
    phi1 = joint_states.value.position[1] #shoulder
    phi2 = joint_states.value.position[2] #ellbow
   
    q1 = phi1
    q2 = phi1+phi2
    q0_1 = 0
    q0_2 = 0
    

    dq1 = joint_states.value.velocity[1]
    dq2 = joint_states.value.velocity[2]
    
    tau1=(-(q1-q0_1))*(k) -d*dq1
    tau2=(-(q2-q0_2))*(k) -d*dq2
    
    tau1_appl = tau1
    tau2_appl = tau2
  
    clearJoints()
    if t >1: ## otherwise gazebo crashes
        applyEffortService.value('myoarm::HumerusBone_shoulder', tau1_appl, rospy.Time(), rospy.Duration(-1))
        applyEffortService.value('myoarm::RadiusBone_ellbow', tau2_appl, rospy.Time(), rospy.Duration(-1))
        
    clientLogger.info(phi1*180/math.pi, phi2*180/math.pi)    
        