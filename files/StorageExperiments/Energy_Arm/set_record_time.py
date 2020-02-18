from std_msgs.msg import Float64
@nrp.MapRobotPublisher("record_time", Topic("/record_time", Float64))

@nrp.Robot2Neuron()
def set_record_time (t, record_time):
    #log the first timestep (20ms), each couple of seconds
    
    t_start = 0.0    
    record_time.send_message(t_start)