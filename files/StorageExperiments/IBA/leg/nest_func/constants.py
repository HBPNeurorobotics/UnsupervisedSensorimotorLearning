
#for mechanical system
def mechanics():
    m1 = 0.5    # mass 1 [kg]
    m2 = 0.5    # mass 2 [kg]
    k0 = 8      # outer spring stiffness [N/m]
    k1 = 15     # inner spring stiffness [N/m]
    d0 = 0.3    # damper [Ns/m^2]
    w1 = 0.1    # weight 1
    w2 = 0.1    # weight 2

    p_mech = [m1, m2, k0, k1, d0, w1, w2]
    return p_mech


#for neural network
def network():
    n_sens = 10     # number of sensor neurons
    n_refl = 10     # number of reflex neurons
    p_con = 1.0     # connectivity probablity
    tau_delSTDP = 30 # connection delay (not implemented yet)
    tau = 0.05      # time constant to calculate fire rate [s]
    w = 5000.0      # weight of poisson neuron
    m_sens = 10     # transform weight sensor input [Hz/m]
    m_f = 0.001     # transform weight force output [Hz/m]

    p_net = [n_sens, n_refl, p_con, tau_delSTDP, tau, w, m_sens, m_f]
    return p_net

