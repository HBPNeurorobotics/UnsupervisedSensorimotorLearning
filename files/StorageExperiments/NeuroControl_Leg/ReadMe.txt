In this experiment set up, a simple one leg hopper model can be controlled either by using a neuronal controller based on a spinal chord model or through a Bang Bang Controller.

When the experiment is open, both possible controllers are activated.
If ou open the TransferFunction Editor in the GUI, you see six TF:
[1] all_neurons_spike_monitor: only necessary to visualize the spikes in the BrainVisualizer
[2] setNeurons : sets the weights for the neuron fire rate
[3] setMuscles : reads the average spike rate of the brain and translates it into torques to apply to the jonts

[4] read_pos   : only necessary to read out and save the joint positions and velocities in a .csv file

[5] ReflexController: the Bang Bang controller, implemented after (Stratmann et al, 2016)
[6] set_weight : TF to change the weighting for the BangBang Controller, e.g. to let the hopper jump forward

To use the Neurocontrol:
activate [1], [2] and [3], optional [4]

To use the Bang Bang Controller:
activate [5] and [6], optional [4]



for further Details on the Controller and model see:
Stratmann, P., Lakatos, D., Özparpucu, M. C., & Albu-Schäffer, A. (2016). Legged elastic multibody systems: adjusting limit cycles to close-to-optimal energy efficiency. IEEE Robotics and Automation Letters, 2(2), 436-443.
