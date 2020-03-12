#!/usr/bin/env python
# coding: utf-8

import nest
import numpy
import scipy
import pylab
import constants
import os
import pdb

'''
some general comments
A NEST simulation consists of 3 main components
- nodes --> neurons, devices, sub-networks with dynamics state that can change over time depending on event
- events --> information pieces like spike-event, voltage and current events
- connections --> communications channels between nodes (weighted, delayeddd and specific to one event type)
'''
'''
v0*numpy.exp((t0-tn)/tau)+sum(1/tau * numpy.exp((ti-tn)/tau))
'''
# find force
def getforce(q, ts, v0):
    t_start = ts[0]
    t_end = ts[1]
    dt = t_end-t_start

    if os.path.exists("lastspikenum.txt"):
        data = numpy.genfromtxt("lastspikenum.txt")
        old_spikes = int(data)
    else:
        old_spikes = 0

    time = dt*1000 # convert from [s] to [ms]
    j1_dict = m_sens*q[0]
    j2_dict = m_sens*q[1]

    if j1_dict < 0:
        nest.SetStatus(pop_j1, {"rate": 0.0})
    else:
        nest.SetStatus(pop_j1, {"rate": j1_dict})

    if j2_dict < 0:
        nest.SetStatus(pop_j2, {"rate": 0.0})
    else:
        nest.SetStatus(pop_j2, {"rate": j2_dict})

    nest.Simulate(time)

#    pdb.set_trace()

    # peak amount
    dSD_refl = nest.GetStatus(spikedetector_refl, keys="events")[0]
    evs_refl = dSD_refl["senders"]
    ts_refl = dSD_refl["times"]
    ts_refl = ts_refl/1000

    all_spikes = len(evs_refl)  # amount of spikes (over whole time)
    n_spikes = all_spikes-old_spikes
    if old_spikes==0:
        cur_spikes_t = ts_refl[old_spikes:all_spikes]
        n_spikes = n_spikes-1
    else:
        cur_spikes_t = ts_refl[(old_spikes-1):all_spikes]

    # calculate average spike rate after time step
    # v_avr = v0*numpy.exp((t0-tn)/tau)+sum(1/tau * numpy.exp((ti-tn)/tau))
    for j in range(n_spikes):
        with open('av_fire.dat', 'a+') as v:
            v.write(str(v0)+'\n')
        y = v0*numpy.exp(-(cur_spikes_t[j+1]-cur_spikes_t[j])/tau_rs)+(1/tau_rs)
        v0 = y

#    pdb.set_trace()
    with open('spike.dat', 'a+') as s:
        s.write(str(ts_refl)+" "+str(evs_refl)+'\n')

    with open('lastspikenum.txt', 'w') as l:
        l.write(str(all_spikes)+'\n')

    # firing rate
    #    new_spike = n_spike-old_spikes
    #    f_rate = new_spike/dt

    f = m_f*v0
    #firing rate [Hz] * m_f
    F = [f, f]

    with open("spikenum.dat", "a+") as f:
        f.write(str(all_spikes)+" "+str(old_spikes)+" "+str(v0)+" "+str(F)+'\n')

    return F, v0


# load constants
params = constants.network()
n_sens, n_refl, p_con, tau_delSTDP, tau_rs, w, m_sens, m_f = params

# Create neurons
# define senory neurons of joint 1
pop_j1 = nest.Create("poisson_generator", n_sens)
# define senory neurons of joint 2
pop_j2 = nest.Create("poisson_generator", n_sens)
# define reflex neurons
pop_refl = nest.Create("iaf_psc_alpha", n_refl)

multimeter = nest.Create("multimeter", params={"withtime":True, "record_from":["V_m"]})
spikedetector_refl = nest.Create("spike_detector", params={"withgid": True, "withtime": True})

## Connections
conn_dict = {'rule': 'pairwise_bernoulli', 'p': p_con}
syn_dict = {'weight': w}
#syn_dict = {'delay': tau_delSTDP}

nest.Connect(pop_j1, pop_refl, conn_dict, syn_dict)
nest.Connect(pop_j2, pop_refl, conn_dict, syn_dict)
nest.Connect(multimeter, pop_refl)
nest.Connect(pop_refl, spikedetector_refl)

