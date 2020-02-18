# -*- coding: utf-8 -*-
# pragma: no cover

__author__ = 'Template'

from hbp_nrp_cle.brainsim import simulator as sim
import numpy as np
import logging

logger = logging.getLogger(__name__)

def create_brain():
    """
    Initializes PyNN with the minimal neuronal network
    """
    #sim.setup(timestep=0.1, min_delay=0.1, max_delay=20.0, threads=1, rng_seeds=[1234])

    ## PARAMETER

    # neuron setup
    #n_sens = 10         # amount of sensory neurons (per joint)
    n_refl = 100         # amount of reflex neurons
    n_raphe = 100
    n_test = 100
    n_total = n_refl+2*n_raphe

    w = 1.0             # neuron weight

    # neuron and synapse parameter
    SENSORPARAMS = {'cm': 0.025,
                'v_rest': -60.5,
                'tau_m': 10.,
                'e_rev_E': 0.0,
                'e_rev_I': -75.0,
                'v_reset': -60.5,
                'v_thresh': -60.0}


    REFL_PARAMS = {'cm': 0.025,
                'tau_m': 10.0,
                'v_reset': -60.0,
                'v_thresh': -55.0}

    RAPHENUCLEI_PARAMS = {'cm': 0.025,
                'v_reset': -60.0,
                'v_thresh': -55.0}

    SYNAPSE_PARAMS = {'weight': w,
                  'delay': 0.0,
                  'U': 1.0,
                  'tau_rec': 1.0,
                  'tau_facil': 1.0}

    # ask PHILIPP which kind of neurons from PyNN
    refl_class = sim.IF_cond_alpha(**SENSORPARAMS)
    neurons = sim.Population(size=n_total, cellclass=refl_class, label='neurons')

    #neurons[0:n_refl].set(**REFL_PARAMS)
    #neurons[n_refl:n_total].set(**RAPHENUCLEI_PARAMS)

    #sim.Projection(presynaptic_population=pop_j1,
    #               postsynaptic_population=pop_refl,
    #               connector=connector,
    #               synapse_type=synapse_type,
    #               receptor_type='excitatory')

    #poisson_class = sim.SpikeSourcePoisson()
    #test_pop = sim.Population(size=n_test, cellclass=poisson_class, label='test_pop')
    #test_pop2= sim.Population(size=1, cellclass=refl_class, label='test_pop2')
    #test_pop.rate = 1000000.0
    #sim.Projection(test_pop, test_pop2, connector = sim.AllToAllConnector())
    #population = sim.Assembly(neurons, test_pop, test_pop2)
    #sim.initialize(population)
    #return population

    sim.initialize(neurons)
    return neurons


circuit = create_brain()
#test_pop = circuit.get_population('test_pop')
#test_pop2 = circuit.get_population('test_pop2')
#neurons = circuit.get_population('neurons')
