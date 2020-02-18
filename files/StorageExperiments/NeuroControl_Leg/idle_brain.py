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
    #sim.setup(timestep=0.1, min_delay=0.1, max_delay=20., threads=1, rng_seeds=[1234])

    ## PARAMETER

    # neuron setup
    n_refl = 100         # amount of reflex neurons
    n_raphe = 100        # amount of Raphe neuros per pool
    n_total = n_refl+2*n_raphe

    w = 1.0             # neuron weight

    # neuron and synapse parameter
    SENSORPARAMS = {'v_rest': -70.0,
                'tau_m': 10.0,
                'v_thresh': -50.0,
                'tau_refrac': 5.0}

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

    refl_class = sim.IF_cond_alpha(**SENSORPARAMS)
    neurons = sim.Population(size=n_total, cellclass=refl_class, label='neurons')

    sim.initialize(neurons)
    return neurons

circuit = create_brain()
