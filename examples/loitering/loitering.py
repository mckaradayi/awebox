#!/usr/bin/python3
"""
Circular pumping trajectory for the Ampyx AP2 aircraft.
Model and constraints as in:

"Performance assessment of a rigid wing Airborne Wind Energy pumping system",
G. Licitra, J. Koenemann, A. Bürger, P. Williams, R. Ruiterkamp, M. Diehl
Energy, Vol.173, pp. 569-585, 2019.

:author: Jochem De Schutter
:edited: Rachel Leuthold
"""

import awebox as awe
from ampyx_ap2_settings import set_ampyx_ap2_settings
from datetime import datetime
import os
import matplotlib.pyplot as plt
import numpy as np
import copy
import casadi as ca

# ask user to enter a suffix for results
description_suffix = input('Enter a test suffix\n')
# timestamp for suffix 
timestamp_suffix = datetime.now().strftime("%Y%m%d_%H%M%S")
# full suffix
full_suffix = f"{timestamp_suffix}_{description_suffix}"
# create a directory named the full_suffix
os.makedirs(f"./results/{full_suffix}")

# indicate desired system architecture
# here: single kite with 6DOF Ampyx AP2 model
options = {}
options['user_options.system_model.architecture'] = {1:0}
options = set_ampyx_ap2_settings(options)

# indicate desired operation mode
# here: lift-mode system with pumping-cycle operation, with a one winding trajectory
options['user_options.trajectory.type'] = 'power_cycle'
options['user_options.trajectory.system_type'] = 'lift_mode'
options['user_options.trajectory.lift_mode.windings'] = 1 # number of initial loops

# indicate desired environment
# here: wind velocity profile according to power-law
options['params.wind.z_ref'] = 100.0
options['params.wind.power_wind.exp_ref'] = 0.15
options['user_options.wind.model'] = 'power'
options['user_options.wind.u_ref'] = 0.0 # reference wind speed

# options['visualization.cosmetics.plot_ref'] = True
# options['visualization.cosmetics.plot_bounds'] = True 

# indicate numerical nlp details
# here: nlp discretization, with a zero-order-hold control parametrization, and a simple phase-fixing routine. also, specify a linear solver to perform the Newton-steps within ipopt.
options['nlp.n_k'] = 20 # number of intervals
options['nlp.collocation.u_param'] = 'zoh' # zero-order-hold controls
options['user_options.trajectory.lift_mode.phase_fix'] = 'simple'
options['solver.linear_solver'] = 'mumps' # if HSL is installed, otherwise 'mumps'

# solver tuning parameters
options['solver.mu_hippo'] = 1e-4
options['solver.max_iter'] = 1000

# build and optimize the NLP (trial)
trial = awe.Trial(options, 'Ampyx_AP2')
trial.build()
trial.optimize(intermediate_solve=True)
intermediate_sol_design = copy.deepcopy(trial.solution_dict)
trial.optimize(options_seed = options, warmstart_file = intermediate_sol_design, intermediate_solve=False, recalibrate_viz = True)
trial.plot(['states', 'invariants', 'constraints', 'quad'])
# plt.show()
plt.savefig(f'./results/{full_suffix}/plots_initial.pdf')
trial.write_to_csv(file_name = f'./results/{full_suffix}/data_initial', frequency=10., rotation_representation='dcm')


# fix params
fixed_params = {}
for theta in trial.model.variables_dict['theta'].keys():
    if theta != 't_f':
        fixed_params[theta] = trial.optimization.V_final['theta', theta].full()[0][0]
options['user_options.trajectory.fixed_params'] = fixed_params

zmin = np.linspace(100, 4000, 40, endpoint=True)
tf = np.linspace(20, 40, 40, endpoint = True)

hom_steps = 10
for idx, z in enumerate(zmin):

    if idx > 0:

        for jdx in range(1, hom_steps+1):

            step = jdx/hom_steps
            z_lb = (1-step)*zmin[idx-1] + step*z
            tf_ub = (1-step)*tf[idx-1] + step*tf[idx]
            options['model.system_bounds.x.q'] =  [np.array([-ca.inf, -ca.inf, z_lb]), np.array([ca.inf, ca.inf, ca.inf])]
            options['model.system_bounds.theta.t_f'] =  [5.0, tf_ub]
            print('================================')
            print('Solve trial for z_lb = {} m'.format(options['model.system_bounds.x.q']))
            print('================================')

            if idx == 1 and jdx == 1:
                trial.optimize(options_seed = options, warmstart_file = intermediate_sol_design, intermediate_solve = True)
            else:
                trial.optimize(options_seed = options, warmstart_file = intermediate_sol, intermediate_solve = True)

            intermediate_sol = copy.deepcopy(trial.solution_dict)

        trial.optimize(options_seed = options, warmstart_file = intermediate_sol, intermediate_solve=False, recalibrate_viz=False)
        trial.plot(['states', 'invariants', 'constraints', 'quad'])
        plt.savefig(f'./results/{full_suffix}/plots_z{round(z)}.pdf')
        trial.write_to_csv(file_name = f'./results/{full_suffix}/data_z{round(z)}', frequency=10., rotation_representation='dcm')