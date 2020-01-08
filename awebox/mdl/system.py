#
#    This file is part of awebox.
#
#    awebox -- A modeling and optimization framework for multi-kite AWE systems.
#    Copyright (C) 2017-2019 Jochem De Schutter, Rachel Leuthold, Moritz Diehl,
#                            ALU Freiburg.
#    Copyright (C) 2018-2019 Thilo Bronnenmeyer, Kiteswarms Ltd.
#    Copyright (C) 2016      Elena Malz, Sebastien Gros, Chalmers UT.
#
#    awebox is free software; you can redistribute it and/or
#    modify it under the terms of the GNU Lesser General Public
#    License as published by the Free Software Foundation; either
#    version 3 of the License, or (at your option) any later version.
#
#    awebox is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#    Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public
#    License along with awebox; if not, write to the Free Software Foundation,
#    Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
#
#
'''
module that describes the awe system under consideration, geometry, etc.
python-3.5 / casadi 3.0.0
- author: elena malz, chalmers 2016
- edited: rachel leuthold, alu-fr 2017
          jochem de schutter, alu-fr 2017

'''

import casadi.tools as cas
import awebox.tools.struct_operations as struct_op
import pdb

def generate_structure(options, architecture):

    kite_dof = options['kite_dof']
    surface_control = options['surface_control']
    tether_control_var = options['tether']['control_var']

    # _system architecture (see _zanon2013a)
    number_of_nodes = architecture.number_of_nodes
    kite_nodes = architecture.kite_nodes
    parent_map = architecture.parent_map

    # _states, generalized coordinates and controls related to the tether
    # connection points
    # connection points only have position and velocity
    tether_states = [('q', (3, 1)), ('dq', (3, 1))]
    tether_gc = ['q']  # generalized coordinates
    tether_controls = []  # [('u',(3,1))]  # artificial force
    tether_multipliers = [('lambda', (1, 1))]

    # _states, generalized coordinates and controls related to kites

    kite_states = [('q', (3, 1)), ('dq', (3, 1))]
    kite_controls = [('f_fict', (3, 1))]
    kite_multipliers = [('lambda', (1, 1))]

    kite_gc = ['q']

    if int(kite_dof) == 3:
        kite_states = kite_states + [('coeff', (2, 1))]
        kite_controls = kite_controls + [('dcoeff', (2, 1))]

    elif int(kite_dof) == 6:
        kite_states = kite_states + [('omega', (3, 1)), ('r', (9, 1))]
        kite_controls = kite_controls + [('m_fict', (3, 1))]

        if int(surface_control) == 0:
            # delta:
            # (aileron left-right [right teu+, rad],
            # elevator [ted+, rad],
            # rudder [tel+, rad])
            kite_controls = kite_controls + [('delta', (3, 1))]

        if int(surface_control) == 1:
            # delta:
            # (aileron left-right [right teu+, rad],
            # elevator [ted+, rad],
            # rudder [tel+, rad])
            kite_states = kite_states + [('delta', (3, 1))]
            kite_controls = kite_controls + [('ddelta', (3, 1))]

    else:
        raise ValueError('kite dof option %s not inluded at present', str(kite_dof))

    # add drag mode states and controls
    if options['trajectory']['system_type'] == 'drag_mode':
        kite_states += [('kappa', (1,1))]
        kite_controls += [('dkappa',(1,1))]

    # _list states, generalized coordinates and controls of all the nodes
    # together
    system_states = []
    system_gc = []
    system_controls = []
    system_multipliers = []

    system_lifted = []

    for n in range(1, number_of_nodes):
        parent = parent_map[n]

        if n in kite_nodes:
            system_states.extend(
                [(kite_states[i][0] + str(n) + str(parent), kite_states[i][1]) for i in range(len(kite_states))])
            system_controls.extend(
                [(kite_controls[i][0] + str(n) + str(parent), kite_controls[i][1]) for i in range(len(kite_controls))])
            system_multipliers.extend(
                [(kite_multipliers[i][0] + str(n) + str(parent), kite_multipliers[i][1]) for i in range(len(kite_multipliers))])

            system_gc.extend([kite_gc[i] + str(n) + str(parent)
                              for i in range(len(kite_gc))])

        else:
            system_states.extend(
                [(tether_states[i][0] + str(n) + str(parent), tether_states[i][1]) for i in range(len(tether_states))])
            system_controls.extend(
                [(tether_controls[i][0] + str(n) + str(parent), tether_controls[i][1]) for i in range(len(tether_controls))])
            system_multipliers.extend(
                [(tether_multipliers[i][0] + str(n) + str(parent), tether_multipliers[i][1]) for i in range(len(tether_multipliers))])

            system_gc.extend([tether_gc[i] + str(n) + str(parent)
                              for i in range(len(tether_gc))])

    # add cross-tethers
    if options['cross_tether'] and len(kite_nodes) > 1:
        for l in architecture.layer_nodes:
            kite_children = architecture.kites_map[l]
            if len(kite_children) == 2:
                system_multipliers.extend(
                        [
                            (
                            tether_multipliers[i][0] + str(kite_children[0]) + str(kite_children[1]),
                            tether_multipliers[i][1]
                            )
                            for i in range(len(tether_multipliers))
                        ]
                    )
            else:
                for k in range(len(kite_children)):
                    system_multipliers.extend(
                        [
                            (
                            tether_multipliers[i][0] + str(kite_children[k]) + str(kite_children[(k+1)%len(kite_children)]),
                            tether_multipliers[i][1]
                            )
                            for i in range(len(tether_multipliers))
                        ]
                    )

    # _add global states and controls
    system_states.extend([('l_t', (1, 1)), ('dl_t', (1, 1))]) # main tether length and speed

    # _energy + main tether length and speed
    if tether_control_var == 'ddl_t':
        system_controls.extend([('ddl_t', (1, 1))])  # main tether acceleration
    elif tether_control_var == 'dddl_t':
        system_states.extend([('ddl_t', (1, 1))]) # main tether acceleration
        system_controls.extend([('dddl_t', (1, 1))])  # main tether jerk
    else:
        raise ValueError('invalid tether control variable chosen')

    if options['integral_outputs']:
        32.0
    else:
        system_states.extend([('e', (1, 1))]) # energy

    # introduce aerodynamics variables
    [system_lifted, system_states] = extend_aerodynamics(options, system_lifted, system_states, architecture)


    # system state derivatives
    system_derivatives = []
    for i in range(len(system_states)):
        system_derivatives.extend([('d'+system_states[i][0], system_states[i][1])])

    # system parameters
    system_parameters = [('l_s', (1, 1)), ('l_i', (1, 1)), ('diam_s', (1, 1)), ('diam_t', (1, 1)), ('t_f',(1,1))]

    # add cross-tether lengths and diameters
    if options['cross_tether'] and len(kite_nodes) > 1:
        for l in architecture.layer_nodes:
            system_parameters.extend(
                [
                    ('l_c{}'.format(l),(1,1)), ('diam_c{}'.format(l),(1,1))
                ]
            )

    # store variables in dict
    system_variables_list = {'xd':system_states,
                             'xddot':system_derivatives,
                             'u':system_controls,
                             'xa':system_multipliers,
                             'theta':system_parameters}

    if not system_lifted == []:
        system_variables_list['xl'] = system_lifted

    return system_variables_list, system_gc

def extend_aerodynamics(options, system_lifted, system_states, architecture):

    induction_model = options['induction_model']
    comparison_labels = options['aero']['actuator']['comparison_labels']

    any_asym = any('asym' in label for label in comparison_labels)
    any_unsteady = any(label[0] == 'u' for label in comparison_labels)

    # induction factor
    if not (induction_model == 'not_in_use'):

        for kite in architecture.kite_nodes:
            parent = architecture.parent_map[kite]
            system_states.extend([('local_a' + str(kite) + str(parent), (1, 1))])

    if induction_model == 'vortex':

        n_k = options['aero']['vortex']['n_k']
        d = options['aero']['vortex']['d']
        full_length = (n_k * d)
        for kite in architecture.kite_nodes:
            parent = architecture.parent_map[kite]
            for dim in ['x', 'y', 'z']:
                system_states.extend([('w' + dim + '_ext' + str(kite) + str(parent), (full_length, 1))])
                system_states.extend([('dw' + dim + '_ext' + str(kite) + str(parent), (full_length, 1))])


    if induction_model == 'actuator':

        for kite in architecture.kite_nodes:
            parent = architecture.parent_map[kite]
            system_lifted.extend([('varrho' + str(kite) + str(parent), (1, 1))])
            system_states.extend([('psi' + str(kite) + str(parent), (1, 1))])
            system_lifted.extend([('cospsi' + str(kite) + str(parent), (1, 1))])
            system_lifted.extend([('sinpsi' + str(kite) + str(parent), (1, 1))])

        for layer_node in architecture.layer_nodes:

            for label in comparison_labels:
                system_states.extend([('a_' + label + str(layer_node), (1, 1))])
                system_lifted.extend([('corr_' + label + str(layer_node), (1, 1))])
                system_lifted.extend([('chi_' + label + str(layer_node), (1, 1))])

                if any_unsteady:
                    system_states.extend([('da_' + label + str(layer_node), (1, 1))])

                if any_asym:
                    system_states.extend([('acos_' + label + str(layer_node), (1, 1))])
                    system_states.extend([('asin_' + label + str(layer_node), (1, 1))])
                    system_lifted.extend([('LL_' + label + str(layer_node), (9, 1))])
                    system_lifted.extend([('c_tilde_' + label + str(layer_node), (3, 1))])
                    system_lifted.extend([('tanhalfchi_' + label + str(layer_node), (1, 1))])
                    system_lifted.extend([('sechalfchi_' + label + str(layer_node), (1, 1))])

                    if any_unsteady:
                        system_states.extend([('dacos_' + label + str(layer_node), (1, 1))])
                        system_states.extend([('dasin_' + label + str(layer_node), (1, 1))])

            system_states.extend([('ct' + str(layer_node), (1, 1))])
            system_states.extend([('bar_varrho' + str(layer_node), (1, 1))])
            system_lifted.extend([('t_star' + str(layer_node), (1, 1))])

            if any_asym:
                system_lifted.extend([('cmy' + str(layer_node), (1, 1))])
                system_lifted.extend([('cmz' + str(layer_node), (1, 1))])

            system_lifted.extend([('rot_matr' + str(layer_node), (9, 1))])
            system_lifted.extend([('uzero_matr' + str(layer_node), (9, 1))])

            system_lifted.extend([('n_hat_slack' + str(layer_node), (6, 1))])

            system_lifted.extend([('n_vec_length' + str(layer_node), (1, 1))])
            system_lifted.extend([('u_vec_length' + str(layer_node), (1, 1))])
            system_lifted.extend([('z_vec_length' + str(layer_node), (1, 1))])

            system_lifted.extend([('qzero' + str(layer_node), (1, 1))])
            system_lifted.extend([('area' + str(layer_node), (1, 1))])

            system_lifted.extend([('gamma' + str(layer_node), (1, 1))])
            system_lifted.extend([('g_vec_length' + str(layer_node), (1, 1))])
            system_lifted.extend([('cosgamma' + str(layer_node), (1, 1))])
            system_lifted.extend([('singamma' + str(layer_node), (1, 1))])

    return system_lifted, system_states

def define_bounds(options, variables):

    variable_bounds = {}
    for variable_type in list(variables.keys()):
        variable_bounds[variable_type] = {}
        for name in struct_op.subkeys(variables,variable_type):

            variable_bounds[variable_type][name] = {}
            if variable_type in list(options.keys()):
                var_name = struct_op.get_node_variable_name(name) # omit node numbers
                if name in list(options[variable_type].keys()): # check if variable has node bounds
                    variable_bounds[variable_type][name]['lb'] = options[variable_type][name][0]
                    variable_bounds[variable_type][name]['ub'] = options[variable_type][name][1]
                elif var_name in list(options[variable_type].keys()): # check if variable has global bounds
                    variable_bounds[variable_type][name]['lb'] = options[variable_type][var_name][0]
                    variable_bounds[variable_type][name]['ub'] = options[variable_type][var_name][1]
                else:
                    variable_bounds[variable_type][name]['lb'] = -cas.inf
                    variable_bounds[variable_type][name]['ub'] = cas.inf
            else:
                variable_bounds[variable_type][name]['lb'] = -cas.inf
                variable_bounds[variable_type][name]['ub'] = cas.inf
    return variable_bounds

def scale_bounds(variable_bounds, scaling):
    for variable_type in list(variable_bounds.keys()):
        for name in list(variable_bounds[variable_type].keys()):
            variable_bounds[variable_type][name]['lb'] = variable_bounds[variable_type][name]['lb']/scaling[variable_type][name]
            variable_bounds[variable_type][name]['ub'] = variable_bounds[variable_type][name]['ub']/scaling[variable_type][name]

    return variable_bounds

##
#  @brief Method to construct a system parameter struct out of the model options.
#  @param options The modeling options.
#  @return sys_params System parameters casadi struct.
def generate_system_parameters(options):

    # extract parametric options
    parametric_options = options['params']

    parameters_dict = {}
    # generate nested casadi structure for system parameters
    parameters_dict['theta0'] = struct_op.generate_nested_dict_struct(parametric_options)
    parameters_dict['phi'] = generate_optimization_parameters()

    parameters = cas.struct_symSX([
        cas.entry('theta0', struct= parameters_dict['theta0']),
        cas.entry('phi', struct= parameters_dict['phi'])]
    )

    return parameters, parameters_dict

def generate_optimization_parameters():

    # variable system parameters
    p_dec = cas.struct_symSX([(
        cas.entry('gamma'), # force homotopy variable
        cas.entry('tau'),   # tether drag homotopy variable
        cas.entry('iota'),  # induction homotopy variable
        cas.entry('psi'),    # power homotopy variable
        cas.entry('eta'),  # nominal landing homotopy variable
        cas.entry('nu'),  # compromised landing homotopy variable
        cas.entry('upsilon'),  # transition homotopy variable
    )])

    optimization_parameters = p_dec

    return optimization_parameters

