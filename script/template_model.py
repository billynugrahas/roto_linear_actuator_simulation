#
#   This file is part of do-mpc
#
#   do-mpc: An environment for the easy, modular and efficient implementation of
#        robust nonlinear model predictive control
#
#   Copyright (c) 2014-2019 Sergio Lucia, Alexandru Tatulea-Codrean
#                        TU Dortmund. All rights reserved
#
#   do-mpc is free software: you can redistribute it and/or modify
#   it under the terms of the GNU Lesser General Public License as
#   published by the Free Software Foundation, either version 3
#   of the License, or (at your option) any later version.
#
#   do-mpc is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU Lesser General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with do-mpc.  If not, see <http://www.gnu.org/licenses/>.

import numpy as np
from casadi import *
from casadi.tools import *
import pdb
import sys
sys.path.append('/home/billy/niagraha_ws/src/spring-mass-MPC/')
import do_mpc


def template_model(symvar_type='SX'):
    """
    --------------------------------------------------------------------------
    template_model: Variables / RHS / AUX
    --------------------------------------------------------------------------
    """
    model_type = 'continuous' # either 'discrete' or 'continuous'
    model = do_mpc.model.Model(model_type, symvar_type)

    # Same example as shown in the Jupyter Notebooks.

    # Model variables:
    x = model.set_variable(var_type='_x', var_name='x')
    x_dot = model.set_variable(var_type='_x', var_name='x_dot')

    x_set_point = model.set_variable(var_type='_tvp', var_name='x_set_point')
    x_dot_set_point = model.set_variable(var_type='_tvp', var_name='x_dot_set_point')

    u = model.set_variable(var_type='_u', var_name='u')
    
    m = 0.07  # mass
    # m = 0.1  # mass
    # k = 0.2  # spring constant
    k = 1  # spring constant
    b = 0   #damping factor

    model.set_rhs('x', x_dot)

    x_dot2 = -k*x/m - b*x_dot/m + u/m
    model.set_rhs('x_dot', x_dot2)

    # Set expression. These can be used in the cost function, as non-linear constraints
    # or just to monitor another output.
    # model.set_expression(expr_name='cost', expr=(x**2 + x_dot**2))

    model.setup()

    return model