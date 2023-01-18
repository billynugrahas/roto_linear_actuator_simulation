#!/usr/bin/python3

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
import matplotlib.pyplot as plt
import matplotlib as mpl
from casadi import *
from casadi.tools import *
import pdb
import sys
import time
# sys.path.append('../../')
sys.path.append('/home/billy/niagraha_ws/src/spring-mass-MPC/')



import do_mpc

from template_model import template_model
from template_mpc import template_mpc
from template_simulator import template_simulator

# ROS
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

rospy.init_node('needle_controller')
# prismatic_pub = rospy.Publisher("/needle_bot/prismatic_position_controller/command", Float64, queue_size=1) #gazebo
prismatic_pub = rospy.Publisher("/joint_states", JointState, queue_size=1) #visualize
msg = Float64()

log_index = '010'
# sys.stdout = open(f'./log/{log_index}_mpc_log.txt', 'w')






sys.stdout = open(f'/home/billy/niagraha_ws/src/spring-mass-MPC/log/{log_index}_mpc_log.txt', 'w')

""" User settings: """
show_animation = True
store_results = True

"""
Get configured do-mpc modules:
"""
model = template_model()
mpc = template_mpc(model)
simulator = template_simulator(model)
estimator = do_mpc.estimator.StateFeedback(model)


"""
Set initial state
"""
np.random.seed(99)

e = np.ones([model.n_x,1])
# x0 = np.random.uniform(0.1*e, 0.5*e) # Values between -0.5 and +0.5 for all states
x0 = np.zeros((2,1))
mpc.x0 = x0
simulator.x0 = x0
estimator.x0 = x0

# Use initial state to set the initial guess.
mpc.set_initial_guess()

"""
Setup graphic:
"""

color = plt.rcParams['axes.prop_cycle'].by_key()['color']


# We just want to create the plot and not show it right now. This "inline magic" supresses the output.
fig, ax = plt.subplots(3, sharex=True, figsize=(8,7))

mpc_plot = do_mpc.graphics.Graphics(mpc.data)
sim_plot = do_mpc.graphics.Graphics(simulator.data)


ax[0].set_title('x position:')
mpc_plot.add_line('_x', 'x', ax[0])
mpc_plot.add_line('_tvp', 'x_set_point', ax[0], color=color[1], linestyle='--', alpha=0.5)

ax[0].legend(
    mpc_plot.result_lines['_x', 'x']+mpc_plot.result_lines['_tvp', 'x_set_point']+mpc_plot.pred_lines['_x', 'x'],
    ['Recorded', 'Setpoint', 'Predicted'], title='X Position')


ax[1].set_title('x speed:')
mpc_plot.add_line('_x', 'x_dot', ax[1])
mpc_plot.add_line('_tvp', 'x_dot_set_point', ax[1], color=color[1], linestyle='--', alpha=0.5)

ax[1].legend(
    mpc_plot.result_lines['_x', 'x_dot']+mpc_plot.result_lines['_tvp', 'x_dot_set_point']+mpc_plot.pred_lines['_x', 'x_dot'],
    ['Recorded', 'Setpoint',  'Predicted'], title='X Speed')


ax[2].set_title('Inputs:')
mpc_plot.add_line('_u', 'u', ax[2])

ax[0].set_ylabel('position [m]')
ax[1].set_ylabel('speed [m/s]')
ax[2].set_ylabel('force [N]')
ax[2].set_xlabel('time [s]')

for sim_line_i in sim_plot.result_lines.full:
    sim_line_i.set_alpha(0.5)
    sim_line_i.set_linewidth(5)


fig.tight_layout()
plt.ion()

"""
Run MPC main loop:
"""
controlled = True
# controlled = False

for k in range(50):

    # breakpoint()

    if controlled:
        u0 = mpc.make_step(x0)
    else:
        if k >= 0 and k < 2:
            u0 = np.array([[1]])
        else:
            u0 = np.array([[0]])

        # Get current tvp, p and time (as well as previous u)
        mpc.u0 = u0
        u_prev = mpc.u0
        tvp0 = mpc.tvp_fun(mpc._t0)
        p0 = mpc.p_fun(mpc._t0)
        t0 = mpc._t0

        # Set the current parameter struct for the optimization problem:
        mpc.opt_p_num['_x0'] = mpc.x0
        mpc.opt_p_num['_u_prev'] = u_prev
        mpc.opt_p_num['_tvp'] = tvp0['_tvp']
        mpc.opt_p_num['_p'] = p0['_p']

        # Extract solution:
        mpc.opt_x_num['_u', 0, 0] = [DM(u0)]
        u0 = mpc.opt_x_num['_u', 0, 0]*mpc._u_scaling
        z0 = mpc.opt_x_num['_z', 0, 0, 0]*mpc._z_scaling
        aux0 = mpc.opt_aux_num['_aux', 0, 0]

        # Store solution:
        mpc.data.update(_x = x0)
        mpc.data.update(_u = u0)
        mpc.data.update(_z = z0)
        mpc.data.update(_tvp = tvp0['_tvp', 0])
        mpc.data.update(_time = mpc.t0)
        mpc.data.update(_aux = aux0)

        mpc.t0 = mpc.t0 + mpc.t_step

    # breakpoint()
    print("Log Index = ", log_index)

    print("State")
    print("x = ", x0[0][0])
    print("x_dot = ", x0[1][0], "\n")

    print("MPC Params")
    print("n_horizon = ", mpc.n_horizon)
    print("t_step = ", mpc.t_step)
    print("x set point = ", mpc.bounds['upper','_x','x'])

    msg.data = x0[0][0] #meter
    joint_state_publisher_msg = JointState()
    joint_state_publisher_msg.header.stamp = rospy.Time.now()
    joint_state_publisher_msg.header.frame_id = "base_footprint"
    joint_state_publisher_msg.name = []
    joint_state_publisher_msg.name.append("prismatic_joint")
    joint_state_publisher_msg.name.append("motor_joint")
    joint_state_publisher_msg.position = [x0[0][0], 0.0]
    joint_state_publisher_msg.velocity = [x0[1][0], 0.0]
    joint_state_publisher_msg.effort = [0.0, 0.0]
    prismatic_pub.publish(joint_state_publisher_msg)

    y_next = simulator.make_step(u0)
    x0 = estimator.make_step(y_next)

    if show_animation:
        mpc_plot.plot_results()
        mpc_plot.plot_predictions()
        sim_plot.plot_results()

        mpc_plot.reset_axes()
        sim_plot.reset_axes()
        plt.show()
        plt.pause(0.01)


input('Press any key to exit.')

joint_state_publisher_msg = JointState()
joint_state_publisher_msg.header.stamp = rospy.Time.now()
joint_state_publisher_msg.header.frame_id = "base_footprint"
joint_state_publisher_msg.name = []
joint_state_publisher_msg.name.append("prismatic_joint")
joint_state_publisher_msg.name.append("motor_joint")
joint_state_publisher_msg.position = [0.0, 0.0]
joint_state_publisher_msg.velocity = [0.0, 0.0]
joint_state_publisher_msg.effort = [0.0, 0.0]
prismatic_pub.publish(joint_state_publisher_msg)

# Store results:
if store_results:
    do_mpc.data.save_results([mpc, simulator], 'spring_mass')

sys.stdout.close()
