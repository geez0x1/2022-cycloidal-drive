#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 26 11:14:38 2021

@author: wesley
"""

from __future__ import print_function

import odrive
from odrive.enums import *
import time
import math
import csv
from common import get_output_filename, save_data

# Experiment parameters (pendulum)
def loadInertia():
	m_weights	= 2.0		# Weight plates mass [kg]
	m_bar		= 0.432		# Pendulum mass [kg]
	m_mounting	= 0.211		# Mounting stuff (thread, nuts, etc) [kg]
	L			= 0.5		# Pendulum length

	return (m_weights+m_mounting) * L**2 + (1/3) * m_bar * L**2

# Experiment parameters (LIGHT SHORT pendulum)
def loadInertia_light():
	m_weights	= 0.0		# Weight plates mass [kg]
	m_bar		= (44/50)*0.432 # Pendulum mass [kg]
	m_mounting	= 0			# Mounting stuff (thread, nuts, etc) [kg]
	L			= 0.44		# Pendulum length

	return (m_weights+m_mounting) * L**2 + (1/3) * m_bar * L**2


# Demo parameters
sample_time		= 0.0125	# Sample time [s] # Limited by USB/UART?
n				= 11	# True gear ratio
turns2deg		= 360	# One turn in [deg]
deg2rad			= math.pi/180 # Degrees to radians
turns2rad		= turns2deg*deg2rad # Turns to rad []
inertia			= 0.0*loadInertia()	# Load inertia

# Motion behaviour
max_speed		= 10	# Max output speed [rad/s]
max_accel		= 10	# Max acceleration/deceleration [rad/s^2]
target_pos		= [math.pi, 1.5*math.pi, 0.5*math.pi, 2*math.pi] # Target positions [rad]
static_time		= 0.05	# How long to stand still between positions [s]
reached_margin	= 0.01	# Acceptable error for reaching setpoint [rad], at output

# Control gains
pos_gain			= 15*20		# [turns/s / turn] - default 20
vel_gain			= 20*0.16	# [Nm / turns/s] - default 0.16
vel_integrator_gain	= 20*0.32	# [Nm / turn/s^2] - default 0.32
current_lim			= 60		# Current limit [A] (40A tops out at around 31 Nm)
requested_current	= 80		# Current sensing limit [A]
# Not touching current control gains current_gain and
# current_integrator_gain - they are automatically set
# for a given current control bandwidth.

# Prompt user for output filename
out_filename = get_output_filename('out_demo1.csv')

# Find a connected ODrive (this will block until you connect one)
print("Finding odrive..")
my_drive = odrive.find_any()

# First set the reference position to whatever the current position is
input_pos_initial = my_drive.axis0.encoder.pos_estimate
my_drive.axis0.controller.input_pos = input_pos_initial
print('Initial position offset is', round(turns2rad*input_pos_initial/n,2), 'rad.')
print('NOTE: Make sure to start this experiment in a down hanging position!')

# Set control mode
# "Trajectory control" - essentially minimum jerk
print('Setting control mode and parameters..')
my_drive.axis0.controller.config.control_mode	= CONTROL_MODE_POSITION_CONTROL
my_drive.axis0.controller.config.input_mode		= INPUT_MODE_TRAP_TRAJ
my_drive.axis0.trap_traj.config.vel_limit		= n * max_speed / turns2rad
my_drive.axis0.trap_traj.config.accel_limit		= n * max_accel / turns2rad
my_drive.axis0.trap_traj.config.decel_limit		= n * max_accel / turns2rad
my_drive.axis0.controller.config.inertia		= inertia

# Set control gains
my_drive.axis0.controller.config.pos_gain = pos_gain
my_drive.axis0.controller.config.vel_gain = vel_gain
my_drive.axis0.controller.config.vel_integrator_gain = vel_integrator_gain

# Set current limit
my_drive.axis0.motor.config.current_lim = current_lim
my_drive.axis0.motor.config.requested_current_range = requested_current

print('Starting closed loop control..')
my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
time.sleep(1.0)

# Record data
print('Recording data..')
data = []
t0 = time.monotonic()

# Iterator through target positions
motion_it = 0
motion_setpoint_reached = True
t_motion_setpoint_reached = t0

# Until we break..
while True:
	# Get iteration start time
	t = time.monotonic()

	# Get data
	motorPos    = my_drive.axis0.encoder.pos_estimate
	outputPos   = 0#my_drive.axis1.encoder.pos_estimate # Encoder not available, and not really needed
	motorVel    = my_drive.axis0.encoder.vel_estimate
	outputVel   = 0#my_drive.axis1.encoder.vel_estimate # Encoder not available, and not really needed
	Iq_set		= my_drive.axis0.motor.current_control.Iq_setpoint
	Iq_meas		= my_drive.axis0.motor.current_control.Iq_measured
	data.append([
		t-t0,
		motorPos,
		outputPos,
		motorVel,
		outputVel,
		my_drive.axis0.controller.input_pos,
		Iq_set,
		Iq_meas,
	])

	# Check if we have reached the setpoint
	# Compute error in [rad], at the output
	if not motion_setpoint_reached:
		e = turns2rad * (my_drive.axis0.controller.input_pos - motorPos) / n
		if math.fabs(e) <= reached_margin:
			t_motion_setpoint_reached = time.monotonic()
			motion_setpoint_reached = True # Don't keep updating the reached time
			print('Setpoint reached.')

	# If there has been more time than `static_time` since we reached the setpoint, let's go
	if motion_setpoint_reached and time.monotonic() - t_motion_setpoint_reached >= static_time:
		# Check whether we have reached all set points, and then the starting position
		if motion_it >= len(target_pos)+1:
			# If we have reached the starting position, we can stop
			break
		elif motion_it >= len(target_pos):
			# If we have reached all target positions, we go to the starting position
			target_pos_motor = input_pos_initial
		else:
			# Otherwise we go to the next target position
			target_pos_motor = input_pos_initial + n * target_pos[motion_it] / turns2rad # Motor pos in [turns]

		print('Updating set point to', round(target_pos_motor*turns2rad/n,2), 'rad..')

		# Set new set point
		my_drive.axis0.controller.input_pos = target_pos_motor

		# Update iterator for setpoints so we set the next one next time
		motion_it += 1

		# We have not reached the new setpoint yet
		motion_setpoint_reached = False

	# Sleep for loop
	try:
		time.sleep(sample_time - (time.monotonic()-t))
	except:
		print('    WARNING: Iteration took longer than sample time of', sample_time, 's')

# Stop motor
# TODO CAN WE REMOVE THIS?
print('Finished, again setting initial position (should be no move, in which case remove this)..')
my_drive.axis0.controller.input_pos = input_pos_initial
time.sleep(1)

# Stop controller
print('Stopping closed-loop control..')
my_drive.axis0.requested_state = AXIS_STATE_IDLE

# Save data
save_data(data, out_filename)

# End
print('Done.')
