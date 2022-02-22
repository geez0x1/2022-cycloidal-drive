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
import sys
from common import get_output_filename, save_data, clip

# Test parameters
sample_time		= 0.02	# Sample time [s] # Limited by USB/UART?
n				= 11	# True gear ratio
min_speed		= 0.5	# Minimum speed (at output) [rad/s]
target_speed	= 10.0	# Target speed (at output) [rad/s] (default, overwrite via command line argument)
duration		= 120	# Data recording duration, for each direction [s]
turns2deg		= 360	# One turn in [deg]
deg2rad			= math.pi/180 # Degrees to radians
turns2rad		= turns2deg*deg2rad # Turns to rad []

# Check for command-line arguments
if len(sys.argv) >= 2:
	arg_target_speed = float(sys.argv[1])
	if arg_target_speed != 0:
		print('Setting target speed of', arg_target_speed, 'rad/s..')
		target_speed = arg_target_speed
	else:
		raise Exception('Invalid target speed supplied by argument.')
else:
	print('Setting default target speed of', target_speed, 'rad/s..')

# Compute ramping rate from min, max, and duration
ramp_rate		= 2 * (target_speed-min_speed) / duration # [rad/s^2]

# Compute speeds in [turns/s] and ramp rate in [turns/s^2] for motor
min_speed_motor		= n * min_speed / turns2rad
target_speed_motor	= n * target_speed / turns2rad
ramp_rate_motor		= n * ramp_rate / turns2rad

# Control gains
# Position gains are not relevant for this experiment.
# Increase vel gain to get constant motor speed under varying load
vel_gain			= 0.16	# [Nm / turns/s] - default 0.16
vel_integrator_gain	= 0.32	# [Nm / turn/s^2] - default 0.32
# Not touching current control gains current_gain and
# current_integrator_gain - they are automatically set
# for a given current control bandwidth.

# Prompt user for output filename
out_filename = get_output_filename('out_experiment_5_friction.csv')

# Find a connected ODrive (this will block until you connect one)
print("Finding odrive..")
my_drive = odrive.find_any()

# Set control mode
# Velocity control, no ramping
print('Setting control mode and parameters..')
my_drive.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

# Set control gains
my_drive.axis0.controller.config.vel_gain = vel_gain
my_drive.axis0.controller.config.vel_integrator_gain = vel_integrator_gain

print('Starting closed loop control..')
my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

# Start, wait for acceleration
print('Starting motor at minimum speed', round(min_speed_motor,2), 'turns/s (', min_speed, 'rad/s at output)..')
input_vel = min_speed_motor
my_drive.axis0.controller.input_vel = input_vel
time.sleep(5)

# Record data
print('Recording data for', duration, 'seconds in positive direction..')
data = []
t0 = time.monotonic()
t_prev = t0

# Ramping direction
ramp = True

# Loop while duration has not passed
while time.monotonic()-t0 < duration:
	# Get iteration start time
	t = time.monotonic()

	# Get data
	motorPos    = 0#my_drive.axis0.encoder.pos_estimate
	outputPos   = 0#my_drive.axis1.encoder.pos_estimate
	motorVel    = my_drive.axis0.encoder.vel_estimate
	outputVel   = my_drive.axis1.encoder.vel_estimate
	Iq_set		= my_drive.axis0.motor.current_control.Iq_setpoint
	Iq_meas		= my_drive.axis0.motor.current_control.Iq_measured
	data.append([
		t-t0,
		motorPos,
		outputPos,
		motorVel,
		outputVel,
		input_vel,
		Iq_set,
		Iq_meas,
	])

	# Increment/decrement reference velocity
	# We start with ramp=True, which means we are increasing velocity
	# After half duration, ramp=False, we start ramping down from top speed
	if ramp:
		input_vel += (t-t_prev)*ramp_rate_motor
	else:
		input_vel -= (t-t_prev)*ramp_rate_motor

	# Set motor speed, but limit for safety
	input_vel = clip(input_vel, min_speed_motor, target_speed_motor)
	print('Setting output velocity of', round(input_vel*turns2rad/n,2), 'rad/s..')
	my_drive.axis0.controller.input_vel = input_vel

	# Halfway duration we start ramping down
	if t-t0 >= duration/2:
		ramp = False
	
	# Save this iteration time to accurately compute velocity increments
	t_prev = t

	# Sleep for loop
	try:
		time.sleep(sample_time - (time.monotonic()-t))
	except:
		print('    WARNING: Iteration took longer than sample time of', sample_time, 's')

print('Finished positive direction. Starting motor at minimum speed in negative direction..')
my_drive.axis0.controller.input_vel = -min_speed_motor
time.sleep(5)

# Reset ramping direction
ramp = True

# Loop while duration has not passed
# Take a new time sample for the start of this duration that we use for timing (not for data out)
t1 = time.monotonic()
t_prev = t1
while time.monotonic()-t1 < duration:
	# Get iteration start time
	t = time.monotonic()

	# Get data
	motorPos    = 0#my_drive.axis0.encoder.pos_estimate
	outputPos   = 0#my_drive.axis1.encoder.pos_estimate
	motorVel    = my_drive.axis0.encoder.vel_estimate
	outputVel   = my_drive.axis1.encoder.vel_estimate
	Iq_set		= my_drive.axis0.motor.current_control.Iq_setpoint
	Iq_meas		= my_drive.axis0.motor.current_control.Iq_measured
	data.append([
		t-t0,
		motorPos,
		outputPos,
		motorVel,
		outputVel,
		input_vel,
		Iq_set,
		Iq_meas,
	])

	# Increment/decrement reference velocity
	# We start with ramp=True, which means we are increasing velocity (in negative direction here)
	# After half duration, ramp=False, we start ramping down from top speed
	if ramp:
		input_vel -= (t-t_prev)*ramp_rate_motor
	else:
		input_vel += (t-t_prev)*ramp_rate_motor

	# Set motor speed, but limit for safety
	input_vel = clip(input_vel, -target_speed_motor, -min_speed_motor)
	print('Setting output velocity of', round(input_vel*turns2rad/n,2), 'rad/s..')
	my_drive.axis0.controller.input_vel = input_vel

	# Halfway duration we start ramping down
	if t-t1 >= duration/2:
		ramp = False
		
	# Save this iteration time to accurately compute velocity increments
	t_prev = t

	# Sleep for loop
	try:
		time.sleep(sample_time - (time.monotonic()-t))
	except:
		print('    WARNING: Iteration took longer than sample time of', sample_time, 's')

# Stop motor
print('Finished, stopping motor..')
my_drive.axis0.controller.input_vel = 0
time.sleep(2)

# Stop controller
print('Stopping closed-loop control..')
my_drive.axis0.requested_state = AXIS_STATE_IDLE

# Save data
save_data(data, out_filename)

# End
print('Done.')
