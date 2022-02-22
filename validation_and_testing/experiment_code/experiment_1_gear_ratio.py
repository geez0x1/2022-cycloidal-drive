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
from common import get_output_filename, save_data

# Test parameters
sample_time		= 0.01	# Sample time [s] # Limited by USB/UART?
n				= 11	# True gear ratio
target_speed	= 2.0	# Target motor speed (at output) [rad/s] (default, overwrite via command line argument)
duration		= 20	# Data recording duration [s]
max_accel		= 5		# Maximum acceleration [turns/s^2]
turns2deg		= 360	# One turn in [deg]
deg2rad			= math.pi/180 # Degrees to radians

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

# Compute target speed in turns/s for motor
target_speed_motor = n * target_speed / (turns2deg * deg2rad)

# Control gains
# Position gains are not relevant for this experiment.
# Increase vel gain to get constant motor speed under varying load
vel_gain			= 5*0.16	# [Nm / turns/s] - default 0.16
vel_integrator_gain	= 5*0.32	# [Nm / turn/s^2] - default 0.32
# Not touching current control gains current_gain and
# current_integrator_gain - they are automatically set
# for a given current control bandwidth.

# Prompt user for output filename
out_filename = get_output_filename('out_experiment_1_gear_ratio_' + str(target_speed) + '_rads.csv')

# Find a connected ODrive (this will block until you connect one)
print("Finding odrive..")
my_drive = odrive.find_any()

# Set control mode
# Velocity control, with ramp to avoid excessive acceleration
print('Setting control mode and parameters..')
my_drive.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
my_drive.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
my_drive.axis0.controller.config.vel_ramp_rate = max_accel

# Set control gains
my_drive.axis0.controller.config.vel_gain = vel_gain
my_drive.axis0.controller.config.vel_integrator_gain = vel_integrator_gain

# Compute wait time for acceleration
acceleration_wait_time = 2 + abs(target_speed_motor) / my_drive.axis0.controller.config.vel_ramp_rate

print('Starting closed loop control..')
my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

# Start, wait for acceleration
print('Starting motor at', target_speed_motor, 'turns/s (', target_speed, 'rad/s at output)..')
my_drive.axis0.controller.input_vel = target_speed_motor
time.sleep(acceleration_wait_time)

# Record data
print('Recording data for', duration, 'seconds..')
data = []
t0 = time.monotonic()

# Loop while duration has not passed
while time.monotonic()-t0 < duration:
	# Get iteration start time
	t = time.monotonic()
	
	# Get data
	motorPos    = my_drive.axis0.encoder.pos_estimate
	outputPos   = my_drive.axis1.encoder.pos_estimate
	motorVel    = my_drive.axis0.encoder.vel_estimate
	outputVel   = my_drive.axis1.encoder.vel_estimate
	data.append([
		t-t0,
		motorPos,
		outputPos,
		motorVel,
		outputVel
	])
	
	try:
		time.sleep(sample_time - (time.monotonic()-t))
	except:
		print('    WARNING: Iteration took longer than sample time of', sample_time, 's')

# Stop motor
print('Finished, stopping motor..')
my_drive.axis0.controller.input_vel = 0
time.sleep(acceleration_wait_time)

# Stop controller
print('Stopping closed-loop control..')
my_drive.axis0.requested_state = AXIS_STATE_IDLE

# Save data
save_data(data, out_filename)

# End
print('Done.')
