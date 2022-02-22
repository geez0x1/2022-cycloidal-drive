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
import curses
from common import get_output_filename, save_data, append_filename

def main(stdscr):
	# Test parameters
	sample_time		= 0.02	# Sample time [s] # Limited by USB/UART?
	n				= 11	# Gear ratio []
	deg_per_step	= 1		# Degrees to increment per step (on the output)
	turns2deg		= 360	# One turn in degrees []
	deg2rad			= math.pi/180 # Degrees to radians []
	turns2rad		= turns2deg*deg2rad # Turns to rad []

	# Control gains
	# Increase the following control gains as appropriate to get
	# very stiff position control for this test.
	pos_gain		= 10*20		# [turns/s / turn] - default 20
	vel_gain		= 10*0.16	# [Nm / turns/s] - default 0.16
	vel_integrator_gain = 10*0.32	# [Nm / turn/s^2] - default 0.32
	current_lim			= 60		# Current limit [A] (40A tops out at around 31 Nm)
	requested_current	= 80		# Current sensing limit [A]
	# Not touching current control gains current_gain and
	# current_integrator_gain - they are automatically set
	# for a given current control bandwidth.

	# Prompt user for output filename
	out_filename = get_output_filename('out_experiment_3_stiffness.csv', stdscr)
	out_filename_stiffness = append_filename(out_filename, '_stiffness')

	# curses: disable echo
	curses.noecho()
	# curses: make getch() non-blocking
	stdscr.nodelay(1)
	line = 0

	# Find a connected ODrive (this will block until you connect one)
	stdscr.addstr(line,0,'Finding odrive..'); line += 1
	my_drive = odrive.find_any()

	# First set the reference position to whatever the current position is
	input_pos = my_drive.axis0.encoder.pos_estimate
	my_drive.axis0.controller.input_pos = input_pos
	input_pos_initial = input_pos
	stdscr.addstr(line,0,'Initial position offset is '+str(round(turns2deg*input_pos_initial/n,2))+' deg.'); line += 1
	stdscr.addstr(line,0,'NOTE: Make sure to start this experiment in a down hanging position!'); line += 1

	# Set control mode
	# Position control
	stdscr.addstr(line,0,'Setting control mode and parameters..'); line += 1
	my_drive.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
	my_drive.axis0.controller.config.input_mode = INPUT_MODE_POS_FILTER
	my_drive.axis0.controller.config.input_filter_bandwidth = 5.0 # [Hz]

	# Set control gains
	my_drive.axis0.controller.config.pos_gain = pos_gain
	my_drive.axis0.controller.config.vel_gain = vel_gain
	my_drive.axis0.controller.config.vel_integrator_gain = vel_integrator_gain
	
	# Set current limit
	my_drive.axis0.motor.config.current_lim = current_lim
	my_drive.axis0.motor.config.requested_current_range = requested_current

	# Compute turns to increment per step [] (on the output)
	turns_per_step	= deg_per_step/turns2deg

	# Then start position control
	stdscr.addstr(line,0,'Starting closed loop control..'); line += 1
	my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
	time.sleep(1.0)

	# Record data
	stdscr.addstr(line,0,'Recording data..'); line += 1
	data = []
	data_stiffness = []
	t0 = time.monotonic()

	# Loop until told to stop
	line += 1; stdscr.addstr(line,0,'Use +/- to increase/decrease position, \'s\' to sample the current deflection, or press \'q\' to stop.'); line += 1; line += 1
	outputPos_deg = round(turns2deg*(input_pos-input_pos_initial)/n,2)
	stdscr.addstr(line,4,'Current output position: '+str(outputPos_deg)+' deg..')
	while True:
		# Get iteration start time
		t = time.monotonic()
		
		# curses: get keyboard input, returns -1 if none available
		c = stdscr.getch()

		if c == ord('+') or c == ord('='):
			input_pos += turns_per_step * n
			my_drive.axis0.controller.input_pos = input_pos

			outputPos_deg = round(turns2deg*(input_pos-input_pos_initial)/n,2)
			stdscr.addstr(line,4,'Current output position: '+str(outputPos_deg)+' deg..')

		elif c == ord('-'):
			input_pos -= turns_per_step * n
			my_drive.axis0.controller.input_pos = input_pos

			outputPos_deg = round(turns2deg*(input_pos-input_pos_initial)/n,2)
			stdscr.addstr(line,4,'Current output position: '+str(outputPos_deg)+' deg..')

		elif c == ord('q'):
			line += 1
			break

		# Get data
		motorPos    = my_drive.axis0.encoder.pos_estimate
		outputPos   = my_drive.axis1.encoder.pos_estimate
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
			input_pos,
			Iq_set,
			Iq_meas,
		])
		
		# Check if we need to save a specific sample for stiffness
		# We just copy the last normal sample for now.
		# It would be better to filter the sensors, but positions should be fine.
		if c == ord('s'):
			data_stiffness.append(data[-1])
			stdscr.addstr(line+1,4,'Saved '+str(len(data_stiffness))+' stiffness samples.')
		
		# Sleep for loop
		try:
			time.sleep(sample_time - (time.monotonic()-t))
		except:
			print('    WARNING: Iteration took longer than sample time of', sample_time, 's')

	# Stop controller
	line += 1; stdscr.addstr(line,0,'Finished, stopping closed-loop control..'); line += 1
	my_drive.axis0.requested_state = AXIS_STATE_IDLE
	c = stdscr.getch()
	time.sleep(1)

	# Stop curses window
	curses.endwin()

	# Save data - both for regular data and stiffness points
	save_data(data, out_filename)
	save_data(data_stiffness, out_filename_stiffness)

	# End
	print('Done.')


# Run
if __name__ == '__main__':
	curses.wrapper(main)
