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
from common import get_output_filename, save_data

def main(stdscr):
	# Test parameters
	sample_time		= 0.01	# Sample time [s] # Limited by USB/UART?
	n				= 11	# Gear ratio []
	turns_per_step	= 1/8	# Turns to increment per step [] (on the output)
	turns2deg		= 360	# One turn in degrees

	# Control gains
	# Increase the following control gains as appropriate to get
	# very stiff position control for this test.
	pos_gain		= 5*20		# [turns/s / turn] - default 20
	vel_gain		= 5*0.16	# [Nm / turns/s] - default 0.16
	vel_integrator_gain = 0.32	# [Nm / turn/s^2] - default 0.32
	# Not touching current control gains current_gain and
	# current_integrator_gain - they are automatically set
	# for a given current control bandwidth.

	# Prompt user for output filename
	out_filename = get_output_filename('out_experiment_2_play.csv', stdscr)

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

	# Set control mode
	# Position control
	stdscr.addstr(line,0,'Setting control mode and parameters..'); line += 1
	my_drive.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
	my_drive.axis0.controller.config.input_mode = INPUT_MODE_POS_FILTER
	my_drive.axis0.controller.config.input_filter_bandwidth = 3.0 # [Hz]

	# Set control gains
	my_drive.axis0.controller.config.pos_gain = pos_gain
	my_drive.axis0.controller.config.vel_gain = vel_gain
	my_drive.axis0.controller.config.vel_integrator_gain = vel_integrator_gain

	# Then start position control
	stdscr.addstr(line,0,'Starting closed loop control..'); line += 1
	my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
	time.sleep(1.0)

	# Record data
	stdscr.addstr(line,0,'Recording data..'); line += 1
	data = []
	t0 = time.monotonic()

	# Keep track of how often we've incremented the position,
	# so that we stop after one output turn
	increments		= 0
	max_increments	= ((2*180/turns2deg)/turns_per_step) - 1

	# Loop until told to stop
	line += 1; stdscr.addstr(line,0,'Press \'p\' to continue to next position, or \'q\' to stop immediately.'); line += 1; line += 1
	while increments <= max_increments:
		# Get iteration start time
		t = time.monotonic()
		
		# curses: get keyboard input, returns -1 if none available
		c = stdscr.getch()

		if c == ord('p'):
			increments += 1
			if increments <= max_increments:
				stdscr.addstr(line,4,'Incrementing output position by '+str(turns_per_step)+' turns (increment '+str(increments)+'/'+str(round(max_increments))+')..'); line += 1
				input_pos += turns_per_step * n
				my_drive.axis0.controller.input_pos = input_pos

		elif c == ord('q'):
			break

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
			outputVel,
			input_pos
		])
		
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

	# Save data
	save_data(data, out_filename)

	# End
	print('Done.')


# Run
if __name__ == '__main__':
	curses.wrapper(main)
