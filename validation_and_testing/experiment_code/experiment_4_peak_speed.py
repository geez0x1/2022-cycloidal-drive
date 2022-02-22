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
	turns2deg		= 360	# One turn in degrees []
	deg2rad			= math.pi/180 # Degrees to radians []
	turns2rad		= turns2deg*deg2rad # Turns to rad []
	vel_increments	= 0.5/turns2rad	# Speed increments (at output) [turns/s]
	max_accel		= 5/turns2rad	# Maximum acceleration (at output) [turns/s^2]

	# Control gains
	# We only care about velocity control here.
	vel_gain			= 5*0.16	# [Nm / turns/s] - default 0.16
	vel_integrator_gain = 5*0.32	# [Nm / turn/s^2] - default 0.32
	# Not touching current control gains current_gain and
	# current_integrator_gain - they are automatically set
	# for a given current control bandwidth.

	# Prompt user for output filename
	out_filename = get_output_filename('out_experiment_4_peak_speed.csv', stdscr)

	# curses: disable echo
	curses.noecho()
	# curses: make getch() non-blocking
	stdscr.nodelay(1)
	line = 0

	# Find a connected ODrive (this will block until you connect one)
	stdscr.addstr(line,0,'Finding odrive..'); line += 1
	my_drive = odrive.find_any()

	# Make sure the reference velocity starts at zero
	input_vel = 0
	my_drive.axis0.controller.input_vel = input_vel

	# Set control mode
	# Position control
	stdscr.addstr(line,0,'Setting control mode and parameters..'); line += 1
	my_drive.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
	my_drive.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
	my_drive.axis0.controller.config.vel_ramp_rate = n * max_accel

	# Set control gains
	my_drive.axis0.controller.config.vel_gain = vel_gain
	my_drive.axis0.controller.config.vel_integrator_gain = vel_integrator_gain

	# Then start position control
	stdscr.addstr(line,0,'Starting closed loop control..'); line += 1
	my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
	time.sleep(1)

	# Record data
	stdscr.addstr(line,0,'Recording data..'); line += 1
	data = []
	t0 = time.monotonic()

	# Loop until told to stop
	line += 1; stdscr.addstr(line,0,'Use +/- to increase/decrease output speed, or press \'q\' to stop.'); line += 1; line += 1
	outputVel_rads = round(turns2deg*deg2rad*input_vel/n,2)
	stdscr.addstr(line,4,'Current output speed: '+str(outputVel_rads)+' rad/s..')
	stopping = False
	stopping_samples_count = 0
	stopping_samples_max_count = 2.0/sample_time # 2s at 100Hz # TODO when ODrive crashes this stops us from saving the data
	while True:
		# Get iteration start time
		t = time.monotonic()
		
		# curses: get keyboard input, returns -1 if none available
		c = stdscr.getch()

		# If we're supposed to be stopping, wait until we have at least x samples
		# at low speed before stopping the measurement.
		if stopping:
			if abs(my_drive.axis0.encoder.vel_estimate) < 0.1: # [turns/s]
				stopping_samples_count += 1
			if stopping_samples_count >= stopping_samples_max_count:
				break
		else:
			if c == ord('+') or c == ord('='):
				input_vel += vel_increments * n
				my_drive.axis0.controller.input_vel = input_vel

				outputVel_rads = round(turns2deg*deg2rad*input_vel/n,2)
				stdscr.addstr(line,4,'Set output speed: '+str(outputVel_rads)+' rad/s..')

			elif c == ord('-'):
				input_vel -= vel_increments * n
				my_drive.axis0.controller.input_vel = input_vel

				outputVel_rads = round(turns2deg*deg2rad*input_vel/n,2)
				stdscr.addstr(line,4,'Set output speed: '+str(outputVel_rads)+' rad/s..')

			elif c == ord('q'):
				input_vel = 0
				my_drive.axis0.controller.input_vel = input_vel

				line += 2; stdscr.addstr(line,0,'Stopping motor..'); line += 1
				stopping = True

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
		
		# Sleep for loop
		try:
				time.sleep(sample_time - (time.monotonic()-t))
		except:
			print('    WARNING: Iteration took longer than sample time of', sample_time, 's')

	# Stop controller
	stdscr.addstr(line,0,'Finished, stopping closed-loop control..'); line += 1
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
