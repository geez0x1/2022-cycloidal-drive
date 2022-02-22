#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Dec  8 14:24:59 2021

@author: wesley
"""

import curses
import csv
import os

# Prompt the user for the filename to save data to
def get_output_filename(default='', stdscr=None):
	# For curses
	if stdscr is not None:
		curses.echo()
		stdscr.clear()
		stdscr.addstr(0,0,'Filename to save data to ['+default+']: ');
		fname = stdscr.getstr(30).decode()
		stdscr.clear()
	else:
		fname = input('Filename to save data to ['+default+']: ')

	if fname == '':
		return default
	else:
		return fname


# Save data
def save_data(data, out_filename):
	if len(data) > 0:
		print('Saving data to', out_filename, '..')

		with open(out_filename, "w", newline='') as csv_file:
			writer = csv.writer(csv_file, delimiter=',', quoting=csv.QUOTE_NONNUMERIC)
			writer.writerows(data)
	else:
		print('Warning: Data is empty, nothing written.')
		
# Append string to filename
def append_filename(filename, appendix):
	parts = os.path.splitext(filename)
	return parts[0] + appendix + parts[1]

# Clip a value
def clip(x, x_min, x_max):
	if x_min > x_max:
		raise Exception('Minimum must be smaller than maximum')

	if x < x_min:
		return x_min
	elif x > x_max:
		return x_max
	else:
		return x
