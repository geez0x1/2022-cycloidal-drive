#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Dec  8 14:24:59 2021

@author: wesley
"""

import curses
import csv
import os
import numpy as np
from types import SimpleNamespace
from string import Template


# Prompt the user for the filename to load data from
def get_filename(default='', stdscr=None):
	# For curses
	if stdscr is not None:
		curses.echo()
		stdscr.clear()
		stdscr.addstr(0,0,'Filename load data from ['+default+']: ');
		fname = stdscr.getstr(30).decode()
		stdscr.clear()
	else:
		fname = input('Filename to load data from ['+default+']: ')

	if fname == '':
		return default
	else:
		return fname


# Define filtering function for 1D arrays
def filter_iir(data, alpha):
	out			= np.zeros(data.shape)
	out[0]		= data[0] # Set first item

	for i in range(1, data.shape[0]):
		out[i] = alpha * out[i-1] + (1-alpha) * data[i]

	return out


# Curve fitting function for variation in gear ratio
def fit_sines(outputPos, a1, a2, b1, b2):
	return	a1 * np.cos(outputPos * 2 * np.pi + b1) + \
			a2 * np.cos(2 * outputPos * 2 * np.pi + b2)


# Curve fitting function for linear fit
def fit_linear(tau, a, b):
	return a * tau + b


# Curve fitting function for linear fit - no offset
def fit_linear_no_offset(tau, a):
	return a * tau


# Fitting Coulomb + viscous friction
def fit_coulomb_viscous(outputVel, d_c, d_v):
	return d_c * np.sign(outputVel) + d_v * outputVel

# Fitting Coulomb friction, with difference in direction
def fit_coulomb_bidirectional(outputVel, d_c, d_c_n, d_v, d_v_n):
	tau =	np.heaviside(outputVel, 1) * d_c + \
			np.heaviside(-outputVel, 0) * -d_c_n
	return tau

# Fitting Coulomb + viscous friction, with difference in direction
def fit_coulomb_viscous_bidirectional(outputVel, d_c, d_c_n, d_v, d_v_n):
	tau =	np.heaviside(outputVel, 1) * (d_c + d_v * outputVel) + \
			np.heaviside(-outputVel, 0) * (-d_c_n + d_v_n * outputVel)
	return tau

# Fitting Coulomb + viscous friction, with difference in direction
def fit_coulomb_viscous_bidirectional_tanh(outputVel, d_c, d_c_n, d_v, d_v_n):
	tau =	np.tanh(100*outputVel) * ( \
				np.heaviside(outputVel, 1) * (d_c + d_v * outputVel) + \
				np.heaviside(-outputVel, 0) * -(-d_c_n + d_v_n * outputVel) \
			)
	return tau

# Curve fitting for piecewise linear
def fit_piecewise_linear_no_offset(tau, a1, a2, b):
	return \
	np.where(tau < b, 1, 0) * (a1 * tau) + \
	np.where(tau >= b, 1, 0) * (a1 * b + a2 * (tau-b))


# Plotting colours
plot_colours = SimpleNamespace(**{
	'reference':		(0,0,0),
	'motorPos':			(0.8,0.4,0.4),
	'motorPos_light':	(0.8,0.4,0.4,0.2),
	'outputPos':		(0.2,0.4,1.0),
	'outputPos_light':	(0.2,0.4,1.0,0.2),
	'rel_pos':			(0.3,0.7,0.3),
	'rel_pos_light':	(0.3,0.7,0.3,0.2),
})


# Append string to filename
def append_filename(filename, appendix):
	parts = os.path.splitext(filename)
	return parts[0] + appendix + parts[1]


# Save data to csv file and create tikzpicture template
# We can also use tikzplotlib for this, see:
# https://github.com/nschloe/tikzplotlib
def export_data(data, headers, filename_base='out'):
	if type(data) is not tuple or type(data[0]) is not np.ndarray:
		raise Exception('Data should be a tuple of numpy arrays')
	if type(headers) is not list or type(headers[0]) is not str:
		raise Exception('Headers should be a list of strings')
	if len(data) != len(headers):
		raise Exception('Number of data sets and headers should be equal')

	# Create 2D array of data
	out_data = np.column_stack(data)

	# Save data to csv file for pgfplots
	filename_csv = filename_base + '.csv'
	np.savetxt(filename_csv, out_data, delimiter=",", header=",".join(headers), comments='')

	# Create tikzpicture template
	with open('tikzpicture_template.tex', 'r') as f:
		t = f.read()
	s = Template(t)
	s = s.substitute(filename=filename_csv)

	# Write template to .tex file
	filename_tex = filename_base + '.tex'
	with open(filename_tex, 'w') as f:
		f.write(s)


# Reduce data points for plotting
def reduce_data(t, dt, data):
	if type(data) is not tuple and not (type(data) is np.ndarray and data.ndims==1):
		raise Exception('Data should be tuple or 1-D np.ndarray')

	print('Creating', int(np.ceil(np.max(t)/dt)), 'data points..')

	# Create new time vector
	n			= int(np.ceil((np.max(t)-np.min(t)) / dt))
	t_RS		= np.linspace(np.min(t), np.max(t), num=n, endpoint=True)

	# Resample data
	if type(data) is tuple:
		data_RS = []
		for d in data:
			d_RS = np.interp(t_RS, t, d)
			data_RS.append(d_RS)
	else:
		data_RS = np.interp(t_RS, t, data)

	return (t_RS, tuple(data_RS))

# Reduce data points by using only every n points
# A better approach would be scipy.interpolate()
def reduce_data_by_skip(data, n):
	# Get total number of data points
	N = data[0].shape[0]

	# Create indices
	idx = np.arange(0, N-1, n)
	if idx[-1] != N-1:
		idx = np.append(idx,N-1)

	# Resample data
	if type(data) is tuple:
		data_RS = []
		for d in data:
			d_RS = d[idx]
			data_RS.append(d_RS)
	else:
		raise Exception('Non-tuple not supported')

	return tuple(data_RS)