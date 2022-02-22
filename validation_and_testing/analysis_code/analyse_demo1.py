import sys
import os
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
from common import get_filename, filter_iir, fit_linear, fit_linear_no_offset, append_filename, export_data, reduce_data
from common import plot_colours as c

# Parameters
n			= 11	# True gear ratio
k_t			= 0.082699 # Motor torque constant [Nm/A]
alpha		= 0.5	# Filtering constant for first-order IIR filter
dt			= 0.017	# Timestep [s] # Based on what differentiation of
					# positions gives us compared to the velocity signals
turns2deg	= 360	# One turn in [deg]
deg2rad		= math.pi/180 # Degree to rad
turns2rad	= turns2deg*deg2rad # Turns to rad
export		= False	# Whether to export data for pgfplots

# Load data
filename	= get_filename('../experiment_code/out_demo1.csv')
data		= np.genfromtxt(filename, delimiter=",")

# Get fields
t			= data[:,0]
motorPos	= data[:,1]
outputPos	= data[:,2]
motorVel	= data[:,3]
outputVel	= data[:,4]
input_pos	= data[:,5]
Iq_set		= data[:,6]
Iq_meas		= data[:,7]

# Filter data
motorVel_filt	= filter_iir(motorVel, alpha)
Iq_meas_filt	= filter_iir(Iq_meas, alpha)

# Set the initial positions to zero
motorPos	= motorPos - motorPos[0]
input_pos	= input_pos - input_pos[0]

# Compute torque reference and measurement
tau_m_set	= Iq_set * k_t * n
tau_m		= Iq_meas_filt * k_t * n

# Export data for pgfplots
if export:
	data = (
		t,
		turns2rad*input_pos/n,
		turns2rad*motorPos/n,
		turns2rad*motorVel/n,
		tau_m_set,
		tau_m,
		Iq_set,
		Iq_meas,
		Iq_meas_filt,
	)

	# Reduce data
	#dt = 0.1
	#(t_RS, data_RS) = reduce_data(t, dt, data)

	export_data(
		data,
		headers = ['t', 'input_pos_reduced', 'motorPos_reduced', 'motorVel_reduced', 'tau_m_set', 'tau_m', 'Iq_set', 'Iq_meas', 'Iq_meas_filt'],
		filename_base = 'demo1'
	)

# Plotting

# Figure and axes
(fig, ax) = plt.subplots(1,3)
fig.suptitle('Demo 1 (p2p position control): ' + filename)

# Motor and output position, and reference, over time
ax[0].plot(t, turns2rad*input_pos/n, '--', color=c.reference, label='input_pos/n')
ax[0].plot(t, turns2rad*motorPos/n, color=c.motorPos, label='motorPos/n')
ax[0].plot(t, turns2rad*motorVel/n, color=c.outputPos, label='motorVel_reduced')
ax[0].set_title('Reference and measured position')
ax[0].set_xlabel('Time [s]')
ax[0].set_ylabel('Position [rad] and velocity [rad/s]')
ax[0].legend()
ax[0].grid()

# Current
ax[1].plot(t, Iq_meas, color=(0.8,0.8,0.8), label='Iq_meas')
ax[1].plot(t, Iq_meas_filt, color=(0.4,0.4,0.4), label='Iq_meas_filt')
ax[1].plot(t, Iq_set, '--', color=(0,0,0), label='Iq_set')
ax[1].set_title('Current')
ax[1].set_xlabel('Time [s]')
ax[1].set_ylabel('Current [A]')
ax[1].legend()
ax[1].grid()

# Torque
ax[2].plot(t, tau_m, color=(0.4,0.4,0.4), label='Torque')
ax[2].plot(t, tau_m_set, color=(0,0,0), label='Reference')
ax[2].set_title('Torque')
ax[2].set_xlabel('Time [s]')
ax[2].set_ylabel('Torque [Nm]')
ax[2].legend()
ax[2].grid()

plt.show()
