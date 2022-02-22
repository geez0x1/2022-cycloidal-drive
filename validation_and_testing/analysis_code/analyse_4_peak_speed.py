import sys
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
from common import get_filename, filter_iir, fit_sines, export_data, reduce_data, fit_coulomb_viscous, fit_coulomb_viscous_bidirectional
from common import plot_colours as c

# Parameters
n			= 11	# True gear ratio
k_t			= 0.082699 # Motor torque constant [Nm/A]
alpha		= 0.8	# Filtering constant for first-order IIR filter
dt			= 0.017	# Timestep [s] # Based on what differentiation of
					# positions gives us compared to the velocity signals
turns2deg	= 360	# One turn in [deg]
deg2rad		= math.pi/180 # Degree to rad
turns2rad	= turns2deg*deg2rad # Turns to rad
export		= False	# Whether to export data for pgfplots

# Load data
filename = get_filename('../experiment_code/out_experiment_4_peak_speed.csv')
data = np.genfromtxt(filename, delimiter=",")

# Get fields
t			= data[:,0]
motorPos	= data[:,1]
outputPos	= data[:,2]
motorVel	= data[:,3]
outputVel	= data[:,4]
input_vel	= data[:,5]
Iq_set		= data[:,6]
Iq_meas		= data[:,7]

# Get indices for data for fitting

# Default
idx = np.where(t)[0]

# Filter data
motorPos_filt	= filter_iir(motorPos, alpha)
outputPos_filt	= filter_iir(outputPos, alpha)
motorVel_filt	= filter_iir(motorVel, alpha)
outputVel_filt	= filter_iir(outputVel, alpha)
Iq_set_filt		= filter_iir(Iq_set, alpha)
Iq_meas_filt	= filter_iir(Iq_meas, alpha)

# Estimate motor torque
# tau_m = k_t * n * Iq_meas
# tau_m_filt = k_t * n * Iq_meas_filt
# tau_m_filt2 = filter_iir(tau_m, 0.95) # Heavy filtering

# Export data for pgfplots
if export:
	data = (
		t,
		turns2rad*input_vel/n,
		turns2rad*outputVel,
		turns2rad*outputVel_filt,
		Iq_meas,
		Iq_meas_filt,
		Iq_set
	)

	# Reduce data
	#dt = 0.01
	#(t_RS, data_RS) = reduce_data(t, dt, data)

	export_data(
		data,
		headers = ['t', 'input_vel', 'outputVel', 'outputVel_filt', 'Iq_meas', 'Iq_meas_filt', 'Iq_set'],
		filename_base = 'exp4'
	)

# Plotting

# Figure and axes
(fig, ax) = plt.subplots(1,3)
fig.suptitle('Experiment 4 (peak speed): ' + filename)

# Motor speed
ax[0].plot(t, turns2rad*input_vel, '--', color=c.reference, label='Reference')
ax[0].plot(t, turns2rad*motorVel, color=c.motorPos_light)
ax[0].plot(t, turns2rad*motorVel_filt, color=c.motorPos, label='Motor speed')
ax[0].set_title('Motor speed')
ax[0].set_xlabel('Time [s]')
ax[0].set_ylabel('Speed [rad/s]')
ax[0].legend()

# Output speed
ax[1].plot(t, turns2rad*input_vel/n, '--', color=c.reference, label='Reference')
ax[1].plot(t, turns2rad*outputVel, color=c.outputPos_light)
ax[1].plot(t, turns2rad*outputVel_filt, color=c.outputPos, label='Output speed')
ax[1].set_title('Output speed')
ax[1].set_xlabel('Time [s]')
ax[1].set_ylabel('Speed [rad/s]')
ax[1].legend()

# Current
ax[2].plot(t, Iq_meas, color=(0.8,0.8,0.8), label='Iq_meas')
ax[2].plot(t, Iq_meas_filt, color=(0.4,0.4,0.4), label='Iq_meas_filt')
ax[2].plot(t, Iq_set, '--', color=(0,0,0), label='Iq_set')
ax[2].set_title('Current')
ax[2].set_xlabel('Time [s]')
ax[2].set_ylabel('Current [A]')
ax[2].legend()

plt.show()
