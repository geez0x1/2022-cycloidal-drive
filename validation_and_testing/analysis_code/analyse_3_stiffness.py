import sys
import os
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
from common import get_filename, filter_iir, fit_linear, fit_linear_no_offset, append_filename, export_data, reduce_data, fit_piecewise_linear_no_offset
from common import plot_colours as c

# Parameters
n			= 11	# True gear ratio
alpha		= 0.8	# Filtering constant for first-order IIR filter
dt			= 0.017	# Timestep [s] # Based on what differentiation of
					# positions gives us compared to the velocity signals
turns2deg	= 360	# One turn in [deg]
deg2rad		= math.pi/180 # Degree to rad
turns2rad	= turns2deg*deg2rad # Turns to rad
export		= False	# Whether to export data for pgfplots
est_func	= fit_linear # Function to use for estimation of stiffness

# Experiment parameters (pendulum)
def outputTorque(outputPos):
	g			= 9.81		# Gravitational constant [m/s^2]
	m_weights	= 7.5		# Weight plates mass [kg]
	m_bar		= 0.432		# Pendulum mass [kg]
	m_mounting	= 0.211		# Mounting stuff (thread, nuts, etc) [kg]
	L			= 0.5		# Pendulum length
	
	# Compute max torque, then multiply by sine
	tau_max		= g * (m_weights+m_mounting) * L \
					+ g * m_bar * 0.5 * L
	tau 		= tau_max * np.sin(outputPos)
	
	return tau

# Load data
filename			= get_filename('../experiment_code/out_experiment_3_stiffness.csv')
filename_stiffness	= append_filename(filename, '_stiffness')
data				= np.genfromtxt(filename, delimiter=",")
data_stiffness		= np.genfromtxt(filename_stiffness, delimiter=",")

# Get fields
t			= data[:,0]
motorPos	= data[:,1]
outputPos	= data[:,2]
motorVel	= data[:,3]
outputVel	= data[:,4]
input_pos	= data[:,5]
Iq_set		= data[:,6]
Iq_meas		= data[:,7]

# Get fields from stiffness data
# Others are not important at the moment
motorPos_s	= data_stiffness[:,1]
outputPos_s	= data_stiffness[:,2]

# Filter data
Iq_meas_filt	= filter_iir(Iq_meas, alpha)

# Set the initial positions to zero
outputPos	= outputPos - outputPos[0]
motorPos	= motorPos - motorPos[0]
input_pos	= input_pos - input_pos[0]

# Same for the stiffness data
outputPos_s	= outputPos_s - outputPos_s[0]
motorPos_s	= motorPos_s - motorPos_s[0]

# Compute the position difference, remove offset
diff		= (motorPos/n) - outputPos
diff		= diff - diff[0] # Only for plotting, as no effect when using fit_linear

# Do the same for the stiffness data
diff_s		= (motorPos_s/n) - outputPos_s
diff_s		= diff_s - diff_s[1] # Center around the SECOND data point

# Compute pendulum angle and resulting applied torque
outputPos_rad = turns2rad * outputPos
tau = outputTorque(outputPos_rad)

# Do the same for stiffness data
outputPos_s_rad = turns2rad * outputPos_s
tau_s = outputTorque(outputPos_s_rad)

# Do curve fitting and estimate stiffness
# NOTE We ignore the first data point, because it is affected by play
(opt_params, parms_cov) = curve_fit(est_func, tau[1:], diff[1:])
k_estimate = 1 / (turns2rad*opt_params[0]) # [Nm/rad]

# Output information on fit
print('Curve fitting parameters:')
print(opt_params)
print('Estimated stiffness:', k_estimate, 'Nm/rad')

# Do the same for stiffness data
# NOTE We ignore the first data point, because it is affected by play
(opt_params_s, parms_cov) = curve_fit(est_func, tau_s[1:], diff_s[1:])
k_estimate_s = 1 / (turns2rad*opt_params_s[0]) # [Nm/rad]

# Output information on fit
print('Curve fitting parameters (stiffness data):')
print(opt_params_s)
print('Estimated stiffness (stiffness data):', k_estimate_s, 'Nm/rad')

# Do a quick estimate of torque constant while we're at it
(opt_params_Kt, parms_cov) = curve_fit(est_func, Iq_set, tau/n)
Kt_estimate = opt_params_Kt[0] # [Nm/A]

# Output information on fit
print('Estimated torque constant:', round(Kt_estimate,3), 'Nm/A')

# Export data for pgfplots
if export:
	data = (
		tau,
		turns2deg*diff,
		turns2deg*(est_func(tau, *opt_params)),
		Iq_set,
		Iq_meas,
		Iq_meas_filt,
	)

	# Reduce data
	#dt = 0.1
	#(t_RS, data_RS) = reduce_data(t, dt, data)

	export_data(
		data,
		headers = ['tau', 'diff', 'fit', 'Iq_set', 'Iq_meas', 'Iq_meas_filt'],
		filename_base = 'exp3'
	)

	data_s = (
		tau_s,
		turns2deg*diff_s,
		turns2deg*(est_func(tau_s, *opt_params_s))
	)

	# Reduce data
	#dt = 0.1
	#(t_RS, data_RS) = reduce_data(t, dt, data)

	export_data(
		data_s,
		headers = ['tau_s', 'diff_s', 'fit_s'],
		filename_base = 'exp3_s'
	)

# Plotting

# Figure and axes
(fig, ax) = plt.subplots(1,4)
fig.suptitle('Experiment 3 (stiffness): ' + filename)

# Motor and output position, and reference, over time
ax[0].plot(t, turns2deg*input_pos/n, '--', color=c.reference, label='input_pos/n')
ax[0].plot(t, turns2deg*motorPos/n, color=c.motorPos, label='motorPos/n')
ax[0].plot(t, turns2deg*outputPos, color=c.outputPos, label='outputPos')
ax[0].set_title('Motor and output position')
ax[0].set_xlabel('Time [s]')
ax[0].set_ylabel('Position [deg]')
ax[0].legend()

# Difference
ax[1].plot(tau, turns2deg*diff, color=c.rel_pos, label='Difference')
ax[1].scatter(tau_s, turns2deg*diff_s, color='k', label='Difference')
ax[1].plot(tau, turns2deg*(est_func(tau, *opt_params)), '--', color='brown', label='Linear fit')
ax[1].plot(tau_s, turns2deg*(est_func(tau_s, *opt_params_s)), color='brown', label='Linear fit (stiffness data)')
ax[1].set_title('Internal deflection (input vs output position)')
ax[1].set_xlabel('Torque [Nm]')
ax[1].set_ylabel('Deflection [deg]')
ax[1].legend()

# Current
ax[2].plot(t, Iq_meas, color=(0.8,0.8,0.8), label='Iq_meas')
ax[2].plot(t, Iq_meas_filt, color=(0.4,0.4,0.4), label='Iq_meas_filt')
ax[2].plot(t, Iq_set, '--', color=(0,0,0), label='Iq_set')
ax[2].set_title('Current')
ax[2].set_xlabel('Time [s]')
ax[2].set_ylabel('Current [A]')
ax[2].legend()

# Torque over current
ax[3].plot(Iq_set, tau/n, color=(0.4,0.4,0.4), label='Torque')
ax[3].plot(Iq_set, est_func(Iq_set, *opt_params_Kt), color=(0,0,0), label='Linear fit (Kt_estimate = '+str(round(Kt_estimate,3))+' Nm/A)')
ax[3].set_title('Torque produced')
ax[3].set_xlabel('Current [A]')
ax[3].set_ylabel('Torque [Nm]')
ax[3].legend()

plt.show()
