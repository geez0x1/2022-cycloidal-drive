import sys
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
from common import get_filename, filter_iir, fit_sines, export_data, reduce_data, fit_coulomb_viscous, fit_coulomb_bidirectional, fit_coulomb_viscous_bidirectional, fit_coulomb_viscous_bidirectional_tanh, reduce_data_by_skip
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
est_func	= fit_coulomb_viscous_bidirectional # Friction model

# Load data
filename	= get_filename('../experiment_code/out_experiment_5_friction.csv')
data		= np.genfromtxt(filename, delimiter=",")

# Get fields
t			= data[:,0]
motorPos	= data[:,1]
outputPos	= data[:,2]
motorVel	= data[:,3]
outputVel	= data[:,4]
input_vel	= data[:,5]
Iq_set		= data[:,6]
Iq_meas		= data[:,7]

# Filter data
motorPos_filt	= filter_iir(motorPos, alpha)
outputPos_filt	= filter_iir(outputPos, alpha)
motorVel_filt	= filter_iir(motorVel, alpha)
outputVel_filt	= filter_iir(outputVel, alpha)
Iq_set_filt		= filter_iir(Iq_set, alpha)
Iq_meas_filt	= filter_iir(Iq_meas, alpha)

# Estimate motor torque
tau_m			= k_t * n * Iq_meas
tau_m_filt		= k_t * n * Iq_meas_filt
tau_m_filt2		= filter_iir(tau_m, 0.95) # Heavy filtering
tau_m_set		= k_t * n * Iq_set
tau_m_set_filt	= k_t * n * Iq_set_filt

# Do curve fitting to friction parameters
# Not using filtered data - no benefit and discontinuities take time to converge
fitdata_x = turns2rad*motorVel/n
fitdata_y = tau_m_set
(opt_params, parms_cov) = curve_fit(est_func, fitdata_x, fitdata_y)
d_c_estimate	= opt_params[0] # Coulomb friction parameter [Nm]
d_c_n_estimate	= opt_params[1] # Coulomb friction parameter [Nm] (negative dir)
d_v_estimate	= opt_params[2] # Viscous friction parameter [Nms/rad]
d_v_n_estimate	= opt_params[3] # Viscous friction parameter [Nms/rad] (negative dir)

# Output information on fit
print('Estimated friction parameters:')
print('d_c =', round(d_c_estimate,3), 'Nm, d_c_n =', round(d_c_n_estimate,3), 'Nm.')
print('d_v =', round(d_v_estimate,3), 'Nms/rad, d_v_n =', round(d_v_n_estimate,3), 'Nms/rad.')

# For better plotting of fit
fit_x = np.linspace(np.min(turns2rad*outputVel_filt), np.max(turns2rad*outputVel_filt), np.size(outputVel_filt))
fit_y = est_func(fit_x, *opt_params)

# Export data for pgfplots
if export:
	data = (
		t,
		turns2rad*input_vel/n,
		turns2rad*motorVel/n,
		turns2rad*outputVel,
		turns2rad*outputVel_filt,
		tau_m,
		tau_m_filt,
		tau_m_set,
		fit_x,
		fit_y,
	)

	# Reduce data
	#dt = 0.1
	#(t_RS, data_RS) = reduce_data(t, dt, data)

	# Reduce data by skipping datapoints
	# A better approach would be scipy.interpolate()
	data_RS = reduce_data_by_skip(data, 5)

	export_data(
		data_RS,
		headers = ['t', 'input_vel_reduced', 'motorVel_reduced', 'outputVel', 'outputVel_filt', 'tau_m', 'tau_m_filt', 'tau_m_set', 'fit_x', 'fit_y'],
		filename_base = 'exp5'
	)

# Plotting

# Figure and axes
(fig, ax) = plt.subplots(1,4)
fig.suptitle('Experiment 4 (peak speed): ' + filename)

# Motor speed
ax[0].plot(t, turns2rad*input_vel, '--', color=c.reference, label='Reference')
ax[0].plot(t, turns2rad*motorVel, color=c.motorPos_light)
ax[0].plot(t, turns2rad*motorVel_filt, color=c.motorPos, label='Motor speed')
ax[0].set_title('Motor speed')
ax[0].set_xlabel('Time [s]')
ax[0].set_ylabel('Speed [rad/s]')
ax[0].legend()
ax[0].grid()

# Output speed
ax[1].plot(t, turns2rad*input_vel/n, '--', color=c.reference, label='Reference')
ax[1].plot(t, turns2rad*outputVel, color=c.outputPos_light)
ax[1].plot(t, turns2rad*outputVel_filt, color=c.outputPos, label='Output speed')
ax[1].set_title('Output speed')
ax[1].set_xlabel('Time [s]')
ax[1].set_ylabel('Speed [rad/s]')
ax[1].legend()
ax[1].grid()

# Current
ax[2].plot(t, Iq_meas, color=(0.8,0.8,0.8), label='Iq_meas')
ax[2].plot(t, Iq_meas_filt, color=(0.4,0.4,0.4), label='Iq_meas_filt')
ax[2].plot(t, Iq_set, '--', color=(0,0,0), label='Iq_set')
ax[2].set_title('Current')
ax[2].set_xlabel('Time [s]')
ax[2].set_ylabel('Current [A]')
ax[2].legend()
ax[2].grid()

# Torque-speed curve
ax[3].plot(fitdata_x, fitdata_y, color=c.outputPos, label='Motor torque')
ax[3].plot(turns2rad*outputVel_filt, est_func(turns2rad*outputVel_filt, *opt_params), color='brown', label='Est. friction model')
ax[3].set_title('Torque-speed')
ax[3].set_xlabel('Speed [rad/s]')
ax[3].set_ylabel('Torque [Nm]')
ax[3].legend()
ax[3].grid()

plt.show()
