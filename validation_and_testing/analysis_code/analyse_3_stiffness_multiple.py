import sys
import os
import math
import numpy as np
from types import SimpleNamespace
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

# Define filenames
filenames = [
	# 2022-02-17 - rotation of output flange in 60deg increments
	'../2022-02-17/exp3_output_flange_rotated/run1_60deg/out_experiment_3_stiffness_stiffness.csv',
	'../2022-02-17/exp3_output_flange_rotated/run2_120deg/out_experiment_3_stiffness_stiffness.csv',
	'../2022-02-17/exp3_output_flange_rotated/run3_180deg/out_experiment_3_stiffness_stiffness.csv',
	'../2022-02-17/exp3_output_flange_rotated/run4_240deg/out_experiment_3_stiffness_stiffness.csv',
	'../2022-02-17/exp3_output_flange_rotated/run5_300deg/out_experiment_3_stiffness_stiffness.csv',
	'../2022-02-17/exp3_output_flange_rotated/run6_360deg/out_experiment_3_stiffness_stiffness.csv',

	# 2022-02-17 - very stiff anomaly
	#'../2022-02-17/exp3_very_stiff_anomaly/run1/out_experiment_3_stiffness_stiffness.csv',
	#'../2022-02-17/exp3_very_stiff_anomaly/run2/out_experiment_3_stiffness_stiffness.csv',
	#'../2022-02-17/exp3_very_stiff_anomaly/run3/out_experiment_3_stiffness_stiffness.csv',
	#'../2022-02-17/exp3_very_stiff_anomaly/run4/out_experiment_3_stiffness_stiffness.csv',

	# 2022-02-11_proto2_rebuild_output
	#'../2022-02-11_proto2_rebuild_output/exp3/run1_25Nm/out_experiment_3_stiffness_stiffness.csv',
	#'../2022-02-11_proto2_rebuild_output/exp3/run3_35Nm/out_experiment_3_stiffness_stiffness.csv',
	#'../2022-02-11_proto2_rebuild_output/exp3/run6_fail_at_30Nm/out_experiment_3_stiffness_stiffness.csv',

	# 2022-02-11_proto2_rebuild_output - Negative direction
	#'../2022-02-11_proto2_rebuild_output/exp3/run2_-25Nm/out_experiment_3_stiffness_stiffness.csv',
	#'../2022-02-11_proto2_rebuild_output/exp3/run7_65deg_back_in_5deg_steps/out_experiment_3_stiffness_stiffness.csv',

	# 2022-01-21_proto2
	#'../2022-01-21_proto2/exp3/50deg_30Nm/out_experiment_3_stiffness_stiffness.csv',
	#'../2022-01-21_proto2/exp3/50deg_30Nm_steps_of_2deg/out_experiment_3_stiffness_stiffness.csv',
	#'../2022-01-21_proto2/exp3/51deg_31Nm/out_experiment_3_stiffness_stiffness.csv',
	#'../2022-01-21_proto2/exp3/62deg_35Nm/out_experiment_3_stiffness_stiffness.csv',

	# 2021-12-10 (proto1) (continuous data, not sampled in the same manner)
	#'../2021-12-10/exp3_stiffness/out_experiment_3_stiffness_5kg_60deg.csv',
]

# Load data for each file
datasets = []
for filename in filenames:
	# Load this file
	d = np.genfromtxt(filename, delimiter=",")

	# Create simple namespace with raw data and other computed fields
	exp = SimpleNamespace()

	# Get fields
	exp.filename	= filename
	exp.t			= d[:,0] # Time not used
	exp.motorPos	= d[:,1]
	exp.outputPos	= d[:,2]
	exp.motorVel	= d[:,3]
	exp.outputVel	= d[:,4]
	exp.input_pos	= d[:,5]
	exp.Iq_set		= d[:,6]
	exp.Iq_meas		= d[:,7]

	# Filter data
	exp.Iq_meas_filt = filter_iir(exp.Iq_meas, alpha)

	# Set the initial positions to zero
	exp.outputPos	= exp.outputPos - exp.outputPos[0]
	exp.motorPos	= exp.motorPos - exp.motorPos[0]
	exp.input_pos	= exp.input_pos - exp.input_pos[0]

	# Compute the position difference
	exp.diff		= (exp.motorPos/n) - exp.outputPos
	exp.diff		= exp.diff - 0.0014639679418157825 # Manually tuned to bring the data down to intersect with (0,0) for plotting purposes (slope remains identical)
	#exp.diff		= exp.diff - exp.diff[1] # Center around the SECOND data point

	# Compute pendulum angle and resulting applied torque
	exp.outputPos_rad = turns2rad * exp.outputPos
	exp.tau = outputTorque(exp.outputPos_rad)

	# Do curve fitting and estimate stiffness
	# NOTE We ignore the first data point, because it is affected by play
	(exp.opt_params, exp.parms_cov) = curve_fit(est_func, exp.tau[1:], exp.diff[1:])
	exp.k_estimate = 1 / (turns2rad * exp.opt_params[0]) # [Nm/rad]

	# Find offset b in ax+b
	# This offset can also be used to remove the play offset
	#exp.diff_offset = exp.opt_params[1]
	#print('Found deflection offset', turns2deg*exp.diff_offset, 'deg')

	# Output information on fit
	print('Curve fitting parameters for', filename, ':')
	print(exp.opt_params)
	print('Estimated stiffness:', exp.k_estimate, 'Nm/rad')

	# Do a quick estimate of torque constant while we're at it
	(exp.opt_params_Kt, exp.parms_cov_Kt) = curve_fit(est_func, exp.Iq_set, exp.tau/n)
	exp.Kt_estimate = exp.opt_params_Kt[0] # [Nm/A]

	# Output information on fit
	print('Estimated torque constant:', round(exp.Kt_estimate,3), 'Nm/A')

	# Append object
	datasets.append(exp)

	print('----')


# Merge all data sets into a single set for fitting
# We remove the first data point for each, as it is affected by play
tau		= np.concatenate(tuple([exp.tau[1:] for exp in datasets]))
diff	= np.concatenate(tuple([exp.diff[1:] for exp in datasets]))

# Do curve fitting and estimate stiffness
(opt_params, parms_cov) = curve_fit(est_func, tau, diff)
k_estimate = 1 / (turns2rad * opt_params[0]) # [Nm/rad]

# Output information on fit
print('Curve fitting parameters for ALL datasets:')
print(opt_params)
print('Estimated stiffness:', k_estimate, 'Nm/rad')

# Export data for pgfplots
if export:
	data = (
		tau,
		turns2deg*diff,
		turns2deg*(est_func(tau, *opt_params)),
	)

	export_data(
		data,
		headers = ['tau', 'diff', 'fit'],
		filename_base = 'exp3'
	)

	# Export data per experiment
	for i, d in enumerate(datasets):
		data = (
			d.tau[1:],
			turns2deg*d.diff[1:],
			turns2deg*(est_func(d.tau[1:], *d.opt_params)),
		)

		export_data(
			data,
			headers = ['tau', 'diff', 'fit'],
			filename_base = 'exp3_dataset'+str(i)
		)

# Plotting

# Figure and axes
(fig, ax) = plt.subplots(1,3)
fig.suptitle('Experiment 3 (stiffness): Multiple datasets')

# Difference
ax[0].scatter(tau, turns2deg*diff, color='g', label='Difference (all data sets)')
ax[0].plot(tau, turns2deg*(est_func(tau, *opt_params)), '--', color='brown', label='Linear fit (K_est = '+str(round(k_estimate,3))+' Nm/rad)')
ax[0].set_title('Internal deflection (input vs output position)')
ax[0].set_xlabel('Torque [Nm]')
ax[0].set_ylabel('Deflection [deg]')
ax[0].legend()

for d in datasets:
	ax[1].scatter(d.tau, turns2deg*d.diff, label=d.filename)
	ax[1].plot(d.tau, turns2deg*(est_func(d.tau, *d.opt_params)), '--', color='brown', label='Linear fit (K_est = '+str(round(d.k_estimate,3))+' Nm/rad)')
ax[1].set_title('Internal deflection (input vs output position)')
ax[1].set_xlabel('Torque [Nm]')
ax[1].set_ylabel('Deflection [deg]')
ax[1].legend()

# Torque over current
for d in datasets:
	ax[2].plot(d.Iq_set, d.tau/n, color=(0.4,0.4,0.4), label='Torque')
	ax[2].plot(d.Iq_set, est_func(d.Iq_set, *d.opt_params_Kt), color=(0,0,0), label='Linear fit (Kt_estimate = '+str(round(d.Kt_estimate,3))+' Nm/A)')
ax[2].set_title('Torque produced')
ax[2].set_xlabel('Current [A]')
ax[2].set_ylabel('Torque [Nm]')
ax[2].legend()

plt.show()
