import sys
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
from common import get_filename, filter_iir, fit_sines, export_data
from common import plot_colours as c

# For pgf plots - Use LaTeX default serif font
plt.rcParams.update({
    "font.serif": [],
})

# Parameters
n			= 11	# True gear ratio
alpha		= 0.9	# Filtering constant for first-order IIR filter
dt			= 0.017	# Timestep [s] # Based on what differentiation of
					# positions gives us compared to the velocity signals
turns2deg	= 360	# One turn in [deg]
deg2rad		= math.pi/180 # Degree to rad
turns2rad	= turns2deg*deg2rad # Turns to rad
export		= False	# Whether to export data for pgfplots

# Load data
filename = get_filename('../experiment_code/out_experiment_1_gear_ratio_2.0_rads.csv')
data = np.genfromtxt(filename, delimiter=",")

# Get fields
t			= data[:,0]
motorPos	= data[:,1]
outputPos	= data[:,2]
motorVel	= data[:,3]
outputVel	= data[:,4]

# Set the initial positions to zero
outputPos = outputPos - outputPos[0]
motorPos = motorPos - motorPos[0]

# Compute the position inaccuracy, removing DC offset
diff = (motorPos/n) - outputPos
diff = diff - np.mean(diff)

# Do curve fitting
(opt_params, parms_cov) = curve_fit(fit_sines, outputPos, diff)

# Output information on fit
print('Curve fitting parameters:')
print(opt_params)

# Compute RMS error
e = turns2deg*diff
rmse = np.sqrt(e.dot(e)/e.size)
print('RMS error:', rmse, 'deg.')

# Filter data
motorPos_filt	= filter_iir(motorPos, alpha)
outputPos_filt	= filter_iir(outputPos, alpha)
motorVel_filt	= filter_iir(motorVel, alpha)
outputVel_filt	= filter_iir(outputVel, alpha)

# Export data for pgfplots
if export:
	data = (
		outputPos,
		turns2deg*diff,
		turns2deg*fit_sines(outputPos, *opt_params)
	)

	# Reduce data
	#dt = 0.01
	#(t_RS, data_RS) = reduce_data(t, dt, data)

	export_data(
		data,
		headers = ['outputPos', 'diff', 'fit'],
		filename_base = 'exp1'
	)

# Plotting

# Figure and axes
(fig, ax) = plt.subplots(3,2)
fig.suptitle('Experiment 1 (gear ratio): ' + filename)

# Motor velocity
ax[0,0].plot(t, turns2rad*motorVel, color=c.motorPos_light)
ax[0,0].plot(t, turns2rad*motorVel_filt, color=c.motorPos)
ax[0,0].set_title('Motor velocity [rad/s]')

# Output velocity
ax[0,1].plot(t, turns2rad*outputVel, color=c.outputPos_light)
ax[0,1].plot(t, turns2rad*outputVel_filt, color=c.outputPos)
ax[0,1].set_title('Output velocity [rad/s]')

# Gear ratio
ax[1,0].plot(t, motorVel / outputVel, color=c.rel_pos_light)
ax[1,0].plot(t, motorVel_filt / outputVel_filt, color=c.rel_pos)
ax[1,0].set_title('Gear ratio []')

# Gear ratio over output turns
ax[1,1].plot(outputPos, motorVel / outputVel, color=c.rel_pos_light)
ax[1,1].plot(outputPos, motorVel_filt / outputVel_filt, color=c.rel_pos)
ax[1,1].set_title('Gear ratio over output turns')

# Gear ratio over output turns
ax[2,0].plot(outputPos, turns2deg*diff, color=c.rel_pos, label='Difference (RMSE = '+str(round(rmse,2))+' deg)')
ax[2,0].plot(outputPos, turns2deg*fit_sines(outputPos, *opt_params), color='orange', label='Cosine fit (%.2e, %.2e)' % tuple(opt_params[0:2]))
ax[2,0].set_title('Difference in input/output position (n ='+str(n)+')')
ax[2,0].set_xlabel('Output turns []')
ax[2,0].set_ylabel('Positioning inaccuracy [deg]')
ax[2,0].legend()

# Gear ratio over output turns
ax[2,1].plot(outputPos % 1, turns2deg*diff, color=c.rel_pos, label='Difference (RMSE = '+str(round(rmse,2))+' deg)')
ax[2,1].plot(outputPos % 1, turns2deg*fit_sines(outputPos, *opt_params), color='orange', label='Cosine fit (%.2e, %.2e)' % tuple(opt_params[0:2]))
ax[2,1].plot(outputPos % 1, turns2deg*(diff-fit_sines(outputPos, *opt_params)), color='brown', label='Residual after fit')
ax[2,1].set_title('Difference in input/output position (n ='+str(n)+')')
ax[2,1].set_xlabel('Output turns []')
ax[2,1].set_ylabel('Positioning inaccuracy [deg]')
ax[2,1].legend()

plt.show()
