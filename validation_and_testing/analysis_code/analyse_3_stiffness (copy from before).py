import sys
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
from common import get_filename, filter_iir, fit_linear
from common import plot_colours as c

# Parameters
n			= 11	# True gear ratio
alpha		= 0.8	# Filtering constant for first-order IIR filter
dt			= 0.017	# Timestep [s] # Based on what differentiation of
					# positions gives us compared to the velocity signals
turns2deg	= 360	# One turn in [deg]
deg2rad		= math.pi/180 # Degree to rad
turns2rad	= turns2deg*deg2rad # Turns to rad

# Experiment parameters
l	= 0.5		# Arm length
m	= 5.1		# Mass
F	= m*9.81	# Linear force on mass

# Load data
filename = get_filename('../2021-12-10/exp3_stiffness/out_experiment_3_stiffness_5kg_60deg.csv')
data = np.genfromtxt(filename, delimiter=",")

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
Iq_meas_filt	= filter_iir(Iq_meas, alpha)

# Set the initial positions to zero
outputPos	= outputPos - outputPos[0]
motorPos	= motorPos - motorPos[0]
input_pos	= input_pos - input_pos[0]

# Compute the position difference, remove offset
diff		= (motorPos/n) - outputPos
#diff		= diff - 0.5/turns2deg # Remove manually tuned offset

# Compute pendulum angle and resulting applied torque
outputPos_rad = turns2rad * outputPos
tau = l * F * np.sin(outputPos_rad)

# Do curve fitting and estimate stiffness
#(opt_params, parms_cov) = curve_fit(fit_linear, tau, diff)
tau_slice	= np.append(tau[500:7040], tau[7500:12450])
diff_slice	= np.append(diff[500:7040]-0.1/turns2deg, diff[7500:12450]-0.5/turns2deg)
(opt_params, parms_cov) = curve_fit(fit_linear, tau_slice, diff_slice)
k_estimate = 1 / (turns2rad*opt_params[0]) # [Nm/rad]

# Output information on fit
print('Curve fitting parameters:')
print(opt_params)
print('Estimated stiffness:', k_estimate, 'Nm/rad')

# Figure and axes
(fig, ax) = plt.subplots(1,3)
fig.suptitle('Experiment 3 (stiffness): ' + filename)

# Motor and output position, and reference, over time
ax[0].plot(turns2deg*input_pos/n, '--', color=c.reference, label='input_pos/n')
ax[0].plot(turns2deg*motorPos/n, color=c.motorPos, label='motorPos/n')
ax[0].plot(turns2deg*outputPos, color=c.outputPos, label='outputPos')
ax[0].set_title('Motor and output position')
ax[0].set_xlabel('Time [s]')
ax[0].set_ylabel('Position [deg]')
ax[0].legend()

# Difference without and with removal of the cosine fit from experiment 1
ax[1].plot(tau, turns2rad*diff, color=c.rel_pos, label='Difference')
ax[1].plot(tau_slice, turns2rad*diff_slice, color='k', label='Difference')
ax[1].plot(tau, turns2rad*(fit_linear(tau, *opt_params)), color='brown', label='Linear fit')
ax[1].set_title('Difference in input/output position')
ax[1].set_xlabel('Torque [Nm]')
ax[1].set_ylabel('Deflection [deg]')
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

plt.savefig('exp_3_stiffness.pdf')

plt.show()
