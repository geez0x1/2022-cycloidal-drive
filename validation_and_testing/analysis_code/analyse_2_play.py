import sys
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
from common import get_filename, filter_iir, fit_sines
from common import plot_colours as c

# Parameters
n			= 11	# True gear ratio
alpha		= 0.8	# Filtering constant for first-order IIR filter
dt			= 0.017	# Timestep [s] # Based on what differentiation of
					# positions gives us compared to the velocity signals
turns2deg	= 360	# One turn in [deg]

# Load data
filename = get_filename('../experiment_code/out_experiment_2_play.csv')
data = np.genfromtxt(filename, delimiter=",")

# Get fields
t			= data[:,0]
motorPos	= data[:,1]
outputPos	= data[:,2]
motorVel	= data[:,3]
outputVel	= data[:,4]
input_pos	= data[:,5]

# Set the initial positions to zero
outputPos = outputPos - outputPos[0]
motorPos = motorPos - motorPos[0]
input_pos = input_pos - input_pos[0]

# Compute the position offset, removing DC offset
diff = (motorPos/n) - outputPos
diff = diff - np.mean(diff)

# Figure and axes
(fig, ax) = plt.subplots(1,2)
fig.suptitle('Experiment 2 (play): ' + filename)

# Motor and output position, and reference, over time
ax[0].plot(t, turns2deg*input_pos/n, '--', color=c.reference, label='input_pos/n')
ax[0].plot(t, turns2deg*motorPos/n, color=c.motorPos, label='motorPos/n')
ax[0].plot(t, turns2deg*outputPos, color=c.outputPos, label='outputPos')
ax[0].set_title('Motor and output position')
ax[0].set_xlabel('Time [s]')
ax[0].set_ylabel('Position [deg]')
ax[0].legend()
ax[0].grid()

# Difference without and with removal of the cosine fit from experiment 1
ax[1].plot(outputPos, turns2deg*diff, color=c.rel_pos, label='Difference')
ax[1].set_title('Difference in input/output position')
ax[1].set_xlabel('Output turns []')
ax[1].set_ylabel('Positioning inaccuracy [deg]')
ax[1].legend()
ax[1].grid()

plt.show()
