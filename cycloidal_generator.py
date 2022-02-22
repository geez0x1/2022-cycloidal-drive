# cycloidal_generator.py

# Copyright © 2022 Wesley Roozing and Glenn Roozing

# Permission is hereby granted, free of charge, to any person obtaining a copy of this
# software and associated documentation files(the “Software”), to deal in the Software
# without restriction, including without limitation the rights to use, copy, modify,
# merge, publish, distribute, sublicense, and / or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to the following
# conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
# INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
# PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
# HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
# CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
# OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

# Acknowledgements:
# Credit to mawildoer for inspiration for this script:
# https://github.com/mawildoer/cycloidal_generator


###############################################################################

# Imports
import numpy as np
import matplotlib.pyplot as pyplot
import math
#import sys
import os
import shutil


###############################################################################
# Functions

# Get a point along the curve
def getPoint(t, R, Rr, E, N):
	psi = math.atan2(math.sin((1-N)*t), ((R/(E*N))-math.cos((1-N)*t)))

	x = (R*math.cos(t))-(Rr*math.cos(t+psi))-(E*math.cos(N*t))
	y = (-R*math.sin(t))+(Rr*math.sin(t+psi))+(E*math.sin(N*t))
	return (x,y)

# Get the Euclidian distance between two points
def getDist(xa, ya, xb, yb):
	return math.sqrt((xa-xb)**2 + (ya-yb)**2)

# Rotate a vector consisting of a tuple
def rotate(v, theta):
	if type(v) is np.ndarray:
		for i,vi in enumerate(v):
			(x,y) = rotate((vi[0],vi[1]),theta)
			v[i][0] = x
			v[i][1] = y

		return v
	else:
		# Get x/y coordinates of point
		x = v[0]
		y = v[1]

		# Return new tuple
		return (	x * np.cos(theta) - y * np.sin(theta),
					x * np.sin(theta) + y * np.cos(theta)		)

# Move points by a vector m
def move(v, m):
	if type(v) is np.ndarray:
		v = v+np.array(m)
		return v
	else:
		return (v[0]+m[0], v[1]+m[1])


###############################################################################
# Parameters

# Major parameters
R = 83/2	# Rotor radius [mm]
N = 12		# Number of rollers [] - needs to be even for two disks
No = 6		# Number of output pins/holes [] - needs to be even for two disks

# Compute gear ratio
g = 1/(N-1)

# Output major parameters
print('Rotor radius R =', R, 'mm')
print('Number of rollers N =', N)
print('Ratio will be 1/(N-1) = g =', g)

# Hardware parameters
bearing_outer = 9/2 # Outer bearing radius [mm]
bearing_lg_outer = 32/2 # Large bearing - outer radius [mm]

# Dependent parameters
Rr = bearing_outer		# Roller radius [mm] (Original expression: housing_cir / (4 * N))
Ro = bearing_outer		# Output pin radius [mm]
# (value for wall thickness 2.4mm to disk edge)
Lo = 0.6277108433734939759036144578313253012048192771084337349397590361 * R # Output hole midpoint location (radially) [mm]
# (not an ugly number)
#Lo = 0.62 * R # Output hole midpoint location (radially) [mm]
E = 0.45 * Rr			# Eccentricity [mm]
Roh = Ro + E			# Output hole radius [mm]
Re = bearing_lg_outer  # Input shaft radius [mm]

# Parameters for point/curve generation of the cycloid disk
maxDist = 0.25 * Rr # Maximum allowed distance between points
minDist = 0.5 * maxDist # Minimum allowed distance between points


###############################################################################
# Main

# Prepare list of points
points = []

# Get starting point
(xs, ys) = getPoint(0, R, Rr, E, N)
points.append((xs,ys))
#print('(xs, ys) = (', xs, ', ', ys, ')')

# Compute ending point
et = 2 * math.pi# / (N-1) # Angular offset for ending point
(xe, ye) = getPoint(et, R, Rr, E, N)
points.append((xe,ye))
#print('(xe, ye) = (', xe, ', ', ye, ')')

# Initialise point generation
x = xs
y = ys
dist = 0
ct = 0
dt = math.pi / N
numPoints = 0


###############################################################################
# Point generation

# Keep generating points until we are close enough to the end point
while ((math.sqrt((x-xe)**2 + (y-ye)**2) > maxDist or ct < et/2) and ct < et):

	# Compute the new point (xt,yt)
	(xt, yt) = getPoint(ct+dt, R, Rr, E, N)

	# Compute the distance to the previous point (x,y)
	dist = getDist(x, y, xt, yt)

	# Set angle increment adjustment
	ddt = dt/2
	lastTooBig = False
	lastTooSmall = False

	# If the distance is not within the specified interval, reduce or increase the angle increment
	# Notice dt should always be positive
	while (dist > maxDist or dist < minDist):
		if (dist > maxDist):
			if (lastTooSmall):
				ddt /= 2

			lastTooSmall = False
			lastTooBig = True

			if (ddt > dt/2):
				ddt = dt/2

			dt -= ddt

		elif (dist < minDist):
			if (lastTooBig):
				ddt /= 2

			lastTooSmall = True
			lastTooBig = False
			dt += ddt

		(xt, yt) = getPoint(ct+dt, R, Rr, E, N)
		dist = getDist(x, y, xt, yt)

	# Update the old point location
	x = xt
	y = yt

	# Append the new point to the list, increase the count
	points.append((x,y))
	numPoints += 1

	# Increment the angular offset by the angle increment with which the new point was computed
	ct += dt

# Show points
#print('Generated', len(points), 'points')
#print(points)


###############################################################################
# Check wall thicknesses
print('Minimum wall thicknesses:')

# Input shaft to output hole
x_input_right_edge = E + Re
x_output_left_edge = E + Lo - Roh
d = x_output_left_edge-x_input_right_edge
print('    Input shaft to output hole:', round(d,3), 'mm.')

# Output hole to disk edge
x_output_edge = E + Lo + Roh
x_disk_edge = E + xs
d = x_disk_edge-x_output_edge
print('    Output hole to disk edge:', round(d,3), 'mm.')

# Output hole to output hole
x1 = Lo * math.cos(1 * 2 * math.pi / No) + E
y1 = Lo * math.sin(1 * 2 * math.pi / No)
x2 = Lo * math.cos(2 * 2 * math.pi / No) + E
y2 = Lo * math.sin(2 * 2 * math.pi / No)
d = getDist(x1, y1, x2, y2) - 2 * Roh
print('    Output hole to output hole:', round(d,3), 'mm.')


###############################################################################
# Plots

# Do we want to make a video?
makeVideo = False

# Plot two disks?
twoDisks = False

# Prepare some variables for video
if makeVideo:
	j = 0 # Frame counter
	theta_end = 2*np.pi/g # One output rotation
	fps = 60
	n = round(fps/g) # fps frames per input rotation

	# Make sure there is a directory for rendered frames
	if not os.path.isdir('frames'):
		os.mkdir('frames')
else:
	theta_end = 2*np.pi # One input rotation
	n = 1 # Number of frames in total

# For a range of thetas..
for theta in np.linspace(0, theta_end, n):
	# Print current value of theta
	if makeVideo:
		print('Rendering theta =', round(theta,2), 'rad')

	# Figure and axes
	fig = pyplot.figure(figsize=(10,10))
	ax = pyplot.axes()

	# x=0 and y=0 grid lines
	ax.plot((-R-1,R+1),(0,0), color=(0.8,0.8,0.8))
	ax.plot((0,0),(-R-1,R+1), color=(0.8,0.8,0.8))

	# Rotor radius
	c = pyplot.Circle((0,0), R, fill=False)
	ax.add_artist(c)

	# Input shaft and axis of rotation
	c = pyplot.Circle(rotate((E,0),theta), Re, color='g')
	ax.add_artist(c)
	c = pyplot.Circle((0,0), 1.0, color='k')
	ax.add_artist(c)

	# Show a line depicting the input axis
	p = rotate((R,0),theta)
	ax.plot((0,p[0]), (0,p[1]), color='b')

	# Plot of rotor curve

	# Get numpy array of points
	dt=np.dtype('float,float')
	points_arr = np.array(points)
	points_arr2 = points_arr.copy()

	# Transform in three steps:
	# 1. Rotate around rollers
	# 2. Offset by eccentricity
	# 3. Rotate with input shaft
	points_arr = rotate(points_arr, -N*g*theta) # Guessed this angle heuristically but it works
	points_arr = move(points_arr, (E,0))
	points_arr = rotate(points_arr, theta)
	ax.plot(points_arr[:,0], points_arr[:,1], color='b')

	# Compute rotational offset for second disk and its holes
	offset = 0

	# Plot a second disk
	if twoDisks:
		points_arr2 = rotate(points_arr2, -N*g*theta+offset)
		points_arr2 = move(points_arr2, (E,0))
		points_arr2 = rotate(points_arr2, theta+math.pi)
		ax.plot(points_arr2[:,0], points_arr2[:,1], color='pink')

	# Start and end point of profile
	#ax.scatter(xs,ys, color='r')
	#ax.scatter(xe,ye, color='g')

	# Add rollers using circles
	for i in range(0,N):
		x = R * math.cos(i * 2 * math.pi / N)
		y = R * math.sin(i * 2 * math.pi / N)
		c = pyplot.Circle((x,y), Rr, color='k')
		ax.add_artist(c)

	# Add output holes on rotor
	for i in range(0,No):
		# Get locations of output holes on disk
		x = Lo * math.cos(i * 2 * math.pi / No)
		y = Lo * math.sin(i * 2 * math.pi / No)

		# Same three transform steps as above
		p = rotate((x,y), -N*g*theta)
		p = move(p, (E,0))
		p = rotate(p, theta)

		# Plot
		c = pyplot.Circle(p, Roh, color='b', fill=False)
		ax.add_artist(c)

		# If we are plotting two disks..
		if twoDisks:
			# Same procedure for second disk holes
			p = rotate((x,y), -N*g*theta+offset)
			p = move(p, (E,0))
			p = rotate(p, theta+math.pi)

			# Plot
			c = pyplot.Circle(p, Roh, color='pink', fill=False)
			ax.add_artist(c)

	# Add output pins
	for i in range(0,No):
		x = Lo * math.cos(i * 2 * math.pi / No)
		y = Lo * math.sin(i * 2 * math.pi / No)
		c = pyplot.Circle(rotate((x,y),-theta*g), Ro, color='r')
		ax.add_artist(c)

	# Show output axis line
	p = rotate((R,0),-theta*g)
	ax.plot((0,p[0]), (0,p[1]), color='r')

	# Plot properties
	ax.set_xlim(-R-10, R+10)
	ax.set_ylim(-R-10, R+10)
	ax.set_title('theta = ' + str(round(theta,2)) + ' rad')

	if makeVideo:
		fig.savefig('frames/frame_' + str(j) + '.png')
		pyplot.close(fig)
		j=j+1

# Display final state
print("Final state:")
print("Input angle:", round(theta,3), "rad, output angle:", -round(theta*g,3), "rad.")

# Create video
if makeVideo:
	print('Converting to mkv video file..')
	os.system('ffmpeg -f image2 -r ' + str(fps) + ' -i frames/frame_%d.png video.mkv')
	shutil.rmtree('frames', ignore_errors=True) # Delete non-empty directory
