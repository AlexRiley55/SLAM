#comes from: https://intellipaat.com/community/7530/how-to-read-pgm-p2-image-in-python

import numpy as np

import matplotlib.pyplot as plt

def readpgm(name):
	
	with open(name) as f:
		lines = f.readlines()
	# This ignores commented lines
	for l in list(lines):
		if l[0] == '#':
			lines.remove(l)

	# here,it makes sure it is ASCII format (P2)
	assert lines[0].strip() == 'P2' 

        # Converts data to a list of integers
	data = []
	for line in lines[1:]:
		data.extend([int(c) for c in line.split()])
	return (np.array(data[3:]),(data[1],data[0]),data[2])

#example
#data = readpgm('/location/of/file.pgm')
#plt.imshow(np.reshape(data[0],data[1])) # Usage example
