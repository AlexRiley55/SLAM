#!/usr/bin/env python3

import math
import numpy as np
from queue import PriorityQueue

from BreezySLAM.examples.mines import MinesLaser, Rover, load_data
from driveable_robot import DriveableLaser as Laser
from driveable_robot import DriveableRobot as Robot

import pgm_parser.pgm_parser

# Map size, scale
MAP_SIZE_PIXELS          = 800
MAP_SIZE_METERS          =  32

data = readpgm('/location/of/file.pgm')
map = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS) #TODO: load from file

# Start Pos
START_POS                = np.array([8.0, 28.0])
START_ROT                = 0
INITIAL_TIMESTAMP        = 0

# Waypoints
WAYPOINTS = np.array([[8, 8], [24, 8], [24, 28]])

SPEED = 0.6

def distance(pos1, pos2):
	return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

def translate(pos, trans):
	pos = (pos[0] + trans[0], pos[1] + trans[1])

def direction(pos1, pos2):
	c = distance(pos1, pos2)
	return ((pos2[0] - pos1[0])/c, (pos2[1] - pos1[1])/c)

def normalize(arr):
	norm = np.linalg.norm(arr)
	if not norm:
		return arr
	return arr / norm

vehicle_pos = START_POS
next_waypoint_index = 0


#timestamps, lidars, odometries = load_data('.', "BreezySLAM/examples/exp1")
#print(lidars)

laser = Laser()
rot_gen = laser.rotation_generator(INITIAL_TIMESTAMP, START_ROT)

robot = Robot()

timestamp_queue = PriorityQueue()

last_timestamp = INITIAL_TIMESTAMP

while next_waypoint_index < WAYPOINTS.shape[0]:
	print(np.linalg.norm(vehicle_pos - WAYPOINTS[next_waypoint_index]))

	timestamp, next_rot = next(rot_gen)
	timestamp_queue.put((timestamp, "lidar", next_rot))
	#other sensor data would go here
        
	next_operation = timestamp_queue.get()
	#TODO: check if multiple operations share the same timestamp
	#TODO: add a check for some minimum distance to travel regardless of sensors (don't record it though)

	delta_time = next_operation[0] - last_timestamp
	delta_time = round(delta_time, 14)
	#better to have less precision then have float errors here as errors quickly compound

	print("dt: ", delta_time)
	dir = normalize(WAYPOINTS[next_waypoint_index] - vehicle_pos)
	vehicle_pos += dir * SPEED * delta_time
	
	if next_operation[2] == "lidar":
		laser_data = laser.calc_laser(vehicle_pos, next_operation[3])
		#TODO: log data

	if np.linalg.norm(vehicle_pos - WAYPOINTS[next_waypoint_index]) < SPEED * 2:
		next_waypoint_index += 1
		print("next waypoint: " + str(next_waypoint_index))
		
	
