#!/usr/bin/env python3

import math
import numpy as np
from queue import PriorityQueue

from BreezySLAM.examples.mines import MinesLaser, Rover, load_data
from driveable_robot import DriveableLaser as Laser
from driveable_robot import DriveableRobot as Robot

import BreezySLAM.examples.pgm_utils as pgm

from engine import Map
from engine import raytrace

#from run_slam import mm2pix
from BreezySLAM.examples.pgm_utils import pgm_save
from engine import save_data
from engine import Data

# Map size, scale
MAP_SIZE_PIXELS          = 800
MAP_SIZE_METERS          =  32

#map, map_size = pgm.pgm_load("input/map_test.pgm") #TODO: move to main
#TODO: check that dims are as expected / adjest program to fit dims

mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

for i in range(0, MAP_SIZE_PIXELS * MAP_SIZE_PIXELS):
	mapbytes[i] = 127

# Start Pos
START_POS                = np.array([1000.0, 1000.0])
START_ROT                = 0
INITIAL_TIMESTAMP        = 0

# Waypoints
WAYPOINTS = np.array([[5000.0, 5000.0], [9000.0, 5000.0], [9000.0, 1000.0], [9000.0, 5000.0], [9000.0, 9000.0], [6500.0, 9000.0], [6500.0, 1000.0], [6500.0, 9000.0],[1000.0, 9000.0], [1000.0, 1000.0]])

SPEED = 1.0

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

def mm2pix(mm):
    return int(mm / (MAP_SIZE_METERS * 1000. / MAP_SIZE_PIXELS))

def recordToMap(point):
	x_mm = point[0]
	y_mm = point[1]

	x_pix = mm2pix(x_mm)
	y_pix = mm2pix(y_mm)
	mapbytes[y_pix * MAP_SIZE_PIXELS + x_pix] = 0

vehicle_pos = START_POS
next_waypoint_index = 0

#timestamps, lidars, odometries = load_data('.', "BreezySLAM/examples/exp1")
#print(lidars)

laser = Laser()
rot_gen = laser.rotation_generator(INITIAL_TIMESTAMP, START_ROT)

robot = Robot()

timestamp_queue = PriorityQueue()

last_timestamp = INITIAL_TIMESTAMP

map = Map()

dataList = []
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
	
	if next_operation[1] == "lidar":
		#laser_data = laser.calc_laser(vehicle_pos, next_operation[3])
		rot = next_operation[2]
		rotVec = np.array([math.sin(rot), math.cos(rot)])
		laser_data = raytrace(vehicle_pos, rotVec, laser.distance_no_detection_mm, map)

		print("---------------------------")
		print("timestamp: ", len(dataList))
		print("time: ", timestamp)
		print("vehicle pos: ", vehicle_pos)
		if laser_data is not None:
			print("distance: ", laser_data.dist)
			print("hit pos: ", laser_data.pos)
			recordToMap(laser_data.pos)

		recordToMap(vehicle_pos)
		dataList.append(Data(timestamp, vehicle_pos, laser_data))

	if np.linalg.norm(vehicle_pos - WAYPOINTS[next_waypoint_index]) < SPEED * delta_time:
		next_waypoint_index += 1
		print("next waypoint: " + str(next_waypoint_index))

save_data(".", "data", dataList)
pgm_save("a.pgm", mapbytes, (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS))