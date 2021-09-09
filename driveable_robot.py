#!/usr/bin/env python3

from breezyslam.vehicles import WheeledVehicle
from breezyslam.sensors import URG04LX

from engine import raytrace

import math
import numpy as np

import map_utilities


class DriveableLaser(URG04LX):

    def __init__(self):

        URG04LX.__init__(self, 70, 145)

    def min_timestamp(self):
        return 1.0 / self.scan_rate_hz

    def rotation_generator(self, initial_timestamp, initial_rot):
        curr_timestamp = initial_timestamp
        delta_rot = self.detection_angle_degrees / self.scan_size
        curr_rot = initial_rot  # should this be -detection_angle_degrees / 2 ?

        while True:
            curr_timestamp += self.min_timestamp()
            yield curr_timestamp, curr_rot
            curr_rot += delta_rot
            if curr_rot > self.detection_angle_degrees / 2:
                curr_rot = -self.detection_angle_degrees / 2

    def laser_generator(self, pos, maxDist, map):
        for timestamp, rot in self.rotation_generator(0, 0):
            rotVec = np.array([math.sin(rot), math.cos(rot)])
            yield timestamp, raytrace(pos, rotVec, maxDist, map)


# Class for MinesRover custom robot ------------------------------------------

class DriveableRobot(WheeledVehicle):

    def __init__(self):
        WheeledVehicle.__init__(self, 77, 165)

        self.ticks_per_cycle = 2000

    def __str__(self):
        return '<%s ticks_per_cycle=%d>' % (WheeledVehicle.__str__(self), self.ticks_per_cycle)

    def computePoseChange(self, odometry):
        return WheeledVehicle.computePoseChange(self, odometry[0], odometry[1], odometry[2])

    # def computeOdometryChange(self, pos_change):

    def extractOdometry(self, timestamp, leftWheel, rightWheel):
        # Convert microseconds to seconds, ticks to angles
        return timestamp / 1e6, \
               self._ticks_to_degrees(leftWheel), \
               self._ticks_to_degrees(rightWheel)

    def odometryStr(self, odometry):
        return '<timestamp=%d usec leftWheelTicks=%d rightWheelTicks=%d>' % \
               (odometry[0], odometry[1], odometry[2])

    def _ticks_to_degrees(self, ticks):
        return ticks * (180. / self.ticks_per_cycle)
