#!/usr/bin/env python3

from breezyslam.vehicles import WheeledVehicle
from breezyslam.sensors import URG04LX

from engine import raytrace

import math
import numpy as np

import engine


class DriveableLaser(URG04LX):

    def __init__(self):

        URG04LX.__init__(self, 70, 145)

    def min_timestamp(self):
        # in ms
        return 1000 / self.scan_rate_hz

    def laser_generator(self, pos, map):
        laserList = []

        rads = math.radians(self.detection_angle_degrees)
        inital_angle = (math.pi - rads) / 2
        delta_angle = rads / self.scan_size

        for i in range(0, self.scan_size):
            theta = inital_angle + i * delta_angle
            rot = np.array([math.sin(theta), math.cos(theta)])
            laserList.append(raytrace(pos, rot, self.distance_no_detection_mm, map))
        return laserList


# Class for MinesRover custom robot ------------------------------------------

class DriveableRobot(WheeledVehicle):

    def __init__(self):
        WheeledVehicle.__init__(self, 77, 165)

        self.ticks_per_cycle = 2000

    def __str__(self):
        return '<%s ticks_per_cycle=%d>' % (WheeledVehicle.__str__(self), self.ticks_per_cycle)

    def computePoseChange(self, odometry):
        return WheeledVehicle.computePoseChange(self, odometry[0], odometry[1], odometry[2])

    def computeWheelPositions(self, pos, vec):

        theta = engine.angleOfVector(vec)

        # print("theta: ", np.rad2deg(theta))

        angle = (np.pi / 2) - theta

        # print("c/s: ", np.rad2deg(np.cos(angle)), np.sin(angle))

        halfAxle = self.halfAxleLengthMillimeters

        wheelOffset = np.array([halfAxle * np.cos(angle), -halfAxle * np.sin(angle)])
        leftPos = pos - wheelOffset
        rightPos = pos + wheelOffset

        # print("Wheel Dist:", self.halfAxleLengthMillimeters)
        # print("Wheel Positions:", leftPos, rightPos)
        # l_axle = leftPos - pos
        # r_axle = rightPos - pos
        #
        # print("Axle Vec:", l_axle, r_axle)
        #
        # print("Wheel Dist:", np.linalg.norm(l_axle), np.linalg.norm(r_axle))
        # print("Wheel Angle:", np.rad2deg(engine.angleBetweenVectors(l_axle, vec)),
        #       np.rad2deg(engine.angleBetweenVectors(r_axle, vec)))


        return leftPos, rightPos

    def computeWheelOdometryChange(self, u1, pos1, u2, pos2):
        # transpose to 0
        # print(pos1[1] - pos2[1], pos1[0] - pos2[0])

        if np.array_equal(pos1, pos2):
            return 0

        # print("p1:", pos1)
        # print("p2:", pos2)

        # translate so that pos1 is on zero
        pos1_trans = np.array([0, 0])
        pos2_trans = pos2 - pos1

        # print("a:", np.rad2deg(engine.angleOfVector(pos2_trans)))

        # now rotate such that pos2 is on the x axis
        theta = engine.angleOfVector(pos2_trans)

        # print("p2:", pos2_trans)
        # print("theta:", np.rad2deg(theta))

        R = engine.rotationVec(-theta)

        u1_trans = R.dot(u1)
        u2_trans = R.dot(u2)

        pos2_trans = R.dot(pos2 - pos1)

        #print("p2:", pos2_trans)

        u1t_slope = math.inf
        u2t_slope = math.inf

        # find the polynomial between the two points given the velocity vectors
        if u1_trans[0] == 0 and u1_trans[1] == 0:
            u1t_slope = 0
        elif u2_trans[0] == 0 and u2_trans[1] == 0:
            u2t_slope = 0
        elif u2_trans[0] != 0 and u2_trans[1] != 0:
            u1t_slope = u1_trans[1] / u1_trans[0]
            u2t_slope = u2_trans[1] / u2_trans[0]

        # ax^2 + bx + c
        a = (u1t_slope - u2t_slope) / (2 * (pos1_trans[0] - pos2_trans[0]))
        b = u1t_slope - 2 * a * pos2_trans[0]
        # c = 0 because of the transpose

        # use formula to find arc length
        pos2_x = pos2_trans[0]
        pos2_x2 = pos2_x ** 2

        # print("x:", pos2_x)
        # print("x^2:", pos2_x2)

        # this is wrong. doesn't use a / b
        eval = 0.25 * np.log(2 * pos2_x + np.sqrt(1 + 4 * pos2_x2)) + 0.5 * pos2_x * np.sqrt(1 + 4 * pos2_x2)
        # eval_pos2 = 0.25 * np.log(2 * 0 + np.sqrt(1 + 4 * 0)) + 0.5 * 0 * np.sqrt(1 + 4 * 0)

        eval = abs(eval)

        return np.linalg.norm(pos2_trans) / self.wheelRadiusMillimeters

        # return eval / self.wheelRadiusMillimeters
        # return np.sqrt(eval) / self.wheelRadiusMillimeters

    def computeOdometryChange(self, u1, pos1, u2, pos2):
        # dxyMillimeters, dthetaDegrees, dtSeconds

        # split into 2 sub-problems

        pos1_l, pos1_r = self.computeWheelPositions(pos1, u1)
        pos2_l, pos2_r = self.computeWheelPositions(pos2, u2)

        left = self.computeWheelOdometryChange(u1, pos1_l, u2, pos2_l)
        right = self.computeWheelOdometryChange(u1, pos1_r, u2, pos2_r)

        # print(left, right)
        return left, right


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


robot = DriveableRobot()

vel1 = np.array([3, 4])
pos1 = np.array([0, 0])
vel2 = np.array([3, 4])
pos2 = np.array([3, 4])

robot.computeOdometryChange(vel1, pos1, vel2, pos2)