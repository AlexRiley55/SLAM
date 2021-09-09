#!/usr/bin/env python3

# Map size, scale
MAP_SIZE_PIXELS          = 800
MAP_SIZE_METERS          =  32

from breezyslam.algorithms import Deterministic_SLAM, RMHC_SLAM

from BreezySLAM.examples.mines import load_data

from driveable_robot import DriveableLaser as Laser
from driveable_robot import DriveableRobot as Robot

from BreezySLAM.examples.progressbar import ProgressBar
from BreezySLAM.examples.pgm_utils import pgm_save

from sys import argv, exit, stdout
from time import time

from engine import Map
from engine import raytrace

import numpy as np

def main():

    map = Map()
    pos = np.array([0,0])
    rot = np.array([-1, 2])

    #print(raytrace(pos, rot, 0.5, map).pos)

    #laser = Laser()
    #for timestamp, raycastHit in laser.laser_generator(pos, 100, map):
    #    print(raycastHit.dist)

    # Bozo filter for input args
    if len(argv) < 4:
        print('Usage:   %s <dataset> <output_dir> <use_odometry> [random_seed]' % argv[0])
        print('Example: %s exp2 output 1 9999' % argv[0])
        exit(1)

    print(argv[3])

    #TODO: handle this input better

    # Grab input args
    dataset = argv[1]
    use_odometry = True if int(argv[3]) else False
    seed =  int(argv[4])
    output_filename = argv[5] if len(argv) > 5 else "output.pgm"
    draw_trajectory = False if len(argv) > 6 and not int(argv[6]) else True
    
    print("dataset: " + dataset)
    print("use_odometry: " + str(use_odometry))
    print("seed: " + str(seed))
    print("output_filename: " + output_filename)
    print("draw_trajectory: " + str(draw_trajectory))

	# Load the data from the file, ignoring timestamps
    _, lidars, odometries = load_data('.', dataset)

    #print("lidars: ", len(lidars))
    #print("odometries: ", len(odometries))

    # Build a robot model if we want odometry
    robot = Robot() if use_odometry else None
        
    # Create a CoreSLAM object with laser params and optional robot object
    slam = RMHC_SLAM(Laser(), MAP_SIZE_PIXELS, MAP_SIZE_METERS, random_seed=seed) \
           if seed \
           else Deterministic_SLAM(Laser(), MAP_SIZE_PIXELS, MAP_SIZE_METERS)
           
    # Report what we're doing
    nscans = len(lidars)
    print('Processing %d scans with%s odometry / with%s particle filter...' % \
        (nscans, \
         '' if use_odometry else 'out', '' if seed else 'out'))
    progbar = ProgressBar(0, nscans, 80)
    
    # Start with an empty trajectory of positions
    trajectory = []

    # Start timing
    start_sec = time()
    
    # Loop over scans    
    for scanno in range(nscans):
    
        if use_odometry:
                  
            # Convert odometry to pose change (dxy_mm, dtheta_degrees, dt_seconds)
            velocities = robot.computePoseChange(odometries[scanno])
                                 
            # Update SLAM with lidar and velocities
            slam.update(lidars[scanno], velocities)
            
        else:
        
            # Update SLAM with lidar alone
            slam.update(lidars[scanno])

        # Get new position
        x_mm, y_mm, theta_degrees = slam.getpos()    
        
        # Add new position to trajectory
        trajectory.append((x_mm, y_mm))
        
        # Tame impatience
        progbar.updateAmount(scanno)
        stdout.write('\r%s' % str(progbar))
        stdout.flush()

    # Report elapsed time
    elapsed_sec = time() - start_sec
    print('\n%d scans in %f sec = %f scans / sec' % (nscans, elapsed_sec, nscans/elapsed_sec))
                    
                                
    # Create a byte array to receive the computed maps
    mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
    
    # Get final map    
    slam.getmap(mapbytes)
    
    if(draw_trajectory):

        # Put trajectory into map as black pixels
        for coords in trajectory:
                
            x_mm, y_mm = coords
                               
            x_pix = mm2pix(x_mm)
            y_pix = mm2pix(y_mm)
                                                                                              
            mapbytes[y_pix * MAP_SIZE_PIXELS + x_pix] = 0
                    
    # Save map and trajectory as PGM file    
    pgm_save(output_filename, mapbytes, (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS))
            
# Helpers ---------------------------------------------------------        

def mm2pix(mm):
    return int(mm / (MAP_SIZE_METERS * 1000. / MAP_SIZE_PIXELS))
    
                    
main()
