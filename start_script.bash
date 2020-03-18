#!/bin/bash

# Dataset path
DATA_DIR="BreezySLAM/examples"

# Output path
OUTPUT_DIR="output"

# Config
Draw_TRAJECTORY=0

# Set these for different experiments
DATASET="exp1"
USE_ODOMETRY=0
RANDOM_SEED=9999

./run_slam.py $DATA_DIR/$DATASET $USE_ODOMETRY $RANDOM_SEED $OUTPUT_DIR/$DATASET.pgm $Draw_TRAJECTORY
