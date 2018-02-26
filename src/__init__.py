#coding:utf-8

# made by Yuki Katsumata 2018.1.15

import rospy
import math
import numpy as np
from time import  sleep
import sys, subprocess
import os.path

TRIALNAME = sys.argv[1]

MAP_TOPIC = "/map"
DATASET_FOLDER = "PATH/catkin_ws/src/spco_mapping/data/"
Descriptor = "CNN_Place205"
DATASET_NAME = "room1dk5_all"

# The upper limit number of spatial concepts
L = 120
CNN_SIZE = 205

# HyperParameter
ALPHA = 10000000.0
GAMMA = 2.1
CHI = 1.0
BETA = 0.1

# Iteration of Gibbs sampling
ITERATION = 5000
