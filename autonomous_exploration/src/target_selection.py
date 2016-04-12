#!/usr/bin/env python

import rospy
import random
import math
import numpy as np
from robot_perception import RobotPerception

# Class for selecting the next best target
class TargetSelection:

    # Constructor
    def __init__(self):
		self.robot_perception = RobotPerception()

    #def selectTarget(self, ogm, coverage, robot_pose):
        
        ## The next target in pixels
        #next_target = [0, 0] 
        
        ## YOUR CODE HERE ------------------------------------------------------
        ## Here you must select the next target of the robot. The next target
        ## should exist in unoccupied and uncovered space. Thus, you must use the
        ## ogm and coverage variables or / and the robot pose. The easier way is to
        ## randomly select points of the map until one such point can be a target
        ## Of course you should try something smarter...!
        #print robot_pose
        #found = False
        #print ogm[1]
        #while not found:
          #x_rand = random.randint(0, ogm.shape[0] - 1)
          #y_rand = random.randint(0, ogm.shape[1] - 1)
          #if ogm[x_rand][y_rand] < 50:
            #next_target = [x_rand, y_rand]
            #found = True
        
        ## ---------------------------------------------------------------------

        #return next_target

    def selectNearestTarget(self, ogm, coverage, robot_pose):
        
        # The next target in pixels
        next_target = [0, 0]
        [rx, ry] = [\
            self.robot_perception.robot_pose['x_px'] - \
                    self.robot_perception.origin['x'] / self.robot_perception.resolution,\
            self.robot_perception.robot_pose['y_px'] - \
                    self.robot_perception.origin['y'] / self.robot_perception.resolution\
                    ]
                    
        distances = [] 
        new_data  = []
        for i in range(ogm.shape[0]-1):
			for j in range(ogm.shape[1]-1):
				if ogm[i][j] < 50 and coverage[i][j] < 100:
					new_data.append([i,j])
					distc = math.sqrt((rx - i)**2 + (ry - j)**2)
					distances.append(distc)

        index_min = np.argmin(distances)
        #print len(distances)
        #print index_min
        xt = new_data[index_min][0]
        yt = new_data[index_min][1]
        next_target = [xt, yt]
        
        return next_target
