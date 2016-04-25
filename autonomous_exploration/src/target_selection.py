#!/usr/bin/env python

import rospy
import random
import math
import numpy as np
from timeit import default_timer as timer
from robot_perception import RobotPerception

# Class for selecting the next best target
class TargetSelection:
    
    # Constructor
    def __init__(self):
        self.robot_perception = RobotPerception()

    def selectTarget(self, ogm, coverage, robot_pose):
        
        # The next target in pixels
        next_target = [0, 0] 
        
        # YOUR CODE HERE ------------------------------------------------------
        # Here you must select the next target of the robot. The next target
        # should exist in unoccupied and uncovered space. Thus, you must use the
        # ogm and coverage variables or / and the robot pose. The easier way is to
        # randomly select points of the map until one such point can be a target
        # Of course you should try something smarter...!
        print robot_pose
        found = False
        print ogm[1]
        while not found:
          x_rand = random.randint(0, ogm.shape[0] - 1)
          y_rand = random.randint(0, ogm.shape[1] - 1)
          if ogm[x_rand][y_rand] < 50:
            next_target = [x_rand, y_rand]
            found = True
        
        # ---------------------------------------------------------------------

        return next_target

    def selectNearestUncovered(self, ogm, coverage, robot_pose, select_another_target):

        print select_another_target
        
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
        
        for i in range(0, ogm.shape[0]-1, 3):
            for j in range(0, ogm.shape[1]-1, 3):
                ogm_part = ogm[i-3:i+3,j-3:j+3]
                cov_part = coverage[i-3:i+3,j-3:j+3]
                if coverage[i][j] != 100 and np.all(ogm_part <= 50) and np.any(cov_part == 100):
                    new_data.append([i,j])
                    distc = math.sqrt((rx - i)**2 + (ry - j)**2)
                    distances.append(distc)
        
        while(select_another_target != 0):
            index_min = np.argmin(distances)
            distances.pop(index_min)
            new_data.pop(index_min)
            select_another_target -= 1
        
        index_min = np.argmin(distances)
        xt = new_data[index_min][0]
        yt = new_data[index_min][1]
        next_target = [xt, yt]
        
        return next_target

    def selectNearestUnexplored(self, ogm, coverage, robot_pose, select_another_target):
        
        # The next target in pixels
        next_target = [0, 0]
        [rx, ry] = [\
            self.robot_perception.robot_pose['x_px'] - \
                    self.robot_perception.origin['x'] / self.robot_perception.resolution,\
            self.robot_perception.robot_pose['y_px'] - \
                    self.robot_perception.origin['y'] / self.robot_perception.resolution\
                    ]
        distances = []
        possible_targets  = []

        for i in range(0, ogm.shape[0]-1, 4):
            for j in range(0, ogm.shape[1]-1, 4):
                
                ogm_part = ogm[i-6:i+6,j-6:j+6]
                cov_part = coverage[i-6:i+6,j-6:j+6]
                
                mean_ogm_part = np.mean(ogm_part)
                var_ogm_part = np.var(ogm_part)
                ogm_part_51 = np.sum(ogm_part == 51) / float(np.size(ogm_part))
                
                if ogm[i][j] < 50 and np.all(ogm_part <= 51) \
                                  and np.all(cov_part != 100) \
                                  and ogm_part_51 >= 0.2 \
                                  and ogm_part_51 <= 0.7 \
                                  : #and var_ogm_part >= (10)**2 and np.any(ogm_part == 51) and and mean_ogm_part >= 30 and mean_ogm_part <= 35
                    possible_targets.append([i,j])
                    distc = math.sqrt((rx - i)**2 + (ry - j)**2)
                    distances.append(distc)
                    #print var_ogm_part
                    #print ogm_part
                    #print np.sum(ogm_part == 51)
                    #print np.size(ogm_part)
                    #print ogm_part_51
        #print possible_targets
        while(select_another_target != 0):
            index_min = np.argmin(distances)
            distances.pop(index_min)
            possible_targets.pop(index_min)
            select_another_target -= 1
        
        index_min = np.argmin(distances)
        xt = possible_targets[index_min][0]
        yt = possible_targets[index_min][1]
        next_target = [xt, yt]
        
        return next_target
        
    def selectNearestUnexploredRounded(self, ogm, coverage, robot_pose, select_another_target):
        
        # The next target in pixels
        next_target = [0, 0]
        [rx, ry] = [\
            self.robot_perception.robot_pose['x_px'] - \
                    self.robot_perception.origin['x'] / self.robot_perception.resolution,\
            self.robot_perception.robot_pose['y_px'] - \
                    self.robot_perception.origin['y'] / self.robot_perception.resolution\
                    ]
        distances = []
        possible_targets  = []
        goal_found = False

        for r in range(20, 500, 1):
            omega = 0.0
            while not goal_found and omega < 2 * math.pi:
                #print r , omega
                i = int(rx + r * math.sin(omega))
                j = int(ry + r * math.cos(omega))
                
                ogm_part = ogm[i-5:i+5,j-5:j+5]
                cov_part = coverage[i-1:i+1,j-1:j+1]
                
                mean_ogm_part = np.mean(ogm_part)
                var_ogm_part = np.var(ogm_part)
                ogm_part_51 = np.sum(ogm_part == 51) / float(np.size(ogm_part))
                
                if ogm[i][j] < 50 and np.all(ogm_part <= 51) \
                                  and np.all(cov_part != 100) \
                                  and ogm_part_51 >= 0.1 \
                                  and ogm_part_51 <= 0.5: # \
                                  #and var_ogm_part >= (10)**2: # and mean_ogm_part >= 30 and mean_ogm_part <= 35
                    if select_another_target == 0:
                        goal_found = True
                        next_target = [i, j]
                        print ogm_part
                    else:
                        print "Another Target!!!!!"
                        select_another_target -= 1
                        goal_found = False
                        omega = omega + 5 / float(r)
                else:
                    goal_found = False
                    omega = omega + 5 / float(r)
            if goal_found == True:
                break
        return next_target



    def selectBestTopology(self, ogm, coverage, robot_pose, select_another_target):
        
        # The next target in pixels
        next_target = [0, 0]
        [rx, ry] = [\
            self.robot_perception.robot_pose['x_px'] - \
                    self.robot_perception.origin['x'] / self.robot_perception.resolution,\
            self.robot_perception.robot_pose['y_px'] - \
                    self.robot_perception.origin['y'] / self.robot_perception.resolution\
                    ]
        goals = []
        topol_factor_goals = []
        
        for i in range(0, ogm.shape[0]-1, 10):
            for j in range(0, ogm.shape[1]-1, 10):
                ogm_part = ogm[i-10:i+10,j-10:j+10]
                dist = []
                if ogm[i][j] < 51 and coverage[i][j] != 100 and np.all(ogm_part <= 51):

                    m = i
                    n = j
                    while(ogm[m][n] <= 50):
                        m += 1
                    if ogm[m][n] != 51:
                        dist = np.append(dist, [m - i])
                    else:
                        dist = np.append(dist, 1000)
                    
                    m = i
                    n = j
                    while(ogm[m][n] <= 50):
                        n += 1
                    if ogm[m][n] != 51:
                        dist = np.append(dist, [n - j])
                    else:
                        dist = np.append(dist, 1000)
                    
                    m = i
                    n = j
                    while(ogm[m][n] <= 50):
                        m -= 1
                    if ogm[m][n] != 51:
                        dist = np.append(dist, [i - m])
                    else:
                        dist = np.append(dist, 1000)
                    
                    m = i
                    n = j
                    while(ogm[m][n] <= 50):
                        n -= 1
                    if ogm[m][n] != 51:
                        dist = np.append(dist, [j - n])
                    else:
                        dist = np.append(dist, 1000)
                    
                    m = i
                    n = j
                    while(ogm[m][n] <= 50):
                        n += 1
                        m += 1
                    if ogm[m][n] != 51:
                        dist = np.append(dist, [math.sqrt(2 * (m - i)**2)])
                    else:
                        dist = np.append(dist, 1000)
                        
                    m = i
                    n = j
                    while(ogm[m][n] <= 50):
                        n += 1
                        m -= 1
                    if ogm[m][n] != 51:
                        dist = np.append(dist, [math.sqrt(2 * (m - i)**2)])
                    else:
                        dist = np.append(dist, 1000)
                        
                    m = i
                    n = j
                    while(ogm[m][n] <= 50):
                        n -= 1
                        m += 1
                    if ogm[m][n] != 51:
                        dist = np.append(dist, [math.sqrt(2 * (m - i)**2)])
                    else:
                        dist = np.append(dist, 1000)
                        
                    m = i
                    n = j
                    while(ogm[m][n] <= 50):
                        n -= 1
                        m -= 1
                    if ogm[m][n] != 51:
                        dist = np.append(dist, [math.sqrt(2 * (m - i)**2)])
                    else:
                        dist = np.append(dist, 1000)
                    
                    mean_sum_dist = np.mean(np.sum(dist))
                    topol_factor_goals.append([mean_sum_dist])
                    goals.append([i, j])
        
        while(select_another_target != 0):
            index_min = np.argmin(topol_factor_goals)
            topol_factor_goals.pop(index_min)
            goals.pop(index_min)
            select_another_target -= 1
            
        index_min = np.argmin(topol_factor_goals)
        xt = goals[index_min][0]
        yt = goals[index_min][1]
        next_target = [xt, yt]
        
        return next_target
