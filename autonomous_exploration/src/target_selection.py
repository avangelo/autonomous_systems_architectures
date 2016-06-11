#!/usr/bin/env python

import rospy
import random
import math
import numpy as np
from timeit import default_timer as timer
from robot_perception import RobotPerception
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

# Class for selecting the next best target
class TargetSelection:
    
    # Constructor
    def __init__(self):
        self.robot_perception = RobotPerception()
        self.goals_position = []
        self.goals_value = []
        self.omega = 0.0
        self.radius = 0
        self.brushfire_matrix = np.zeros((1500, 1500))
        self.brushfire_matrix.fill(-1)
        self.current_step_brushfire_positions = []
        self.next_step_brushfire_positions = []
        self.brushfire_step = 0
        
        # ROS Publisher for the subtargets
        self.subtargets_publisher = rospy.Publisher(rospy.get_param('goals_pub_topic'),\
            MarkerArray, queue_size = 10)

    def show_targets(self):
    
    # Publish the targets for visualization purposes
        ros_goals = MarkerArray()
        #print self.goals_position
        c = 0
        for s in self.goals_position:
            c += 1
            st = Marker()
            st.header.frame_id = "map"
            st.ns = 'as_namespace'
            st.id = c
            st.header.stamp = rospy.Time(0)
            st.type = 2 # sphere
            st.action = 0 # add
            st.pose.position.x = s[0] * self.robot_perception.resolution + \
                    self.robot_perception.origin['x']
            st.pose.position.y = s[1] * self.robot_perception.resolution + \
                    self.robot_perception.origin['y']

            st.color.r = 0.8
            st.color.g = 0
            st.color.b = 0.5
            st.color.a = 0.8
            st.scale.x = 0.2
            st.scale.y = 0.2
            st.scale.z = 0.2
            ros_goals.markers.append(st)

        self.subtargets_publisher.publish(ros_goals)

    def selectTarget(self, ogm, coverage, robot_pose):
        
        # The next target in pixels
        next_target = [0, 0] 
        
        # YOUR CODE HERE ------------------------------------------------------
        # Here you must select the next target of the robot. The next target
        # should exist in unoccupied and uncovered space. Thus, you must use the
        # ogm and coverage variables or / and the robot pose. The easier way is to
        # randomly select points of the map until one such point can be a target
        # Of course you should try something smarter...!
        found = False
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
                    self.goals_position.append([i,j])
                    distc = math.sqrt((rx - i)**2 + (ry - j)**2)
                    self.goals_value.append(distc)
        
        while(select_another_target != 0):
            index_min = np.argmin(self.goals_value)
            self.goals_value.pop(index_min)
            self.goals_position.pop(index_min)
            select_another_target -= 1
            
        # Publish the targets for visualization purposes
        self.show_targets()
        
        index_min = np.argmin(self.goals_value)
        xt = self.goals_position[index_min][0]
        yt = self.goals_position[index_min][1]
        next_target = [xt, yt]
        
        return next_target
    
    def selectNearestUncoveredBrush(self, ogm, coverage, robot_pose, select_another_target):
        
        # The next target in pixels
        next_target = [0, 0]
        [rx, ry] = [\
            self.robot_perception.robot_pose['x_px'] - \
                    self.robot_perception.origin['x'] / self.robot_perception.resolution,\
            self.robot_perception.robot_pose['y_px'] - \
                    self.robot_perception.origin['y'] / self.robot_perception.resolution\
                    ]
        rx = int(rx)
        ry = int(ry)
        #print rx, ry
        goal_found = False
        
        if select_another_target == 0:
            #print 10
            self.current_step_brushfire_positions = []
            self.next_step_brushfire_positions = []
            self.brushfire_matrix.fill(-1)
            self.brushfire_matrix[rx][ry] = 0
            self.brushfire_step = 0
            self.current_step_brushfire_positions.append([rx, ry])
        
        while(goal_found == False):
            if len(self.next_step_brushfire_positions) == 0:
                self.brushfire_step += 1
                #print 1
                for position in self.current_step_brushfire_positions:
                    for w in range(0, 359, 45):
                        x = position[0] + int(round(math.cos(w * math.pi / 180)))
                        y = position[1] + int(round(math.sin(w * math.pi / 180)))
                        if self.brushfire_matrix[x][y] == -1 and ogm[x][y] < 51:
                            self.brushfire_matrix[x][y] = self.brushfire_step
                            self.next_step_brushfire_positions.append([x, y])
                    #print self.brushfire_matrix[rx-7:rx+7, ry-7:ry+7]
                self.current_step_brushfire_positions = self.next_step_brushfire_positions[:]
            brushfire_position_test = self.next_step_brushfire_positions[:]
            
            if self.next_step_brushfire_positions == []:
                next_target = [0, 0]
                break
            
            for position in self.next_step_brushfire_positions:
                brushfire_position_test.remove(position)
                i = position[0]
                j = position[1]
                ogm_part = ogm[i-6:i+6,j-6:j+6]
                cov_part = coverage[i-6:i+6,j-6:j+6]
                
                if coverage[i][j] != 100 and np.all(ogm_part <= 50) and np.any(cov_part == 100):
                    next_target = [i, j]
                    goal_found = True
                    #self.next_step_brushfire_positions = brushfire_position_test
                    #print brushfire_position_test
                    break
                    
            self.next_step_brushfire_positions = brushfire_position_test
        #print ogm[i-7:i+7, j-7:j+7]
        #print self.brushfire_matrix[rx-7:rx+7, ry-7:ry+7]
        return next_target

    def selectNearestUncoveredCircle(self, ogm, coverage, robot_pose, select_another_target):
        
        # The next target in pixels
        next_target = [0, 0]
        [rx, ry] = [\
            self.robot_perception.robot_pose['x_px'] - \
                    self.robot_perception.origin['x'] / self.robot_perception.resolution,\
            self.robot_perception.robot_pose['y_px'] - \
                    self.robot_perception.origin['y'] / self.robot_perception.resolution\
                    ]
        
        goal_found = False
        step_in_circle = 2
        
        if select_another_target == 0:
            self.radius = 19
            self.omega = 0.0

        for r in range(self.radius, 500, 1):
            omega = self.omega
            while goal_found == False and omega < 2 * math.pi:
                i = int(rx + r * math.sin(omega))
                j = int(ry + r * math.cos(omega))
                
                ogm_part = ogm[i-5:i+5,j-5:j+5]
                cov_part = coverage[i-1:i+1,j-1:j+1]
                
                mean_ogm_part = np.mean(ogm_part)
                var_ogm_part = np.var(ogm_part)
                ogm_part_51 = np.sum(ogm_part == 51) / float(np.size(ogm_part))
                
                if coverage[i][j] != 100 and np.all(ogm_part <= 50) and np.any(cov_part == 100):
                    goal_found = True
                    next_target = [i, j]
                    self.radius = r
                    self.omega = omega + step_in_circle / float(r)
                    
                    #if select_another_target == 0:
                        #goal_found = True
                        #next_target = [i, j]
                    #else:
                        #select_another_target -= 1
                        #goal_found = False
                        #omega = omega + step_in_circle / float(r)
                else:
                    goal_found = False
                    omega = omega + step_in_circle / float(r)
            if goal_found == True:
                break
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
                    self.goals_position.append([i,j])
                    distc = math.sqrt((rx - i)**2 + (ry - j)**2)
                    self.goals_value.append(distc)
        
        while(select_another_target != 0):
            index_min = np.argmin(self.goals_value)
            self.goals_value.pop(index_min)
            self.goals_position.pop(index_min)
            select_another_target -= 1
        
        # Publish the targets for visualization purposes
        self.show_targets()
        
        index_min = np.argmin(self.goals_value)
        xt = self.goals_position[index_min][0]
        yt = self.goals_position[index_min][1]
        next_target = [xt, yt]
        
        return next_target
    
    def selectNearestUnexploredBrush(self, ogm, coverage, robot_pose, select_another_target):
        
        # The next target in pixels
        next_target = [0, 0]
        [rx, ry] = [\
            self.robot_perception.robot_pose['x_px'] - \
                    self.robot_perception.origin['x'] / self.robot_perception.resolution,\
            self.robot_perception.robot_pose['y_px'] - \
                    self.robot_perception.origin['y'] / self.robot_perception.resolution\
                    ]
        rx = int(rx)
        ry = int(ry)
        print rx, ry
        goal_found = False
        
        if select_another_target == 0:
            #print 10
            self.current_step_brushfire_positions = []
            self.next_step_brushfire_positions = []
            self.brushfire_matrix.fill(-1)
            self.brushfire_matrix[rx][ry] = 0
            self.brushfire_step = 0
            self.current_step_brushfire_positions.append([rx, ry])
        
        while(goal_found == False):
            if len(self.next_step_brushfire_positions) == 0:
                self.brushfire_step += 1
                #print 1
                for position in self.current_step_brushfire_positions:
                    for w in range(0, 359, 45):
                        x = position[0] + int(round(math.cos(w * math.pi / 180)))
                        y = position[1] + int(round(math.sin(w * math.pi / 180)))
                        if self.brushfire_matrix[x][y] == -1 and ogm[x][y] < 51:
                            self.brushfire_matrix[x][y] = self.brushfire_step
                            self.next_step_brushfire_positions.append([x, y])
                    #print self.brushfire_matrix[rx-7:rx+7, ry-7:ry+7]
                self.current_step_brushfire_positions = self.next_step_brushfire_positions[:]
            brushfire_position_test = self.next_step_brushfire_positions[:]
            
            if self.next_step_brushfire_positions == []:
                next_target = [0, 0]
                break
            
            for position in self.next_step_brushfire_positions:
                brushfire_position_test.remove(position)
                i = position[0]
                j = position[1]
                ogm_part = ogm[i-6:i+6,j-6:j+6]
                ogm_part_51 = np.sum(ogm_part == 51) / float(np.size(ogm_part))
                ogm_small_part = ogm[i-3:i+3,j-3:j+3]
                
                #if ogm_part_51 > 0.2 and ogm_part_51 < 0.7 and np.all(ogm_part <= 51) and np.all(ogm_small_part < 51) and coverage[i][j] != 100:
                if ogm_part_51 > 0.1 and np.all(ogm_part <= 51) and np.all(ogm_small_part < 51) and coverage[i][j] != 100:
                    next_target = [i, j]
                    goal_found = True
                    #self.next_step_brushfire_positions = brushfire_position_test
                    #print brushfire_position_test
                    self.goals_position.append([i, j])
                    # Publish the targets for visualization purposes
                    self.show_targets()
                    #print ogm_part
                    break
                    
            self.next_step_brushfire_positions = brushfire_position_test
        #print ogm[i-7:i+7, j-7:j+7]
        #print self.brushfire_matrix[rx-7:rx+7, ry-7:ry+7]
        return next_target
    
    def selectNearestUnexploredCircle(self, ogm, coverage, robot_pose, select_another_target):
        
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
        step_in_circle = 2
        
        if select_another_target == 0:
            self.radius = 19
            self.omega = 0.0
        
        for r in range(self.radius, 500, 1):
            omega = self.omega
            #print r
            while goal_found == False and omega < 2 * math.pi:
                #print omega ,
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
                                  and ogm_part_51 <= 0.9: 
                                  #and var_ogm_part >= (10)**2: # and mean_ogm_part >= 30 and mean_ogm_part <= 35
                    goal_found = True
                    next_target = [i, j]
                    self.radius = r
                    self.omega = omega + step_in_circle / float(r)
                    
                    #if select_another_target == 0:
                        #goal_found = True
                        #next_target = [i, j]
                    #else:
                        #select_another_target -= 1
                        #goal_found = False
                        #omega = omega + 5 / float(r)
                    
                else:
                    goal_found = False
                    omega = omega + step_in_circle / float(r)
            if goal_found == True:
                break
        return next_target
        
    def selectBestTopology2(self, ogm, coverage, robot_pose, select_another_target):
        
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
        unexplored_ray_value = 200
        
        for i in range(0, ogm.shape[0]-1, 10):
            for j in range(0, ogm.shape[1]-1, 10):
                ogm_part = ogm[i-5:i+5,j-5:j+5]
                dist = []
                if ogm[i][j] < 51 and coverage[i][j] != 100 and np.all(ogm_part <= 51):

                    # Ray axis +x
                    m = i
                    n = j
                    rep = 0
                    while(ogm[m][n] < 51 and rep <= unexplored_ray_value):
                        m += 1
                        rep += 1
                    if ogm[m][n] != 51:
                        dist = np.append(dist, [m - i])
                    else:
                        dist = np.append(dist, unexplored_ray_value)
                        
                    # Ray axis +y
                    m = i
                    n = j
                    rep = 0
                    while(ogm[m][n] < 51 and rep <= unexplored_ray_value):
                        n += 1
                        rep += 1
                    if ogm[m][n] != 51:
                        dist = np.append(dist, [n - j])
                    else:
                        dist = np.append(dist, unexplored_ray_value)
                    
                    # Ray axis -x
                    m = i
                    n = j
                    rep = 0
                    while(ogm[m][n] < 51 and rep <= unexplored_ray_value):
                        m -= 1
                        rep += 1
                    if ogm[m][n] != 51:
                        dist = np.append(dist, [i - m])
                    else:
                        dist = np.append(dist, unexplored_ray_value)
                    
                    # Ray axis -y
                    m = i
                    n = j
                    rep = 0
                    while(ogm[m][n] < 51 and rep <= unexplored_ray_value):
                        n -= 1
                        rep += 1
                    if ogm[m][n] != 51:
                        dist = np.append(dist, [j - n])
                    else:
                        dist = np.append(dist, unexplored_ray_value)
                    
                    # Ray axis +x and +y
                    m = i
                    n = j
                    rep = 0
                    while(ogm[m][n] < 51 and rep <= unexplored_ray_value):
                        m += 1
                        n += 1
                        rep += 1
                    if ogm[m][n] != 51:
                        dist = np.append(dist, [math.sqrt(2 * (m - i)**2)])
                    else:
                        dist = np.append(dist, unexplored_ray_value)
                    
                    # Ray axis +x and -y
                    m = i
                    n = j
                    rep = 0
                    while(ogm[m][n] < 51 and rep <= unexplored_ray_value):
                        m += 1
                        n -= 1
                        rep += 1
                    if ogm[m][n] != 51:
                        dist = np.append(dist, [math.sqrt(2 * (m - i)**2)])
                    else:
                        dist = np.append(dist, unexplored_ray_value)
                    
                    # Ray axis -x and +y
                    m = i
                    n = j
                    rep = 0
                    while(ogm[m][n] < 51 and rep <= unexplored_ray_value):
                        m -= 1
                        n += 1
                        rep += 1
                    if ogm[m][n] != 51:
                        dist = np.append(dist, [math.sqrt(2 * (m - i)**2)])
                    else:
                        dist = np.append(dist, unexplored_ray_value)
                    
                    # Ray axis -x and -y
                    m = i
                    n = j
                    rep = 0
                    while(ogm[m][n] < 51 and rep <= unexplored_ray_value):
                        m -= 1
                        n -= 1
                        rep += 1
                    if ogm[m][n] != 51:
                        dist = np.append(dist, [math.sqrt(2 * (m - i)**2)])
                    else:
                        dist = np.append(dist, unexplored_ray_value)

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
        
    def selectBestTopology(self, ogm, coverage, robot_pose, select_another_target):
        
        # The next target in pixels
        next_target = [0, 0]
        [rx, ry] = [\
            self.robot_perception.robot_pose['x_px'] - \
                    self.robot_perception.origin['x'] / self.robot_perception.resolution,\
            self.robot_perception.robot_pose['y_px'] - \
                    self.robot_perception.origin['y'] / self.robot_perception.resolution\
                    ]
        unexplored_ray_value = 200
        max_ray = 5
        
        if select_another_target == 0:
            self.goals_position = []
            self.goals_value = []
            for i in range(0, ogm.shape[0]-1, 10):
                for j in range(0, ogm.shape[1]-1, 10):
                    
                    ogm_part = ogm[i-6:i+6,j-6:j+6]
                    #cov_part = coverage[i-6:i+6,j-6:j+6]
                    
                    dist = []
                    if coverage[i][j] != 100 and np.all(ogm_part < 51):
                        for w in range(0, 359, 45):
                            w = w * 2 * math.pi / 360
                            x = i
                            y = j
                            length_ray = 0
                            next_pixel = 0
                            while(ogm[x][y] < 51 and length_ray <= max_ray):
                                next_pixel += 1
                                x = i + next_pixel * math.cos(w)
                                y = j + next_pixel * math.sin(w)
                                length_ray = math.sqrt((x - i)**2 + (y - j)**2) * self.robot_perception.resolution
                            if ogm[x][y] == 51:
                                dist = np.append(dist, max_ray / self.robot_perception.resolution)
                            else:
                                dist = np.append(dist, length_ray / self.robot_perception.resolution)
                        #print dist
                        sum_dist = np.sum(dist)
                        #print i, j
                        #print mean_sum_dist
                        #wait = input("PRESS 1 TO CONTINUE.")
                        self.goals_value.append([sum_dist])
                        self.goals_position.append([i, j])
        else:
            while(select_another_target != 0):
                if self.goals_value == []:
                    break
                index_min = np.argmin(self.goals_value)
                self.goals_value.pop(index_min)
                self.goals_position.pop(index_min)
                select_another_target -= 1
        
        # Publish the targets for visualization purposes
        self.show_targets()
        
        if self.goals_value == []:
            next_target = [0, 0]
        else:
            index_min = np.argmin(self.goals_value)
            xt = self.goals_position[index_min][0]
            yt = self.goals_position[index_min][1]
            next_target = [xt, yt]
        
        return next_target
        
    def selectNextBestView2(self, ogm, coverage, robot_pose, select_another_target):
        
        # The next target in pixels
        next_target = [0, 0]
        [rx, ry] = [\
            self.robot_perception.robot_pose['x_px'] - \
                    self.robot_perception.origin['x'] / self.robot_perception.resolution,\
            self.robot_perception.robot_pose['y_px'] - \
                    self.robot_perception.origin['y'] / self.robot_perception.resolution\
                    ]
        possible_targets  = []
        useful_areas = []
        for i in range(0, ogm.shape[0]-1, 10):
            for j in range(0, ogm.shape[1]-1, 10):
                
                ogm_part = ogm[i-20:i+20,j-20:j+20]
                ogm_part_51 = np.sum(ogm_part == 51) / float(np.size(ogm_part))
                ogm_part_100 = np.sum(ogm_part == 100) / float(np.size(ogm_part))
                ogm_small_part = ogm[i-3:i+3,j-3:j+3]
                
                if ogm[i][j] < 50 and ogm_part_51 >= 0.4 and ogm_part_51 <= 0.75 and np.all(ogm_small_part != 100):
                    useful_rays = []
                    for w in range(0, 359, 3):
                        w = w * 2 * math.pi / 360
                        next_pixel = 0
                        x = i
                        y = j
                        ray_length = 0
                        while ray_length <= 10 and ogm[x][y] < 51:
                            next_pixel += 1
                            x = i + next_pixel * math.cos(w)
                            y = j + next_pixel * math.sin(w)
                            if x >= 1500 or y >= 1500:
                                break
                            ray_length = math.sqrt((i - x)**2 + (j - y)**2) * self.robot_perception.resolution
                        if ogm[x][y] == 51:
                            x_entrance = x
                            y_entrance = y
                            while ray_length <= 10 and ogm[x][y] == 51:
                                next_pixel += 1
                                x = i + next_pixel * math.cos(w)
                                y = j + next_pixel * math.sin(w)
                                if x >= 1500 or y >= 1500:
                                    break
                                ray_length = math.sqrt((i - x)**2 + (j - y)**2) * self.robot_perception.resolution
                                useful_ray_length = math.sqrt((x_entrance - x)**2 + (y_entrance - y)**2) * self.robot_perception.resolution
                            useful_rays.append([useful_ray_length])
                    useful_area = np.sum(useful_rays)
                    useful_areas.append([useful_area])
                    possible_targets.append([i, j])
        while(select_another_target != 0):
            index_max = np.argmax(useful_areas)
            useful_areas.pop(index_max)
            possible_targets.pop(index_max)
            select_another_target -= 1
        
        index_max = np.argmax(useful_areas)
        xt = possible_targets[index_max][0]
        yt = possible_targets[index_max][1]
        next_target = [xt, yt]
        
        return next_target
        
    def selectNextBestView(self, ogm, coverage, robot_pose, select_another_target):
            
        # The next target in pixels
        next_target = [0, 0]
        [rx, ry] = [\
            self.robot_perception.robot_pose['x_px'] - \
                    self.robot_perception.origin['x'] / self.robot_perception.resolution,\
            self.robot_perception.robot_pose['y_px'] - \
                    self.robot_perception.origin['y'] / self.robot_perception.resolution\
                    ]
        possible_targets  = []
        useful_areas = []
        if select_another_target == 0:
            self.goals_position = []
            self.goals_value = []
            for i in range(0, ogm.shape[0]-1, 3):
                for j in range(0, ogm.shape[1]-1, 3):
                    
                    #ogm_part = ogm[i-10:i+10,j-10:j+10]
                    #ogm_small_part = ogm[i-7:i+7,j-7:j+7]
                    
                    ogm_part = ogm[i-8:i+8,j-8:j+8]
                    ogm_small_part = ogm[i-4:i+4,j-4:j+4]
                    ogm_part_51 = np.sum(ogm_part == 51) / float(np.size(ogm_part))
                    
                    
                    #if ogm[i][j] < 51 and ogm_part_51 > 0.1 and ogm_part_51 < 0.3 and np.all(ogm_small_part <= 51) and coverage[i][j] != 100:
                    if ogm_part_51 > 0.1 and np.all(ogm_part <= 51) and np.all(ogm_small_part < 51) and coverage[i][j] != 100:
                        useful_rays = []
                        #print i, j
                        for w in range(0, 359, 5):
                            w = w * 2 * math.pi / 360
                            next_pixel = 0
                            x = i
                            y = j
                            ray_length = 0
                            useful_ray_length = 0
                            while ray_length <= 5:
                                next_pixel += 2
                                x = i + next_pixel * math.cos(w)
                                y = j + next_pixel * math.sin(w)
                                #print x, y
                                if x >= ogm.shape[0]-1 or y >= ogm.shape[1]-1 or ogm[x][y] > 51:
                                    break
                                if ogm[x][y] == 51:
                                    useful_ray_length += 1
                                ray_length = math.sqrt((i - x)**2 + (j - y)**2) * self.robot_perception.resolution
                                #print ray_length
                            #wait = input("PRESS 1 TO CONTINUE.")
                            useful_rays.append([useful_ray_length])
                        useful_area = np.sum(useful_rays)
                        if useful_area > 500:
                            #print useful_area
                            #wait = input("PRESS 1 TO CONTINUE.")
                            self.goals_value.append([useful_area])
                            self.goals_position.append([i, j])
            print self.goals_value
        else:
            while(select_another_target != 0):
                print "Select another target"
                if self.goals_value == []:
                    break
                index_max = np.argmax(self.goals_value)
                self.goals_value.pop(index_max)
                self.goals_position.pop(index_max)
                select_another_target -= 1
                
        print len(self.goals_position)
        print self.goals_position

        # Publish the targets for visualization purposes
        self.show_targets()
        
        if self.goals_value == []:
            next_target = [0, 0]
        else:
            index_max = np.argmax(self.goals_value)
            xt = self.goals_position[index_max][0]
            yt = self.goals_position[index_max][1]
            next_target = [xt, yt]
            
        return next_target
