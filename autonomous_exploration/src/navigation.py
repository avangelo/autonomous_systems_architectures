#!/usr/bin/env python

import rospy
import math
import time
import numpy as np
from robot_perception import RobotPerception
from target_selection import TargetSelection
from path_planning import PathPlanning

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

# Class for implementing the navigation module of the robot
class Navigation:

    # Constructor
    def __init__(self):

        # Initializations
        self.robot_perception = RobotPerception()
        self.target_selection = TargetSelection()
        self.path_planning = PathPlanning()

        print "AOUA"
        # Check if the robot moves with target or just wanders
        self.move_with_target = rospy.get_param("calculate_target")

        # Flag to check if the vehicle has a target or not
        self.target_exists = False
        self.select_another_target = 0
        self.inner_target_exists = False

        # Container for the current path
        self.path = []
        # Container for the subgoals in the path
        self.subtargets = []

        # Container for the next subtarget. Holds the index of the next subtarget
        self.next_subtarget = 0

        # Check if subgoal is reached via a timer callback
        rospy.Timer(rospy.Duration(0.10), self.checkTarget)
        
        # Read the target function
        self.target_selector = rospy.get_param("target_selector")
        print "The selected target function is " + self.target_selector

        # ROS Publisher for the path
        self.path_publisher = rospy.Publisher(rospy.get_param('path_pub_topic'), \
            Path, queue_size = 10)
        
        # ROS Publisher for the subtargets
        self.subtargets_publisher = rospy.Publisher(rospy.get_param('subgoals_pub_topic'),\
            MarkerArray, queue_size = 10)
        
        # ROS Publisher for the current target
        self.current_target_publisher = rospy.Publisher(rospy.get_param('curr_target_pub_topic'),\
            Marker, queue_size = 10)
        
        # Time Initialization
        self.start_time = time.time()
        self.end_time = 0
        self.time_limit = rospy.get_param("time_limit")
        self.time_path_planning = 0
        self.time_coverage = 0
        self.time_target_selection = 0
        self.time_exploration = 0
        self.time_exploration_coverage = 0
        self.time_exploration_path_planning = 0
        self.time_exploration_target_selection = 0
        self.num_of_targets = 0
        
    def checkTarget(self, event):
        # Check if we have a target or if the robot just wanders
        if self.inner_target_exists == False or self.move_with_target == False or\
                self.next_subtarget == len(self.subtargets):
          return
        # Get the robot pose in pixels
        [rx, ry] = [\
            self.robot_perception.robot_pose['x_px'] - \
                    self.robot_perception.origin['x'] / self.robot_perception.resolution,\
            self.robot_perception.robot_pose['y_px'] - \
                    self.robot_perception.origin['y'] / self.robot_perception.resolution\
                    ]

        # YOUR CODE HERE ------------------------------------------------------
        # Here the next subtarget is checked for proximity. If the robot is too
        # close to the subtarget it is considered approached and the next
        # subtarget is assigned as next. However there are cases where the
        # robot misses one subtarget due to obstacle detection. Enhance the below
        # functionality by checking all the subgoals (and the final goal) for
        # proximity and assign the proper next subgoal

        # Find the distance between the robot pose and the next subtarget
        dist = math.hypot(\
            rx - self.subtargets[self.next_subtarget][0], \
            ry - self.subtargets[self.next_subtarget][1])

        # Check if distance is less than 7 px (14 cm)
        if dist < 7:
          print "Sub target reached!"
          self.next_subtarget += 1

          # Check if the final subtarget has been approached
          if self.next_subtarget == len(self.subtargets):
            print "Final goal reached!"
            self.target_exists = False

        # ---------------------------------------------------------------------

        # Publish the current target
        if self.next_subtarget == len(self.subtargets):
            return
        st = Marker()
        st.header.frame_id = "map"
        st.ns = 'as_curr_namespace'
        st.id = 0
        st.header.stamp = rospy.Time(0)
        st.type = 1 # arrow
        st.action = 0 # add
        st.pose.position.x = self.subtargets[self.next_subtarget][0]\
                * self.robot_perception.resolution + \
                self.robot_perception.origin['x']
        st.pose.position.y = self.subtargets[self.next_subtarget][1]\
                * self.robot_perception.resolution + \
                self.robot_perception.origin['y']

        st.color.r = 0
        st.color.g = 0
        st.color.b = 0.8
        st.color.a = 0.8
        st.scale.x = 0.2
        st.scale.y = 0.2
        st.scale.z = 0.2

        self.current_target_publisher.publish(st)

    # Function that seelects the next target, produces the path and updates
    # the coverage field. This is called from the speeds assignment code, since
    # it contains timer callbacks
    def selectTarget(self):
        # IMPORTANT: The robot must be stopped if you call this function until
        # it is over
        # Check if we have a map
        while self.robot_perception.have_map == False:
          return
        
        print "Navigation: Producing new target"
        # We are good to continue the exploration
        # Make this true in order not to call it again from the speeds assignment
        
        if  self.target_selector == "nearest_unexplored_brush" or self.target_selector == "next_best_view" or self.target_selector == "best_topology" or self.target_selector == "combi":
            if self.select_another_target == 0:
                
                self.num_of_targets += 1
                time.sleep(0)
            
        self.target_exists = True
        
        
        start = time.time()
        
        if self.select_another_target == 0:
            # Manually update the coverage field
            self.robot_perception.updateCoverage()
            print "Navigation: Coverage updated"
          
        end = time.time()
        print "Time till now(coverage update):", self.time_coverage
        self.time_coverage += (end - start)
        print end - start
        print self.time_coverage
        
        
        # Gets copies of the map and coverage
        local_ogm = self.robot_perception.getMap()
        local_ros_ogm = self.robot_perception.getRosMap()
        local_coverage = self.robot_perception.getCoverage()
        print "Got the map and Coverage"
        
        
        # Call the target selection function to select the next best goal
        # Choose target function
        
        start = time.time()
        
        if self.time_limit == 0:
            if self.target_selector == "random":
                target = self.target_selection.selectTarget(\
                    local_ogm,\
                    local_coverage,\
                    self.robot_perception.robot_pose)
            elif self.target_selector == "nearest_uncovered":
                target = self.target_selection.selectNearestUncovered(\
                    local_ogm,\
                    local_coverage,\
                    self.robot_perception.robot_pose,\
                    self.select_another_target)
            elif self.target_selector == "nearest_uncovered_brush":
                target = self.target_selection.selectNearestUncoveredBrush(\
                    local_ogm,\
                    local_coverage,\
                    self.robot_perception.robot_pose,\
                    self.select_another_target,\
                    self.robot_perception.origin,\
                    self.robot_perception.resolution)
            elif self.target_selector == "nearest_uncovered_circle":
                target = self.target_selection.selectNearestUncoveredCircle(\
                    local_ogm,\
                    local_coverage,\
                    self.robot_perception.robot_pose,\
                    self.select_another_target)
            elif self.target_selector == "nearest_unexplored":
                target = self.target_selection.selectNearestUnexplored(\
                    local_ogm,\
                    local_coverage,\
                    self.robot_perception.robot_pose,\
                    self.select_another_target)
            elif self.target_selector == "nearest_unexplored_brush":
                target = self.target_selection.selectNearestUnexploredBrush(\
                    local_ogm,\
                    local_coverage,\
                    self.robot_perception.robot_pose,\
                    self.select_another_target,\
                    self.robot_perception.origin,\
                    self.robot_perception.resolution)
            elif self.target_selector == "nearest_unexplored_circle":
                target = self.target_selection.selectNearestUnexploredCircle(\
                    local_ogm,\
                    local_coverage,\
                    self.robot_perception.robot_pose,\
                    self.select_another_target)
            elif self.target_selector == "best_topology":
                target = self.target_selection.selectBestTopology(\
                    local_ogm,\
                    local_coverage,\
                    self.robot_perception.robot_pose,\
                    self.select_another_target,\
                    self.robot_perception.origin,\
                    self.robot_perception.resolution)
            elif self.target_selector == "next_best_view":
                target = self.target_selection.selectNextBestView(\
                    local_ogm,\
                    local_coverage,\
                    self.robot_perception.robot_pose,\
                    self.select_another_target,\
                    self.robot_perception.origin,\
                    self.robot_perception.resolution)
            elif self.target_selector == "combi":
                target = self.target_selection.selectCombi(\
                    local_ogm,\
                    local_coverage,\
                    self.robot_perception.robot_pose,\
                    self.select_another_target,\
                    self.robot_perception.origin,\
                    self.robot_perception.resolution)
            else:
                print "Target function selected doesn't exist"
            
            if target == [0, 0]:
                
                if self.target_selector == "next_best_view" or self.target_selector == "nearest_unexplored_brush":
                    print "Can't find new unexplored target. Calling Nearest Uncovered Brushfire"
                    self.target_selector = "nearest_uncovered_brush"
                    target = self.target_selection.selectNearestUncoveredBrush(\
                        local_ogm,\
                        local_coverage,\
                        self.robot_perception.robot_pose,\
                        self.select_another_target,\
                        self.robot_perception.origin,\
                        self.robot_perception.resolution)
                    end_time_exploration = time.time()
                    self.time_exploration = end_time_exploration - self.start_time
                    self.time_exploration_coverage = self.time_coverage
                    self.time_exploration_path_planning = self.time_path_planning
                    self.time_exploration_target_selection = self.time_target_selection
                    print "The time needed to explore the area is:"
                    print self.time_exploration
                else:
                    print "Can't find new target. The exploration is complete"
                    self.end_time = time.time()
                    print "Temporal Cost Overall:"
                    print self.end_time - self.start_time
                    print "Coverage Update Tine Overall:"
                    print self.time_coverage
                    print "Path Planning Time Overall:"
                    print self.time_path_planning
                    print "Target Selection Time Overall:"
                    print self.time_target_selection
                    print "Spatial Cost:"
                    print np.sum(self.robot_perception.cell_matrix != 0)
                    print np.sum(self.robot_perception.cell_matrix)
                    print "Exploration Overall Time:"
                    print self.time_exploration
                    print "Exploration Coverage Update Tine Overall:"
                    print self.time_exploration_coverage
                    print "Exploration Path Planning Tine Overall:"
                    print self.time_exploration_path_planning
                    print "Exploration Target Selection Time Overall:"
                    print self.time_exploration_target_selection
                    print "OGM waiting time:"
                    print self.num_of_targets * 10
                    return
            else:
                print "Navigation: New target: " + str(target)
        else:
            if time.time() - self.start_time < self.time_limit:
                print "The target function is Next Best View"
                target = self.target_selection.selectNextBestView(\
                    local_ogm,\
                    local_coverage,\
                    self.robot_perception.robot_pose,\
                    self.select_another_target,\
                    self.robot_perception.origin,\
                    self.robot_perception.resolution)
            elif time.time() - self.start_time < 2 * self.time_limit:
                print "The target function is Nearest Unexplored Brushfire"
                target = self.target_selection.selectNearestUnexploredBrush(\
                    local_ogm,\
                    local_coverage,\
                    self.robot_perception.robot_pose,\
                    self.select_another_target,\
                    self.robot_perception.origin,\
                    self.robot_perception.resolution)
            elif time.time() - self.start_time < 3 * self.time_limit:
                print "The target function is Best Topology"
                target = self.target_selection.selectBestTopology(\
                    local_ogm,\
                    local_coverage,\
                    self.robot_perception.robot_pose,\
                    self.select_another_target,\
                    self.robot_perception.origin,\
                    self.robot_perception.resolution)
            elif time.time() - self.start_time < 4 * self.time_limit:
                print "The target function is Nearest Uncovered Brushfire"
                target = self.target_selection.selectNearestUncoveredBrush(\
                    local_ogm,\
                    local_coverage,\
                    self.robot_perception.robot_pose,\
                    self.select_another_target,\
                    self.robot_perception.origin,\
                    self.robot_perception.resolution)
                if target == []:
                    print "Can't find new target. The exploration is complete"
                    self.end_time = time.time()
                    print "Temporal Cost:"
                    print self.end_time - self.start_time
                    print "Spatial Cost:"
                    print np.sum(self.robot_perception.cell_matrix != 0)
                    print np.sum(self.robot_perception.cell_matrix)
                    return
            else:
                print "Time is Up!!! Exploration is complete"
                return
        
        end = time.time()
        print "Time till now(target_selection):", self.time_target_selection
        self.time_target_selection += (end - start)
        print end - start
        print self.time_target_selection
        
        
        # Once the target has been found, find the path to it
        # Get the global robot pose
        g_robot_pose = self.robot_perception.getGlobalCoordinates(\
            [self.robot_perception.robot_pose['x_px'],\
            self.robot_perception.robot_pose['y_px']])
        
        
        start = time.time()
        
        # Need to reverse the path??
        self.path = self.path_planning.createPath(\
            local_ros_ogm,\
            g_robot_pose,\
            target)
        print "Navigation: Path for target found with " + str(len(self.path)) +\
            " points"
        # Reverse the path to start from the robot
        self.path = self.path[::-1]
        #self.path = []
        
        
        end = time.time()
        print "Time till now(path planning):", self.time_path_planning
        self.time_path_planning += (end - start)
        print end - start
        print self.time_path_planning
        
        
        # Check if path was not produced. If not, another target should be created
        if len(self.path) == 0:
            self.target_exists = False
            self.select_another_target = 1
            return
        else:
            self.select_another_target = 0
        
        # Break the path to subgoals every 2 pixels (1m = 20px)
        step = 2
        n_subgoals = (int) (len(self.path)/step)
        self.subtargets = []
        for i in range(0, n_subgoals):
          self.subtargets.append(self.path[i * step])
        self.subtargets.append(self.path[-1])
        self.next_subtarget = 0
        print "The path produced " + str(len(self.subtargets)) + " subgoals"

        # Publish the path for visualization purposes
        ros_path = Path()
        ros_path.header.frame_id = "map"
        for p in self.path:
          ps = PoseStamped()
          ps.header.frame_id = "map"
          ps.pose.position.x = p[0] * self.robot_perception.resolution + \
                  self.robot_perception.origin['x']
          ps.pose.position.y = p[1] * self.robot_perception.resolution + \
                  self.robot_perception.origin['y']
          ros_path.poses.append(ps)

        self.path_publisher.publish(ros_path)

        # Publish the subtargets for visualization purposes
        ros_subgoals = MarkerArray()
        count = 0
        for s in self.subtargets:
            st = Marker()
            st.header.frame_id = "map"
            st.ns = 'as_namespace'
            st.id = count
            st.header.stamp = rospy.Time(0)
            st.type = 2 # sphere
            st.action = 0 # add
            st.pose.position.x = s[0] * self.robot_perception.resolution + \
                    self.robot_perception.origin['x']
            st.pose.position.y = s[1] * self.robot_perception.resolution + \
                    self.robot_perception.origin['y']

            st.color.r = 0
            st.color.g = 0.8
            st.color.b = 0
            st.color.a = 0.8
            st.scale.x = 0.2
            st.scale.y = 0.2
            st.scale.z = 0.2
            ros_subgoals.markers.append(st)
            count += 1

        self.subtargets_publisher.publish(ros_subgoals)

        self.inner_target_exists = True

    def velocitiesToNextSubtarget(self):
        
        [linear, angular] = [0, 0]

        # YOUR CODE HERE ------------------------------------------------------
        # The velocities of the robot regarding the next subtarget should be 
        # computed. The known parameters are the robot pose [x,y,th] from 
        # robot_perception and the next_subtarget [x,y]. From these, you can 
        # compute the robot velocities for the vehicle to approach the target.
        # Hint: Trigonometry is required
    
        max_angular = 2
        max_linear  = 0.25

        [rx, ry] = [\
            self.robot_perception.robot_pose['x_px'] - \
                    self.robot_perception.origin['x'] / self.robot_perception.resolution,\
            self.robot_perception.robot_pose['y_px'] - \
                    self.robot_perception.origin['y'] / self.robot_perception.resolution\
                    ]
        theta = self.robot_perception.robot_pose['th']
        
        if self.subtargets and self.next_subtarget <= len(self.subtargets) - 1:
            #print self.subtargets
            #print self.next_subtarget
            st_x = self.subtargets[self.next_subtarget][0]
            st_y = self.subtargets[self.next_subtarget][1]
            phi = math.atan2(st_y - ry, st_x - rx)
            ang_diff = (phi - theta)
            if ang_diff >= 0 and ang_diff < math.pi:
                angular = ang_diff / math.pi
            if ang_diff > 0 and ang_diff >= math.pi:
                angular = (ang_diff - 2 * math.pi) / math.pi
            if ang_diff <= 0 and ang_diff > -math.pi:
                angular = ang_diff / math.pi
            if ang_diff < 0 and ang_diff < -math.pi:
                angular = (ang_diff + 2 * math.pi) / math.pi
            linear = (1 - abs(angular))**16 * max_linear
            angular = angular * max_angular
            #linear = (1 - abs(angular))**16 * max_linear
			#print "[linear,angular] = [%f,%f]" % (linear,angular)
        # ---------------------------------------------------------------------

        return [linear, angular]

    
