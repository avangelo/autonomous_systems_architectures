#!/usr/bin/env python

import rospy
import math
import random

from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from sonar_data_aggregator import SonarDataAggregator
from laser_data_aggregator import LaserDataAggregator
from navigation import Navigation

# Class for assigning the robot speeds 
class RobotController:

    # Constructor
    def __init__(self):
        
      # Debugging purposes
      self.print_velocities = rospy.get_param('print_velocities')

      # Where and when should you use this?
      self.stop_robot = False

      # Create the needed objects
      self.sonar_aggregation = SonarDataAggregator()
      self.laser_aggregation = LaserDataAggregator()
      self.navigation  = Navigation()

      self.linear_velocity  = 0
      self.angular_velocity = 0

      # Check if the robot moves with target or just wanders
      self.move_with_target = rospy.get_param("calculate_target")

      # The timer produces events for sending the speeds every 110 ms
      rospy.Timer(rospy.Duration(0.11), self.publishSpeeds)
      self.velocity_publisher = rospy.Publisher(\
              rospy.get_param('speeds_pub_topic'), Twist,\
              queue_size = 10)

      # Read the velocities architecture
      self.velocity_arch = rospy.get_param("velocities_architecture")
      print "The selected velocities architecture is " + self.velocity_arch

    # This function publishes the speeds and moves the robot
    def publishSpeeds(self, event):
        
      # Choose architecture
      if self.velocity_arch == "subsumption":
        self.produceSpeedsSubsumption()
      else:
        self.produceSpeedsMotorSchema()

      # Create the commands message
      twist = Twist()
      twist.linear.x = self.linear_velocity
      twist.linear.y = 0
      twist.linear.z = 0
      twist.angular.x = 0 
      twist.angular.y = 0
      twist.angular.z = self.angular_velocity

      # Send the command
      self.velocity_publisher.publish(twist)

      # Print the speeds for debuggind purposes
      if self.print_velocities == True:
        print "[L,R] = [" + str(twist.linear.x) + " , " + \
            str(twist.angular.z) + "]"

    # Produce speeds from sonars
    def produceSpeedsSonars(self):
      # Get the sonars' measurements
      front   = self.sonar_aggregation.sonar_front_range
      left    = self.sonar_aggregation.sonar_left_range
      right   = self.sonar_aggregation.sonar_right_range
      r_left  = self.sonar_aggregation.sonar_rear_left_range
      r_right = self.sonar_aggregation.sonar_rear_right_range
        
      linear  = 0
      angular = 0

      # YOUR CODE HERE ------------------------------------------------------
      # Adjust the linear and angular velocities using the five sonars values

      ##### 1 #####
      Ff = front
      Fl = 3 - left
      Fr = 3 - right
      Frr = 1 / r_right
      Frl = 1 / r_left
      Frrx = Frr * math.cos(20 * math.pi / 360)
      Frry = Frr * math.sin(20 * math.pi / 360)
      Frlx = Frl * math.cos(20 * math.pi / 360)
      Frly = Frl * math.sin(20 * math.pi / 360)     
      SFx = Ff + 0.25 * Frrx + 0.25 * Frlx
      SFy = Fr - Fl + Frry - Frly
      linear = SFx
      linear = linear / 25
      angular = SFy
      angular = angular / 15

      if front < 0.25:
        linear = 0.005
      if angular > 0.2:
	angular = 0.2
      if angular < -0.2:
	angular = -0.2
      if left < 0.2:
	angular = -0.2
      if right < 0.2:
	angular = 0.2
      if angular > 0.2:
	angular = 0.2
      if angular < -0.2:
	angular = -0.2
      if linear > 0.2:
	linear = 0.2

      # ---------------------------------------------------------------------

      return [linear, angular]

    # Produces speeds from the laser
    def produceSpeedsLaser(self):

      # Get the laser scan
      scan   = self.laser_aggregation.laser_scan
        
      linear  = 0
      angular = 0

      # YOUR CODE HERE ------------------------------------------------------
      # Adjust the linear and angular velocities using the laser scan
      
      ##### 2 #####

      SFx = 0
      SFy = 0
      SFyl = 0
      SFyr = 0
      for x in range(0,283):
          SFyl = SFyl + 1 / scan[x]
      SFyl = SFyl / 284
      for x in range(384,667):
          SFyr = SFyr + 1 / scan[x]
      SFyr = SFyr / 284
      SFx = min(scan[233:433])
      SFy = SFyl - SFyr
      linear = SFx / 25
      angular = SFy
      if angular > 0.2:
	angular = 0.2
      if angular < -0.2:
	angular = -0.2
      # ---------------------------------------------------------------------
      return [linear, angular]

    # Combines the speeds into one output using a subsumption approach
    def produceSpeedsSubsumption(self):
      
      # Produce target if not existent
      if self.move_with_target == True and self.navigation.target_exists == False:
	# Create the commands message
	twist = Twist()
	twist.linear.x = 0
	twist.linear.y = 0
	twist.linear.z = 0
	twist.angular.x = 0
	twist.angular.y = 0
	twist.angular.z = 0

	# Send the command
	self.velocity_publisher.publish(twist)
        self.navigation.selectTarget()

      # Get the submodules' speeds
      [l_sonar, a_sonar] = self.produceSpeedsSonars()
      [l_laser, a_laser] = self.produceSpeedsLaser()
      [l_goal, a_goal] = self.navigation.velocitiesToNextSubtarget()

      self.linear_velocity  = 0
      self.angular_velocity = 0

      front   = self.sonar_aggregation.sonar_front_range
      left    = self.sonar_aggregation.sonar_left_range
      right   = self.sonar_aggregation.sonar_right_range

      dist_front_laser = 0
      dist_left_laser = 0
      dist_right_laser = 0

      # Combine the speeds following the subsumption architecture
      # YOUR CODE HERE ------------------------------------------------------
      
     
      
      ##### 3 #####
      #if front < 0.2:
	#self.linear_velocity = l_laser
      #else:
	#self.linear_velocity = l_sonar
      #if right < 0.2 or left < 0.2:
	#self.angular_velocity = a_laser
      #else:
	#self.angular_velocity = a_sonar
      

      ##### 8 #####
      #scan = self.laser_aggregation.laser_scan
      #left_laser = scan[0:284]
      #front_laser = scan[284:384]
      #right_laser = scan[384:667]
      #min_left_laser = min(left_laser)
      #min_front_laser = min(front_laser)
      #min_right_laser = min(right_laser)
       
      #if min_front_laser <= 0.2 or min_right_laser <= 0.2 or min_left_laser <= 0.2:
	#self.linear_velocity = l_laser
	#self.angular_velocity = a_laser
      #elif (right < 0.3 and right > 0.2) or (left < 0.3 and left > 0.2) or (front < 0.3 and front > 0.2):
	#self.linear_velocity = l_sonar
	#self.angular_velocity = a_sonar
      #else: #right >= 0.4 and left >= 0.4 and right >= 0.4:
	#self.linear_velocity = l_goal
	#self.angular_velocity = a_goal


      # ---------------------------------------------------------------------

    # Combines the speeds into one output using a motor schema approach
    def produceSpeedsMotorSchema(self):
 
      # Produce target if not existent
      if self.move_with_target == True and self.navigation.target_exists == False:
	# Create the commands message
	twist = Twist()
	twist.linear.x = 0
	twist.linear.y = 0
	twist.linear.z = 0
	twist.angular.x = 0
	twist.angular.y = 0
	twist.angular.z = 0

	# Send the command
	self.velocity_publisher.publish(twist)
        self.navigation.selectTarget()

      # Get the submodule's speeds
      [l_sonar, a_sonar] = self.produceSpeedsSonars()
      [l_laser, a_laser] = self.produceSpeedsLaser()
      [l_goal, a_goal] = self.navigation.velocitiesToNextSubtarget()

      self.linear_velocity  = 0
      self.angular_velocity = 0

      front   = self.sonar_aggregation.sonar_front_range
      left    = self.sonar_aggregation.sonar_left_range
      right   = self.sonar_aggregation.sonar_right_range

      dist_front_laser = 0
      dist_left_laser = 0
      dist_right_laser = 0


        
      # Get the speeds using the motor schema approach
      # YOUR CODE HERE ------------------------------------------------------
      
      ##### 1 #####
      #self.linear_velocity = l_sonar
      #self.angular_velocity = a_sonar

      ##### 2 #####
      #self.linear_velocity = l_laser
      #self.angular_velocity = a_laser 
      
      ##### 4 #####
      self.linear_velocity = 0.5 * l_sonar + 0.5 * l_laser
      self.angular_velocity = 0.5 * a_sonar + 0.5 * a_laser

      ##### 9 #####
      #~ scan = self.laser_aggregation.laser_scan
      #~ left_laser = scan[0:284]
      #~ front_laser = scan[284:384]
      #~ right_laser = scan[384:667]
      #~ min_left_laser = min(left_laser)
      #~ min_front_laser = min(front_laser)
      #~ min_right_laser = min(right_laser)
#~ 
#~ 
      #~ if min_front_laser <= 0.2 or min_right_laser <= 0.2 or min_left_laser <= 0.2:
	#~ self.linear_velocity = 0.125 * l_goal + 0.125 * l_sonar + 0.75 * l_laser
	#~ self.angular_velocity = 0.125 * a_goal + 0.125 * a_sonar + 0.75 * a_laser
      #~ elif (right < 0.3 and right > 0.2) or (left < 0.3 and left > 0.2) or (front < 0.3 and front > 0.2):
	#~ self.linear_velocity = 0.125 * l_goal + 0.75 * l_sonar + 0.125 * l_laser
	#~ self.angular_velocity = 0.125 * a_goal + 0.75 * a_sonar + 0.125 * a_laser
      #~ else: #elif front >= 0.3 and left >= 0.3 and right >= 0.3:
	#~ self.linear_velocity = 0.75 * l_goal + 0.125 * l_sonar + 0.125 * l_laser
	#~ self.angular_velocity = 0.75 * a_goal + 0.125 * a_sonar + 0.125 * a_laser
      # ---------------------------------------------------------------------

    # Assistive functions - Do you have to call them somewhere?
    def stopRobot(self):
      self.stop_robot = True

    def resumeRobot(self):
      self.stop_robot = False
