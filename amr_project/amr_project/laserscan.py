# 
'''
ROS2 Humble node to control a turtlebot3 using the velocity topic

'''

import rclpy
from rclpy.node import Node
import numpy as np   

# Import \cmd_vel topic message type
from geometry_msgs.msg import Twist

# Import Laser scanner message type
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

# FSM State definition
TB3_MOVING = 0
TB3_ROTATING = 1
TB3_RECOVERY = 2
MIN_SAFE_DISTANCE = 1.0 # in meters

class velocityController():

   def __init__(self) -> None:

      # Initialize Node
         
      self.node = rclpy.create_node('VelocityController')
       
      # Create Publisher and Subscriber
      self.VelocityController = self.node.create_publisher(Twist, '/cmd_vel', 10)
      self.LaserScanner = self.node.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
      self.BoolSubscriber = self.node.create_subscription(Bool, 'localization_complete', self.bool_callback, 10)


      # Create Rate to specify the frequency of the main control loop
      # Alternative to control using time
      self.rate = self.node.create_rate(30) #Hz

      # initialize the states
      self.tb3_state = TB3_MOVING
      self.laser_data = None
      self.localization_complete = False
      pass


   def move(self):
      # Function to move forward the turtlebot
      vel_msg = Twist()
      vel_msg.linear.x = 0.15
      vel_msg.angular.z = 0.05

      self.VelocityController.publish(vel_msg)
      # self.node.get_logger().info(f'Publishing new velocity X: {vel_msg.linear.x} Z: {vel_msg.angular.z}')
      return

   def rotate(self):
      # Function to rotate the turtlebot
      vel_msg = Twist()
      vel_msg.linear.x = 0.0
      vel_msg.angular.z = 0.1

      self.VelocityController.publish(vel_msg)
      # self.node.get_logger().info(f'Publishing new velocity X: {vel_msg.linear.x} Z: {vel_msg.angular.z}')

      return


   def stop(self):
      # Function to stop the turtlebot
      vel_msg = Twist()
      vel_msg.linear.x = 0.0
      vel_msg.angular.z = 0.0

      self.VelocityController.publish(vel_msg)
      # self.node.get_logger().info(f'Publishing new velocity X: {vel_msg.linear.x} Z: {vel_msg.angular.z}')
      return


   def laser_callback(self, msg):
      # Function to store the laser scanner data
      self.laser_data = msg
      print("data received")
      return self.laser_data

   def check_collision(self):
      # Function that uses laser scanner to check for collision
      if self.laser_data is None:
         print("No laser data received")
         return False
     
      front_data = self.laser_data.ranges[-25:] + self.laser_data.ranges[:25]

      for i in range(len(front_data)):
         if front_data[i] < MIN_SAFE_DISTANCE:
            # print("Collision detected at", min(front_data), "meters")
            return True
         
      # self.tb3_state = TB3_MOVING
      return False
   
   def bool_callback(self, msg):
      # Function to check if localization has been completed to shut down the node
      self.localization_complete = msg.data
      if self.localization_complete:
         print("Localization complete, I am shutting down the laserscan node.")
         self.shutdown_node()

   def shutdown_node(self):
      # Function to shut down the node
      self.stop()
      self.node.destroy_node()
      rclpy.shutdown()

# Call the shutdown_node function to shut down the node

   def control_loop(self):
      # Main control loop to control the movement of the turtlebot based on the laser scanner data
      # enter state machine
      # self.node.get_logger().info(f'Entering state machine: {self.tb3_state}')
      while rclpy.ok():

         rclpy.spin_once(self.node)
         # TB3 moving forward
         if self.tb3_state == TB3_MOVING:
            self.move()
            # Control with laser scanner
            if self.check_collision():
               self.tb3_state = TB3_ROTATING
         # TB3 is rotating
         elif self.tb3_state == TB3_ROTATING:
            self.rotate()
            if not self.check_collision():
               self.tb3_state = TB3_MOVING
         # TB3 is stopped and is not moving
         elif self.tb3_state == TB3_RECOVERY:
            # Example of additional usfull state
            # !!!!
            continue
     

def main(args=None):
   # class initialization and main function to run the node

   rclpy.init()
   rdv = velocityController()
   rdv.control_loop()
   rclpy.spin(rdv.node)    
   rdv.node.destroy_node()
   rclpy.shutdown()

if __name__ == '__main__':
   main()
