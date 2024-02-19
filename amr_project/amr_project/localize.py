
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class localize(Node):
    def __init__(self):
        super().__init__('localization')
        # Create the subscriber to the 'costmap' topic
       
        # Create the publisher to 'localization_complete' topic to communicate with other nodes
        self.publisher = self.create_publisher(Bool, 'localization_complete', 10)
        
        amcl_pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        # Create the client to the 'reinitialize_global_localization' service
        self.reinitialize_global_localization_client = self.create_client(Empty, '/reinitialize_global_localization')
        while not self.reinitialize_global_localization_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        # Create the subscriber to 'amcl_pose' topic using the QoSProfile specified
        self.model_pose_sub = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._amclPoseCallback, amcl_pose_qos)
        # Create a subscriber to the 'scan' topic
     
        # Create a subscriber to the 'odom' topic
        self.odometry_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odometry_callback,
            10)

        # Create the publisher to 'initialpose' for the initialization
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.initial_pose = None
        self.initial_pose_received = False
        self.average_covariance = float('inf')
        self.x_cov = float('inf')
        self.y_cov = float('inf')
        self.z_cov = float('inf')



        self.localization_done = False
        self.robot_pose = None

    
    def odometry_callback(self, msg):
        # Update the robot's initial position using odometry data
        self.initial_pose = msg.pose.pose

        
    def _amclPoseCallback(self, msg):
        # Update the robot's initial position using amcl_pose data and covariance
            self.initial_pose = msg.pose.pose
            self.x_cov = msg.pose.covariance[0]
            self.y_cov = msg.pose.covariance[7]
            self.z_cov = msg.pose.covariance[35]
            self.average_covariance = (self.x_cov + self.y_cov) / 2
            self.robot_pose = msg.pose.pose.position.x, msg.pose.pose.position.y
            # print('covariance:',msg.pose.covariance)


def main(args=None):
    rclpy.init(args=args)

    localization = localize()
    # Wait until the initial pose is received
    while localization.initial_pose is None:
        rclpy.spin_once(localization)
    # Set the initial pose of the robot, this is the first guess of the robot's position
    inital_pose = PoseWithCovarianceStamped()  
    inital_pose.header = Header()
    inital_pose.header.frame_id = 'map'
    inital_pose.pose.pose.position.x = localization.initial_pose.position.x
    inital_pose.pose.pose.position.y = localization.initial_pose.position.y
    inital_pose.pose.pose.position.z = 0.0

    localization.initial_pose_pub.publish(inital_pose)

    print("Robot position: ", localization.initial_pose)
 
    request = Empty.Request()
    future = localization.reinitialize_global_localization_client.call_async(request)


    covariance_threshold_x = 1.0
    covariance_threshold_y = 1.5
    covariance_threshold_z = 0.5
    covariance_threshold = (covariance_threshold_x + covariance_threshold_y) / 2
    k = 0
    guessed_positions = []

    # Keep setting the initial pose until the average covariance is below the threshold
    while localization.x_cov > covariance_threshold_x or localization.y_cov > covariance_threshold_y or localization.z_cov > covariance_threshold_z:
    
        print('x_cov: ', localization.x_cov)
        print('y_cov: ', localization.y_cov)
        print('z_cov: ', localization.z_cov)
    
        rclpy.spin_once(localization)
        k += 1

        print('k: ', k)
        
        if k % 20000 == 0 and localization.average_covariance > 1.25 * covariance_threshold:  
               
            print('Robot is not in the right position. Reinitializing localization...')
            rclpy.spin_once(localization)
            request = Empty.Request()
            future = localization.reinitialize_global_localization_client.call_async(request)
            inital_pose = PoseWithCovarianceStamped()  
            inital_pose.header = Header()
            inital_pose.header.frame_id = 'map'
            inital_pose.pose.pose.position.x = localization.initial_pose.position.x
            inital_pose.pose.pose.position.y = localization.initial_pose.position.y
            inital_pose.pose.pose.position.z = 0.0   
       
            

    
    localization.localization_done = True
    print('Localization complete!')
    if localization.localization_done:
        print('Localization complete!')
        localization.publisher.publish(Bool(data=True))
 
    rclpy.spin(localization)
    

    localization.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 

