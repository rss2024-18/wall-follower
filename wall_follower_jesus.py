#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
import time

from wall_follower.visualization_tools import VisualizationTools


class WallFollower(Node):

    def __init__(self):
        super().__init__("wall_follower")
        # Declare parameters to make them available for use
        self.declare_parameter("scan_topic", "default")
        self.declare_parameter("drive_topic", "default")
        self.declare_parameter("side", "default")
        self.declare_parameter("velocity", "default")
        self.declare_parameter("desired_distance", "default")

        # Fetch constants from the ROS parameter server
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value

        # self.get_logger().info(str(self.SIDE))
        # self.get_logger().info(self.VELOCITY)
		
        # a publisher for our line marker
        self.line_pub = self.create_publisher(Marker, self.WALL_TOPIC, 1)

        # a subscriber to get the laserscan data
        self.create_subscription(LaserScan, self.SCAN_TOPIC, self.laser_callback, 10)

        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)  # Change topic name to '/drive'
        self.prev_error = 0

        self.get_logger().info('Line publisher initialized')
        self.get_logger().info(self.SCAN_TOPIC)

    def laser_callback(self, laser_scan):
        
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value
        # x and y should be points on your detected wall
        # here we are just plotting a parabola as a demo
        # x = np.linspace(-2., 2., num=20)
        # y = np.square(x)
        # VisualizationTools.plot_line(x, y, self.line_pub, frame="/laser")

        rangesLength = len(laser_scan.ranges)

        # self.get_logger().info('Ranges before %s' % laser_scan.ranges)
        # self.get_logger().info('Ranges length is: ' + str(len(laser_scan.ranges)))  

        angles = np.linspace(laser_scan.angle_min, laser_scan.angle_max, rangesLength)
        modRanges = []
        if self.SIDE == 1:
            modRanges = laser_scan.ranges[(rangesLength//2):]
            angles = angles[(rangesLength//2):]
            # modRanges = laser_scan.ranges[(5*(rangesLength//8)):(7*(rangesLength//8))]
            # angles = angles[(5*(rangesLength//8)):(7*(rangesLength//8))]
        elif self.SIDE == -1:
            modRanges = laser_scan.ranges[:(rangesLength//2)]
            angles = angles[:(rangesLength//2)]
            # modRanges = laser_scan.ranges[(1*(rangesLength//8)):(3*(rangesLength//8))]
            # angles = angles[(1*(rangesLength//8)):(3*(rangesLength//8))]
        else:
            raise Exception("Expected self.SIDE to be either -1 or 1. Got: " + str(self.SIDE))
        
        # filter high values

        highIdxs = [index for index, value in enumerate(modRanges) if value > 20]
        modRanges = [value for index, value in enumerate(modRanges) if not(index in highIdxs)]
        angles = [value for index, value in enumerate(angles) if not(index in highIdxs)]
    
        xVals = np.array(modRanges) * np.cos(np.array(angles))
        yVals = np.array(modRanges) * np.sin(np.array(angles))

        slope, intercept = np.polyfit(xVals, yVals, 1)
        
        xPoints = np.linspace(-2., 2., num=20)
        yPoints = [slope*xi + intercept for xi in xPoints]

        VisualizationTools.plot_line(xPoints, yPoints, self.line_pub, frame="/laser")

        # self.get_logger().info('Plotted the dude with a slope of :' + str(slope) + ' and an intercept of ' + str(intercept)) 
        # self.get_logger().info('Max dist value of: ' + str(max(laser_scan.ranges))) 

        self.Kp = 0.2  # Proportional gain
        # self.Ki = 0  # Integral gain
        self.Kd = 2  # Derivative gain

        current_time = time.time()
        time_difference = current_time - self.last_callback_time
        self.last_callback_time = current_time

        error = self.DESIRED_DISTANCE - abs(intercept)  # Calculate the error

        # Proportional term
        proportional = self.Kp * error

        # # Integral term
        # self.integral += error
        # integral = self.Ki * self.integral

        # Derivative term
        derivative = self.Kd * (error - self.prev_error)/time_difference
        self.prev_error = error

        # Calculate the control signal
        control_signal = proportional + derivative

        ackermann_msg = AckermannDriveStamped()
        ackermann_msg.drive.steering_angle = control_signal  # Example: Control signal is used as steering angle
        ackermann_msg.drive.speed = self.VELOCITY*max(0.5, 1-0.1*control_signal**2)
        self.drive_pub.publish(ackermann_msg)
        self.get_logger().info('Published Ackerman file with Kp' + str(proportional))
        self.get_logger().info('Published Ackerman file with Kd' + str(derivative))
        self.get_logger().info('Published Ackerman file with angle' + str(control_signal))
                               
                            
        # self.get_logger().info('Ranges after %s' % modRanges)   
        # self.get_logger().info('Mod ranges length is: ' + str(len(modRanges)))
        # self.get_logger().info('min angle: ' + str(laser_scan.angle_min))
        # self.get_logger().info('min angle: ' + str(laser_scan.angle_max))
        # self.get_logger().info('min angle: ' + str(laser_scan.angle_))




def main():
    
    rclpy.init()
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
