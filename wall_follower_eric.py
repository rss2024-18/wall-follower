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
        self.prev_time = None
        self.dt = 0
        self.prev_error = None
        self.de = 0
	
        self.subscription = self.create_subscription(LaserScan, self.SCAN_TOPIC, self.lidar_callback, 10)
        self.publisher = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 10)

    def lidar_callback(self, msg):
        """
        Process Lidar data
        """
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value

        curr_time = time.time()

        if self.prev_time is not None:
            self.dt = curr_time - self.prev_time

        self.prev_time = curr_time

        error = self.error_calculation(msg)
        error_derivative = (error - self.prev_error) / self.dt if self.dt > 0 else 0.0
        self.prev_error = error

        steer = self.PD_controller(error, error_derivative)

        detect = self.safety_control(msg)

        # self.get_logger().info('LIDAR ranges: {}\n{}\n\n'.format(detect))

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.speed = detect
        drive_msg.drive.steering_angle = steer

        self.publisher.publish(drive_msg)
        self.dt = time.time() - curr_time

    def slice_scan(self, ranges):
        """
        slice scan into managable pieces a
        """
        back_front_divider = int(len(ranges) * 16/33)
        front_middle_divider = int(len(ranges) * 16/33)
        total_length = int(len(ranges))
        bls = ranges[total_length - back_front_divider:total_length]
        ms = ranges[front_middle_divider:total_length - front_middle_divider]
        brs = ranges[0:back_front_divider]
        return {'brs': brs, "ms": ms, "bls" : bls}
    
    def return_data(self, data, flag = 0):
        """
        Returns appropriate data slice for side and speed
        """
        data = self.slice_scan(data)
        if flag:
            return np.array(data["ms"])
        if self.SIDE == -1:
            return np.array(data["brs"])
        return np.flip(np.array(data["bls"]))

    def scale_data(self, msg):
        distance_data = self.return_data(msg.ranges)
        angle_arr = self.return_data(np.arange(msg.angle_min, msg.angle_max, msg.angle_increment).tolist())
        angle_data = np.array(np.cos(angle_arr - (np.pi/2 * self.SIDE)))
        modified_data = np.multiply(angle_data, distance_data)
        return modified_data

    def error_calculation(self, msg):
        """
        Returns error between desired distance and actual distace. Positive values mean that the 
        car is to the right and negative values mean the car is to the left
        """
        real_data = self.scale_data(msg)
        goal = self.DESIRED_DISTANCE
        ### CHECK TO SEE IF DESIRED DISTANCE IS ALWAYS POSITIVE
        weights = np.flip(np.linspace(.7, 1.5, len(real_data)))
        error = self.SIDE*-1*(goal - real_data)
        #If error is positive, car is to the right
        return np.sum(np.multiply(weights, error))


    def safety_control(self, msg):
        """
        Compute distance to front wall and stop the car if the wall is too close
        """
        distance_needed_to_stop = self.VELOCITY * 1/3
        front_data = np.array(self.return_data(msg.ranges, 1))
        middle_angles = np.array(np.cos(self.return_data(np.arange(msg.angle_min, msg.angle_max, msg.angle_increment).tolist(), 1)))
        true_dist = np.multiply(middle_angles, front_data)
        if np.median(true_dist) < distance_needed_to_stop:
            return 0.0
        return self.VELOCITY
    
    def PD_controller(self, error, error_derivative):
        kd = 0.008
        #0.01 seems to be a good value for kp
        kp = 0.01       
        steering_angle = kp * error + max(min(kd * error_derivative, 0.34), -0.34)
        return max(min(steering_angle, 0.34), -0.34)


def main(args=None):
    
    rclpy.init(args=args)
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
