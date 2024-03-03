#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
import math
import numpy.ma as mask

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
		
	# TODO: Initialize your publishers and subscribers here
        # subscribe to scan topic
        self.subscription = self.create_subscription(LaserScan, 'scan', self.listener_callback, 10)
        # publish driving commands to drive topic
        self.publisher = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.plot = self.create_publisher(Marker, 'point', 10)

        self.speed = 1.0
        self.prev_angle = 0.0

        self.prev_error = 0.0
        self.integral = 0.0
        self.Kp = 50
        self.Ki = 0
        self.Kd = 0

    # TODO: Write your callback functions here 
    def listener_callback(self, msg):
        # msg will be a LaserScan and contain the following:
        #       angle_min/max, angle_increment, time_increment, scan_time, range_min/max, ranges, intensities
        # "In order to make the car drive autonomously you will need to publish messages of type AckermannDriveStamped to the /drive topic."
        # AckermannDriveStamped
        ans = AckermannDriveStamped()
        # set Header: uint32 seq, time stamp, string frame_id
        ans.header.stamp = rclpy.time.Time().to_msg()
        ans.header.frame_id = msg.header.frame_id

        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value


        # set AckermannDrive: float32 steering_angle, float32 steering_angle_velocity, float32 speed, float32 acceleration, float32 jerk
        #  slice the half we want
        if self.SIDE==-1: #right
            angleslice_min, angleslice_max = self.SIDE * math.pi*0.55, math.pi*0.1
        else: # left
            angleslice_min, angleslice_max = -1*math.pi*0.1, math.pi*0.55
        # calculate the indices in ranges[] that fall in [angleslice_min, angleslice_max]
        ind_min = math.floor((angleslice_min-msg.angle_min)/msg.angle_increment) + 1
        ind_max = math.floor((angleslice_max-msg.angle_min)/msg.angle_increment)
        # slice out the part between these indices, add a dimension for their exact angles
        slice = np.array(msg.ranges)[ind_min:ind_max+1]
        
        angles = np.array([msg.angle_min+i*msg.angle_increment for i in range(ind_min, ind_max+1)])

        
        # mask the ones that have values > neighbor
        avg = np.average(slice)
        larger = [False if slice[i] > min(avg, 15*self.DESIRED_DISTANCE) else True for i in range(len(slice))]
        slice, angles = slice[larger], angles[larger]
        

        # convert slice array to x,y points, with car as origin
        vsin, vcos = np.vectorize(math.sin), np.vectorize(math.cos)
        sin, cos = np.array([vsin(angles)]), np.array([vcos(angles)])
        x, y = np.array(np.multiply(cos, slice)), np.array(np.multiply(sin, slice))
       

        # find line of best fit
        lobf = np.polyfit(x[0], y[0], 1)

        # plot the wall with visualization??
        ones = np.ones_like(x)
        input = np.column_stack((x[0], ones[0]))
        output = np.matmul(input, [[lobf[0]], [lobf[1]]])
        output = np.transpose(output)
        VisualizationTools.plot_line(x[0], output[0], self.plot)

        # PID stuff
        # find intersection of lobf & perpendicular line to car -> distance to car
        intersection = np.linalg.solve([[lobf[0], -1], [1/lobf[0], 1]], [[-1*lobf[1]], [0]])
        actual_dist = math.sqrt(intersection[0][0]**2 + intersection[1][0]**2)
        self.get_logger().info('desired distance: "%s"' % self.DESIRED_DISTANCE)
        self.get_logger().info('actual distance: "%s"' % actual_dist)

        # find angle of current wall + difference cmp to car trajectory
        wall_angle = math.atan(lobf[1])
        diff = wall_angle - self.prev_angle
        self.prev_angle = ans.drive.steering_angle

        
        # TODO: based on error, set a new steering angle
        #ans.drive.steering_angle = self.prev_angle
        #self.prev_angle = ans.drive.steering_angle
        error = self.DESIRED_DISTANCE - actual_dist
        self.integral = self.integral + error
        derivative = (error - self.prev_error) / 1
        ans.drive.speed = self.VELOCITY

        ans.drive.steering_angle = self.Kp*error + self.Ki*self.integral + self.Kd*derivative
        self.prev_error = error
        
        self.get_logger().info('error: "%s"' % error)
        self.get_logger().info('side: "%s"' % self.SIDE)
        

        # publish
        self.publisher.publish(ans)
        


def main():
    
    rclpy.init()
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
