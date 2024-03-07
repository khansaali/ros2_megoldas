lfrom __future__ import print_function
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, PointStamped, Transform
from visualization_msgs.msg import Marker

import std_msgs.msg
import math
import numpy as np
import tf2_ros
import tf2_geometry_msgs



KOZEPISKOLA_NEVE = "Ismeretlen kozepiskola"
KOZEPISKOLA_AZON = "A00"
ANGLE_RANGE = 360 # LSN10 LIDAR has 360 degrees scan
DESIRED_DISTANCE_RIGHT = 1.0 #0.9 # meters
DESIRED_DISTANCE_LEFT = 0.8 # 0.55
VELOCITY = 1.00 # meters per second
CAR_LENGTH = 0.445 # 0.445 meters
WHEELBASE = 0.3187; # documention based | measured: ~32 cm


class DistFinderNode(Node):
    def __init__(self):
        super().__init__('dist_finder')
        self.create_subscription(LaserScan, 'scan', self.callback_laser, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.pubst1 = self.create_publisher(std_msgs.msg.String, 'pid_data', 10)
        self.pubst2 = self.create_publisher(std_msgs.msg.String, 'kozepiskola', 10)
        self.marker_pub = self.create_publisher(Marker, '/debug_marker', 1)
        self.trans = Transform()
        self.prev_steering_err = 0.0
        self.prev_velocity = 0.0
        self.marker_points = Marker()
        self.marker_points.header.frame_id = "base_link" # "laser"
        self.marker_points.type = Marker.SPHERE_LIST
        self.marker_points.action = self.marker_points.MODIFY
        self.marker_points.color.r = 0.0
        self.marker_points.color.g = 0.0
        self.marker_points.color.a = 1.0
        self.marker_points.color.b = 1.0
        self.marker_points.scale.x = 0.1
        self.marker_points.scale.y = 0.1
        self.marker_points.scale.z = 0.1
        self.marker_points.pose.orientation.x = 0.0
        self.marker_points.pose.orientation.y = 0.0
        self.marker_points.pose.orientation.z = 0.0
        self.marker_points.pose.orientation.w = 1.0

    # pure pursuit steering angle calc
    def calcPursuitAngle(self,goal_x, goal_y):
        alpha = math.atan2(goal_y, goal_x)
        lookahead_distance = math.sqrt(pow(goal_x, 2) + pow(goal_y, 2))
        steering_angle = math.atan2(2.0 * WHEELBASE * np.sin(alpha) / (lookahead_distance), 1)
        return steering_angle
    def calcPointPos(self,range, angle):
        x1 = range * math.cos(angle)
        y1 = range * math.sin(angle)
        return x1, y1
    def getDistance(self,ranges, angles):
        global marker_points
        if(len(ranges) > 50):
            center1_min_index = np.where(math.radians(160) < angles)[0][0]
            center1_max_index = np.where(math.radians(179.9) < angles)[0][0]
            tmp1 = np.arange(center1_min_index, center1_max_index, 1)
            center2_min_index = np.where(math.radians(-179.9) < angles)[0][0]
            center2_max_index = np.where(math.radians(-160) < angles)[0][0]
            tmp2 = np.arange(center2_min_index, center2_max_index, 1)
            tmp = np.concatenate((tmp1, tmp2))
            max_x = -10.0
            for t in tmp:
                point = Point()
                point.x, point.y = self.calcPointPos(ranges[t], angles[t])
                if not math.isinf(point.x):
                    # find max (flipped upside min)
                    if point.x > max_x:
                        max_x = point.x
                          """
            # debug
            if not math.isinf(point.x):
                point.z = 1
                marker_points.points.append(point)
            """
        if math.isinf(max_x):
            max_x = -5.0
        # within 40 cm reverse - tolatas    
        if max_x > -0.4:
            max_x = 0.5
            #print("tolatas - backward motion")            
        """
        # debug
        marker_points.header.frame_id = "laser"
        """
        distance = max_x
    else: 
        distance = 0.4
    return distance


    
    def callback_laser(self, data):
        # Does a simple follow
        error_steering, velocity = self.follow_simple(data)
        
        # Create a new Twist message for velocity and steering command
        msg_cmd = Twist()
        msg_cmd.linear.x = velocity * 0.5  # Adjusting linear velocity (scaled for testing low speed)
        msg_cmd.angular.z = error_steering  # Setting angular velocity (steering angle)
        
        # Publish the Twist message to the "cmd_vel" topic
        self.pub.publish(msg_cmd)   





def main(args=None):
    rclpy.init(args=args)
    node = DistFinderNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()




