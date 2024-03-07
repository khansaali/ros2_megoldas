from __future__ import print_function
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, PointStamped, Transform
from visualization_msgs.msg import Marker
import std_msgs
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
        marker_points.header.frame_id = "base_link" # "laser"
        self.marker_points.type = Marker.SPHERE_LIST
        self.marker_points.action = marker_points.MODIFY
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
