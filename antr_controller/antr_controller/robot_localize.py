import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import numpy as np
from cv_bridge import CvBridge
from tf2_geometry_msgs import TransformStamped
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import os
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid
import math

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

def detect_green_box(image):

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, (36, 25, 25), (70, 255,255))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    x = 0
    y = 0
    for contour in contours:

        if cv2.contourArea(contour) < 500:
            continue

        # Get the minimum area rectangle around the contour
        rect = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        x,y = rect[0][0],rect[0][1]
        print("Rect ",rect)

        cv2.drawContours(image, [box], 0, (255, 255, 255), 2)
        center = (int(x), int(y))
        cv2.circle(image, center, 5, (255, 0, 0), -1)

        cv2.putText(image,f"ori_Z{rect[2]}, X : {x/370} Y : {y/370} ",(10,50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2)
        print(f"Yellow box position: x={int(x)}, y={int(y)}")
    

    return (image,x,y,float(rect[2]))

class robot_localize(Node):
    def __init__(self):
        super().__init__("robo")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        # os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH")
        self.img = self.create_subscription(Image,"/camera",self.image_callback_,10)
        self.tf_broad = TransformBroadcaster(self)

        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        ))
        self.map_resolution = 0.0027  # Map resolution in meters/pixel
        self.map_origin = [0.0, 0.0, 0.0]  # Map origin [x, y, theta]
        self.map_width = 480  # Width in pixels
        self.map_height = 640  # Height in pixels

        self.broadcaster = TransformBroadcaster(self)
        self.ballState = self.create_publisher(Bool,"/ballState",10)

    def image_callback_(self,msg:Image):
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(msg, "bgr8")
        
        frame,x,y,o_z = detect_green_box(image)
        self.map_height,self.map_width,channels = image.shape  # Height in pixels
        # self.resolution = self.map_height/472
        cord = self.pixel_to_world(x,y,self.map_resolution,self.map_origin,self.map_height)

        odom_tf = TransformStamped()
        odom_tf.header.frame_id = "odom"
        odom_tf.child_frame_id = "base_footprint"
        odom_tf.header.stamp = self.get_clock().now().to_msg()


        print(cord)
        odom_tf.transform.translation.x = x/370
        odom_tf.transform.translation.y = y/370
        odom_tf.transform.rotation.x,odom_tf.transform.rotation.y,odom_tf.transform.rotation.z,odom_tf.transform.rotation.w = quaternion_from_euler(0, 0, -math.radians(o_z))
        self.tf_broad.sendTransform(odom_tf)
        occupancy_grid = self.create_occupancy_grid(frame)
        self.map_publisher.publish(occupancy_grid)

        cv2.imshow('Received Image', frame)
        # cv2.imshow("Map", occupancy_grid.data)
        cv2.waitKey(1)  


    def create_occupancy_grid(self, image):
        # Convert to HSV and mask red regions
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 70, 50])
        upper_red1 = np.array([0, 255, 255])
        lower_red2 = np.array([170, 70, 50])
        upper_red2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        # Convert the mask to a binary grid (0: free, 100: occupied, -1: unknown)
        grid = np.full((self.map_height, self.map_width), 0, dtype=np.int8)  # Unknown
        grid[red_mask > 0] = 100  # Occupied
        grid[red_mask == 0] = 0  # Free

        # Flatten the grid for the occupancy grid message
        grid_flattened = grid.flatten().tolist()

        # Create OccupancyGrid message
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header = Header()
        occupancy_grid.header.frame_id = 'map'
        occupancy_grid.header.stamp = self.get_clock().now().to_msg()

        # Map metadata
        occupancy_grid.info.resolution = self.map_resolution
        occupancy_grid.info.width = self.map_width
        occupancy_grid.info.height = self.map_height
        occupancy_grid.info.origin.position.x = self.map_origin[0]
        occupancy_grid.info.origin.position.y = self.map_origin[1]
        occupancy_grid.info.origin.position.z = 0.0
        occupancy_grid.info.origin.orientation.x = 0.0
        occupancy_grid.info.origin.orientation.y = 0.0
        occupancy_grid.info.origin.orientation.z = 0.0
        occupancy_grid.info.origin.orientation.w = 1.0

        # Assign grid data
        occupancy_grid.data = grid_flattened

        return occupancy_grid
    
    def pixel_to_world(self,u, v, resolution, origin, map_height):
        origin_x, origin_y,_ = origin
        world_x = origin_x + u * resolution
        world_y = origin_y + (map_height - v - 1) * resolution
        return world_x, world_y

def main():
    rclpy.init()

    node = robot_localize()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()