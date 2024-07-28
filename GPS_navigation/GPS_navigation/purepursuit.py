#for gps_navigation_and_direction(3)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist, Vector3
import math
import os

class PurePursuit(Node):

    def __init__(self):
        super().__init__('pure_pursuit')
        self.subscription_gps = self.create_subscription(Float32MultiArray, 'from_zedf9p_gps', self.callback_gps, 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.current_lat = 0.0
        self.current_lon = 0.0
        self.target_lat = 0.0
        self.target_lon = 0.0
        self.L = 10000  # Example vehicle wheelbase
        self.axle_width = 100
        self.wheel_width = 80
        self.robot_v = 0.2
        self.waypoints = self.load_waypoints()
        self.current_waypoint_index = 0

    def load_waypoints(self):
        waypoints = []
        waypoints_file = os.path.join(os.path.dirname(__file__), 'waypoints', 'waypoints.txt')
        try:
            with open(waypoints_file, 'r') as f:
                for line in f:
                    lat, lon = map(float, line.strip().split(','))
                    waypoints.append((lat, lon))
        except FileNotFoundError:
            self.get_logger().error('Waypoints file not found')

        print(waypoints)
        return waypoints

    def callback_gps(self, msg):
        self.current_lat = msg.data[0]
        self.current_lon = msg.data[1]
        self.update_target()
        self.compute_steering_angle()

    def update_target(self):
        if self.current_waypoint_index < len(self.waypoints):
            self.target_lat, self.target_lon = self.waypoints[self.current_waypoint_index]
            if self.reached_waypoint():
                self.current_waypoint_index += 1

    def reached_waypoint(self):
        distance = math.sqrt((self.target_lat - self.current_lat)**2 + (self.target_lon - self.current_lon)**2)
        return distance < 0.0001  # Adjust the threshold as needed

    def compute_steering_angle(self):
        if self.current_waypoint_index >= len(self.waypoints):
            # 최종 목적지에 도착한 경우
            self.robot_v = 0.0
            robot_w = 0.0
        else:
            delta_lon = self.target_lon - self.current_lon
            delta_lat = self.target_lat - self.current_lat
            Ld = math.sqrt(delta_lon**2 + delta_lat**2)

            if Ld == 0:
                return  # Avoid division by zero

            alpha = math.atan2(delta_lat, delta_lon)
            r = Ld / (2 * math.sin(alpha))
            robot_w = self.robot_v / r

        self.get_logger().info(f'robot_v: {self.robot_v}, robot_w: {robot_w}')

        purepursuit_msg = Twist()
        purepursuit_msg.linear = Vector3(x=self.robot_v, y=0.0, z=0.0)
        purepursuit_msg.angular = Vector3(x=0.0, y=0.0, z=robot_w)
        self.publisher_.publish(purepursuit_msg)



def main(args=None):
    rclpy.init(args=args)
    pure_pursuit = PurePursuit()
    rclpy.spin(pure_pursuit)
    pure_pursuit.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
