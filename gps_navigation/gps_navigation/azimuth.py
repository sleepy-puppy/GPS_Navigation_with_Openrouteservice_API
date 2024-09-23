#for gps_navigation_and_direction(2)
#방위각을 이용한 로봇의 현재 방향과 목표 방향을 계산하는 파일임.

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import os

class DirectionPublisher(Node):
    def __init__(self):
        super().__init__('direction_node')
        
        # Publisher with Float32MultiArray
        self.publisher = self.create_publisher(Float64MultiArray, 'azimuth', 10)
        self.subscription = self.create_subscription(Float64MultiArray, 'from_zedf9p_gps', self.callback, 10)
        
        self.coordinates = [None, None]  # List to store coordinates
        self.waypoints = self.load_waypoints()
        self.current_waypoint_index = 0
        self.current_lat = None
        self.current_lon = None
        self.target_lat = None
        self.target_lon = None

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
        return waypoints

    def callback(self, msg):
        # Update the coordinates
        print(msg.data[0], msg.data[1])
        self.coordinates[0] = self.coordinates[1]
        self.coordinates[1] = (msg.data[0], msg.data[1])

        lat1, lon1 = self.coordinates[0] if self.coordinates[0] else (None, None)
        self.current_lat, self.current_lon = self.coordinates[1]

        if lat1 is not None and lon1 is not None and self.current_lat is not None and self.current_lon is not None:
            current_bearing = self.calculate_azimuth(lat1, lon1, self.current_lat, self.current_lon)
            self.update_target(self.current_lat, self.current_lon)
            
            # Create and publish Float32MultiArray message
            msg = Float64MultiArray()
            msg.data = [current_bearing, self.calculate_target_bearing()]
            self.publisher.publish(msg)
            print(f"current azimuth : {current_bearing}, target azimuth : {self.calculate_target_bearing()}")

    def calculate_azimuth(self, lat1, lon1, lat2, lon2):
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        dLon = lon2 - lon1
        x = math.sin(dLon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(dLon))
        initial_azimuth = math.atan2(x, y)
        compass_azimuth = math.degrees(initial_azimuth)
        if compass_azimuth < 0:
            compass_azimuth += 360

        return compass_azimuth

    def calculate_target_bearing(self):
        if self.target_lat is not None and self.target_lon is not None:
            return self.calculate_azimuth(self.current_lat, self.current_lon, self.target_lat, self.target_lon)
        return 0.0

    def update_target(self, current_lat, current_lon):
        if self.current_waypoint_index < len(self.waypoints):
            self.target_lat, self.target_lon = self.waypoints[self.current_waypoint_index]
            if self.reached_waypoint():
                self.current_waypoint_index += 1
                if self.current_waypoint_index >= len(self.waypoints):
                    self.target_lat, self.target_lon = None
                    self.get_logger().info("목적지에 도착했습니다.")

    def reached_waypoint(self):
        distance = math.sqrt((self.target_lat - self.current_lat)**2 + (self.target_lon - self.current_lon)**2)
        return distance < 0.0001  # Adjust the threshold as needed

def main(args=None):
    rclpy.init(args=args)
    node = DirectionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
