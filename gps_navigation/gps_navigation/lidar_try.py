import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import math
import time

class Lidar(Node):
    def __init__(self):
        super().__init__('lidar_node')

        # 퍼블리셔와 구독자 설정
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription_pointcloud = self.create_subscription(PointCloud2, '/pointcloud_topic', self.pointcloud_callback, 10)

        # 라이다의 속도 및 회전각 변수
        self.robot_v = 0.1
        self.robot_w = 0.0 

        self.avoid_obstacle = False

        # 포인트 클라우드 이전 데이터 저장
        self.prev_points = None
        self.prev_time = None

    def pointcloud_callback(self, msg):
            current_time = time.time()
            points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))

            if self.prev_points is not None and self.prev_time is not None:
                delta_time = current_time - self.prev_time
                theta = self.robot_w * delta_time
                dx = self.robot_v * delta_time

                rotation_matrix = np.array([
                    [math.cos(theta), -math.sin(theta), 0],
                    [math.sin(theta), math.cos(theta), 0],
                    [0, 0, 1]
                ])
                expected_points = np.dot(self.prev_points, rotation_matrix.T) + [dx, 0, 0]

                distances = np.linalg.norm(points - expected_points, axis=1)
                point_speeds = distances / delta_time

                filtered_points = points[(points[:, 0] > 0.5) & (abs(points[:, 1]) > 0.35) & (points[:, 2] > 0.3)]
                if filtered_points.size > 0:
                    closest_point_idx = np.argmin(np.linalg.norm(filtered_points, axis=1))
                    closest_point = filtered_points[closest_point_idx]
                    closest_point_speed = point_speeds[closest_point_idx]

                    if closest_point_speed <= self.robot_v and np.linalg.norm(closest_point) < 0.7:
                        self.avoid_obstacle = True
                        if abs(closest_point[1]) < 0.1:
                            self.robot_w, self.robot_v = 0.5, 0.0
                        else:
                            self.robot_w = -0.5 if closest_point[1] > 0 else 0.5
                            self.robot_v = 0.1
                        self.get_logger().info(f"Turning to avoid obstacle at {closest_point}.")
                        self.publish_cmd_vel()
                    else:
                        self.avoid_obstacle = False
                        self.get_logger().info("Obstacle is farther than 0.2m, resuming pure pursuit.")
                        self.robot_v = 0.1
                        self.robot_w = 0
                        self.publish_cmd_vel()

                else:
                    self.avoid_obstacle = False
                    self.get_logger().info("No obstacles found.")
                    self.robot_v = 0.1
                    self.robot_w = 0
                    self.publish_cmd_vel()

            self.prev_points = points
            self.prev_time = current_time

    def publish_cmd_vel(self):
        twist_msg = Twist()
        twist_msg.linear = Vector3(x=self.robot_v, y=0.0, z=0.0)
        twist_msg.angular = Vector3(x=0.0, y=0.0, z=self.robot_w)
        self.publisher_.publish(twist_msg)
        time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    pure_pursuit = Lidar()
    rclpy.spin(pure_pursuit)
    pure_pursuit.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
