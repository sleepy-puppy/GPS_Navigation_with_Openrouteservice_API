import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import math
import time

class Lidar(Node):
    def __init__(self):
        super().__init__('lidar_node')

        # 퍼블리셔와 구독자 설정
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription_pointcloud = self.create_subscription(PointCloud2, '/livox/environment', self.pointcloud_callback, 10)

        # 라이다의 속도 및 회전각 변수
        self.robot_v = 0.1
        self.robot_w = 0.0
        self.points = []  # PointCloud2에서 추출된 포인트들을 저장
        self.ignore_pointcloud = False  # PointCloud2 메시지를 무시할지 여부
        self.ignore_start_time = None  # 무시를 시작한 시간
        self.obstacle_detected_count = 0  # 장애물이 1m 이내에서 연속적으로 감지된 횟수
        self.obstacle_detected = False  # 1m 이내에 장애물이 감지되었는지 여부

        # 타이머 설정 (0.5초마다 cmd_vel 메시지 발행)
        self.timer = self.create_timer(0.5, self.control_loop)

    def pointcloud_callback(self, msg):
        if self.ignore_pointcloud:
            # 2초간 메시지를 무시하는 로직
            if time.time() - self.ignore_start_time > 2:  # 2초가 지나면 다시 메시지 처리
                self.ignore_pointcloud = False
                self.obstacle_detected_count = 0  # 무시 시간 후 장애물 카운트 리셋
                self.obstacle_detected = True
                self.get_logger().info("2 seconds have passed. Resuming message processing.")
            else:
                self.get_logger().info("Ignoring PointCloud messages...")
                return

        # PointCloud2 메시지에서 x, y, z 좌표를 추출하여 리스트로 변환
        self.points = [[point[0], point[1], point[2]] for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)]

    def start_ignore_timer(self):
        # 2초 동안 메시지를 무시하도록 설정
        self.ignore_pointcloud = True
        self.ignore_start_time = time.time()

    def publish_cmd_vel(self):
        twist_msg = Twist()
        twist_msg.linear = Vector3(x=self.robot_v, y=0.0, z=0.0)
        twist_msg.angular = Vector3(x=0.0, y=0.0, z=self.robot_w)
        self.publisher_.publish(twist_msg)
        self.get_logger().info(f"Published cmd_vel - v: {self.robot_v}, w: {self.robot_w}")

    def control_loop(self):
        closest_x = closest_y = None

        if not self.points:
            self.robot_v = 0.1  # 기본 속도를 0.1로 설정
            self.robot_w = 0.0  # 각속도를 0으로 설정
            self.publish_cmd_vel()
            self.get_logger().info("No PointCloud data. Moving straight with default velocity.")
            return

        # 현재 포인트 리스트를 확인하여 가장 가까운 장애물 정보 업데이트
        if self.points:
            closest_point = min(self.points, key=lambda point: math.sqrt(point[0]**2 + point[1]**2))
            closest_distance = math.sqrt(closest_point[0]**2 + closest_point[1]**2)
            closest_x = closest_point[0]
            closest_y = closest_point[1]
        else:
            closest_distance = float('inf')

        self.get_logger().info(f"Closest obstacle distance: {closest_distance}, y: {closest_y}")

        # 장애물이 1m 이내에 들어오면
        if closest_distance < 1.0 and closest_y is not None and closest_x is not None:
            if not self.obstacle_detected and not self.ignore_pointcloud:
                # 처음 장애물 감지 시
                self.robot_v = 0.0
                self.robot_w = 0.0
                self.publish_cmd_vel()
                self.get_logger().info("Obstacle detected within 1m! Stopping for 2 seconds.")

                # 2초 동안 메시지를 무시하도록 설정
                self.start_ignore_timer()
            elif self.obstacle_detected:
                # 2초 후에도 다시 장애물이 감지되면 카운트 증가
                self.obstacle_detected_count += 1
                self.get_logger().info(f"Obstacle detected again. Count: {self.obstacle_detected_count}")

        else:
            # 1m 이상이면 장애물 감지 상태 해제
            self.obstacle_detected = False
            self.obstacle_detected_count = 0

        if self.obstacle_detected and self.obstacle_detected_count != 0:
            # 3번 연속으로 장애물이 1m 이내에 감지되면 회피 동작 수행
            if self.obstacle_detected_count >= 3:
                self.get_logger().info("Obstacle detected 3 times in a row. Initiating avoidance maneuvers.")

                obstacle_angle = math.atan2(closest_y, closest_x)  # 장애물의 각도를 계산
                self.get_logger().info(f"Calculated obstacle angle (radians): {obstacle_angle}")

                # 각도에 따라 선형적으로 robot_w 설정 및 부호 변경
                sign = 1 if obstacle_angle < 0 else -1  # 우회전(양수) 또는 좌회전(음수)을 결정
                self.robot_w = sign * 0.3 * (1 - abs(obstacle_angle) / math.pi)  # 각도가 클수록 0에 가까워짐
                self.robot_w = max(-0.3, min(0.3, self.robot_w))  # 범위를 -0.3 ~ 0.5로 제한

                self.get_logger().info(f"Setting robot_w to: {self.robot_w} based on obstacle angle.")

                self.robot_v = 0.0
                self.publish_cmd_vel()

            # 장애물이 없을 경우 정상적인 전진
            if closest_distance >= 1.0:
                self.robot_v = 1.0
                self.robot_w = 0.0
                self.publish_cmd_vel()

def main(args=None):
    rclpy.init(args=args)
    lidar_node = Lidar()
    rclpy.spin(lidar_node)
    lidar_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
