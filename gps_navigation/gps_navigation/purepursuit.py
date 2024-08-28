#for gps_navigation_and_direction(3)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist, Vector3
import math
import time

class PurePursuit(Node):

    def __init__(self):
        super().__init__('pure_pursuit_node')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.subscription_current_direction = self.create_subscription(Float32MultiArray, 'azimuth', self.callback_azimuth, 10)

        self.current_azimuth = None
        self.target_azimuth = None

        self.robot_v = 0.2
        self.robot_w = 0.0 

        self.max_omega = 9.0

    def callback_azimuth(self, msg):
        if len(msg.data) >= 2:
            self.current_azimuth = msg.data[0]
            self.target_azimuth = msg.data[1]
            self.get_logger().info(f"Received azimuths: current={self.current_azimuth}, target={self.target_azimuth}")
            self.compute_steering_angle()

    def compute_steering_angle(self):
        if self.target_azimuth == 0.0:
            self.robot_v = 0.0
            self.robot_w = 0.0
            self.get_logger().info('최종 목적지에 도착했습니다.')
            return
        
        if self.current_azimuth == 0.0 :
            self.robot_w = 0.0
            self.get_logger().info(f'현재 방위각 계산 기다리는 중')
            return
        
        else:
            alpha = self.current_azimuth - self.target_azimuth
            self.get_logger().info(f"alpha : {alpha}")


            if abs(alpha) > 90:
                self.get_logger().info(f'alpha: {alpha}, 제자리 회전 필요')
                self.robot_w = 0.5 * (math.copysign(1, alpha))  # 회전 속도를 0.5 rad/s로 설정 (필요에 따라 조정 가능)
                self.robot_v = 0.0 
            else:
                self.robot_v = 0.2

                if abs(alpha) < 10:
                    self.robot_w = 0.0
                    self.get_logger().info(f'alpha: {alpha}, 직진')
                else:
                    # 선형적으로 오메가 값을 계산
                    self.robot_w = (1.5 / 90.0) * (math.copysign(1, alpha))   #copysing(a,b)는 a에 b의 부호 붙여서 반환
            self.get_logger().info(f'alpha: {alpha}, robot_v: {self.robot_v}, robot_w: {self.robot_w}')

        self.publish_cmd_vel()

    def publish_cmd_vel(self):
        purepursuit_msg = Twist()
        purepursuit_msg.linear = Vector3(x=self.robot_v, y=0.0, z=0.0)
        purepursuit_msg.angular = Vector3(x=0.0, y=0.0, z=self.robot_w)
        self.publisher_.publish(purepursuit_msg)
        time.sleep(0.5)  # 0.5초마다 발행


def main(args=None):
    rclpy.init(args=args)
    pure_pursuit = PurePursuit()
    rclpy.spin(pure_pursuit)
    pure_pursuit.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
