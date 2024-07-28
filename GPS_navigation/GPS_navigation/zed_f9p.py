#for gps_navigation_and_direction(1)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from serial import Serial
from pyubx2 import UBXReader

class Zedf9pNode(Node):
    def __init__(self):
        super().__init__('zedf9p_node')
        self.publisher_gps = self.create_publisher(Float32MultiArray, 'from_zedf9p_gps', 10)

    def gps_data_fetch(self):
        with Serial("/dev/ttyACM0", 9600, timeout=3) as stream:
            ubr = UBXReader(stream)
            for raw, parsed in ubr:
                if hasattr(parsed, "lat") and hasattr(parsed, "lon"):
                    return parsed.lat, parsed.lon
        return None, None


def main(args=None):
    rclpy.init(args=args)
    node = Zedf9pNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
