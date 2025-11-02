# velocity_subscriber.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from autoware_adapi_v1_msgs.msg import VelocityCommand
from datetime import datetime

class VelocitySub(Node):
    def __init__(self):
        super().__init__('velocity_sub')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # 発行側がBEST_EFFORTなら合わせる
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.sub = self.create_subscription(
            VelocityCommand,
            '/api/control/command/velocity',
            self.cb,
            qos
        )

    def cb(self, msg: VelocityCommand):
        print(f"time={datetime.now().isoformat()}  velocity={msg.velocity:.3f} m/s")




def main():
    rclpy.init()
    node = VelocitySub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
