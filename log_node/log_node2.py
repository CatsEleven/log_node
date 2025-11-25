# GPS情報
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from autoware_adapi_v1_msgs.msg import VelocityCommand
from geometry_msgs.msg import PoseStamped
from datetime import datetime

class VelocityAndGnssSub(Node):
    def __init__(self):
        super().__init__('velocity_and_gnss_sub')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # 発行側がRELIABLEなら変更
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.sub_vel = self.create_subscription(
            VelocityCommand, '/api/control/command/velocity', self.cb_velocity, qos)
        self.sub_gnss = self.create_subscription(
            PoseStamped, '/sensing/gnss/pose', self.cb_gnss, qos)

    def cb_velocity(self, msg: VelocityCommand):
        # 注意: これは壁時計。ROS時刻を表示したいなら msg.stamp を使う。
        print(f"time={datetime.now().isoformat()}  velocity={msg.velocity:.3f} m/s")

    def cb_gnss(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        print(f"time={datetime.now().isoformat()}  gnss_xy=({x:.3f}, {y:.3f})")

def main():
    rclpy.init()
    n = VelocityAndGnssSub()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
