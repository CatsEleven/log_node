#!/usr/bin/env python3　AEB作動時に通知する。初速だけ表示する。
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from autoware_adapi_v1_msgs.msg import VelocityCommand
from datetime import datetime

# 検出したいキーワード
KEYWORDS_NAME = ("autonomous_emergency_braking", "aeb_emergency_stop", "vehicle_cmd_emergency", "AEB")
KEYWORDS_MSG  = ("AEB", "Emergency", "Brake", "Stopped")

class AEBStreamWithFirstVelocity(Node):
    def __init__(self):
        super().__init__('aeb_stream_logger')
        
        self.current_velocity = None
        self.has_triggered_once = False  # 「最初の1回」が終わったかどうかのフラグ

        # --- 車速取得 (BestEffort) ---
        qos_vel = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(VelocityCommand, '/api/control/command/velocity', self.on_velocity, qos_vel)

        # --- AEB検知 ---
        self.create_subscription(DiagnosticArray, '/diagnostics', self.on_diag, 50)

    def on_velocity(self, msg: VelocityCommand):
        self.current_velocity = msg.velocity

    def on_diag(self, msg: DiagnosticArray):
        # 受信した診断情報のリストを走査
        for st in msg.status:
            if st.level == DiagnosticStatus.ERROR:
                # キーワード判定
                name_match = any(k.lower() in (st.name or "").lower() for k in KEYWORDS_NAME)
                msg_match = any(k.lower() in (st.message or "").lower() for k in KEYWORDS_MSG)

                if name_match or msg_match:
                    # ---------------------------------------------------------
                    # 出力ロジック
                    # ---------------------------------------------------------
                    time_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    
                    if not self.has_triggered_once:
                        # === 一番最初の検知 (速度付き) ===
                        vel_str = f"{self.current_velocity:.3f}" if self.current_velocity is not None else "Unknown"
                        
                        print("\n" + "="*80)
                        print(f"[{time_str}] ★ FIRST AEB TRIGGER! (Velocity: {vel_str} m/s)")
                        print(f"  Source : {st.name}")
                        print(f"  Message: {st.message}")
                        print("="*80 + "\n")
                        
                        self.has_triggered_once = True  # フラグを立てる
                    
                    else:
                        # === 2回目以降 (速度なし・通常のログ出力) ===
                        # シンプルに流すだけ
                        print(f"[{time_str}] [AEB ACTIVE] {st.name} : {st.message}")

def main():
    rclpy.init()
    node = AEBStreamWithFirstVelocity()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()