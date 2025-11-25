#!/usr/bin/env python3.AEBが発火したら、ただそのことをコンソールに表示させる
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

# 検出したいキーワード定義
KEYWORDS_NAME = ("autonomous_emergency_braking", "aeb_emergency_stop", "vehicle_cmd_emergency", "AEB")
KEYWORDS_MSG  = ("AEB", "Emergency", "Brake", "Stopped")

class AEBFromDiagnostics(Node):
    def __init__(self):
        super().__init__('aeb_from_diagnostics')
        # QoS depthは適宜調整してください（標準的な診断メッセージならこれでOK）
        self.sub = self.create_subscription(DiagnosticArray, '/diagnostics', self.on_diag, 50)

    def on_diag(self, msg: DiagnosticArray):
        """
        /diagnostics を受信するたびに全ステータスを走査し、
        条件に合致すれば即座にprintする
        """
        for st in msg.status:
            # ERRORレベルかどうかチェック
            if st.level == DiagnosticStatus.ERROR:
                name_l = (st.name or "").lower()
                message_l = (st.message or "").lower()

                # キーワードが含まれているかチェック
                name_match = any(k.lower() in name_l for k in KEYWORDS_NAME)
                msg_match = any(k.lower() in message_l for k in KEYWORDS_MSG)

                if name_match or msg_match:
                    # 制限ロジックを通さず、見つけたらすぐに出力
                    print("[AEB] diagnostics ERROR detected:")
                    print(f"  - name    = {st.name}")
                    print(f"  - message = {st.message}")
                    print("-" * 40)

def main():
    rclpy.init()
    node = AEBFromDiagnostics()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()