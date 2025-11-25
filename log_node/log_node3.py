#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

KEYWORDS_NAME = ("autonomous_emergency_braking", "aeb_emergency_stop", "vehicle_cmd_emergency", "AEB")
KEYWORDS_MSG  = ("AEB", "Emergency", "Brake", "Stopped")

COOLDOWN_SEC = 5.0          # 同一イベント再出力の最小間隔
UNLATCH_SEC  = 2.0          # 非アクティブがこの秒数連続したら解除

class AEBFromDiagnostics(Node):
    def __init__(self):
        super().__init__('aeb_from_diagnostics')
        self.sub = self.create_subscription(DiagnosticArray, '/diagnostics', self.on_diag, 50)
        self.latched_active = False
        self.last_inactive_ns = None     # 直近で「active=False」だった受信時刻
        self.last_print_ns_by_key = {}   # (name,message) -> 最終出力時刻

    def _ns(self, t: Time) -> int:
        return t.nanoseconds

    def on_diag(self, msg: DiagnosticArray):
        now_ns = self._ns(Time.from_msg(msg.header.stamp)) if msg.header.stamp.sec or msg.header.stamp.nanosec else self.get_clock().now().nanoseconds

        active = False
        hit_items = []
        for st in msg.status:
            if st.level == DiagnosticStatus.ERROR:
                name_l = (st.name or "").lower()
                message_l = (st.message or "").lower()
                if any(k.lower() in name_l for k in KEYWORDS_NAME) or any(k.lower() in message_l for k in KEYWORDS_MSG):
                    active = True
                    hit_items.append((st.name, st.message))

        # ラッチ制御：非アクティブが一定時間続いたら解除
        if not active:
            # 非アクティブ連続時間を蓄積
            if self.last_inactive_ns is None:
                self.last_inactive_ns = now_ns
            else:
                if self.latched_active and (now_ns - self.last_inactive_ns) >= int(UNLATCH_SEC * 1e9):
                    self.latched_active = False
        else:
            # active なら非アクティブ連続時間をリセット
            self.last_inactive_ns = None

        # 出力判定：アクティブになった瞬間 かつ クールダウン超過のみ
        if active and not self.latched_active:
            printed_any = False
            for name, message in hit_items:
                key = (name, message)
                last = self.last_print_ns_by_key.get(key, 0)
                if (now_ns - last) >= int(COOLDOWN_SEC * 1e9):
                    print("[AEB] diagnostics ERROR detected:")
                    print(f"  - name={name} | message={message}")
                    self.last_print_ns_by_key[key] = now_ns
                    printed_any = True
            if printed_any:
                self.latched_active = True

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
