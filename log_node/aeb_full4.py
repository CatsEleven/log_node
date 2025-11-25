#!/usr/bin/env python3
import os
import cv2
import rclpy
from pathlib import Path
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles, ReliabilityPolicy, HistoryPolicy, QoSProfile
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from autoware_adapi_v1_msgs.msg import VelocityCommand
from geometry_msgs.msg import PoseStamped  # ★追加: GNSS位置情報用
from datetime import datetime

# 検出キーワード
KEYWORDS_NAME = ("autonomous_emergency_braking", "aeb_emergency_stop", "vehicle_cmd_emergency", "AEB")
KEYWORDS_MSG  = ("AEB", "Emergency", "Brake", "Stopped")

# --- 設定パラメータ ---
AEB_ACTIVE_TIMEOUT_SEC = 1.0   # AEB信号が途絶えてから何秒後まで「作動中」とみなすか
MIN_SAVE_INTERVAL_SEC  = 0.5   # 画像保存の最小間隔 (秒) [0.2s = 5fps程度]

class AEBLoggerFull(Node):
    def __init__(self):
        super().__init__('aeb_logger_full')

        # ---- 保存先設定 ----
        self.save_dir = (Path.cwd() / '../images/aeb_sequence').resolve()
        self.save_dir.mkdir(parents=True, exist_ok=True)

        # ---- 状態管理 ----
        self.last_aeb_msg_time = None    # 最後にAEBエラーを受信した時刻
        self.last_image_save_time = None # 最後に画像を保存した時刻
        
        self.current_velocity = None     # 現在の車速
        self.current_pose = None         # ★追加: 現在のGNSS位置
        self.has_printed_first_log = False 
        
        self.bridge = CvBridge()
        self.image_count = 0 

        # ---- QoS設定 (共通) ----
        # Autowareのセンサ・ステータス系はBestEffortが多いので合わせます
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ---- 1. 車速サブスクライバー ----
        self.create_subscription(
            VelocityCommand, 
            '/api/control/command/velocity', 
            self._on_velocity, 
            qos_sensor
        )

        # ---- 2. GNSSサブスクライバー (★追加) ----
        self.create_subscription(
            PoseStamped,
            '/sensing/gnss/pose',
            self._on_gnss,
            qos_sensor
        )

        # ---- 3. 画像サブスクライバー (SensorData) ----
        self.create_subscription(
            Image,
            '/sensing/camera/traffic_light/image_raw',
            self._on_image,
            QoSPresetProfiles.SENSOR_DATA.value
        )

        # ---- 4. AEB監視サブスクライバー ----
        self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self._on_diag,
            50
        )
        
        self.get_logger().info(f"Monitor started. Saving to: {self.save_dir}")

    def _on_velocity(self, msg: VelocityCommand):
        """常に最新の車速を保持"""
        self.current_velocity = msg.velocity

    def _on_gnss(self, msg: PoseStamped):
        """★追加: 常に最新のGNSS位置を保持"""
        self.current_pose = msg

    def _on_diag(self, msg: DiagnosticArray):
        """AEB診断情報の監視"""
        is_aeb = False
        target_name = ""
        target_msg = ""

        # AEBエラーがあるかチェック
        for st in msg.status:
            if st.level == DiagnosticStatus.ERROR:
                name_match = any(k.lower() in (st.name or "").lower() for k in KEYWORDS_NAME)
                msg_match = any(k.lower() in (st.message or "").lower() for k in KEYWORDS_MSG)
                if name_match or msg_match:
                    is_aeb = True
                    target_name = st.name
                    target_msg = st.message
                    break
        
        if is_aeb:
            self.last_aeb_msg_time = self.get_clock().now()

            # --- 初回のみ詳細ログ出力 (速度 + GNSS) ---
            if not self.has_printed_first_log:
                # 車速情報の整形
                vel_str = f"{self.current_velocity:.3f}" if self.current_velocity is not None else "Unknown"
                
                # GNSS情報の整形 (★追加)
                if self.current_pose is not None:
                    p = self.current_pose.pose.position
                    gnss_str = f"x={p.x:.3f}, y={p.y:.3f}"
                else:
                    gnss_str = "Unknown (No Data)"

                time_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                
                print("\n" + "="*80)
                print(f"[{time_str}] ★ FIRST AEB TRIGGER DETECTED!")
                print(f"  Velocity : {vel_str} m/s")
                print(f"  GNSS Loc : {gnss_str}")  # ★ここで位置を出力
                print(f"  Source   : {target_name}")
                print(f"  Message  : {target_msg}")
                print("="*80 + "\n")
                
                self.has_printed_first_log = True

    def _on_image(self, msg: Image):
        """画像保存ロジック（AEB有効期限内、かつ間引き条件を満たせば保存）"""
        if self.last_aeb_msg_time is None:
            return

        now = self.get_clock().now()
        
        # 1. AEB有効期限チェック
        elapsed_since_aeb = (now - self.last_aeb_msg_time).nanoseconds / 1e9
        if elapsed_since_aeb >= AEB_ACTIVE_TIMEOUT_SEC:
            return # タイムアウト

        # 2. 保存間隔（間引き）チェック
        if self.last_image_save_time is not None:
            interval = (now - self.last_image_save_time).nanoseconds / 1e9
            if interval < MIN_SAVE_INTERVAL_SEC:
                return # スキップ

        # 条件を満たしたので保存
        self._save_image(msg)
        self.last_image_save_time = now 

    def _save_image(self, msg: Image):
        try:
            img_bgr = self._to_bgr(msg)
            ts_str = datetime.now().strftime("%H%M%S_%f")[:10]
            filename = f"aeb_{self.image_count:05d}_{ts_str}.png"
            out_path = os.path.join(self.save_dir, filename)

            cv2.imwrite(out_path, img_bgr)
            
            # ログ出力（速度のみ簡易表示）
            vel_info = f"v={self.current_velocity:.1f}" if self.current_velocity is not None else "v=?"
            print(f" [SAVED] {filename} ({vel_info})")
            
            self.image_count += 1

        except Exception as e:
            self.get_logger().error(f"Save failed: {e}")

    def _to_bgr(self, msg: Image):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        enc = (msg.encoding or '').lower()
        if enc == 'rgb8':
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)
        elif enc in ('mono8', '8uc1'):
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_GRAY2BGR)
        return cv_img

def main():
    rclpy.init()
    node = AEBLoggerFull()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()