#!/usr/bin/env python3
import os
import cv2
import rclpy
import time
from pathlib import Path
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles, ReliabilityPolicy, HistoryPolicy, QoSProfile
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from autoware_adapi_v1_msgs.msg import VelocityCommand
from geometry_msgs.msg import PoseStamped
from datetime import datetime

# IPC用ライブラリ
from multiprocessing.connection import Client

# 検出キーワード
KEYWORDS_NAME = ("autonomous_emergency_braking", "aeb_emergency_stop", "vehicle_cmd_emergency", "AEB")
KEYWORDS_MSG  = ("AEB", "Emergency", "Brake", "Stopped")

# --- 設定パラメータ ---
AEB_ACTIVE_TIMEOUT_SEC = 1.0
MIN_SAVE_INTERVAL_SEC  = 0.3
IPC_ADDRESS = ('localhost', 6000)
IPC_AUTHKEY = b'secret'

class AEBLoggerIPC(Node):
    def __init__(self):
        super().__init__('aeb_logger_ipc')

        # ---- 保存先設定 ----
        self.save_dir = (Path.cwd() / '../images/aeb_sequence').resolve()
        self.save_dir.mkdir(parents=True, exist_ok=True)

        # ---- 状態管理 ----
        self.last_aeb_msg_time = None
        self.last_image_save_time = None
        self.current_velocity = None
        self.current_pose = None
        self.has_printed_first_log = False
        
        self.bridge = CvBridge()
        self.image_count = 0 

        # ---- IPC接続管理 ----
        self.conn = None
        self.last_conn_try = 0
        # 起動時に一度接続を試みる（失敗しても落ちないようにする）
        self._try_connect_ipc()

        # ---- QoS設定 ----
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ---- サブスクライバー定義 ----
        self.create_subscription(VelocityCommand, '/api/control/command/velocity', self._on_velocity, qos_sensor)
        self.create_subscription(PoseStamped, '/sensing/gnss/pose', self._on_gnss, qos_sensor)
        self.create_subscription(Image, '/sensing/camera/traffic_light/image_raw', self._on_image, QoSPresetProfiles.SENSOR_DATA.value)
        self.create_subscription(DiagnosticArray, '/diagnostics', self._on_diag, 50)
        
        self.get_logger().info(f"IPC Node Started. Images -> {self.save_dir}")

    # --- IPC接続・送信関連 ---
    def _try_connect_ipc(self):
        """受信側サーバーへの接続を試行する"""
        try:
            self.conn = Client(IPC_ADDRESS, authkey=IPC_AUTHKEY)
            self.get_logger().info("IPC: Connected to server.")
        except ConnectionRefusedError:
            self.conn = None
            # 頻繁にログが出るとうるさいので、デバッグレベルか初回のみ推奨
            # self.get_logger().warn("IPC: Server not found. Will retry later.")

    def _send_ipc_data(self, data_dict):
        """データを送信する。切断されていたら再接続を試みる"""
        now = time.time()
        
        # 接続がない場合、1秒に1回程度再接続を試みる
        if self.conn is None:
            if now - self.last_conn_try > 1.0:
                self.last_conn_try = now
                self._try_connect_ipc()
            if self.conn is None:
                return # 接続不可なら今回は諦める

        try:
            self.conn.send(data_dict)
        except (BrokenPipeError, EOFError, OSError):
            self.get_logger().error("IPC: Connection lost.")
            self.conn = None

    # --- コールバック ---
    def _on_velocity(self, msg: VelocityCommand):
        self.current_velocity = msg.velocity

    def _on_gnss(self, msg: PoseStamped):
        self.current_pose = msg

    def _on_diag(self, msg: DiagnosticArray):
        is_aeb = False
        target_name = ""
        
        for st in msg.status:
            if st.level == DiagnosticStatus.ERROR:
                if any(k.lower() in (st.name or "").lower() for k in KEYWORDS_NAME) or \
                   any(k.lower() in (st.message or "").lower() for k in KEYWORDS_MSG):
                    is_aeb = True
                    target_name = st.name
                    break
        
        if is_aeb:
            self.last_aeb_msg_time = self.get_clock().now()

            if not self.has_printed_first_log:
                # ログ表示
                v_val = self.current_velocity if self.current_velocity is not None else 0.0
                p_val = self.current_pose.pose.position if self.current_pose else None
                
                print("\n" + "="*80)
                print(f"★ FIRST AEB TRIGGER! Vel={v_val:.3f}")
                print("="*80 + "\n")

                # ★ IPC送信: 初動検知イベント
                ipc_msg = {
                    "type": "AEB_TRIGGER",
                    "timestamp": datetime.now().isoformat(),
                    "velocity": v_val,
                    "gnss_x": p_val.x if p_val else None,
                    "gnss_y": p_val.y if p_val else None,
                    "source": target_name
                }
                self._send_ipc_data(ipc_msg)
                
                self.has_printed_first_log = True

    def _on_image(self, msg: Image):
        if self.last_aeb_msg_time is None:
            return

        now = self.get_clock().now()
        elapsed_since_aeb = (now - self.last_aeb_msg_time).nanoseconds / 1e9

        if elapsed_since_aeb >= AEB_ACTIVE_TIMEOUT_SEC:
            return 

        if self.last_image_save_time is not None:
            interval = (now - self.last_image_save_time).nanoseconds / 1e9
            if interval < MIN_SAVE_INTERVAL_SEC:
                return 

        self._save_image(msg)
        self.last_image_save_time = now 

    def _save_image(self, msg: Image):
        try:
            img_bgr = self._to_bgr(msg)
            ts_str = datetime.now().strftime("%H%M%S_%f")[:10]
            filename = f"aeb_{self.image_count:05d}_{ts_str}.png"
            out_path = os.path.join(self.save_dir, filename)

            cv2.imwrite(out_path, img_bgr)
            
            # コンソール出力
            vel_info = f"{self.current_velocity:.2f}" if self.current_velocity is not None else "N/A"
            print(f" [SAVED] {filename} (v={vel_info})")
            
            # ★ IPC送信: 画像保存イベント
            # 画像そのものではなく、パスとメタデータを送る
            ipc_msg = {
                "type": "IMAGE_SAVED",
                "timestamp": datetime.now().isoformat(),
                "path": out_path,      # 受信側はこれを開けば画像が見れる
                "velocity": self.current_velocity,
                "gnss": (self.current_pose.pose.position.x, self.current_pose.pose.position.y) if self.current_pose else None
            }
            self._send_ipc_data(ipc_msg)

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
    node = AEBLoggerIPC()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()