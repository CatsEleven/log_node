
import os
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rclpy.time import Time
from sensor_msgs.msg import Image


class TrafficLightImageSaver(Node):
    def __init__(self):
        super().__init__('traffic_light_image_saver')

        # ---- 保存先設定（現在ディレクトリ基準の ../images）----
        save_dir = (Path.cwd() / '../images').resolve()
        save_dir.mkdir(parents=True, exist_ok=True)
        self.save_dir = str(save_dir)
        self.file_index = 0

        # ---- QoS: カメラ系に合わせて SensorData (BestEffort / Volatile / KeepLast) ----
        qos = QoSPresetProfiles.SENSOR_DATA.value

        # ---- 購読設定 ----
        self.bridge = CvBridge()
        self.latest_msg: Image | None = None
        self.topic = '/sensing/camera/traffic_light/image_raw'
        self.sub = self.create_subscription(Image, self.topic, self._on_image, qos)

        # ---- タイマー（5秒ごとに最新フレーム1枚を保存）----
        self.timer = self.create_timer(5.0, self._save_latest)

        # ---- 起動ログ＆Publisher QoSの可視化 ----
        self.get_logger().info(f"Saving every 5s to: {self.save_dir}")
        self._log_publishers_qos()

    def _log_publishers_qos(self):
        infos = self.get_publishers_info_by_topic(self.topic)
        if not infos:
            self.get_logger().warn(f'No publishers detected on {self.topic}')
            return
        for i, info in enumerate(infos):
            q = info.qos_profile
            self.get_logger().info(
                f"pub[{i}] reliability={q.reliability.name} "
                f"durability={q.durability.name} history={q.history.name} depth={q.depth}"
            )

    def _on_image(self, msg: Image):
        # 最新のみ保持（高フレームレートでも負荷を抑制）
        self.latest_msg = msg

    def _save_latest(self):
        if self.latest_msg is None:
            self.get_logger().warn("No image received yet; skipping this cycle.")
            return

        # タイムスタンプは診断用に残しておく（保存ファイル名は連番）
        stamp_ns = 0
        try:
            t = Time.from_msg(self.latest_msg.header.stamp)
            stamp_ns = t.nanoseconds
        except Exception:
            pass

        try:
            img_bgr = self._to_bgr(self.latest_msg)
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        out_path = os.path.join(self.save_dir, f"traffic_light_{self.file_index:04d}.png")
        ok = cv2.imwrite(out_path, img_bgr)
        if not ok:
            self.get_logger().error(f"Failed to write: {out_path}")
            return

        self.get_logger().info(
            f"Saved: {out_path} (seq={self.file_index} ts={stamp_ns})"
        )
        self.file_index += 1

    def _to_bgr(self, msg: Image):
        """
        可能な限り生エンコードで受け取り、必要な場合のみBGRへ変換。
        bgr8/rgb8/mono8 等に対応。
        """
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        enc = (msg.encoding or '').lower()

        if enc == 'rgb8':
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)
        elif enc in ('mono8', '8uc1'):
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_GRAY2BGR)
        # bgr8 / その他はそのまま
        return cv_img


def main():
    rclpy.init()
    node = TrafficLightImageSaver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
